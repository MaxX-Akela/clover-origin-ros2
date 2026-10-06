#!/usr/bin/env python3
# Smoke run for the rc node: UDP packets, fake GCS heartbeat, latched state.
# Usage (workspace sourced): python3 smoke_rc.py

import os
import signal
import socket
import struct
import subprocess
import sys
import threading
import time

os.environ.setdefault('ROS_DOMAIN_ID', '87')

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from mavros_msgs.msg import ManualControl, Mavlink, State

PORT = 35612
BEST_EFFORT = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
# mavros publishes the state as reliable and transient local
MAVROS_STATE = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def wait(condition, timeout):
    deadline = time.time() + timeout
    while time.time() < deadline and not condition():
        time.sleep(0.02)
    return condition()


def main():
    proc = subprocess.Popen(['ros2', 'run', 'clover', 'rc', '--ros-args', '-p', 'port:=%d' % PORT],
                            start_new_session=True)
    rclpy.init()
    node = rclpy.create_node('smoke_rc')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    try:
        controls, heartbeats, states = [], [], []
        node.create_subscription(ManualControl, 'mavros/manual_control/send', controls.append, 10)  # reliable, as mavros
        node.create_subscription(Mavlink, '/uas1/mavlink_sink', heartbeats.append, BEST_EFFORT)  # as mavros router
        node.create_subscription(State, 'state_latched', states.append, LATCHED)

        # initial unknown state is latched
        assert wait(lambda: states, 10.0), 'no latched state'
        assert not states[-1].connected and states[-1].mode == ''
        assert wait(lambda: node.count_subscribers('mavros/manual_control/send') >= 1, 5.0)
        time.sleep(1.5)
        assert not heartbeats, 'heartbeat is sent without manual control'

        # UDP packets
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        assert wait(lambda: sock.sendto(struct.pack('<hhhh', 100, -200, 300, -400), ('127.0.0.1', PORT)) and controls,
                    5.0), 'no manual control messages'
        c = controls[-1]
        print('manual control: x=%g y=%g z=%g r=%g' % (c.x, c.y, c.z, c.r))
        assert (c.x, c.y, c.z, c.r) == (100, -200, 300, -400)

        # fake GCS heartbeat follows manual control
        assert wait(lambda: heartbeats, 3.0), 'no fake GCS heartbeat'
        hb = heartbeats[-1]
        print('heartbeat: sysid=%d msgid=%d len=%d payload64=%s' % (hb.sysid, hb.msgid, hb.len, list(hb.payload64)))
        assert hb.sysid == 255 and hb.msgid == 0 and hb.len == 9 and hb.magic == Mavlink.MAVLINK_V20

        # state is republished on change only
        state_pub = node.create_publisher(State, 'mavros/state', MAVROS_STATE)
        state = State(connected=True, mode='OFFBOARD')
        timer = node.create_timer(0.5, lambda: state_pub.publish(state))
        assert wait(lambda: states[-1].connected, 5.0), 'state is not passed'
        count = len(states)
        time.sleep(2.0)
        assert len(states) == count, 'unchanged state is republished'
        state.armed = True
        assert wait(lambda: states[-1].armed, 5.0), 'state change is not passed'
        print('latched state: connected=%s mode=%s armed=%s' % (states[-1].connected, states[-1].mode, states[-1].armed))

        # state timeout
        timer.cancel()
        start = time.time()
        assert wait(lambda: not states[-1].connected, 5.0), 'no state timeout'
        print('state timeout in %.1f s' % (time.time() - start))
        assert 2.0 < time.time() - start < 4.0
        count = len(states)
        time.sleep(3.5)
        assert len(states) == count, 'state timeout is not one-shot'

        # the node stops cleanly (socket thread is joined)
        start = time.time()
        os.killpg(proc.pid, signal.SIGINT)
        code = proc.wait(timeout=10)
        print('stopped in %.1f s, exit code %d' % (time.time() - start, code))
        print('OK')
    finally:
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGKILL)
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
