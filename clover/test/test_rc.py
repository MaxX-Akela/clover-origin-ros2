# Test of the rc node: UDP packets, fake GCS heartbeat, latched state.

import os
import socket
import struct
import sys
import time
import unittest

import launch
import launch_testing.actions
import launch_testing.asserts
import pytest
from rclpy.qos import DurabilityPolicy, QoSProfile
from mavros_msgs.msg import ManualControl, Mavlink, State

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from clover_test_utils import BEST_EFFORT, CloverTestCase, LATCHED, clover_node, wait  # noqa: E402

PORT = 35612
# mavros publishes the state as reliable and transient local
MAVROS_STATE = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)


@pytest.mark.launch_test
def generate_test_description():
    rc = clover_node('rc', port=PORT)
    return launch.LaunchDescription([
        rc,
        launch_testing.actions.ReadyToTest(),
    ]), {'rc': rc}


class TestRC(CloverTestCase):
    def test_rc(self):
        node = self.node
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
        assert (c.x, c.y, c.z, c.r) == (100, -200, 300, -400)

        # fake GCS heartbeat follows manual control
        assert wait(lambda: heartbeats, 3.0), 'no fake GCS heartbeat'
        hb = heartbeats[-1]
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

        # state timeout
        timer.cancel()
        start = time.time()
        assert wait(lambda: not states[-1].connected, 5.0), 'no state timeout'
        assert 2.0 < time.time() - start < 4.0
        count = len(states)
        time.sleep(3.5)
        assert len(states) == count, 'state timeout is not one-shot'


@launch_testing.post_shutdown_test()
class TestRCShutdown(unittest.TestCase):
    def test_exit_code(self, proc_info, rc):
        # the node stops cleanly (socket thread is joined)
        launch_testing.asserts.assertExitCodes(proc_info, process=rc)
