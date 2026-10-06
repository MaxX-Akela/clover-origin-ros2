#!/usr/bin/env python3
# Smoke run for the vpe_publisher node with mock TF, pose and local position.
# Usage (workspace sourced): python3 smoke_vpe_publisher.py

import os
import signal
import subprocess
import sys
import threading
import time

os.environ.setdefault('ROS_DOMAIN_ID', '87')

import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster

BEST_EFFORT = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)


def wait(condition, timeout):
    deadline = time.time() + timeout
    while time.time() < deadline and not condition():
        time.sleep(0.02)
    return condition()


def start(*params):
    args = ['ros2', 'run', 'clover', 'vpe_publisher', '--ros-args']
    for param in params:
        args += ['-p', param]
    return subprocess.Popen(args, start_new_session=True)


def stop(proc):
    os.killpg(proc.pid, signal.SIGINT)
    proc.wait(timeout=10)


def test_offset(node):
    # pose from the topic, offset to the local frame
    received, static = [], []
    vpe_sub = node.create_subscription(PoseStamped, 'vpe_publisher/vpe', received.append, 10)
    static_qos = QoSProfile(depth=10, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL)
    static_sub = node.create_subscription(TFMessage, 'tf_static', static.append, static_qos)
    pose_pub = node.create_publisher(PoseStamped, 'vpe_publisher/pose', 1)
    br = TransformBroadcaster(node)

    def publish():
        now = node.get_clock().now()
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = 1.0
        t.transform.translation.y = 2.0
        t.transform.translation.z = 3.0
        t.transform.rotation.w = 1.0
        br.sendTransform(t)

        p = PoseStamped()
        p.header.stamp = (now - Duration(seconds=0.1)).to_msg()  # transform is available at this time
        p.header.frame_id = 'aruco_map'
        p.pose.position.x = 0.5
        p.pose.orientation.w = 1.0
        pose_pub.publish(p)

    timer = node.create_timer(0.05, publish)
    proc = start('local_frame:=map', 'fcu_frame:=base_link', 'offset_frame_id:=vpe_offset')
    try:
        assert wait(lambda: received, 15.0), 'no vpe messages'
        vpe = received[-1]
        print('vpe: frame %s, position %.2f %.2f %.2f' %
              (vpe.header.frame_id, vpe.pose.position.x, vpe.pose.position.y, vpe.pose.position.z))
        assert vpe.header.frame_id == 'map'
        # the offset makes visual pose coincide with the local position
        assert abs(vpe.pose.position.x - 1) < 1e-6 and abs(vpe.pose.position.y - 2) < 1e-6 and \
            abs(vpe.pose.position.z - 3) < 1e-6

        assert wait(lambda: static, 5.0), 'no offset transform'
        offset = static[-1].transforms[0]
        print('offset: %s -> %s, %.2f %.2f %.2f' % (offset.header.frame_id, offset.child_frame_id,
              offset.transform.translation.x, offset.transform.translation.y, offset.transform.translation.z))
        assert offset.header.frame_id == 'map' and offset.child_frame_id == 'vpe_offset'
        assert abs(offset.transform.translation.x - 0.5) < 1e-6

        # reset makes the node publish the offset again
        count = len(static)
        reset = node.create_client(Trigger, 'vpe_publisher/reset')
        assert reset.wait_for_service(timeout_sec=5.0)
        future = reset.call_async(Trigger.Request())
        assert wait(future.done, 5.0) and future.result().success
        assert wait(lambda: len(static) > count, 5.0), 'offset is not reset'
        print('reset: offset published again')
    finally:
        stop(proc)
        timer.cancel()
        node.destroy_subscription(vpe_sub)
        node.destroy_subscription(static_sub)


def test_force_init(node, executor):
    # frames are read from the parameters of (mock) mavros local_position node
    mavros = rclpy.create_node('local_position', namespace='mavros')
    mavros.declare_parameter('tf.frame_id', 'mock_map')
    mavros.declare_parameter('tf.child_frame_id', 'mock_base_link')
    executor.add_node(mavros)

    received = []
    vpe_sub = node.create_subscription(PoseStamped, 'vpe_publisher/vpe', received.append, 10)
    proc = start('force_init:=true', 'force_init_duration:=1.0')
    try:
        assert wait(lambda: len(received) >= 10, 15.0), 'no zero poses'
        zero = received[-1]
        print('zero: frame %s, %d messages' % (zero.header.frame_id, len(received)))
        assert zero.header.frame_id == 'mock_map', 'frame is not read from mavros'
        assert zero.pose.position.x == 0 and zero.pose.orientation.w == 1

        # local position appears: zero poses stop in force_init_duration
        local_pub = node.create_publisher(PoseStamped, 'mavros/local_position/pose', BEST_EFFORT)

        def publish():
            p = PoseStamped()
            p.header.stamp = node.get_clock().now().to_msg()
            local_pub.publish(p)

        timer = node.create_timer(0.1, publish)
        time.sleep(2.0)
        count = len(received)
        time.sleep(1.0)
        print('zero poses in 1 s with local position: %d' % (len(received) - count))
        assert len(received) == count, 'zero poses are still published'
        timer.cancel()
    finally:
        stop(proc)
        node.destroy_subscription(vpe_sub)


def main():
    rclpy.init()
    node = rclpy.create_node('smoke_vpe_publisher')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    try:
        test_offset(node)
        test_force_init(node, executor)
        print('OK')
    finally:
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
