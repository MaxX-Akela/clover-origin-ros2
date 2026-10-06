#!/usr/bin/env python3
# Smoke run for the camera_markers node: mock camera_info, check the latched markers.
# Usage (workspace sourced): python3 smoke_camera_markers.py

import os
import signal
import subprocess
import sys
import time

os.environ.setdefault('ROS_DOMAIN_ID', '87')

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo
from visualization_msgs.msg import MarkerArray


def spin_until(node, condition, timeout):
    deadline = time.time() + timeout
    while time.time() < deadline and not condition():
        rclpy.spin_once(node, timeout_sec=0.05)
    return condition()


def main():
    proc = subprocess.Popen(['ros2', 'run', 'clover', 'camera_markers', '--ros-args', '-p', 'scale:=2.0'],
                            start_new_session=True)
    rclpy.init()
    node = rclpy.create_node('smoke_camera_markers')
    try:
        # camera driver may publish camera_info as best effort
        info_pub = node.create_publisher(CameraInfo, 'camera_info',
                                         QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        info = CameraInfo()
        info.header.frame_id = 'main_camera_optical'
        node.create_timer(0.1, lambda: info_pub.publish(info))

        # no markers topic before camera_info is received by the node
        assert spin_until(node, lambda: node.count_publishers('camera_markers') == 1, 10.0), 'no markers publisher'
        time.sleep(1.0)  # markers have been published by now

        # late subscriber gets the markers (latched)
        received = []
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(MarkerArray, 'camera_markers', received.append, latched)
        assert spin_until(node, lambda: received, 5.0), 'no markers received by a late subscriber'

        markers = received[0].markers
        print('markers: %d, frame: %s, lens scale: %g' % (len(markers), markers[0].header.frame_id, markers[0].scale.x))
        assert len(markers) == 3
        assert all(m.header.frame_id == 'main_camera_optical' for m in markers)
        assert abs(markers[0].scale.x - 0.013 * 2.0) < 1e-9

        # markers are published once, the node unsubscribes from camera_info
        assert spin_until(node, lambda: info_pub.get_subscription_count() == 0, 5.0), 'still subscribed'
        assert len(received) == 1

        print('OK')
    finally:
        os.killpg(proc.pid, signal.SIGINT)
        proc.wait(timeout=10)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
