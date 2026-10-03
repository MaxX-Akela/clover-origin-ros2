"""Helpers shared by the aruco_pose launch tests."""

import os
import time
import unittest

import pytest
import rclpy
from rclpy.duration import Duration
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from rclpy.wait_for_message import wait_for_message
import tf2_ros

from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

TEST_DIR = os.path.dirname(os.path.abspath(__file__))
TIMEOUT = 40  # components are loaded asynchronously and best-effort images may be dropped, so be generous

# QoS of the topics published as "latched" (map, visualization of the map, map image)
LATCHED_QOS = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


def approx(expected):
    return pytest.approx(expected, abs=1e-4)  # compare floats more roughly


def image_publisher(name, image, namespace=None):
    """Publish an image file and camera info as <namespace>/image_raw, <namespace>/camera_info."""
    return Node(
        package='image_publisher',
        executable='image_publisher_node',
        name=name,
        namespace=namespace if namespace is not None else name,
        arguments=[os.path.join(TEST_DIR, image)],
        parameters=[{
            'frame_id': 'main_camera_optical',
            'publish_rate': 10.0,
            'camera_info_url': 'file://' + os.path.join(TEST_DIR, 'camera_info.yaml'),
        }],
    )


def detect_node(name='aruco_detect', camera='main_camera', **parameters):
    return ComposableNode(
        package='aruco_pose',
        plugin='aruco_pose::ArucoDetect',
        name=name,
        parameters=[parameters],
        remappings=[
            ('image_raw', camera + '/image_raw'),
            ('camera_info', camera + '/camera_info'),
        ],
    )


def map_node(name='aruco_map', camera='main_camera', markers='aruco_detect/markers', **parameters):
    return ComposableNode(
        package='aruco_pose',
        plugin='aruco_pose::ArucoMap',
        name=name,
        parameters=[parameters],
        remappings=[
            ('image_raw', camera + '/image_raw'),
            ('camera_info', camera + '/camera_info'),
            ('markers', markers),
        ],
    )


def container(name, nodes):
    return ComposableNodeContainer(
        name=name,
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=nodes,
        output='screen',
    )


class ArucoTestCase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('aruco_pose_test')
        cls.tf_buffer = tf2_ros.Buffer()
        cls.tf_listener = tf2_ros.TransformListener(cls.tf_buffer, cls.node, spin_thread=False)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def wait_for(self, msg_type, topic, qos=1, timeout=TIMEOUT):
        ok, msg = wait_for_message(msg_type, self.node, topic, qos_profile=qos, time_to_wait=timeout)
        self.assertTrue(ok, 'no message on {}'.format(topic))
        return msg

    def lookup(self, target, source, timeout=TIMEOUT):
        end = time.monotonic() + timeout
        while True:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            try:
                return self.tf_buffer.lookup_transform(target, source, Time(), Duration(seconds=0))
            except tf2_ros.TransformException:
                if time.monotonic() > end:
                    raise
