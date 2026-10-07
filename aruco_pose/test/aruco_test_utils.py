"""Helpers shared by the aruco_pose launch tests."""

import os
import time
import unittest

import pytest
import rclpy
from rclpy.duration import Duration
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
import tf2_ros

from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

TEST_DIR = os.path.dirname(os.path.abspath(__file__))
TIMEOUT = 40  # components are loaded asynchronously and best-effort images may be dropped, so be generous
READY_TIMEOUT = 120  # the first message of the whole pipeline: the components are loaded and the first frame is processed

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


class Waiter:
    """The first message of a topic (the topics of the tests publish the same data every time)."""

    def __init__(self, node, msg_type, topic, qos):
        self.node = node
        self.topic = topic
        self.message = None
        self.count = 0
        self.subscription = node.create_subscription(msg_type, topic, self.callback, qos)

    def callback(self, msg):
        self.count += 1
        if self.message is None:
            self.message = msg

    def release(self):
        """Unsubscribe when there is a message, large images are not received for nothing."""
        if self.subscription is not None and self.message is not None:
            self.node.destroy_subscription(self.subscription)
            self.subscription = None


class ArucoTestCase(unittest.TestCase):
    # The topics (type, name, QoS) subscribed once before the tests and the topic which shows that
    # the pipeline works. A subscription of a test is created when it is already running, and the
    # messages published before it are lost.
    PRELOAD = ()
    READY = None

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('aruco_pose_test')
        cls.tf_buffer = tf2_ros.Buffer()
        cls.tf_listener = tf2_ros.TransformListener(cls.tf_buffer, cls.node, spin_thread=False)
        cls.waiters = {}
        for msg_type, topic, *qos in cls.PRELOAD:
            cls.waiters[topic] = Waiter(cls.node, msg_type, topic, qos[0] if qos else 1)
        if cls.READY is not None:
            msg_type, topic = cls.READY
            cls.waiters.setdefault(topic, Waiter(cls.node, msg_type, topic, 1))
            # not an error here: the tests report which of the messages they need are missing
            cls.spin_until(lambda: cls.waiters[topic].message is not None, READY_TIMEOUT)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def spin_until(cls, condition, timeout):
        """Spin the node until the condition, which is checked after every event, or the timeout."""
        end = time.monotonic() + timeout
        while not condition() and time.monotonic() < end:
            rclpy.spin_once(cls.node, timeout_sec=0.1)
            for waiter in cls.waiters.values():
                waiter.release()
        return condition()

    def received(self):
        return ', '.join('{}: {}'.format(topic, waiter.count) for topic, waiter in sorted(self.waiters.items()))

    def wait_for(self, msg_type, topic, qos=1, timeout=TIMEOUT):
        waiter = self.waiters.get(topic)
        if waiter is None or waiter.message is None and waiter.subscription is None:
            waiter = self.waiters[topic] = Waiter(self.node, msg_type, topic, qos)
        self.spin_until(lambda: waiter.message is not None, timeout)
        self.assertIsNotNone(waiter.message, 'no message on {} (messages received: {})'.format(topic, self.received()))
        return waiter.message

    def lookup(self, target, source, timeout=TIMEOUT):
        end = time.monotonic() + timeout
        while True:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            try:
                return self.tf_buffer.lookup_transform(target, source, Time(), Duration(seconds=0))
            except tf2_ros.TransformException:
                if time.monotonic() > end:
                    raise
