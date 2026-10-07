"""Helpers shared by the clover launch tests."""

import os
import sys
import threading
import time
import unittest

# keep the tests away from the nodes running on the machine; the launched processes inherit the variable
os.environ.setdefault('ROS_DOMAIN_ID', '87')

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from ament_index_python.packages import PackageNotFoundError, get_package_prefix
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

TEST_DIR = os.path.dirname(os.path.abspath(__file__))

BEST_EFFORT = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
# QoS of the topics published as "latched"
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

GEOID = 'geoids/egm96-5.pgm'
GEOID_SYSTEM_DIR = '/usr/share/GeographicLib'
GEOID_USER_DIR = os.path.expanduser('~/.local/share/GeographicLib')
GEOID_HELP = ('mavros_node needs the GeographicLib geoid dataset, install it with: '
              'sudo /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh '
              '(or put egm96-5 to ~/.local/share/GeographicLib/geoids)')


def has_package(name):
    try:
        get_package_prefix(name)
        return True
    except PackageNotFoundError:
        return False


def setup_geoid():
    """Make the geoid dataset available for mavros_node, return False if there is no one.

    GEOGRAPHICLIB_DATA is set to ~/.local/share/GeographicLib if it is not set and the directory exists.
    """
    if not os.environ.get('GEOGRAPHICLIB_DATA') and os.path.isdir(GEOID_USER_DIR):
        os.environ['GEOGRAPHICLIB_DATA'] = GEOID_USER_DIR
    data = os.environ.get('GEOGRAPHICLIB_DATA') or GEOID_SYSTEM_DIR
    return os.path.exists(os.path.join(data, GEOID))


def clover_node(executable, name=None, namespace=None, **parameters):
    return Node(
        package='clover',
        executable=executable,
        name=name or executable,
        namespace=namespace,
        parameters=[parameters] if parameters else None,
        output='screen',
    )


def static_transform(name, x, y, z, frame_id, child_frame_id, yaw=0, pitch=0, roll=0):
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=name,
        arguments=['--x', str(x), '--y', str(y), '--z', str(z), '--yaw', str(yaw), '--pitch', str(pitch),
                   '--roll', str(roll), '--frame-id', frame_id, '--child-frame-id', child_frame_id],
    )


def mock_local_position(frame_id, child_frame_id, namespace='/mavros'):
    """Run the mock of mavros local_position plugin node, the nodes read the frames from its parameters."""
    return ExecuteProcess(
        cmd=[sys.executable, os.path.join(TEST_DIR, 'mock_local_position.py'), '--ros-args',
             '-r', '__ns:=' + namespace, '-p', 'tf.frame_id:=' + frame_id, '-p', 'tf.child_frame_id:=' + child_frame_id],
        output='screen',
    )


def wait(condition, timeout, period=0.02):
    deadline = time.time() + timeout
    while time.time() < deadline and not condition():
        time.sleep(period)
    return condition()


def make_request(srv_type, **kwargs):
    """Create a service request; integers are accepted for the float fields, as in rospy."""
    request = srv_type.Request()
    types = request.get_fields_and_field_types()
    for name, value in kwargs.items():
        if types[name] in ('float', 'double') and isinstance(value, int):
            value = float(value)
        setattr(request, name, value)
    return request


class CloverTestCase(unittest.TestCase):
    """Test case with a node, which is spun in a background thread."""

    NODE_NAME = 'clover_test'

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node(cls.NODE_NAME)
        cls.executor = MultiThreadedExecutor()
        cls.executor.add_node(cls.node)
        cls.lock = threading.Lock()  # creating and destroying of the entities
        threading.Thread(target=cls.executor.spin, daemon=True).start()

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        rclpy.shutdown()

    def wait_for_message(self, msg_type, topic, qos=BEST_EFFORT, timeout=10.0):
        """Wait for a message with a new subscription (as rospy.wait_for_message)."""
        received = []
        event = threading.Event()

        def callback(msg):
            received.append(msg)
            event.set()

        with self.lock:
            sub = self.node.create_subscription(msg_type, topic, callback, qos)
        try:
            assert event.wait(timeout), 'no message on {}'.format(topic)
        finally:
            with self.lock:
                self.node.destroy_subscription(sub)
        return received[0]

    def wait_for_service(self, name, timeout=5.0):
        """Wait for a service to appear in the graph (as rospy.wait_for_service)."""
        name = name if name.startswith('/') else '/' + name
        assert wait(lambda: name in dict(self.node.get_service_names_and_types()), timeout, 0.1), \
            'service {} is not available'.format(name)

    def wait_for_nodes(self, names, timeout=30.0):
        def missing():
            found = {(ns if ns.endswith('/') else ns + '/') + name
                     for name, ns in self.node.get_node_names_and_namespaces()}
            return sorted(set(names) - found)

        assert wait(lambda: not missing(), timeout, 0.1), 'nodes are not running: {}'.format(', '.join(missing()))

    def call(self, client, request, timeout=30.0):
        future = client.call_async(request)
        assert wait(future.done, timeout), 'no response from {}'.format(client.srv_name)
        return future.result()

    def service_proxy(self, name, srv_type, timeout=30.0):
        """Return a function calling the service with keyword arguments (as rospy.ServiceProxy)."""
        with self.lock:
            client = self.node.create_client(srv_type, name)

        def proxy(**kwargs):
            # a request sent before the client has discovered the server would be lost
            assert client.wait_for_service(timeout_sec=timeout), 'service {} is not available'.format(name)
            return self.call(client, make_request(srv_type, **kwargs), timeout)

        proxy.client = client
        return proxy
