# Test of the rosbridge side of the web pages: a websocket client sends the same JSON as roslib.js (the vendored
# ROS 1 version) and the TF client in viz.js. The pages themselves are not tested here (a browser is needed).
# Skipped without rosbridge_server.

import asyncio
import json
import os
import sys
import time
import unittest

import launch
import launch_testing.actions
import pytest
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from mavros_msgs.msg import State, StatusText
from sensor_msgs.msg import BatteryState
from visualization_msgs.msg import Marker, MarkerArray

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from clover_test_utils import CloverTestCase, LATCHED, has_package, static_transform  # noqa: E402

PORT = 9191
HAS_ROSBRIDGE = has_package('rosbridge_server')
HAS_TF_REPUBLISHER = has_package('tf2_web_republisher')

try:
    from tornado.websocket import websocket_connect
except ImportError:
    websocket_connect = None


@pytest.mark.launch_test
def generate_test_description():
    actions = []
    if HAS_ROSBRIDGE:
        actions.append(IncludeLaunchDescription(
            AnyLaunchDescriptionSource(os.path.join(
                get_package_share_directory('rosbridge_server'), 'launch', 'rosbridge_websocket_launch.xml')),
            launch_arguments={'port': str(PORT)}.items()))
    if HAS_TF_REPUBLISHER:
        actions.append(Node(package='tf2_web_republisher', executable='tf2_web_republisher_node',
                            name='tf2_web_republisher', output='screen'))
    actions += [
        static_transform('map_test', 10, 20, 30, 'map', 'test'),
        static_transform('test_test2', 1, 2, 3, 'test', 'test2'),
    ]
    if not HAS_ROSBRIDGE:  # launch_testing stops if there are no processes
        actions.append(ExecuteProcess(cmd=[sys.executable, '-c', 'import time; time.sleep(3600)']))
    actions.append(launch_testing.actions.ReadyToTest())
    return launch.LaunchDescription(actions)


class Client:
    """Websocket client speaking the rosbridge protocol."""

    def __init__(self, loop):
        self.loop = loop
        self.ws = None
        self.counter = 0

    async def connect(self, timeout=30.0):
        deadline = time.time() + timeout
        while True:
            try:
                self.ws = await websocket_connect('ws://127.0.0.1:%d' % PORT)
                return
            except Exception:
                if time.time() > deadline:
                    raise
                await asyncio.sleep(0.3)

    def send(self, message):
        self.ws.write_message(json.dumps(message))

    async def receive(self, predicate, timeout=10.0):
        """Wait for a message matching the predicate, the others are dropped."""
        deadline = time.time() + timeout
        while True:
            left = deadline - time.time()
            if left <= 0:
                raise AssertionError('no expected message in %s s' % timeout)
            try:
                text = await asyncio.wait_for(self.ws.read_message(), left)
            except asyncio.TimeoutError:
                raise AssertionError('no expected message in %s s' % timeout)
            assert text is not None, 'connection closed'
            message = json.loads(text)
            if predicate(message):
                return message

    # what Topic.subscribe does in roslib.js
    def subscribe(self, topic, type=None, **kwargs):
        self.counter += 1
        message = {'op': 'subscribe', 'id': 'subscribe:%s:%d' % (topic, self.counter), 'topic': topic,
                   'compression': 'none', 'throttle_rate': 0, 'queue_length': 0}
        if type is not None:
            message['type'] = type
        message.update(kwargs)
        self.send(message)

    async def message(self, topic, timeout=10.0):
        return (await self.receive(lambda m: m.get('op') == 'publish' and m.get('topic') == topic, timeout))['msg']

    # what Service.callService does in roslib.js
    async def call_service(self, service, type, args, timeout=10.0):
        self.counter += 1
        call_id = 'call_service:%s:%d' % (service, self.counter)
        self.send({'op': 'call_service', 'id': call_id, 'service': service, 'type': type, 'args': args})
        response = await self.receive(lambda m: m.get('op') == 'service_response' and m.get('id') == call_id, timeout)
        assert response['result'], response
        return response['values']


@unittest.skipUnless(HAS_ROSBRIDGE, 'rosbridge_server package is not installed')
@unittest.skipUnless(websocket_connect, 'tornado is not installed')
class TestWeb(CloverTestCase):
    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        node = cls.node
        # the topics which the pages use; mavros state and the markers are latched, the others are periodic
        state = State()
        state.connected = True
        state.mode = 'OFFBOARD'
        cls.state_pub = node.create_publisher(State, '/mavros/state', LATCHED)
        cls.state_pub.publish(state)

        markers = MarkerArray()
        marker = Marker()
        marker.header.frame_id = 'test'
        marker.ns = 'mock'
        marker.id = 1
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        markers.markers.append(marker)
        cls.marker_pubs = {}
        for topic in '/main_camera/camera_markers', '/aruco_map/visualization':  # latched in the nodes
            cls.marker_pubs[topic] = node.create_publisher(MarkerArray, topic, LATCHED)
            cls.marker_pubs[topic].publish(markers)

        battery_pub = node.create_publisher(BatteryState, '/mavros/battery', 10)
        text_pub = node.create_publisher(StatusText, '/mavros/statustext/recv', 10)

        def periodic():
            battery = BatteryState()
            battery.cell_voltage = [3.9, 3.9, 3.9]
            battery_pub.publish(battery)
            text = StatusText()
            text.severity = 4
            text.text = 'mock status'
            text_pub.publish(text)

        node.create_timer(0.2, periodic)

    def run_client(self, coroutine_function):
        async def main():
            client = Client(asyncio.get_running_loop())
            await client.connect()
            try:
                await coroutine_function(client)
            finally:
                client.ws.close()

        asyncio.run(main())

    def test_topics(self):
        # roslib.js sends the type, the ROS 2 form of it; gcs.js subscribes to these three
        async def check(client):
            client.subscribe('/mavros/state', 'mavros_msgs/msg/State')
            client.subscribe('/mavros/statustext/recv', 'mavros_msgs/msg/StatusText')
            client.subscribe('/mavros/battery', 'sensor_msgs/msg/BatteryState', throttle_rate=5000)
            # the state is published once before the connection: a late subscriber gets it (transient local)
            state = await client.message('/mavros/state')
            assert state['mode'] == 'OFFBOARD' and state['connected'] is True
            text = await client.message('/mavros/statustext/recv')
            assert text['text'] == 'mock status' and text['severity'] == 4
            battery = await client.message('/mavros/battery')
            assert abs(battery['cell_voltage'][0] - 3.9) < 1e-5

        self.run_client(check)

    def test_unknown_type_and_header(self):
        # topics.js subscribes without the type; the time fields of a header are sec/nanosec in ROS 2
        async def check(client):
            client.subscribe('/mavros/state')
            state = await client.message('/mavros/state')
            assert state['mode'] == 'OFFBOARD'
            assert set(state['header']['stamp']) == {'sec', 'nanosec'}

        self.run_client(check)

    def test_rosapi(self):
        async def check(client):
            topics = await client.call_service('/rosapi/topics', 'rosapi/Topics', {})
            assert len(topics['topics']) == len(topics['types'])
            by_name = dict(zip(topics['topics'], topics['types']))
            assert by_name['/mavros/state'] == 'mavros_msgs/msg/State'
            assert by_name['/mavros/battery'] == 'sensor_msgs/msg/BatteryState'
            topic_type = await client.call_service('/rosapi/topic_type', 'rosapi/TopicType',
                                                   {'topic': '/mavros/state'})
            assert topic_type['type'] == 'mavros_msgs/msg/State'
            # the type string is split to pkg/msg/Type in topics.js
            assert len(topic_type['type'].split('/')) == 3

        self.run_client(check)

    def test_latched_markers(self):
        async def check(client):
            for topic in self.marker_pubs:
                client.subscribe(topic, 'visualization_msgs/msg/MarkerArray')
            for topic in self.marker_pubs:
                markers = await client.message(topic)
                assert markers['markers'][0]['header']['frame_id'] == 'test'

        self.run_client(check)

    @unittest.skipUnless(HAS_TF_REPUBLISHER, 'tf2_web_republisher package is not installed')
    def test_tf(self):
        # the same messages as TFActionClient in viz.js: the action of tf2_web_republisher over rosbridge
        action = '/tf2_web_republisher'
        action_type = 'tf2_web_republisher_interfaces/action/TFSubscription'

        def goal(goal_id, frames):
            return {'op': 'send_action_goal', 'id': goal_id, 'action': action, 'action_type': action_type,
                    'feedback': True,
                    'args': {'source_frames': frames, 'target_frame': 'map', 'angular_thres': 0.01,
                             'trans_thres': 0.01, 'rate': 10.0}}

        def transforms(goal_id, frame):
            def predicate(m):
                return m.get('op') == 'action_feedback' and m.get('id') == goal_id and \
                    frame in [t['child_frame_id'] for t in m['values']['transforms']]
            return predicate

        def translation(message, frame):
            transform = [t for t in message['values']['transforms'] if t['child_frame_id'] == frame][0]
            assert transform['header']['frame_id'] == 'map'
            return [transform['transform']['translation'][a] for a in 'xyz']

        async def check(client):
            client.send(goal('tf_goal:1', ['test']))
            message = await client.receive(transforms('tf_goal:1', 'test'), 20.0)
            assert translation(message, 'test') == [10.0, 20.0, 30.0]

            # a frame is added: the previous goal is cancelled and a new one is sent
            client.send({'op': 'cancel_action_goal', 'id': 'tf_goal:1', 'action': action})
            client.send(goal('tf_goal:2', ['test', 'test2']))
            message = await client.receive(transforms('tf_goal:2', 'test2'), 20.0)
            assert translation(message, 'test2') == [11.0, 22.0, 33.0]

            client.send({'op': 'cancel_action_goal', 'id': 'tf_goal:2', 'action': action})

        self.run_client(check)
