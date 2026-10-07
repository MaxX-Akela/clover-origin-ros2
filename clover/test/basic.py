#!/usr/bin/env python3
import os
import sys

import launch
import launch_testing.actions
import pytest
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from mavros_msgs.msg import State
from clover import srv
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from clover_test_utils import CloverTestCase, GEOID_HELP, LATCHED, clover_node, has_package, \
    setup_geoid, static_transform, wait  # noqa: E402

HAS_GEOID = setup_geoid()
HAS_WEB_VIDEO_SERVER = has_package('web_video_server')


@pytest.mark.launch_test
def generate_test_description():
    # Verify all the required nodes basically work
    actions = []

    if HAS_GEOID:  # mavros_node dies on start without the geoid
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('clover'), 'launch',
                                                       'mavros.launch.py')),
            launch_arguments={
                'fcu_conn': 'udp',
                'fcu_ip': '127.0.0.1',
                'gcs_bridge': 'false',
                'respawn': 'false',
                'viz': 'false',  # mavros_extras for ROS 2 has no visualization node
            }.items(),
        ))

    if HAS_WEB_VIDEO_SERVER:
        actions.append(Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server',
            parameters=[{'default_stream_type': 'ros_compressed', 'publish_rate': 1.0}],
            output='screen',
        ))

    actions += [
        static_transform('map_flipped_frame', 0, 0, 0, 'map', 'map_flipped', yaw=3.1415926, pitch=3.1415926),
        clover_node('simple_offboard'),
        clover_node('rc'),
        clover_node('shell'),
        clover_node('led', name='led_effect', namespace='led',
                    **{'notify.startup.r': 255, 'notify.startup.g': 255, 'notify.startup.b': 255}),
        launch_testing.actions.ReadyToTest(),
    ]
    return launch.LaunchDescription(actions)


class TestBasic(CloverTestCase):
    def test_state(self):
        if not HAS_GEOID:
            self.skipTest(GEOID_HELP)
        state = self.wait_for_message(State, 'mavros/state', LATCHED, timeout=30)
        assert state.connected == False
        assert state.armed == False
        assert state.guided == False
        assert state.mode == ''

    def test_nodes_running(self):
        # the nodes were marked as required in the original test
        self.wait_for_nodes(['/simple_offboard', '/rc', '/shell', '/led/led_effect', '/map_flipped_frame'])

    def test_simple_offboard_services_available(self):
        # the node reads the frames from mavros on start, which takes several seconds without mavros
        self.wait_for_service('get_telemetry', timeout=30)
        self.wait_for_service('navigate', timeout=5)
        self.wait_for_service('navigate_global', timeout=5)
        self.wait_for_service('set_position', timeout=5)
        self.wait_for_service('set_velocity', timeout=5)
        self.wait_for_service('set_attitude', timeout=5)
        self.wait_for_service('set_rates', timeout=5)
        self.wait_for_service('land', timeout=5)
        self.wait_for_service('simple_offboard/release', timeout=5)

    def test_web_video_server(self):
        if not HAS_WEB_VIDEO_SERVER:
            self.skipTest('web_video_server package is not installed')
        import urllib.request as urllib

        def read():
            try:
                return urllib.urlopen("http://localhost:8080").read()
            except OSError:
                return None

        assert wait(read, 10, 0.5) is not None

    def test_blocks(self):
        self.skipTest('clover_blocks package is not ported to ROS 2')

    def test_long_callback(self):
        from clover import long_callback
        from time import sleep

        # very basic test for long_callback
        @long_callback
        def cb(i):
            cb.counter += i
        cb.counter = 0
        cb(2)
        sleep(0.1)
        cb(3)
        sleep(1)
        assert cb.counter == 5
