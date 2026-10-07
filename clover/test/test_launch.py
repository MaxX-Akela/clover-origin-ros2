# Test of clover.launch.py without the FCU and the camera: waits for the key nodes, services and topics
# and checks some parameters. The lists are shared with smoke_launch.py, which also runs the launch file
# with mavros_node and with the camera.

import os
import sys

import launch
import launch_testing.actions
import pytest
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from rclpy.parameter import parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from clover_test_utils import CloverTestCase, wait  # noqa: E402
from smoke_launch import NODES, SERVICES, TOPICS, missing  # noqa: E402


@pytest.mark.launch_test
def generate_test_description():
    return launch.LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('clover'), 'launch',
                                                       'clover.launch.py')),
            launch_arguments={'fcu_conn': 'none', 'main_camera': 'false', 'aruco': 'false'}.items(),
        ),
        launch_testing.actions.ReadyToTest(),
    ])


class TestLaunch(CloverTestCase):
    NODE_NAME = 'launch_test'

    def get_param(self, name, param):
        client = AsyncParameterClient(self.node, name)
        assert client.wait_for_services(timeout_sec=5.0), 'no parameter services of %s' % name
        future = client.get_parameters([param])
        assert wait(future.done, 5.0), 'no response from %s' % name
        return parameter_value_to_python(future.result().values[0])

    def test_graph(self):
        ok = wait(lambda: not missing(self.node, NODES, SERVICES, TOPICS), 60.0, 0.2)
        assert ok, 'not found: %s' % ', '.join(missing(self.node, NODES, SERVICES, TOPICS))

    def test_parameters(self):
        assert wait(lambda: not missing(self.node, NODES, [], []), 60.0, 0.2)
        assert self.get_param('simple_offboard', 'terrain_frame_mode') == 'range'
        assert self.get_param('optical_flow', 'roi_rad') == 0.8
        assert self.get_param('vpe_publisher', 'offset_frame_id') == 'aruco_map'
        assert self.get_param('led_effect', 'notify.low_battery.threshold') == 3.6
