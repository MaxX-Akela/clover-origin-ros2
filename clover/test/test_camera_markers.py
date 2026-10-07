# Test of the camera_markers node: mock camera_info, check the latched markers.

import os
import sys
import time

import launch
import launch_testing.actions
import pytest
from sensor_msgs.msg import CameraInfo
from visualization_msgs.msg import MarkerArray

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from clover_test_utils import BEST_EFFORT, CloverTestCase, LATCHED, clover_node, wait  # noqa: E402


@pytest.mark.launch_test
def generate_test_description():
    return launch.LaunchDescription([
        clover_node('camera_markers', scale=2.0),
        launch_testing.actions.ReadyToTest(),
    ])


class TestCameraMarkers(CloverTestCase):
    def test_markers(self):
        node = self.node
        time.sleep(1.0)
        # no markers topic before camera_info is received by the node
        assert node.count_publishers('camera_markers') == 0, 'markers are published without camera_info'

        # camera driver may publish camera_info as best effort
        info_pub = node.create_publisher(CameraInfo, 'camera_info', BEST_EFFORT)
        info = CameraInfo()
        info.header.frame_id = 'main_camera_optical'
        node.create_timer(0.1, lambda: info_pub.publish(info))

        assert wait(lambda: node.count_publishers('camera_markers') == 1, 10.0), 'no markers publisher'
        time.sleep(1.0)  # markers have been published by now

        # late subscriber gets the markers (latched)
        received = []
        node.create_subscription(MarkerArray, 'camera_markers', received.append, LATCHED)
        assert wait(lambda: received, 5.0), 'no markers received by a late subscriber'

        markers = received[0].markers
        assert len(markers) == 3
        assert all(m.header.frame_id == 'main_camera_optical' for m in markers)
        assert abs(markers[0].scale.x - 0.013 * 2.0) < 1e-9

        # markers are published once, the node unsubscribes from camera_info
        assert wait(lambda: info_pub.get_subscription_count() == 0, 5.0), 'still subscribed'
        assert len(received) == 1
