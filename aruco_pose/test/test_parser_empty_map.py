import os
import sys

import launch
import launch_testing.actions
import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from aruco_test_utils import *  # noqa: E402,F401,F403

from sensor_msgs.msg import Image  # noqa: E402,F401
from geometry_msgs.msg import PoseWithCovarianceStamped  # noqa: E402,F401
from aruco_pose.msg import MarkerArray  # noqa: E402,F401
from visualization_msgs.msg import MarkerArray as VisMarkerArray, Marker as VisMarker  # noqa: E402,F401


@pytest.mark.launch_test
def generate_test_description():
    return launch.LaunchDescription([
        container('aruco_container', [
            map_node(type='map', map=os.path.join(TEST_DIR, 'test_parser_empty_map.txt')),
        ]),
        launch_testing.actions.ReadyToTest(),
    ])


class TestArucoPose(ArucoTestCase):
    def test_empty_map(self):
        markers = self.wait_for(VisMarkerArray, 'aruco_map/visualization', LATCHED_QOS)
        assert len(markers.markers) == 0
