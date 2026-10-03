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
            map_node(type='map', map=os.path.join(TEST_DIR, 'test_parser_pass.txt')),
        ]),
        launch_testing.actions.ReadyToTest(),
    ])


class TestArucoPose(ArucoTestCase):
    def test_markers(self):
        markers = self.wait_for(VisMarkerArray, 'aruco_map/visualization', LATCHED_QOS)
        assert len(markers.markers) == 6

        assert markers.markers[0].pose.position.x == approx(0)
        assert markers.markers[0].pose.position.y == approx(0)
        assert markers.markers[0].pose.position.z == approx(0)

        assert markers.markers[1].pose.position.x == approx(1)
        assert markers.markers[1].pose.position.y == approx(1)
        assert markers.markers[1].pose.position.z == approx(1)

        assert markers.markers[2].pose.position.x == approx(1)
        assert markers.markers[2].pose.position.y == approx(0)
        assert markers.markers[2].pose.position.z == approx(0.5)

        assert markers.markers[3].pose.position.x == approx(0)
        assert markers.markers[3].pose.position.y == approx(1)
        assert markers.markers[3].pose.position.z == approx(0)

        assert markers.markers[4].pose.position.x == approx(1)
        assert markers.markers[4].pose.position.y == approx(0.5)
        assert markers.markers[4].pose.position.z == approx(0)

        assert markers.markers[5].pose.position.x == approx(2.2)
        assert markers.markers[5].pose.position.y == approx(0.2)
        assert markers.markers[5].pose.position.z == approx(0)

        assert markers.markers[0].scale.x == approx(0.33)
        assert markers.markers[0].scale.y == approx(0.33)
        assert markers.markers[1].scale.x == approx(0.225)
        assert markers.markers[1].scale.y == approx(0.225)
        assert markers.markers[2].scale.x == approx(0.45)
        assert markers.markers[2].scale.y == approx(0.45)
        assert markers.markers[3].scale.x == approx(0.15)
        assert markers.markers[3].scale.y == approx(0.15)
        assert markers.markers[4].scale.x == approx(0.25)
        assert markers.markers[4].scale.y == approx(0.25)
        assert markers.markers[5].scale.x == approx(0.35)
        assert markers.markers[5].scale.y == approx(0.35)

    def test_map_image(self):
        img = self.wait_for(Image, 'aruco_map/image', LATCHED_QOS)
        assert img.width == 2000
        assert img.height == 2000
        assert img.encoding in ('mono8', 'rgb8')
