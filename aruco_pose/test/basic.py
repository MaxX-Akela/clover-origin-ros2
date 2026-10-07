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
        image_publisher('main_camera', 'map.png'),
        container('aruco_container', [
            detect_node(length=0.33, estimate_poses=True, send_tf=True, cornerRefinementMethod=1,
                        **{'length_override.3': 0.1}),
            map_node(type='map', map=os.path.join(TEST_DIR, 'basic.txt'),
                     **{'markers.frame_id': 'aruco_map', 'markers.child_frame_id_prefix': 'aruco_in_map_'}),
        ]),
        launch_testing.actions.ReadyToTest(),
    ])


class TestArucoPose(ArucoTestCase):
    # all the outputs are subscribed before the first test, the first message of each of them is kept
    PRELOAD = (
        (MarkerArray, 'aruco_detect/markers'),
        (VisMarkerArray, 'aruco_detect/visualization'),
        (Image, 'aruco_detect/debug'),
        (PoseWithCovarianceStamped, 'aruco_map/pose'),
        (Image, 'aruco_map/debug'),
        (Image, 'aruco_map/image', LATCHED_QOS),
        (MarkerArray, 'aruco_map/map', LATCHED_QOS),
        (VisMarkerArray, 'aruco_map/visualization', LATCHED_QOS),
    )
    READY = (MarkerArray, 'aruco_detect/markers')

    def test_markers(self):
        markers = self.wait_for(MarkerArray, 'aruco_detect/markers')
        assert len(markers.markers) == 5
        assert markers.header.frame_id == 'main_camera_optical'

        assert markers.markers[0].id == 2
        assert markers.markers[0].length == approx(0.33)
        assert markers.markers[0].pose.position.x == approx(0.36706567854)
        assert markers.markers[0].pose.position.y == approx(0.290484516644)
        assert markers.markers[0].pose.position.z == approx(2.18787602301)
        assert markers.markers[0].pose.orientation.x == approx(0.993997406299)
        assert markers.markers[0].pose.orientation.y == approx(-0.00532003481626)
        assert markers.markers[0].pose.orientation.z == approx(-0.107390951553)
        assert markers.markers[0].pose.orientation.w == approx(0.0201999263402)
        assert markers.markers[0].c1.x == approx(415.557739258)
        assert markers.markers[0].c1.y == approx(335.557739258)
        assert markers.markers[0].c2.x == approx(509.442260742)
        assert markers.markers[0].c2.y == approx(335.557739258)
        assert markers.markers[0].c3.x == approx(509.442260742)
        assert markers.markers[0].c3.y == approx(429.442260742)
        assert markers.markers[0].c4.x == approx(415.557739258)
        assert markers.markers[0].c4.y == approx(429.442260742)

        assert markers.markers[4].id == 3
        assert markers.markers[4].length == approx(0.1)
        assert markers.markers[4].pose.position.x == approx(-0.1805169666)
        assert markers.markers[4].pose.position.y == approx(-0.200697302327)
        assert markers.markers[4].pose.position.z == approx(0.585767514823)
        assert markers.markers[4].pose.orientation.x == approx(-0.961738074009)
        assert markers.markers[4].pose.orientation.y == approx(-0.0375180244707)
        assert markers.markers[4].pose.orientation.z == approx(-0.0115387773672)
        assert markers.markers[4].pose.orientation.w == approx(0.271144115664)
        assert markers.markers[4].c1.x == approx(129.557723999)
        assert markers.markers[4].c1.y == approx(49.557723999)
        assert markers.markers[4].c2.x == approx(223.442276001)
        assert markers.markers[4].c2.y == approx(49.557723999)
        assert markers.markers[4].c3.x == approx(223.442276001)
        assert markers.markers[4].c3.y == approx(143.442276001)
        assert markers.markers[4].c4.x == approx(129.557723999)
        assert markers.markers[4].c4.y == approx(143.442276001)

        assert markers.markers[1].id == 1
        assert markers.markers[1].length == approx(0.33)
        assert markers.markers[3].id == 4
        assert markers.markers[3].length == approx(0.33)

        assert markers.markers[2].id == 100
        assert markers.markers[2].length == approx(0.33)
        assert markers.markers[2].pose.position.x == approx(-1.37600105389)
        assert markers.markers[2].pose.position.y == approx(-0.323028530991)
        assert markers.markers[2].pose.position.z == approx(2.94611272668)
        assert markers.markers[2].pose.orientation.x == approx(-0.955543925678)
        assert markers.markers[2].pose.orientation.y == approx(0.0458801909197)
        assert markers.markers[2].pose.orientation.z == approx(-0.249604946264)
        assert markers.markers[2].pose.orientation.w == approx(-0.150093920537)
        assert markers.markers[2].c1.x == approx(52.557723999)
        assert markers.markers[2].c1.y == approx(205.557723999)
        assert markers.markers[2].c2.x == approx(113.442276001)
        assert markers.markers[2].c2.y == approx(205.557723999)
        assert markers.markers[2].c3.x == approx(113.442276001)
        assert markers.markers[2].c3.y == approx(265.442260742)
        assert markers.markers[2].c4.x == approx(52.557723999)
        assert markers.markers[2].c4.y == approx(265.442260742)

    def test_markers_frames(self):
        marker_2 = self.lookup('main_camera_optical', 'aruco_2')
        assert marker_2.transform.translation.x == approx(0.36706567854)
        assert marker_2.transform.translation.y == approx(0.290484516644)
        assert marker_2.transform.translation.z == approx(2.18787602301)
        assert marker_2.transform.rotation.x == approx(0.993997406299)
        assert marker_2.transform.rotation.y == approx(-0.00532003481626)
        assert marker_2.transform.rotation.z == approx(-0.107390951553)
        assert marker_2.transform.rotation.w == approx(0.0201999263402)

    def test_map_markers_frames(self):
        marker_1 = self.lookup('aruco_map', 'aruco_in_map_1')
        assert marker_1.transform.translation.x == approx(0)
        assert marker_1.transform.translation.y == approx(0)
        assert marker_1.transform.translation.z == approx(0)

        marker_4 = self.lookup('aruco_map', 'aruco_in_map_4')
        assert marker_4.transform.translation.x == approx(1)
        assert marker_4.transform.translation.y == approx(1)
        assert marker_4.transform.translation.z == approx(0)

        marker_12 = self.lookup('aruco_map', 'aruco_in_map_12')
        assert marker_12.transform.translation.x == approx(0.2)
        assert marker_12.transform.translation.y == approx(0.5)
        assert marker_12.transform.translation.z == approx(0)

    def test_visualization(self):
        vis = self.wait_for(VisMarkerArray, 'aruco_detect/visualization')
        assert len(vis.markers) == 11

    def test_debug(self):
        img = self.wait_for(Image, 'aruco_detect/debug')
        assert img.width == 640
        assert img.height == 480
        assert img.header.frame_id == 'main_camera_optical'

    def test_map(self):
        pose = self.wait_for(PoseWithCovarianceStamped, 'aruco_map/pose')
        assert pose.header.frame_id == 'main_camera_optical'
        assert pose.pose.pose.position.x == approx(-0.629167753342)
        assert pose.pose.pose.position.y == approx(0.293822650809)
        assert pose.pose.pose.position.z == approx(2.12641343155)
        assert pose.pose.pose.orientation.x == approx(-0.998383794799)
        assert pose.pose.pose.orientation.y == approx(-5.20919098575e-06)
        assert pose.pose.pose.orientation.z == approx(-0.0300861070302)
        assert pose.pose.pose.orientation.w == approx(0.0482143590507)

    def test_map_image(self):
        img = self.wait_for(Image, 'aruco_map/image', LATCHED_QOS)
        assert img.width == 2000
        assert img.height == 2000
        assert img.encoding in ('mono8', 'rgb8')

    def test_map_markers(self):
        markers = self.wait_for(MarkerArray, 'aruco_map/map', LATCHED_QOS)
        assert markers.markers[0].id == 1
        assert markers.markers[1].id == 2
        assert markers.markers[2].id == 3
        assert markers.markers[3].id == 4
        assert markers.markers[4].id == 10
        assert markers.markers[5].id == 11
        assert markers.markers[6].id == 12

        assert markers.markers[0].pose.position.x == 0
        assert markers.markers[0].pose.position.y == 0
        assert markers.markers[0].pose.position.z == 0
        assert markers.markers[0].pose.orientation.x == 0
        assert markers.markers[0].pose.orientation.y == 0
        assert markers.markers[0].pose.orientation.z == 0
        assert markers.markers[0].pose.orientation.w == 1
        assert markers.markers[0].length == approx(0.33)

        assert markers.markers[1].pose.position.x == 1
        assert markers.markers[1].pose.position.y == 0
        assert markers.markers[1].pose.position.z == 0
        assert markers.markers[1].pose.orientation.x == 0
        assert markers.markers[1].pose.orientation.y == 0
        assert markers.markers[1].pose.orientation.z == 0
        assert markers.markers[1].pose.orientation.w == 1
        assert markers.markers[1].length == approx(0.33)

        assert markers.markers[2].pose.position.x == 0
        assert markers.markers[2].pose.position.y == 1
        assert markers.markers[2].pose.position.z == 0
        assert markers.markers[2].pose.orientation.x == 0
        assert markers.markers[2].pose.orientation.y == 0
        assert markers.markers[2].pose.orientation.z == 0
        assert markers.markers[2].pose.orientation.w == 1
        assert markers.markers[2].length == approx(0.33)

        assert markers.markers[3].pose.position.x == 1
        assert markers.markers[3].pose.position.y == 1
        assert markers.markers[3].pose.position.z == 0
        assert markers.markers[3].pose.orientation.x == 0
        assert markers.markers[3].pose.orientation.y == 0
        assert markers.markers[3].pose.orientation.z == 0
        assert markers.markers[3].pose.orientation.w == 1
        assert markers.markers[3].length == approx(0.33)

        assert markers.markers[4].pose.position.x == approx(0.5)
        assert markers.markers[4].pose.position.y == 2
        assert markers.markers[4].pose.position.z == 0
        assert markers.markers[4].pose.orientation.x == 0
        assert markers.markers[4].pose.orientation.y == 0
        assert markers.markers[4].pose.orientation.z == approx(0.5646424733950354)
        assert markers.markers[4].pose.orientation.w == approx(0.8253356149096783)
        assert markers.markers[4].length == approx(0.5)

    def test_map_visualization(self):
        vis = self.wait_for(VisMarkerArray, 'aruco_map/visualization', LATCHED_QOS)
        assert len(vis.markers) == 7
        assert vis.markers[0].header.frame_id == 'aruco_map'
        assert vis.markers[0].type == VisMarker.CUBE
        assert vis.markers[0].action == VisMarker.ADD
        assert vis.markers[0].pose.position.x == 0
        assert vis.markers[0].pose.position.y == 0
        assert vis.markers[0].pose.position.z == 0
        assert vis.markers[0].pose.orientation.x == 0
        assert vis.markers[0].pose.orientation.y == 0
        assert vis.markers[0].pose.orientation.z == 0
        assert vis.markers[0].pose.orientation.w == 1
        assert vis.markers[0].scale.x == approx(0.33)
        assert vis.markers[0].scale.y == approx(0.33)
        assert vis.markers[0].scale.z == approx(0.001)
        assert vis.markers[1].pose.position.x == 1
        assert vis.markers[1].pose.position.y == 0
        assert vis.markers[1].pose.position.z == 0
        assert vis.markers[1].pose.orientation.x == 0
        assert vis.markers[1].pose.orientation.y == 0
        assert vis.markers[1].pose.orientation.z == 0
        assert vis.markers[1].pose.orientation.w == 1
        # non-zero yaw marker:
        assert vis.markers[4].scale.x == approx(0.5)
        assert vis.markers[4].pose.position.x == approx(0.5)
        assert vis.markers[4].pose.position.y == 2
        assert vis.markers[4].pose.position.z == 0
        assert vis.markers[4].pose.orientation.x == 0
        assert vis.markers[4].pose.orientation.y == 0
        assert vis.markers[4].pose.orientation.z == approx(0.5646424733950354)
        assert vis.markers[4].pose.orientation.w == approx(0.8253356149096783)

    def test_map_debug(self):
        img = self.wait_for(Image, 'aruco_map/debug')
