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
        image_publisher('imgpub_01', 'crash_image_01.png'),
        image_publisher('imgpub_02', 'crash_image_02.png'),
        image_publisher('imgpub_03', 'crash_image_03.png'),
        container('aruco_container_01', [
            detect_node('aruco_detect_01', camera='imgpub_01', length=0.33, cornerRefinementMethod=2)]),
        container('aruco_container_02', [
            detect_node('aruco_detect_02', camera='imgpub_02', length=0.33, cornerRefinementMethod=2)]),
        container('aruco_container_03', [
            detect_node('aruco_detect_03', camera='imgpub_03', length=0.33, cornerRefinementMethod=2)]),
        launch_testing.actions.ReadyToTest(),
    ])


class TestArucoPose(ArucoTestCase):
    def test_opencv_crashes_img01(self):
        self.wait_for(VisMarkerArray, 'aruco_detect_01/visualization')

    def test_opencv_crashes_img02(self):
        self.wait_for(VisMarkerArray, 'aruco_detect_02/visualization')

    def test_opencv_crashes_img03(self):
        self.wait_for(VisMarkerArray, 'aruco_detect_03/visualization')
