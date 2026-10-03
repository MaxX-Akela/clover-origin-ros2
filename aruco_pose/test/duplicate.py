import os
import subprocess
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
        image_publisher('main_camera', 'duplicate.png'),
        container('aruco_container', [
            detect_node(length=0.33, estimate_poses=True, send_tf=True, cornerRefinementMethod=1),
        ]),
        launch_testing.actions.ReadyToTest(),
    ])


class TestArucoPose(ArucoTestCase):
    def test_no_tf_repeated_data(self):
        # run a separate tf listener and check that it doesn't complain about repeated data
        script = (
            'import time, rclpy, tf2_ros\n'
            'rclpy.init()\n'
            'node = rclpy.create_node("foo")\n'
            'buf = tf2_ros.Buffer()\n'
            'listener = tf2_ros.TransformListener(buf, node)\n'
            'end = time.monotonic() + 2\n'
            'while time.monotonic() < end:\n'
            '    rclpy.spin_once(node, timeout_sec=0.1)\n'
        )
        output = subprocess.run([sys.executable, '-c', script], stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, universal_newlines=True).stdout
        assert 'TF_REPEATED_DATA' not in output, 'TF_REPEATED_DATA was logged on duplicate markers'
