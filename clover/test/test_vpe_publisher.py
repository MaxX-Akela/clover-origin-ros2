# Test of the vpe_publisher node with mock TF, pose and local position.

import os
import sys
import time

import launch
import launch_testing.actions
import pytest
from launch.actions import TimerAction
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from clover_test_utils import BEST_EFFORT, CloverTestCase, clover_node, mock_local_position, wait  # noqa: E402


@pytest.mark.launch_test
def generate_test_description():
    return launch.LaunchDescription([
        # the frames are set with the parameters
        clover_node('vpe_publisher', local_frame='map', fcu_frame='base_link', offset_frame_id='vpe_offset'),
        # the frames are read from the parameters of (mock) mavros local_position node
        mock_local_position('mock_map', 'mock_base_link', namespace='/force_init/mavros'),
        TimerAction(period=3.0, actions=[
            clover_node('vpe_publisher', namespace='force_init', force_init=True, force_init_duration=1.0),
        ]),
        launch_testing.actions.ReadyToTest(),
    ])


class TestVpePublisher(CloverTestCase):
    def test_offset(self):
        # pose from the topic, offset to the local frame
        node = self.node
        received, static = [], []
        node.create_subscription(PoseStamped, 'vpe_publisher/vpe', received.append, 10)
        static_qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(TFMessage, 'tf_static', static.append, static_qos)
        pose_pub = node.create_publisher(PoseStamped, 'vpe_publisher/pose', 1)
        br = TransformBroadcaster(node)

        def publish():
            now = node.get_clock().now()
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = 'map'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = 1.0
            t.transform.translation.y = 2.0
            t.transform.translation.z = 3.0
            t.transform.rotation.w = 1.0
            br.sendTransform(t)

            p = PoseStamped()
            p.header.stamp = (now - Duration(seconds=0.1)).to_msg()  # transform is available at this time
            p.header.frame_id = 'aruco_map'
            p.pose.position.x = 0.5
            p.pose.orientation.w = 1.0
            pose_pub.publish(p)

        timer = node.create_timer(0.05, publish)
        try:
            assert wait(lambda: received, 15.0), 'no vpe messages'
            vpe = received[-1]
            assert vpe.header.frame_id == 'map'
            # the offset makes visual pose coincide with the local position
            assert abs(vpe.pose.position.x - 1) < 1e-6 and abs(vpe.pose.position.y - 2) < 1e-6 and \
                abs(vpe.pose.position.z - 3) < 1e-6

            assert wait(lambda: static, 5.0), 'no offset transform'
            offset = static[-1].transforms[0]
            assert offset.header.frame_id == 'map' and offset.child_frame_id == 'vpe_offset'
            assert abs(offset.transform.translation.x - 0.5) < 1e-6

            # reset makes the node publish the offset again
            count = len(static)
            reset = self.service_proxy('vpe_publisher/reset', Trigger)
            assert reset.client.wait_for_service(timeout_sec=5.0)
            assert reset().success
            assert wait(lambda: len(static) > count, 5.0), 'offset is not reset'
        finally:
            timer.cancel()

    def test_force_init(self):
        node = self.node
        received = []
        node.create_subscription(PoseStamped, 'force_init/vpe_publisher/vpe', received.append, 10)
        assert wait(lambda: len(received) >= 10, 30.0), 'no zero poses'
        zero = received[-1]
        assert zero.header.frame_id == 'mock_map', 'frame is not read from mavros'
        assert zero.pose.position.x == 0 and zero.pose.orientation.w == 1

        # local position appears: zero poses stop in force_init_duration
        local_pub = node.create_publisher(PoseStamped, 'force_init/mavros/local_position/pose', BEST_EFFORT)

        def publish():
            p = PoseStamped()
            p.header.stamp = node.get_clock().now().to_msg()
            local_pub.publish(p)

        timer = node.create_timer(0.1, publish)
        time.sleep(2.0)
        count = len(received)
        time.sleep(1.0)
        assert len(received) == count, 'zero poses are still published'
        timer.cancel()
