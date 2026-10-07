# Test of the optical_flow node with a mock camera (synthetic shifted frames):
# as a separate process and as a component loaded to a container.

import math
import os
import sys

import launch
import launch_testing.actions
import numpy as np
import pytest
from launch_ros.actions import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from composition_interfaces.srv import LoadNode
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped, Vector3Stamped
from mavros_msgs.msg import OpticalFlowRad
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import StaticTransformBroadcaster

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from clover_test_utils import CloverTestCase, clover_node, mock_local_position, wait  # noqa: E402

WIDTH, HEIGHT, FOCAL = 320, 240, 300.0
SHIFT_X, SHIFT_Y = 3, -2  # pixels per frame
RATE = 20.0
NAMESPACES = '', 'component/'  # of the separate process and of the component


@pytest.mark.launch_test
def generate_test_description():
    return launch.LaunchDescription([
        # the frames are set with the parameters
        clover_node('optical_flow', local_frame='map', fcu_frame='base_link'),
        # the component reads the frames from the parameters of (mock) mavros local_position node
        mock_local_position('map', 'mock_base_link', namespace='/component/mavros'),
        Node(package='rclcpp_components', executable='component_container_mt', name='ComponentManager',
             output='screen'),
        launch_testing.actions.ReadyToTest(),
    ])


class MockCamera:
    def __init__(self, node):
        self.node = node
        self.image_pubs = [node.create_publisher(Image, ns + 'image_raw', 1) for ns in NAMESPACES]
        self.info_pubs = [node.create_publisher(CameraInfo, ns + 'camera_info', 1) for ns in NAMESPACES]
        rng = np.random.default_rng(1)
        self.frame = np.kron(rng.integers(0, 255, (HEIGHT // 8, WIDTH // 8)), np.ones((8, 8))).astype(np.uint8)
        self.index = 0  # number of the last published frame
        self.stamps = {}  # (sec, nanosec) of a published frame -> its number
        node.create_timer(1 / RATE, self.publish)

    def publish(self):
        self.frame = np.roll(self.frame, (SHIFT_Y, SHIFT_X), axis=(0, 1))
        stamp = self.node.get_clock().now().to_msg()
        # the timer can be late (loaded machine), so the tests use the stamps and numbers of the frames, not the rate
        self.stamps[(stamp.sec, stamp.nanosec)] = self.index + 1
        self.index += 1

        img = Image()
        img.header.stamp = stamp
        img.header.frame_id = 'main_camera_optical'
        img.width, img.height, img.step = WIDTH, HEIGHT, WIDTH
        img.encoding = 'mono8'
        img.data = self.frame.tobytes()

        info = CameraInfo()
        info.header = img.header
        info.width, info.height = WIDTH, HEIGHT
        info.k = [FOCAL, 0.0, WIDTH / 2, 0.0, FOCAL, HEIGHT / 2, 0.0, 0.0, 1.0]
        info.d = [0.0] * 5

        for pub in self.image_pubs:
            pub.publish(img)
        for pub in self.info_pubs:
            pub.publish(info)


class TestOpticalFlow(CloverTestCase):
    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        node = cls.node
        cls.camera = MockCamera(node)

        cls.static_br = StaticTransformBroadcaster(node)
        transforms = []
        for fcu_frame in 'base_link', 'mock_base_link':
            t = TransformStamped()
            t.header.stamp = node.get_clock().now().to_msg()
            t.header.frame_id = fcu_frame
            t.child_frame_id = 'main_camera_optical' if fcu_frame == 'base_link' else 'base_link'
            t.transform.rotation.w = 1.0
            transforms.append(t)
        cls.static_br.sendTransform(transforms)

    def subscribe(self, ns):
        shifts, flows, velos = [], [], []
        self.node.create_subscription(Vector3Stamped, ns + 'optical_flow/shift', shifts.append, 10)
        self.node.create_subscription(OpticalFlowRad, ns + 'mavros/px4flow/raw/send', flows.append, 10)  # reliable, as mavros
        self.node.create_subscription(TwistStamped, ns + 'optical_flow/angular_velocity', velos.append, 10)
        return shifts, flows, velos

    @staticmethod
    def stamp_ns(header):
        return header.stamp.sec * 10 ** 9 + header.stamp.nanosec

    def frame_number(self, header):
        """Number of the published frame by the stamp of a message made of it."""
        return self.camera.stamps.get((header.stamp.sec, header.stamp.nanosec))

    def previous_frame_number(self, flow):
        """Number of the frame the flow was integrated from.

        integration_time_us is the difference of the stamps of two frames (truncated to microseconds),
        so stamp - integration_time_us must be the stamp of a published frame."""
        previous = self.stamp_ns(flow.header) - flow.integration_time_us * 1000
        for (sec, nanosec), number in list(self.camera.stamps.items()):
            if abs(sec * 10 ** 9 + nanosec - previous) <= 2000:
                return number
        return None

    def frames_integrated(self, flow):
        """How many frames passed between the two frames of the flow (1 if no frame was lost or skipped)."""
        current, previous = self.frame_number(flow.header), self.previous_frame_number(flow)
        return None if current is None or previous is None else current - previous

    def check_flow(self, shifts, flows, velos):
        assert wait(lambda: len(flows) >= 10 and len(shifts) >= 10 and len(velos) >= 10, 30.0), \
            'no flow messages: %d shift, %d flow, %d velocity' % (len(shifts), len(flows), len(velos))

        # integration_time_us is the exact difference of the stamps of the frames, whatever the load is
        for flow in list(flows):
            assert self.previous_frame_number(flow) is not None, \
                'integration_time_us %d does not match the stamps of the published frames' % flow.integration_time_us

        # the checks of the values need a flow made of two successive frames: under load a frame can be lost
        # (then the shift is twice as large) and the timer of the camera can be late
        def last_single_frame_flow():
            for flow in reversed(list(flows)):
                if self.frames_integrated(flow) == 1:
                    return flow

        assert wait(lambda: last_single_frame_flow() is not None, 30.0), \
            'no flow of two successive frames in %d messages' % len(flows)
        flow = last_single_frame_flow()

        def with_stamp(messages):
            found = [m for m in list(messages) if self.stamp_ns(m.header) == self.stamp_ns(flow.header)]
            return found[0] if found else None

        assert wait(lambda: with_stamp(shifts) is not None and with_stamp(velos) is not None, 5.0), \
            'no shift or velocity made of the same frame as the flow'
        shift, velo = with_stamp(shifts), with_stamp(velos)
        assert abs(shift.vector.x - SHIFT_X) < 0.2 and abs(shift.vector.y - SHIFT_Y) < 0.2
        # camera frame coincides with the FCU frame in this run
        assert abs(flow.integrated_x - math.atan2(SHIFT_Y, FOCAL)) < 1e-3
        assert abs(flow.integrated_y + math.atan2(SHIFT_X, FOCAL)) < 1e-3
        assert math.isnan(flow.integrated_xgyro) and flow.distance == -1
        # the period of the camera: the median of the flows of successive frames, one late timer does not matter
        times = sorted(f.integration_time_us for f in list(flows) if self.frames_integrated(f) == 1)
        assert 0.5 / RATE * 1e6 < times[len(times) // 2] < 2 / RATE * 1e6
        return velo

    def flows_after(self, flows, number):
        """Flows made of the frames published after the frame with the given number."""
        return [f for f in list(flows) if (self.frame_number(f.header) or 0) > number]

    def test_standalone(self):
        node = self.node
        shifts, flows, velos = self.subscribe('')
        debug = []
        node.create_subscription(Image, 'optical_flow/debug', debug.append, 1)

        velo = self.check_flow(shifts, flows, velos)
        assert velo.header.frame_id == 'base_link'
        assert wait(lambda: debug, 5.0), 'no debug image'
        assert (debug[-1].width, debug[-1].height) == (128, 128)  # default ROI

        # enabled parameter (dynamic_reconfigure in ROS 1)
        params = AsyncParameterClient(node, 'optical_flow')
        assert params.wait_for_services(timeout_sec=5.0)
        future = params.set_parameters([Parameter('enabled', value=False)])
        assert wait(future.done, 30.0) and future.result().results[0].successful
        # the frames published before the response may be still in flight, the next ones must give no flow
        disabled_at = self.camera.index
        assert wait(lambda: self.camera.index >= disabled_at + 10, 30.0), 'the camera stopped'
        assert not self.flows_after(flows, disabled_at), 'flow is published when disabled'

        future = params.set_parameters([Parameter('enabled', value=True)])
        assert wait(future.done, 30.0) and future.result().results[0].successful
        enabled_at = self.camera.index
        assert wait(lambda: len(self.flows_after(flows, enabled_at)) > 5, 30.0), \
            'flow is not published when enabled again'

    def test_component(self):
        node = self.node
        shifts, flows, velos = self.subscribe('component/')

        # the same as `ros2 component load /ComponentManager clover clover::OpticalFlow
        # --node-namespace /component -p disable_on_vpe:=true`
        assert wait(lambda: '/component/mavros/local_position' in node.get_fully_qualified_node_names(), 20.0), \
            'no mock mavros node'
        load = node.create_client(LoadNode, 'ComponentManager/_container/load_node')
        assert load.wait_for_service(timeout_sec=10.0), 'no component container'
        request = LoadNode.Request(package_name='clover', plugin_name='clover::OpticalFlow',
                                   node_namespace='/component')
        request.parameters = [Parameter('disable_on_vpe', value=True).to_parameter_msg()]
        res = self.call(load, request)
        assert res.success and res.full_node_name == '/component/optical_flow', res.error_message

        velo = self.check_flow(shifts, flows, velos)
        assert velo.header.frame_id == 'mock_base_link', 'frame is not read from mavros'

        # disable_on_vpe: no flow while visual pose is published
        vpe_pub = node.create_publisher(PoseStamped, 'component/mavros/vision_pose/pose', 1)
        vpe_stamps = []

        def publish():
            vpe = PoseStamped()
            vpe.header.stamp = node.get_clock().now().to_msg()
            vpe_stamps.append(self.stamp_ns(vpe.header))
            vpe_pub.publish(vpe)

        timer = node.create_timer(0.03, publish)
        started_at = self.camera.index
        assert wait(lambda: self.camera.index >= started_at + 30, 60.0), 'the camera stopped'
        # the node drops a frame if the last visual pose is less than 0.1 s older than the stamp of the frame
        # (the first frames are skipped: the poses are not delivered yet)
        for flow in self.flows_after(flows, started_at + 5):
            stamp = self.stamp_ns(flow.header)
            previous = [v for v in list(vpe_stamps) if v <= stamp]
            assert not previous or stamp - max(previous) >= 0.1e9, 'flow is published with visual pose'
        timer.cancel()
        stopped_at = self.camera.index
        assert wait(lambda: len(self.flows_after(flows, stopped_at + 10)) > 5, 30.0), \
            'flow is not published after visual pose is lost'
