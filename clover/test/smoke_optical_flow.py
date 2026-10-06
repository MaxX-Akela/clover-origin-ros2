#!/usr/bin/env python3
# Smoke run for the optical_flow node with a mock camera (synthetic shifted frames):
# as a separate process and as a component loaded to a container.
# Usage (workspace sourced): python3 smoke_optical_flow.py

import math
import os
import signal
import subprocess
import sys
import threading
import time

os.environ.setdefault('ROS_DOMAIN_ID', '87')

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import QoSProfile, ReliabilityPolicy
from composition_interfaces.srv import LoadNode
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped, Vector3Stamped
from mavros_msgs.msg import OpticalFlowRad
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import StaticTransformBroadcaster

WIDTH, HEIGHT, FOCAL = 320, 240, 300.0
SHIFT_X, SHIFT_Y = 3, -2  # pixels per frame
RATE = 20.0
BEST_EFFORT = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)


def wait(condition, timeout):
    deadline = time.time() + timeout
    while time.time() < deadline and not condition():
        time.sleep(0.02)
    return condition()


def stop(proc):
    os.killpg(proc.pid, signal.SIGINT)
    proc.wait(timeout=10)


class MockCamera:
    def __init__(self, node):
        self.node = node
        self.image_pub = node.create_publisher(Image, 'image_raw', 1)
        self.info_pub = node.create_publisher(CameraInfo, 'camera_info', 1)
        rng = np.random.default_rng(1)
        self.frame = np.kron(rng.integers(0, 255, (HEIGHT // 8, WIDTH // 8)), np.ones((8, 8))).astype(np.uint8)
        node.create_timer(1 / RATE, self.publish)

    def publish(self):
        self.frame = np.roll(self.frame, (SHIFT_Y, SHIFT_X), axis=(0, 1))
        stamp = self.node.get_clock().now().to_msg()

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

        self.image_pub.publish(img)
        self.info_pub.publish(info)


def check_flow(shifts, flows, velos):
    assert wait(lambda: len(flows) >= 10 and len(shifts) >= 10 and len(velos) >= 10, 15.0), \
        'no flow messages: %d shift, %d flow, %d velocity' % (len(shifts), len(flows), len(velos))
    shift, flow, velo = shifts[-1], flows[-1], velos[-1]
    print('shift: %.2f %.2f px; flow: %.5f %.5f rad, quality %d, integration time %d us; velocity frame %s' %
          (shift.vector.x, shift.vector.y, flow.integrated_x, flow.integrated_y, flow.quality,
           flow.integration_time_us, velo.header.frame_id))
    assert abs(shift.vector.x - SHIFT_X) < 0.2 and abs(shift.vector.y - SHIFT_Y) < 0.2
    # camera frame coincides with the FCU frame in this run
    assert abs(flow.integrated_x - math.atan2(SHIFT_Y, FOCAL)) < 1e-3
    assert abs(flow.integrated_y + math.atan2(SHIFT_X, FOCAL)) < 1e-3
    assert math.isnan(flow.integrated_xgyro) and flow.distance == -1
    assert 0.5 / RATE * 1e6 < flow.integration_time_us < 2 / RATE * 1e6
    return velo


def test_standalone(node, shifts, flows, velos):
    debug = []
    debug_sub = node.create_subscription(Image, 'optical_flow/debug', debug.append, 1)
    proc = subprocess.Popen(['ros2', 'run', 'clover', 'optical_flow', '--ros-args',
                             '-p', 'local_frame:=map', '-p', 'fcu_frame:=base_link'], start_new_session=True)
    try:
        velo = check_flow(shifts, flows, velos)
        assert velo.header.frame_id == 'base_link'
        assert wait(lambda: debug, 5.0), 'no debug image'
        print('debug image: %dx%d %s' % (debug[-1].width, debug[-1].height, debug[-1].encoding))
        assert (debug[-1].width, debug[-1].height) == (128, 128)  # default ROI

        # enabled parameter (dynamic_reconfigure in ROS 1)
        params = AsyncParameterClient(node, 'optical_flow')
        assert params.wait_for_services(timeout_sec=5.0)
        future = params.set_parameters([Parameter('enabled', value=False)])
        assert wait(future.done, 5.0) and future.result().results[0].successful
        time.sleep(0.5)
        count = len(flows)
        time.sleep(1.0)
        print('flow messages in 1 s when disabled: %d' % (len(flows) - count))
        assert len(flows) == count, 'flow is published when disabled'

        future = params.set_parameters([Parameter('enabled', value=True)])
        assert wait(future.done, 5.0) and future.result().results[0].successful
        assert wait(lambda: len(flows) > count + 5, 5.0), 'flow is not published when enabled again'
    finally:
        stop(proc)
        node.destroy_subscription(debug_sub)


def test_component(node, executor, shifts, flows, velos):
    # frames are read from the parameters of (mock) mavros local_position node
    mavros = rclpy.create_node('local_position', namespace='mavros')
    mavros.declare_parameter('tf.frame_id', 'map')
    mavros.declare_parameter('tf.child_frame_id', 'mock_base_link')
    executor.add_node(mavros)

    container = subprocess.Popen(['ros2', 'run', 'rclcpp_components', 'component_container_mt'], start_new_session=True)
    try:
        # the same as `ros2 component load /ComponentManager clover clover::OpticalFlow -p disable_on_vpe:=true`
        load = node.create_client(LoadNode, 'ComponentManager/_container/load_node')
        assert load.wait_for_service(timeout_sec=10.0), 'no component container'
        request = LoadNode.Request(package_name='clover', plugin_name='clover::OpticalFlow')
        request.parameters = [Parameter('disable_on_vpe', value=True).to_parameter_msg()]
        future = load.call_async(request)
        assert wait(future.done, 30.0), 'component is not loaded'
        print('component: %s, success %s %s' % (future.result().full_node_name, future.result().success,
              future.result().error_message))
        assert future.result().success and future.result().full_node_name == '/optical_flow'

        del shifts[:], flows[:], velos[:]
        velo = check_flow(shifts, flows, velos)
        assert velo.header.frame_id == 'mock_base_link', 'frame is not read from mavros'

        # disable_on_vpe: no flow while visual pose is published
        vpe_pub = node.create_publisher(PoseStamped, 'mavros/vision_pose/pose', 1)

        def publish():
            vpe = PoseStamped()
            vpe.header.stamp = node.get_clock().now().to_msg()
            vpe_pub.publish(vpe)

        timer = node.create_timer(0.03, publish)
        time.sleep(0.5)
        count = len(flows)
        time.sleep(1.0)
        print('flow messages in 1 s with visual pose: %d' % (len(flows) - count))
        assert len(flows) == count, 'flow is published with visual pose'
        timer.cancel()
        assert wait(lambda: len(flows) > count + 5, 5.0), 'flow is not published after visual pose is lost'
    finally:
        stop(container)


def main():
    rclpy.init()
    node = rclpy.create_node('smoke_optical_flow')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    try:
        MockCamera(node)

        static_br = StaticTransformBroadcaster(node)
        transforms = []
        for fcu_frame in 'base_link', 'mock_base_link':
            t = TransformStamped()
            t.header.stamp = node.get_clock().now().to_msg()
            t.header.frame_id = fcu_frame
            t.child_frame_id = 'main_camera_optical' if fcu_frame == 'base_link' else 'base_link'
            t.transform.rotation.w = 1.0
            transforms.append(t)
        static_br.sendTransform(transforms)

        shifts, flows, velos = [], [], []
        node.create_subscription(Vector3Stamped, 'optical_flow/shift', shifts.append, 10)
        node.create_subscription(OpticalFlowRad, 'mavros/px4flow/raw/send', flows.append, 10)  # reliable, as mavros
        node.create_subscription(TwistStamped, 'optical_flow/angular_velocity', velos.append, 10)

        test_standalone(node, shifts, flows, velos)
        test_component(node, executor, shifts, flows, velos)
        print('OK')
    finally:
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
