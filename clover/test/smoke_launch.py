#!/usr/bin/env python3
# Smoke run for clover.launch.py: starts the launch file, waits for the key nodes and services
# and checks that no processes are left after the shutdown.
# Usage (workspace sourced):
#   python3 smoke_launch.py           # fcu_conn:=none main_camera:=false
#   python3 smoke_launch.py --mavros  # with mavros_node without an autopilot (needs GeographicLib geoid)
#   python3 smoke_launch.py --camera  # with main_camera.launch.py, aruco and a mock camera

import os
import signal
import subprocess
import sys
import time

os.environ.setdefault('ROS_DOMAIN_ID', '87')

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import QoSProfile, ReliabilityPolicy
from aruco_pose.msg import MarkerArray
from sensor_msgs.msg import CameraInfo, Image

WIDTH, HEIGHT = 320, 240

NODES = ['/simple_offboard', '/vpe_publisher', '/optical_flow', '/main_camera_container', '/led_effect',
         '/rangefinder_frame']
SERVICES = ['/navigate', '/navigate_global', '/get_telemetry', '/set_position', '/set_velocity', '/set_attitude',
            '/set_rates', '/set_altitude', '/set_yaw', '/set_yaw_rate', '/land', '/simple_offboard/release',
            '/vpe_publisher/reset']
TOPICS = ['/mavros/vision_pose/pose', '/mavros/px4flow/raw/send', '/simple_offboard/state']

MAVROS_NODES = ['/mavros', '/mavros_node', '/mavros_router', '/mavros_params', '/mavros/sys', '/mavros/local_position',
                '/mavros/setpoint_raw', '/mavros/distance_sensor', '/mavros/vision_pose', '/mavros/px4flow']
MAVROS_SERVICES = ['/mavros/set_mode', '/mavros/cmd/arming']
MAVROS_TOPICS = ['/mavros/state', '/mavros/local_position/pose', '/mavros/distance_sensor/rangefinder',
                 '/mavros/distance_sensor/rangefinder_sub', '/uas1/mavlink_sink']

CAMERA_NODES = ['/main_camera_frame', '/main_camera/main_camera_markers', '/aruco_detect']
CAMERA_TOPICS = ['/aruco_detect/markers', '/main_camera/camera_markers']


def spin_until(node, condition, timeout):
    deadline = time.time() + timeout
    while time.time() < deadline and not condition():
        rclpy.spin_once(node, timeout_sec=0.1)
    return condition()


def missing(node, nodes, services, topics):
    found_nodes = {(ns if ns.endswith('/') else ns + '/') + name for name, ns in node.get_node_names_and_namespaces()}
    found_services = {name for name, _ in node.get_service_names_and_types()}
    found_topics = {name for name, _ in node.get_topic_names_and_types()}
    return sorted(set(nodes) - found_nodes) + sorted(set(services) - found_services) + sorted(set(topics) - found_topics)


def get_param(node, name, param):
    client = AsyncParameterClient(node, name)
    assert client.wait_for_services(timeout_sec=5.0), 'no parameter services of %s' % name
    future = client.get_parameters([param])
    assert spin_until(node, future.done, 5.0), 'no response from %s' % name
    return rclpy.parameter.parameter_value_to_python(future.result().values[0])


def mock_camera(node):
    image_pub = node.create_publisher(Image, 'main_camera/image_raw', 1)
    info_pub = node.create_publisher(CameraInfo, 'main_camera/camera_info', 1)
    # calibration of the real camera rescaled to the image size (with a pinhole calibration
    # roi_rad of optical_flow gives a ROI outside of the image)
    path = os.path.join(get_package_share_directory('clover'), 'camera_info', 'fisheye_cam.yaml')
    with open(path) as f:
        calibration = yaml.safe_load(f)
    k = [float(v) for v in calibration['camera_matrix']['data']]
    for i in 0, 2:
        k[i] *= WIDTH / calibration['image_width']
    for i in 4, 5:
        k[i] *= HEIGHT / calibration['image_height']
    rng = np.random.default_rng(1)
    frame = np.kron(rng.integers(0, 255, (HEIGHT // 8, WIDTH // 8)), np.ones((8, 8))).astype(np.uint8)

    def publish():
        img = Image()
        img.header.stamp = node.get_clock().now().to_msg()
        img.header.frame_id = 'main_camera_optical'
        img.width, img.height, img.step = WIDTH, HEIGHT, WIDTH
        img.encoding = 'mono8'
        img.data = frame.tobytes()
        info = CameraInfo()
        info.header = img.header
        info.width, info.height = WIDTH, HEIGHT
        info.distortion_model = calibration['distortion_model']
        info.k = k
        info.d = calibration['distortion_coefficients']['data']
        image_pub.publish(img)
        info_pub.publish(info)

    node.create_timer(0.05, publish)


def main():
    mavros = '--mavros' in sys.argv
    camera = '--camera' in sys.argv

    cmd = ['ros2', 'launch', 'clover', 'clover.launch.py',
           'fcu_conn:=' + ('udp' if mavros else 'none'),
           'main_camera:=' + ('true' if camera else 'false'),
           'aruco:=' + ('true' if camera else 'false')]
    nodes, services, topics = list(NODES), list(SERVICES), list(TOPICS)
    if mavros:
        nodes += MAVROS_NODES
        services += MAVROS_SERVICES
        topics += MAVROS_TOPICS
    if camera:
        nodes += CAMERA_NODES
        topics += CAMERA_TOPICS

    print(' '.join(cmd))
    proc = subprocess.Popen(cmd, start_new_session=True)
    rclpy.init()
    node = rclpy.create_node('smoke_launch')
    try:
        if camera:
            mock_camera(node)

        ok = spin_until(node, lambda: not missing(node, nodes, services, topics), 60.0)
        assert ok, 'not found: %s' % ', '.join(missing(node, nodes, services, topics))
        print('found %d nodes, %d services, %d topics' % (len(nodes), len(services), len(topics)))
        assert proc.poll() is None, 'launch has exited'

        assert get_param(node, 'simple_offboard', 'terrain_frame_mode') == 'range'
        assert get_param(node, 'optical_flow', 'roi_rad') == 0.8
        assert get_param(node, 'vpe_publisher', 'offset_frame_id') == 'aruco_map'
        assert get_param(node, 'led_effect', 'notify.low_battery.threshold') == 3.6

        if mavros:
            assert get_param(node, 'mavros_node', 'fcu_url') == 'udp://@127.0.0.1:14557'
            assert get_param(node, 'mavros_node', 'gcs_url') == 'tcp-l://0.0.0.0:5760'
            # parameters of the plugins are set by mavros_params
            assert spin_until(node, lambda: get_param(node, 'mavros/sys', 'conn_timeout') == 8.0, 10.0)
            assert spin_until(node, lambda: get_param(node, 'mavros/local_position', 'tf.send') is True, 10.0)
            print('mavros parameters are set')

        if camera:
            markers = []
            node.create_subscription(MarkerArray, 'aruco_detect/markers', markers.append,
                                     QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
            assert spin_until(node, lambda: markers, 15.0), 'aruco_detect does not process the images'
            assert get_param(node, 'aruco_detect', 'length') == 0.22
            assert get_param(node, 'aruco_detect', 'known_vertical') == 'map'
            assert get_param(node, 'aruco_detect', 'cornerRefinementMethod') == 2
            print('aruco_detect processes the images of the mock camera')
    finally:
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=20)
        except subprocess.TimeoutExpired:
            pass
        node.destroy_node()
        rclpy.shutdown()

    # no processes should be left in the process group of the launch
    deadline = time.time() + 5.0
    left = True
    while left and time.time() < deadline:
        left = subprocess.run(['pgrep', '-g', str(proc.pid), '-a'], capture_output=True, text=True).stdout.strip()
        time.sleep(0.2)
    if left:
        os.killpg(proc.pid, signal.SIGKILL)
    assert not left, 'processes left after the shutdown:\n%s' % left
    print('launch exit code: %s, no processes left' % proc.returncode)
    print('OK')


if __name__ == '__main__':
    sys.exit(main())
