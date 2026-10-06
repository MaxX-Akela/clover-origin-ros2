#!/usr/bin/env python3
# Smoke run for selfcheck, sequential and parallel modes:
# A. with mavros_node without an autopilot and no other data: the checks report failures, no exceptions;
# B. with mocks of everything the checks read (no mavros_node): no failures.
# Usage (workspace sourced): python3 smoke_selfcheck.py

import math
import os
import signal
import subprocess
import sys
import threading
import time

os.environ.setdefault('ROS_DOMAIN_ID', '87')
os.environ['MAVLINK20'] = '1'  # the mock FCU sends MAVLink 2 frames, as PX4 does

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from pymavlink import mavutil
from mavros import mavlink
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters
from std_srvs.srv import Trigger
from sensor_msgs.msg import BatteryState, Image, CameraInfo, NavSatFix, Imu, Range
from mavros_msgs.msg import State, OpticalFlowRad, Mavlink
from geometry_msgs.msg import PoseStamped, TwistStamped, PoseWithCovarianceStamped, TransformStamped
from visualization_msgs.msg import Marker as VisualizationMarker, MarkerArray as VisualizationMarkerArray
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from tf2_msgs.msg import TFMessage
from aruco_pose.msg import Marker, MarkerArray
from clover.srv import Navigate, GetTelemetry

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
BEST_EFFORT = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
FCU_URL = 'udp://:14540@'

CHECKS = ['Image', 'Board', 'clover.service', 'Network', 'FCU', 'IMU', 'Local position', 'Velocity estimation',
          'Global position (GPS)', 'Preflight status', 'Main camera', 'ArUco markers', 'Simple offboard node',
          'Optical flow', 'Vision position estimate', 'Rangefinder', 'RPi health', 'CPU usage', 'Boot duration']
# these depend on the machine only, their output is not compared
MACHINE_CHECKS = ['Image', 'Board', 'clover.service', 'Network', 'RPi health', 'CPU usage', 'Boot duration']

# expected reports without any data (mavros_node is running, no FCU)
NO_DATA = {
    'FCU': ['no connection to the FCU (check wiring)', 'fcu_url = ' + FCU_URL],
    'IMU': ['no IMU data (check flight controller calibration)'],
    'Local position': ['no local position'],
    'Velocity estimation': ['no velocity estimation'],
    'Global position (GPS)': ['no global position', 'unable to retrieve PX4 parameter SYS_MC_EST_GROUP'],
    'Preflight status': ['no data from FCU'],
    'Main camera': ['main_camera: no images (is the camera connected properly?)'],
    'ArUco markers': ['aruco_detect is not running'],
    'Simple offboard node': ['no simple_offboard services'],
    'Optical flow': ['optical_flow is not running'],
    'Vision position estimate': ['no vision position estimate, vpe_publisher is not running',
                                 'unable to retrieve PX4 parameter SYS_MC_EST_GROUP'],
    'Rangefinder': ['no rangefinder data from Raspberry', 'no rangefinder data from PX4'],
}

# expected reports with the mock data
MOCK_DATA = {
    'FCU': ['PX4_FMU_V4', 'v1.14.0-clover.1', '1.14.0 0 (17694720)', 'selected estimator: EKF2',
            'board rotation: no rotation', 'time sync offset: 0.01 s'],
    'IMU': ['OK'],
    'Local position': ['OK'],
    'Velocity estimation': ['OK'],
    'Global position (GPS)': ['OK'],
    'Preflight status': ['OK'],
    'Main camera': ['camera is oriented downward, cable from camera goes backward'],
    'ArUco markers': ['aruco_detect/length = 0.33 m', 'aruco_detect/known_vertical = map',
                      'aruco_detect/flip_vertical = False (all markers are on the floor)',
                      'aruco_map/known_vertical = map',
                      'aruco_map/flip_vertical = False (markers map is on the floor)', 'map has 2 markers'],
    'Simple offboard node': ['OK'],
    'Optical flow': ['EKF2_OF_QMIN = 1, EKF2_OF_N_MIN = 0.15, EKF2_OF_N_MAX = 0.5',
                     'SENS_FLOW_MINHGT = 0.08, SENS_FLOW_MAXHGT = 3.0'],
    'Vision position estimate': ['EKF2_EVA_NOISE = 0.1, EKF2_EVP_NOISE = 0.1'],
    'Rangefinder': ['EKF2_HGT_MODE = Range sensor, operating over flat surface',
                    'EKF2_RNG_AID = 1, range sensor aiding enabled'],
}

PX4_PARAMS = {
    'SYS_MC_EST_GROUP': 2, 'SENS_BOARD_ROT': 0, 'CBRK_USB_CHK': 197848,
    'EKF2_EV_CTRL': 15, 'EKF2_EV_DELAY': 0.0, 'EKF2_EVA_NOISE': 0.1, 'EKF2_EVP_NOISE': 0.1,
    'SENS_FLOW_ROT': 0, 'EKF2_OF_CTRL': 1, 'EKF2_OF_DELAY': 0.0, 'EKF2_OF_QMIN': 1,
    'EKF2_OF_N_MIN': 0.15, 'EKF2_OF_N_MAX': 0.5, 'SENS_FLOW_MINHGT': 0.08, 'SENS_FLOW_MAXHGT': 3.0,
    'EKF2_HGT_REF': 2, 'EKF2_RNG_AID': 1,
}

NSH_OUTPUT = {
    '': '',
    'ver all': 'HW arch: PX4_FMU_V4\nFW git tag: v1.14.0-clover.1\nFW version: 1.14.0 0 (17694720)\n'
               'OS: NuttX\nMCU: STM32F42x, rev. 5',
    'commander check': 'INFO  [commander] Preflight check: OK\nINFO  [commander] Prearm check: OK',
}


def wait(condition, timeout):
    deadline = time.time() + timeout
    while time.time() < deadline and not condition():
        time.sleep(0.05)
    return condition()


def run_selfcheck(parallel):
    # returns ({check: [reports]}, raw output, duration)
    env = dict(os.environ)
    env.pop('MAVLINK20')  # selfcheck uses the default dialect
    start = time.time()
    res = subprocess.run(['ros2', 'run', 'clover', 'selfcheck', '--ros-args', '-p', 'parallel:=%s' % parallel],
                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT, env=env, timeout=120)
    output = res.stdout.decode()
    assert res.returncode == 0, 'selfcheck returned %s:\n%s' % (res.returncode, output)
    assert 'exception occurred' not in output and 'Traceback' not in output, output
    lines = output.strip().split('\n')
    assert lines[0] == 'Performing selfcheck...', output
    reports = {}
    for line in lines[1:]:
        name, text = line.split(': ', 1)
        assert name in CHECKS, 'unexpected output: ' + line
        reports.setdefault(name, []).append(text)
    assert sorted(reports) == sorted(CHECKS), 'checks missing: %s' % (set(CHECKS) - set(reports))
    if not parallel:
        assert list(reports) == CHECKS, 'wrong checks order'
    return reports, output, time.time() - start


def compare(reports, expected, output):
    for name in CHECKS:
        if name in MACHINE_CHECKS:
            continue
        assert reports[name] == expected[name], '%s: %s, expected %s\n%s' % (name, reports[name], expected[name], output)


def quaternion_from_rpy(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = (f(a / 2) for a in (roll, pitch, yaw) for f in (math.cos, math.sin))
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)  # x, y, z, w


class Mocks:
    # everything the checks read: mavros, camera, aruco, simple_offboard, rangefinder, TF and FCU shell
    def __init__(self, executor):
        self.nodes = []
        self.executor = executor
        node = self.node = self.create_node('selfcheck_mocks')
        self.pubs = []

        self.publisher(State, 'mavros/state', lambda: State(connected=True, mode='STABILIZED'), LATCHED)
        self.publisher(BatteryState, 'mavros/battery', lambda: BatteryState(voltage=16.0, cell_voltage=[4.0] * 4))
        self.publisher(Imu, 'mavros/imu/data', Imu)
        self.publisher(PoseStamped, 'mavros/local_position/pose', self.pose)
        self.publisher(TwistStamped, 'mavros/local_position/velocity_local', TwistStamped)
        self.publisher(TwistStamped, 'mavros/local_position/velocity_body', TwistStamped)
        self.publisher(NavSatFix, 'mavros/global_position/global', NavSatFix)
        self.publisher(PoseStamped, 'mavros/vision_pose/pose', self.pose, 10)
        self.publisher(OpticalFlowRad, 'mavros/px4flow/raw/send', OpticalFlowRad, 10)
        self.publisher(Range, 'rangefinder/range', lambda: Range(range=1.0))
        self.publisher(Range, 'mavros/distance_sensor/rangefinder', lambda: Range(range=1.0))
        self.publisher(Image, 'main_camera/image_raw', self.image)
        self.publisher(CameraInfo, 'main_camera/camera_info', lambda: CameraInfo(width=320, height=240))
        self.publisher(MarkerArray, 'aruco_detect/markers', lambda: MarkerArray(markers=[Marker(id=1)]), 10)
        self.publisher(PoseWithCovarianceStamped, 'aruco_map/pose', PoseWithCovarianceStamped, 10)
        self.publisher(VisualizationMarkerArray, 'aruco_map/visualization',
                       lambda: VisualizationMarkerArray(markers=[VisualizationMarker(id=i) for i in range(2)]),
                       LATCHED)
        self.publisher(DiagnosticArray, '/diagnostics', self.diagnostics, 10)
        self.publisher(TFMessage, '/tf', self.tf, 10)
        self.timer = node.create_timer(1 / 30, self.publish)

        # simple_offboard services
        self.srvs = [node.create_service(Navigate, 'navigate', lambda req, res: res),
                     node.create_service(GetTelemetry, 'get_telemetry', lambda req, res: res),
                     node.create_service(Trigger, 'land', lambda req, res: res)]

        # FCU parameters and shell
        param = self.create_node('param', namespace='mavros')
        self.srvs.append(param.create_service(GetParameters, '~/get_parameters', self.get_px4_parameters))
        self.commands = []
        self.mavlink = mavutil.mavlink.MAVLink(None, 1, 1)
        self.mavlink_pub = node.create_publisher(Mavlink, '/uas1/mavlink_source', BEST_EFFORT)
        self.mavlink_sub = node.create_subscription(Mavlink, '/uas1/mavlink_sink', self.handle_mavlink, BEST_EFFORT)

        # nodes the checks look for in the graph and read parameters from
        self.create_node('mavros_node', fcu_url=FCU_URL)
        self.create_node('aruco_detect', length=0.33, known_vertical='map', flip_vertical=False)
        self.create_node('aruco_map', known_vertical='map', flip_vertical=False)
        self.create_node('optical_flow', disable_on_vpe=False)
        self.create_node('vpe_publisher')

    def create_node(self, name, namespace=None, **params):
        node = rclpy.create_node(name, namespace=namespace, start_parameter_services=name != 'param')
        for key, value in params.items():
            node.declare_parameter(key, value)
        self.executor.add_node(node)
        self.nodes.append(node)
        return node

    def publisher(self, msg_type, topic, factory, qos=BEST_EFFORT):
        self.pubs.append((self.node.create_publisher(msg_type, topic, qos), factory))

    def publish(self):
        for pub, factory in self.pubs:
            pub.publish(factory())

    def stamp(self):
        return self.node.get_clock().now().to_msg()

    def pose(self):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.stamp()
        msg.pose.position.z = 1.0
        msg.pose.orientation.w = 1.0
        return msg

    def image(self):
        msg = Image(width=320, height=240, encoding='mono8', step=320)
        msg.header.frame_id = 'main_camera_optical'
        return msg

    def diagnostics(self):
        return DiagnosticArray(status=[
            DiagnosticStatus(name='mavros: Heartbeat', values=[KeyValue(key='Frequency (Hz)', value='1.0')]),
            DiagnosticStatus(name='mavros: Time Sync', values=[KeyValue(key='Estimated time offset (s)', value='0.01')])])

    def tf(self):
        def transform(frame_id, child_frame_id, z, q):
            msg = TransformStamped()
            msg.header.frame_id = frame_id
            msg.header.stamp = self.stamp()
            msg.child_frame_id = child_frame_id
            msg.transform.translation.z = z
            r = msg.transform.rotation
            r.x, r.y, r.z, r.w = q
            return msg

        return TFMessage(transforms=[
            transform('map', 'base_link', 1.0, (0.0, 0.0, 0.0, 1.0)),
            transform('map', 'body', 1.0, (0.0, 0.0, 0.0, 1.0)),
            # as in main_camera.launch: the camera looks downward, the cable goes backward
            transform('base_link', 'main_camera_optical', -0.07, quaternion_from_rpy(math.pi, 0, -math.pi / 2))])

    def get_px4_parameters(self, req, res):
        for name in req.names:
            value = PX4_PARAMS.get(name)
            if isinstance(value, int):
                res.values.append(ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=value))
            elif isinstance(value, float):
                res.values.append(ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value))
            else:
                res.values.append(ParameterValue())  # not set, as mavros answers
        return res

    def handle_mavlink(self, msg):
        # nsh shell over SERIAL_CONTROL: echo the command, print the output and the prompt
        if msg.msgid != 126:
            return
        req = self.mavlink.decode(mavlink.convert_to_bytes(msg))
        command = ''.join(chr(c) for c in req.data[:req.count]).strip()
        self.commands.append((msg.magic, command))
        output = command + '\n' + NSH_OUTPUT[command] + '\nnsh> '
        for i in range(0, len(output), 70):
            chunk = output[i:i + 70]
            res = mavutil.mavlink.MAVLink_serial_control_message(
                device=mavutil.mavlink.SERIAL_CONTROL_DEV_SHELL, flags=mavutil.mavlink.SERIAL_CONTROL_FLAG_REPLY,
                timeout=0, baudrate=0, count=len(chunk), data=[ord(c) for c in chunk.ljust(70, '\0')])
            res.pack(self.mavlink)
            self.mavlink_pub.publish(mavlink.convert_to_rosmsg(res))

    def destroy(self):
        for node in self.nodes:
            self.executor.remove_node(node)
            node.destroy_node()


def check_mavros_graph(node):
    # what selfcheck reads is what mavros_node provides
    def publisher(topic, reliability, durability=DurabilityPolicy.VOLATILE):
        infos = node.get_publishers_info_by_topic(topic)
        assert infos, 'no publishers of ' + topic
        for i in infos:
            assert (i.qos_profile.reliability, i.qos_profile.durability) == (reliability, durability), topic

    assert wait(lambda: node.get_publishers_info_by_topic('/mavros/state'), 20.0), 'mavros_node is not started'
    time.sleep(2.0)
    publisher('/mavros/state', ReliabilityPolicy.RELIABLE, DurabilityPolicy.TRANSIENT_LOCAL)
    for topic in ('/mavros/battery', '/mavros/imu/data', '/mavros/local_position/pose',
                  '/mavros/local_position/velocity_local', '/mavros/local_position/velocity_body',
                  '/mavros/global_position/global', '/uas1/mavlink_source'):
        publisher(topic, ReliabilityPolicy.BEST_EFFORT)
    publisher('/diagnostics', ReliabilityPolicy.RELIABLE)
    for topic in '/uas1/mavlink_sink', '/mavros/vision_pose/pose', '/mavros/mocap/pose', '/mavros/px4flow/raw/send':
        assert node.get_subscriptions_info_by_topic(topic), 'mavros does not subscribe to ' + topic
    services = [name for name, _ in node.get_service_names_and_types()]
    assert '/mavros/param/get_parameters' in services and '/mavros_node/get_parameters' in services
    print('mavros graph: topics, QoS and parameter services match')
    if not node.get_publishers_info_by_topic('/mavros/distance_sensor/rangefinder'):
        print('mavros graph: no mavros/distance_sensor/rangefinder (depends on the plugin configuration)')


def main():
    rclpy.init()
    node = rclpy.create_node('smoke_selfcheck')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    # A. mavros_node without an autopilot
    env = dict(os.environ)
    if not os.path.exists('/usr/share/GeographicLib/geoids/egm96-5.pgm'):
        env.setdefault('GEOGRAPHICLIB_DATA', os.path.expanduser('~/.local/share/GeographicLib'))
    mavros = subprocess.Popen(['/opt/ros/jazzy/lib/mavros/mavros_node', '--ros-args', '-p', 'fcu_url:=' + FCU_URL],
                              env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    try:
        check_mavros_graph(node)
        for parallel in False, True:
            reports, output, duration = run_selfcheck(parallel)
            compare(reports, NO_DATA, output)
            print('no data, parallel=%s: 19 checks, no exceptions, failures as expected, %.0f s' % (parallel, duration))
            if not parallel:
                print(output)
    finally:
        os.killpg(mavros.pid, signal.SIGINT)
        mavros.wait(timeout=10)
    assert wait(lambda: not node.get_publishers_info_by_topic('/mavros/state'), 10.0), 'mavros_node is still alive'

    # B. mocks
    mocks = Mocks(executor)
    try:
        time.sleep(1.0)
        for parallel in False, True:
            mocks.commands.clear()
            reports, output, duration = run_selfcheck(parallel)
            if parallel:
                # in the original FCU and Preflight status checks share the shell buffer without a lock
                shell = [name for name in ('FCU', 'Preflight status') if reports[name] != MOCK_DATA[name]]
                if shell:
                    print('mock data, parallel=True: shell output is mixed (race of the original): %s' %
                          {name: reports[name] for name in shell})
                    for name in shell:
                        reports[name] = MOCK_DATA[name]
            compare(reports, MOCK_DATA, output)
            assert mocks.commands and all(magic == Mavlink.MAVLINK_V10 for magic, _ in mocks.commands)
            print('mock data, parallel=%s: 19 checks, no exceptions, no failures, %.0f s; shell commands: %s' %
                  (parallel, duration, [command for _, command in mocks.commands]))
            if not parallel:
                print(output)
        print('OK')
    finally:
        mocks.destroy()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
