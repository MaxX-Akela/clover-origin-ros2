#!/usr/bin/env python3
# coding=utf-8

# Copyright (C) 2018 Copter Express Technologies
#
# Author: Oleg Kalachev <okalachev@gmail.com>
#
# Distributed under MIT License (available at https://opensource.org/licenses/MIT).
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# Threading rules:
# - the node is spun by an executor in a background thread; its callbacks never block, they
#   only store a value and set an event;
# - the checks run in the main thread or (parallel mode) in their own threads and do all the
#   blocking waits there;
# - subscriptions and clients are created and destroyed under entities_lock.

import os, sys
import math
import subprocess
import re
import time
from collections import OrderedDict
import traceback
import threading
from threading import Event, Thread, Lock
import numpy
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.parameter import parameter_value_to_python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import tf2_ros
import tf2_geometry_msgs
from pymavlink import mavutil
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import BatteryState, Image, CameraInfo, NavSatFix, Imu, Range
from mavros_msgs.msg import State, OpticalFlowRad, Mavlink
from geometry_msgs.msg import PoseStamped, TwistStamped, PoseWithCovarianceStamped, Vector3Stamped
from visualization_msgs.msg import MarkerArray as VisualizationMarkerArray
from diagnostic_msgs.msg import DiagnosticArray
from aruco_pose.msg import MarkerArray
from mavros import mavlink
import locale


# the node, tf buffer and mavlink publisher are created in main()
node = None
tf_buffer = None
tf_listener = None
callback_group = ReentrantCallbackGroup()
entities_lock = Lock()

# volatile best effort subscription matches any publisher
BEST_EFFORT = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
# for latched topics (mavros/state, aruco_map/visualization)
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)

# mavros_node holds the connection parameters (fcu_url), mavros/param holds the FCU parameters
MAVROS_NODE = 'mavros_node'
MAVROS_PARAM_NODE = 'mavros/param'


thread_local = threading.local()
reports_lock = Lock()


# formatting colors
if sys.stderr.isatty():
    GREY = '\033[90m'
    GREEN = '\033[92m'
    RED = '\033[31m'
    END = '\033[0m'
else:
    GREY = GREEN = RED = END = ''


class ROSException(Exception):
    pass


class ServiceException(Exception):
    pass


def get_time():
    return node.get_clock().now().nanoseconds / 1e9


def failure(text, *args):
    msg = text % args
    thread_local.reports += [{'failure': msg}]


def info(text, *args):
    msg = text % args
    thread_local.reports += [{'info': msg}]


def check(name):
    def inner(fn):
        def wrapper(*args, **kwargs):
            start = get_time()
            thread_local.reports = []
            try:
                fn(*args, **kwargs)
            except Exception as e:
                traceback.print_exc()
                node.get_logger().error('%s: exception occurred' % name)
            with reports_lock:
                for report in thread_local.reports:
                    if 'failure' in report:
                        node.get_logger().error('%s: %s' % (name, report['failure']))
                    elif 'info' in report:
                        node.get_logger().info(GREY + name + END + ': ' + report['info'])
                if not thread_local.reports:
                    node.get_logger().info(GREY + name + END + ': ' + GREEN + 'OK' + END)
                if node.get_parameter('time').value:
                    node.get_logger().info('%s: %.1f sec' % (name, get_time() - start))
        return wrapper
    return inner


def ff(value, precision=2):
    # safely format float or int
    if value is None:
        return RED + '???' + END
    if isinstance(value, float):
        return ('{:.' + str(precision + 1) + '}').format(value)
    elif isinstance(value, int):
        return str(value)


def wait_for_message(topic, topic_type, timeout, qos=BEST_EFFORT):
    # rclpy.wait_for_message can't be used here: the background executor may take the message first
    e = Event()
    def cb(msg):
        if not e.is_set():
            cb.msg = msg
            e.set()

    with entities_lock:
        sub = node.create_subscription(topic_type, topic, cb, qos, callback_group=callback_group)
    try:
        if not e.wait(timeout):
            raise ROSException('timeout exceeded while waiting for message on topic %s' % topic)
        return cb.msg
    finally:
        with entities_lock:
            node.destroy_subscription(sub)


def wait_for_service(service, timeout):
    name = node.resolve_service_name(service)
    deadline = time.time() + timeout
    while True:
        if any(s == name for s, _ in node.get_service_names_and_types()):
            return
        if time.time() >= deadline:
            raise ROSException('timeout exceeded while waiting for service %s' % name)
        time.sleep(0.1)


param_clients = {}


def call_get_parameters(node_name, names, timeout=3.0):
    # read parameters of another node
    with entities_lock:
        if node_name not in param_clients:
            param_clients[node_name] = node.create_client(GetParameters, node_name + '/get_parameters',
                                                          callback_group=callback_group)
        client = param_clients[node_name]

    service = node.resolve_service_name(node_name + '/get_parameters')
    if not client.wait_for_service(timeout_sec=1.0):
        raise ServiceException('service [%s] unavailable' % service)

    e = Event()
    future = client.call_async(GetParameters.Request(names=names))
    future.add_done_callback(lambda _: e.set())
    if not e.wait(timeout) and not future.done():
        client.remove_pending_request(future)
        raise ServiceException('service [%s] did not respond' % service)
    return future.result().values


def get_node_param(node_name, name, *default):
    # get parameter of another node; without default raises KeyError if the parameter is not available
    try:
        values = call_get_parameters(node_name, [name], timeout=1.0)
    except ServiceException:
        msg = 'can\'t read parameters of %s node' % node_name
        if {'info': msg} not in thread_local.reports:
            info('%s', msg)
        values = []

    if not values or values[0].type == ParameterType.PARAMETER_NOT_SET:
        if default:
            return default[0]
        raise KeyError(name)
    return parameter_value_to_python(values[0])


def get_param(name, default=None, strict=True):
    try:
        values = call_get_parameters(MAVROS_PARAM_NODE, [name])
    except ServiceException as e:
        failure('%s: %s', name, str(e))
        return None

    if not values or values[0].type not in (ParameterType.PARAMETER_INTEGER, ParameterType.PARAMETER_DOUBLE):
        if strict:
            failure('unable to retrieve PX4 parameter %s', name)
        return default
    else:
        if values[0].type == ParameterType.PARAMETER_INTEGER:
            if values[0].integer_value != 0:
                return values[0].integer_value
            return 0.0
        return values[0].double_value


def get_paramf(name, precision=2):
    return ff(get_param(name), precision)


def euler_from_quaternion(quaternion, axes='rzyx'):
    # yaw, pitch, roll from quaternion (x, y, z, w), replaces tf.transformations.euler_from_quaternion
    assert axes == 'rzyx'
    x, y, z, w = quaternion
    n = x * x + y * y + z * z + w * w
    if n < numpy.finfo(float).eps * 4.0:
        return 0.0, 0.0, 0.0
    s = 2.0 / n
    # rotation matrix elements
    m00 = 1.0 - s * (y * y + z * z)
    m10 = s * (x * y + z * w)
    m20 = s * (x * z - y * w)
    m21 = s * (y * z + x * w)
    m22 = 1.0 - s * (x * x + y * y)
    m11 = 1.0 - s * (x * x + z * z)
    m12 = s * (y * z - x * w)
    cy = math.sqrt(m00 * m00 + m10 * m10)
    if cy > numpy.finfo(float).eps * 4.0:
        roll = math.atan2(m21, m22)
        pitch = math.atan2(-m20, cy)
        yaw = math.atan2(m10, m00)
    else:
        roll = math.atan2(-m12, m11)
        pitch = math.atan2(-m20, cy)
        yaw = 0.0
    return yaw, pitch, roll


recv_event = Event()
link = mavutil.mavlink.MAVLink('', 255, 1)
mavlink_pub = None
mavlink_sub = None
mavlink_recv = ''


def mavlink_message_handler(msg):
    global mavlink_recv
    if msg.msgid == 126:
        try:
            mav_bytes_msg = mavlink.convert_to_bytes(msg)
            mav_msg = link.decode(mav_bytes_msg)
            mavlink_recv += ''.join(chr(x) for x in mav_msg.data[:mav_msg.count])
            if 'nsh>' in mavlink_recv:
                # Remove the last line, including newline before prompt
                mavlink_recv = mavlink_recv[:mavlink_recv.find('nsh>') - 1]
                recv_event.set()
        except Exception:
            # exception in a callback would stop the executor
            traceback.print_exc()


def mavlink_exec(cmd, timeout=3.0):
    global mavlink_recv
    mavlink_recv = ''
    recv_event.clear()
    if not cmd.endswith('\n'):
        cmd += '\n'
    msg = mavutil.mavlink.MAVLink_serial_control_message(
        device=mavutil.mavlink.SERIAL_CONTROL_DEV_SHELL,
        flags=mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND | mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE |
              mavutil.mavlink.SERIAL_CONTROL_FLAG_MULTI,
        timeout=3,
        baudrate=0,
        count=len(cmd),
        data=[ord(c) for c in cmd.ljust(70, '\0')])
    msg.pack(link)
    ros_msg = mavlink.convert_to_rosmsg(msg)
    mavlink_pub.publish(ros_msg)
    recv_event.wait(timeout)
    return mavlink_recv


def read_diagnostics(name, key):
    e = Event()
    def cb(msg):
        for status in msg.status:
            if status.name.lower() == name.lower():
                for value in status.values:
                    if value.key.lower() == key.lower():
                        cb.value = value.value
                        e.set()
                        return

    cb.value = None
    with entities_lock:
        sub = node.create_subscription(DiagnosticArray, '/diagnostics', cb,
                                       QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
                                       callback_group=callback_group)
    e.wait(1.0)  # wait to read all the diagnostics from nodes publishing them
    with entities_lock:
        node.destroy_subscription(sub)
    return cb.value


BOARD_ROTATIONS = {
    0: 'no rotation',
    1: 'yaw 45°',
    2: 'yaw 90°',
    3: 'yaw 135°',
    4: 'yaw 180°',
    5: 'yaw 225°',
    6: 'yaw 270°',
    7: 'yaw 315°',
    8: 'roll 180°',
    9: 'roll 180°, yaw 45°',
    10: 'roll 180°, yaw 90°',
    11: 'roll 180°, yaw 135°',
    12: 'pitch 180°',
    13: 'roll 180°, yaw 225°',
    14: 'roll 180°, yaw 270°',
    15: 'roll 180°, yaw 315°',
    16: 'roll 90°',
    17: 'roll 90°, yaw 45°',
    18: 'roll 90°, yaw 90°',
    19: 'roll 90°, yaw 135°',
    20: 'roll 270°',
    21: 'roll 270°, yaw 45°',
    22: 'roll 270°, yaw 90°',
    23: 'roll 270°, yaw 135°',
    24: 'pitch 90°',
    25: 'pitch 270°',
    26: 'roll 270°, yaw 270°',
    27: 'roll 180°, pitch 270°',
    28: 'pitch 90°, yaw 180',
    29: 'pitch 90°, roll 90°',
    30: 'yaw 293°, pitch 68°, roll 90°',
    31: 'pitch 90°, roll 270°',
    32: 'pitch 9°, yaw 180°',
    33: 'pitch 45°',
    34: 'pitch 315°',
}


@check('FCU')
def check_fcu():
    try:
        state = wait_for_message('mavros/state', State, timeout=3, qos=LATCHED)
        if not state.connected:
            failure('no connection to the FCU (check wiring)')
            info('fcu_url = %s', get_node_param(MAVROS_NODE, 'fcu_url', '?'))
            return

        if not is_process_running('px4', exact=True): # can't use px4 console in SITL
            clover_tag = re.compile(r'-cl[oe]ver\.\d+$')
            clover_fw = False

            # Make sure the console is available to us
            mavlink_exec('\n')
            version_str = mavlink_exec('ver all')
            if version_str == '':
                info('no version data available from SITL')

            for line in version_str.split('\n'):
                if line.startswith('FW version: '):
                    info(line[len('FW version: '):])
                elif line.startswith('FW git tag: '): # only Clover's firmware
                    tag = line[len('FW git tag: '):]
                    clover_fw = clover_tag.search(tag)
                    info(tag)
                elif line.startswith('HW arch: '):
                    info(line[len('HW arch: '):])

            if not clover_fw:
                info('not Clover PX4 firmware, check https://clover.coex.tech/firmware')

        est = get_param('SYS_MC_EST_GROUP')
        if est == 1:
            info('selected estimator: LPE')
            fuse = int(get_param('LPE_FUSION'))
            if fuse & (1 << 4):
                info('LPE_FUSION: land detector fusion is enabled')
            else:
                info('LPE_FUSION: land detector fusion is disabled')
            if fuse & (1 << 7):
                info('LPE_FUSION: barometer fusion is enabled')
            else:
                info('LPE_FUSION: barometer fusion is disabled')

            mag_yaw_w = get_param('ATT_W_MAG')
            if mag_yaw_w == 0:
                info('magnetometer weight (ATT_W_MAG) is zero, better for indoor flights')
            else:
                info('magnetometer weight (ATT_W_MAG) is non-zero (%.2f), better for outdoor flights', mag_yaw_w)

        elif est == 2:
            info('selected estimator: EKF2')
        else:
            failure('unknown selected estimator: %s', est)

        rot = get_param('SENS_BOARD_ROT')
        if rot is not None:
            try:
                info('board rotation: %s', BOARD_ROTATIONS[rot])
            except KeyError:
                failure('unknown board rotation %s', rot)

        cbrk_usb_chk = get_param('CBRK_USB_CHK')
        if cbrk_usb_chk != 197848:
            failure('set parameter CBRK_USB_CHK to 197848 for flying with USB connected')

        if not is_process_running('px4', exact=True): # skip battery check in SITL
            try:
                battery = wait_for_message('mavros/battery', BatteryState, timeout=3)
                if not battery.cell_voltage:
                    failure('cell voltage is not available, https://clover.coex.tech/power')
                else:
                    cell = battery.cell_voltage[0]
                    # number of cells 1 means this is overall voltage
                    if len(battery.cell_voltage) == 1:
                        n_cells = get_param('BAT1_N_CELLS', strict=False)
                        if n_cells is None:
                            # older PX4
                            n_cells = get_param('BAT_N_CELLS', strict=True)
                        cell /= n_cells

                    if cell > 4.3 or cell < 3.0:
                        failure('incorrect cell voltage: %.2f V, https://clover.coex.tech/power', cell)
                    elif cell < 3.7:
                        failure('critically low cell voltage: %.2f V, recharge battery', cell)
            except ROSException:
                failure('no battery state')

        # time sync check
        try:
            info('time sync offset: %.2f s', float(read_diagnostics('mavros: Time Sync', 'Estimated time offset (s)')))
        except:
            failure('cannot read time sync offset')

    except ROSException:
        failure('no MAVROS state')
        fcu_url = get_node_param(MAVROS_NODE, 'fcu_url', '?')
        if fcu_url == '/dev/px4fmu':
            if not os.path.exists('/lib/udev/rules.d/99-px4fmu.rules'):
                info('udev rules are not installed, install udev rules or change usb_device to /dev/ttyACM0 in mavros.launch')
            else:
                info('udev did\'t recognize px4fmu device, check wiring or change usb_device to /dev/ttyACM0 in mavros.launch')
        info('fcu_url = %s', get_node_param(MAVROS_NODE, 'fcu_url', '?'))


def describe_direction(v):
    if v.x > 0.9:
        return 'forward'
    elif v.x < - 0.9:
        return 'backward'
    elif v.y > 0.9:
        return 'left'
    elif v.y < -0.9:
        return 'right'
    elif v.z > 0.9:
        return 'upward'
    elif v.z < -0.9:
        return 'downward'
    else:
        return None


def check_camera(name):
    try:
        img = wait_for_message(name + '/image_raw', Image, timeout=1)
    except ROSException:
        failure('%s: no images (is the camera connected properly?)', name)
        return
    try:
        camera_info = wait_for_message(name + '/camera_info', CameraInfo, timeout=1)
    except ROSException:
        failure('%s: no calibration info', name)
        return

    if img.width != camera_info.width:
        failure('%s: calibration width doesn\'t match image width (%d != %d)', name, camera_info.width, img.width)
    if img.height != camera_info.height:
        failure('%s: calibration height doesn\'t match image height (%d != %d))', name, camera_info.height, img.height)

    try:
        optical = Vector3Stamped()
        optical.header.frame_id = img.header.frame_id
        optical.vector.z = 1.0
        cable = Vector3Stamped()
        cable.header.frame_id = img.header.frame_id
        cable.vector.y = 1.0

        optical = describe_direction(tf_buffer.transform(optical, 'base_link').vector)
        cable = describe_direction(tf_buffer.transform(cable, 'base_link').vector)
        if not optical or not cable:
            info('%s: custom camera orientation detected', name)
        else:
            info('camera is oriented %s, cable from camera goes %s', optical, cable)

    except tf2_ros.TransformException:
        failure('cannot transform from base_link to camera frame')


@check('Main camera')
def check_main_camera():
    check_camera('main_camera')


def is_process_running(binary, exact=False, full=False):
    try:
        args = ['pgrep']
        if exact:
            args.append('-x')  # match exactly with the command name
        if full:
            args.append('-f')  # use full command line (including arguments) to match
        args.append(binary)
        subprocess.check_output(args)
        return True
    except subprocess.CalledProcessError:
        return False
    except OSError:
        return False  # no pgrep


def is_node_running(name):
    # ROS 2 nodes may run inside a component container, so look for the node in the graph, not for a process
    namespace = node.get_namespace()
    return any(n == name and ns == namespace for n, ns in node.get_node_names_and_namespaces())


@check('ArUco markers')
def check_aruco():
    markers = None

    if is_node_running('aruco_detect'):
        try:
            info('aruco_detect/length = %g m', get_node_param('aruco_detect', 'length'))
        except KeyError:
            failure('aruco_detect/length parameter is not set')
        known_vertical = get_node_param('aruco_detect', 'known_vertical', '')
        flip_vertical = get_node_param('aruco_detect', 'flip_vertical', False)
        description = ''
        if known_vertical == 'map' and not flip_vertical:
            description = ' (all markers are on the floor)'
        elif known_vertical == 'map' and flip_vertical:
            description = ' (all markers are on the ceiling)'
        info('aruco_detect/known_vertical = %s', known_vertical)
        info('aruco_detect/flip_vertical = %s%s', flip_vertical, description)
        try:
            markers = wait_for_message('aruco_detect/markers', MarkerArray, timeout=0.8)
        except ROSException:
            failure('no markers detection')
            return
    else:
        info('aruco_detect is not running')
        return

    if is_node_running('aruco_map'):
        known_vertical = get_node_param('aruco_map', 'known_vertical', '')
        flip_vertical = get_node_param('aruco_map', 'flip_vertical', False)
        description = ''
        if known_vertical == 'map' and not flip_vertical:
            description += ' (markers map is on the floor)'
        elif known_vertical == 'map' and flip_vertical:
            description += ' (markers map is on the ceiling)'
        info('aruco_map/known_vertical = %s', known_vertical)
        info('aruco_map/flip_vertical = %s%s', flip_vertical, description)

        try:
            visualization = wait_for_message('aruco_map/visualization', VisualizationMarkerArray, timeout=0.8, qos=LATCHED)
            info('map has %s markers', len(visualization.markers))
        except:
            failure('cannot read aruco_map/visualization topic')

        try:
            wait_for_message('aruco_map/pose', PoseWithCovarianceStamped, timeout=0.8)
        except ROSException:
            if not markers:
                info('no map detection as no markers detection')
            elif not markers.markers:
                info('no map detection as no markers detected')
            else:
                failure('no map detection')
    else:
        info('aruco_map is not running')


def is_on_the_floor():
    try:
        dist = wait_for_message('rangefinder/range', Range, timeout=1)
        return dist.range < 0.3
    except ROSException:
        return False


@check('Vision position estimate')
def check_vpe():
    vis = None
    try:
        vis = wait_for_message('mavros/vision_pose/pose', PoseStamped, timeout=0.8)
    except ROSException:
        try:
            vis = wait_for_message('mavros/mocap/pose', PoseStamped, timeout=0.8)
        except ROSException:
            if not is_node_running('vpe_publisher'):
                info('no vision position estimate, vpe_publisher is not running')
            elif get_node_param('aruco_map', 'known_vertical', '') == 'map' \
                   and get_node_param('aruco_map', 'flip_vertical', False):
                failure('no vision position estimate, markers are on the ceiling')
            elif is_on_the_floor():
                info('no vision position estimate, the drone is on the floor')
            else:
                failure('no vision position estimate')

    # check PX4 settings
    est = get_param('SYS_MC_EST_GROUP')
    if est == 1:
        ext_yaw = get_param('ATT_EXT_HDG_M')
        if ext_yaw != 1:
            failure('vision yaw is disabled, change ATT_EXT_HDG_M parameter')
        vision_yaw_w = get_param('ATT_W_EXT_HDG')
        if vision_yaw_w == 0:
            failure('vision yaw weight is zero, change ATT_W_EXT_HDG parameter')
        else:
            info('vision yaw weight: %s', ff(vision_yaw_w))
        fuse = int(get_param('LPE_FUSION'))
        if not fuse & (1 << 2):
            failure('vision position fusion is disabled, change LPE_FUSION parameter')
        delay = get_param('LPE_VIS_DELAY')
        if delay != 0:
            failure('LPE_VIS_DELAY = %s, but it should be zero', delay)
        info('LPE_VIS_XY = %s m, LPE_VIS_Z = %s m', get_paramf('LPE_VIS_XY'), get_paramf('LPE_VIS_Z'))
    elif est == 2:
        ev_ctrl = get_param('EKF2_EV_CTRL', strict=False)
        if ev_ctrl is not None: # PX4 after v1.14
            ev_ctrl = int(ev_ctrl)
            if not ev_ctrl & (1 << 0):
                failure('vision horizontal position fusion is disabled, change EKF2_EV_CTRL parameter')
            if not ev_ctrl & (1 << 1):
                failure('vision vertical position fusion is disabled, change EKF2_EV_CTRL parameter')
            if not ev_ctrl & (1 << 3):
                failure('vision yaw fusion is disabled, change EKF2_EV_CTRL parameter')
        else: # PX4 before v1.14
            fuse = int(get_param('EKF2_AID_MASK'))
            if not fuse & (1 << 3):
                failure('vision position fusion is disabled, change EKF2_AID_MASK parameter')
            if not fuse & (1 << 4):
                failure('vision yaw fusion is disabled, change EKF2_AID_MASK parameter')

        delay = get_param('EKF2_EV_DELAY')
        if delay != 0:
            failure('EKF2_EV_DELAY = %.2f, but it should be zero', delay)
        info('EKF2_EVA_NOISE = %s, EKF2_EVP_NOISE = %s',
            get_paramf('EKF2_EVA_NOISE', 3),
            get_paramf('EKF2_EVP_NOISE', 3))

    if not vis:
        return

    # check vision pose and estimated pose inconsistency
    try:
        pose = wait_for_message('mavros/local_position/pose', PoseStamped, timeout=1)
    except:
        return
    horiz = math.hypot(vis.pose.position.x - pose.pose.position.x, vis.pose.position.y - pose.pose.position.y)
    if horiz > 0.5:
        failure('horizontal position inconsistency: %.2f m', horiz)
    vert = vis.pose.position.z - pose.pose.position.z
    if abs(vert) > 0.5:
        failure('vertical position inconsistency: %.2f m', vert)
    op = pose.pose.orientation
    ov = vis.pose.orientation
    yawp, _, _ = euler_from_quaternion((op.x, op.y, op.z, op.w), axes='rzyx')
    yawv, _, _ = euler_from_quaternion((ov.x, ov.y, ov.z, ov.w), axes='rzyx')
    yawdiff = yawp - yawv
    yawdiff = math.degrees((yawdiff + 180) % 360 - 180)
    if abs(yawdiff) > 8:
        failure('yaw inconsistency: %.2f deg', yawdiff)


@check('Simple offboard node')
def check_simpleoffboard():
    try:
        wait_for_service('navigate', timeout=3)
        wait_for_service('get_telemetry', timeout=3)
        wait_for_service('land', timeout=3)
    except ROSException:
        failure('no simple_offboard services')


@check('IMU')
def check_imu():
    try:
        wait_for_message('mavros/imu/data', Imu, timeout=1)
    except ROSException:
        failure('no IMU data (check flight controller calibration)')


@check('Local position')
def check_local_position():
    try:
        pose = wait_for_message('mavros/local_position/pose', PoseStamped, timeout=1)
        o = pose.pose.orientation
        _, pitch, roll = euler_from_quaternion((o.x, o.y, o.z, o.w), axes='rzyx')
        MAX_ANGLE = math.radians(2)
        if abs(pitch) > MAX_ANGLE:
            failure('pitch is %.2f deg; place copter horizontally or redo level horizon calib',
                    math.degrees(pitch))
        if abs(roll) > MAX_ANGLE:
            failure('roll is %.2f deg; place copter horizontally or redo level horizon calib',
                    math.degrees(roll))

        if not tf_buffer.can_transform('base_link', pose.header.frame_id, node.get_clock().now(), Duration(seconds=0.5)):
            failure('can\'t transform from %s to base_link (timeout 0.5 s): is TF enabled?', pose.header.frame_id)

        if not tf_buffer.can_transform('body', pose.header.frame_id, node.get_clock().now(), Duration(seconds=0.5)):
            failure('can\'t transform from %s to body (timeout 0.5 s)', pose.header.frame_id)

    except ROSException:
        failure('no local position')


@check('Velocity estimation')
def check_velocity():
    try:
        velocity = wait_for_message('mavros/local_position/velocity_local', TwistStamped, timeout=1)
        horiz = math.hypot(velocity.twist.linear.x, velocity.twist.linear.y)
        vert = velocity.twist.linear.z
        if abs(horiz) > 0.1:
            failure('horizontal velocity estimation is %.2f m/s; is copter staying still?' % horiz)
        if abs(vert) > 0.1:
            failure('vertical velocity estimation is %.2f m/s; is copter staying still?' % vert)

        velocity = wait_for_message('mavros/local_position/velocity_body', TwistStamped, timeout=1)
        angular = velocity.twist.angular
        ANGULAR_VELOCITY_LIMIT = 0.1
        if abs(angular.x) > ANGULAR_VELOCITY_LIMIT:
            failure('pitch rate estimation is %.2f rad/s (%.2f deg/s); is copter staying still?',
                    angular.x, math.degrees(angular.x))
        if abs(angular.y) > ANGULAR_VELOCITY_LIMIT:
            failure('pitch rate estimation is %.2f rad/s (%.2f deg/s); is copter staying still?',
                    angular.y, math.degrees(angular.y))
        if abs(angular.z) > ANGULAR_VELOCITY_LIMIT:
            failure('pitch rate estimation is %.2f rad/s (%.2f deg/s); is copter staying still?',
                    angular.z, math.degrees(angular.z))
    except ROSException:
        failure('no velocity estimation')


@check('Global position (GPS)')
def check_global_position():
    try:
        wait_for_message('mavros/global_position/global', NavSatFix, timeout=0.8)
    except ROSException:
        info('no global position')
        if get_param('SYS_MC_EST_GROUP') == 2:
            gps_ctrl = get_param('EKF2_GPS_CTRL', strict=False)
            if gps_ctrl is not None: # PX4 after v1.14
                if int(gps_ctrl) & (1 << 0):
                    failure('GPS fusion enabled may suppress vision position aiding, change EKF2_GPS_CTRL')
            else: # PX4 before v1.14
                if int(get_param('EKF2_AID_MASK', 0)) & (1 << 0):
                    failure('GPS fusion enabled may suppress vision position aiding, change EKF2_AID_MASK')


@check('Optical flow')
def check_optical_flow():
    if not is_node_running('optical_flow'):
        info('optical_flow is not running')
        return

    # TODO:check FPS!
    try:
        wait_for_message('mavros/px4flow/raw/send', OpticalFlowRad, timeout=0.5)

        # check PX4 settings
        rot = get_param('SENS_FLOW_ROT')
        if rot != 0:
            failure('SENS_FLOW_ROT = %s, but it should be zero', rot)
        est = get_param('SYS_MC_EST_GROUP')
        if est == 1:
            fuse = int(get_param('LPE_FUSION'))
            if not fuse & (1 << 1):
                failure('optical flow fusion is disabled, change LPE_FUSION parameter')
            if not fuse & (1 << 1):
                failure('flow gyro compensation is disabled, change LPE_FUSION parameter')
            scale = get_param('LPE_FLW_SCALE', 1)
            if not numpy.isclose(scale, 1.0):
                failure('LPE_FLW_SCALE = %.2f, but it should be 1.0', scale)

            info('LPE_FLW_QMIN = %s, LPE_FLW_R = %s, LPE_FLW_RR = %s',
                          get_paramf('LPE_FLW_QMIN'),
                          get_paramf('LPE_FLW_R', 4),
                          get_paramf('LPE_FLW_RR', 4))
        elif est == 2:
            of_ctrl = get_param('EKF2_OF_CTRL', strict=False)
            if of_ctrl is not None: # PX4 after v1.14
                if of_ctrl == 0:
                    failure('optical flow fusion is disabled, change EKF2_OF_CTRL')
            else: # PX4 before v1.14
                fuse = int(get_param('EKF2_AID_MASK', 0))
                if not fuse & (1 << 1):
                    failure('optical flow fusion is disabled, change EKF2_AID_MASK parameter')
            delay = get_param('EKF2_OF_DELAY', 0)
            if delay != 0:
                failure('EKF2_OF_DELAY = %.2f, but it should be zero', delay)
            info('EKF2_OF_QMIN = %s, EKF2_OF_N_MIN = %s, EKF2_OF_N_MAX = %s',
                          get_paramf('EKF2_OF_QMIN'),
                          get_paramf('EKF2_OF_N_MIN', 4),
                          get_paramf('EKF2_OF_N_MAX', 4))
        info('SENS_FLOW_MINHGT = %s, SENS_FLOW_MAXHGT = %s', get_paramf('SENS_FLOW_MINHGT', 3), get_paramf('SENS_FLOW_MAXHGT', 3))

    except ROSException:
        if get_node_param('optical_flow', 'disable_on_vpe', False):
            try:
                wait_for_message('mavros/vision_pose/pose', PoseStamped, timeout=1)
                info('no optical flow as disable_on_vpe is true')
            except:
                failure('no optical flow on RPi, disable_on_vpe is true, but no vision pose also')
        else:
            failure('no optical flow on RPi')


@check('Rangefinder')
def check_rangefinder():
    # TODO: check FPS!
    rng = False
    try:
        wait_for_message('rangefinder/range', Range, timeout=4)
        rng = True
    except ROSException:
        failure('no rangefinder data from Raspberry')

    try:
        wait_for_message('mavros/distance_sensor/rangefinder', Range, timeout=4)
        rng = True
    except ROSException:
        failure('no rangefinder data from PX4')

    if not rng:
        return

    est = get_param('SYS_MC_EST_GROUP')
    if est == 1:
        fuse = int(get_param('LPE_FUSION', 0))
        if not fuse & (1 << 5):
            info('"pub agl as lpos down" in LPE_FUSION is disabled, NOT operating over flat surface')
        else:
            info('"pub agl as lpos down" in LPE_FUSION is enabled, operating over flat surface')

    elif est == 2:
        hgt = get_param('EKF2_HGT_REF', strict=False)
        if hgt is None: # PX4 before v1.14
            hgt = get_param('EKF2_HGT_MODE')
        if hgt != 2:
            info('EKF2_HGT_MODE != Range sensor, NOT operating over flat surface')
        else:
            info('EKF2_HGT_MODE = Range sensor, operating over flat surface')
        aid = get_param('EKF2_RNG_AID', strict=False)
        if aid is not None: # PX4 before v1.14
            if aid != 1:
                info('EKF2_RNG_AID != 1, range sensor aiding disabled')
            else:
                info('EKF2_RNG_AID = 1, range sensor aiding enabled')


@check('Boot duration')
def check_boot_duration():
    if not os.path.exists('/etc/clover_version'):
        info('skip check')
        return # Don't check not on Clover's image

    try:
        output = subprocess.check_output('systemd-analyze', stderr=subprocess.STDOUT).decode()
    except (OSError, subprocess.CalledProcessError) as e:
        info('could not call systemd-analyze: %s', str(e))
        return
    r = re.compile(r'([\d\.]+)s\s*$', flags=re.MULTILINE)
    match = r.search(output)
    if not match:
        info('could not parse systemd-analyze output')
        return
    duration = float(match.groups()[0])
    if duration > 20:
        failure('long Raspbian boot duration: %ss (systemd-analyze for analyzing)', duration)


@check('CPU usage')
def check_cpu_usage():
    # component_conta is the name of component_container process as top prints it
    WHITELIST = 'component_conta', 'gzclient', 'gzserver', 'selfcheck'
    CMD = "top -n 1 -b -i | tail -n +8 | awk '{ printf(\"%-8s\\t%-8s\\t%-8s\\n\", $1, $9, $12); }'"
    output = subprocess.check_output(CMD, shell=True).decode()
    processes = output.split('\n')
    for process in processes:
        if not process:
            continue
        pid, cpu, cmd = process.split('\t')

        if cmd.strip() not in WHITELIST and locale.atof(cpu) > 30:
            failure('high CPU usage (%s%%) detected: %s (PID %s)',
                    cpu.strip(), cmd.strip(), pid.strip())


@check('clover.service')
def check_clover_service():
    if not os.path.exists('/etc/clover_version'):
        return # Don't check not on Clover's image

    try:
        output = subprocess.check_output('systemctl show -p ActiveState --value clover.service'.split(),
                                         stderr=subprocess.STDOUT).decode()
    except subprocess.CalledProcessError as e:
        failure('systemctl returned %s: %s', e.returncode, e.output)
        return
    except OSError as e:
        failure('could not call systemctl: %s', str(e))
        return
    if 'inactive' in output:
        failure('service is not running, try sudo systemctl restart clover')
        return
    elif 'failed' in output:
        failure('service failed to run, check your launch-files')

    BLACKLIST = 'Unexpected command 520', 'Time jump detected', 'different index:'

    # ROS 2 console format: [node-1] [ERROR] [1234.5678] [node_name]: message
    r = re.compile(r'^(.*)\[(FATAL|ERROR|WARN)\] \[\d+.\d+\] \[.*?\]: (.*?)(\x1b(.*))?$')
    error_count = OrderedDict()
    try:
        for line in open('/tmp/clover.err', 'r'):
            skip = False
            for substr in BLACKLIST:
                if substr in line:
                    skip = True
            if skip:
                continue

            node_error = r.search(line)
            if node_error:
                msg = node_error.groups()[1].strip() + ': ' + node_error.groups()[2]
                if msg in error_count:
                    error_count[msg] += 1
                else:
                    error_count.update({msg: 1})
            else:
                error_count.update({line.strip(): 1})

        for error in error_count:
            if error_count[error] == 1:
                failure(error)
            else:
                failure('%s (%d)', error, error_count[error])

    except IOError as e:
        failure('%s', e)


@check('Image')
def check_image():
    try:
        info('version: %s', open('/etc/clover_version').read().strip())
    except IOError:
        try:
            info('VM version: %s', open('/etc/clover_vm_version').read().strip())
        except IOError:
            info('no /etc/clover_version file, not the Clover image?')


@check('Preflight status')
def check_preflight_status():
    if is_process_running('px4', exact=True):
        info('can\'t check in SITL')
        return

    # Make sure the console is available to us
    mavlink_exec('\n')
    cmdr_output = mavlink_exec('commander check')
    if cmdr_output == '':
        failure('no data from FCU')
        return
    cmdr_lines = cmdr_output.split('\n')
    r = re.compile(r'^(.*)(Preflight|Prearm) check: (.*)')
    for line in cmdr_lines:
        if 'WARN' in line:
            failure(line[line.find(']') + 2:])
            continue
        match = r.search(line)
        if match is not None:
            check_status = match.groups()[2]
            if check_status != 'OK':
                failure(' '.join([match.groups()[1], 'check:', check_status]))


@check('Network')
def check_network():
    if not os.path.exists('/etc/clover_version'):
        # TODO:
        return # Don't check not on Clover's image

    # there is no ROS_HOSTNAME in ROS 2; show the settings the nodes discovery depends on
    for name in 'ROS_DOMAIN_ID', 'ROS_AUTOMATIC_DISCOVERY_RANGE', 'ROS_LOCALHOST_ONLY':
        value = os.environ.get(name, '').strip()
        if value:
            info('%s = %s', name, value)


@check('RPi health')
def check_rpi_health():
    try:
        import shutil
        total, used, free = shutil.disk_usage('/')
        if free < 1024 * 1024 * 1024:
            failure('disk space is less than 1 GB; consider removing logs (~/.ros/log/)')
    except Exception as e:
        info('could not check the disk free space: %s', str(e))

    # `vcgencmd get_throttled` output codes taken from
    # https://github.com/raspberrypi/documentation/blob/JamesH65-patch-vcgencmd-vcdbg-docs/raspbian/applications/vcgencmd.md#get_throttled
    # TODO: support more base platforms?
    FLAG_UNDERVOLTAGE_NOW = 0x1
    FLAG_FREQ_CAP_NOW = 0x2
    FLAG_THROTTLING_NOW = 0x4
    FLAG_THERMAL_LIMIT_NOW = 0x8
    FLAG_UNDERVOLTAGE_OCCURRED = 0x10000
    FLAG_FREQ_CAP_OCCURRED = 0x20000
    FLAG_THROTTLING_OCCURRED = 0x40000
    FLAG_THERMAL_LIMIT_OCCURRED = 0x80000

    FLAG_DESCRIPTIONS = (
        (FLAG_THROTTLING_NOW, 'system throttled to prevent damage'),
        (FLAG_THROTTLING_OCCURRED, 'your system is susceptible to throttling'),
        (FLAG_UNDERVOLTAGE_NOW, 'not enough power for onboard computer, flight inadvisable'),
        (FLAG_UNDERVOLTAGE_OCCURRED, 'power supply cannot provide enough power'),
        (FLAG_FREQ_CAP_NOW, 'CPU reached thermal limit and is throttled now'),
        (FLAG_FREQ_CAP_OCCURRED, 'CPU may overheat during drone operation, consider additional cooling'),
        (FLAG_THERMAL_LIMIT_NOW, 'CPU reached soft thermal limit, frequency reduced'),
        (FLAG_THERMAL_LIMIT_OCCURRED, 'CPU may reach soft thermal limit, consider additional cooling'),
    )

    try:
        # vcgencmd outputs a single string in a form of
        # <parameter>=<value>
        # In case of `get_throttled`, <value> is a hexadecimal number
        # with some of the FLAGs OR'ed together
        output = subprocess.check_output(['vcgencmd', 'get_throttled']).decode()
    except OSError:
        info('could not call vcgencmd binary; not a Raspberry Pi?')
        return
    except subprocess.CalledProcessError as e:
        info('vcgencmd returned %s', e.returncode)
        return

    try:
        throttle_mask = int(output.split('=')[1], base=16)
    except (IndexError, ValueError):
        info('could not parse vcgencmd output: %s', output.strip())
        return
    for flag_description in FLAG_DESCRIPTIONS:
        if throttle_mask & flag_description[0]:
            failure(flag_description[1])


@check('Board')
def check_board():
    try:
        info('%s', open('/proc/device-tree/model').readline())
    except IOError:
        info('could not open /proc/device-tree/model, not a Raspberry Pi?')


def parallel_for(fns):
    threads = []
    for fn in fns:
        thread = Thread(target=fn, daemon=True)
        thread.start()
        threads.append(thread)
    for thread in threads:
        thread.join()


def consequentially_for(fns):
    for fn in fns:
        fn()


def selfcheck():
    checks = [
        check_image,
        check_board,
        check_clover_service,
        check_network,
        check_fcu,
        check_imu,
        check_local_position,
        check_velocity,
        check_global_position,
        check_preflight_status,
        check_main_camera,
        check_aruco,
        check_simpleoffboard,
        check_optical_flow,
        check_vpe,
        check_rangefinder,
        check_rpi_health,
        check_cpu_usage,
        check_boot_duration,
    ]
    if node.get_parameter('parallel').value:
        parallel_for(checks)
    else:
        consequentially_for(checks)


def spin(executor):
    try:
        executor.spin()
    except ExternalShutdownException:
        pass


def main():
    global node, tf_buffer, tf_listener, mavlink_pub, mavlink_sub

    os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{message}'

    # use user's locale to convert numbers, etc
    locale.setlocale(locale.LC_ALL, '')

    rclpy.init()
    node = Node('selfcheck')
    node.declare_parameter('parallel', False)
    node.declare_parameter('time', False)
    # mavros 2 has no mavlink/to and mavlink/from topics, the messages are passed through the UAS topics
    mavlink_topic = node.declare_parameter('mavlink_topic', '/uas1/mavlink_sink').value
    mavlink_from_topic = node.declare_parameter('mavlink_from_topic', '/uas1/mavlink_source').value

    tf_buffer = tf2_ros.Buffer(node=node)
    tf_listener = tf2_ros.TransformListener(tf_buffer, node)

    mavlink_pub = node.create_publisher(Mavlink, mavlink_topic, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
    # default callback group: the handler is never called concurrently, the output chunks keep their order
    mavlink_sub = node.create_subscription(Mavlink, mavlink_from_topic, mavlink_message_handler,
                                           QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT))

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = Thread(target=spin, args=(executor,), daemon=True)
    spin_thread.start()

    # FIXME: not sleeping here still breaks things
    time.sleep(0.5)

    try:
        node.get_logger().info('Performing selfcheck...')
        selfcheck()
    finally:
        executor.shutdown()
        spin_thread.join(1.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
