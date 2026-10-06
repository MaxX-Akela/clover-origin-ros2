import os

from ament_index_python.packages import PackageNotFoundError, get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# the same list as plugin_whitelist in ROS 1 (vision_pose_estimate is called vision_pose now)
PLUGINS = [
    'altitude',
    'command',
    'distance_sensor',
    'ftp',
    'global_position',
    'imu',
    'local_position',
    'manual_control',
    # 'mocap_pose_estimate',
    'param',
    'px4flow',
    'rc_io',
    'setpoint_attitude',
    'setpoint_position',
    'setpoint_raw',
    'setpoint_velocity',
    'sys_status',
    'sys_time',
    'vision_pose',
    # 'vision_speed',
    # 'waypoint',
]


def has_package(name):
    try:
        get_package_prefix(name)
        return True
    except PackageNotFoundError:
        return False


def is_true(value):
    return value.lower() in ('true', '1')


def launch_setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    fcu_conn = arg('fcu_conn')  # options: usb, uart, udp, sitl, hitl, none
    fcu_ip = arg('fcu_ip')
    gcs_bridge = arg('gcs_bridge')
    gcs_host = arg('gcs_host')
    usb_device = arg('usb_device')
    distance_sensor_remap = arg('distance_sensor_remap')

    actions = []

    if is_true(arg('viz')):
        # mavros_extras for ROS 2 has no visualization executable
        actions.append(LogInfo(msg='mavros_extras visualization node is not available in ROS 2, skipping'))

    if fcu_conn == 'none':
        return actions

    params = {
        'tgt_system': int(arg('fcu_sys_id')),
        # allowlist only overrides denylist in mavros 2, so deny everything first
        'plugin_denylist': ['*'],
        'plugin_allowlist': PLUGINS,
    }

    prefix = None
    if fcu_conn in ('', 'uart'):
        params['fcu_url'] = '/dev/ttyAMA0:921600'  # UART connection
    elif fcu_conn == 'usb':
        params['fcu_url'] = usb_device  # USB connection
        prefix = os.path.join(get_package_prefix('clover'), 'lib', 'clover', 'waitfile') + ' ' + usb_device
    elif fcu_conn == 'udp':
        params['fcu_url'] = 'udp://@%s:14557' % fcu_ip  # sitl before PX4 1.9.0
    elif fcu_conn == 'sitl':
        params['fcu_url'] = 'udp://@%s:14580' % fcu_ip  # sitl since PX4 1.9.0
    elif fcu_conn == 'hitl':
        params['fcu_url'] = 'udp://%s:14540@' % fcu_ip  # hitl connection (to gazebo_mavlink_interface plugin)
    else:
        actions.append(LogInfo(msg='Unknown fcu_conn value: %s, using default fcu_url of mavros' % fcu_conn))

    # gcs bridge
    if gcs_bridge == 'tcp':
        params['gcs_url'] = 'tcp-l://0.0.0.0:5760'
    elif gcs_bridge == 'udp':
        params['gcs_url'] = 'udp://0.0.0.0:14550@14550'
    elif gcs_bridge == 'udp-b':
        params['gcs_url'] = 'udp-b://%s:14550@14550' % gcs_host
    elif gcs_bridge == 'udp-pb':
        params['gcs_url'] = 'udp-pb://%s:14550@14550' % gcs_host
    elif gcs_bridge in ('', 'false', 'False'):
        params['gcs_url'] = ''

    config = os.path.join(get_package_share_directory('clover'), 'config', 'mavros.yaml')

    # The node must have neither a name nor a namespace here: mavros_node process creates
    # mavros_node, mavros_router and mavros nodes itself, topics are named mavros/... after the latter
    actions.append(Node(
        package='mavros',
        executable='mavros_node',
        prefix=prefix,
        parameters=[
            config,  # basic params
            params,
        ],
        respawn=is_true(arg('respawn')),
        respawn_delay=1.0,
        output='screen',
    ))

    # Plugins of mavros 2.15 don't get the parameters and the remappings passed to mavros_node,
    # so set the parameters of the plugins with their parameter services
    actions.append(Node(
        package='clover',
        executable='mavros_params',
        name='mavros_params',
        parameters=[{'config': config}],
        output='screen',
    ))

    # remap rangefinder
    if distance_sensor_remap:
        if has_package('topic_tools'):
            actions.append(Node(
                package='topic_tools',
                executable='relay',
                name='rangefinder_relay',
                arguments=[distance_sensor_remap, 'mavros/distance_sensor/rangefinder_sub'],
            ))
        else:
            actions.append(LogInfo(msg='WARNING: topic_tools package is not installed, %s topic is not passed to mavros'
                                       % distance_sensor_remap))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('fcu_conn', default_value='usb'),  # options: usb, uart, udp, sitl, hitl, none
        DeclareLaunchArgument('fcu_ip', default_value='127.0.0.1'),
        DeclareLaunchArgument('fcu_sys_id', default_value='1'),
        DeclareLaunchArgument('gcs_bridge', default_value='tcp'),  # options: tcp, udp, udp-b, udp-pb
        DeclareLaunchArgument('gcs_host', default_value=''),  # host to bind for udp-b and udp-pb
        DeclareLaunchArgument('viz', default_value='true'),
        DeclareLaunchArgument('respawn', default_value='true'),
        DeclareLaunchArgument('distance_sensor_remap', default_value='rangefinder/range'),
        DeclareLaunchArgument('usb_device', default_value='/dev/px4fmu'),
        OpaqueFunction(function=launch_setup),
    ])
