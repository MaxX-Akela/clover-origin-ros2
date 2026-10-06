import os

from ament_index_python.packages import PackageNotFoundError, get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            SetEnvironmentVariable)
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode

CONTAINER = 'main_camera_container'


def is_true(value):
    return value.lower() in ('true', '1')


def has_package(name):
    try:
        get_package_prefix(name)
        return True
    except PackageNotFoundError:
        return False


def find_executable(package, names):
    lib = os.path.join(get_package_prefix(package), 'lib', package)
    for name in names:
        if os.path.exists(os.path.join(lib, name)):
            return name
    return None


def launch_setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    def include(name, arguments):
        path = os.path.join(get_package_share_directory('clover'), 'launch', name)
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(path), launch_arguments=arguments.items())

    def missing(package, what):
        return LogInfo(msg='WARNING: %s package is not installed, %s' % (package, what))

    fcu_sys_id = arg('fcu_sys_id')
    web_video_server = is_true(arg('web_video_server'))
    rosbridge = is_true(arg('rosbridge'))
    main_camera = is_true(arg('main_camera'))
    optical_flow = is_true(arg('optical_flow'))
    aruco = is_true(arg('aruco'))
    rangefinder_vl53l1x = is_true(arg('rangefinder_vl53l1x'))
    led = is_true(arg('led'))
    rc = is_true(arg('rc'))
    force_init = is_true(arg('force_init'))  # force estimator to init by publishing zero pose
    simulator = is_true(arg('simulator'))  # flag that we are operating on a simulated drone

    actions = []

    # mavros
    actions.append(include('mavros.launch.py', {
        'fcu_conn': arg('fcu_conn'),
        'fcu_ip': arg('fcu_ip'),
        'fcu_sys_id': fcu_sys_id,
        'gcs_bridge': arg('gcs_bridge'),
        'gcs_host': arg('gcs_host'),
    }))

    # web video server
    if web_video_server:
        if has_package('web_video_server'):
            actions.append(Node(
                package='web_video_server',
                executable='web_video_server',
                name='web_video_server',
                parameters=[{
                    'default_stream_type': 'ros_compressed',
                    'publish_rate': 1.0,
                }],
                respawn=True,
                respawn_delay=5.0,
            ))
        else:
            actions.append(missing('web_video_server', 'web video server is not started'))

    # aruco markers
    if aruco or force_init:
        actions.append(include('aruco.launch.py', {
            'force_init': str(force_init),
            'disable': str(not aruco),
        }))

    # optical flow
    if optical_flow:
        actions.append(LoadComposableNodes(
            target_container=CONTAINER,
            composable_node_descriptions=[ComposableNode(
                package='clover',
                plugin='clover::OpticalFlow',
                name='optical_flow',
                remappings=[
                    ('image_raw', 'main_camera/image_raw'),
                    ('camera_info', 'main_camera/camera_info'),
                ],
                parameters=[{
                    'calc_flow_gyro': True,
                    'roi_rad': 0.8,
                    'disable_on_vpe': True,
                }],
            )],
        ))

    # simplified offboard control
    actions.append(Node(
        package='clover',
        executable='simple_offboard',
        name='simple_offboard',
        parameters=[{
            'reference_frames.main_camera_optical': 'map',
            'terrain_frame_mode': 'range',
        }],
        output='screen',
    ))

    # main camera
    if main_camera:
        actions.append(include('main_camera.launch.py', {'simulator': str(simulator)}))
    elif optical_flow or aruco:
        # no camera driver, but the components still need a container (the image may be published by another node)
        actions.append(ComposableNodeContainer(
            name=CONTAINER,
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            parameters=[{'thread_num': 2}],
            output='screen',
        ))

    # rosbridge
    if rosbridge or rc:
        if has_package('rosbridge_server'):
            actions.append(IncludeLaunchDescription(AnyLaunchDescriptionSource(os.path.join(
                get_package_share_directory('rosbridge_server'), 'launch', 'rosbridge_websocket_launch.xml'))))
        else:
            actions.append(missing('rosbridge_server', 'rosbridge is not started'))

    # tf2 republisher for web visualization
    if rosbridge:
        if has_package('tf2_web_republisher'):
            executable = find_executable('tf2_web_republisher', ['tf2_web_republisher', 'tf2_web_republisher_node'])
            if executable is not None:
                actions.append(Node(
                    package='tf2_web_republisher',
                    executable=executable,
                    name='tf2_web_republisher',
                    output='screen',
                ))
            else:
                actions.append(LogInfo(msg='WARNING: tf2_web_republisher executable is not found'))
        else:
            actions.append(missing('tf2_web_republisher', 'tf2 republisher for web visualization is not started'))

    # vl53l1x ToF rangefinder
    if rangefinder_vl53l1x and not simulator:
        if has_package('vl53l1x'):
            actions.append(Node(
                package='vl53l1x',
                executable='vl53l1x_node',
                name='rangefinder',
                parameters=[{
                    'frame_id': 'rangefinder',
                    'min_signal': 0.4,
                    'pass_statuses': [0, 6, 7, 11],
                }],
                output='screen',
            ))
        else:
            actions.append(missing('vl53l1x', 'rangefinder driver is not started, no data in rangefinder/range topic'))

    # rangefinder's frame
    if rangefinder_vl53l1x:
        actions.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='rangefinder_frame',
            arguments=['--x', '0', '--y', '0', '--z', '-0.05', '--yaw', '0', '--pitch', '1.5707963268', '--roll', '0',
                       '--frame-id', 'base_link', '--child-frame-id', 'rangefinder'],
        ))

    # led strip
    if led:
        actions.append(include('led.launch.py', {'simulator': str(simulator)}))

    # rc backend
    if rc:
        actions.append(Node(
            package='clover',
            executable='rc',
            name='rc',
            parameters=[{
                # Send fake GCS heartbeats. Set to "true" for upstream PX4
                'use_fake_gcs': False,
                # mavros gets and sends MAVLink messages in topics named after the target system
                'mavlink_topic': '/uas%s/mavlink_sink' % fcu_sys_id,
            }],
            output='screen',
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('fcu_conn', default_value='usb'),
        DeclareLaunchArgument('fcu_ip', default_value='127.0.0.1'),
        DeclareLaunchArgument('fcu_sys_id', default_value='1'),
        DeclareLaunchArgument('gcs_bridge', default_value='tcp'),
        DeclareLaunchArgument('gcs_host', default_value=''),  # host to bind for udp-b and udp-pb gcs_bridge
        DeclareLaunchArgument('web_video_server', default_value='true'),
        DeclareLaunchArgument('rosbridge', default_value='true'),
        DeclareLaunchArgument('main_camera', default_value='true'),
        DeclareLaunchArgument('optical_flow', default_value='true'),
        DeclareLaunchArgument('aruco', default_value='false'),
        DeclareLaunchArgument('rangefinder_vl53l1x', default_value='true'),
        DeclareLaunchArgument('led', default_value='true'),
        DeclareLaunchArgument('rc', default_value='false'),
        DeclareLaunchArgument('force_init', default_value='true'),  # force estimator to init by publishing zero pose
        DeclareLaunchArgument('simulator', default_value='false'),  # flag that we are operating on a simulated drone

        # log formatting
        SetEnvironmentVariable('RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{severity}] [{time}]: {name}: {message}'),

        OpaqueFunction(function=launch_setup),
    ])
