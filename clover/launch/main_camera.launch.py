import os
import tempfile

import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode

# article about camera setup: https://clover.coex.tech/camera_setup

CONTAINER = 'main_camera_container'

# camera resolution
IMAGE_WIDTH = 320
IMAGE_HEIGHT = 240

# Camera position and orientation are represented by base_link -> main_camera_optical transform
# (direction_z, direction_y): x y z yaw pitch roll
# direction_y is not used when the camera points forward or backward
CAMERA_FRAMES = {
    ('down', 'backward'): ('0.05', '0', '-0.07', '-1.5707963', '0', '3.1415926'),
    ('down', 'forward'): ('0.05', '0', '-0.07', '1.5707963', '0', '3.1415926'),
    ('up', 'backward'): ('0.05', '0', '0.07', '1.5707963', '0', '0'),
    ('up', 'forward'): ('0.05', '0', '0.07', '-1.5707963', '0', '0'),
    ('forward', None): ('0.03', '0', '0.05', '-1.5707963', '0', '-1.5707963'),
    ('backward', None): ('-0.03', '0', '0.05', '1.5707963', '0', '-1.5707963'),
}

# Template for custom camera orientation:
# CAMERA_FRAMES[('down', 'backward')] = ('0.05', '0', '-0.07', '-1.5707963', '0', '3.1415926')


def is_true(value):
    return value.lower() in ('true', '1')


def has_package(name):
    try:
        get_package_prefix(name)
        return True
    except PackageNotFoundError:
        return False


def rescale_camera_info(path, width, height):
    # v4l2_camera can't rescale camera calibration info (rescale_camera_info parameter of cv_camera),
    # so make a rescaled copy of the calibration file
    with open(path) as f:
        info = yaml.safe_load(f)

    if info['image_width'] == width and info['image_height'] == height:
        return path

    width_coeff = width / info['image_width']
    height_coeff = height / info['image_height']
    info['image_width'] = width
    info['image_height'] = height
    k = info['camera_matrix']['data']
    k[0] *= width_coeff
    k[2] *= width_coeff
    k[4] *= height_coeff
    k[5] *= height_coeff
    p = info['projection_matrix']['data']
    p[0] *= width_coeff
    p[2] *= width_coeff
    p[5] *= height_coeff
    p[6] *= height_coeff

    fd, rescaled = tempfile.mkstemp(prefix='clover_camera_info_', suffix='.yaml')
    with os.fdopen(fd, 'w') as f:
        yaml.safe_dump(info, f)
    return rescaled


def launch_setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    direction_z = arg('direction_z')
    direction_y = arg('direction_y')
    device = arg('device')

    actions = []

    frame = CAMERA_FRAMES.get((direction_z, direction_y)) or CAMERA_FRAMES.get((direction_z, None))
    if frame is not None:
        x, y, z, yaw, pitch, roll = frame
        actions.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='main_camera_frame',
            arguments=['--x', x, '--y', y, '--z', z, '--yaw', yaw, '--pitch', pitch, '--roll', roll,
                       '--frame-id', 'base_link', '--child-frame-id', 'main_camera_optical'],
        ))

    # camera components container
    actions.append(ComposableNodeContainer(
        name=CONTAINER,
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        parameters=[{'thread_num': 2}],
        output='screen',
    ))

    # camera node
    if not is_true(arg('simulator')):
        if has_package('v4l2_camera'):
            camera_info = os.path.join(get_package_share_directory('clover'), 'camera_info', 'fisheye_cam.yaml')
            if is_true(arg('rescale_camera_info')):
                # automatically rescale camera calibration info
                camera_info = rescale_camera_info(camera_info, IMAGE_WIDTH, IMAGE_HEIGHT)

            camera = LoadComposableNodes(
                target_container=CONTAINER,
                composable_node_descriptions=[ComposableNode(
                    package='v4l2_camera',
                    plugin='v4l2_camera::V4L2Camera',
                    name='main_camera',
                    namespace='main_camera',
                    parameters=[{
                        'video_device': device,
                        'camera_frame_id': 'main_camera_optical',
                        'camera_info_url': 'file://' + camera_info,
                        'time_per_frame': [1, 40],  # camera FPS
                        'image_size': [IMAGE_WIDTH, IMAGE_HEIGHT],
                    }],
                )],
            )
            # load the camera when the device appears
            waitfile = ExecuteProcess(
                cmd=[os.path.join(get_package_prefix('clover'), 'lib', 'clover', 'waitfile'), device, 'true'],
                name='main_camera_waitfile',
                output='screen',
            )
            actions.append(waitfile)
            actions.append(RegisterEventHandler(OnProcessExit(
                target_action=waitfile,
                on_exit=lambda event, context: [camera] if event.returncode == 0 else [],
            )))
        else:
            actions.append(LogInfo(msg='WARNING: v4l2_camera package is not installed, main camera is not started'))

    # camera visualization markers
    actions.append(Node(
        package='clover',
        executable='camera_markers',
        namespace='main_camera',
        name='main_camera_markers',
        parameters=[{'scale': 3.0}],
    ))

    # image topic throttled
    if is_true(arg('throttled_topic')):
        if has_package('topic_tools'):
            actions.append(Node(
                package='topic_tools',
                executable='throttle',
                name='main_camera_throttle',
                namespace='main_camera',
                arguments=['messages', 'image_raw', arg('throttled_topic_rate'), 'image_raw_throttled'],
            ))
        else:
            actions.append(LogInfo(msg='WARNING: topic_tools package is not installed, throttled image topic is disabled'))

    # rectified image topic
    if is_true(arg('rectify')):
        if has_package('image_proc'):
            actions.append(LoadComposableNodes(
                target_container=CONTAINER,
                composable_node_descriptions=[ComposableNode(
                    package='image_proc',
                    plugin='image_proc::RectifyNode',
                    name='rectify',
                    remappings=[
                        ('image', 'main_camera/image_raw'),
                        ('camera_info', 'main_camera/camera_info'),
                        ('image_rect', 'main_camera/image_rect'),
                    ],
                )],
            ))
        else:
            actions.append(LogInfo(msg='WARNING: image_proc package is not installed, rectification is disabled'))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('direction_z', default_value='down'),  # direction the camera points: down, up
        DeclareLaunchArgument('direction_y', default_value='backward'),  # direction the camera cable points: backward, forward
        DeclareLaunchArgument('device', default_value='/dev/video0'),  # v4l2 device
        DeclareLaunchArgument('throttled_topic', default_value='true'),  # enable throttled image topic
        DeclareLaunchArgument('throttled_topic_rate', default_value='5.0'),  # throttled image topic rate
        DeclareLaunchArgument('rectify', default_value='false'),  # enable rectification
        DeclareLaunchArgument('rescale_camera_info', default_value='true'),  # rescale camera calibration info
        DeclareLaunchArgument('simulator', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
