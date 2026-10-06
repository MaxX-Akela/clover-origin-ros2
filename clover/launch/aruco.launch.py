import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode

# For additional help go to https://clover.coex.tech/aruco


def is_true(value):
    return value.lower() in ('true', '1')


def launch_setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    aruco_detect = is_true(arg('aruco_detect'))
    aruco_map = is_true(arg('aruco_map'))
    aruco_vpe = is_true(arg('aruco_vpe'))
    placement = arg('placement')
    force_init = is_true(arg('force_init'))
    disable = is_true(arg('disable'))

    vertical = {}
    if placement in ('floor', 'ceiling'):
        vertical['known_vertical'] = 'map'
    if placement == 'ceiling':
        vertical['flip_vertical'] = True

    actions = []
    components = []

    # aruco_detect: detect aruco markers, estimate poses
    if aruco_detect and not disable:
        components.append(ComposableNode(
            package='aruco_pose',
            plugin='aruco_pose::ArucoDetect',
            name='aruco_detect',
            remappings=[
                ('image_raw', 'main_camera/image_raw'),
                ('camera_info', 'main_camera/camera_info'),
                ('map_markers', 'aruco_map/map'),
            ],
            parameters=[{
                'dictionary': 2,  # DICT_4X4_250
                'estimate_poses': True,
                'send_tf': True,
                'use_map_markers': aruco_map,
                'length': float(arg('length')),
                'transform_timeout': 0.1,
                # aruco detector parameters
                'cornerRefinementMethod': 2,  # contour refinement
                'minMarkerPerimeterRate': 0.075,  # 0.075 for 320x240, 0.0375 for 640x480
                # length override example:
                # 'length_override.3': 0.1,
            }, vertical],
        ))

    # aruco_map: estimate aruco map pose
    if aruco_map and not disable:
        components.append(ComposableNode(
            package='aruco_pose',
            plugin='aruco_pose::ArucoMap',
            name='aruco_map',
            remappings=[
                ('image_raw', 'main_camera/image_raw'),
                ('camera_info', 'main_camera/camera_info'),
                ('markers', 'aruco_detect/markers'),
            ],
            parameters=[{
                'map': os.path.join(get_package_share_directory('aruco_pose'), 'map', arg('map')),
                'image_axis': True,
                'frame_id': 'aruco_map_detected' if aruco_vpe else 'aruco_map',
                'markers.frame_id': 'aruco_map',
                'markers.child_frame_id_prefix': 'aruco_',
            }, vertical],
        ))

    if components:
        actions.append(LoadComposableNodes(
            target_container=arg('container'),
            composable_node_descriptions=components,
        ))

    # vpe publisher from aruco markers
    if aruco_vpe or force_init:
        remappings = [('~/vpe', 'mavros/vision_pose/pose')]
        params = {
            'force_init': force_init,
            'offset_frame_id': 'aruco_map',
        }
        if aruco_vpe:
            remappings.append(('~/pose_cov', 'aruco_map/pose'))
            params['frame_id'] = 'aruco_map_detected'
        actions.append(Node(
            package='clover',
            executable='vpe_publisher',
            name='vpe_publisher',
            remappings=remappings,
            parameters=[params],
            output='screen',
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('aruco_detect', default_value='true'),
        DeclareLaunchArgument('aruco_map', default_value='false'),
        DeclareLaunchArgument('aruco_vpe', default_value='false'),
        DeclareLaunchArgument('placement', default_value='floor'),  # markers placement: floor, ceiling, unknown
        DeclareLaunchArgument('length', default_value='0.22'),  # not-in-map markers length, m
        DeclareLaunchArgument('map', default_value='map.txt'),  # markers map file name
        DeclareLaunchArgument('force_init', default_value='false'),
        DeclareLaunchArgument('disable', default_value='false'),  # only force init
        DeclareLaunchArgument('container', default_value='main_camera_container'),  # components container
        OpaqueFunction(function=launch_setup),
    ])
