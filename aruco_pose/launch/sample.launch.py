import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    share = get_package_share_directory('aruco_pose')

    # The camera node is not started here: run any camera driver that publishes
    # main_camera/image_raw and main_camera/camera_info (or change the arguments below).
    image_raw = LaunchConfiguration('image_raw')
    camera_info = LaunchConfiguration('camera_info')

    aruco_detect = ComposableNode(
        package='aruco_pose',
        plugin='aruco_pose::ArucoDetect',
        name='aruco_detect',
        parameters=[{'length': LaunchConfiguration('length')}],
        remappings=[
            ('image_raw', image_raw),
            ('camera_info', camera_info),
        ],
    )

    aruco_map = ComposableNode(
        package='aruco_pose',
        plugin='aruco_pose::ArucoMap',
        name='aruco_map',
        parameters=[{'map': LaunchConfiguration('map')}],
        remappings=[
            ('image_raw', image_raw),
            ('camera_info', camera_info),
            ('markers', 'aruco_detect/markers'),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('image_raw', default_value='main_camera/image_raw'),
        DeclareLaunchArgument('camera_info', default_value='main_camera/camera_info'),
        DeclareLaunchArgument('length', default_value='0.33'),
        DeclareLaunchArgument('map', default_value=os.path.join(share, 'map', 'map.txt')),
        ComposableNodeContainer(
            name='aruco_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[aruco_detect, aruco_map],
            output='screen',
        ),
    ])
