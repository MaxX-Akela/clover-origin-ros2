import os

from ament_index_python.packages import PackageNotFoundError, get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# For additional help go to https://clover.coex.tech/led


def is_true(value):
    return value.lower() in ('true', '1')


def has_package(name):
    try:
        get_package_prefix(name)
        return True
    except PackageNotFoundError:
        return False


def launch_setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    actions = []

    # ws281x led strip driver
    if is_true(arg('ws281x')) and not is_true(arg('simulator')):
        if has_package('ws281x'):
            actions.append(Node(
                package='ws281x',
                executable='ws281x_node',
                name='led',
                parameters=[{
                    'led_count': int(arg('led_count')),
                    'gpio_pin': int(arg('gpio_pin')),
                    'brightness': 64,
                    'strip_type': 'WS2811_STRIP_GRB',
                    'target_frequency': 800000,
                    'dma': 10,
                    'invert': False,
                }],
                output='screen',
            ))
        else:
            actions.append(LogInfo(msg='WARNING: ws281x package (led strip driver) is not installed, '
                                       'led/set_leds service and led/state topic are not available'))

    # high level led effects control, events notification with leds
    if is_true(arg('led_effect')):
        params = [{
            'led': 'led',
            'blink_rate': 2.0,
            'fade_period': 0.5,
            'rainbow_period': 5.0,
        }]
        if is_true(arg('led_notify')):
            # events effects table
            params.append(os.path.join(get_package_share_directory('clover'), 'config', 'led_notify.yaml'))
        actions.append(Node(
            package='clover',
            executable='led',
            name='led_effect',
            parameters=params,
            output='screen',
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('ws281x', default_value='true'),
        DeclareLaunchArgument('led_effect', default_value='true'),
        DeclareLaunchArgument('led_notify', default_value='true'),
        DeclareLaunchArgument('led_count', default_value='72'),
        DeclareLaunchArgument('gpio_pin', default_value='21'),
        DeclareLaunchArgument('simulator', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
