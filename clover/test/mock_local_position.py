#!/usr/bin/env python3
# Mock of mavros local_position plugin node: only holds tf.frame_id and tf.child_frame_id parameters,
# which the nodes of the package read the frames from.
# Usage: python3 mock_local_position.py --ros-args -r __ns:=/mavros -p tf.frame_id:=map -p tf.child_frame_id:=base_link

import rclpy
from rclpy.executors import ExternalShutdownException


def main():
    rclpy.init()
    node = rclpy.create_node('local_position', automatically_declare_parameters_from_overrides=True)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
