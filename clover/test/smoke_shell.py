#!/usr/bin/env python3
# Smoke run for the shell node: starts the node and calls the exec service.
# Usage (workspace sourced): python3 smoke_shell.py

import os
import signal
import subprocess
import sys

os.environ.setdefault('ROS_DOMAIN_ID', '87')

import rclpy
from rclpy.parameter_client import AsyncParameterClient
from clover.srv import Execute


def call(node, client, request, timeout=10.0):
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    assert future.done(), 'no response from the service'
    return future.result()


def main():
    proc = subprocess.Popen(['ros2', 'run', 'clover', 'shell', '--ros-args', '-p', 'timeout:=5.0'],
                            start_new_session=True)
    rclpy.init()
    node = rclpy.create_node('smoke_shell')
    try:
        client = node.create_client(Execute, 'exec')
        assert client.wait_for_service(timeout_sec=10.0), 'exec service is not available'

        res = call(node, client, Execute.Request(cmd='echo hello; echo world'))
        print('output: %r, code: %d' % (res.output, res.code))
        assert res.output == 'hello\nworld\n'
        assert res.code == 0

        res = call(node, client, Execute.Request(cmd='exit 3'))
        print('exit 3 -> code: %d' % res.code)
        assert res.code == 3 << 8  # raw pclose() status, as in the original

        # the node answers parameter requests while a command is running
        slow = client.call_async(Execute.Request(cmd='sleep 2; echo done'))
        params = AsyncParameterClient(node, 'shell')
        assert params.wait_for_services(timeout_sec=5.0)
        future = params.get_parameters(['timeout'])
        rclpy.spin_until_future_complete(node, future, timeout_sec=1.0)
        assert future.done() and not slow.done(), 'parameters are blocked by a running command'
        print('timeout parameter: %s' % future.result().values[0].double_value)
        assert future.result().values[0].double_value == 5.0
        rclpy.spin_until_future_complete(node, slow, timeout_sec=10.0)
        assert slow.result().output == 'done\n'

        print('OK')
    finally:
        os.killpg(proc.pid, signal.SIGINT)
        proc.wait(timeout=10)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
