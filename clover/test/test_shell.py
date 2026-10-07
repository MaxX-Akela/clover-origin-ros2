# Test of the shell node: calls the exec service.

import os
import sys

import launch
import launch_testing.actions
import pytest
from rclpy.parameter_client import AsyncParameterClient
from clover.srv import Execute

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from clover_test_utils import CloverTestCase, clover_node, wait  # noqa: E402


@pytest.mark.launch_test
def generate_test_description():
    return launch.LaunchDescription([
        clover_node('shell', timeout=5.0),
        launch_testing.actions.ReadyToTest(),
    ])


class TestShell(CloverTestCase):
    def test_exec(self):
        execute = self.service_proxy('exec', Execute)
        assert execute.client.wait_for_service(timeout_sec=10.0), 'exec service is not available'

        res = execute(cmd='echo hello; echo world')
        assert res.output == 'hello\nworld\n'
        assert res.code == 0

        res = execute(cmd='exit 3')
        assert res.code == 3 << 8  # raw pclose() status, as in the original

    def test_parameters_while_running(self):
        # the node answers parameter requests while a command is running
        execute = self.service_proxy('exec', Execute)
        assert execute.client.wait_for_service(timeout_sec=10.0), 'exec service is not available'
        slow = execute.client.call_async(Execute.Request(cmd='sleep 2; echo done'))
        params = AsyncParameterClient(self.node, 'shell')
        assert params.wait_for_services(timeout_sec=5.0)
        future = params.get_parameters(['timeout'])
        assert wait(future.done, 1.0) and not slow.done(), 'parameters are blocked by a running command'
        assert future.result().values[0].double_value == 5.0
        assert wait(slow.done, 10.0)
        assert slow.result().output == 'done\n'
