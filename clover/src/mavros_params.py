#!/usr/bin/env python3

# Set parameters of mavros plugins from a parameters file
#
# Plugins of mavros 2.15 are separate nodes that ignore the command line arguments of
# the mavros_node process, so the parameters file passed to mavros doesn't reach them.
# This node sets the parameters with the parameter services of the plugin nodes
# and repeats this when mavros is restarted.
#
# Distributed under MIT License (available at https://opensource.org/licenses/MIT).
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

import sys

import rclpy
import yaml
from rcl_interfaces.srv import SetParameters
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter

PREFIX = '/**/'

# Parameters that can't be changed in a running mavros 2.15: setting use_quaternion fails
# and leaves setpoint_attitude plugin without its subscribers
SKIP = {'setpoint_attitude': ['use_quaternion']}


def flatten(params, prefix=''):
    # {'tf': {'send': True}} => {'tf.send': True}
    result = {}
    for name, value in params.items():
        if isinstance(value, dict):
            result.update(flatten(value, prefix + name + '.'))
        else:
            result[prefix + name] = value
    return result


class MavrosParams(Node):
    def __init__(self):
        super().__init__('mavros_params')
        config = self.declare_parameter('config', '').value
        mavros = self.declare_parameter('mavros', 'mavros').value

        with open(config) as f:
            data = yaml.safe_load(f)

        self.plugins = []
        for key, value in data.items():
            if not key.startswith(PREFIX):
                continue  # parameters of the main nodes are passed to mavros_node itself
            plugin = key[len(PREFIX):]
            name = mavros + '/' + plugin
            params = [Parameter(param_name, value=param_value).to_parameter_msg()
                      for param_name, param_value in flatten(value.get('ros__parameters', {})).items()
                      if param_name not in SKIP.get(plugin, [])]
            self.plugins.append({
                'name': name,
                'params': params,
                'client': self.create_client(SetParameters, name + '/set_parameters'),
                'sent': False,
            })

        self.create_timer(1.0, self.update)

    def update(self):
        for plugin in self.plugins:
            if not plugin['client'].service_is_ready():
                plugin['sent'] = False  # mavros isn't running or the plugin is disabled
            elif not plugin['sent']:
                plugin['sent'] = True
                future = plugin['client'].call_async(SetParameters.Request(parameters=plugin['params']))
                future.add_done_callback(lambda future, plugin=plugin: self.done(plugin, future))

    def done(self, plugin, future):
        if future.exception() is not None:
            self.get_logger().error('%s: %s' % (plugin['name'], future.exception()))
            return
        for param, result in zip(plugin['params'], future.result().results):
            if not result.successful:
                self.get_logger().error('%s: can\'t set %s: %s' % (plugin['name'], param.name, result.reason))
        self.get_logger().info('%s: parameters are set' % plugin['name'])


def main():
    try:
        rclpy.init()
        rclpy.spin(MavrosParams())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    sys.exit(main())
