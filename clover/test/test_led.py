# Test of the led node with a mock LED driver (set_leds service and state topic),
# mock mavros state and battery, and messages published to /rosout.

import os
import sys
import tempfile
import time

import launch
import launch_testing.actions
import pytest
from launch_ros.actions import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rcl_interfaces.msg import Log
from sensor_msgs.msg import BatteryState
from mavros_msgs.msg import State
from led_msgs.msg import LEDState, LEDStateArray
from led_msgs.srv import SetLEDs
from clover.srv import SetLEDEffect

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from clover_test_utils import BEST_EFFORT, CloverTestCase, LATCHED, wait  # noqa: E402

LED_COUNT = 10
OFF = (0, 0, 0)

PARAMS = '''
led:
  ros__parameters:
    blink_rate: 5.0
    fade_period: 0.5
    wipe_period: 0.5
    flash_delay: 0.1
    rainbow_period: 2.55
    # events effects table, as in led.launch
    notify:
      startup: { r: 255, g: 255, b: 255 }
      connected: { effect: rainbow }
      disconnected: { effect: blink, r: 255, g: 50, b: 50 }
      offboard: { r: 220, g: 20, b: 250 }
      manual: { r: 1, g: 2, b: 3 }
      low_battery: { threshold: 3.6, effect: blink_fast, r: 255, g: 0, b: 0 }
      error: { effect: flash, r: 255, g: 0, b: 0, ignore: [ "[lpe] vision position timeout" ] }
'''


class MockDriver:
    # LED strip driver: led/set_leds service, led/state topic (latched)
    def __init__(self, node):
        self.strip = [OFF] * LED_COUNT
        self.calls = []  # (time, {index: color})
        self.delay = 0
        self.state_pub = node.create_publisher(LEDStateArray, 'led/state', LATCHED)
        self.publish_state()
        self.srv = node.create_service(SetLEDs, 'led/set_leds', self.set_leds,
                                       callback_group=MutuallyExclusiveCallbackGroup())

    def publish_state(self):
        self.state_pub.publish(LEDStateArray(leds=[LEDState(index=i, r=c[0], g=c[1], b=c[2])
                                                   for i, c in enumerate(self.strip)]))

    def set_leds(self, req, res):
        time.sleep(self.delay)
        strip = list(self.strip)
        for led in req.leds:
            if led.index >= LED_COUNT:
                res.message = 'wrong index'
                return res
            strip[led.index] = (led.r, led.g, led.b)
        self.strip = strip
        self.calls.append((time.time(), {led.index: (led.r, led.g, led.b) for led in req.leds}))
        self.publish_state()
        res.success = True
        return res

    def filled(self, color):
        return all(c == tuple(color) for c in self.strip)

    def calls_since(self, mark):
        return self.calls[mark:]


@pytest.mark.launch_test
def generate_test_description():
    params = tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False)
    params.write(PARAMS)
    params.close()
    return launch.LaunchDescription([
        Node(package='clover', executable='led', name='led', parameters=[params.name], output='screen'),
        launch_testing.actions.ReadyToTest(),
    ])


class TestLed(CloverTestCase):
    def test_led(self):
        node = self.node
        set_effect = node.create_client(SetLEDEffect, 'led/set_effect')

        def effect(effect='', r=0, g=0, b=0):
            future = set_effect.call_async(SetLEDEffect.Request(effect=effect, r=r, g=g, b=b))
            assert wait(future.done, 5.0), 'no response from set_effect'
            return future.result()

        # the node waits for the driver
        assert not set_effect.wait_for_service(timeout_sec=3.0), 'set_effect is available without the LED driver'
        driver = MockDriver(node)
        assert set_effect.wait_for_service(timeout_sec=10.0), 'no set_effect service'

        services = dict(node.get_service_names_and_types())
        assert services['/led/set_effect'] == ['clover/srv/SetLEDEffect']
        assert services['/led/set_leds'] == ['led_msgs/srv/SetLEDs']

        # startup event
        assert wait(lambda: driver.filled((255, 255, 255)), 5.0), 'no startup notification'

        # fill
        assert effect('fill', 10, 20, 30).success
        assert wait(lambda: driver.filled((10, 20, 30)), 2.0), 'fill'
        assert effect('', 30, 20, 10).success  # empty effect is fill
        assert wait(lambda: driver.filled((30, 20, 10)), 2.0), 'empty effect'

        # blink, 5 Hz
        mark = len(driver.calls)
        assert effect('blink', 0, 0, 255).success
        time.sleep(1.05)
        calls = driver.calls_since(mark)
        colors = [c[0] for t, c in calls]
        period = (calls[-1][0] - calls[0][0]) / (len(calls) - 1)
        assert 5 <= len(calls) <= 7 and abs(period - 0.2) < 0.03
        assert all(len(c) == LED_COUNT for t, c in calls)
        assert colors[0] == (0, 0, 255) and all(a != b for a, b in zip(colors, colors[1:]))  # on, off, on...
        res = effect('blink', 0, 0, 255)
        assert res.success and res.message == 'Effect already set, skip'

        # blink_fast, 10 Hz by default
        mark = len(driver.calls)
        assert effect('blink_fast', 0, 255, 255).success
        time.sleep(1.05)
        calls = driver.calls_since(mark)
        period = (calls[-1][0] - calls[0][0]) / (len(calls) - 1)
        assert 9 <= len(calls) <= 12 and abs(period - 0.1) < 0.02

        # fade
        effect('fill', 0, 0, 100)
        assert wait(lambda: driver.filled((0, 0, 100)), 2.0)
        time.sleep(0.2)  # the node gets the state
        mark = len(driver.calls)
        assert effect('fade', 200, 100, 0).success
        assert wait(lambda: driver.filled((200, 100, 0)), 2.0), 'fade is not finished'
        calls = driver.calls_since(mark)
        reds = [c[0][0] for t, c in calls]
        assert reds == sorted(reds) and reds[0] == 0 and 0 < reds[len(reds) // 2] < 200
        assert 0.4 < calls[-1][0] - calls[0][0] < 0.7
        count = len(driver.calls)
        time.sleep(0.3)
        assert len(driver.calls) == count, 'fade timer is not stopped'

        # wipe
        mark = len(driver.calls)
        assert effect('wipe', 0, 255, 0).success
        assert wait(lambda: driver.filled((0, 255, 0)), 2.0), 'wipe is not finished'
        calls = driver.calls_since(mark)
        assert [list(c) for t, c in calls] == [[i] for i in range(LED_COUNT)]  # one LED in a frame
        count = len(driver.calls)
        time.sleep(0.3)
        assert len(driver.calls) == count, 'wipe timer is not stopped'

        # rainbow, 10 ms frame
        mark = len(driver.calls)
        assert effect('rainbow').success
        time.sleep(1.0)
        calls = driver.calls_since(mark)
        assert 60 <= len(calls) <= 110
        assert all(len(set(c.values())) > 1 for t, c in calls)  # different colors in the strip
        assert calls[0][1] != calls[10][1]

        assert effect('rainbow_fill').success
        time.sleep(0.1)  # the last frames of the previous effect are done
        mark = len(driver.calls)
        time.sleep(1.0)
        calls = driver.calls_since(mark)
        assert 60 <= len(calls) <= 110
        assert all(len(set(c.values())) == 1 for t, c in calls)  # the same color in the strip
        assert calls[0][1] != calls[10][1]

        # flash, restores the filling
        effect('fill', 5, 5, 5)
        assert wait(lambda: driver.filled((5, 5, 5)), 2.0)
        mark = len(driver.calls)
        start = time.time()
        assert effect('flash', 255, 0, 0).success
        duration = time.time() - start
        assert wait(lambda: driver.filled((5, 5, 5)), 2.0), 'filling is not restored after flash'
        colors = [c[0] for t, c in driver.calls_since(mark)]
        assert colors[:5] == [OFF, (255, 0, 0), OFF, (255, 0, 0), OFF] and colors[-1] == (5, 5, 5)
        assert 0.45 < duration < 0.7

        # unknown effect; the node's error is in /rosout, which is an error event itself
        mark = len(driver.calls)
        res = effect('disco', 1, 1, 1)
        assert not res.success and res.message.startswith('Unknown effect: disco')
        assert wait(lambda: any(c[0] == (255, 0, 0) for t, c in driver.calls_since(mark)), 2.0), 'no flash on own error'
        assert wait(lambda: driver.filled((5, 5, 5)), 2.0)
        time.sleep(0.3)

        # mavros state events
        state_pub = node.create_publisher(State, 'mavros/state', LATCHED)  # as mavros
        mark = len(driver.calls)
        state_pub.publish(State(connected=True))
        assert wait(lambda: len(driver.calls_since(mark)) > 20, 2.0), 'no connected notification'
        assert len(set(driver.calls[-1][1].values())) > 1

        state_pub.publish(State(connected=True, mode='OFFBOARD'))
        assert wait(lambda: driver.filled((220, 20, 250)), 2.0), 'no offboard notification'
        state_pub.publish(State(connected=True, mode='MANUAL'))  # event is not in the known list
        assert wait(lambda: driver.filled((1, 2, 3)), 2.0), 'no manual notification'
        state_pub.publish(State(connected=True, mode='AUTO.LAND'))  # no effect is set for this event
        time.sleep(0.5)
        assert driver.filled((1, 2, 3))

        # parameters of the table are read on each event
        params_client = AsyncParameterClient(node, 'led')
        future = params_client.set_parameters([Parameter('notify.armed.r', value=9)])
        assert wait(future.done, 5.0) and future.result().results[0].successful
        state_pub.publish(State(connected=True, mode='AUTO.LAND', armed=True))
        assert wait(lambda: driver.filled((9, 0, 0)), 2.0), 'no armed notification'

        mark = len(driver.calls)
        state_pub.publish(State(connected=False))
        assert wait(lambda: len(driver.calls_since(mark)) >= 3, 2.0), 'no disconnected notification'
        colors = [c[0] for t, c in driver.calls_since(mark)]
        assert colors[:3] == [(255, 50, 50), OFF, (255, 50, 50)]

        # battery
        effect('fill', 7, 7, 7)
        time.sleep(0.3)
        battery_pub = node.create_publisher(BatteryState, 'mavros/battery', BEST_EFFORT)  # as mavros
        mark = len(driver.calls)
        battery_pub.publish(BatteryState(cell_voltage=[3.9, 3.7, 1.0]))  # threshold is 3.6
        time.sleep(0.5)
        assert not driver.calls_since(mark), 'low battery is notified for normal voltage'
        battery_pub.publish(BatteryState(cell_voltage=[3.9, 3.5]))
        assert wait(lambda: len(driver.calls_since(mark)) >= 4, 2.0), 'no low battery notification'
        calls = driver.calls_since(mark)
        assert [c[0] for t, c in calls[:2]] == [(255, 0, 0), OFF]

        # errors in the log
        effect('fill', 7, 7, 7)
        assert wait(lambda: driver.filled((7, 7, 7)), 2.0)
        time.sleep(0.3)
        rosout_pub = node.create_publisher(Log, '/rosout', 10)
        assert wait(lambda: rosout_pub.get_subscription_count() >= 1, 5.0)
        mark = len(driver.calls)
        rosout_pub.publish(Log(level=Log.WARN, name='mock', msg='just a warning'))
        rosout_pub.publish(Log(level=Log.ERROR, name='mock', msg='[lpe] vision position timeout'))  # ignored
        time.sleep(1.0)
        assert not driver.calls_since(mark), 'warning or ignored error is notified'
        rosout_pub.publish(Log(level=Log.ERROR, name='mock', msg='something is broken'))
        assert wait(lambda: len(driver.calls_since(mark)) >= 6, 2.0), 'no error notification'
        colors = [c[0] for t, c in driver.calls_since(mark)]
        assert colors[:5] == [OFF, (255, 0, 0), OFF, (255, 0, 0), OFF] and driver.filled((7, 7, 7))
        mark = len(driver.calls)
        rosout_pub.publish(Log(level=Log.FATAL, name='mock', msg='fatal'))
        assert wait(lambda: len(driver.calls_since(mark)) >= 6, 2.0), 'no fatal error notification'
        time.sleep(0.3)

        # slow driver: frames are merged, the result is the same
        driver.delay = 0.12
        mark = len(driver.calls)
        assert effect('wipe', 0, 0, 255).success
        assert wait(lambda: driver.filled((0, 0, 255)), 5.0), 'wipe is not finished with slow driver'
        calls = driver.calls_since(mark)
        assert len(calls) < LED_COUNT and sum(len(c) for t, c in calls) == LED_COUNT
        driver.delay = 0
