# Test of the examples: all the examples are compiled; get_telemetry, navigate_wait, leds and camera
# are run with the nodes of the package and the mocks of the FCU, the LED driver and the camera.
# The other examples (flight, flight_marker, gps, red_circle, subscriber) are not run.

import glob
import os
import signal
import subprocess
import sys
import time

import launch
import launch_testing.actions
import numpy as np
import pytest
from pytest import approx
from geometry_msgs.msg import PoseStamped, TransformStamped
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode
from sensor_msgs.msg import Image
from tf2_ros import TransformBroadcaster

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from clover_test_utils import BEST_EFFORT, CloverTestCase, LATCHED, TEST_DIR, clover_node, wait  # noqa: E402
from test_led import MockDriver  # noqa: E402

EXAMPLES_DIR = os.path.join(os.path.dirname(TEST_DIR), 'examples')
EXAMPLES = ['camera', 'flight', 'flight_marker', 'get_telemetry', 'gps', 'leds', 'navigate_wait', 'red_circle',
            'subscriber']


@pytest.mark.launch_test
def generate_test_description():
    return launch.LaunchDescription([
        clover_node('simple_offboard', local_frame='map', fcu_frame='base_link'),
        clover_node('led'),
        launch_testing.actions.ReadyToTest(),
    ])


class MockFCU:
    # mavros with an FCU, which reaches the position setpoint immediately
    def __init__(self, node):
        self.node = node
        self.state = State(connected=True, mode='STABILIZED')
        self.pose = PoseStamped()
        self.pose.header.frame_id = 'map'
        self.pose.pose.orientation.w = 1.0
        self.state_pub = node.create_publisher(State, 'mavros/state', LATCHED)
        self.pose_pub = node.create_publisher(PoseStamped, 'mavros/local_position/pose', 1)
        self.br = TransformBroadcaster(node)  # as mavros with local_position tf.send
        node.create_subscription(PoseStamped, 'mavros/setpoint_position/local', self.handle_setpoint, BEST_EFFORT)
        node.create_subscription(PositionTarget, 'mavros/setpoint_raw/local', self.handle_setpoint_raw, BEST_EFFORT)
        node.create_service(SetMode, 'mavros/set_mode', self.set_mode)
        node.create_service(CommandBool, 'mavros/cmd/arming', self.arming)
        node.create_timer(1 / 30, self.publish)

    def publish(self):
        stamp = self.node.get_clock().now().to_msg()
        self.state.header.stamp = stamp
        self.pose.header.stamp = stamp
        self.state_pub.publish(self.state)
        self.pose_pub.publish(self.pose)
        t = TransformStamped()
        t.header = self.pose.header
        t.child_frame_id = 'base_link'
        p = self.pose.pose.position
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = p.x, p.y, p.z
        t.transform.rotation = self.pose.pose.orientation
        self.br.sendTransform(t)

    def flying(self):
        return self.state.armed and self.state.mode == 'OFFBOARD'

    def handle_setpoint(self, msg):
        if self.flying():
            self.pose.pose.position = msg.pose.position

    def handle_setpoint_raw(self, msg):
        if self.flying() and not msg.type_mask & (PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY |
                                                  PositionTarget.IGNORE_PZ):
            self.pose.pose.position = msg.position

    def set_mode(self, req, res):
        self.state.mode = req.custom_mode
        res.mode_sent = True
        return res

    def arming(self, req, res):
        self.state.armed = req.value
        res.success = True
        return res


def run_example(name, timeout=60):
    # returns the lines printed by the example
    res = subprocess.run([sys.executable, '-u', os.path.join(EXAMPLES_DIR, name + '.py')],
                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=timeout)
    output = res.stdout.decode()
    assert res.returncode == 0, '%s returned %s:\n%s' % (name, res.returncode, output)
    return output.strip().split('\n')


class TestExamples(CloverTestCase):
    NODE_NAME = 'examples_test'

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        cls.fcu = MockFCU(cls.node)
        cls.driver = MockDriver(cls.node)

    def test_compile(self):
        assert sorted(os.path.basename(f)[:-3] for f in glob.glob(os.path.join(EXAMPLES_DIR, '*.py'))) == EXAMPLES
        for name in EXAMPLES:
            path = os.path.join(EXAMPLES_DIR, name + '.py')
            with open(path) as f:
                compile(f.read(), path, 'exec')

    def test_1_get_telemetry(self):
        self.wait_for_service('get_telemetry', timeout=30)
        time.sleep(1.0)  # the node gets the state and the local position
        output = run_example('get_telemetry')
        assert len(output) == 1 and output[0].startswith('clover.srv.GetTelemetry_Response('), output
        assert "frame_id='map'" in output[0] and 'connected=True' in output[0] and 'armed=False' in output[0]
        assert "mode='STABILIZED'" in output[0] and 'x=0.0, y=0.0, z=0.0' in output[0]

    def test_2_navigate_wait(self):
        self.wait_for_service('navigate', timeout=30)
        output = run_example('navigate_wait')
        assert output == ['Take off 1 meter', 'Fly forward 1 m', 'Land'], output
        assert self.fcu.state.armed and self.fcu.state.mode == 'AUTO.LAND'
        position = self.fcu.pose.pose.position
        assert (position.x, position.y, position.z) == approx((1, 0, 1), abs=0.2)

    def test_leds(self):
        self.wait_for_service('led/set_effect', timeout=30)
        mark = len(self.driver.calls)
        output = run_example('leds')
        assert output == ['Fill red', 'Fill green', 'Fade to blue', 'Flash red', 'Blink white', 'Rainbow'], output
        colors = [list(c.values()) for _, c in self.driver.calls_since(mark)]
        for color in (255, 0, 0), (0, 100, 0), (0, 0, 255), (255, 255, 255):
            assert [color] * len(self.driver.strip) in colors, 'the strip was not filled with %s' % (color,)
        assert len(set(colors[-1])) > 1, 'no rainbow'

    def test_camera(self):
        node = self.node
        image_pub = node.create_publisher(Image, 'main_camera/image_raw', 1)
        frame = np.zeros((240, 320, 3), np.uint8)
        frame[:, :] = (255, 0, 0)  # blue in bgr8
        img = Image(width=320, height=240, step=320 * 3, encoding='bgr8', data=frame.tobytes())
        img.header.frame_id = 'main_camera_optical'
        timer = node.create_timer(0.1, lambda: image_pub.publish(img))
        centers = []
        node.create_subscription(Image, 'cv/center', centers.append, 1)

        proc = subprocess.Popen([sys.executable, '-u', os.path.join(EXAMPLES_DIR, 'camera.py')],
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        try:
            assert wait(lambda: centers, 30.0), 'no cropped image'
            assert (centers[-1].width, centers[-1].height, centers[-1].encoding) == (40, 40, 'bgr8')
            center = np.frombuffer(centers[-1].data, np.uint8).reshape(40, 40, 3)
            assert (center == (255, 0, 0)).all()
        finally:
            timer.cancel()
            proc.send_signal(signal.SIGINT)
            output = proc.communicate(timeout=10)[0].decode()
        assert output.split('\n')[0] == 'blue', output
