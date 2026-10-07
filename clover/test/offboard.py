import os
import sys
import time

import launch
import launch_testing.actions
import pytest
from pytest import approx
import mavros_msgs.msg
from mavros_msgs.srv import SetMode
from geometry_msgs.msg import PoseStamped
from clover import srv
from clover.msg import State
from std_srvs.srv import Trigger
from math import nan, inf
from rclpy.duration import Duration
import tf2_ros

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from clover_test_utils import CloverTestCase, LATCHED, clover_node, static_transform  # noqa: E402


@pytest.mark.launch_test
def generate_test_description():
    return launch.LaunchDescription([
        clover_node('simple_offboard'),
        static_transform('test_frame', 10, 20, 30, 'map', 'test'),
        static_transform('test2_frame', 100, 200, 300, 'map', 'test2'),
        launch_testing.actions.ReadyToTest(),
    ])


class TestOffboard(CloverTestCase):
    NODE_NAME = 'offboard_test'

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        cls.tf_buffer = tf2_ros.Buffer()
        cls.tf_listener = tf2_ros.TransformListener(cls.tf_buffer, cls.node, spin_thread=False)

    def get_state(self):
        return self.wait_for_message(State, '/simple_offboard/state', LATCHED, timeout=5)

    def get_navigate_target(self):
        target = self.tf_buffer.lookup_transform('map', 'navigate_target', self.node.get_clock().now(),
                                                 Duration(seconds=3))
        assert target.child_frame_id == 'navigate_target'
        return target

    def test_offboard(self):
        node = self.node
        tf_buffer = self.tf_buffer
        get_state = self.get_state

        navigate = self.service_proxy('navigate', srv.Navigate)
        set_position = self.service_proxy('set_position', srv.SetPosition)
        set_altitude = self.service_proxy('set_altitude', srv.SetAltitude)
        set_yaw = self.service_proxy('set_yaw', srv.SetYaw)
        set_yaw_rate = self.service_proxy('set_yaw_rate', srv.SetYawRate)
        set_velocity = self.service_proxy('set_velocity', srv.SetVelocity)
        set_attitude = self.service_proxy('set_attitude', srv.SetAttitude)
        set_rates = self.service_proxy('set_rates', srv.SetRates)
        get_telemetry = self.service_proxy('get_telemetry', srv.GetTelemetry)
        land = self.service_proxy('land', Trigger)

        # the node reads the frames from mavros on start, which takes several seconds without mavros
        self.wait_for_service('land', timeout=30)

        res = navigate()
        assert res.success == False
        assert res.message.startswith('State timeout')

        telem = get_telemetry()
        assert telem.connected == False

        # mocked state publisher
        state_pub = node.create_publisher(mavros_msgs.msg.State, '/mavros/state', LATCHED)
        state_msg = mavros_msgs.msg.State(mode='OFFBOARD', armed=True)

        def publish_state():
            state_msg.header.stamp = node.get_clock().now().to_msg()
            state_pub.publish(state_msg)

        # start publishing state
        node.create_timer(1 / 2, publish_state)
        time.sleep(0.5)

        # set_mode service mock
        def set_mode(req, res):
            state_msg.mode = req.custom_mode # set mocked mode to requested
            res.mode_sent = True
            return res

        node.create_service(SetMode, '/mavros/set_mode', set_mode)

        telem = get_telemetry()
        assert telem.connected == False

        res = navigate()
        assert res.success == False
        assert res.message.startswith('No connection to FCU')

        state_msg.connected = True
        time.sleep(1)

        telem = get_telemetry()
        assert telem.connected == True

        res = navigate()
        assert res.success == False
        assert res.message.startswith('No local position')

        local_position_pub = node.create_publisher(PoseStamped, '/mavros/local_position/pose', 1)
        local_position_msg = PoseStamped()
        local_position_msg.header.frame_id = 'map'
        local_position_msg.pose.position.x = 1.0
        local_position_msg.pose.position.y = 2.0
        local_position_msg.pose.position.z = 3.0
        local_position_msg.pose.orientation.w = 1.0

        def publish_local_position():
            local_position_msg.header.stamp = node.get_clock().now().to_msg()
            local_position_pub.publish(local_position_msg)

        # start publishing local position
        node.create_timer(1 / 30, publish_local_position)
        time.sleep(0.5)

        # check body frame
        body = tf_buffer.lookup_transform('map', 'body', node.get_clock().now(), Duration(seconds=3))
        assert body.child_frame_id == 'body'
        assert body.transform.translation.x == approx(1)
        assert body.transform.translation.y == approx(2)
        assert body.transform.translation.z == approx(3)

        res = navigate(x=3, y=2, z=1, frame_id='map')
        assert res.success == True
        state = get_state()
        assert state.mode == State.MODE_NAVIGATE
        assert state.yaw_mode == State.YAW_MODE_YAW
        assert state.x == 3
        assert state.y == 2
        assert state.z == 1
        assert state.yaw == 0
        assert state.xy_frame_id == 'map'
        assert state.z_frame_id == 'map'
        assert state.yaw_frame_id == 'map'
        target = self.get_navigate_target()
        assert target.header.frame_id == 'map'
        assert target.transform.translation.x == approx(3)
        assert target.transform.translation.y == approx(2)
        assert target.transform.translation.z == approx(1)
        assert target.transform.rotation.x == 0
        assert target.transform.rotation.y == 0
        assert target.transform.rotation.z == 0
        assert target.transform.rotation.w == 1

        # try to set only the y
        res = navigate(x=nan, y=1, z=nan)
        assert res.success == False
        assert res.message.startswith('x and y can be set only together')

        # set z in body frame
        res = navigate(x=nan, y=nan, z=1, frame_id='body')
        assert res.success == True
        state = get_state()
        assert state.mode == State.MODE_NAVIGATE
        assert state.yaw_mode == State.YAW_MODE_YAW
        assert state.x == 3
        assert state.y == 2
        assert state.z == 4
        assert state.yaw == 0
        assert state.xy_frame_id == 'map'
        assert state.z_frame_id == 'map'
        assert state.yaw_frame_id == 'map'

        # set xy in test frame
        res = navigate(x=1, y=2, z=nan, frame_id='test')
        assert res.success == True
        state = get_state()
        assert state.mode == State.MODE_NAVIGATE
        assert state.yaw_mode == State.YAW_MODE_YAW
        assert state.x == 1
        assert state.y == 2
        assert state.z == 4
        assert state.yaw == 0
        assert state.xy_frame_id == 'test'
        assert state.z_frame_id == 'map'
        assert state.yaw_frame_id == 'test'

        # auto_arm should not invalidate the setpoint if not effective
        res = navigate(x=nan, y=nan, z=1, frame_id='map', auto_arm=True)
        assert res.success == True
        state = get_state()
        assert state.mode == State.MODE_NAVIGATE
        assert state.yaw_mode == State.YAW_MODE_YAW
        assert state.x == 1
        assert state.y == 2
        assert state.z == 1
        assert state.yaw == 0
        assert state.xy_frame_id == 'test'
        assert state.z_frame_id == 'map'
        assert state.yaw_frame_id == 'map'

        # auto_arm should invalidate the setpoint if effective
        state_msg.mode = 'STABILIZED' # pretend we are not in OFFBOARD mode
        time.sleep(1)
        res = navigate(x=nan, y=nan, z=1, frame_id='map', auto_arm=True)
        assert res.success == True
        state = get_state()
        assert state.mode == State.MODE_NAVIGATE
        assert state.yaw_mode == State.YAW_MODE_YAW
        assert state.x == 1
        assert state.y == 2
        assert state.z == 1
        assert state.yaw == 0
        assert state.xy_frame_id == 'map'
        assert state.z_frame_id == 'map'
        assert state.yaw_frame_id == 'map'
        state_msg.mode = 'OFFBOARD'
        time.sleep(1)

        # set_attitude should invalidate the setpoint
        res = set_attitude()
        assert res.success == True

        res = navigate(x=5, y=6, z=nan, yaw=nan, frame_id='map')
        assert res.success == True
        state = get_state()
        assert state.mode == State.MODE_NAVIGATE
        assert state.yaw_mode == State.YAW_MODE_YAW
        assert state.x == 5
        assert state.y == 6
        assert state.z == 3
        assert state.yaw == 0
        assert state.xy_frame_id == 'map'
        assert state.z_frame_id == 'map'
        assert state.yaw_frame_id == 'map'

        # test set_altitude
        res = set_altitude(z=7, frame_id='test')
        assert res.success == True
        state = get_state()
        assert state.mode == State.MODE_NAVIGATE
        assert state.yaw_mode == State.YAW_MODE_YAW
        assert state.x == 5
        assert state.y == 6
        assert state.z == 7
        assert state.yaw == 0
        assert state.xy_frame_id == 'map'
        assert state.z_frame_id == 'test'
        assert state.yaw_frame_id == 'map'

        # test set_yaw
        res = set_yaw(yaw=0.5, frame_id='test2')
        assert res.success == True
        state = get_state()
        assert state.mode == State.MODE_NAVIGATE
        assert state.yaw_mode == State.YAW_MODE_YAW
        assert state.x == 5
        assert state.y == 6
        assert state.z == 7
        assert state.yaw == 0.5
        assert state.xy_frame_id == 'map'
        assert state.z_frame_id == 'test'
        assert state.yaw_frame_id == 'test2'

        # test set_yaw_rate
        res = set_yaw_rate(yaw_rate=2)
        assert res.success == True
        state = get_state()
        assert state.mode == State.MODE_NAVIGATE
        assert state.yaw_mode == State.YAW_MODE_YAW_RATE
        assert state.x == 5
        assert state.y == 6
        assert state.z == 7
        assert state.yaw_rate == 2
        assert state.xy_frame_id == 'map'
        assert state.z_frame_id == 'test'

        # navigate(yaw=nan) should keep yaw rate mode
        res = navigate(x=nan, y=nan, z=nan, yaw=nan)
        assert res.success == True
        state = get_state()
        assert state.mode == State.MODE_NAVIGATE
        assert state.yaw_mode == State.YAW_MODE_YAW_RATE
        assert state.x == 5
        assert state.y == 6
        assert state.z == 7
        assert state.yaw_rate == 2
        assert state.xy_frame_id == 'map'
        assert state.z_frame_id == 'test'

        # set_yaw(nan) should change back to yaw mode
        res = set_yaw(yaw=nan)
        assert res.success == True
        state = get_state()
        assert state.mode == State.MODE_NAVIGATE
        assert state.yaw_mode == State.YAW_MODE_YAW
        assert state.yaw == 0
        assert state.yaw_frame_id == 'map'

        # test set_position
        res = set_position(x=nan, y=nan, z=13, yaw=nan, frame_id='test2')
        assert res.success == True
        state = get_state()
        assert state.mode == State.MODE_POSITION
        assert state.yaw_mode == State.YAW_MODE_YAW
        assert state.x == 5
        assert state.y == 6
        assert state.z == 13
        assert state.yaw == 0
        assert state.xy_frame_id == 'map'
        assert state.z_frame_id == 'test2'
        assert state.yaw_frame_id == 'map'

        # set_altitude should not change the mode
        res = set_altitude(z=3, frame_id='test')
        assert res.success == True
        state = get_state()
        assert state.mode == State.MODE_POSITION
        assert state.yaw_mode == State.YAW_MODE_YAW
        assert state.x == 5
        assert state.y == 6
        assert state.z == 3
        assert state.yaw == 0
        assert state.xy_frame_id == 'map'
        assert state.z_frame_id == 'test'
        assert state.yaw_frame_id == 'map'

        # set_yaw should not change the main mode
        res = set_yaw(yaw=1, frame_id='test2')
        assert res.success == True
        state = get_state()
        assert state.mode == State.MODE_POSITION
        assert state.yaw_mode == State.YAW_MODE_YAW
        assert state.x == 5
        assert state.y == 6
        assert state.z == 3
        assert state.yaw == 1
        assert state.xy_frame_id == 'map'
        assert state.z_frame_id == 'test'
        assert state.yaw_frame_id == 'test2'

        # test set_velocity
        res = set_velocity(vx=1, frame_id='body')
        state = get_state()
        assert state.mode == State.MODE_VELOCITY
        assert state.yaw_mode == State.YAW_MODE_YAW
        assert state.vx == 1
        assert state.vy == 0
        assert state.vz == 0
        assert state.yaw == 0
        assert state.xy_frame_id == 'map'
        assert state.z_frame_id == 'map'
        assert state.yaw_frame_id == 'map'

        # set_altitude should not work in velocity mode
        res = set_altitude(z=3, frame_id='test')
        assert res.success == False
        assert res.message.startswith('Altitude cannot be set in')

        # test set_attitude
        res = set_attitude(roll=0.1, pitch=0.2, yaw=0.3, thrust=0.5)
        assert res.success == True
        state = get_state()
        assert state.mode == State.MODE_ATTITUDE
        assert state.yaw_mode == State.YAW_MODE_YAW
        assert state.roll == approx(0.1)
        assert state.pitch == approx(0.2)
        assert state.yaw == approx(0.3)
        assert state.thrust == approx(0.5)
        assert state.yaw_frame_id == 'map'
        msg = self.wait_for_message(PoseStamped, '/mavros/setpoint_attitude/attitude', timeout=3)
        # Tait-Bryan ZYX angle (rzyx) converted to quaternion
        assert msg.pose.orientation.x == approx(0.0342708)
        assert msg.pose.orientation.y == approx(0.10602051)
        assert msg.pose.orientation.z == approx(0.14357218)
        assert msg.pose.orientation.w == approx(0.98334744)
        msg = self.wait_for_message(mavros_msgs.msg.Thrust, '/mavros/setpoint_attitude/thrust', timeout=3)
        assert msg.thrust == approx(0.5)

        # set_yaw should work in attitude mode
        res = set_yaw(yaw=0.7, frame_id='test2')
        assert res.success == True
        state = get_state()
        assert state.mode == State.MODE_ATTITUDE
        assert state.yaw_mode == State.YAW_MODE_YAW
        assert state.roll == approx(0.1)
        assert state.pitch == approx(0.2)
        assert state.yaw == approx(0.7)
        assert state.thrust == approx(0.5)
        assert state.yaw_frame_id == 'test2'

        # set_yaw_rate should not work in attitude mode
        res = set_yaw_rate(yaw_rate=0.3)
        assert res.success == False
        assert res.message.startswith('Yaw rate cannot be set in')

        # test set_rates
        res = set_rates(roll_rate=nan, pitch_rate=nan, yaw_rate=0.3, thrust=0.6)
        assert res.success == True
        state = get_state()
        assert state.mode == State.MODE_RATES
        assert state.yaw_mode == State.YAW_MODE_YAW_RATE
        assert state.roll_rate == approx(0)
        assert state.pitch_rate == approx(0)
        assert state.yaw_rate == approx(0.3)
        assert state.thrust == approx(0.6)
        msg = self.wait_for_message(mavros_msgs.msg.AttitudeTarget, '/mavros/setpoint_raw/attitude', timeout=3)
        assert msg.thrust == approx(0.6)

        res = set_rates(roll_rate=0.3, pitch_rate=0.2, yaw_rate=0.1, thrust=0.4)
        assert res.success == True
        state = get_state()
        assert state.mode == State.MODE_RATES
        assert state.yaw_mode == State.YAW_MODE_YAW_RATE
        assert state.roll_rate == approx(0.3)
        assert state.pitch_rate == approx(0.2)
        assert state.yaw_rate == approx(0.1)
        assert state.thrust == approx(0.4)

        res = set_rates(roll_rate=nan, pitch_rate=nan, yaw_rate=nan, thrust=0.3)
        assert res.success == True
        state = get_state()
        assert state.mode == State.MODE_RATES
        assert state.yaw_mode == State.YAW_MODE_YAW_RATE
        assert state.roll_rate == approx(0.3)
        assert state.pitch_rate == approx(0.2)
        assert state.yaw_rate == approx(0.1)
        assert state.thrust == approx(0.3)
        msg = self.wait_for_message(mavros_msgs.msg.AttitudeTarget, '/mavros/setpoint_raw/attitude', timeout=3)
        assert msg.type_mask == mavros_msgs.msg.AttitudeTarget.IGNORE_ATTITUDE
        assert msg.body_rate.x == approx(0.3)
        assert msg.body_rate.y == approx(0.2)
        assert msg.body_rate.z == approx(0.1)

        # set_yaw_rate should work in rates mode
        res = set_yaw_rate(yaw_rate=0.4)
        assert res.success == True
        state = get_state()
        assert state.mode == State.MODE_RATES
        assert state.yaw_mode == State.YAW_MODE_YAW_RATE
        assert state.roll_rate == approx(0.3)
        assert state.pitch_rate == approx(0.2)
        assert state.yaw_rate == approx(0.4)
        assert state.thrust == approx(0.3)

        res = set_rates(roll_rate=inf)
        assert res.success == False
        assert res.message == 'roll_rate argument cannot be Inf'

        # test land service
        res = land()
        assert res.success == True
        assert state_msg.mode == 'AUTO.LAND' # check that the mode was set correctly
