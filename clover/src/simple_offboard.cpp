/*
 * Simplified copter control in OFFBOARD mode
 * Copyright (C) 2019 Copter Express Technologies
 *
 * Author: Oleg Kalachev <okalachev@gmail.com>
 *
 * Distributed under MIT License (available at https://opensource.org/licenses/MIT).
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 */

/*
 * Threading rules (ROS 2):
 *
 * 1. All the state below is guarded by state_mutex. Every callback (service,
 *    subscription, timer) locks it for its whole duration, so callbacks never
 *    run simultaneously, like in the single-threaded ROS 1 node.
 * 2. Any waiting inside a callback MUST release state_mutex: sleeping in a
 *    loop (spinSleep) and waiting for a mavros service response (callService)
 *    as well. The mutex is never held while waiting for anything, otherwise
 *    the callbacks the wait depends on (state updates, setpoint timer, service
 *    responses) would be blocked. These are the places where the ROS 1 node
 *    called ros::spinOnce().
 * 3. Mavros service clients live in a separate callback group and are only
 *    called asynchronously (async_send_request), never with a blocking call.
 */

#include <algorithm>
#include <string>
#include <cmath>
#include <map>
#include <mutex>
#include <thread>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <GeographicLib/Geodesic.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/attitude_target.hpp>
#include <mavros_msgs/msg/thrust.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/status_text.hpp>
#include <mavros_msgs/msg/manual_control.hpp>
#include <mavros_msgs/msg/altitude.hpp>

#include <clover/srv/get_telemetry.hpp>
#include <clover/srv/navigate.hpp>
#include <clover/srv/navigate_global.hpp>
#include <clover/srv/set_altitude.hpp>
#include <clover/srv/set_yaw.hpp>
#include <clover/srv/set_yaw_rate.hpp>
#include <clover/srv/set_position.hpp>
#include <clover/srv/set_velocity.hpp>
#include <clover/srv/set_attitude.hpp>
#include <clover/srv/set_rates.hpp>
#include <clover/msg/state.hpp>

using std::string;
using std::isnan;
using namespace geometry_msgs::msg;
using namespace sensor_msgs::msg;
using namespace clover::srv;
using mavros_msgs::msg::PositionTarget;
using mavros_msgs::msg::AttitudeTarget;
using mavros_msgs::msg::Thrust;
using mavros_msgs::msg::Altitude;

rclcpp::Node::SharedPtr node;

// Guards all the state, see threading rules above
std::mutex state_mutex;

// tf2
std::shared_ptr<tf2_ros::Buffer> tf_buffer;
std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster;
std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_transform_broadcaster;

// Parameters
string mavros;
string local_frame;
string fcu_frame;
rclcpp::Duration transform_timeout(0, 0);
rclcpp::Duration telemetry_transform_timeout(0, 0);
rclcpp::Duration offboard_timeout(0, 0);
rclcpp::Duration land_timeout(0, 0);
rclcpp::Duration arming_timeout(0, 0);
rclcpp::Duration local_position_timeout(0, 0);
rclcpp::Duration state_timeout(0, 0);
rclcpp::Duration velocity_timeout(0, 0);
rclcpp::Duration global_position_timeout(0, 0);
rclcpp::Duration battery_timeout(0, 0);
rclcpp::Duration manual_control_timeout(0, 0);
float default_speed;
bool auto_release;
bool land_only_in_offboard, nav_from_sp, check_kill_switch;
std::map<string, string> reference_frames;
string terrain_frame_mode;

// Publishers
rclcpp::Publisher<PoseStamped>::SharedPtr attitude_pub, position_pub;
rclcpp::Publisher<AttitudeTarget>::SharedPtr attitude_raw_pub;
rclcpp::Publisher<PositionTarget>::SharedPtr position_raw_pub;
rclcpp::Publisher<TwistStamped>::SharedPtr rates_pub;
rclcpp::Publisher<Thrust>::SharedPtr thrust_pub;
rclcpp::Publisher<clover::msg::State>::SharedPtr state_pub;

// Service clients
rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming;
rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode;

// Containers
rclcpp::TimerBase::SharedPtr setpoint_timer;
PoseStamped position_msg;
PositionTarget position_raw_msg;
//TwistStamped rates_msg;
TransformStamped target, setpoint;
geometry_msgs::msg::TransformStamped body;
geometry_msgs::msg::TransformStamped terrain;

// State
PoseStamped nav_start;
PointStamped setpoint_position;
PointStamped setpoint_altitude;
Vector3Stamped setpoint_velocity;
float setpoint_yaw, setpoint_roll, setpoint_pitch;
Vector3 setpoint_rates;
string yaw_frame_id;
float setpoint_thrust;
float nav_speed;
float setpoint_lat = NAN, setpoint_lon = NAN;
bool busy = false;
bool wait_armed = false;
bool nav_from_sp_flag = false;

// Last published
PoseStamped setpoint_pose_local;
Vector3Stamped setpoint_velocity_local;
float yaw_local;

enum setpoint_type_t {
	NONE,
	NAVIGATE,
	NAVIGATE_GLOBAL,
	POSITION,
	VELOCITY,
	ATTITUDE,
	RATES,
	_ALTITUDE,
	_YAW,
	_YAW_RATE,
};

enum setpoint_type_t setpoint_type = NONE;

enum { YAW, YAW_RATE, TOWARDS } setpoint_yaw_type;

// Last received telemetry messages
mavros_msgs::msg::State state;
mavros_msgs::msg::StatusText statustext;
mavros_msgs::msg::ManualControl manual_control;
PoseStamped local_position;
TwistStamped velocity;
NavSatFix global_position;
BatteryState battery;

// Sleep letting other callbacks run (see threading rules above)
inline void spinSleep(rclcpp::Rate& r)
{
	state_mutex.unlock();
	r.sleep();
	state_mutex.lock();
}

inline bool isZero(const builtin_interfaces::msg::Time& stamp)
{
	return stamp.sec == 0 && stamp.nanosec == 0;
}

inline Quaternion createQuaternionMsgFromRollPitchYaw(double roll, double pitch, double yaw)
{
	tf2::Quaternion q;
	q.setRPY(roll, pitch, yaw);
	return tf2::toMsg(q);
}

inline Quaternion createQuaternionMsgFromYaw(double yaw)
{
	return createQuaternionMsgFromRollPitchYaw(0, 0, yaw);
}

inline double getYaw(const Quaternion& q_msg)
{
	tf2::Quaternion q;
	tf2::fromMsg(q_msg, q);
	double roll, pitch, yaw;
	tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
	return yaw;
}

// Common subscriber callback template that stores message to the variable
template<typename T, T& STORAGE>
void handleMessage(const T& msg)
{
	std::lock_guard<std::mutex> lock(state_mutex);
	STORAGE = msg;
}

void handleState(const mavros_msgs::msg::State& s)
{
	std::lock_guard<std::mutex> lock(state_mutex);
	state = s;
	if (s.mode != "OFFBOARD") {
		// flight intercepted
		nav_from_sp_flag = false;
	}
}

inline void publishBodyFrame()
{
	if (body.child_frame_id.empty()) return;
	if (!isZero(body.header.stamp) && body.header.stamp == local_position.header.stamp) {
		return; // avoid TF_REPEATED_DATA warnings
	}

	tf2::Quaternion q;
	q.setRPY(0, 0, getYaw(local_position.pose.orientation));
	body.transform.rotation = tf2::toMsg(q);

	body.transform.translation.x = local_position.pose.position.x;
	body.transform.translation.y = local_position.pose.position.y;
	body.transform.translation.z = local_position.pose.position.z;
	body.header.frame_id = local_position.header.frame_id;
	body.header.stamp = local_position.header.stamp;
	transform_broadcaster->sendTransform(body);
}

void handleLocalPosition(const PoseStamped& pose)
{
	std::lock_guard<std::mutex> lock(state_mutex);
	local_position = pose;
	publishBodyFrame();
	// TODO: home?
}

// wait for transform without interrupting publishing setpoints
inline bool waitTransform(const string& target, const string& source,
                          const rclcpp::Time& stamp, const rclcpp::Duration& timeout) // editorconfig-checker-disable-line
{
	rclcpp::Rate r(100, node->get_clock());
	auto start = node->now();
	while (rclcpp::ok()) {
		if (node->now() - start > timeout) return false;
		if (tf_buffer->canTransform(target, source, stamp)) return true;
		spinSleep(r);
	}
	return false;
}

// call mavros service without blocking other callbacks, returns nullptr on failure
template<typename T>
typename T::Response::SharedPtr callService(const typename rclcpp::Client<T>::SharedPtr& client,
                                            const typename T::Request::SharedPtr& request, // editorconfig-checker-disable-line
                                            const rclcpp::Time& deadline) // editorconfig-checker-disable-line
{
	if (!client->service_is_ready()) return nullptr;

	auto future = client->async_send_request(request);
	rclcpp::Rate r(100, node->get_clock());
	while (rclcpp::ok()) {
		if (future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
			return future.get();
		}
		if (node->now() > deadline) break;
		spinSleep(r);
	}
	client->remove_pending_request(future);
	return nullptr;
}

void publishTerrain(const double distance, const rclcpp::Time& stamp)
{
	if (!waitTransform(local_frame, body.child_frame_id, stamp, rclcpp::Duration::from_seconds(0.1))) return;

	auto t = tf_buffer->lookupTransform(local_frame, body.child_frame_id, stamp);
	t.child_frame_id = terrain.child_frame_id;
	t.transform.translation.z -= distance;
	static_transform_broadcaster->sendTransform(t);
}

void handleAltitude(const Altitude& alt)
{
	std::lock_guard<std::mutex> lock(state_mutex);
	if (!std::isfinite(alt.bottom_clearance)) return;
	publishTerrain(alt.bottom_clearance, alt.header.stamp);
}

void handleRange(const Range& range)
{
	std::lock_guard<std::mutex> lock(state_mutex);
	if (!std::isfinite(range.range)) return;
	// TODO: check it's facing down
	publishTerrain(range.range, range.header.stamp);
}

#define TIMEOUT(msg, timeout) (isZero(msg.header.stamp) || (node->now() - rclcpp::Time(msg.header.stamp) > timeout))

void getTelemetry(std::shared_ptr<GetTelemetry::Request> req, std::shared_ptr<GetTelemetry::Response> res)
{
	std::lock_guard<std::mutex> lock(state_mutex);
	rclcpp::Time stamp = node->now();

	if (req->frame_id.empty())
		req->frame_id = local_frame;

	res->frame_id = req->frame_id;
	res->x = NAN;
	res->y = NAN;
	res->z = NAN;
	res->lat = NAN;
	res->lon = NAN;
	res->alt = NAN;
	res->vx = NAN;
	res->vy = NAN;
	res->vz = NAN;
	res->roll = NAN;
	res->pitch = NAN;
	res->yaw = NAN;
	res->roll_rate = NAN;
	res->pitch_rate = NAN;
	res->yaw_rate = NAN;
	res->voltage = NAN;
	res->cell_voltage = NAN;

	if (!TIMEOUT(state, state_timeout)) {
		res->connected = state.connected;
		res->armed = state.armed;
		res->mode = state.mode;
	}

	try {
		waitTransform(req->frame_id, fcu_frame, stamp, telemetry_transform_timeout);
		auto transform = tf_buffer->lookupTransform(req->frame_id, fcu_frame, stamp);
		res->x = transform.transform.translation.x;
		res->y = transform.transform.translation.y;
		res->z = transform.transform.translation.z;

		double yaw, pitch, roll;
		tf2::getEulerYPR(transform.transform.rotation, yaw, pitch, roll);
		res->yaw = yaw;
		res->pitch = pitch;
		res->roll = roll;
	} catch (const tf2::TransformException& e) {
		RCLCPP_DEBUG(node->get_logger(), "%s", e.what());
	}

	if (!TIMEOUT(velocity, velocity_timeout)) {
		try {
			// transform velocity
			waitTransform(req->frame_id, fcu_frame, velocity.header.stamp, telemetry_transform_timeout);
			Vector3Stamped vec, vec_out;
			vec.header.stamp = velocity.header.stamp;
			vec.header.frame_id = velocity.header.frame_id;
			vec.vector = velocity.twist.linear;
			tf_buffer->transform(vec, vec_out, req->frame_id);

			res->vx = vec_out.vector.x;
			res->vy = vec_out.vector.y;
			res->vz = vec_out.vector.z;
		} catch (const tf2::TransformException& e) {}

		// use angular velocities as they are
		res->yaw_rate = velocity.twist.angular.z;
		res->pitch_rate = velocity.twist.angular.y;
		res->roll_rate = velocity.twist.angular.x;
	}

	if (!TIMEOUT(global_position, global_position_timeout)) {
		res->lat = global_position.latitude;
		res->lon = global_position.longitude;
		res->alt = global_position.altitude;
	}

	if (!TIMEOUT(battery, battery_timeout)) {
		res->voltage = battery.voltage;
		if (!battery.cell_voltage.empty()) {
			res->cell_voltage = battery.cell_voltage[0];
		}
	}
}

// throws std::runtime_error
void offboardAndArm()
{
	rclcpp::Rate r(10, node->get_clock());

	if (state.mode != "OFFBOARD") {
		auto start = node->now();
		RCLCPP_INFO(node->get_logger(), "switch to OFFBOARD");
		auto sm = std::make_shared<mavros_msgs::srv::SetMode::Request>();
		sm->custom_mode = "OFFBOARD";

		if (!callService<mavros_msgs::srv::SetMode>(set_mode, sm, start + offboard_timeout))
			throw std::runtime_error("Error calling set_mode service");

		// wait for OFFBOARD mode
		while (rclcpp::ok()) {
			if (state.mode == "OFFBOARD") {
				break;
			} else if (node->now() - start > offboard_timeout) {
				string report = "OFFBOARD timed out";
				if (rclcpp::Time(statustext.header.stamp) > start)
					report += ": " + statustext.text;
				throw std::runtime_error(report);
			}
			spinSleep(r);
		}
	}

	if (!state.armed) {
		rclcpp::Time start = node->now();
		RCLCPP_INFO(node->get_logger(), "arming");
		auto srv = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
		srv->value = true;
		if (!callService<mavros_msgs::srv::CommandBool>(arming, srv, start + arming_timeout)) {
			throw std::runtime_error("Error calling arming service");
		}

		// wait until armed
		while (rclcpp::ok()) {
			if (state.armed) {
				break;
			} else if (node->now() - start > arming_timeout) {
				string report = "Arming timed out";
				if (rclcpp::Time(statustext.header.stamp) > start)
					report += ": " + statustext.text;
				throw std::runtime_error(report);
			}
			spinSleep(r);
		}
	}
}

inline double hypot3(double x, double y, double z)
{
	return std::sqrt(x * x + y * y + z * z);
}

inline float getDistance(const Point& from, const Point& to)
{
	return hypot3(from.x - to.x, from.y - to.y, from.z - to.z);
}

void getNavigateSetpoint(const rclcpp::Time& stamp, const float speed, Point& nav_setpoint)
{
	if (wait_armed) {
		// don't start navigating if we're waiting arming
		nav_start.header.stamp = stamp;
	}

	float distance = getDistance(nav_start.pose.position, setpoint_pose_local.pose.position);
	float time = distance / speed;
	float passed = std::min((stamp - rclcpp::Time(nav_start.header.stamp)).seconds() / time, 1.0);

	nav_setpoint.x = nav_start.pose.position.x + (setpoint_pose_local.pose.position.x - nav_start.pose.position.x) * passed;
	nav_setpoint.y = nav_start.pose.position.y + (setpoint_pose_local.pose.position.y - nav_start.pose.position.y) * passed;
	nav_setpoint.z = nav_start.pose.position.z + (setpoint_pose_local.pose.position.z - nav_start.pose.position.z) * passed;
}

PoseStamped globalToLocal(double lat, double lon)
{
	auto earth = GeographicLib::Geodesic::WGS84();

	// Determine azimuth and distance between current and destination point
	double _, distance, azimuth;
	earth.Inverse(global_position.latitude, global_position.longitude, lat, lon, distance, _, azimuth);

	double x_offset, y_offset;
	double azimuth_radians = azimuth * M_PI / 180;
	x_offset = distance * sin(azimuth_radians);
	y_offset = distance * cos(azimuth_radians);

	if (!waitTransform(local_frame, fcu_frame, global_position.header.stamp, rclcpp::Duration::from_seconds(0.2))) {
		throw std::runtime_error("No local position");
	}

	auto local = tf_buffer->lookupTransform(local_frame, fcu_frame, global_position.header.stamp);

	PoseStamped pose;
	pose.header.stamp = global_position.header.stamp; // TODO: ?
	pose.header.frame_id = local_frame;
	pose.pose.position.x = local.transform.translation.x + x_offset;
	pose.pose.position.y = local.transform.translation.y + y_offset;
	pose.pose.orientation.w = 1;
	return pose;
}

// publish navigate_target frame
void publishTarget(rclcpp::Time stamp, bool _static = false)
{
	bool single_frame = (setpoint_position.header.frame_id == setpoint_altitude.header.frame_id);

	// handle yaw for target frame
	if (setpoint_yaw_type == YAW || setpoint_yaw_type == YAW_RATE) { // use last set yaw for yaw_rate
		if (setpoint_altitude.header.frame_id == yaw_frame_id) {
			target.transform.rotation = createQuaternionMsgFromYaw(setpoint_yaw);
		} else {
			single_frame = false;
			target.transform.rotation = createQuaternionMsgFromYaw(yaw_local);
		}
	} else if (setpoint_yaw_type == TOWARDS) {
		single_frame = false;
		target.transform.rotation = createQuaternionMsgFromYaw(yaw_local);
	}

	if (_static && single_frame) {
		// publish at user's command, if all frames are the same
		target.header.frame_id = setpoint_position.header.frame_id;
		target.header.stamp = stamp;
		target.transform.translation.x = setpoint_position.point.x;
		target.transform.translation.y = setpoint_position.point.y;
		target.transform.translation.z = setpoint_position.point.z;

	} else if (!_static) {
		// publish at each iteration, if frames are different
		target.header = setpoint_pose_local.header;
		target.transform.translation.x = setpoint_pose_local.pose.position.x;
		target.transform.translation.y = setpoint_pose_local.pose.position.y;
		target.transform.translation.z = setpoint_pose_local.pose.position.z;
	}

	static_transform_broadcaster->sendTransform(target);
}

void publish(const rclcpp::Time stamp)
{
	if (setpoint_type == NONE) return;

	position_raw_msg.header.stamp = stamp;

	// transform position
	if (setpoint_type == NAVIGATE || setpoint_type == NAVIGATE_GLOBAL || setpoint_type == POSITION) {
		setpoint_position.header.stamp = stamp;
		setpoint_altitude.header.stamp = stamp;
		// transform xy
		try {
			auto xy = tf_buffer->transform(setpoint_position, local_frame, tf2::durationFromSec(0.05));
			setpoint_pose_local.header = xy.header;
			setpoint_pose_local.pose.position.x = xy.point.x;
			setpoint_pose_local.pose.position.y = xy.point.y;
		} catch (tf2::TransformException& ex) {
			// can't transform xy, use last known
			RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 10000, "can't transform: %s", ex.what());
		}
		// transform altitude
		try {
			setpoint_pose_local.pose.position.z = tf_buffer->transform(setpoint_altitude, local_frame, tf2::durationFromSec(0.05)).point.z;
		} catch (tf2::TransformException& ex) {
			// can't transform altitude, use last known
			RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 10000, "can't transform: %s", ex.what());
		}
	}

	// transform yaw
	if (setpoint_yaw_type == YAW) {
		try {
			QuaternionStamped q;
			q.header.stamp = stamp;
			q.header.frame_id = yaw_frame_id;
			q.quaternion = createQuaternionMsgFromYaw(setpoint_yaw);
			yaw_local = tf2::getYaw(tf_buffer->transform(q, local_frame, tf2::durationFromSec(0.05)).quaternion);
		} catch (tf2::TransformException& ex) {
			// can't transform yaw, use last known
			RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 10000, "can't transform: %s", ex.what());
		}
	}

	// compute navigate setpoint
	if (setpoint_type == NAVIGATE || setpoint_type == NAVIGATE_GLOBAL) {
		getNavigateSetpoint(stamp, nav_speed, position_msg.pose.position);

		if (setpoint_yaw_type == TOWARDS) {
			yaw_local = atan2(position_msg.pose.position.y - nav_start.pose.position.y,
			                  position_msg.pose.position.x - nav_start.pose.position.x);
		}

		position_msg.pose.orientation = createQuaternionMsgFromYaw(yaw_local);
	}

	if (setpoint_type == POSITION) {
		position_msg = setpoint_pose_local;
		position_msg.pose.orientation = createQuaternionMsgFromYaw(yaw_local);
	}

	if (setpoint_type == POSITION || setpoint_type == NAVIGATE || setpoint_type == NAVIGATE_GLOBAL) {
		position_msg.header.stamp = stamp;

		if (setpoint_yaw_type == YAW || setpoint_yaw_type == TOWARDS) {
			position_pub->publish(position_msg);

		} else {
			position_raw_msg.type_mask = PositionTarget::IGNORE_VX +
			                             PositionTarget::IGNORE_VY +
			                             PositionTarget::IGNORE_VZ +
			                             PositionTarget::IGNORE_AFX +
			                             PositionTarget::IGNORE_AFY +
			                             PositionTarget::IGNORE_AFZ +
			                             PositionTarget::IGNORE_YAW;
			position_raw_msg.yaw_rate = setpoint_rates.z;
			position_raw_msg.position = position_msg.pose.position;
			position_raw_pub->publish(position_raw_msg);
		}

		// publish setpoint frame
		if (!setpoint.child_frame_id.empty()) {
			if (rclcpp::Time(setpoint.header.stamp) >= rclcpp::Time(position_msg.header.stamp)) {
				return; // avoid TF_REPEATED_DATA warnings
			}

			setpoint.transform.translation.x = position_msg.pose.position.x;
			setpoint.transform.translation.y = position_msg.pose.position.y;
			setpoint.transform.translation.z = position_msg.pose.position.z;
			setpoint.transform.rotation = position_msg.pose.orientation;
			setpoint.header.frame_id = position_msg.header.frame_id;
			setpoint.header.stamp = position_msg.header.stamp;
			transform_broadcaster->sendTransform(setpoint);
		}

		// publish dynamic target frame
		publishTarget(stamp);
	}

	if (setpoint_type == VELOCITY) {
		// transform velocity to local frame
		setpoint_velocity.header.stamp = stamp;
		try {
			setpoint_velocity_local = tf_buffer->transform(setpoint_velocity, local_frame, tf2::durationFromSec(0.05));
		} catch (tf2::TransformException& ex) {
			// can't transform velocity, use last known
			RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 10000, "can't transform: %s", ex.what());
		}

		// publish velocity
		position_raw_msg.type_mask = PositionTarget::IGNORE_PX +
		                             PositionTarget::IGNORE_PY +
		                             PositionTarget::IGNORE_PZ +
		                             PositionTarget::IGNORE_AFX +
		                             PositionTarget::IGNORE_AFY +
		                             PositionTarget::IGNORE_AFZ;
		position_raw_msg.type_mask += setpoint_yaw_type == YAW ? PositionTarget::IGNORE_YAW_RATE : PositionTarget::IGNORE_YAW;
		position_raw_msg.velocity = setpoint_velocity_local.vector;
		position_raw_msg.yaw = yaw_local;
		position_raw_msg.yaw_rate = setpoint_rates.z;
		position_raw_pub->publish(position_raw_msg);
	}

	if (setpoint_type == ATTITUDE) {
		PoseStamped msg;
		msg.header.stamp = stamp;
		msg.header.frame_id = local_frame;
		msg.pose.orientation = createQuaternionMsgFromRollPitchYaw(setpoint_roll, setpoint_pitch, yaw_local);
		attitude_pub->publish(msg);

		Thrust thrust_msg;
		thrust_msg.header.stamp = stamp;
		thrust_msg.thrust = setpoint_thrust;
		thrust_pub->publish(thrust_msg);
	}

	if (setpoint_type == RATES) {
		// rates_pub.publish(rates_msg);
		// thrust_pub.publish(thrust_msg);
		// mavros rates topics waits for rates in local frame
		// use rates in body frame for simplicity
		AttitudeTarget att_raw_msg;
		att_raw_msg.header.stamp = stamp;
		att_raw_msg.header.frame_id = fcu_frame;
		att_raw_msg.type_mask = AttitudeTarget::IGNORE_ATTITUDE;
		att_raw_msg.body_rate = setpoint_rates;
		att_raw_msg.thrust = setpoint_thrust;
		attitude_raw_pub->publish(att_raw_msg);
	}
}

void publishSetpoint()
{
	std::lock_guard<std::mutex> lock(state_mutex);
	publish(node->now());
}

// start publishing setpoints, does nothing if already started
inline void startSetpointTimer()
{
	if (setpoint_timer->is_canceled()) setpoint_timer->reset();
}

inline void checkManualControl()
{
	if (manual_control_timeout.nanoseconds() != 0 && TIMEOUT(manual_control, manual_control_timeout)) {
		throw std::runtime_error("Manual control timeout, RC is switched off?");
	}

	if (check_kill_switch) {
		// switch values: https://github.com/PX4/PX4-Autopilot/blob/c302514a0809b1765fafd13c014d705446ae1113/msg/manual_control_setpoint.msg#L3
		const uint8_t SWITCH_POS_NONE = 0; // switch is not mapped
		const uint8_t SWITCH_POS_ON = 1; // switch activated
		const uint8_t SWITCH_POS_MIDDLE = 2; // middle position
		const uint8_t SWITCH_POS_OFF = 3; // switch not activated
		(void)SWITCH_POS_NONE;
		(void)SWITCH_POS_MIDDLE;
		(void)SWITCH_POS_OFF;

		const int KILL_SWITCH_BIT = 12; // https://github.com/PX4/Firmware/blob/c302514a0809b1765fafd13c014d705446ae1113/src/modules/mavlink/mavlink_messages.cpp#L3975
		uint8_t kill_switch = (manual_control.buttons & (0b11 << KILL_SWITCH_BIT)) >> KILL_SWITCH_BIT;

		if (kill_switch == SWITCH_POS_ON)
			throw std::runtime_error("Kill switch is on");
	}
}

inline void checkState()
{
	if (TIMEOUT(state, state_timeout))
		throw std::runtime_error("State timeout, check mavros settings");

	if (!state.connected)
		throw std::runtime_error("No connection to FCU, https://clover.coex.tech/connection");
}

void publishState()
{
	clover::msg::State msg;
	msg.mode = setpoint_type;
	msg.yaw_mode = setpoint_yaw_type;

	if (setpoint_position.header.frame_id.empty()) {
		msg.x = NAN;
		msg.y = NAN;
		msg.z = NAN;
	} else {
		msg.x = setpoint_position.point.x;
		msg.y = setpoint_position.point.y;
		msg.z = setpoint_altitude.point.z;
	}

	msg.speed = nav_speed;
	msg.lat = setpoint_lat;
	msg.lon = setpoint_lon;
	msg.vx = setpoint_velocity.vector.x;
	msg.vy = setpoint_velocity.vector.y;
	msg.vz = setpoint_velocity.vector.z;
	msg.roll = setpoint_roll;
	msg.pitch = setpoint_pitch;
	msg.yaw = !yaw_frame_id.empty() ? setpoint_yaw : NAN;

	msg.roll_rate = setpoint_rates.x;
	msg.pitch_rate = setpoint_rates.y;
	msg.yaw_rate = setpoint_rates.z;
	msg.thrust = setpoint_thrust;

	if (setpoint_type == VELOCITY) {
		msg.xy_frame_id = setpoint_velocity.header.frame_id;
		msg.z_frame_id = setpoint_velocity.header.frame_id;
	} else {
		msg.xy_frame_id = setpoint_position.header.frame_id;
		msg.z_frame_id = setpoint_altitude.header.frame_id;
	}
	msg.yaw_frame_id = yaw_frame_id;

	state_pub->publish(msg);
}

inline float safe(float value) {
	return std::isfinite(value) ? value : 0;
}

#define ENSURE_FINITE(var) { if (!std::isfinite(var)) throw std::runtime_error(#var " argument cannot be NaN or Inf"); }

#define ENSURE_NON_INF(var) { if (std::isinf(var)) throw std::runtime_error(#var " argument cannot be Inf"); }

bool serve(enum setpoint_type_t sp_type, float x, float y, float z, float vx, float vy, float vz,
           float roll, float pitch, float yaw, float roll_rate, float pitch_rate, float yaw_rate, // editorconfig-checker-disable-line
           float lat, float lon, float thrust, float speed, string frame_id, bool auto_arm, // editorconfig-checker-disable-line
           bool& success, string& message) // editorconfig-checker-disable-line
{
	std::lock_guard<std::mutex> lock(state_mutex);
	auto stamp = node->now();

	try {
		if (busy)
			throw std::runtime_error("Busy");

		busy = true;

		// Checks
		checkState();

		if (auto_arm) {
			checkManualControl();
		}

		// default frame is local frame
		if (frame_id.empty())
			frame_id = local_frame;

		// look up for reference frame
		auto search = reference_frames.find(frame_id);
		const string& reference_frame = search == reference_frames.end() ? frame_id : search->second;

		ENSURE_NON_INF(x);
		ENSURE_NON_INF(y);
		ENSURE_NON_INF(z);
		ENSURE_NON_INF(speed); // TODO: allow inf
		ENSURE_NON_INF(vx);
		ENSURE_NON_INF(vy);
		ENSURE_NON_INF(vz);
		ENSURE_NON_INF(roll);
		ENSURE_NON_INF(pitch);
		ENSURE_NON_INF(roll_rate);
		ENSURE_NON_INF(pitch_rate);
		ENSURE_NON_INF(yaw_rate);
		ENSURE_NON_INF(thrust);

		if (sp_type == NAVIGATE_GLOBAL) {
			ENSURE_FINITE(lat);
			ENSURE_FINITE(lon);
		}

		if (std::isfinite(x) != std::isfinite(y)) {
			throw std::runtime_error("x and y can be set only together");
		}

		if (std::isfinite(yaw_rate)) {
			if (sp_type > RATES && setpoint_type == ATTITUDE) {
				throw std::runtime_error("Yaw rate cannot be set in attitude mode.");
			}
		}

		// set_altitude
		if (sp_type == _ALTITUDE) {
			if (setpoint_type == VELOCITY || setpoint_type == ATTITUDE || setpoint_type == RATES) {
				throw std::runtime_error("Altitude cannot be set in velocity, attitude or rates mode.");
			}
		}

		if (sp_type == NAVIGATE || sp_type == NAVIGATE_GLOBAL) {
			if (TIMEOUT(local_position, local_position_timeout))
				throw std::runtime_error("No local position, check settings");

			if (speed < 0)
				throw std::runtime_error("Navigate speed must be positive, " + std::to_string(speed) + " passed");

			if (speed == 0)
				speed = default_speed;
		}

		if (sp_type == NAVIGATE_GLOBAL) {
			if (TIMEOUT(global_position, global_position_timeout))
				throw std::runtime_error("No global position");
		}

		// if any value need to be transformed to reference frame
		if (std::isfinite(x) || std::isfinite(y) || std::isfinite(z) || std::isfinite(vx) || std::isfinite(vy) || std::isfinite(vz) || std::isfinite(yaw)) {
			// make sure transform from frame_id to reference frame available
			if (!waitTransform(reference_frame, frame_id, stamp, transform_timeout))
				throw std::runtime_error("Can't transform from " + frame_id + " to " + reference_frame);

			// make sure transform from reference frame to local frame available
			if (!waitTransform(local_frame, reference_frame, stamp, transform_timeout))
				throw std::runtime_error("Can't transform from " + reference_frame + " to " + local_frame);
		}

		if (sp_type == NAVIGATE_GLOBAL) {
			// Calculate x and from lat and lot in request's frame
			auto pose_local = globalToLocal(lat, lon);
			pose_local.header.stamp = stamp; // TODO: fix
			auto xy_in_req_frame = tf_buffer->transform(pose_local, frame_id);
			x = xy_in_req_frame.pose.position.x;
			y = xy_in_req_frame.pose.position.y;
			setpoint_lat = lat;
			setpoint_lon = lon;
		}

		// Everything fine - switch setpoint type
		if (sp_type <= RATES) {
			setpoint_type = sp_type;
		}

		if (setpoint_type != NAVIGATE && setpoint_type != NAVIGATE_GLOBAL) {
			nav_from_sp_flag = false;
		}

		bool to_auto_arm = auto_arm && (state.mode != "OFFBOARD" || !state.armed);
		if (to_auto_arm || setpoint_type == VELOCITY || setpoint_type == ATTITUDE || setpoint_type == RATES) {
			// invalidate position setpoint
			setpoint_position.header.frame_id = "";
			setpoint_altitude.header.frame_id = "";
			yaw_frame_id = "";
		}

		if (sp_type == NAVIGATE || sp_type == NAVIGATE_GLOBAL) {
			// starting point
			if (nav_from_sp && nav_from_sp_flag) {
				message = "Navigating from current setpoint";
				nav_start = position_msg;
			} else {
				nav_start = local_position;
			}

			if (!isnan(speed)) {
				nav_speed = speed;
			}

			nav_from_sp_flag = true;
		}

		// handle position
		if (setpoint_type == NAVIGATE || setpoint_type == NAVIGATE_GLOBAL || setpoint_type == POSITION) {

			PointStamped desired;
			desired.header.frame_id = frame_id;
			desired.header.stamp = stamp;
			desired.point.x = safe(x);
			desired.point.y = safe(y);
			desired.point.z = safe(z);

			// transform to reference frame
			desired = tf_buffer->transform(desired, reference_frame);

			// set horizontal position
			if (std::isfinite(x) && std::isfinite(y)) {
				setpoint_position = desired;
			} else if (setpoint_position.header.frame_id.empty()) {
				 // TODO: use transform for current stamp
				setpoint_position.header = local_position.header;
				setpoint_position.point = local_position.pose.position;
			}

			// set altitude
			if (std::isfinite(z)) {
				setpoint_altitude = desired;
			} else if (setpoint_altitude.header.frame_id.empty()) {
				setpoint_altitude.header = local_position.header;
				setpoint_altitude.point = local_position.pose.position;
			}
		}

		// handle velocity
		if (sp_type == VELOCITY) {
			// TODO: allow setting different modes by altitude and xy
			Vector3Stamped desired;
			desired.header.frame_id = frame_id;
			desired.header.stamp = stamp;
			desired.vector.x = safe(vx);
			desired.vector.y = safe(vy);
			desired.vector.z = safe(vz);

			// transform to reference frame
			desired = tf_buffer->transform(desired, reference_frame);
			setpoint_velocity.header = desired.header;

			// set horizontal velocity
			if (std::isfinite(vx) && std::isfinite(vy)) {
				setpoint_velocity.vector.x = desired.vector.x;
				setpoint_velocity.vector.y = desired.vector.y;
			}

			// set vertical velocity
			if (std::isfinite(vz)) {
				setpoint_velocity.vector.z = desired.vector.z;
			}
		}

		// handle yaw
		if (sp_type == NAVIGATE || sp_type == NAVIGATE_GLOBAL || sp_type == POSITION || sp_type == VELOCITY || sp_type == ATTITUDE || sp_type == _YAW) {
			if (std::isfinite(yaw)) {
				setpoint_yaw_type = YAW;
				QuaternionStamped desired;
				desired.header.frame_id = frame_id;
				desired.header.stamp = stamp;
				desired.quaternion = createQuaternionMsgFromYaw(yaw);

				// transform to reference frame
				desired = tf_buffer->transform(desired, reference_frame);
				setpoint_yaw = tf2::getYaw(desired.quaternion);
				yaw_frame_id = reference_frame;

			} else if (std::isinf(yaw) && yaw > 0) {
				// yaw towards
				setpoint_yaw_type = TOWARDS;

			} else if (yaw_frame_id.empty() || sp_type == _YAW) {
				// yaw is nan and not set previously OR set_yaw(yaw=nan) was called
				setpoint_yaw_type = YAW;
				setpoint_yaw = tf2::getYaw(local_position.pose.orientation); // set yaw to current yaw
				yaw_frame_id = local_position.header.frame_id;
			}
		}

		// handle roll
		if (std::isfinite(roll)) {
			setpoint_roll = roll;
		}

		// handle pitch
		if (std::isfinite(pitch)) {
			setpoint_pitch = pitch;
		}

		// handle yaw rate
		if (std::isfinite(yaw_rate)) {
			setpoint_yaw_type = YAW_RATE;
			setpoint_rates.z = yaw_rate;
		}

		// handle pitch rate
		if (std::isfinite(roll_rate)) {
			setpoint_rates.x = roll_rate;
		}

		// handle roll rate
		if (std::isfinite(pitch_rate)) {
			setpoint_rates.y = pitch_rate;
		}

		// handle thrust
		if (std::isfinite(thrust)) {
			setpoint_thrust = thrust;
		}

		wait_armed = auto_arm;

		publish(stamp); // calculate initial transformed messages first
		startSetpointTimer();

		if (setpoint_type == NAVIGATE || setpoint_type == NAVIGATE_GLOBAL || setpoint_type == POSITION) {
			publishTarget(stamp, true);
		}

		publishState();

		if (auto_arm) {
			offboardAndArm();
			wait_armed = false;
		} else if (state.mode != "OFFBOARD") {
			setpoint_timer->cancel();
			throw std::runtime_error("Copter is not in OFFBOARD mode, use auto_arm?");
		} else if (!state.armed) {
			setpoint_timer->cancel();
			throw std::runtime_error("Copter is not armed, use auto_arm?");
		}

	} catch (const std::exception& e) {
		message = e.what();
		RCLCPP_INFO(node->get_logger(), "%s", message.c_str());
		busy = false;
		return true;
	}

	success = true;
	busy = false;
	return true;
}

void navigate(const std::shared_ptr<Navigate::Request> req, std::shared_ptr<Navigate::Response> res) {
	serve(NAVIGATE, req->x, req->y, req->z, NAN, NAN, NAN, NAN, NAN, req->yaw, NAN, NAN, NAN, NAN, NAN, NAN, req->speed, req->frame_id, req->auto_arm, res->success, res->message);
}

void navigateGlobal(const std::shared_ptr<NavigateGlobal::Request> req, std::shared_ptr<NavigateGlobal::Response> res) {
	serve(NAVIGATE_GLOBAL, NAN, NAN, req->z, NAN, NAN, NAN, NAN, NAN, req->yaw, NAN, NAN, NAN, req->lat, req->lon, NAN, req->speed, req->frame_id, req->auto_arm, res->success, res->message);
}

void setAltitude(const std::shared_ptr<SetAltitude::Request> req, std::shared_ptr<SetAltitude::Response> res) {
	serve(_ALTITUDE, NAN, NAN, req->z, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, req->frame_id, false, res->success, res->message);
}

void setYaw(const std::shared_ptr<SetYaw::Request> req, std::shared_ptr<SetYaw::Response> res) {
	serve(_YAW, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, req->yaw, NAN, NAN, NAN, NAN, NAN, NAN, NAN, req->frame_id, false, res->success, res->message);
}

void setYawRate(const std::shared_ptr<SetYawRate::Request> req, std::shared_ptr<SetYawRate::Response> res) {
	serve(_YAW_RATE, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, req->yaw_rate, NAN, NAN, NAN, NAN, "", false, res->success, res->message);
}

void setPosition(const std::shared_ptr<SetPosition::Request> req, std::shared_ptr<SetPosition::Response> res) {
	serve(POSITION, req->x, req->y, req->z, NAN, NAN, NAN, NAN, NAN, req->yaw, NAN, NAN, NAN, NAN, NAN, NAN, NAN, req->frame_id, req->auto_arm, res->success, res->message);
}

void setVelocity(const std::shared_ptr<SetVelocity::Request> req, std::shared_ptr<SetVelocity::Response> res) {
	serve(VELOCITY, NAN, NAN, NAN, req->vx, req->vy, req->vz, NAN, NAN, req->yaw, NAN, NAN, NAN, NAN, NAN, NAN, NAN, req->frame_id, req->auto_arm, res->success, res->message);
}

void setAttitude(const std::shared_ptr<SetAttitude::Request> req, std::shared_ptr<SetAttitude::Response> res) {
	serve(ATTITUDE, NAN, NAN, NAN, NAN, NAN, NAN, req->roll, req->pitch, req->yaw, NAN, NAN, NAN, NAN, NAN, req->thrust, NAN, req->frame_id, req->auto_arm, res->success, res->message);
}

void setRates(const std::shared_ptr<SetRates::Request> req, std::shared_ptr<SetRates::Response> res) {
	serve(RATES, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, req->roll_rate, req->pitch_rate, req->yaw_rate, NAN, NAN, req->thrust, NAN, "", req->auto_arm, res->success, res->message);
}

void land(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
	std::lock_guard<std::mutex> lock(state_mutex);

	try {
		if (busy)
			throw std::runtime_error("Busy");

		busy = true;

		checkState();

		if (land_only_in_offboard) {
			if (state.mode != "OFFBOARD") {
				throw std::runtime_error("Copter is not in OFFBOARD mode");
			}
		}

		auto sm = std::make_shared<mavros_msgs::srv::SetMode::Request>();
		sm->custom_mode = "AUTO.LAND";

		auto sm_response = callService<mavros_msgs::srv::SetMode>(set_mode, sm, node->now() + land_timeout);
		if (!sm_response)
			throw std::runtime_error("Can't call set_mode service");

		if (!sm_response->mode_sent)
			throw std::runtime_error("Can't send set_mode request");

		rclcpp::Rate r(10, node->get_clock());
		auto start = node->now();
		while (rclcpp::ok()) {
			if (state.mode == "AUTO.LAND") {
				break;
			}
			if (node->now() - start > land_timeout)
				throw std::runtime_error("Land request timed out");

			spinSleep(r);
		}

		// stop setpoints and invalidate position setpoint
		setpoint_timer->cancel();
		setpoint_type = NONE;
		setpoint_position.header.frame_id = "";
		setpoint_altitude.header.frame_id = "";
		yaw_frame_id = "";
		publishState();

		res->success = true;
		busy = false;

	} catch (const std::exception& e) {
		res->message = e.what();
		RCLCPP_INFO(node->get_logger(), "%s", e.what());
		busy = false;
	}
}

void release(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
	std::lock_guard<std::mutex> lock(state_mutex);
	setpoint_timer->cancel();
	setpoint_type = NONE;
	setpoint_position.header.frame_id = "";
	setpoint_altitude.header.frame_id = "";
	yaw_frame_id = "";
	publishState();
	res->success = true;
}

// Read frames from the parameters of mavros local_position plugin node
void readMavrosFrames()
{
	local_frame = "map";
	fcu_frame = "base_link";

	const string plugin = mavros + "/local_position";
	auto params = std::make_shared<rclcpp::SyncParametersClient>(node, plugin);
	if (!params->wait_for_service(std::chrono::seconds(5))) {
		RCLCPP_WARN(node->get_logger(), "can't read frames from %s, using %s and %s",
		            plugin.c_str(), local_frame.c_str(), fcu_frame.c_str()); // editorconfig-checker-disable-line
		return;
	}

	for (auto& param : params->get_parameters({"tf.frame_id", "tf.child_frame_id"}, std::chrono::seconds(5))) {
		if (param.get_type() != rclcpp::ParameterType::PARAMETER_STRING) continue;
		if (param.get_name() == "tf.frame_id") local_frame = param.as_string();
		if (param.get_name() == "tf.child_frame_id") fcu_frame = param.as_string();
	}
}

inline rclcpp::Duration durationParam(const string& name, double default_value)
{
	return rclcpp::Duration::from_seconds(node->declare_parameter(name, default_value));
}

// create all the node's entities and spin
void run()
{
	tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
	tf2_ros::TransformListener tf_listener(*tf_buffer, node, true);
	transform_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);
	static_transform_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);

	// Params
	mavros = node->declare_parameter("mavros", string("mavros")); // for case of using multiple connections
	local_frame = node->declare_parameter("local_frame", string("")); // read from mavros if empty
	fcu_frame = node->declare_parameter("fcu_frame", string("")); // read from mavros if empty
	if (local_frame.empty() || fcu_frame.empty()) {
		string local_frame_param = local_frame, fcu_frame_param = fcu_frame;
		readMavrosFrames();
		if (!local_frame_param.empty()) local_frame = local_frame_param;
		if (!fcu_frame_param.empty()) fcu_frame = fcu_frame_param;
	}
	target.child_frame_id = node->declare_parameter("target_frame", string("navigate_target"));
	setpoint.child_frame_id = node->declare_parameter("setpoint", string("setpoint"));
	auto_release = node->declare_parameter("auto_release", true);
	land_only_in_offboard = node->declare_parameter("land_only_in_offboard", true);
	nav_from_sp = node->declare_parameter("nav_from_sp", true);
	check_kill_switch = node->declare_parameter("check_kill_switch", true);
	default_speed = node->declare_parameter("default_speed", 0.5);
	body.child_frame_id = node->declare_parameter("body_frame", string("body"));
	terrain.child_frame_id = node->declare_parameter("terrain_frame", string("terrain"));
	terrain_frame_mode = node->declare_parameter("terrain_frame_mode", string("altitude"));

	// reference_frames.<frame>: <reference frame>
	for (auto& override : node->get_node_parameters_interface()->get_parameter_overrides()) {
		if (override.first.rfind("reference_frames.", 0) == 0) {
			node->declare_parameter(override.first, override.second);
		}
	}
	node->get_parameters("reference_frames", reference_frames);

	// Default reference frames
	std::map<string, string> default_reference_frames;
	default_reference_frames[body.child_frame_id] = local_frame;
	default_reference_frames[fcu_frame] = local_frame;
	if (!target.child_frame_id.empty()) default_reference_frames[target.child_frame_id] = local_frame;
	reference_frames.insert(default_reference_frames.begin(), default_reference_frames.end()); // merge defaults

	state_timeout = durationParam("state_timeout", 3.0);
	local_position_timeout = durationParam("local_position_timeout", 2.0);
	velocity_timeout = durationParam("velocity_timeout", 2.0);
	global_position_timeout = durationParam("global_position_timeout", 10.0);
	battery_timeout = durationParam("battery_timeout", 2.0);
	manual_control_timeout = durationParam("manual_control_timeout", 0.0);

	transform_timeout = durationParam("transform_timeout", 0.5);
	telemetry_transform_timeout = durationParam("telemetry_transform_timeout", 0.5);
	offboard_timeout = durationParam("offboard_timeout", 3.0);
	land_timeout = durationParam("land_timeout", 3.0);
	arming_timeout = durationParam("arming_timeout", 4.0);

	// Callback groups, see threading rules above
	auto services_group = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
	auto clients_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
	auto telemetry_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
	auto terrain_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
	auto timer_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

	// Service clients
	arming = node->create_client<mavros_msgs::srv::CommandBool>(mavros + "/cmd/arming", rclcpp::ServicesQoS(), clients_group);
	set_mode = node->create_client<mavros_msgs::srv::SetMode>(mavros + "/set_mode", rclcpp::ServicesQoS(), clients_group);

	// Telemetry subscribers
	// mavros publishes most of the telemetry as best effort, such subscription is compatible with any publisher
	auto telemetry_qos = rclcpp::QoS(1).best_effort();
	rclcpp::SubscriptionOptions telemetry_options, terrain_options;
	telemetry_options.callback_group = telemetry_group;
	terrain_options.callback_group = terrain_group;

	auto state_sub = node->create_subscription<mavros_msgs::msg::State>(mavros + "/state", telemetry_qos, &handleState, telemetry_options);
	auto velocity_sub = node->create_subscription<TwistStamped>(mavros + "/local_position/velocity_body", telemetry_qos, &handleMessage<TwistStamped, velocity>, telemetry_options);
	auto global_position_sub = node->create_subscription<NavSatFix>(mavros + "/global_position/global", telemetry_qos, &handleMessage<NavSatFix, global_position>, telemetry_options);
	auto battery_sub = node->create_subscription<BatteryState>(mavros + "/battery", telemetry_qos, &handleMessage<BatteryState, battery>, telemetry_options);
	auto statustext_sub = node->create_subscription<mavros_msgs::msg::StatusText>(mavros + "/statustext/recv", telemetry_qos, &handleMessage<mavros_msgs::msg::StatusText, statustext>, telemetry_options);
	auto manual_control_sub = node->create_subscription<mavros_msgs::msg::ManualControl>(mavros + "/manual_control/control", telemetry_qos, &handleMessage<mavros_msgs::msg::ManualControl, manual_control>, telemetry_options);
	auto local_position_sub = node->create_subscription<PoseStamped>(mavros + "/local_position/pose", telemetry_qos, &handleLocalPosition, telemetry_options);

	rclcpp::SubscriptionBase::SharedPtr altitude_sub;
	if (!body.child_frame_id.empty() && !terrain.child_frame_id.empty()) {
		terrain.header.frame_id = local_frame;
		if (terrain_frame_mode == "altitude") {
			altitude_sub = node->create_subscription<Altitude>(mavros + "/altitude", telemetry_qos, &handleAltitude, terrain_options);
		} else if (terrain_frame_mode == "range") {
			string range_topic = node->declare_parameter("range_topic", string("rangefinder/range"));
			altitude_sub = node->create_subscription<Range>(range_topic, telemetry_qos, &handleRange, terrain_options);
		} else {
			RCLCPP_FATAL(node->get_logger(), "Unknown terrain_frame_mode: %s, valid values: altitude, range", terrain_frame_mode.c_str());
			rclcpp::shutdown();
			return;
		}
	}

	// Setpoint publishers
	position_pub = node->create_publisher<PoseStamped>(mavros + "/setpoint_position/local", 1);
	position_raw_pub = node->create_publisher<PositionTarget>(mavros + "/setpoint_raw/local", 1);
	attitude_pub = node->create_publisher<PoseStamped>(mavros + "/setpoint_attitude/attitude", 1);
	attitude_raw_pub = node->create_publisher<AttitudeTarget>(mavros + "/setpoint_raw/attitude", 1);
	rates_pub = node->create_publisher<TwistStamped>(mavros + "/setpoint_attitude/cmd_vel", 1);
	thrust_pub = node->create_publisher<Thrust>(mavros + "/setpoint_attitude/thrust", 1);

	// State publisher
	state_pub = node->create_publisher<clover::msg::State>("~/state", rclcpp::QoS(1).transient_local());

	// Setpoint timer
	setpoint_timer = rclcpp::create_timer(node, node->get_clock(),
	                                      rclcpp::Duration::from_seconds(1 / node->declare_parameter("setpoint_rate", 30.0)), // editorconfig-checker-disable-line
	                                      &publishSetpoint, timer_group, false); // editorconfig-checker-disable-line

	 // Service servers
	auto gt_serv = node->create_service<GetTelemetry>("get_telemetry", &getTelemetry, rclcpp::ServicesQoS(), services_group);
	auto na_serv = node->create_service<Navigate>("navigate", &navigate, rclcpp::ServicesQoS(), services_group);
	auto ng_serv = node->create_service<NavigateGlobal>("navigate_global", &navigateGlobal, rclcpp::ServicesQoS(), services_group);
	auto sl_serv = node->create_service<SetAltitude>("set_altitude", &setAltitude, rclcpp::ServicesQoS(), services_group);
	auto ya_serv = node->create_service<SetYaw>("set_yaw", &setYaw, rclcpp::ServicesQoS(), services_group);
	auto yr_serv = node->create_service<SetYawRate>("set_yaw_rate", &setYawRate, rclcpp::ServicesQoS(), services_group);
	auto sp_serv = node->create_service<SetPosition>("set_position", &setPosition, rclcpp::ServicesQoS(), services_group);
	auto sv_serv = node->create_service<SetVelocity>("set_velocity", &setVelocity, rclcpp::ServicesQoS(), services_group);
	auto sa_serv = node->create_service<SetAttitude>("set_attitude", &setAttitude, rclcpp::ServicesQoS(), services_group);
	auto sr_serv = node->create_service<SetRates>("set_rates", &setRates, rclcpp::ServicesQoS(), services_group);
	auto ld_serv = node->create_service<std_srvs::srv::Trigger>("land", &land, rclcpp::ServicesQoS(), services_group);
	auto rl_serv = node->create_service<std_srvs::srv::Trigger>("~/release", &release, rclcpp::ServicesQoS(), services_group);

	position_msg.header.frame_id = local_frame;
	position_raw_msg.header.frame_id = local_frame;
	position_raw_msg.coordinate_frame = PositionTarget::FRAME_LOCAL_NED;
	//rates_msg.header.frame_id = fcu_frame;

	RCLCPP_INFO(node->get_logger(), "ready");

	// several threads are needed for a callback to wait while other callbacks run
	rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), std::max(4u, std::thread::hardware_concurrency()));
	executor.add_node(node);
	executor.spin();
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	node = std::make_shared<rclcpp::Node>("simple_offboard");

	run();

	// destroy global ROS entities before the context and the middleware are destroyed
	setpoint_timer.reset();
	arming.reset();
	set_mode.reset();
	attitude_pub.reset();
	position_pub.reset();
	attitude_raw_pub.reset();
	position_raw_pub.reset();
	rates_pub.reset();
	thrust_pub.reset();
	state_pub.reset();
	transform_broadcaster.reset();
	static_transform_broadcaster.reset();
	tf_buffer.reset();
	node.reset();

	rclcpp::shutdown();
	return 0;
}
