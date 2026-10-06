/*
 * VPE publisher node
 * Copyright (C) 2018 Copter Express Technologies
 *
 * Author: Oleg Kalachev <okalachev@gmail.com>
 *
 * Distributed under MIT License (available at https://opensource.org/licenses/MIT).
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 */

/*
 * Threading (ROS 2): all the callbacks run one at a time in a single-threaded
 * executor, like in the ROS 1 node, so the state is not guarded by a mutex.
 * TF listener spins in its own thread, which makes waiting for a transform
 * inside a callback possible.
 */

#include <string>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
// #include <aruco_pose/MarkerArray.h>

#include "mavros_frames.hpp"

using std::string;
using namespace geometry_msgs::msg;

rclcpp::Node::SharedPtr node;

bool reset_flag = true; // offset should be reset on the start
string local_frame_id, frame_id, child_frame_id, offset_frame_id;
std::shared_ptr<tf2_ros::Buffer> tf_buffer;
std::shared_ptr<tf2_ros::StaticTransformBroadcaster> br;
rclcpp::Publisher<PoseStamped>::SharedPtr vpe_pub;
rclcpp::Subscription<PoseStamped>::SharedPtr local_position_sub;
rclcpp::TimerBase::SharedPtr zero_timer;
PoseStamped vpe, pose;
rclcpp::Time got_local_pos(0, 0, RCL_ROS_TIME);
rclcpp::Duration publish_zero_timeout(0, 0), publish_zero_duration(0, 0), offset_timeout(0, 0);
TransformStamped offset;

inline bool isZero(const builtin_interfaces::msg::Time& stamp)
{
	return stamp.sec == 0 && stamp.nanosec == 0;
}

void publishZero()
{
	rclcpp::Time current_real = node->now();

	if (!isZero(vpe.header.stamp) && current_real - vpe.header.stamp < publish_zero_timeout) return; // have vpe

	if (!isZero(pose.header.stamp) && current_real - pose.header.stamp < publish_zero_timeout) { // have local position
		if (got_local_pos.nanoseconds() == 0) {
			RCLCPP_INFO(node->get_logger(), "got local position");
			got_local_pos = current_real;
		}

		if (current_real - got_local_pos > publish_zero_duration) return; // stop publishing zero
	} else {
		// lost local position
		got_local_pos = rclcpp::Time(0, 0, RCL_ROS_TIME);
	}

	RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 10000, "publish zero");
	PoseStamped zero;
	zero.header.frame_id = local_frame_id;
	zero.header.stamp = current_real;
	zero.pose.orientation.w = 1;
	vpe_pub->publish(zero);
}

void localPositionCallback(const PoseStamped& msg) { pose = msg; }

inline Pose getPose(const PoseStamped::ConstSharedPtr& pose) { return pose->pose; }

inline Pose getPose(const PoseWithCovarianceStamped::ConstSharedPtr& pose) { return pose->pose.pose; }

inline double getYaw(const Quaternion& q_msg)
{
	tf2::Quaternion q;
	tf2::fromMsg(q_msg, q);
	double roll, pitch, yaw;
	tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
	return yaw;
}

inline void keepYaw(Quaternion& quaternion)
{
	tf2::Quaternion q;
	q.setRPY(0, 0, getYaw(quaternion));
	quaternion = tf2::toMsg(q);
}

template <typename T>
void callback(const std::shared_ptr<const T> msg)
{
	const rclcpp::Time stamp = msg->header.stamp;

	try {
		if (!frame_id.empty()) {
			// get VPE transform from TF
			auto transform = tf_buffer->lookupTransform(frame_id, child_frame_id,
			                                            stamp, rclcpp::Duration::from_seconds(0.02)); // editorconfig-checker-disable-line
			vpe.pose.position.x = transform.transform.translation.x;
			vpe.pose.position.y = transform.transform.translation.y;
			vpe.pose.position.z = transform.transform.translation.z;
			vpe.pose.orientation = transform.transform.rotation;
		} else {
			vpe.pose = getPose(msg);
		}

		// offset
		if (!offset_frame_id.empty()) {
			if (reset_flag || stamp - vpe.header.stamp > offset_timeout) {
				// calculate the offset
				if (!frame_id.empty()) {
					// calculate from TF
					offset = tf_buffer->lookupTransform(local_frame_id, frame_id,
					                                    stamp, rclcpp::Duration::from_seconds(0.02)); // editorconfig-checker-disable-line
					// offset.header.frame_id = vpe.header.frame_id;
					offset.child_frame_id = offset_frame_id;

				} else {
					// calculate transform between pose in vpe frame and pose in local frame
					TransformStamped local_pose = tf_buffer->lookupTransform(local_frame_id, child_frame_id,
					                                                         stamp, rclcpp::Duration::from_seconds(0.02)); // editorconfig-checker-disable-line
					keepYaw(local_pose.transform.rotation);

					tf2::Transform vpeTransform, poseTransform;
					tf2::fromMsg(vpe.pose, vpeTransform);
					tf2::fromMsg(local_pose.transform, poseTransform);
					tf2::Transform offset_tf = vpeTransform.inverseTimes(poseTransform);
					offset.transform = tf2::toMsg(offset_tf);
					offset.header.frame_id = local_frame_id;
					offset.header.stamp = msg->header.stamp;
					offset.child_frame_id = offset_frame_id;
				}

				br->sendTransform(offset);
				reset_flag = false;
				RCLCPP_INFO(node->get_logger(), "offset reset");
			}
			// apply the offset
			tf2::doTransform(vpe, vpe, offset);
		}

		vpe.header.frame_id = local_frame_id;
		vpe.header.stamp = msg->header.stamp;
		vpe_pub->publish(vpe);

	} catch (const tf2::TransformException& e) {
		RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 5000, "%s", e.what());
	}
}

void reset(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
	reset_flag = true;
	res->success = true;
}

// create all the node's entities and spin
void run()
{
	tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
	tf2_ros::TransformListener tf_listener(*tf_buffer, node, true);
	br = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);

	frame_id = node->declare_parameter("frame_id", string("")); // name for used visual pose frame
	offset_frame_id = node->declare_parameter("offset_frame_id", string("")); // name for published offset frame

	clover::readMavrosFrames(node.get(), local_frame_id, child_frame_id);
	offset_timeout = rclcpp::Duration::from_seconds(node->declare_parameter("offset_timeout", 3.0));

	if (!frame_id.empty()) {
		RCLCPP_INFO(node->get_logger(), "using data from TF");
	} else {
		RCLCPP_INFO(node->get_logger(), "using data topic");
	}

	// compatible with any publisher
	auto input_qos = rclcpp::QoS(1).best_effort();

	auto pose_sub = node->create_subscription<PoseStamped>("~/pose", input_qos, &callback<PoseStamped>);
	auto pose_cov_sub = node->create_subscription<PoseWithCovarianceStamped>("~/pose_cov", input_qos, &callback<PoseWithCovarianceStamped>);
	//auto markers_sub = nh_priv.subscribe<aruco_pose::MarkerArray>("markers", 1, &callback);

	vpe_pub = node->create_publisher<PoseStamped>("~/vpe", 1);
	//vpe_cov_pub = nh_priv_.advertise<PoseStamped>("pose_cov_pub", 1);

	bool force_init = node->declare_parameter("force_init", false);
	bool publish_zero = node->declare_parameter("publish_zero", false); // publish_zero is old name
	if (force_init || publish_zero) {
		// publish zero to initialize the local position
		publish_zero_timeout = rclcpp::Duration::from_seconds(node->declare_parameter("force_init_timeout", 5.0));
		publish_zero_duration = rclcpp::Duration::from_seconds(node->declare_parameter("force_init_duration", 5.0));
		local_position_sub = node->create_subscription<PoseStamped>("mavros/local_position/pose", input_qos, &localPositionCallback);
		zero_timer = rclcpp::create_timer(node, node->get_clock(), rclcpp::Duration::from_seconds(0.1), &publishZero);
	}

	auto reset_serv = node->create_service<std_srvs::srv::Trigger>("~/reset", &reset);

	RCLCPP_INFO(node->get_logger(), "ready");
	rclcpp::spin(node);
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	node = std::make_shared<rclcpp::Node>("vpe_publisher");

	run();

	// destroy global ROS entities before the context and the middleware are destroyed
	zero_timer.reset();
	local_position_sub.reset();
	vpe_pub.reset();
	br.reset();
	tf_buffer.reset();
	node.reset();

	rclcpp::shutdown();
	return 0;
}
