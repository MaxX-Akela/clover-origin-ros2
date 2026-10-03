/*
 * Utility functions
 * Copyright (C) 2018 Copter Express Technologies
 *
 * Author: Oleg Kalachev <okalachev@gmail.com>
 *
 * Distributed under MIT License (available at https://opensource.org/licenses/MIT).
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 */

#pragma once

#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <opencv2/opencv.hpp>

// Read required param or shutdown the node
template<typename T>
static void param(rclcpp::Node& node, const std::string& param_name, T& param_val)
{
	if (!node.has_parameter(param_name)) {
		node.declare_parameter(param_name, rclcpp::ParameterValue(param_val).get_type());
	}
	if (!node.get_parameter(param_name, param_val)) {
		RCLCPP_FATAL(node.get_logger(), "Required param %s is not defined", param_name.c_str());
		rclcpp::shutdown();
	}
}

static void parseCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cinfo, cv::Mat& matrix, cv::Mat& dist)
{
	for (unsigned int i = 0; i < 3; ++i)
		for (unsigned int j = 0; j < 3; ++j)
			matrix.at<double>(i, j) = cinfo->k[3 * i + j];
	dist = cv::Mat(cinfo->d, true);
}

inline void rotatePoint(cv::Point3f& p, cv::Point3f origin, float angle)
{
	float s = sin(angle);
	float c = cos(angle);

	// translate point back to origin:
	p.x -= origin.x;
	p.y -= origin.y;

	// rotate point
	float xnew = p.x * c - p.y * s;
	float ynew = p.x * s + p.y * c;

	// translate point back:
	p.x = xnew + origin.x;
	p.y = ynew + origin.y;
}

inline void fillPose(geometry_msgs::msg::Pose& pose, const cv::Vec3d& rvec, const cv::Vec3d& tvec)
{
	pose.position.x = tvec[0];
	pose.position.y = tvec[1];
	pose.position.z = tvec[2];

	double angle = norm(rvec);
	cv::Vec3d axis = rvec / angle;

	tf2::Quaternion q;
	q.setRotation(tf2::Vector3(axis[0], axis[1], axis[2]), angle);

	pose.orientation.w = q.w();
	pose.orientation.x = q.x();
	pose.orientation.y = q.y();
	pose.orientation.z = q.z();
}

inline void fillTransform(geometry_msgs::msg::Transform& transform, const cv::Vec3d& rvec, const cv::Vec3d& tvec)
{
	transform.translation.x = tvec[0];
	transform.translation.y = tvec[1];
	transform.translation.z = tvec[2];

	double angle = norm(rvec);
	cv::Vec3d axis = rvec / angle;

	tf2::Quaternion q;
	q.setRotation(tf2::Vector3(axis[0], axis[1], axis[2]), angle);

	transform.rotation.w = q.w();
	transform.rotation.x = q.x();
	transform.rotation.y = q.y();
	transform.rotation.z = q.z();
}

inline void fillTranslation(geometry_msgs::msg::Vector3& translation, const cv::Vec3d& tvec)
{
	translation.x = tvec[0];
	translation.y = tvec[1];
	translation.z = tvec[2];
}

inline tf2::Quaternion createQuaternionFromRPY(double roll, double pitch, double yaw)
{
	tf2::Quaternion q;
	q.setRPY(roll, pitch, yaw);
	return q;
}

inline bool isFlipped(tf2::Quaternion& q)
{
	double yaw, pitch, roll;
	tf2::Matrix3x3(q).getEulerYPR(yaw, pitch, roll);
	return (abs(pitch) > M_PI / 2) || (abs(roll) > M_PI / 2);
}

/* Apply a vertical to an orientation */
inline void applyVertical(geometry_msgs::msg::Quaternion& orientation, const geometry_msgs::msg::Quaternion& vertical,
                          bool flip_vertical = false, bool auto_flip = false) // editorconfig-checker-disable-line
{
	tf2::Quaternion _vertical, _orientation;
	tf2::fromMsg(vertical, _vertical);
	tf2::fromMsg(orientation, _orientation);

	if (flip_vertical || (auto_flip && !isFlipped(_orientation))) {
		static const tf2::Quaternion flip = createQuaternionFromRPY(M_PI, 0, 0);
		_vertical *= flip; // flip vertical
	}

	auto diff = tf2::Matrix3x3(_orientation).transposeTimes(tf2::Matrix3x3(_vertical));
	double _, yaw;
	diff.getRPY(_, _, yaw);
	auto q = createQuaternionFromRPY(0, 0, -yaw);
	_vertical = _vertical * q; // set yaw from orientation to vertical
	orientation = tf2::toMsg(_vertical); // set vertical to orientation
}

inline void transformToPose(const geometry_msgs::msg::Transform& transform, geometry_msgs::msg::Pose& pose)
{
	pose.position.x = transform.translation.x;
	pose.position.y = transform.translation.y;
	pose.position.z = transform.translation.z;
	pose.orientation = transform.rotation;
}
