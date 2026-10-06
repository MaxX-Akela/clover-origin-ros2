/*
 * Visualization marker for camera alignment
 * Copyright (C) 2018 Copter Express Technologies
 *
 * Author: Oleg Kalachev <okalachev@gmail.com>
 *
 * Distributed under MIT License (available at https://opensource.org/licenses/MIT).
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 */

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace visualization_msgs::msg;

double markers_scale;
std::string camera_frame;

MarkerArray createMarkers() {
	MarkerArray markers;

	Marker lens;
	lens.header.frame_id = camera_frame;
	lens.ns = "camera_markers";
	lens.id = 0;
	lens.action = Marker::ADD;
	lens.type = Marker::CYLINDER;
	lens.frame_locked = true;
	lens.scale.x = 0.013 * markers_scale;
	lens.scale.y = 0.013 * markers_scale;
	lens.scale.z = 0.015 * markers_scale;
	lens.color.r = 0.3;
	lens.color.g = 0.3;
	lens.color.b = 0.3;
	lens.color.a = 0.9;
	lens.pose.position.z = 0.0075 * markers_scale;
	lens.pose.orientation.w = 1;

	Marker board;
	board.header.frame_id = camera_frame;
	board.ns = "camera_markers";
	board.id = 1;
	board.action = Marker::ADD;
	board.type = Marker::CUBE;
	board.frame_locked = true;
	board.scale.x = 0.024 * markers_scale;
	board.scale.y = 0.024 * markers_scale;
	board.scale.z = 0.001 * markers_scale;
	board.color.r = 0.0;
	board.color.g = 0.8;
	board.color.b = 0.0;
	board.color.a = 0.9;
	board.pose.orientation.w = 1;

	Marker wire;
	wire.header.frame_id = camera_frame;
	wire.ns = "camera_markers";
	wire.id = 2;
	wire.action = Marker::ADD;
	wire.type = Marker::CUBE;
	wire.frame_locked = true;
	wire.scale.x = 0.014 * markers_scale;
	wire.scale.y = 0.04 * markers_scale;
	wire.scale.z = 0.001 * markers_scale;
	wire.color.r = 0.9;
	wire.color.g = 0.9;
	wire.color.b = 1.0;
	wire.color.a = 0.8;
	wire.pose.position.x = 0;
	wire.pose.position.y = (0.01 + 0.02) * markers_scale;
	wire.pose.position.z = 0.002 * markers_scale;
	wire.pose.orientation.w = 1;

	markers.markers.push_back(lens);
	markers.markers.push_back(board);
	markers.markers.push_back(wire);

	return markers;
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<rclcpp::Node>("camera_markers");

	markers_scale = node->declare_parameter("scale", 1.0);

	rclcpp::Publisher<MarkerArray>::SharedPtr markers_pub;
	rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub;

	// wait for camera info
	camera_info_sub = node->create_subscription<sensor_msgs::msg::CameraInfo>("camera_info", rclcpp::QoS(1).best_effort(),
		[&](const sensor_msgs::msg::CameraInfo& camera_info) {
			if (markers_pub) return; // already initialized
			camera_frame = camera_info.header.frame_id;

			markers_pub = node->create_publisher<MarkerArray>("camera_markers", rclcpp::QoS(1).transient_local());
			markers_pub->publish(createMarkers());

			RCLCPP_INFO(node->get_logger(), "Camera markers initialized");
			camera_info_sub.reset();
		});

	rclcpp::spin(node);

	camera_info_sub.reset();
	markers_pub.reset();
	node.reset();
	rclcpp::shutdown();
	return 0;
}
