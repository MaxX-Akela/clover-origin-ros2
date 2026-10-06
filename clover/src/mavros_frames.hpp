/*
 * Frames of the local position, shared by the nodes of the package
 *
 * Distributed under MIT License (available at https://opensource.org/licenses/MIT).
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 */

#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>

namespace clover
{

// Get local and FCU frames the same way simple_offboard does:
// 1. local_frame and fcu_frame parameters of the node, if they are not empty;
// 2. tf.frame_id and tf.child_frame_id parameters of mavros local_position plugin node;
// 3. map and base_link with a warning.
// Blocks up to several seconds waiting for mavros, so should be called before
// the node's subscriptions are created and not from a callback.
inline void readMavrosFrames(rclcpp::Node *node, std::string& local_frame, std::string& fcu_frame,
                             const std::string& mavros = "mavros") // editorconfig-checker-disable-line
{
	const std::string local_frame_param = node->declare_parameter("local_frame", std::string("")); // read from mavros if empty
	const std::string fcu_frame_param = node->declare_parameter("fcu_frame", std::string("")); // read from mavros if empty

	local_frame = "map";
	fcu_frame = "base_link";

	if (local_frame_param.empty() || fcu_frame_param.empty()) {
		const std::string plugin = mavros + "/local_position";
		auto params = std::make_shared<rclcpp::SyncParametersClient>(node, plugin);
		if (!params->wait_for_service(std::chrono::seconds(5))) {
			RCLCPP_WARN(node->get_logger(), "can't read frames from %s, using %s and %s",
			            plugin.c_str(), local_frame.c_str(), fcu_frame.c_str()); // editorconfig-checker-disable-line
		} else {
			for (auto& param : params->get_parameters({"tf.frame_id", "tf.child_frame_id"}, std::chrono::seconds(5))) {
				if (param.get_type() != rclcpp::ParameterType::PARAMETER_STRING) continue;
				if (param.get_name() == "tf.frame_id") local_frame = param.as_string();
				if (param.get_name() == "tf.child_frame_id") fcu_frame = param.as_string();
			}
		}
	}

	if (!local_frame_param.empty()) local_frame = local_frame_param;
	if (!fcu_frame_param.empty()) fcu_frame = fcu_frame_param;
}

} // namespace clover
