#include <rclcpp/rclcpp.hpp>
#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <array>

#include <clover/srv/execute.hpp>

using clover::srv::Execute;

rclcpp::Node::SharedPtr node;
rclcpp::Duration timeout(0, 0);

// TODO: handle timeout
void handle(const std::shared_ptr<Execute::Request> req, std::shared_ptr<Execute::Response> res)
{
	RCLCPP_INFO(node->get_logger(), "Execute: %s", req->cmd.c_str());

	std::array<char, 128> buffer;
	std::string result;

	FILE *fp = popen(req->cmd.c_str(), "r");

	if (fp == NULL) {
		res->code = Execute::Request::CODE_FAIL;
		res->output = "popen() failed";
		return;
	}

	while (fgets(buffer.data(), buffer.size(), fp) != nullptr) {
		res->output += buffer.data();
	}

	res->code = pclose(fp);
}


int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	node = std::make_shared<rclcpp::Node>("shell");

	timeout = rclcpp::Duration::from_seconds(node->declare_parameter("timeout", 3.0));

	// commands are executed one at a time, the node itself (parameters) stays responsive
	auto exec_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
	auto gt_serv = node->create_service<Execute>("exec", &handle, rclcpp::ServicesQoS(), exec_group);

	RCLCPP_INFO(node->get_logger(), "shell: ready");

	rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
	executor.add_node(node);
	executor.spin();

	gt_serv.reset();
	node.reset();
	rclcpp::shutdown();
	return 0;
}
