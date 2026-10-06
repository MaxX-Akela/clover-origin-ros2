/*
 * Clover mobile remote control backend
 * Send ManualControl messages through UDP
 * 'latched_state' topic
 *
 * Copyright (C) 2019 Copter Express Technologies
 *
 * Author: Oleg Kalachev <okalachev@gmail.com>
 *
 * Distributed under MIT License (available at https://opensource.org/licenses/MIT).
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 */

/*
 * Threading (ROS 2):
 *
 * 1. UDP socket is read in a separate thread (socketThread), not in a callback.
 *    The thread is joined in the destructor.
 * 2. State subscription and state timeout timer share a mutually exclusive
 *    callback group, so state_msg is not guarded.
 * 3. Fake GCS heartbeat is sent by a timer in its own callback group.
 * 4. last_manual_control is written by the socket thread and read by the fake
 *    GCS timer, it's guarded by manual_control_mutex.
 */

#include <sys/socket.h>
#include <netinet/in.h>
#include <poll.h>
#include <unistd.h>
#include <errno.h>
#include <cstring>
#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/msg/manual_control.hpp"
#include "mavros_msgs/msg/mavlink.hpp"

struct ControlMessage
{
	int16_t x, y, z, r;
} __attribute__((packed));

class RC : public rclcpp::Node
{
public:
	RC():
		Node("rc")
	{
		bool use_fake_gcs = declare_parameter("use_fake_gcs", true);
		port = declare_parameter("port", 35602);
		manual_control_pub = create_publisher<mavros_msgs::msg::ManualControl>("mavros/manual_control/send", 1);

		// Create socket thread
		socket_thread = std::thread(&RC::socketThread, this);

		if (use_fake_gcs) {
			initFakeGCS();
		}

		initLatchedState();
	}

	~RC()
	{
		stop = true;
		socket_thread.join();
	}

private:
	rclcpp::CallbackGroup::SharedPtr state_group, gcs_group;
	rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub;
	rclcpp::Publisher<mavros_msgs::msg::State>::SharedPtr state_pub;
	rclcpp::Publisher<mavros_msgs::msg::ManualControl>::SharedPtr manual_control_pub;
	rclcpp::Publisher<mavros_msgs::msg::Mavlink>::SharedPtr mavlink_pub;
	rclcpp::TimerBase::SharedPtr state_timeout_timer, gcs_timer;
	std::mutex manual_control_mutex;
	rclcpp::Time last_manual_control{0, 0, RCL_ROS_TIME};
	mavros_msgs::msg::State::ConstSharedPtr state_msg;
	mavros_msgs::msg::Mavlink hb;
	std::thread socket_thread;
	std::atomic<bool> stop{false};
	int port;

	void handleState(const mavros_msgs::msg::State::ConstSharedPtr& state)
	{
		state_timeout_timer->reset(); // restart the countdown

		if (!state_msg ||
			state->connected != state_msg->connected ||
			state->mode != state_msg->mode ||
			state->armed != state_msg->armed) {
				state_msg = state;
				state_pub->publish(*state_msg);
			}
	}

	void stateTimedOut()
	{
		state_timeout_timer->cancel(); // one-shot
		RCLCPP_INFO(get_logger(), "State timeout");
		mavros_msgs::msg::State unknown_state;
		state_pub->publish(unknown_state);
		state_msg = nullptr;
	}

	void initLatchedState()
	{
		state_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		rclcpp::SubscriptionOptions options;
		options.callback_group = state_group;

		// compatible with any publisher (mavros publishes the state as reliable and transient local)
		state_sub = create_subscription<mavros_msgs::msg::State>("mavros/state", rclcpp::QoS(1).best_effort(),
			std::bind(&RC::handleState, this, std::placeholders::_1), options);
		state_pub = create_publisher<mavros_msgs::msg::State>("state_latched", rclcpp::QoS(1).transient_local());
		state_timeout_timer = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(3),
			std::bind(&RC::stateTimedOut, this), state_group, false);

		// Publish initial state
		mavros_msgs::msg::State unknown_state;
		state_pub->publish(unknown_state);
	}

	void initFakeGCS()
	{
		// Awful workaround for fixing PX4 not sending STATUSTEXTs
		// if there is no GCS heartbeats.
		// TODO: remove, when PX4 get this fixed.

		// mavros 2 has no mavlink/to topic, messages are passed to the router through the UAS sink topic
		std::string mavlink_topic = declare_parameter("mavlink_topic", std::string("/uas1/mavlink_sink"));
		mavlink_pub = create_publisher<mavros_msgs::msg::Mavlink>(mavlink_topic, rclcpp::QoS(1).best_effort());

		// HEARTBEAT from GCS message
		hb.framing_status = mavros_msgs::msg::Mavlink::FRAMING_OK;
		hb.magic = mavros_msgs::msg::Mavlink::MAVLINK_V20;
		hb.len = 9;
		hb.incompat_flags = 0;
		hb.compat_flags = 0;
		hb.seq = 0;
		hb.sysid = 255;
		hb.compid = 0;
		hb.checksum = 26460;
		hb.payload64.push_back(342282393542983680);
		hb.payload64.push_back(3);

		gcs_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		gcs_timer = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(1),
			std::bind(&RC::sendHeartbeat, this), gcs_group);
	}

	void sendHeartbeat()
	{
		rclcpp::Time last;
		{
			std::lock_guard<std::mutex> lock(manual_control_mutex);
			last = last_manual_control;
		}
		if (now() - last < rclcpp::Duration::from_seconds(8)) {
			mavlink_pub->publish(hb);
		}
	}

	int createSocket(int port)
	{
		int sockfd = socket(AF_INET, SOCK_DGRAM, 0);

		sockaddr_in sin;
		sin.sin_family = AF_INET;
		sin.sin_addr.s_addr = htonl(INADDR_ANY);
		sin.sin_port = htons(port);

		if (bind(sockfd, (sockaddr *)&sin, sizeof(sin)) < 0) {
			RCLCPP_FATAL(get_logger(), "socket bind error: %s", strerror(errno));
			close(sockfd);
			rclcpp::shutdown();
			return -1;
		}

		return sockfd;
	}

	void socketThread()
	{
		int sockfd = createSocket(port);

		char buff[9999];

		mavros_msgs::msg::ManualControl manual_control_msg;

		sockaddr_in client_addr;
		socklen_t client_addr_size = sizeof(client_addr);

		RCLCPP_INFO(get_logger(), "UDP RC initialized on port %d", port);

		while (rclcpp::ok() && !stop) {
			// wait for a packet, checking regularly if the node is being stopped
			pollfd pfd = { sockfd, POLLIN, 0 };
			if (poll(&pfd, 1, 500) <= 0) continue;

			// read next UDP packet
			int bsize = recvfrom(sockfd, &buff[0], sizeof(buff) - 1, 0, (sockaddr *) &client_addr, &client_addr_size);

			if (bsize < 0) {
				RCLCPP_ERROR(get_logger(), "recvfrom() error: %s", strerror(errno));
			} else if (bsize != sizeof(ControlMessage)) {
				RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 30000, "Wrong UDP packet size: %d", bsize);
			}

			// unpack message
			// warning: ignore endianness, so the code is platform-dependent
			ControlMessage *msg = (ControlMessage *)buff;

			manual_control_msg.x = msg->x;
			manual_control_msg.y = msg->y;
			manual_control_msg.z = msg->z;
			manual_control_msg.r = msg->r;
			manual_control_pub->publish(manual_control_msg);

			std::lock_guard<std::mutex> lock(manual_control_mutex);
			last_manual_control = now();
		}

		if (sockfd >= 0) close(sockfd);
	}
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	{
		auto rc = std::make_shared<RC>();
		rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
		executor.add_node(rc);
		executor.spin();
	}
	rclcpp::shutdown();
	return 0;
}
