/*
 * High level control for the LED strip
 * Indicate flight events with the LED strip
 * Copyright (C) 2019 Copter Express Technologies
 *
 * Author: Oleg Kalachev <okalachev@gmail.com>
 *
 * Distributed under MIT License (available at https://opensource.org/licenses/MIT).
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 */

/*
 * Threading rules (ROS 2), the same as in simple_offboard:
 *
 * 1. All the state below is guarded by state_mutex. Every callback (service,
 *    subscription, timer, service response) locks it for its whole duration,
 *    so callbacks never run simultaneously, like in the single-threaded ROS 1 node.
 * 2. Any waiting inside a callback MUST release state_mutex (spinSleep): the
 *    delays of the flash effect and waiting for the flash to finish.
 * 3. set_leds service is only called asynchronously, nobody waits for
 *    the response. While a request is in progress, new frames are merged
 *    to one pending request (see callSetLeds).
 */

#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <clover/srv/set_led_effect.hpp>
#include <led_msgs/srv/set_le_ds.hpp>
#include <led_msgs/msg/led_state.hpp>
#include <led_msgs/msg/led_state_array.hpp>

#include <sensor_msgs/msg/battery_state.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <rcl_interfaces/msg/log.hpp>

using clover::srv::SetLEDEffect;
using led_msgs::srv::SetLEDs;

rclcpp::Node::SharedPtr node;

// Guards all the state, see threading rules above
std::mutex state_mutex;

SetLEDEffect::Request current_effect;
int led_count;
bool got_state = false;
rclcpp::TimerBase::SharedPtr timer;
rclcpp::CallbackGroup::SharedPtr timer_group;
unsigned int timer_generation = 0; // changes each time the timer is restarted
rclcpp::Time start_time(0, 0, RCL_ROS_TIME);
double blink_rate, blink_fast_rate, flash_delay, fade_period, wipe_period, rainbow_period;
double low_battery_threshold;
std::vector<std::string> error_ignore;
struct { SetLEDs::Request request; } set_leds; // current frame
led_msgs::msg::LEDStateArray state, start_state;
rclcpp::Client<SetLEDs>::SharedPtr set_leds_srv;
mavros_msgs::msg::State mavros_state;
int counter;
bool flashing = false; // flash effect is in progress

// set_leds calls
std::map<uint32_t, led_msgs::msg::LEDState> pending_leds; // frames waiting for the current call to finish
bool call_in_progress = false;
int64_t call_id;
std::chrono::steady_clock::time_point call_time;
const std::chrono::seconds CALL_TIMEOUT(1);

// Sleep letting other callbacks run (see threading rules above)
inline void spinSleep(const rclcpp::Duration& duration)
{
	state_mutex.unlock();
	node->get_clock()->sleep_for(duration);
	state_mutex.lock();
}

void handleSetLedsResponse(rclcpp::Client<SetLEDs>::SharedFuture future);

// send pending frames, if there is no call in progress
void sendSetLeds()
{
	if (pending_leds.empty()) return;

	if (call_in_progress) {
		if (std::chrono::steady_clock::now() - call_time < CALL_TIMEOUT) return; // wait for the response
		// no response
		RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 5000, "Error calling set_leds service");
		set_leds_srv->remove_pending_request(call_id);
		call_in_progress = false;
	}

	if (!set_leds_srv->service_is_ready()) {
		RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 5000, "Error calling set_leds service");
		pending_leds.clear();
		return;
	}

	auto request = std::make_shared<SetLEDs::Request>();
	request->leds.reserve(pending_leds.size());
	for (auto const& led : pending_leds) {
		request->leds.push_back(led.second);
	}
	pending_leds.clear();

	call_in_progress = true;
	call_time = std::chrono::steady_clock::now();
	call_id = set_leds_srv->async_send_request(request, &handleSetLedsResponse).request_id;
}

void handleSetLedsResponse(rclcpp::Client<SetLEDs>::SharedFuture future)
{
	std::lock_guard<std::mutex> lock(state_mutex);
	call_in_progress = false;
	auto response = future.get();
	if (!response->success) {
		RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 5000, "Calling set_leds failed: %s", response->message.c_str());
	}
	sendSetLeds();
}

// Send the current frame without waiting for the result. If the previous call
// is not finished yet, the frame is merged to the pending one (the last color
// of each LED wins), so no more than one call is waiting.
void callSetLeds()
{
	for (auto const& led : set_leds.request.leds) {
		pending_leds[led.index] = led;
	}
	sendSetLeds();
}

void rainbow(uint8_t n, uint8_t& r, uint8_t& g, uint8_t& b)
{
	if (n < 255 / 3) {
		r = n * 3;
		g = 255 - n * 3;
		b = 0;
	} else if (n < 255 / 3 * 2) {
		n -= 255 / 3;
		r = 255 - n * 3;
		g = 0;
		b = n * 3;
	} else {
		n -= 255 / 3 * 2;
		r = 0;
		g = n * 3;
		b = 255 - n * 3;
	}
}

void fill(uint8_t r, uint8_t g, uint8_t b)
{
	set_leds.request.leds.resize(led_count);
	for (int i = 0; i < led_count; i++) {
		set_leds.request.leds[i].index = i;
		set_leds.request.leds[i].r = r;
		set_leds.request.leds[i].g = g;
		set_leds.request.leds[i].b = b;
	}
	callSetLeds();
}

void proceed(const rclcpp::Time& current_real)
{
	counter++;
	uint8_t r, g, b;
	set_leds.request.leds.clear();
	set_leds.request.leds.resize(led_count);

	if (current_effect.effect == "blink" || current_effect.effect == "blink_fast") {
		// enable on odd counter
		if (counter % 2 != 0) {
			fill(current_effect.r, current_effect.g, current_effect.b);
		} else {
			fill(0, 0, 0);
		}

	} else if (current_effect.effect == "fade") {
		// fade all leds from starting state
		double passed = std::min((current_real - start_time).seconds() / fade_period, 1.0);
		double one_minus_passed = 1 - passed;
		for (int i = 0; i < led_count; i++) {
			set_leds.request.leds[i].index = i;
			set_leds.request.leds[i].r = one_minus_passed * start_state.leds[i].r + passed * current_effect.r;
			set_leds.request.leds[i].g = one_minus_passed * start_state.leds[i].g + passed * current_effect.g;
			set_leds.request.leds[i].b = one_minus_passed * start_state.leds[i].b + passed * current_effect.b;
		}
		callSetLeds();
		if (passed >= 1.0) {
			// fade finished
			timer->cancel();
		}

	} else if (current_effect.effect == "wipe") {
		set_leds.request.leds.resize(1);
		set_leds.request.leds[0].index = counter - 1;
		set_leds.request.leds[0].r = current_effect.r;
		set_leds.request.leds[0].g = current_effect.g;
		set_leds.request.leds[0].b = current_effect.b;
		callSetLeds();
		if (counter == led_count) {
			// wipe finished
			timer->cancel();
		}

	} else if (current_effect.effect == "rainbow_fill") {
		rainbow(counter % 255, r, g, b);
		for (int i = 0; i < led_count; i++) {
			set_leds.request.leds[i].index = i;
			set_leds.request.leds[i].r = r;
			set_leds.request.leds[i].g = g;
			set_leds.request.leds[i].b = b;
		}
		callSetLeds();

	} else if (current_effect.effect == "rainbow") {
		for (int i = 0; i < led_count; i++) {
			int pos = (int)round(counter + (255.0 * i / led_count)) % 255;
			rainbow(pos % 255, r, g, b);
			set_leds.request.leds[i].index = i;
			set_leds.request.leds[i].r = r;
			set_leds.request.leds[i].g = g;
			set_leds.request.leds[i].b = b;
		}
		callSetLeds();
	}
}

void handleTimer(unsigned int generation)
{
	std::lock_guard<std::mutex> lock(state_mutex);
	if (generation != timer_generation) return; // the timer has been restarted
	if (flashing) return; // don't interfere with flash effect
	proceed(node->now());
}

// (re)start the timer with a new period
void startTimer(double period)
{
	if (timer) timer->cancel();
	unsigned int generation = ++timer_generation;
	timer = rclcpp::create_timer(node, node->get_clock(), rclcpp::Duration::from_seconds(period),
	                             [generation]() { handleTimer(generation); }, timer_group); // editorconfig-checker-disable-line
}

// state_mutex should be locked
bool setEffect(SetLEDEffect::Request& req, SetLEDEffect::Response& res)
{
	// effects are set one at a time: wait for flash effect to finish
	while (flashing && rclcpp::ok()) {
		spinSleep(rclcpp::Duration::from_seconds(0.01));
	}

	res.success = true;

	if (req.effect == "") {
		req.effect = "fill";
	}

	if (req.effect != "flash" && req.effect != "fill" && current_effect.effect == req.effect &&
	    current_effect.r == req.r && current_effect.g == req.g && current_effect.b == req.b) {
		res.message = "Effect already set, skip";
		return true;
	}

	if (req.effect == "fill") {
		fill(req.r, req.g, req.b);

	} else if (req.effect == "blink") {
		startTimer(1 / blink_rate);

	} else if (req.effect == "blink_fast") {
		startTimer(1 / blink_fast_rate);

	} else if (req.effect == "fade") {
		startTimer(0.05);

	} else if (req.effect == "wipe") {
		startTimer(wipe_period / led_count);

	} else if (req.effect == "flash") {
		rclcpp::Duration delay = rclcpp::Duration::from_seconds(flash_delay);
		flashing = true;
		fill(0, 0, 0);
		spinSleep(delay);
		fill(req.r, req.g, req.b);
		spinSleep(delay);
		fill(0, 0, 0);
		spinSleep(delay);
		fill(req.r, req.g, req.b);
		spinSleep(delay);
		fill(0, 0, 0);
		spinSleep(delay);
		flashing = false;
		if (current_effect.effect == "fill"||
		    current_effect.effect == "fade" ||
		    current_effect.effect == "wipe") {
			// restore previous filling
			for (int i = 0; i < led_count; i++) {
				fill(current_effect.r, current_effect.g, current_effect.b);
			}
			callSetLeds();
		}
		return true; // this effect happens only once

	} else if (req.effect == "rainbow_fill") {
		startTimer(rainbow_period / 255);

	} else if (req.effect == "rainbow") {
		startTimer(rainbow_period / 255);

	} else {
		res.message = "Unknown effect: " + req.effect + ". Available effects are fill, fade, wipe, blink, blink_fast, flash, rainbow, rainbow_fill.";
		RCLCPP_ERROR(node->get_logger(), "%s", res.message.c_str());
		res.success = false;
		return true;
	}

	// set current effect
	current_effect = req;
	counter = 0;
	start_state = state;
	start_time = node->now();
	proceed(start_time);

	return true;
}

void handleSetEffect(const std::shared_ptr<SetLEDEffect::Request> req, std::shared_ptr<SetLEDEffect::Response> res)
{
	std::lock_guard<std::mutex> lock(state_mutex);
	setEffect(*req, *res);
}

void handleState(const led_msgs::msg::LEDStateArray& msg)
{
	std::lock_guard<std::mutex> lock(state_mutex);
	state = msg;
	led_count = state.leds.size();
	got_state = true;
}

// get parameter of the events table, returns false if it's not set
bool getNotifyParam(const std::string& name, rclcpp::Parameter& param)
{
	if (!node->has_parameter(name)) return false;
	try {
		param = node->get_parameter(name);
	} catch (const rclcpp::exceptions::ParameterUninitializedException&) {
		return false;
	}
	return param.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET;
}

inline int getNotifyColor(const rclcpp::Parameter& param)
{
	return param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER ? param.as_int() : 0;
}

// state_mutex should be locked
void notify(const std::string& event)
{
	rclcpp::Parameter effect_param, r, g, b;
	bool has_effect = getNotifyParam("notify." + event + ".effect", effect_param);
	bool has_r = getNotifyParam("notify." + event + ".r", r);
	bool has_g = getNotifyParam("notify." + event + ".g", g);
	bool has_b = getNotifyParam("notify." + event + ".b", b);

	if (has_effect || has_r || has_g || has_b) {
		RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 5000, "led: notify %s", event.c_str());
		SetLEDEffect::Request request;
		SetLEDEffect::Response response;
		request.effect = effect_param.get_type() == rclcpp::ParameterType::PARAMETER_STRING ? effect_param.as_string() : std::string("");
		request.r = getNotifyColor(r);
		request.g = getNotifyColor(g);
		request.b = getNotifyColor(b);
		setEffect(request, response);
	}
}

// the same rule as ros::names::validate in ROS 1
bool validateName(const std::string& name)
{
	if (name.empty()) return true;

	// first element is special, can be only ~ / or alpha
	char c = name[0];
	if (!isalpha(c) && c != '/' && c != '~') return false;

	for (size_t i = 1; i < name.size(); ++i) {
		c = name[i];
		if (!isalnum(c) && c != '_' && c != '/') return false;
	}
	return true;
}

void handleMavrosState(const mavros_msgs::msg::State& msg)
{
	std::lock_guard<std::mutex> lock(state_mutex);
	if (msg.connected && !mavros_state.connected) {
		notify("connected");
	} else if (!msg.connected && mavros_state.connected) {
		notify("disconnected");
	} else if (msg.armed && !mavros_state.armed) {
		notify("armed");
	} else if (!msg.armed && mavros_state.armed) {
		notify("disarmed");
	} else if (msg.mode != mavros_state.mode) {
		// mode changed
		std::string mode = msg.mode;
		std::transform(mode.begin(), mode.end(), mode.begin(), [](unsigned char c) { return std::tolower(c); });
		if (mode.find(".") != std::string::npos) {
			// remove the part before "."
			mode = mode.substr(mode.find(".") + 1);
		}
		if (validateName(mode)) {
			notify(mode);
		}
	}
	mavros_state = msg;
}

void handleLog(const rcl_interfaces::msg::Log& log)
{
	std::lock_guard<std::mutex> lock(state_mutex);
	if (log.level >= rcl_interfaces::msg::Log::ERROR) {
		// check if ignored
		for (auto const& str : error_ignore) {
			if (log.msg.find(str) != std::string::npos) return;
		}
		notify("error");
	}
}

void handleBattery(const sensor_msgs::msg::BatteryState& msg)
{
	std::lock_guard<std::mutex> lock(state_mutex);
	for (auto const& voltage : msg.cell_voltage) {
		if (voltage < low_battery_threshold &&
		    voltage > 2.0) { // voltage < 2.0 likely indicates incorrect voltage measurement
			// notify low battery every time
			notify("low_battery");
		}
	}
}

// events table: notify.<event>.effect, notify.<event>.r, notify.<event>.g, notify.<event>.b
void declareNotifyParams()
{
	// known events are declared without values: an event is notified only if some of its parameters is set
	for (auto const& event : { "startup", "connected", "disconnected", "armed", "disarmed", "acro", "stabilized",
	                           "altctl", "posctl", "offboard", "low_battery", "error" }) { // editorconfig-checker-disable-line
		const std::string prefix = std::string("notify.") + event;
		node->declare_parameter(prefix + ".effect", rclcpp::ParameterType::PARAMETER_STRING);
		node->declare_parameter(prefix + ".r", rclcpp::ParameterType::PARAMETER_INTEGER);
		node->declare_parameter(prefix + ".g", rclcpp::ParameterType::PARAMETER_INTEGER);
		node->declare_parameter(prefix + ".b", rclcpp::ParameterType::PARAMETER_INTEGER);
	}

	// other events (flight modes), passed on the start
	for (auto& override : node->get_node_parameters_interface()->get_parameter_overrides()) {
		if (override.first.rfind("notify.", 0) == 0 && !node->has_parameter(override.first)) {
			node->declare_parameter(override.first, override.second);
		}
	}
}

// create all the node's entities and spin
void run()
{
	blink_rate = node->declare_parameter("blink_rate", 2.0);
	blink_fast_rate = node->declare_parameter("blink_fast_rate", blink_rate * 2);
	fade_period = node->declare_parameter("fade_period", 0.5);
	wipe_period = node->declare_parameter("wipe_period", 0.5);
	flash_delay = node->declare_parameter("flash_delay", 0.1);
	rainbow_period = node->declare_parameter("rainbow_period", 5.0);

	low_battery_threshold = node->declare_parameter("notify.low_battery.threshold", 3.7);
	error_ignore = node->declare_parameter("notify.error.ignore", std::vector<std::string>{});
	declareNotifyParams();

	std::string led = node->declare_parameter("led", std::string("led")); // led namespace
	if (!led.empty()) led += "/";

	// Callback groups, see threading rules above
	auto service_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
	auto client_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
	auto state_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
	auto events_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
	timer_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
	rclcpp::SubscriptionOptions state_options, events_options;
	state_options.callback_group = state_group;
	events_options.callback_group = events_group;

	set_leds_srv = node->create_client<SetLEDs>(led + "set_leds", rclcpp::ServicesQoS(), client_group);

	// LED driver should publish the state as transient local (latched)
	auto state_sub = node->create_subscription<led_msgs::msg::LEDStateArray>(led + "state", rclcpp::QoS(1).transient_local(),
	                                                                         &handleState, state_options); // editorconfig-checker-disable-line

	// several threads are needed for a callback to wait while other callbacks run
	rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), std::max(4u, std::thread::hardware_concurrency()));
	executor.add_node(node);
	std::thread spinner([&executor]() { executor.spin(); });

	// waiting is done outside of the callbacks
	while (rclcpp::ok() && !set_leds_srv->wait_for_service(std::chrono::milliseconds(100))) {} // cannot work without set_leds service

	// wait for leds count info
	while (rclcpp::ok()) {
		{
			std::lock_guard<std::mutex> lock(state_mutex);
			if (got_state) break;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}

	rclcpp::Service<SetLEDEffect>::SharedPtr set_effect;
	rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr mavros_state_sub;
	rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub;
	rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr rosout_sub;

	if (rclcpp::ok()) {
		// startup is notified before any other event
		std::lock_guard<std::mutex> lock(state_mutex);

		set_effect = node->create_service<SetLEDEffect>(led + "set_effect", &handleSetEffect, rclcpp::ServicesQoS(), service_group);

		// mavros publishes the state as reliable and transient local, the battery as best effort
		mavros_state_sub = node->create_subscription<mavros_msgs::msg::State>("mavros/state", rclcpp::QoS(1).transient_local(),
		                                                                      &handleMavrosState, events_options); // editorconfig-checker-disable-line
		battery_sub = node->create_subscription<sensor_msgs::msg::BatteryState>("mavros/battery", rclcpp::QoS(1).best_effort(),
		                                                                        &handleBattery, events_options); // editorconfig-checker-disable-line
		// volatile: don't get the errors happened before the start
		rosout_sub = node->create_subscription<rcl_interfaces::msg::Log>("/rosout", rclcpp::QoS(1), &handleLog, events_options);

		RCLCPP_INFO(node->get_logger(), "ready");
		notify("startup");
	}

	spinner.join();
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	node = std::make_shared<rclcpp::Node>("led");

	run();

	// destroy global ROS entities before the context and the middleware are destroyed
	timer.reset();
	timer_group.reset();
	set_leds_srv.reset();
	node.reset();

	rclcpp::shutdown();
	return 0;
}
