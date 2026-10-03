/*
 * Detecting and pose estimation of ArUco markers
 * Copyright (C) 2018 Copter Express Technologies
 *
 * Author: Oleg Kalachev <okalachev@gmail.com>
 *
 * Distributed under MIT License (available at https://opensource.org/licenses/MIT).
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 */

/*
 * Code is based on https://github.com/UbiquityRobotics/fiducials, which is distributed
 * under the BSD license.
 */

#include <math.h>
#include <vector>
#include <string>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <memory>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <aruco_pose/msg/marker.hpp>
#include <aruco_pose/msg/marker_array.hpp>
#include <aruco_pose/srv/set_markers.hpp>

#include "draw.h"
#include "utils.h"

using std::vector;
using cv::Mat;

namespace aruco_pose
{

class ArucoDetect : public rclcpp::Node {
private:
	std::unique_ptr<tf2_ros::TransformBroadcaster> br_;
	std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
	std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
	OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
	std::unordered_map<std::string, std::function<void(const rclcpp::Parameter&)>> detector_params_;
	bool enabled_param_ = true;
	bool enabled_ = true;
	cv::Ptr<cv::aruco::Dictionary> dictionary_;
	cv::Ptr<cv::aruco::DetectorParameters> parameters_;
	image_transport::Publisher debug_pub_;
	image_transport::CameraSubscriber img_sub_;
	rclcpp::Publisher<aruco_pose::msg::MarkerArray>::SharedPtr markers_pub_;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vis_markers_pub_;
	rclcpp::Subscription<aruco_pose::msg::MarkerArray>::SharedPtr map_markers_sub_;
	rclcpp::Service<aruco_pose::srv::SetMarkers>::SharedPtr set_markers_srv_;
	bool estimate_poses_, send_tf_, flip_vertical_, auto_flip_, use_map_markers_;
	bool waiting_for_map_;
	double length_ = 0;
	tf2::Duration transform_timeout_;
	std::unordered_map<int, double> length_override_;
	std::string frame_id_prefix_, known_vertical_;
	Mat camera_matrix_, dist_coeffs_;
	aruco_pose::msg::MarkerArray array_;
	std::unordered_set<int> map_markers_ids_;
	visualization_msgs::msg::MarkerArray vis_array_;

public:
	explicit ArucoDetect(const rclcpp::NodeOptions& options) : rclcpp::Node("aruco_detect", options)
	{
		br_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
		tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
		tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, this);

		int dictionary = declare_parameter("dictionary", 2);
		estimate_poses_ = declare_parameter("estimate_poses", true);
		send_tf_ = declare_parameter("send_tf", true);
		use_map_markers_ = declare_parameter("use_map_markers", false);
		waiting_for_map_ = use_map_markers_;
		transform_timeout_ = tf2::durationFromSec(declare_parameter("transform_timeout", 0.02));

		known_vertical_ = declare_parameter("known_tilt", std::string("")); // known_tilt is an old name
		known_vertical_ = declare_parameter("known_vertical", known_vertical_);
		flip_vertical_ = declare_parameter("flip_vertical", false);
		auto_flip_ = declare_parameter("auto_flip", false);

		frame_id_prefix_ = declare_parameter("frame_id_prefix", std::string("aruco_"));

		camera_matrix_ = cv::Mat::zeros(3, 3, CV_64F);

		dictionary_ = cv::aruco::getPredefinedDictionary(static_cast<cv::aruco::PREDEFINED_DICTIONARY_NAME>(dictionary));
		parameters_ = cv::aruco::DetectorParameters::create();

		declareDetectorParameters();
		readLengthOverride();

		auto overrides = get_node_parameters_interface()->get_parameter_overrides();
		if (estimate_poses_ && overrides.find("length") == overrides.end()) {
			RCLCPP_FATAL(get_logger(), "can't estimate marker's poses as ~length parameter is not defined");
			rclcpp::shutdown();
		}

		param_cb_handle_ = add_on_set_parameters_callback(
			std::bind(&ArucoDetect::paramCallback, this, std::placeholders::_1));

		set_markers_srv_ = create_service<aruco_pose::srv::SetMarkers>("~/set_length_override",
			std::bind(&ArucoDetect::setMarkers, this, std::placeholders::_1, std::placeholders::_2));

		debug_pub_ = image_transport::create_publisher(this, "~/debug", rmw_qos_profile_default);
		markers_pub_ = create_publisher<aruco_pose::msg::MarkerArray>("~/markers", 1);
		vis_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/visualization", 1);
		img_sub_ = image_transport::create_camera_subscription(this, "image_raw",
			std::bind(&ArucoDetect::imageCallback, this, std::placeholders::_1, std::placeholders::_2),
			"raw", rmw_qos_profile_sensor_data);
		map_markers_sub_ = create_subscription<aruco_pose::msg::MarkerArray>("map_markers",
			rclcpp::QoS(1).transient_local(),
			std::bind(&ArucoDetect::mapMarkersCallback, this, std::placeholders::_1));

		RCLCPP_INFO(get_logger(), "ready");
	}

private:
	void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg,
	                   const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cinfo)
	{
		if (!enabled_) return;
		if (waiting_for_map_) return;

		Mat image = cv_bridge::toCvShare(msg)->image;

		vector<int> ids;
		vector<vector<cv::Point2f>> corners, rejected;
		vector<cv::Vec3d> rvecs, tvecs;
		vector<cv::Point3f> obj_points;
		geometry_msgs::msg::TransformStamped vertical;

		// Detect markers
		cv::aruco::detectMarkers(image, dictionary_, corners, ids, parameters_, rejected);

		array_.header.stamp = msg->header.stamp;
		array_.header.frame_id = msg->header.frame_id;
		array_.markers.clear();

		if (ids.size() != 0) {
			parseCameraInfo(cinfo, camera_matrix_, dist_coeffs_);

			// Estimate individual markers' poses
			if (estimate_poses_) {
				cv::aruco::estimatePoseSingleMarkers(corners, length_, camera_matrix_, dist_coeffs_,
				                                     rvecs, tvecs);

				// process length override, TODO: efficiency
				if (!length_override_.empty()) {
					for (unsigned int i = 0; i < ids.size(); i++) {
						int id = ids[i];
						auto item = length_override_.find(id);
						if (item != length_override_.end()) { // found override
							vector<cv::Vec3d> rvecs_current, tvecs_current;
							vector<vector<cv::Point2f>> corners_current;
							corners_current.push_back(corners[i]);
							cv::aruco::estimatePoseSingleMarkers(corners_current, item->second,
							                                     camera_matrix_, dist_coeffs_,
							                                     rvecs_current, tvecs_current);
							rvecs[i] = rvecs_current[0];
							tvecs[i] = tvecs_current[0];
						}
					}
				}

				if (!known_vertical_.empty()) {
					try {
						vertical = tf_buffer_->lookupTransform(msg->header.frame_id, known_vertical_,
						                                       tf2_ros::fromMsg(msg->header.stamp), transform_timeout_);
					} catch (const tf2::TransformException& e) {
						RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "can't retrieve known vertical: %s", e.what());
					}
				}
			}

			array_.markers.reserve(ids.size());
			aruco_pose::msg::Marker marker;
			vector<geometry_msgs::msg::TransformStamped> transforms;
			transforms.reserve(ids.size());
			geometry_msgs::msg::TransformStamped transform;
			transform.header.stamp = msg->header.stamp;
			transform.header.frame_id = msg->header.frame_id;

			for (unsigned int i = 0; i < ids.size(); i++) {
				marker.id = ids[i];
				marker.length = getMarkerLength(marker.id);
				fillCorners(marker, corners[i]);

				if (estimate_poses_) {
					fillPose(marker.pose, rvecs[i], tvecs[i]);

					// apply known vertical (if enabled and vertical frame available)
					if (!known_vertical_.empty() && !vertical.header.frame_id.empty()) {
						applyVertical(marker.pose.orientation, vertical.transform.rotation, false, auto_flip_);
					}

					if (send_tf_) {
						transform.child_frame_id = getChildFrameId(ids[i]);

						// check if such static transform is in the map
						if (map_markers_ids_.find(ids[i]) == map_markers_ids_.end()) {
							// check if a markers with that id is already added
							bool send = true;
							for (auto &t : transforms) {
								if (t.child_frame_id == transform.child_frame_id) {
									send = false;
									break;
								}
							}
							if (send) {
								transform.transform.rotation = marker.pose.orientation;
								fillTranslation(transform.transform.translation, tvecs[i]);
								transforms.push_back(transform);
							}
						}
					}
				}
				array_.markers.push_back(marker);
			}

			if (send_tf_) {
				br_->sendTransform(transforms);
			}
		}

		markers_pub_->publish(array_);

		// Publish visualization markers
		if (estimate_poses_ && vis_markers_pub_->get_subscription_count() != 0) {
			// Delete all markers
			visualization_msgs::msg::Marker vis_marker;
			vis_marker.action = visualization_msgs::msg::Marker::DELETEALL;
			vis_array_.markers.clear();
			vis_array_.markers.reserve(ids.size() + 1);
			vis_array_.markers.push_back(vis_marker);

			for (unsigned int i = 0; i < ids.size(); i++)
				pushVisMarkers(msg->header.frame_id, msg->header.stamp, array_.markers[i].pose,
				               getMarkerLength(ids[i]), ids[i], i);

			vis_markers_pub_->publish(vis_array_);
		}

		// Publish debug image
		if (debug_pub_.getNumSubscribers() != 0) {
			Mat debug = image.clone();
			cv::aruco::drawDetectedMarkers(debug, corners, ids); // draw markers
			if (estimate_poses_)
				for (unsigned int i = 0; i < ids.size(); i++)
					_drawAxis(debug, camera_matrix_, dist_coeffs_, rvecs[i], tvecs[i], getMarkerLength(ids[i]));

			cv_bridge::CvImage out_msg;
			out_msg.header.frame_id = msg->header.frame_id;
			out_msg.header.stamp = msg->header.stamp;
			out_msg.encoding = sensor_msgs::image_encodings::BGR8;
			out_msg.image = debug;
			debug_pub_.publish(out_msg.toImageMsg());
		}
	}

	inline void fillCorners(aruco_pose::msg::Marker& marker, const vector<cv::Point2f>& corners) const
	{
		marker.c1.x = corners[0].x;
		marker.c2.x = corners[1].x;
		marker.c3.x = corners[2].x;
		marker.c4.x = corners[3].x;
		marker.c1.y = corners[0].y;
		marker.c2.y = corners[1].y;
		marker.c3.y = corners[2].y;
		marker.c4.y = corners[3].y;
	}

	inline void fillPose(geometry_msgs::msg::Pose& pose, const cv::Vec3d& rvec, const cv::Vec3d& tvec) const
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

	inline void fillTranslation(geometry_msgs::msg::Vector3& translation, const cv::Vec3d& tvec) const
	{
		translation.x = tvec[0];
		translation.y = tvec[1];
		translation.z = tvec[2];
	}

	void pushVisMarkers(const std::string& frame_id, const builtin_interfaces::msg::Time& stamp,
	                    const geometry_msgs::msg::Pose &pose, double length, int id, int index)
	{
		visualization_msgs::msg::Marker marker;
		marker.header.frame_id = frame_id;
		marker.header.stamp = stamp;
		marker.action = visualization_msgs::msg::Marker::ADD;
		marker.id = index;

		// Marker
		marker.ns = "aruco_marker";
		marker.type = visualization_msgs::msg::Marker::CUBE;
		marker.scale.x = length;
		marker.scale.y = length;
		marker.scale.z = 0.001;
		marker.color.r = 1;
		marker.color.g = 1;
		marker.color.b = 1;
		marker.color.a = 0.9;
		marker.pose = pose;
		vis_array_.markers.push_back(marker);

		// Label
		marker.ns = "aruco_marker_label";
		marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
		marker.scale.z = length * 0.6;
		marker.color.r = 0;
		marker.color.g = 0;
		marker.color.b = 0;
		marker.color.a = 1;
		marker.text = std::to_string(id);
		marker.pose = pose;
		vis_array_.markers.push_back(marker);
	}

	inline std::string getChildFrameId(int id) const
	{
		return frame_id_prefix_ + std::to_string(id);
	}

	void readLengthOverride()
	{
		// length_override.<id> parameters (ROS 2 uses '.' as a namespace separator)
		static const std::string prefix = "length_override.";
		for (auto const& item : get_node_parameters_interface()->get_parameter_overrides()) {
			if (item.first.compare(0, prefix.size(), prefix) != 0) continue;
			double value = declare_parameter(item.first, 0.0);
			length_override_[std::stoi(item.first.substr(prefix.size()))] = value;
		}
	}

	inline double getMarkerLength(int id)
	{
		auto item = length_override_.find(id);
		if (item != length_override_.end()) {
			return item->second;
		} else {
			return length_;
		}
	}

	void setMarkers(const std::shared_ptr<aruco_pose::srv::SetMarkers::Request> req,
	                std::shared_ptr<aruco_pose::srv::SetMarkers::Response> res)
	{
		for (auto const& marker : req->markers) {
			if (marker.id > 999) {
				res->message = "Invalid marker id: " + std::to_string(marker.id);
				RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
				return;
			}
			if (!std::isfinite(marker.length) || marker.length <= 0) {
				res->message = "Invalid marker " + std::to_string(marker.id) + " length: " + std::to_string(marker.length);
				RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
				return;
			}
		}

		for (auto const& marker : req->markers) {
			length_override_[marker.id] = marker.length;
		}

		res->success = true;
	}

	void mapMarkersCallback(const aruco_pose::msg::MarkerArray::ConstSharedPtr msg)
	{
		map_markers_ids_.clear();
		for (auto const& marker : msg->markers) {
			map_markers_ids_.insert(marker.id);
			if (use_map_markers_) {
				if (length_override_.find(marker.id) == length_override_.end()) {
					length_override_[marker.id] = marker.length;
				}
			}
		}
		waiting_for_map_ = false;
	}

	// Parameter helpers: declare a parameter with a range (as in the former Detector.cfg)
	// and register a function that applies its value to the detector parameters
	template<typename F>
	void addDoubleParam(const std::string& name, double def, double min, double max,
	                    const std::string& description, F apply)
	{
		rcl_interfaces::msg::ParameterDescriptor descriptor;
		descriptor.description = description;
		descriptor.floating_point_range.resize(1);
		descriptor.floating_point_range[0].from_value = min;
		descriptor.floating_point_range[0].to_value = max;
		apply(declare_parameter(name, def, descriptor));
		detector_params_[name] = [apply](const rclcpp::Parameter& p) { apply(p.as_double()); };
	}

	template<typename F>
	void addIntParam(const std::string& name, int def, int min, int max,
	                 const std::string& description, F apply)
	{
		rcl_interfaces::msg::ParameterDescriptor descriptor;
		descriptor.description = description;
		descriptor.integer_range.resize(1);
		descriptor.integer_range[0].from_value = min;
		descriptor.integer_range[0].to_value = max;
		apply(declare_parameter(name, def, descriptor));
		detector_params_[name] = [apply](const rclcpp::Parameter& p) { apply(p.as_int()); };
	}

	template<typename F>
	void addBoolParam(const std::string& name, bool def, const std::string& description, F apply)
	{
		rcl_interfaces::msg::ParameterDescriptor descriptor;
		descriptor.description = description;
		apply(declare_parameter(name, def, descriptor));
		detector_params_[name] = [apply](const rclcpp::Parameter& p) { apply(p.as_bool()); };
	}

#define DOUBLE_PARAM(field, min, max, description) \
	addDoubleParam(#field, parameters_->field, min, max, description, \
	               [this](double v) { parameters_->field = v; })
#define INT_PARAM(field, min, max, description) \
	addIntParam(#field, parameters_->field, min, max, description, \
	            [this](int v) { parameters_->field = v; })

	void declareDetectorParameters()
	{
		// length is required if poses are estimated; it stays unset (0) otherwise
		rcl_interfaces::msg::ParameterDescriptor length_descriptor;
		length_descriptor.description = "markers' side length";
		length_descriptor.floating_point_range.resize(1);
		length_descriptor.floating_point_range[0].from_value = 0;
		length_descriptor.floating_point_range[0].to_value = 10;
		length_ = declare_parameter("length", 0.0, length_descriptor);

		rcl_interfaces::msg::ParameterDescriptor enabled_descriptor;
		enabled_descriptor.description = "if detection enabled";
		enabled_param_ = declare_parameter("enabled", true, enabled_descriptor);
		enabled_ = enabled_param_ && length_ > 0;

		DOUBLE_PARAM(adaptiveThreshConstant, 0, 100,
			"Constant for adaptive thresholding before finding contours");
		INT_PARAM(adaptiveThreshWinSizeMin, 1, 100,
			"Minimum window size for adaptive thresholding before finding contours");
		INT_PARAM(adaptiveThreshWinSizeMax, 1, 100,
			"Maximum window size for adaptive thresholding before finding contours");
		INT_PARAM(adaptiveThreshWinSizeStep, 1, 100,
			"Increments from adaptiveThreshWinSizeMin to adaptiveThreshWinSizeMax during the thresholding");
		INT_PARAM(cornerRefinementMaxIterations, 1, 1000,
			"Maximum number of iterations for stop criteria of the corner refinement process");
		addIntParam("cornerRefinementMethod", 0, 0, 3,
			"Corner refinement method: 0 - none, 1 - subpixel, 2 - contour, 3 - AprilTag2",
			[this](int v) { parameters_->cornerRefinementMethod = v; });
		DOUBLE_PARAM(cornerRefinementMinAccuracy, 0, 1,
			"Minimum error for the stop criteria of the corner refinement process");
		INT_PARAM(cornerRefinementWinSize, 1, 100,
			"Window size for the corner refinement process (in pixels)");
		addBoolParam("detectInvertedMarker", false,
			"check if there is a white marker. In order to generate a 'white' marker just invert a normal marker by using a tilde",
			[this](bool v) { parameters_->detectInvertedMarker = v; });
		DOUBLE_PARAM(errorCorrectionRate, 0, 1,
			"Error correction rate respect to the maximum error correction capability for each dictionary");
		DOUBLE_PARAM(minCornerDistanceRate, 0, 0.25,
			"Minimum distance between corners for detected markers relative to its perimeter");
		INT_PARAM(markerBorderBits, 1, 10,
			"Number of bits of the marker border, i.e. marker border width");
		DOUBLE_PARAM(maxErroneousBitsInBorderRate, 0, 1,
			"Maximum number of accepted erroneous bits in the border (i.e. number of allowed white bits in the border)");
		INT_PARAM(minDistanceToBorder, 0, 1000,
			"Minimum distance of any corner to the image border for detected markers (in pixels)");
		DOUBLE_PARAM(minMarkerDistanceRate, 0, 1,
			"minimum mean distance beetween two marker corners to be considered similar, so that the smaller one is removed. The rate is relative to the smaller perimeter of the two markers");
		DOUBLE_PARAM(minMarkerPerimeterRate, 0, 4,
			"Determine minimum perimeter for marker contour to be detected. This is defined as a rate respect to the maximum dimension of the input image");
		DOUBLE_PARAM(maxMarkerPerimeterRate, 0, 4,
			"Determine maximum perimeter for marker contour to be detected. This is defined as a rate respect to the maximum dimension of the input image");
		DOUBLE_PARAM(minOtsuStdDev, 0, 100,
			"Minimun standard deviation in pixels values during the decodification step to apply Otsu thresholding (otherwise, all the bits are set to 0 or 1 depending on mean higher than 128 or not)");
		DOUBLE_PARAM(perspectiveRemoveIgnoredMarginPerCell, 0, 1,
			"Width of the margin of pixels on each cell not considered for the determination of the cell bit. Represents the rate respect to the total size of the cell, i.e. perpectiveRemovePixelPerCell");
		INT_PARAM(perspectiveRemovePixelPerCell, 1, 100,
			"Number of bits (per dimension) for each cell of the marker when removing the perspective");
		DOUBLE_PARAM(polygonalApproxAccuracyRate, 0, 1,
			"Minimum accuracy during the polygonal approximation process to determine which contours are squares");
		addDoubleParam("aprilTagQuadDecimate", 0, 0, 1000,
			"Detection of quads can be done on a lower-resolution image, improving speed at a cost of pose accuracy and a slight decrease in detection rate. Decoding the binary payload is still done at full resolution",
			[this](double v) { parameters_->aprilTagQuadDecimate = v; });
		addDoubleParam("aprilTagQuadSigma", 0, 0, 1000,
			"What Gaussian blur should be applied to the segmented image (used for quad detection?) Parameter is the standard deviation in pixels. Very noisy images benefit from non-zero values",
			[this](double v) { parameters_->aprilTagQuadSigma = v; });
	}

#undef DOUBLE_PARAM
#undef INT_PARAM

	rcl_interfaces::msg::SetParametersResult paramCallback(const std::vector<rclcpp::Parameter>& params)
	{
		rcl_interfaces::msg::SetParametersResult result;
		result.successful = true;

		for (auto const& param : params) {
			if (param.get_name() == "enabled") {
				enabled_param_ = param.as_bool();
			} else if (param.get_name() == "length") {
				length_ = param.as_double();
			} else {
				auto item = detector_params_.find(param.get_name());
				if (item != detector_params_.end()) item->second(param);
			}
		}
		enabled_ = enabled_param_ && length_ > 0;

		return result;
	}
};

} // namespace aruco_pose

RCLCPP_COMPONENTS_REGISTER_NODE(aruco_pose::ArucoDetect)
