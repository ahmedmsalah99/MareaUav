#pragma once

// ros
#include "lcm/lcm-cpp.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include <apriltag_msgs/msg/april_tag_detection.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <lcm_messages/geometry/vision.hpp>
// apriltag
#include <apriltag.h>
#include <apriltag_pose.h>


#include <common/pjpeg.h>
#include "RescueTag.h"

#include <vector>
#include <queue>
#include <Eigen/Geometry>
#include <Eigen/Core>

class AprilTag2RescueNode : public rclcpp::Node {
public:
	AprilTag2RescueNode();
	~AprilTag2RescueNode();

private:
	typedef Eigen::Matrix<double, 3, 3, Eigen::RowMajor> Mat3;

	apriltag_family_t *tf;
	apriltag_detector_t *td;
	std::string tag_family;
	double tag_edge_size;
	int max_hamming;
	int n_tags;
	geometry::vision _drone_pose;
	std::map<int, std::string> tracked_tags;
	std::map<int, float> rescue_tags;
	lcm::LCM _handler;
	typedef std::queue<double> queue_t;
	queue_t _Queue_roll;
	queue_t _Queue_pitch;
	queue_t _Queue_yaw;
	double _integral_roll = 0;
	double _integral_pitch = 0;
	double _integral_yaw = 0;
	double _roll;
	double _pitch;
	double _yaw;
	// counter threshold when computing the errors between drone and tags
	double _iter_err_thld = 0;
	// threshold for each computed error, loaded from configurations file
	double _x_err_thld;
	double _y_err_thld;
	double _z_err_thld;
	// counter for each computed error
	double _x_err_count = 0;
	double _y_err_count = 0;
	double _z_err_count = 0;

	double _previous_x_err = 0;
	double _previous_y_err = 0;
	double _previous_z_err = 0;

	Mat3 K;
	bool z_up;

	// function pointer for tag family creation / destruction
	static std::map<std::string, apriltag_family_t *(*)(void)> tag_create;
	static std::map<std::string, void (*)(apriltag_family_t*)> tag_destroy;

	rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_img;
	rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_info;
	rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_sub;
	rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr pub_tf;
	rclcpp::Publisher<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr pub_detections;

	rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr pub_rescuePosition;

	void onImage(const sensor_msgs::msg::CompressedImage::SharedPtr msg_img);
	void getDroneOdometry(const px4_msgs::msg::VehicleOdometry::SharedPtr msg_odom);
	void getPose(const matd_t& H, geometry_msgs::msg::Transform& t, const int ID, const bool z_up = false);
	double movingFilter(double raw, std::queue<double> *queue_t, double *integral);
	void toEulerAngle(double quaternion[], double *Roll, double *Pitch, double *Yaw);
	void publish_data_center(const apriltag_msgs::msg::AprilTagDetectionArray& Detections, const tf2_msgs::msg::TFMessage& t_f);

};
