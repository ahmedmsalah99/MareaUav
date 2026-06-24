#pragma once

// ros
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include <apriltag_msgs/msg/april_tag_detection.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
// apriltag
#include <apriltag.h>
#include <apriltag_pose.h>
#include <opencv2/calib3d.hpp>

// 
// #include <rclcpp/time.hpp>
// #include <rclcpp/clock.hpp>

#include <common/pjpeg.h>
#include "LandingPlatform.h"

#include <vector>
#include <queue>
#include <deque>
#include <Eigen/Geometry>
#include <Eigen/Core>

class AprilTag2Node : public rclcpp::Node {
public:
	AprilTag2Node();
	~AprilTag2Node();

private:
	typedef Eigen::Matrix<double, 3, 3, Eigen::RowMajor> Mat3;

	apriltag_family_t *_tagFamily;
	apriltag_detector_t *_tagDetector;
	std::string tag_family;
	double tag_edge_size;
	int max_hamming;
	int n_tags;
	geometry_msgs::msg::TransformStamped _drone_pose;
	double _drone_yaw;
	std::map<int, std::string> tracked_tags;
	std::map<int, float> platform_tags;
	typedef std::queue<double> queue_t;
	queue_t _Queue_roll;
	queue_t _Queue_pitch;
	queue_t _Queue_yaw;
	queue_t _Queue_x;
	queue_t _Queue_y;
	double _integral_roll = 0;
	double _integral_pitch = 0;
	double _integral_yaw = 0;
	double _integral_x = 0;
	double _integral_y = 0;
	double _roll;
	double _pitch;
	double _yaw;
	double _x;
	double _y;
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
	std::array<double, 4> intrinsics;
	bool z_up;

	// Member variables
	double _filtered_vision_pos[3] = {0.0, 0.0, 0.0};
	bool _first_vision_run = true;
	rclcpp::Time _last_vision_time;

	// Filter configuration (Adjust cutoff as needed)
	const double _vision_cutoff_freq = 0.05;
	const double _max_vision_velocity = 2.0;  // m/s — reject if position jumps faster than this

	// function pointer for tag family creation / destruction
	static std::map<std::string, apriltag_family_t *(*)(void)> tag_create;
	static std::map<std::string, void (*)(apriltag_family_t*)> tag_destroy;

	rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_img;
	rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_info;
	rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_sub;
	rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr pub_tf;
	rclcpp::Publisher<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr pub_detections;

	rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr pub_platformCenter;
	rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr pub_platformCenter_gt;

	void onImage(const sensor_msgs::msg::CompressedImage::SharedPtr msg_img);
	void getDroneOdometry(const px4_msgs::msg::VehicleOdometry::SharedPtr msg_odom);
	void getPose(const matd_t& H, geometry_msgs::msg::Transform& t, const int ID, const bool z_up = false);
	double movingFilter(double raw, std::queue<double> *queue_t, double *integral);
	void toEulerAngle(double quaternion[], double *Roll, double *Pitch, double *Yaw);
	void publish_data_center(const apriltag_msgs::msg::AprilTagDetectionArray& detections, const tf2_msgs::msg::TFMessage& transforms);
	geometry_msgs::msg::Transform
	// homography(apriltag_detection_t* const detection, const std::array<double, 4>& intr, double tagsize);
	pnp(apriltag_detection_t* const detection, const std::array<double, 4>& intr, double tagsize, int tag_id);
	// Previous tvec per tag for IPPE solution disambiguation (temporal consistency).
	std::map<int, cv::Mat> _prev_tvec_;
	// ... inside your class ...
	std::deque<double> _buffer_x, _buffer_y, _buffer_z;
	const size_t _median_window_size = 5; // Use an odd number (5-9 is usually best)

	double getMedian(std::deque<double> buffer);
	geometry_msgs::msg::Transform
	toMsg(const std::pair<cv::Mat_<double>, cv::Mat_<double>>& pose);
	void convert(const Eigen::Quaterniond& eigen_quat, geometry_msgs::msg::Quaternion& msg_quat);
	void convert(const cv::Mat_<double>& vec, geometry_msgs::msg::Vector3& msg_vec);
};
