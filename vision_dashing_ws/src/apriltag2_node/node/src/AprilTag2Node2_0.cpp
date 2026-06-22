#include <AprilTag2Node.hpp>
#include <class_loader/register_macro.hpp>
#include "LandingPlatform.h"
#include <sstream>
#include <iostream>
#include <fstream>
// default tag families
#include <tag16h5.h>
#include <tag25h9.h>
#include <tag36h11.h>
#include <math.h>
#include <Eigen/Dense>
#include <cstdlib>
static Platform_tag TAGS;

std::ofstream detectionLog;

AprilTag2Node::AprilTag2Node() : Node("apriltag2", "apriltag", rclcpp::NodeOptions().use_intra_process_comms(true))
{
	// using then the topic name to create a subscription
	sub_img = this->create_subscription<sensor_msgs::msg::CompressedImage>("/apriltag/image/compressed/compressed",
		rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history,rmw_qos_profile_sensor_data.depth),rmw_qos_profile_sensor_data),
		std::bind(&AprilTag2Node::onImage, this, std::placeholders::_1));
	rmw_qos_profile_t tf_qos_profile = rmw_qos_profile_default;
	tf_qos_profile.depth = 1;


	odometry_sub = this->create_subscription<px4_msgs::msg::VehicleOdometry>("/fmu/out/vehicle_odometry",
		rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history,rmw_qos_profile_sensor_data.depth),rmw_qos_profile_sensor_data),
		std::bind(&AprilTag2Node::getDroneOdometry, this, std::placeholders::_1));
	pub_platformCenter = this->create_publisher<geometry_msgs::msg::TransformStamped>("/platform_center",1);
	pub_platformCenter_gt = this->create_publisher<geometry_msgs::msg::TransformStamped>("/platform_center_gt",1);

	// calibration values for 640x480
	const double cx = 323.83127;
	const double cy = 248.651286;
	const double fx = 381.36246688113556;
	const double fy = 381.36246688113556;

	K << fx, 0, cx, 0, fy, cy, 0, 0, 1;
	intrinsics = {fx, fy, cx,cy};
	//std::cout<<"K: "<<K<<std::endl;
	// getting the value of the environment variable, to check whenever we are running in SITL or in a RPi
	char *env_val;
	env_val = std::getenv("M2M_SIM");
	bool env_bool;

	if(atoi(env_val) >= 1)
		env_bool = 1;
	else
		env_bool = 0;

	std::cout << "M2M_SIM IS " << env_bool << std::endl;

	declare_parameter<std::string>("family", "36h11");
	declare_parameter<double>("size", 2.0);
	declare_parameter<int>("max_hamming", 0);
	declare_parameter<bool>("z_up", false);
	get_parameter("family", tag_family);
	get_parameter("size", tag_edge_size);
	get_parameter("max_hamming", max_hamming);
	get_parameter("z_up", z_up);


	static const std::string tag_prefix = "tag_";
	declare_parameter<int>("n_tags", 0);
	get_parameter("n_tags", n_tags);

	for(int i=0; i<n_tags; i++)
	{
		std::string tag_name = tag_prefix+std::to_string(i);
		declare_parameter<double>(tag_name, 2.0);
		get_parameter(tag_name, platform_tags[i]);
		// tracked_tags[i+1]=i+1;
	}

	SetMatrix(&TAGS, env_bool); //load transformation matrices respect to the platform center


	if(!tag_create.count(tag_family)) {
		throw std::runtime_error("unsupported tag family: " + tag_family);
	}
	_tagFamily = tag_create.at(tag_family)();
	_tagDetector = apriltag_detector_create();

	declare_parameter<float>("decimate", 1.0);
	declare_parameter<float>("blur", 0.0);
	declare_parameter<int>("threads", 1);
	declare_parameter<int>("debug", 0);
	declare_parameter<int>("refine", 1);
	declare_parameter<double>("x_err_thld", 100.0);
	declare_parameter<double>("y_err_thld", 100.0);
	declare_parameter<double>("z_err_thld", 100.0);
	declare_parameter<double>("iter_err_thld", 10.0);

	get_parameter("decimate", _tagDetector->quad_decimate);
	get_parameter("blur", _tagDetector->quad_sigma);
	get_parameter("threads", _tagDetector->nthreads);
	get_parameter("debug", _tagDetector->debug);
	get_parameter("refine-edges", _tagDetector->refine_edges);
	get_parameter("x_err_thld", _x_err_thld);
	get_parameter("y_err_thld", _y_err_thld);
	get_parameter("z_err_thld", _z_err_thld);
	get_parameter("iter_err_thld", _iter_err_thld);

	apriltag_detector_add_family(_tagDetector, _tagFamily);


	detectionLog.open("matlab_data_sim.txt");
	detectionLog << "det_tags,horiz_dist,vert_dist,timestamp\n";

}

AprilTag2Node::~AprilTag2Node()
{
	apriltag_detector_destroy(_tagDetector);
	tag_destroy.at(tag_family)(_tagFamily);
	detectionLog.close();
}

void AprilTag2Node::onImage(const sensor_msgs::msg::CompressedImage::SharedPtr msg_img)
{
	// decode image
	image_u8_t* im = nullptr;

	if(msg_img->format=="rgb8; jpeg compressed bgr8") {
		// convert jpeg data
		int err = 0;
		pjpeg_t *pj = pjpeg_create_from_buffer(msg_img->data.data(), msg_img->data.size(), 0, &err);
		if (pj == nullptr) {
			RCLCPP_ERROR(get_logger(), "pjpeg error");
			return;
		}

		im = pjpeg_to_u8_baseline(pj);

		pjpeg_destroy(pj);
	} else {
		RCLCPP_ERROR(get_logger(), "not supported: %s",  msg_img->format.c_str());
		return;
	}

	if (im == nullptr) {
		RCLCPP_ERROR(get_logger(), "could not load image");
		return;
	}

	// decode image
	zarray_t* detections = apriltag_detector_detect(_tagDetector, im);

	apriltag_msgs::msg::AprilTagDetectionArray msg_detections;
	msg_detections.header = msg_img->header;

	tf2_msgs::msg::TFMessage tfs;

	for (int i = 0; i < zarray_size(detections); i++)
	{
		apriltag_detection_t *det;
		zarray_get(detections, i, &det);

		// ignore untracked tags
		if (tracked_tags.size() > 0 && !tracked_tags.count(det->id)) { continue; }

		// reject detections with more corrected bits than allowed
		if (det->hamming>max_hamming) { continue; }

		// detection
		apriltag_msgs::msg::AprilTagDetection msg_detection;
		msg_detection.family = std::string(det->family->name);
		msg_detection.id = det->id;
		msg_detection.hamming = det->hamming;
		msg_detection.decision_margin = det->decision_margin;
		msg_detection.centre.x = det->c[0];
		msg_detection.centre.y = det->c[1];
		std::memcpy(msg_detection.corners.data(), det->p, sizeof(double) * 8);
		std::memcpy(msg_detection.homography.data(), det->H->data, sizeof(double) * 9);
		

		// 3D orientation and position
		geometry_msgs::msg::TransformStamped tagTransform;
		tagTransform.header = msg_img->header;
		// set child frame name by generic tag name or configured tag name
		tagTransform.child_frame_id = tracked_tags.size() ? tracked_tags.at(det->id) : std::string(det->family->name) + ":" + std::to_string(det->id);

		//This function computes the relative translation and rotation of each tag in the scene w.r.t the camera.
		//Then, it projects this information in the centre of the platform (related to the tag in the centre).
		try{
			tagTransform.transform = pnp(det, intrinsics, platform_tags[det->id], det->id);
		}catch (const cv::Exception& e) {
			continue;
		}
		if(std::isnan(tagTransform.transform.translation.x) || std::isnan(tagTransform.transform.translation.y) || std::isnan(tagTransform.transform.translation.z)) continue;
		if(std::abs(tagTransform.transform.translation.x) > 20 || std::abs(tagTransform.transform.translation.y) > 20 || std::abs(tagTransform.transform.translation.z) > 10) continue;
		msg_detections.detections.push_back(msg_detection);
		tfs.transforms.push_back(tagTransform);

	}
	if(zarray_size(detections) > 0 && msg_detections.detections.size() > 0) {
		publish_data_center( msg_detections,tfs );
	}


	apriltag_detections_destroy(detections);

	image_u8_destroy(im);
}


void AprilTag2Node::publish_data_center(const apriltag_msgs::msg::AprilTagDetectionArray& detections, const tf2_msgs::msg::TFMessage& transforms)
{
	if(detections.detections.size()>0)
	{
		Eigen::MatrixXd tagAreaWeights(detections.detections.size(), 2);
		geometry_msgs::msg::TransformStamped msg_relative_pose;
		geometry_msgs::msg::TransformStamped vision_pose; // ROS2-friendly internal container
		// vision_pose.pose is not used; we store position/orientation in the transform fields.
		vision_pose.transform.rotation.w = 1.0;
		vision_pose.transform.rotation.x = 0.0;
		vision_pose.transform.rotation.y = 0.0;
		vision_pose.transform.rotation.z = 0.0;
		vision_pose.transform.translation.x = 0.0;
		vision_pose.transform.translation.y = 0.0;
		vision_pose.transform.translation.z = 0.0;

		std::array<double, 7> vision_pose2;


		for( int i=0; i< detections.detections.size(); i++ ) {
			double side1 = sqrt( pow(detections.detections.at(i).corners.at(1).y - detections.detections.at(i).corners.at(0).y,2) + pow(detections.detections.at(i).corners.at(1).x - detections.detections.at(i).corners.at(0).x,2));
			double side2 = sqrt( pow(detections.detections.at(i).corners.at(2).y - detections.detections.at(i).corners.at(1).y,2) + pow(detections.detections.at(i).corners.at(2).x - detections.detections.at(i).corners.at(1).x,2));
			double side3 = sqrt( pow(detections.detections.at(i).corners.at(3).y - detections.detections.at(i).corners.at(2).y,2) + pow(detections.detections.at(i).corners.at(3).x - detections.detections.at(i).corners.at(2).x,2));
			double side4 = sqrt( pow(detections.detections.at(i).corners.at(0).y - detections.detections.at(i).corners.at(3).y,2) + pow(detections.detections.at(i).corners.at(0).x - detections.detections.at(i).corners.at(3).x,2));
			tagAreaWeights(i,0) = i;
			tagAreaWeights(i,1) = pow( (side1 + side2 + side3 + side4)/4,2 ); // weight = squared mean side length (larger/closer tag is more reliable)
		}

		double totalWeight = 0;


		int num_of_valid_ids = 0;
		std::vector<double> xs, ys, zs;

		for( int i = 0; i< tagAreaWeights.rows(); i++ ) {
			vision_pose.transform.translation.x += transforms.transforms.at( tagAreaWeights(i, 0) ).transform.translation.x * tagAreaWeights(i, 1);
			vision_pose.transform.translation.y += transforms.transforms.at( tagAreaWeights(i, 0) ).transform.translation.y * tagAreaWeights(i, 1);
			vision_pose.transform.translation.z += transforms.transforms.at( tagAreaWeights(i, 0) ).transform.translation.z * tagAreaWeights(i, 1);

			vision_pose.transform.rotation.w += transforms.transforms.at( tagAreaWeights(i, 0) ).transform.rotation.w * tagAreaWeights(i, 1);
			vision_pose.transform.rotation.x += transforms.transforms.at( tagAreaWeights(i, 0) ).transform.rotation.x * tagAreaWeights(i, 1);
			vision_pose.transform.rotation.y += transforms.transforms.at( tagAreaWeights(i, 0) ).transform.rotation.y * tagAreaWeights(i, 1);
			vision_pose.transform.rotation.z += transforms.transforms.at( tagAreaWeights(i, 0) ).transform.rotation.z * tagAreaWeights(i, 1);



			vision_pose2[0] += transforms.transforms.at( tagAreaWeights(i, 0) ).transform.translation.x * tagAreaWeights(i, 1);
			vision_pose2[1] += transforms.transforms.at( tagAreaWeights(i, 0) ).transform.translation.y * tagAreaWeights(i, 1);
			vision_pose2[2] += transforms.transforms.at( tagAreaWeights(i, 0) ).transform.translation.z * tagAreaWeights(i, 1);

			vision_pose2[3] += transforms.transforms.at( tagAreaWeights(i, 0) ).transform.rotation.w * tagAreaWeights(i, 1);
			vision_pose2[4] += transforms.transforms.at( tagAreaWeights(i, 0) ).transform.rotation.x * tagAreaWeights(i, 1);
			vision_pose2[5] += transforms.transforms.at( tagAreaWeights(i, 0) ).transform.rotation.y * tagAreaWeights(i, 1);
			vision_pose2[6] += transforms.transforms.at( tagAreaWeights(i, 0) ).transform.rotation.z * tagAreaWeights(i, 1);

			xs.push_back(transforms.transforms.at( tagAreaWeights(i, 0) ).transform.translation.x);
			ys.push_back(transforms.transforms.at( tagAreaWeights(i, 0) ).transform.translation.y);
			zs.push_back(transforms.transforms.at( tagAreaWeights(i, 0) ).transform.translation.z);

			totalWeight += tagAreaWeights(i, 1);

			double xx = transforms.transforms.at( tagAreaWeights(i, 0) ).transform.translation.x;
			double yy = transforms.transforms.at( tagAreaWeights(i, 0) ).transform.translation.y;
			double rot_yaw = -1*_drone_yaw;
			auto temp = -(sinf(rot_yaw) * xx + cosf(rot_yaw) * yy);	
		}
		auto median = [](std::vector<double> v)
		{
			std::sort(v.begin(), v.end());

			size_t n = v.size();

			if(n % 2 == 0)
				return 0.5 * (v[n/2 - 1] + v[n/2]);

			return v[n/2];
		};


		vision_pose.transform.translation.x = vision_pose.transform.translation.x / totalWeight;
		vision_pose.transform.translation.y = vision_pose.transform.translation.y / totalWeight;
		vision_pose.transform.translation.z = vision_pose.transform.translation.z / totalWeight;



		vision_pose.transform.rotation.w = vision_pose.transform.rotation.w / totalWeight;
		vision_pose.transform.rotation.x = vision_pose.transform.rotation.x / totalWeight;
		vision_pose.transform.rotation.y = vision_pose.transform.rotation.y / totalWeight;
		vision_pose.transform.rotation.z = vision_pose.transform.rotation.z / totalWeight;


		vision_pose2[0] = vision_pose2[0] / totalWeight;
		vision_pose2[1] = vision_pose2[1] / totalWeight;
		vision_pose2[2] = vision_pose2[2] / totalWeight;

		vision_pose2[3] = vision_pose2[3] / totalWeight;
		vision_pose2[4] = vision_pose2[4] / totalWeight;
		vision_pose2[5] = vision_pose2[5] / totalWeight;
		vision_pose2[6] = vision_pose2[6] / totalWeight;

		double horiz_dist_norm = sqrt(pow(vision_pose2[0],2) + pow(vision_pose2[1],2));

		detectionLog << std::to_string(detections.detections.size()) + "," + std::to_string(horiz_dist_norm) + "," + std::to_string(vision_pose2[2]) + "," + std::to_string(this->now().seconds()) + "\n";

		// preparing for last rotation: the computed position is wrt drone frame: the whole should be
		// rotated in global frame.
		// creating then a temporary store
		geometry_msgs::msg::TransformStamped vision_temp;
		// the rotation sign depends on the choosen direction of z as well
		int sign = 1;
		if(z_up)
			sign = -1;

		double rotation_yaw = sign*_drone_yaw;
		// performing a scalar product with a rotation matrix on drone's z-axis: this one
		// rotates the error in the world frame, so that it corresponds to real working environment
		vision_temp.transform.translation.x = (cosf(rotation_yaw) * vision_pose.transform.translation.x - sinf(rotation_yaw) * vision_pose.transform.translation.y);
		vision_temp.transform.translation.y = (sinf(rotation_yaw) * vision_pose.transform.translation.x + cosf(rotation_yaw) * vision_pose.transform.translation.y);

		

		// finally, assigning the computed rotation, adapting it to NED framework
		// (rotations along Z by 90 degrees counterclockwise and X by 180 degrees for z_up)
		// (rotation along Z by 180 degrees for z_down)
		// this is due to the sign of rotation angle
		if(z_up)
		{
			vision_pose.transform.translation.x = -vision_temp.transform.translation.y;
			vision_pose.transform.translation.y = vision_temp.transform.translation.x;
		}
		else
		{
			vision_pose.transform.translation.x = -vision_temp.transform.translation.x;
			vision_pose.transform.translation.y = -vision_temp.transform.translation.y;
		}

		// checking whenever we would have anomalies in the computed errors
		if(abs(vision_pose.transform.translation.x) > _x_err_thld)
		{
			vision_pose.transform.translation.x = _previous_x_err;
			// RCLCPP_WARN(get_logger(), "X error is too high, selecting previous one!");
			_x_err_count++;
		}
		else
		{
			// if it's good instead, saving it for the next iteration
			_previous_x_err = vision_pose.transform.translation.x;
			_x_err_count = 0;
		}

		// doing the same for Y and Z
		if(abs(vision_pose.transform.translation.y) > _y_err_thld)
		{
			vision_pose.transform.translation.y = _previous_y_err;
			// RCLCPP_WARN(get_logger(), "Y error is too high, selecting previous one!");
			_y_err_count++;
		}
		else
		{
			_previous_y_err = vision_pose.transform.translation.y;
			_y_err_count = 0;
		}

		if(abs(vision_pose.transform.translation.z) > _z_err_thld)
		{
			vision_pose.transform.translation.z = _previous_z_err;
			// RCLCPP_WARN(get_logger(), "Z error is too high, selecting previous one!");
			_z_err_count++;
		}
		else
		{
			_previous_z_err = vision_pose.transform.translation.z;
			_z_err_count = 0;
		}
		
		// _handler.publish("apriltag_vision_system",&vision_pose);
		geometry_msgs::msg::TransformStamped msg_tf;
		geometry_msgs::msg::TransformStamped gt_msg_tf;
		msg_tf.transform.translation.x = std::isnan(vision_pose.transform.translation.x) ? -1.0 : vision_pose.transform.translation.x;
		msg_tf.transform.translation.y = std::isnan(vision_pose.transform.translation.y) ? -1.0 : vision_pose.transform.translation.y;
		msg_tf.transform.translation.z = std::isnan(vision_pose.transform.translation.z) ? -1.0 : vision_pose.transform.translation.z;
		pub_platformCenter->publish(msg_tf);
		gt_msg_tf.transform.translation.x = 2 - _drone_pose.transform.translation.x;
		gt_msg_tf.transform.translation.y = -2 - _drone_pose.transform.translation.y;
		pub_platformCenter_gt->publish(gt_msg_tf);
	}
}


void AprilTag2Node::getPose(const matd_t& H, geometry_msgs::msg::Transform& t, const int ID, const bool z_up)
{
	const Eigen::Map<const Mat3> Hm(H.data);

	// compute extrinsic camera parameter
	// https://dsp.stackexchange.com/a/2737/31703
	// H = K * T  =>  T = K^(-1) * H
	const Mat3 T = K.inverse() * Hm / Hm(2, 2);
	Mat3 R;
	R.col(0) = T.col(0).normalized();
	R.col(1) = T.col(1).normalized();
	R.col(2) = R.col(0).cross(R.col(1));

	if(z_up) {
		// z_up convention: flip Y and Z axes to match coordinate system
		R.col(1) *= -1;
		R.col(2) *= -1;
	}

	// the corner coordinates of the tag in the canonical frame are (+/-1, +/-1)
	// hence the scale is half of the edge size
	std::cout << "platform_tags[ID]  " << platform_tags[ID]  << std::endl;
	const Eigen::Vector3d tt = T.rightCols<1>() / ((T.col(0).norm() + T.col(1).norm()) / 2.0) * (platform_tags[ID] / 2.0);

	Eigen::Matrix4d pose;

	pose << R(0,0), R(0,1), R(0,2), tt[0],
				R(1,0), R(1,1), R(1,2), tt[1],
				R(2,0), R(2,1), R(2,2), tt[2],
					 0,      0,      0,     1;

	// RCLCPP_INFO(get_logger(), "TAG RECOGNIZED, ID: %li ", ID);


	if (ID == 0) {
		pose = pose * TAGS.TAG0_T_TAG5;
	} else if (ID == 1) {
		pose = pose * TAGS.TAG1_T_TAG5;
	} else if (ID == 2) {
		pose = pose * TAGS.TAG2_T_TAG5;
	} else if (ID == 3) {
		pose = pose * TAGS.TAG3_T_TAG5;
	} else if (ID == 4) {
		pose = pose * TAGS.TAG4_T_TAG5;
	} else if (ID == 5) {
		pose = pose * TAGS.TAG5_T_TAG5;
	} else if (ID == 6) {
		pose = pose * TAGS.TAG6_T_TAG5;
	} else if (ID == 7) {
		pose = pose * TAGS.TAG7_T_TAG5;
	} else if (ID == 8) {
		pose = pose * TAGS.TAG8_T_TAG5;
	} else if (ID == 9) {
		pose = pose * TAGS.TAG9_T_TAG5;
		detectionLog << "BABBALA9" << "\n";
	} else if (ID == 10) {
		pose = pose * TAGS.TAG10_T_TAG5;
		detectionLog << "BABBALA10" << "\n";
	} else if (ID == 11) {
		pose = pose * TAGS.TAG11_T_TAG5;
		detectionLog << "BABBALA11" << "\n";
	} else if (ID == 12) {
		pose = pose * TAGS.TAG12_T_TAG5;
		detectionLog << "BABBALA12" << "\n";
	}

	pose = TAGS.robot_T_cam * pose;

	const Eigen::Quaterniond q(R);

	t.translation.x = pose(0, 3);
	t.translation.y = pose(1, 3);
	t.translation.z = pose(2, 3);
	t.rotation.w = q.w();
	t.rotation.x = q.x();
	t.rotation.y = q.y();
	t.rotation.z = q.z();
}

double AprilTag2Node::movingFilter(double raw, std::queue<double> *queue_t, double *integral){

	int window = 120;
	if(queue_t->size() == window){

		*integral += raw - queue_t->front();
		queue_t->pop();

	}
	else
		*integral +=raw;

	queue_t->push(raw);

	return *integral/queue_t->size();


}

void AprilTag2Node::toEulerAngle(double quaternion[], double *Roll, double *Pitch, double *Yaw){

	//roll (x-axis rotation)
	double sinr_cosp = 2.0 *( quaternion[0]*quaternion[1] + quaternion[2]*quaternion[3] );
	double cosr_cosp = 1.0 - 2.0*( quaternion[1]*quaternion[1] + quaternion[2]*quaternion[2] );

	*Roll = atan2( sinr_cosp, cosr_cosp );

	//pitch (y-axis rotation)
	double sinp = 2.0*( quaternion[0]*quaternion[2] + quaternion[3]*quaternion[1] );

	if( fabs(sinp) >= 1 )

		*Pitch = copysign(M_PI/2,sinp); //use 90 degrees if out of range

	else
		*Pitch = asin(sinp);

	//yaw (z-axis rotation)
	double siny_cosp = 2.0 *( quaternion[0]*quaternion[3] + quaternion[1]*quaternion[2] );
	double cosy_cosp = 1.0 - 2.0*( quaternion[2]*quaternion[2] + quaternion[3]*quaternion[3] );
	*Yaw = atan2(siny_cosp,cosy_cosp);

}

void AprilTag2Node::getDroneOdometry(const px4_msgs::msg::VehicleOdometry::SharedPtr msg_odom)
{
	// PX4 NED→ROS ENU: swap X/Y axes, negate Z
	_drone_pose.transform.translation.x =  msg_odom->position[1];
	_drone_pose.transform.translation.y =  msg_odom->position[0];
	_drone_pose.transform.translation.z =  -msg_odom->position[2];

	_drone_pose.transform.rotation.w = msg_odom->q[0];
	_drone_pose.transform.rotation.x = msg_odom->q[1];
	_drone_pose.transform.rotation.y = msg_odom->q[2];
	_drone_pose.transform.rotation.z = msg_odom->q[3];

	_drone_yaw = atan2(2.0 * (msg_odom->q[0]*msg_odom->q[3] + msg_odom->q[1]*msg_odom->q[2]), 1.0 - 2.0 * (msg_odom->q[2]*msg_odom->q[2] + msg_odom->q[3]*msg_odom->q[3])); // yaw from quaternion [w,x,y,z]
}

double AprilTag2Node::getMedian(std::deque<double> buffer) {
    std::vector<double> sort_vec(buffer.begin(), buffer.end());
    std::sort(sort_vec.begin(), sort_vec.end());
    return sort_vec[sort_vec.size() / 2];
}

geometry_msgs::msg::Transform
AprilTag2Node::pnp(apriltag_detection_t* const detection, const std::array<double, 4>& intr, double tagsize, int tag_id)
{
    const std::vector<cv::Point3d> objectPoints{
        {-tagsize / 2, -tagsize / 2, 0},
        {+tagsize / 2, -tagsize / 2, 0},
        {+tagsize / 2, +tagsize / 2, 0},
        {-tagsize / 2, +tagsize / 2, 0},
    };

    const std::vector<cv::Point2d> imagePoints{
        {detection->p[0][0], detection->p[0][1]},
        {detection->p[1][0], detection->p[1][1]},
        {detection->p[2][0], detection->p[2][1]},
        {detection->p[3][0], detection->p[3][1]},
    };

    cv::Matx33d cameraMatrix;
    cameraMatrix(0, 0) = intr[0];// fx
    cameraMatrix(1, 1) = intr[1];// fy
    cameraMatrix(0, 2) = intr[2];// cx
    cameraMatrix(1, 2) = intr[3];// cy

    // IPPE returns both solutions for the planar-target ambiguity, ranked by
    // reprojection error. We then pick the solution closest to the previous
    // tvec for this tag (temporal consistency), falling back to solution[0]
    // on the first frame.
    std::vector<cv::Mat> rvecs, tvecs;
    std::vector<double> reprojErrors;
    cv::solvePnPGeneric(objectPoints, imagePoints, cameraMatrix, cv::noArray(),
                        rvecs, tvecs, false, cv::SOLVEPNP_IPPE,
                        cv::noArray(), cv::noArray(), reprojErrors);

    int chosen = 0;
    auto it = _prev_tvec_.find(tag_id);
    if (it != _prev_tvec_.end() && tvecs.size() >= 2) {
        double d0 = cv::norm(tvecs[0] - it->second);
        double d1 = cv::norm(tvecs[1] - it->second);
        if (d1 < d0) chosen = 1;
    }
    _prev_tvec_[tag_id] = tvecs[chosen].clone();

    cv::Mat rvec = rvecs[chosen];
    cv::Mat tvec = tvecs[chosen];
	Eigen::Matrix4d pose;
	cv::Mat R_cv;
	cv::Rodrigues(rvec, R_cv);
	if (z_up) {
		// z_up convention: flip Y and Z axes to match coordinate system
		R_cv.col(1) *= -1;
    	R_cv.col(2) *= -1;
	}

	pose = Eigen::Matrix4d::Identity();
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			pose(i, j) = R_cv.at<double>(i, j);
		}
		pose(i, 3) = tvec.at<double>(i);
	}

	if (detection->id == 0) {
		pose = pose * TAGS.TAG0_T_TAG5;
	} else if (detection->id == 1) {
		pose = pose * TAGS.TAG1_T_TAG5;
	} else if (detection->id == 2) {
		pose = pose * TAGS.TAG2_T_TAG5;
	} else if (detection->id == 3) {
		pose = pose * TAGS.TAG3_T_TAG5;
	} else if (detection->id == 4) {
		pose = pose * TAGS.TAG4_T_TAG5;
	} else if (detection->id == 5) {
		pose = pose * TAGS.TAG5_T_TAG5;
	} else if (detection->id == 6) {
		pose = pose * TAGS.TAG6_T_TAG5;
	} else if (detection->id == 7) {
		pose = pose * TAGS.TAG7_T_TAG5;
	} else if (detection->id == 8) {
		pose = pose * TAGS.TAG8_T_TAG5;
	} else if (detection->id == 9) {
		pose = pose * TAGS.TAG9_T_TAG5;
		detectionLog << "BABBALA9" << "\n";
	} else if (detection->id == 10) {
		pose = pose * TAGS.TAG10_T_TAG5;
		detectionLog << "BABBALA10" << "\n";
	} else if (detection->id == 11) {
		pose = pose * TAGS.TAG11_T_TAG5;
		detectionLog << "BABBALA11" << "\n";
	} else if (detection->id == 12) {
		pose = pose * TAGS.TAG12_T_TAG5;
		detectionLog << "BABBALA12" << "\n";
	}

	pose = TAGS.robot_T_cam * pose;
	Eigen::Matrix3d final_rotation = pose.block<3, 3>(0, 0);
	Eigen::Quaterniond q(final_rotation);
	geometry_msgs::msg::Transform t;
	t.translation.x = pose(0, 3);
	t.translation.y = pose(1, 3);
	t.translation.z = pose(2, 3);
	t.rotation.w = q.w();
	t.rotation.x = q.x();
	t.rotation.y = q.y();
	t.rotation.z = q.z();
	return t;
}
geometry_msgs::msg::Transform
AprilTag2Node::toMsg(const std::pair<cv::Mat_<double>, cv::Mat_<double>>& pose)
{
    assert((pose.first.rows == 3 && pose.first.cols == 1) || (pose.first.rows == 1 && pose.first.cols == 3));
    assert((pose.second.rows == 3 && pose.second.cols == 1) || (pose.second.rows == 1 && pose.second.cols == 3));

    // convert compact rotation vector to angle-axis to quaternion
    const Eigen::Map<const Eigen::Vector3d> rvec(reinterpret_cast<double*>(pose.second.data));
    const Eigen::Quaterniond q({rvec.norm(), rvec.normalized()});

    geometry_msgs::msg::Transform t;
    convert(pose.first, t.translation);
    convert(q, t.rotation);
    return t;
}
void AprilTag2Node::convert(const Eigen::Quaterniond& eigen_quat, geometry_msgs::msg::Quaternion& msg_quat)
{
    msg_quat.w = eigen_quat.w();
    msg_quat.x = eigen_quat.x();
    msg_quat.y = eigen_quat.y();
    msg_quat.z = eigen_quat.z();
}
void AprilTag2Node::convert(const cv::Mat_<double>& vec, geometry_msgs::msg::Vector3& msg_vec)
{
    assert((vec.rows == 3 && vec.cols == 1) || (vec.rows == 1 && vec.cols == 3));

    msg_vec.x = vec.at<double>(0);
    msg_vec.y = vec.at<double>(1);
    msg_vec.z = vec.at<double>(2);
}

std::map<std::string, apriltag_family_t *(*)(void)> AprilTag2Node::tag_create =
{
	{"16h5", tag16h5_create},
	{"25h9", tag25h9_create},
	{"36h11", tag36h11_create},
};

std::map<std::string, void (*)(apriltag_family_t*)> AprilTag2Node::tag_destroy =
{
	{"16h5", tag16h5_destroy},
	{"25h9", tag25h9_destroy},
	{"36h11", tag36h11_destroy},
};

CLASS_LOADER_REGISTER_CLASS(AprilTag2Node, rclcpp::Node)
