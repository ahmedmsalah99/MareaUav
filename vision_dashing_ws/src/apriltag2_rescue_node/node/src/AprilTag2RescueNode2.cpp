#include <AprilTag2RescueNode.hpp>
#include <class_loader/register_macro.hpp>
#include "RescueTag.h"
// #include <apriltag_pose.h>
// #include <apriltag.h>
#include <sstream>
#include <stdexcept>
#include <string>


// default tag families
#include <tag16h5.h>
#include <tag25h9.h>
#include <tag36h11.h>
#include <math.h>
#include <Eigen/Dense>

static Rescue_tag TAGS;

AprilTag2RescueNode::AprilTag2RescueNode() : Node("apriltag2rescue", "apriltagrescue", rclcpp::NodeOptions().use_intra_process_comms(true))
{

	sub_img = this->create_subscription<sensor_msgs::msg::CompressedImage>("/apriltag/image/compressed/compressed",
		rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history,rmw_qos_profile_sensor_data.depth),rmw_qos_profile_sensor_data),
		std::bind(&AprilTag2RescueNode::onImage, this, std::placeholders::_1));
	rmw_qos_profile_t tf_qos_profile = rmw_qos_profile_default;
	tf_qos_profile.depth = 1;



	odometry_sub = this->create_subscription<px4_msgs::msg::VehicleOdometry>("/fmu/out/vehicle_odometry",
		rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history,rmw_qos_profile_sensor_data.depth),rmw_qos_profile_sensor_data),
	 	std::bind(&AprilTag2RescueNode::getDroneOdometry, this, std::placeholders::_1));
	//tf of each tags in the scene
	//pub_tf = this->create_publisher<tf2_msgs::msg::TFMessage>("/tf", tf_qos_profile);

	//information from the vision system(hamming, goodness, corners..)
	//pub_detections = this->create_publisher<apriltag_msgs::msg::AprilTagDetectionArray>("detections");

	// rescue tag topic
	pub_rescuePosition = this->create_publisher<geometry_msgs::msg::TransformStamped>("/rescue",1);

	// set camera info message, these values are obtained after calibration and they are good for 640x480.
	// const double fx = 508.624627;
	// const double cx = 323.83127;
	// const double fy = 508.77675;
	// const double cy = 248.651286;

	const double fx = 381.36246688113556;
	const double cx = 320;
	const double fy = 381.36246688113556;
	const double cy = 240.5;

	K << fx, 0, cx, 0, fy, cy, 0, 0, 1;
	intrinsics = {fx, fy, cx,cy};
	//std::cout<<"K: "<<K<<std::endlrescue_tags;

	// getting the value of the environment variable, to check whenever we are running in SITL or in a RPi
	char *env_val;
	// M2M_MODE: 0 = sim, 1 = mocap, 2 = hw. Only sim uses the simulated tag
	// layout; mocap and hw both fly the real platform.
	env_val = std::getenv("M2M_MODE");
	const std::string mode_str = env_val ? env_val : "";
	if (mode_str != "0" && mode_str != "1" && mode_str != "2")
		throw std::runtime_error("M2M_MODE must be set to 0 (sim), 1 (mocap) or 2 (hw); got '" + mode_str + "'");
	bool env_bool = (mode_str == "0");

	declare_parameter<std::string>("family", "36h11");
	declare_parameter<double>("size", 2.0);
	declare_parameter<int>("max_hamming", 0);
	declare_parameter<bool>("z_up", false);
	get_parameter("family", tag_family);
	get_parameter("size", tag_edge_size);
	get_parameter("max_hamming", max_hamming);
	get_parameter("z_up", z_up);

	// std::cout<<"EDGE SIZE: "<<z_up<<std::endl;

	// get tag names and IDs
	static const std::string tag_prefix = "tag_";
	declare_parameter<int>("n_tags", 0);
	get_parameter("n_tags", n_tags);

	for(int i=0; i<n_tags; i++)
	{
		std::string tag_name = tag_prefix+std::to_string(i+1);
		declare_parameter<double>(tag_name, 2.0);
		get_parameter(tag_name, rescue_tags[i]);
	}

	SetMatrix(&TAGS); //load transformation matrices respect to the center


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

}

AprilTag2RescueNode::~AprilTag2RescueNode()
{
	apriltag_detector_destroy(_tagDetector);
	tag_destroy.at(tag_family)(_tagFamily);
}

void AprilTag2RescueNode::onImage(const sensor_msgs::msg::CompressedImage::SharedPtr msg_img)
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

	for (int i = 0; i < zarray_size(detections); i++) {
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
		try{
			tagTransform.transform = pnp(det, intrinsics, rescue_tags[det->id]);
		}catch (const cv::Exception& e) {
			continue;
		}

		if(std::isnan(tagTransform.transform.translation.x) || std::isnan(tagTransform.transform.translation.y) || std::isnan(tagTransform.transform.translation.z)) continue;
		if(std::abs(tagTransform.transform.translation.x) > 20 || std::abs(tagTransform.transform.translation.y) > 20 || std::abs(tagTransform.transform.translation.z) > 10) continue;
		msg_detections.detections.push_back(msg_detection);

		tfs.transforms.push_back(tagTransform);
	}

	if(zarray_size(detections) > 0) {
		publish_data_center( msg_detections,tfs );
	}

	//pub_detections->publish(msg_detections);
	//pub_tf->publish(tfs);

	apriltag_detections_destroy(detections);

	image_u8_destroy(im);
}


void AprilTag2RescueNode::publish_data_center(const apriltag_msgs::msg::AprilTagDetectionArray& detections, const tf2_msgs::msg::TFMessage& transforms)
{
	if(detections.detections.size()>0)
	{
		Eigen::MatrixXd tagAreaWeights(detections.detections.size(), 2);
		// geometry_msgs::msg::TransformStamped msg_relative_pose;
		geometry_msgs::msg::TransformStamped vision_pose; //lcm message for vision topic.

		for( int i=0; i< detections.detections.size(); i++ ) {
			double side1 = sqrt( pow(detections.detections.at(i).corners.at(1).y - detections.detections.at(i).corners.at(0).y,2) + pow(detections.detections.at(i).corners.at(1).x - detections.detections.at(i).corners.at(0).x,2));
			double side2 = sqrt( pow(detections.detections.at(i).corners.at(2).y - detections.detections.at(i).corners.at(1).y,2) + pow(detections.detections.at(i).corners.at(2).x - detections.detections.at(i).corners.at(1).x,2));
			double side3 = sqrt( pow(detections.detections.at(i).corners.at(3).y - detections.detections.at(i).corners.at(2).y,2) + pow(detections.detections.at(i).corners.at(3).x - detections.detections.at(i).corners.at(2).x,2));
			double side4 = sqrt( pow(detections.detections.at(i).corners.at(0).y - detections.detections.at(i).corners.at(3).y,2) + pow(detections.detections.at(i).corners.at(0).x - detections.detections.at(i).corners.at(3).x,2));

			// RCLCPP_INFO(get_logger(), "Lati: %lf %lf %lf %lf", l1, l2, l3, l4);

			tagAreaWeights(i,0) = i;
			tagAreaWeights(i,1) = pow( (side1 + side2 + side3 + side4)/4,2 ); // weight = squared mean side length (larger/closer tag is more reliable)

			// RCLCPP_INFO(get_logger(), "Area: %lf %lf", tagAreaWeights(i,0), tagAreaWeights(i, 1));
		}

		double totalWeight = 0;

		//std::cout<<"detection_size: "<<detections.detections.size()<< " Rows: "<<tagAreaWeights.rows()<<std::endl;


		for( int i = 0; i< tagAreaWeights.rows(); i++ ) {
			vision_pose.transform.translation.x += transforms.transforms.at( tagAreaWeights(i, 0) ).transform.translation.x * tagAreaWeights(i, 1);
			vision_pose.transform.translation.y += transforms.transforms.at( tagAreaWeights(i, 0) ).transform.translation.y * tagAreaWeights(i, 1);
			vision_pose.transform.translation.z += transforms.transforms.at( tagAreaWeights(i, 0) ).transform.translation.z * tagAreaWeights(i, 1);

			vision_pose.transform.rotation.w += transforms.transforms.at( tagAreaWeights(i, 0) ).transform.rotation.w * tagAreaWeights(i, 1);
			vision_pose.transform.rotation.x += transforms.transforms.at( tagAreaWeights(i, 0) ).transform.rotation.x * tagAreaWeights(i, 1);
			vision_pose.transform.rotation.y += transforms.transforms.at( tagAreaWeights(i, 0) ).transform.rotation.y * tagAreaWeights(i, 1);
			vision_pose.transform.rotation.z += transforms.transforms.at( tagAreaWeights(i, 0) ).transform.rotation.z * tagAreaWeights(i, 1);

			totalWeight += tagAreaWeights(i, 1);
		}




		vision_pose.transform.translation.x = vision_pose.transform.translation.x / totalWeight;
		vision_pose.transform.translation.y = vision_pose.transform.translation.y / totalWeight;
		vision_pose.transform.translation.z = vision_pose.transform.translation.z / totalWeight;

		// vision_pose.velocity[0] = 0;
		// vision_pose.velocity[1] = 0;
		// vision_pose.velocity[2] = 0;

		vision_pose.transform.rotation.w = vision_pose.transform.rotation.w / totalWeight;
		vision_pose.transform.rotation.x = vision_pose.transform.rotation.x / totalWeight;
		vision_pose.transform.rotation.y = vision_pose.transform.rotation.y / totalWeight;
		vision_pose.transform.rotation.z = vision_pose.transform.rotation.z / totalWeight;


		// toEulerAngle(vision_pose.orientation,&_roll,&_pitch,&_yaw);
		// //moving filter to clean a bit the data

		// vision_pose.roll = movingFilter(_roll,&_Queue_roll,&_integral_roll);
		// vision_pose.pitch = movingFilter(_pitch,&_Queue_pitch,&_integral_pitch);
		// vision_pose.yaw = movingFilter(_yaw,&_Queue_yaw,&_integral_yaw);


		// preparing for last rotation: the computed position is wrt drone frame: the whole should be
		// rotated in global frame.
		// creating then a temporary store
		geometry_msgs::msg::TransformStamped vision_temp;
		// the rotation sign depends on the choosen direction of z as well
		int sign = 1;
		if(z_up)
			sign = -1;

		double rotation_yaw = sign*_droneYaw;
		// performing a scalar product with a rotation matrix on drone's z-axis: this one
		// rotates the error in the world frame, so that it corresponds to real working environment
		vision_pose.transform.translation.x = (cosf(rotation_yaw) * vision_pose.transform.translation.x - sinf(rotation_yaw) * vision_pose.transform.translation.y);
		vision_pose.transform.translation.y = (sinf(rotation_yaw) * vision_pose.transform.translation.x + cosf(rotation_yaw) * vision_pose.transform.translation.y);

		// finally, assigning the computed rotation, adapting it to NED framework
		// (rotations along Z by 90 degrees counterclockwise and X by 180 degrees for z_up)
		// (rotation along Z by 180 degrees for z_down)
		// this is due to the sign of rotation angle
		if(z_up)
		{
			vision_pose.transform.translation.x = -_drone_pose.transform.translation.y;
			vision_pose.transform.translation.y = _drone_pose.transform.translation.x;
		}
		else
		{
			vision_pose.transform.translation.x = -_drone_pose.transform.translation.x;
			vision_pose.transform.translation.y = -_drone_pose.transform.translation.y;
		}


		// checking whenever we would have anomalies in the computed errors
		if(abs(vision_pose.transform.translation.x) > _x_err_thld)
		{
			vision_pose.transform.translation.x = _previous_x_err;
			RCLCPP_WARN(get_logger(), "X error is too high, selecting previous one!");
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
			RCLCPP_WARN(get_logger(), "Y error is too high, selecting previous one!");
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
			RCLCPP_WARN(get_logger(), "Z error is too high, selecting previous one!");
			_z_err_count++;
		}
		else
		{
			_previous_z_err = vision_pose.transform.translation.z;
			_z_err_count = 0;
		}

	}
}


void AprilTag2RescueNode::getPose(const matd_t& H, geometry_msgs::msg::Transform& t, const int ID, const bool z_up)
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



	// double angle = (-120.0)*3.14/180.0;
	//
	// Mat3 R2_z;
	//
	// R2_z << cos(angle), -sin(angle), 0,
	// 			sin(angle),  cos(angle), 0,
	// 				0,               0, 1;
	//
	// R = R2_z * R;

	if(z_up) {
		// z_up convention: flip Y and Z axes to match coordinate system
		R.col(1) *= -1;
		R.col(2) *= -1;
	}
	// the corner coordinates of the tag in the canonical frame are (+/-1, +/-1)
	// hence the scale is half of the edge size

	const Eigen::Vector3d tt = T.rightCols<1>() / ((T.col(0).norm() + T.col(0).norm()) / 2.0) * (rescue_tags[ID] / 2.0);

	Eigen::Matrix4d pose;

	pose << R(0,0), R(0,1), R(0,2), tt[0],
				R(1,0), R(1,1), R(1,2), tt[1],
				R(2,0), R(2,1), R(2,2), tt[2],
					 0,      0,      0,     1;

	/*if (ID == 0) {
		pose = pose * TAGS.TAG1;
	}*/

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

double AprilTag2RescueNode::movingFilter(double raw, std::queue<double> *queue_t, double *integral){

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

void AprilTag2RescueNode::toEulerAngle(double quaternion[], double *Roll, double *Pitch, double *Yaw){

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


void AprilTag2RescueNode::getDroneOdometry(const px4_msgs::msg::VehicleOdometry::SharedPtr msg_odom)
{
	// PX4 NED→ROS ENU: swap X/Y axes, negate Z
	_drone_pose.transform.translation.x =  msg_odom->position[1];
	_drone_pose.transform.translation.y =  msg_odom->position[0];
	_drone_pose.transform.translation.z =  -msg_odom->position[2];

	// _drone_pose.velocity[0] = msg_odom->velocity[1];
	// _drone_pose.velocity[1] = msg_odom->velocity[0];
	// _drone_pose.velocity[2] = -msg_odom->velocity[2];

	_drone_pose.transform.rotation.w = msg_odom->q[0];
	_drone_pose.transform.rotation.x = msg_odom->q[1];
	_drone_pose.transform.rotation.y = msg_odom->q[2];
	_drone_pose.transform.rotation.z = msg_odom->q[3];

	_droneYaw = atan2(2.0 * (msg_odom->q[0]*msg_odom->q[3] + msg_odom->q[0]*msg_odom->q[2]), 1.0 - 2.0 * (msg_odom->q[2]*msg_odom->q[2] + msg_odom->q[3]*msg_odom->q[3])); // yaw from quaternion [w,x,y,z]
}
geometry_msgs::msg::Transform
AprilTag2RescueNode::pnp(apriltag_detection_t* const detection, const std::array<double, 4>& intr, double tagsize)
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

    cv::Mat rvec, tvec;
    cv::solvePnP(objectPoints, imagePoints, cameraMatrix, {}, rvec, tvec);
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
    // return toMsg(std::make_pair(tvec, rvec));
	return t;
}

std::map<std::string, apriltag_family_t *(*)(void)> AprilTag2RescueNode::tag_create =
{
	{"16h5", tag16h5_create},
	{"25h9", tag25h9_create},
	{"36h11", tag36h11_create},
};

std::map<std::string, void (*)(apriltag_family_t*)> AprilTag2RescueNode::tag_destroy =
{
	{"16h5", tag16h5_destroy},
	{"25h9", tag25h9_destroy},
	{"36h11", tag36h11_destroy},
};

CLASS_LOADER_REGISTER_CLASS(AprilTag2RescueNode, rclcpp::Node)
