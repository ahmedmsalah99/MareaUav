#include <AprilTag2Node.hpp>
#include <class_loader/register_macro.hpp>
#include "LandingPlatform.h"
#include <apriltag_pose.h>
#include <apriltag.h>


// default tag families
#include <tag16h5.h>
#include <tag25h9.h>
#include <tag36h11.h>
#include <math.h>
#include <Eigen/Dense>

static Platform_tag TAGS;

AprilTag2Node::AprilTag2Node() : Node("apriltag2", "apriltag", true)
{
	sub_img = this->create_subscription<sensor_msgs::msg::CompressedImage>("image/compressed",
		std::bind(&AprilTag2Node::onImage, this, std::placeholders::_1),
		rmw_qos_profile_sensor_data);
	rmw_qos_profile_t tf_qos_profile = rmw_qos_profile_default;
	tf_qos_profile.depth = 1;
	pub_tf = this->create_publisher<tf2_msgs::msg::TFMessage>("/tf", tf_qos_profile);

	pub_detections = this->create_publisher<apriltag_msgs::msg::AprilTagDetectionArray>("detections");

	pub_platformCenter = this->create_publisher<geometry_msgs::msg::TransformStamped>("/platform_center");

	// get single camera info message
	sub_info = this->create_subscription<sensor_msgs::msg::CameraInfo>(
		"image/camera_info",
		[this](sensor_msgs::msg::CameraInfo::UniquePtr info){
			RCLCPP_INFO(get_logger(), "got camera parameters");
			std::memcpy(K.data(), info->k.data(), 9 * sizeof(double));
			// delete subscription
			sub_info.reset();
		}
	);

	get_parameter_or<std::string>("family", tag_family, "36h11");
	get_parameter_or<double>("size", tag_edge_size, 2.0);
	get_parameter_or<int>("max_hamming", max_hamming, 0);

	// get tag names and IDs
	static const std::string tag_list_prefix = "tag_lists";
	auto parameters_and_prefixes = list_parameters({tag_list_prefix}, 10);
	for (const std::string &name : parameters_and_prefixes.names) {
		const int id = stoi( name.substr(tag_list_prefix.size()+1, name.size()) );
		platform_tags[id] = get_parameter(name).get_value<float>();
	}

	SetMatrix(&TAGS); //load transformation matrices respect to the platform center

	get_parameter_or<bool>("z_up", z_up, false);

	if(!tag_create.count(tag_family)) {
		throw std::runtime_error("unsupported tag family: " + tag_family);
	}
	tf = tag_create.at(tag_family)();
	td = apriltag_detector_create();
	get_parameter_or<float>("decimate", td->quad_decimate, 1.0);
	get_parameter_or<float>("blur", td->quad_sigma, 0.0);
	get_parameter_or<int>("threads", td->nthreads, 1);
	get_parameter_or<int>("debug", td->debug, false);
	get_parameter_or<int>("refine-edges", td->refine_edges, true);
	apriltag_detector_add_family(td, tf);

	//std::cout<<"eeeeeeeeeeeeeeeeeeeeeeeeeeee"<<td->quad_decimate<<std::endl;
}

AprilTag2Node::~AprilTag2Node()
{
	apriltag_detector_destroy(td);
	tag_destroy.at(tag_family)(tf);
}

void AprilTag2Node::onImage(const sensor_msgs::msg::CompressedImage::SharedPtr msg_img)
{
	// decode image
	image_u8_t* im = nullptr;

	if(msg_img->format=="jpeg" || msg_img->format=="jpg") {
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
	zarray_t* detections = apriltag_detector_detect(td, im);

	apriltag_msgs::msg::AprilTagDetectionArray msg_detections;
	msg_detections.header = msg_img->header;

	tf2_msgs::msg::TFMessage tfs;

	//AREA
	//Eigen::MatrixXd areaID(zarray_size(detections),2);

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
		msg_detections.detections.push_back(msg_detection);

		// 3D orientation and position
		geometry_msgs::msg::TransformStamped tf;
		tf.header = msg_img->header;
		// set child frame name by generic tag name or configured tag name
		tf.child_frame_id = tracked_tags.size() ? tracked_tags.at(det->id) : std::string(det->family->name) + ":" + std::to_string(det->id);
		getPose(*(det->H), tf.transform, det->id, z_up);

		tfs.transforms.push_back(tf);
	}

	if(zarray_size(detections) > 0) {
		publish_data_center( msg_detections,tfs );
	}

	pub_detections->publish(msg_detections);
	pub_tf->publish(tfs);

	apriltag_detections_destroy(detections);

	image_u8_destroy(im);
}


void AprilTag2Node::publish_data_center(const apriltag_msgs::msg::AprilTagDetectionArray& Detections, const tf2_msgs::msg::TFMessage& t_f)
{
	Eigen::MatrixXd areaID(Detections.detections.size(), 2);
	geometry_msgs::msg::TransformStamped msg_relative_pose;

	for( int i=0; i< Detections.detections.size(); i++ ) {
		double l1 = sqrt( pow(Detections.detections.at(i).corners[1].y - Detections.detections.at(i).corners[0].y,2) + pow(Detections.detections.at(i).corners[1].x - Detections.detections.at(i).corners[0].x,2));
		double l2 = sqrt( pow(Detections.detections.at(i).corners[2].y - Detections.detections.at(i).corners[1].y,2) + pow(Detections.detections.at(i).corners[2].x - Detections.detections.at(i).corners[1].x,2));
		double l3 = sqrt( pow(Detections.detections.at(i).corners[3].y - Detections.detections.at(i).corners[2].y,2) + pow(Detections.detections.at(i).corners[3].x - Detections.detections.at(i).corners[2].x,2));
		double l4 = sqrt( pow(Detections.detections.at(i).corners[0].y - Detections.detections.at(i).corners[3].y,2) + pow(Detections.detections.at(i).corners[0].x - Detections.detections.at(i).corners[3].x,2));

		RCLCPP_INFO(get_logger(), "Lati: %lf %lf %lf %lf", l1, l2, l3, l4);

		areaID(i,0) = i;
		areaID(i,1) = pow( (l1 + l2 + l3 +l4)/4,2 );

		RCLCPP_INFO(get_logger(), "Area: %lf", areaID(i, 1));
	}

	double weighTot = 0;

	for( int i = 0; i< areaID.rows(); i++ ) {
		msg_relative_pose.transform.translation.x += t_f.transforms[areaID(i, 0)].transform.translation.x * areaID(i, 1);
		msg_relative_pose.transform.translation.y += t_f.transforms[areaID(i, 0)].transform.translation.y * areaID(i, 1);
		msg_relative_pose.transform.translation.z += t_f.transforms[areaID(i, 0)].transform.translation.z * areaID(i, 1);

		msg_relative_pose.transform.rotation.x += t_f.transforms[areaID(i, 0)].transform.rotation.x * areaID(i, 1);
		msg_relative_pose.transform.rotation.y += t_f.transforms[areaID(i, 0)].transform.rotation.y * areaID(i, 1);
		msg_relative_pose.transform.rotation.z += t_f.transforms[areaID(i, 0)].transform.rotation.z * areaID(i, 1);
		msg_relative_pose.transform.rotation.w += t_f.transforms[areaID(i, 0)].transform.rotation.w * areaID(i, 1);

		weighTot += areaID(i, 1);
	}

	msg_relative_pose.transform.translation.x = msg_relative_pose.transform.translation.x / weighTot;
	msg_relative_pose.transform.translation.y = msg_relative_pose.transform.translation.y / weighTot;
	msg_relative_pose.transform.translation.z = msg_relative_pose.transform.translation.z / weighTot;

	msg_relative_pose.transform.rotation.x = msg_relative_pose.transform.rotation.x / weighTot;
	msg_relative_pose.transform.rotation.y = msg_relative_pose.transform.rotation.y / weighTot;
	msg_relative_pose.transform.rotation.z = msg_relative_pose.transform.rotation.z / weighTot;
	msg_relative_pose.transform.rotation.w = msg_relative_pose.transform.rotation.w / weighTot;

	pub_platformCenter->publish(msg_relative_pose);
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
		// rotate by half rotation about x-axis
		R.col(1) *= -1;
		R.col(2) *= -1;
	}

	// the corner coordinates of the tag in the canonical frame are (+/-1, +/-1)
	// hence the scale is half of the edge size

	const Eigen::Vector3d tt = T.rightCols<1>() / ((T.col(0).norm() + T.col(0).norm()) / 2.0) * (platform_tags[ID] / 2.0);

	Eigen::Matrix4d pose;

	pose << R(0,0), R(0,1), R(0,2), tt[0],
				R(1,0), R(1,1), R(1,2), tt[1],
				R(2,0), R(2,1), R(2,2), tt[2],
					 0,      0,      0,     1;

	if (ID == 1) {
		pose = pose * TAGS.TAG1_T_TAG5;
	} else if (ID == 2) {
		pose = pose * TAGS.TAG2_T_TAG5;
	} else if (ID == 3) {
		pose = pose * TAGS.TAG3_T_TAG5;
	} else if (ID == 4) {
		pose = pose * TAGS.TAG4_T_TAG5;
	} else if (ID == 6) {
		pose = pose * TAGS.TAG6_T_TAG5;
	} else if (ID == 7) {
		pose = pose * TAGS.TAG7_T_TAG5;
	} else if (ID == 8) {
		pose = pose * TAGS.TAG8_T_TAG5;
	} else if (ID == 9) {
		pose = pose * TAGS.TAG9_T_TAG5;
	}

	const Eigen::Quaterniond q(R);

	t.translation.x = pose(0, 3);
	t.translation.y = pose(1, 3);
	t.translation.z = pose(2, 3);
	t.rotation.w = q.w();
	t.rotation.x = q.x();
	t.rotation.y = q.y();
	t.rotation.z = q.z();
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

/*

	std::cout<<"lati: "<<" "<<l1<<" "<<l2<<" "<<l3<<" "<<l4<<std::endl;

	areaID(i,0) = i;
	areaID(i,1) = pow( (l1 + l2 + l3 +l4)/4,2 );
	
	std::cout<<"Area: "<<areaID(i,1)<<std::endl;
	
	}



double weighTot = 0;

for( int i=0; i< areaID.rows(); i++ ){
	
	msg_relative_pose.transform.translation.x += t_f.transforms[areaID(i,0)].transform.translation.x*areaID(i,1);
	msg_relative_pose.transform.translation.y += t_f.transforms[areaID(i,0)].transform.translation.y*areaID(i,1);
	msg_relative_pose.transform.translation.z += t_f.transforms[areaID(i,0)].transform.translation.z*areaID(i,1);
	
	msg_relative_pose.transform.rotation.x += t_f.transforms[areaID(i,0)].transform.rotation.x*areaID(i,1);
	msg_relative_pose.transform.rotation.y += t_f.transforms[areaID(i,0)].transform.rotation.y*areaID(i,1);
	msg_relative_pose.transform.rotation.z += t_f.transforms[areaID(i,0)].transform.rotation.z*areaID(i,1);
	msg_relative_pose.transform.rotation.w += t_f.transforms[areaID(i,0)].transform.rotation.w*areaID(i,1);

	weighTot += areaID(i,1);
	}

	msg_relative_pose.transform.translation.x = msg_relative_pose.transform.translation.x/weighTot;
	msg_relative_pose.transform.translation.y = msg_relative_pose.transform.translation.y/weighTot;
	msg_relative_pose.transform.translation.z = msg_relative_pose.transform.translation.z/weighTot;
	
	msg_relative_pose.transform.rotation.x = msg_relative_pose.transform.rotation.x/weighTot;
	msg_relative_pose.transform.rotation.y = msg_relative_pose.transform.rotation.y/weighTot;
	msg_relative_pose.transform.rotation.z = msg_relative_pose.transform.rotation.z/weighTot;
	msg_relative_pose.transform.rotation.w = msg_relative_pose.transform.rotation.w/weighTot;

pub_platformCenter->publish(msg_relative_pose);

*/

