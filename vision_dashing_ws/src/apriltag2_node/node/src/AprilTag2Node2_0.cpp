#include <AprilTag2Node.hpp>
#include <class_loader/register_macro.hpp>
#include "LandingPlatform.h"
// #include <apriltag_pose.h>
// #include <apriltag.h>
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
// #include <apriltag/common/homography.h>
// #include <opencv2/calib3d.hpp>
// #include <tf2/convert.hpp>
static Platform_tag TAGS;

std::ofstream myfile;

// rclcpp::Time prev;
// rclcpp::Time curr;
// double max = 0;
// double min = 10;
// double mean = 0;
// double sum = 0;
// int c = 0;
// rclcpp::Clock clocksis;

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
	//tf of each tags in the scene
	//pub_tf = this->create_publisher<tf2_msgs::msg::TFMessage>("/tf", tf_qos_profile);

	//information from the vision system(hamming, goodness, corners..)
	//pub_detections = this->create_publisher<apriltag_msgs::msg::AprilTagDetectionArray>("detections");

	//platform center pose
	// pub_platformCenter = this->create_publisher<geometry_msgs::msg::TransformStamped>("/platform_center",1);
	pub_platformCenter = this->create_publisher<geometry_msgs::msg::TransformStamped>("/platform_center",1);
	pub_platformCenter_gt = this->create_publisher<geometry_msgs::msg::TransformStamped>("/platform_center_gt",1);

	// set camera info message, these values are obtained after calibration and they are good for 640x480.
	// const double fx = 508.624627;
	// const double fx = 510.06217068128194;
	// const double fx = 320.25492609007654;
	const double cx = 323.83127;
	// const double fy = 508.77675;
	// const double fy = 510.06217068128194;
	// const double fy = 320.25492609007654;
	const double cy = 248.651286;

	const double fx = 381.36246688113556;
	// const double cx = 320;
	const double fy = 381.36246688113556;
	// const double cy = 240.5;

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


	// get tag names and IDs
	// static const std::string tag_list_prefix = "tag_lists";
	// auto parameters_and_prefixes = list_parameters({tag_list_prefix}, 10);
	// for (const std::string &name : parameters_and_prefixes.names) {
	// 	// declare_parameter(name);
	// 	// get_parameter(name,platform_tags[])
	// 	const int id = stoi( name.substr(tag_list_prefix.size()+1, name.size()) );
	// 	platform_tags[id] = get_parameter(name).get_value<float>();
	// 	// get_parameter(name, platform_tags[id]);
	// 	std::cout<<id<<platform_tags[id]<<std::endl;
	// }
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
	tf = tag_create.at(tag_family)();
	td = apriltag_detector_create();

	declare_parameter<float>("decimate", 1.0);
	declare_parameter<float>("blur", 0.0);
	declare_parameter<int>("threads", 1);
	declare_parameter<int>("debug", 0);
	declare_parameter<int>("refine", 1);
	declare_parameter<double>("x_err_thld", 100.0);
	declare_parameter<double>("y_err_thld", 100.0);
	declare_parameter<double>("z_err_thld", 100.0);
	declare_parameter<double>("iter_err_thld", 10.0);

	get_parameter("decimate", td->quad_decimate);
	get_parameter("blur", td->quad_sigma);
	get_parameter("threads", td->nthreads);
	get_parameter("debug", td->debug);
	get_parameter("refine-edges", td->refine_edges);
	get_parameter("x_err_thld", _x_err_thld);
	get_parameter("y_err_thld", _y_err_thld);
	get_parameter("z_err_thld", _z_err_thld);
	get_parameter("iter_err_thld", _iter_err_thld);

	apriltag_detector_add_family(td, tf);


	myfile.open("matlab_data_sim.txt");
	myfile << "det_tags,horiz_dist,vert_dist,timestamp\n";

}

AprilTag2Node::~AprilTag2Node()
{
	apriltag_detector_destroy(td);
	tag_destroy.at(tag_family)(tf);
	myfile.close();
}

void AprilTag2Node::onImage(const sensor_msgs::msg::CompressedImage::SharedPtr msg_img)
{
	// prev = curr;
	// curr = clocksis.now();
	// auto delta = (curr.nanoseconds() - prev.nanoseconds())*0.0000001;
	// std::cout << "delta = " << delta << std::endl;
	// if((double)delta < 100.0)
	// {
	// 	if((double)delta > max)
	// 		max = (double)delta;
	// 	if((double)delta < min)
	// 		min = (double)delta;
	// 	c++;
	// 	sum += (double)delta;
	// 	mean = sum/c;
	//
	// 	std::cout << "max = " << max << std::endl;
	// 	std::cout << "min = " << min << std::endl;
	// 	std::cout << "mean = " << mean << std::endl;
	// }

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
	zarray_t* detections = apriltag_detector_detect(td, im);

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
		geometry_msgs::msg::TransformStamped tf;
		tf.header = msg_img->header;
		// set child frame name by generic tag name or configured tag name
		tf.child_frame_id = tracked_tags.size() ? tracked_tags.at(det->id) : std::string(det->family->name) + ":" + std::to_string(det->id);

		//This function computes the relative translation and rotation of each tag in the scene w.r.t the camera.
		//Then, it projects this information in the centre of the platform (related to the tag in the centre).
		// getPose(*(det->H), tf.transform, det->id, z_up);
		// tf.transform = homography(det, intrinsics, platform_tags[det->id]);
		try{
			tf.transform = pnp(det, intrinsics, platform_tags[det->id]);
		}catch (const cv::Exception& e) {
			// RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
            //              "solvePnP failed: %s", e.what());
			continue;
		}
		// RCLCPP_INFO(get_logger(), "ID: %li ", msg_detection.id);
		// std::cout << "family " << msg_detection.family << std::endl;
		if(std::isnan(tf.transform.translation.x) || std::isnan(tf.transform.translation.y) || std::isnan(tf.transform.translation.z)) continue;
		if(std::abs(tf.transform.translation.x) > 20 || std::abs(tf.transform.translation.y) > 20 || std::abs(tf.transform.translation.z) > 10) continue; 
		msg_detections.detections.push_back(msg_detection);
		tfs.transforms.push_back(tf);
		// RCLCPP_INFO(get_logger(), "ID: %li ", i);
		// msg_relative_pose.header.stamp = this->get_clock()->now();
	}
	// msg_relative_pose.header.stamp = this->get_clock()->now();
	// msg_relative_pose.transform.translation.x = std::isnan(vision_pose.position[0]) ? -1 : vision_pose.position[0];
	// msg_relative_pose.transform.translation.y = std::isnan(vision_pose.position[1]) ? -1 : vision_pose.position[1];
	// msg_relative_pose.transform.translation.z = std::isnan(rotation_yaw) ? -1 : rotation_yaw;
	// pub_platformCenter->publish(tfs[0]);
	if(zarray_size(detections) > 0 && msg_detections.detections.size() > 0) {
		publish_data_center( msg_detections,tfs );
	}

	//pub_detections->publish(msg_detections);
	//pub_tf->publish(tfs);

	apriltag_detections_destroy(detections);

	image_u8_destroy(im);
}
void AprilTag2Node::lowPassFilter(geometry::vision vision_pose){
	rclcpp::Time current_time = this->get_clock()->now();
	if(std::isnan(vision_pose.position[0]) || std::isnan(vision_pose.position[1]) || std::isnan(vision_pose.position[2])) return;
	if (_first_vision_run) {
		_last_vision_time = current_time;
		for(int i=0; i<3; ++i) _filtered_vision_pos[i] = vision_pose.position[i];
		_first_vision_run = false;
	}
	double dt = (current_time - _last_vision_time).seconds();
	_last_vision_time = current_time;

	// 3. Filtering and Validation Logic
	if (dt > 0.0 && dt < 0.5) 
	{
		// Low Pass Filter Math
		double tau = 1.0 / (2.0 * M_PI * _vision_cutoff_freq);
		double alpha = dt / (dt + tau);

		for (int i = 0; i < 3; ++i) {
			_filtered_vision_pos[i] = _filtered_vision_pos[i] + alpha * (vision_pose.position[i] - _filtered_vision_pos[i]);
			vision_pose.position[i] = _filtered_vision_pos[i];
		}
		
	}
	else 
	{
		// Handle large dt or first run by snapping to raw value
		for(int i=0; i<3; ++i) _filtered_vision_pos[i] = vision_pose.position[i];
	}
}

void AprilTag2Node::publish_data_center(const apriltag_msgs::msg::AprilTagDetectionArray& Detections, const tf2_msgs::msg::TFMessage& t_f)
{
	if(Detections.detections.size()>0)
	{
		Eigen::MatrixXd areaID(Detections.detections.size(), 2);
		geometry_msgs::msg::TransformStamped msg_relative_pose;
		geometry::vision vision_pose; //lcm message for vision topic.
		std::array<double, 7> vision_pose2;

		for( int i=0; i< Detections.detections.size(); i++ ) {
			double l1 = sqrt( pow(Detections.detections.at(i).corners.at(1).y - Detections.detections.at(i).corners.at(0).y,2) + pow(Detections.detections.at(i).corners.at(1).x - Detections.detections.at(i).corners.at(0).x,2));
			double l2 = sqrt( pow(Detections.detections.at(i).corners.at(2).y - Detections.detections.at(i).corners.at(1).y,2) + pow(Detections.detections.at(i).corners.at(2).x - Detections.detections.at(i).corners.at(1).x,2));
			double l3 = sqrt( pow(Detections.detections.at(i).corners.at(3).y - Detections.detections.at(i).corners.at(2).y,2) + pow(Detections.detections.at(i).corners.at(3).x - Detections.detections.at(i).corners.at(2).x,2));
			double l4 = sqrt( pow(Detections.detections.at(i).corners.at(0).y - Detections.detections.at(i).corners.at(3).y,2) + pow(Detections.detections.at(i).corners.at(0).x - Detections.detections.at(i).corners.at(3).x,2));

			// RCLCPP_INFO(get_logger(), "Lati: %lf %lf %lf %lf", l1, l2, l3, l4);

			areaID(i,0) = i;
			areaID(i,1) = pow( (l1 + l2 + l3 +l4)/4,2 );

			// RCLCPP_INFO(get_logger(), "Area: %lf %lf", areaID(i,0), areaID(i, 1));
			// RCLCPP_INFO(get_logger(), "DETECTED TAG: %lf", areaID(i,0));
		}

		double weighTot = 0;

		//std::cout<<"detection_size: "<<Detections.detections.size()<< " Rows: "<<areaID.rows()<<std::endl;

		int num_of_valid_ids = 0;
		std::vector<double> xs, ys, zs;

		for( int i = 0; i< areaID.rows(); i++ ) {
			vision_pose.position[0] += t_f.transforms.at( areaID(i, 0) ).transform.translation.x * areaID(i, 1);
			vision_pose.position[1] += t_f.transforms.at( areaID(i, 0) ).transform.translation.y * areaID(i, 1);
			vision_pose.position[2] += t_f.transforms.at( areaID(i, 0) ).transform.translation.z * areaID(i, 1);

			vision_pose.orientation[0] += t_f.transforms.at( areaID(i, 0) ).transform.rotation.w * areaID(i, 1);
			vision_pose.orientation[1] += t_f.transforms.at( areaID(i, 0) ).transform.rotation.x * areaID(i, 1);
			vision_pose.orientation[2] += t_f.transforms.at( areaID(i, 0) ).transform.rotation.y * areaID(i, 1);
			vision_pose.orientation[3] += t_f.transforms.at( areaID(i, 0) ).transform.rotation.z * areaID(i, 1);



			vision_pose2[0] += t_f.transforms.at( areaID(i, 0) ).transform.translation.x * areaID(i, 1);
			vision_pose2[1] += t_f.transforms.at( areaID(i, 0) ).transform.translation.y * areaID(i, 1);
			vision_pose2[2] += t_f.transforms.at( areaID(i, 0) ).transform.translation.z * areaID(i, 1);

			vision_pose2[3] += t_f.transforms.at( areaID(i, 0) ).transform.rotation.w * areaID(i, 1);
			vision_pose2[4] += t_f.transforms.at( areaID(i, 0) ).transform.rotation.x * areaID(i, 1);
			vision_pose2[5] += t_f.transforms.at( areaID(i, 0) ).transform.rotation.y * areaID(i, 1);
			vision_pose2[6] += t_f.transforms.at( areaID(i, 0) ).transform.rotation.z * areaID(i, 1);

			xs.push_back(t_f.transforms.at( areaID(i, 0) ).transform.translation.x);
			ys.push_back(t_f.transforms.at( areaID(i, 0) ).transform.translation.y);
			zs.push_back(t_f.transforms.at( areaID(i, 0) ).transform.translation.z);

			weighTot += areaID(i, 1);

			double xx = t_f.transforms.at( areaID(i, 0) ).transform.translation.x;
			double yy = t_f.transforms.at( areaID(i, 0) ).transform.translation.y;
			double rot_yaw = -1*_drone_pose.yaw;
			auto temp = -(sinf(rot_yaw) * xx + cosf(rot_yaw) * yy);	
			// RCLCPP_INFO(get_logger(), "Area: %lf %lf", areaID(i,0), areaID(i, 1));
			// RCLCPP_INFO(get_logger(), "DETECTED TAG: %lf x %f gt_x_comp %f gtx %f y %f z %f", areaID(i,0),xx,temp,(2 - _drone_pose.position[0]),
			// yy,t_f.transforms.at( areaID(i, 0) ).transform.translation.z);
		}
		// std::cout<<"weighTot: "<<weighTot<<std::endl;
		auto median = [](std::vector<double> v)
		{
			std::sort(v.begin(), v.end());

			size_t n = v.size();

			if(n % 2 == 0)
				return 0.5 * (v[n/2 - 1] + v[n/2]);

			return v[n/2];
		};

		// double median_x = median(xs);
		// double median_y = median(ys);
		// double median_z = median(zs);

		// vision_pose.position[0] = median_x;
		// vision_pose.position[1] = median_y;
		// vision_pose.position[2] = median_z;


		vision_pose.position[0] = vision_pose.position[0] / weighTot;
		vision_pose.position[1] = vision_pose.position[1] / weighTot;
		vision_pose.position[2] = vision_pose.position[2] / weighTot;

		vision_pose.velocity[0] = 0;
		vision_pose.velocity[1] = 0;
		vision_pose.velocity[2] = 0;

		vision_pose.orientation[0] = vision_pose.orientation[0] / weighTot;
		vision_pose.orientation[1] = vision_pose.orientation[1] / weighTot;
		vision_pose.orientation[2] = vision_pose.orientation[2] / weighTot;
		vision_pose.orientation[3] = vision_pose.orientation[3] / weighTot;


		toEulerAngle(vision_pose.orientation,&_roll,&_pitch,&_yaw);
		//moving filter to clean a bit the data

		vision_pose.roll = movingFilter(_roll,&_Queue_roll,&_integral_roll);
		vision_pose.pitch = movingFilter(_pitch,&_Queue_pitch,&_integral_pitch);
		vision_pose.yaw = movingFilter(_yaw,&_Queue_yaw,&_integral_yaw);

		// std::cout << "Z: " << vision_pose.position[2] << std::endl;
		// std::cout << "YAW: " << _drone_pose.yaw*180.0/3.14 << std::endl;


		vision_pose2[0] = vision_pose2[0] / weighTot;
		vision_pose2[1] = vision_pose2[1] / weighTot;
		vision_pose2[2] = vision_pose2[2] / weighTot;

		vision_pose2[3] = vision_pose2[3] / weighTot;
		vision_pose2[4] = vision_pose2[4] / weighTot;
		vision_pose2[5] = vision_pose2[5] / weighTot;
		vision_pose2[6] = vision_pose2[6] / weighTot;

		double horiz_dist_norm = sqrt(pow(vision_pose2[0],2) + pow(vision_pose2[1],2));

		myfile << std::to_string(Detections.detections.size()) + "," + std::to_string(horiz_dist_norm) + "," + std::to_string(vision_pose2[2]) + "," + std::to_string(this->now().seconds()) + "\n";

		// preparing for last rotation: the computed position is wrt drone frame: the whole should be
		// rotated in global frame.
		// creating then a temporary store
		geometry::vision vision_temp;
		// the rotation sign depends on the choosen direction of z as well
		int sign = 1;
		if(z_up)
			sign = -1;

		double rotation_yaw = sign*_drone_pose.yaw;
		// performing a scalar product with a rotation matrix on drone's z-axis: this one
		// rotates the error in the world frame, so that it corresponds to real working environment
		vision_temp.position[0] = (cosf(rotation_yaw) * vision_pose.position[0] - sinf(rotation_yaw) * vision_pose.position[1]);
		vision_temp.position[1] = (sinf(rotation_yaw) * vision_pose.position[0] + cosf(rotation_yaw) * vision_pose.position[1]);

		

		// finally, assigning the computed rotation, adapting it to NED framework
		// (rotations along Z by 90 degrees counterclockwise and X by 180 degrees for z_up)
		// (rotation along Z by 180 degrees for z_down)
		// this is due to the sign of rotation angle
		if(z_up)
		{
			vision_pose.position[0] = -vision_temp.position[1];
			vision_pose.position[1] = vision_temp.position[0];
		}
		else
		{
			vision_pose.position[0] = -vision_temp.position[0];
			vision_pose.position[1] = -vision_temp.position[1];
		}

		// checking whenever we would have anomalies in the computed errors
		if(abs(vision_pose.position[0]) > _x_err_thld)
		{
			vision_pose.position[0] = _previous_x_err;
			// RCLCPP_WARN(get_logger(), "X error is too high, selecting previous one!");
			_x_err_count++;
		}
		else
		{
			// if it's good instead, saving it for the next iteration
			_previous_x_err = vision_pose.position[0];
			_x_err_count = 0;
		}

		// doing the same for Y and Z
		if(abs(vision_pose.position[1]) > _y_err_thld)
		{
			vision_pose.position[1] = _previous_y_err;
			// RCLCPP_WARN(get_logger(), "Y error is too high, selecting previous one!");
			_y_err_count++;
		}
		else
		{
			_previous_y_err = vision_pose.position[1];
			_y_err_count = 0;
		}

		if(abs(vision_pose.position[2]) > _z_err_thld)
		{
			vision_pose.position[2] = _previous_z_err;
			// RCLCPP_WARN(get_logger(), "Z error is too high, selecting previous one!");
			_z_err_count++;
		}
		else
		{
			_previous_z_err = vision_pose.position[2];
			_z_err_count = 0;
		}

		// check on the errors: if it's already some iterations they pass the threshold
		// then we signal the data to be inconsistent; on the other side, the drone will keep the positon.
		// otherwise, everything is ok
		if(_x_err_count > _iter_err_thld || _y_err_count > _iter_err_thld || _z_err_count > _iter_err_thld)
		{
			vision_pose.isValid = false;
		}
		else
		{
			vision_pose.isValid = true;
			// applyChangeLimiter(vision_pose);
			// vision_pose.position[0] = _filtered_vision_pos[1];
			// vision_pose.position[1] = _filtered_vision_pos[0];
			// vision_pose.position[2] = _filtered_vision_pos[2];

			// applyMedianFilter(vision_pose);
			// lowPassFilter(vision_pose);
			// vision_pose.position[0] = _filtered_vision_pos[0];
			// vision_pose.position[1] = _filtered_vision_pos[1];
			// vision_pose.position[2] = _filtered_vision_pos[2];
		}

		// std::cout << "IS DATA VALID: "<< (bool)vision_pose.isValid << std::endl;
		// auto tmp = vision_pose.position[0];
		// vision_pose.position[0] = vision_pose.position[1];
		// vision_pose.position[1] = tmp;
		// std::cout << "xnon filtered " << vision_pose.position[0] << std::endl;
		// std::cout << "z is " << vision_pose.position[2] << std::endl;
		// vision_pose.position[0] = movingFilter(vision_pose.position[0], &_Queue_x,&_integral_x);
		// vision_pose.position[1] = movingFilter(vision_pose.position[1], &_Queue_y,&_integral_y);
		//lcm publication.

		// vision_pose.position[0] = -2 + _drone_pose.position[1];
		// vision_pose.position[1] = 2 + _drone_pose.position[0];

		// vision_pose.position[0] = 2 - _drone_pose.position[0];
		// vision_pose.position[1] = -2 - _drone_pose.position[1];
		
		_handler.publish("apriltag_vision_system",&vision_pose);
		geometry_msgs::msg::TransformStamped msg_tf;
		geometry_msgs::msg::TransformStamped gt_msg_tf;
		if(vision_pose.isValid){
			msg_tf.transform.translation.x = std::isnan(vision_pose.position[0]) ? -1.0 : vision_pose.position[0];
			msg_tf.transform.translation.y = std::isnan(vision_pose.position[1]) ? -1.0 : vision_pose.position[1];
			msg_tf.transform.translation.z = std::isnan(vision_pose.position[2]) ? -1.0 : vision_pose.position[2];
			// std::cout << "det->id " << msg_detection.id << "tf x is "<< msg_tf.transform.translation.x << " msg_detection.decision_margin " << msg_detection.decision_margin << std::endl;
			pub_platformCenter->publish(msg_tf);
			gt_msg_tf.transform.translation.x = 2 - _drone_pose.position[0];
			gt_msg_tf.transform.translation.y = -2 - _drone_pose.position[1];
			pub_platformCenter_gt->publish(gt_msg_tf);
		}
		
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
		// rotate by half rotation about x-axis
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
		myfile << "BABBALA9" << "\n";
	} else if (ID == 10) {
		pose = pose * TAGS.TAG10_T_TAG5;
		myfile << "BABBALA10" << "\n";
	} else if (ID == 11) {
		pose = pose * TAGS.TAG11_T_TAG5;
		myfile << "BABBALA11" << "\n";
	} else if (ID == 12) {
		pose = pose * TAGS.TAG12_T_TAG5;
		myfile << "BABBALA12" << "\n";
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
	_drone_pose.position[0] =  msg_odom->position[1];
	_drone_pose.position[1] =  msg_odom->position[0];
	_drone_pose.position[2] =  -msg_odom->position[2];

	_drone_pose.velocity[0] = msg_odom->velocity[1];
	_drone_pose.velocity[1] = msg_odom->velocity[0];
	_drone_pose.velocity[2] = -msg_odom->velocity[2];

	_drone_pose.orientation[0] = msg_odom->q[0];
	_drone_pose.orientation[1] = msg_odom->q[1];
	_drone_pose.orientation[2] = msg_odom->q[2];
	_drone_pose.orientation[3] = msg_odom->q[3];

	_drone_pose.yaw = atan2(2.0 * (msg_odom->q[0]*msg_odom->q[3] + msg_odom->q[0]*msg_odom->q[2]), 1.0 - 2.0 * (msg_odom->q[2]*msg_odom->q[2] + msg_odom->q[3]*msg_odom->q[3]));
}

void AprilTag2Node::applyChangeLimiter(geometry::vision &vision_pose) {
    rclcpp::Time current_time = this->get_clock()->now();
    
    // 1. Sanity check for NaNs
    if(std::isnan(vision_pose.position[0]) || 
       std::isnan(vision_pose.position[1]) || 
       std::isnan(vision_pose.position[2])) {
        return;
    }

    // 2. Initialize on first run
    if (_first_vision_run) {
        _last_vision_time = current_time;
        for(int i=0; i<3; ++i) _filtered_vision_pos[i] = vision_pose.position[i];
        _first_vision_run = false;
        return;
    }

    double dt = (current_time - _last_vision_time).seconds();
    _last_vision_time = current_time;

    // 3. Apply Slew Rate (Velocity) Limiting
    if (dt > 0.0 && dt < 0.5) 
    {
        double max_step = _max_vision_velocity * dt;

        for (int i = 0; i < 3; ++i) {
            double delta = vision_pose.position[i] - _filtered_vision_pos[i];

            // Clamp the change to +/- max_step
            if (delta > max_step) {
                delta = max_step;
            } else if (delta < -max_step) {
                delta = -max_step;
            }

            _filtered_vision_pos[i] += delta;
            vision_pose.position[i] = _filtered_vision_pos[i];
        }
    }
    else 
    {
        // Handle large dt gaps by snapping to current to avoid huge jumps later
        for(int i=0; i<3; ++i) _filtered_vision_pos[i] = vision_pose.position[i];
    }
}

double AprilTag2Node::getMedian(std::deque<double> buffer) {
    std::vector<double> sort_vec(buffer.begin(), buffer.end());
    std::sort(sort_vec.begin(), sort_vec.end());
    return sort_vec[sort_vec.size() / 2];
}

void AprilTag2Node::applyMedianFilter(geometry::vision &vision_pose) {
    // 1. NaN Guard
    if(std::isnan(vision_pose.position[0])) return;

    // 2. Update Buffers
    _buffer_x.push_back(vision_pose.position[0]);
    _buffer_y.push_back(vision_pose.position[1]);
    _buffer_z.push_back(vision_pose.position[2]);

    // 3. Maintain Window Size
    if (_buffer_x.size() > _median_window_size) {
        _buffer_x.pop_front();
        _buffer_y.pop_front();
        _buffer_z.pop_front();
    }

    // 4. Compute Median (only if buffer is full enough to be effective)
    if (_buffer_x.size() >= 3) {
        vision_pose.position[0] = getMedian(_buffer_x);
        vision_pose.position[1] = getMedian(_buffer_y);
        vision_pose.position[2] = getMedian(_buffer_z);
    }
}
geometry_msgs::msg::Transform
AprilTag2Node::pnp(apriltag_detection_t* const detection, const std::array<double, 4>& intr, double tagsize)
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
		myfile << "BABBALA9" << "\n";
	} else if (detection->id == 10) {
		pose = pose * TAGS.TAG10_T_TAG5;
		myfile << "BABBALA10" << "\n";
	} else if (detection->id == 11) {
		pose = pose * TAGS.TAG11_T_TAG5;
		myfile << "BABBALA11" << "\n";
	} else if (detection->id == 12) {
		pose = pose * TAGS.TAG12_T_TAG5;
		myfile << "BABBALA12" << "\n";
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
// geometry_msgs::msg::Transform
// AprilTag2Node::homography(apriltag_detection_t* const detection, const std::array<double, 4>& intr, double tagsize)
// {
//     apriltag_detection_info_t info = {detection, tagsize, intr[0], intr[1], intr[2], intr[3]};

//     apriltag_pose_t pose;
//     estimate_pose_for_tag_homography(&info, &pose);

//     // rotate frame such that z points in the opposite direction towards the camera
//     for(int i = 0; i < 3; i++) {
//         // swap x and y axes
//         std::swap(MATD_EL(pose.R, 0, i), MATD_EL(pose.R, 1, i));
//         // invert z axis
//         MATD_EL(pose.R, 2, i) *= -1;
//     }

//     return tf2::toMsg<apriltag_pose_t, geometry_msgs::msg::Transform>(const_cast<const apriltag_pose_t&>(pose));
// }

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
