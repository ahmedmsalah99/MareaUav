#include "lcm/lcm-cpp.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
// #include "px4_msgs/msg/vehicle_local_position_setpoint.hpp"
// #include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_attitude_setpoint.hpp"
// #include "sensor_msgs/msg/battery_state.hpp"
#include "px4_msgs/msg/battery_status.hpp"
// #include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "px4_msgs/msg/vehicle_gps_position.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
// #include "mavros_msgs/msg/state.hpp"
// #include "px4_msgs/msg/actuator_armed.hpp"
#include "px4_msgs/msg/vehicle_control_mode.hpp"
// #include "mavros_msgs/msg/extended_state.hpp"
#include "px4_msgs/msg/vehicle_land_detected.hpp"
// #include "mavros_msgs/msg/param_set.h"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include <std_msgs/msg/float32.hpp>
#include "lcm_messages/geometry/pose.hpp"
// #include "lcm_messages/geometry/vision.hpp"
#include "lcm_messages/exec/state.hpp"
#include "lcm_messages/exec/battery.hpp"
#include "lcm_messages/exec/gps_status.hpp"
#include "lcm_messages/exec/camera_info.hpp"
#include "lcm_messages/exec/release_payload.hpp"
#include "common/common.h"
#include "common/MavState.h"
#include "common/CallbackHandler.hpp"
#include <poll.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
// #include "apriltag_msgs/msg/april_tag_detection_array.hpp"
// #include "tf2_msgs/msg/tf_message.hpp"
#include <cmath>
// #include <queue>
#include <sys/time.h>
#include <class_loader/register_macro.hpp>
#include "rclcpp/logger.hpp"
#include "rcutils/logging_macros.h"
#include <chrono>
#include <ctime>
#include <string>

using namespace std::chrono_literals;

lcm::LCM handler, handler2, handler3, handler5, handler6, handler7, handler8, handler9;
geometry::pose lcm_pose;
exec::state robot_state;
exec::battery battery_lcm;
exec::camera_info camera_info_lcm;
exec::gps_status gps_status_lcm;
exec::release_payload rel_pay_fb_lcm;

//nav_msgs::Odometry platPos;
auto platPos = std::make_shared<px4_msgs::msg::VehicleOdometry>();

//mavros_msgs::State disarmed;
auto disarmed = std::make_shared<px4_msgs::msg::VehicleControlMode>();

//mavros_msgs::ExtendedState landed;
auto landed = std::make_shared<px4_msgs::msg::VehicleLandDetected>();

CallbackHandler call;


//these variable are used to estimate the position of the camera respect to the UAV.
Eigen::Matrix4d Cam_T_plat;
Eigen::Matrix4d zero_T_plat;
Eigen::Matrix4d zero_T_drone;
Eigen::Matrix4d drone_T_plat;
Eigen::Matrix4d drone_T_camera;


bool drone = false;
bool camera = false;
bool platform = false;

//STOP



// geometry::vision vision_pose; //lcm message for vision topic.

// typedef std::queue<double> queue_t;
// queue_t Queue_roll;
// queue_t Queue_pitch;
// queue_t Queue_yaw;
// double integral_roll = 0;
// double integral_pitch = 0;
// double integral_yaw = 0;
// double roll;
// double pitch;
// double yaw;

bool firstState = true;
bool firstEState = true;

bool offboard_enabled = 0;
//void odometryCallback(nav_msgs::Odometry pose){ //the callback of pose of the UAV computed via the motion capture system.


void odometryCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr pose){

    lcm_pose.position[0] =  pose->y;
    lcm_pose.position[1] =  pose->x;
    lcm_pose.position[2] =  -pose->z;

    lcm_pose.velocity[0] = pose->vy;
    lcm_pose.velocity[1] = pose->vx;
    lcm_pose.velocity[2] = -pose->vz;

    lcm_pose.orientation[0] = pose->q[0];
    lcm_pose.orientation[1] = pose->q[1];
    lcm_pose.orientation[2] = pose->q[2];
    lcm_pose.orientation[3] = pose->q[3];

    lcm_pose.isValid = offboard_enabled;

    handler.publish("vision_position_estimate",&lcm_pose);

}

// double movingFilter(double raw, std::queue<double> *queue_t, double *integral){
//
// 	int window = 12;
// 	if(queue_t->size() == window){
//
// 		*integral += raw - queue_t->front();
// 		queue_t->pop();
//
// 	}
// 	else
// 		*integral +=raw;
//
// 	queue_t->push(raw);
//
// 	return *integral/queue_t->size();
//
//
// }
//
// void toEulerAngle(double quaternion[], double *Roll, double *Pitch, double *Yaw){
//
// 	//roll (x-axis rotation)
// 	double sinr_cosp = 2.0 *( quaternion[0]*quaternion[1] + quaternion[2]*quaternion[3] );
// 	double cosr_cosp = 1.0 - 2.0*( quaternion[1]*quaternion[1] + quaternion[2]*quaternion[2] );
//
// 	*Roll = atan2( sinr_cosp, cosr_cosp );
//
// 	//pitch (y-axis rotation)
// 	double sinp = 2.0*( quaternion[0]*quaternion[2] + quaternion[3]*quaternion[1] );
//
// 	if( fabs(sinp) >= 1 )
//
// 		*Pitch = copysign(M_PI/2,sinp); //use 90 degrees if out of range
//
// 	else
// 		*Pitch = asin(sinp);
//
// 	//yaw (z-axis rotation)
// 	double siny_cosp = 2.0 *( quaternion[0]*quaternion[3] + quaternion[1]*quaternion[2] );
// 	double cosy_cosp = 1.0 - 2.0*( quaternion[2]*quaternion[2] + quaternion[3]*quaternion[3] );
// 	*Yaw = atan2(siny_cosp,cosy_cosp);
//
// }

void batteryStatusCallback(const px4_msgs::msg::BatteryStatus::SharedPtr bs)
{
  battery_lcm.voltage = bs->voltage_v;
  battery_lcm.current = bs->current_a;
  battery_lcm.remaining = bs->remaining;


  handler5.publish("battery", &battery_lcm);
}

void gpsStatusCallback(const px4_msgs::msg::VehicleGpsPosition::SharedPtr vgp)
{
  // scaling the position to the conventional one (the topic gives it in integers)
  gps_status_lcm.lon = 0.0000001 * vgp->lon;
  gps_status_lcm.lat = 0.0000001 * vgp->lat;
  gps_status_lcm.alt = 0.001 * vgp->alt;
  gps_status_lcm.status = vgp->fix_type;
  gps_status_lcm.timestamp = (double)(std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1));
  handler6.publish("gps_status", &gps_status_lcm);
}

void CameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr ci)
{
    // saving the time the callback gets called.
    // this is used for safety purposes, to know if the camera is dead.
    camera_info_lcm.timestamp = (double)(std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1));
    handler7.publish("camera_info", &camera_info_lcm);
}

void ReleasePayloadFbCallback(const std_msgs::msg::String::SharedPtr s)
{
    rel_pay_fb_lcm.data = s->data;
    handler8.publish("release_payload_feedback", &rel_pay_fb_lcm);
}

void EStateCallback(const px4_msgs::msg::VehicleLandDetected::SharedPtr es){

    if (firstEState) {
        landed->landed = es->landed;
        firstEState = false;
    }

    if(landed->landed == es->landed) robot_state.landed = 1;
    else robot_state.landed = 0;

}

void controlModeCallback(const px4_msgs::msg::VehicleControlMode::SharedPtr vcm)
{

    offboard_enabled = vcm->flag_control_offboard_enabled;

    if (firstState)
    {
        disarmed->flag_armed = vcm->flag_armed;
        firstState = false;
    }

    //Store arming state
    if(disarmed->flag_armed == vcm->flag_armed) robot_state.armed = 0;
    else robot_state.armed = 1;
}


// void ApriltagCallback(const geometry_msgs::msg::TransformStamped::SharedPtr platform_det){
//
//     vision_pose.position[0] = platform_det->transform.translation.x;
//     vision_pose.position[1] = platform_det->transform.translation.y;
//     vision_pose.position[2] = platform_det->transform.translation.z;
//
//     vision_pose.velocity[0] = 0;
//     vision_pose.velocity[1] = 0;
//     vision_pose.velocity[2] = 0;
//
//     vision_pose.orientation[0] = platform_det->transform.rotation.w;
//     vision_pose.orientation[1] = platform_det->transform.rotation.x;
//     vision_pose.orientation[2] = platform_det->transform.rotation.y;
//     vision_pose.orientation[3] = platform_det->transform.rotation.z;
//
//   	//DEBUG
//   	// std::cout<<"x: "<<vision_pose.position[0]<<std::endl;
//     // std::cout<<"y: "<<vision_pose.position[1]<<std::endl;
// 		// std::cout<<"z: "<<vision_pose.position[2]<<std::endl;
//
//     toEulerAngle(vision_pose.orientation,&roll,&pitch,&yaw);
// 		//moving filter to clean a bit the data
//
// 		vision_pose.roll = movingFilter(roll,&Queue_roll,&integral_roll);
// 		vision_pose.pitch = movingFilter(pitch,&Queue_pitch,&integral_pitch);
// 		vision_pose.yaw = movingFilter(yaw,&Queue_yaw,&integral_yaw);
//
//     //lcm publication.
//     handler4.publish("apriltag_vision_system",&vision_pose);
//
// }


int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("ros2lcm_bridge");

    //topic with position and velocity of the robot
    auto odometry_sub = node->create_subscription<px4_msgs::msg::VehicleOdometry>("/VehicleOdometry_PubSubTopic", 1, &odometryCallback);
    // another one to know the actual control mode
    auto control_mode_sub = node->create_subscription<px4_msgs::msg::VehicleControlMode>("/VehicleControlMode_PubSubTopic", 1, &controlModeCallback);

    // auto state_sub = node->create_subscription<px4_msgs::msg::ActuatorArmed>("/ActuatorArmed_PubSubTopic", 1, &stateCallback);

    auto land_detected_sub = node->create_subscription<px4_msgs::msg::VehicleLandDetected>("/VehicleLandDetected_PubSubTopic", 1, &EStateCallback);

    //vision system
    // auto relative_pose_sub = node->create_subscription<geometry_msgs::msg::TransformStamped>("/platform_center", 1, &ApriltagCallback);


    auto battery_status_sub = node->create_subscription<px4_msgs::msg::BatteryStatus>("/BatteryStatus_PubSubTopic", 1, &batteryStatusCallback);
    auto gps_status_sub = node->create_subscription<px4_msgs::msg::VehicleGpsPosition>("/VehicleGpsPosition_PubSubTopic", 1, &gpsStatusCallback);
    auto camera_info_sub = node->create_subscription<sensor_msgs::msg::CameraInfo>("/apriltag/image/camera_info", 1, &CameraInfoCallback);

    auto pay_rel_fb_sub = node->create_subscription<std_msgs::msg::String>("release_payload_feedback", 1, &ReleasePayloadFbCallback);
    //publishers
    // auto pub = node->create_publisher<px4_msgs::msg::VehicleLocalPositionSetpoint>("/VehicleLocalPositionSetpoint_PubSubTopic",1);
    //
    // // auto pub1 = node->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/TrajectorySetpoint_PubSubTopic",1);
    //
    // // auto pub1 = node->create_publisher<px4_msgs::msg::VehicleOdometry>("/global_platform_position",1);
    //
    // auto pub2 = node->create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>("/VehicleAttitudeSetpoint_PubSubTopic",1);

    auto pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/setpoint_position/local",1);

    auto pub1 = node->create_publisher<std_msgs::msg::String>("/release_payload",1);

    auto pub2 = node->create_publisher<geometry_msgs::msg::TwistStamped>("/mavros/setpoint_velocity/cmd_vel",1);

    auto pub3 = node->create_publisher<std_msgs::msg::Float32>("/drone_state",1);



    //LCM stuff
    lcm::Subscription *sub2 = handler2.subscribe("local_position_sp", &CallbackHandler::positionSetpointCallback, &call);

    lcm::Subscription *sub3 = handler3.subscribe("release_payload", &CallbackHandler::releasePayloadCallback, &call);

    lcm::Subscription *sub4 = handler9.subscribe("actual_task", &CallbackHandler::actualTaskCallback, &call);

    sub2->setQueueCapacity(1);
    sub3->setQueueCapacity(1);
    sub4->setQueueCapacity(1);

    struct pollfd fds[3];

    fds[0].fd = handler2.getFileno(); // Robot position
    fds[0].events = POLLIN;

    fds[1].fd = handler3.getFileno(); // Release Payload
    fds[1].events = POLLIN;

    fds[2].fd = handler9.getFileno(); // Actual Task
    fds[2].events = POLLIN;


    robot_state.landed = 1;
    robot_state.armed  = 0;

    //main loop
    rclcpp::Rate loop_rate(30);


    int stateRate = 0;
    int*  payloadDataRec = new int(0);
    int*  robotDataRec    = new int(0);
    int*  actualTaskRec    = new int(0);

    struct timeval time_;

    while (rclcpp::ok())
    {

        //Poll file descriptors
        int ret = poll(fds,3,0);

        *robotDataRec = fds[0].revents & POLLIN;

        *payloadDataRec = fds[1].revents & POLLIN;

        *actualTaskRec = fds[2].revents & POLLIN;

	      //Release Payload POLLIN
	      if(*payloadDataRec)
        {

            handler3.handle();

            auto relPayData = std::make_shared<std_msgs::msg::String>();
            relPayData->data = call.release_payload.data;
	          pub1->publish(*relPayData);

        }

        // Actual Task POLLIN
        if(*actualTaskRec)
        {
            handler9.handle();
            // this pub is needed for the fsm_raspicam package, that runs on RPi
            auto droneStateData = std::make_shared<std_msgs::msg::Float32>();
            // depending on the actual drone state, the camera state needs to be updated
            if(call._task.action == common::actions::LAND)
            {
                droneStateData->data = 0;
            }
            else if(call._task.action == common::actions::SEARCH)
            {
                droneStateData->data = 1;
            }
            else if(call._task.action == common::actions::EMERG_LAND || robot_state.landed == 1)
            {
                // STOP state: when the camera is not needed
                droneStateData->data = 2;
            }
            else
            {
                // IDLE state
                droneStateData->data = 3;
            }
            pub3->publish(*droneStateData);
        }

        //Position Command POLLIN
        if(*robotDataRec){

            if(call._position_sp.getType() == MavState::type::POSITION) {

                // auto commandPose = std::make_shared<px4_msgs::msg::VehicleLocalPositionSetpoint>();
                // // auto commandPose1 = std::make_shared<px4_msgs::msg::TrajectorySetpoint>();
                // auto commandPose2 = std::make_shared<px4_msgs::msg::VehicleAttitudeSetpoint>();
                //
                // handler2.handle();
                //
                // commandPose->x = call._position_sp.getX();
                // commandPose->y = call._position_sp.getY();
                // commandPose->z = call._position_sp.getZ();
                //
     	          // commandPose->timestamp = node->now().nanoseconds() * 1E-3;
                // //
                // // commandPose1->x = call._position_sp.getX();
                // // commandPose1->y = call._position_sp.getY();
                // // commandPose1->z = call._position_sp.getZ();
                // //
     	          // // commandPose1->timestamp = node->now().nanoseconds() * 1E-3;
                //
                //
                // commandPose2->q_d[0] = call._position_sp.getOrientation().w();
                // commandPose2->q_d[1] = call._position_sp.getOrientation().x();
                // commandPose2->q_d[2] = call._position_sp.getOrientation().y();
                // commandPose2->q_d[3] = call._position_sp.getOrientation().z();
                //
                // commandPose2->timestamp = node->now().nanoseconds() * 1E-3;
                //
                // // std::cout << "command: " << commandPose->x << " " << commandPose->y << " " <<commandPose->z << std::endl;
                // // std::cout << "timestamp1: " << commandPose->timestamp << std::endl;
                // // std::cout << "timestamp2: " << commandPose2->timestamp << std::endl;
                // pub->publish(*commandPose);
                // // pub1->publish(*commandPose1);
                // pub2->publish(*commandPose2);
                auto commandPose = std::make_shared<geometry_msgs::msg::PoseStamped>();

                handler2.handle();

                commandPose->pose.position.x = call._position_sp.getX();
                commandPose->pose.position.y = call._position_sp.getY();
                commandPose->pose.position.z = call._position_sp.getZ();

                commandPose->pose.orientation.x = call._position_sp.getOrientation().x();
                commandPose->pose.orientation.y = call._position_sp.getOrientation().y();
                commandPose->pose.orientation.z = call._position_sp.getOrientation().z();
                commandPose->pose.orientation.w = call._position_sp.getOrientation().w();
                //
                // std::cout << "YAW DES " << call._position_sp.getYaw()*180.0/3.14 << std::endl;
                //
                // geometry::pose pose;
                // pose.orientation[0] = commandPose->pose.orientation.w;
                // pose.orientation[1] = commandPose->pose.orientation.x;
                // pose.orientation[2] = commandPose->pose.orientation.y;
                // pose.orientation[3] = commandPose->pose.orientation.z;
                //
                //
                // double pitch, roll, yaw;
                // toEulerAngle(pose.orientation, &roll, &pitch, &yaw);
                // std::cout << "command: " << commandPose->pose.position.x << " " << commandPose->pose.position.y << " " <<commandPose->pose.position.z << std::endl;
                pub->publish(*commandPose);
            }
            else if(call._position_sp.getType() == MavState::type::VELOCITY) {

                // auto commandPose = std::make_shared<px4_msgs::msg::VehicleLocalPositionSetpoint>();
                // // auto commandPose1 = std::make_shared<px4_msgs::msg::TrajectorySetpoint>();
                //
                //
                // handler2.handle();
                //
                // commandPose->vx =  call._position_sp.getVx();
                // commandPose->vy =  call._position_sp.getVy();
                // commandPose->vz =  call._position_sp.getVz();
                //
                // commandPose->timestamp = node->now().nanoseconds() * 1E-3;
                //
                //
                // // commandPose1->x = call._position_sp.getX();
                // // commandPose1->y = call._position_sp.getY();
                // // commandPose1->z = call._position_sp.getZ();
                // //
     	          // // commandPose1->timestamp = node->now().nanoseconds() * 1E-3;
                //
                // std::cout << "commandV: " << commandPose->vx << " " << commandPose->vy << " " <<commandPose->vz << std::endl;
                // pub->publish(*commandPose);
                // pub1->publish(*commandPose1);
                auto commandPose = std::make_shared<geometry_msgs::msg::TwistStamped>();

                handler2.handle();

                commandPose->twist.linear.x =  call._position_sp.getVx();
                commandPose->twist.linear.y =  call._position_sp.getVy();
                commandPose->twist.linear.z =  call._position_sp.getVz();

                // std::cout << "commandV: " << commandPose->twist.linear.x << " " << commandPose->twist.linear.y << " " <<commandPose->twist.linear.z << std::endl;
                pub2->publish(*commandPose);

            }

	        RCLCPP_INFO_ONCE(rclcpp::get_logger("lcm_bridge"),"Publish ros command");
        }

        //Publish state sometimes
        if (stateRate++ > 10){
            handler2.publish("state",&robot_state);
            stateRate = 0;
        }


        //ROS_INFO_ONCE("Spinning");
	      RCLCPP_INFO_ONCE(rclcpp::get_logger("lcm_bridge"), "spinning");
        rclcpp::spin_some(node);
        loop_rate.sleep();

    }

   // delete platformDataRec;
    delete robotDataRec;

    return 0;

}
