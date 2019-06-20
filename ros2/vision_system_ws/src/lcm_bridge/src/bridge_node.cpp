#include "lcm/lcm-cpp.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/msg/extended_state.hpp"
#include "lcm_messages/geometry/pose.hpp"
#include "lcm_messages/geometry/vision.hpp"
#include "lcm_messages/exec/state.hpp"
#include "common/MavState.h"
#include "common/CallbackHandler.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <poll.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "apriltag_msgs/msg/april_tag_detection_array.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include <cmath>
#include <queue>
#include <sys/time.h>
#include <class_loader/register_macro.hpp>
#include "rclcpp/logger.hpp"
#include "rcutils/logging_macros.h"


lcm::LCM handler, handler2, handler3, handler4;
geometry::pose lcm_pose;
exec::state robot_state;

//nav_msgs::Odometry platPos;
auto platPos = std::make_shared<nav_msgs::msg::Odometry>();

//mavros_msgs::State disarmed;
auto disarmed = std::make_shared<mavros_msgs::msg::State>();

//mavros_msgs::ExtendedState landed;
auto landed = std::make_shared<mavros_msgs::msg::ExtendedState>();

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



geometry::vision vision_pose; //lcm message for vision topic.

typedef std::queue<double> queue_t;
queue_t Queue_roll;
queue_t Queue_pitch;
queue_t Queue_yaw;
double integral_roll = 0;
double integral_pitch = 0;
double integral_yaw = 0;
double roll;
double pitch;
double yaw;

bool firstState = true;
bool firstEState = true;

//void odometryCallback(nav_msgs::Odometry pose){ //the callback of pose of the UAV computed via the motion capture system.


void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr pose){

    lcm_pose.position[0] =  pose->pose.pose.position.x;
    lcm_pose.position[1] =  pose->pose.pose.position.y;
    lcm_pose.position[2] =  pose->pose.pose.position.z;

    lcm_pose.velocity[0] = pose->twist.twist.linear.x;
    lcm_pose.velocity[1] = pose->twist.twist.linear.y;
    lcm_pose.velocity[2] = pose->twist.twist.linear.z;

    lcm_pose.orientation[0] = pose->pose.pose.orientation.w;
    lcm_pose.orientation[1] = pose->pose.pose.orientation.x;
    lcm_pose.orientation[2] = pose->pose.pose.orientation.y;
    lcm_pose.orientation[3] = pose->pose.pose.orientation.z;

    handler.publish("vision_position_estimate",&lcm_pose);

}

double movingFilter(double raw, std::queue<double> *queue_t, double *integral){
	
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

void toEulerAngle(double quaternion[], double *Roll, double *Pitch, double *Yaw){

	//roll (x-axis rotation)
	double sinr_cosp = 2.0 *( quaternion[3]*quaternion[0] + quaternion[1]*quaternion[2] );
	double cosr_cosp = 1.0 - 2.0*( quaternion[0]*quaternion[0] + quaternion[1]*quaternion[1] );

	*Roll = atan( sinr_cosp/cosr_cosp );

	//pitch (y-axis rotation)
	double sinp = 2.0*( quaternion[3]*quaternion[1] + quaternion[2]*quaternion[0] );
	
	if( fabs(sinp) >= 1 )
		
		*Pitch = copysign(M_PI/2,sinp); //use 90 degrees if out of range

	else
		*Pitch = asin(sinp);

	//yaw (z-axis rotation)
	double siny_cosp = 2.0 *( quaternion[3]*quaternion[2] + quaternion[0]*quaternion[1] );
	double cosy_cosp = 1.0 - 2.0*( quaternion[1]*quaternion[1] + quaternion[3]*quaternion[3] );
	*Yaw = atan2(siny_cosp,cosy_cosp);

}

void stateCallback(const mavros_msgs::msg::State::SharedPtr s){

    if (firstState) {
        disarmed->armed = s->armed;
        firstState = false;
    }

    //Store arming state
    if(disarmed->armed == s->armed) robot_state.armed = 0;
    else robot_state.armed = 1;

}


void EStateCallback(const mavros_msgs::msg::ExtendedState::SharedPtr es){

    if (firstEState) {
        landed->landed_state = es->landed_state;
        firstEState = false;
    }

    if(landed->landed_state == es->landed_state) robot_state.landed = 1;
    else robot_state.landed = 0;

}



void ApriltagCallback(const geometry_msgs::msg::TransformStamped::SharedPtr platform_det){

                vision_pose.position[0] = platform_det->transform.translation.x;
                vision_pose.position[1] = platform_det->transform.translation.y;
                vision_pose.position[2] = platform_det->transform.translation.z;

                vision_pose.velocity[0] = 0;
                vision_pose.velocity[1] = 0;
                vision_pose.velocity[2] = 0;

                vision_pose.orientation[0] = platform_det->transform.rotation.w;
                vision_pose.orientation[1] = platform_det->transform.rotation.x;
                vision_pose.orientation[2] = platform_det->transform.rotation.y;
                vision_pose.orientation[3] = platform_det->transform.rotation.z;

            	//DEBUG
            	std::cout<<"x: "<<vision_pose.position[0]<<std::endl;
                std::cout<<"y: "<<vision_pose.position[1]<<std::endl;
		std::cout<<"z: "<<vision_pose.position[2]<<std::endl;

                toEulerAngle(vision_pose.orientation,&roll,&pitch,&yaw);
		//moving filter to clean a bit the data

		vision_pose.roll = movingFilter(roll,&Queue_roll,&integral_roll);
		vision_pose.pitch = movingFilter(pitch,&Queue_pitch,&integral_pitch);
		vision_pose.yaw = movingFilter(yaw,&Queue_yaw,&integral_yaw);

                handler4.publish("apriltag_vision_system",&vision_pose);
               //lcm publication.

}

int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("ros2lcm_bridge");

    //topic with position and velocity of the robot
    auto odometry_sub = node->create_subscription<nav_msgs::msg::Odometry>("/mavros/local_position/odom", odometryCallback,1);

    auto state_sub = node->create_subscription<mavros_msgs::msg::State>("/mavros/state", stateCallback,1);

    auto state_extended_sub = node->create_subscription<mavros_msgs::msg::ExtendedState>("/mavros/extended_state", EStateCallback,1);

    //vision system
    auto relative_pose_sub = node->create_subscription<geometry_msgs::msg::TransformStamped>("/platform_center", ApriltagCallback,1);

    //publishers
    auto pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/setpoint_position/local",1);

    //auto pub1 = node->create_publisher<nav_msgs::msg::Odometry>("/global_platform_position",1);

    auto pub2 = node->create_publisher<geometry_msgs::msg::TwistStamped>("/mavros/setpoint_velocity/cmd_vel",1);


    //LCM stuff
    lcm::Subscription *sub2 = handler2.subscribe("local_position_sp", &CallbackHandler::positionSetpointCallback, &call);
    //lcm::Subscription *sub3 = handler3.subscribe("Landing_Site/pose"    , &CallbackHandler::visionEstimateCallback, &call);
    sub2->setQueueCapacity(1);
    //sub3->setQueueCapacity(1);

    struct pollfd fds[2];
    fds[0].fd = handler2.getFileno(); // Robot position
    fds[0].events = POLLIN;

    //fds[1].fd = handler3.getFileno(); // Platform position
    //fds[1].events = POLLIN;

    robot_state.landed = 1;
    robot_state.armed  = 0;

    //main loop
    rclcpp::Rate loop_rate(30);


    int stateRate = 0;
    //int*  platformDataRec = new int(0);
    int*  robotDataRec    = new int(0);

    struct timeval time_;

    while (rclcpp::ok()){

        //Poll file descriptors
        int ret = poll(fds,2,0);

	*robotDataRec    = fds[0].revents & POLLIN;

	//*platformDataRec = fds[1].revents & POLLIN;

	//Platform position POLLIN
        /*
	if(*platformDataRec){

            handler3.handle();
            platPos->pose.pose.position.x = call._vision_pos.getX();
            platPos->pose.pose.position.y = call._vision_pos.getY();
            platPos->pose.pose.position.z = call._vision_pos.getZ();

            platPos->twist.twist.linear.x = call._vision_pos.getVx();
            platPos->twist.twist.linear.y = call._vision_pos.getVy();
            platPos->twist.twist.linear.z = call._vision_pos.getVz();

            gettimeofday(&time_,NULL);
            platPos->header.stamp.sec = time_.tv_sec ;
 	    platPos->header.stamp.nanosec = 1000 * time_.tv_usec;

	    pub1->publish(platPos);

        }
	*/

        //Position Command POLLIN
        if(*robotDataRec){

            if(call._position_sp.getType() == MavState::type::POSITION) {

                auto commandPose = std::make_shared<geometry_msgs::msg::PoseStamped>();

                handler2.handle();

                commandPose->pose.position.x = call._position_sp.getX();
                commandPose->pose.position.y = call._position_sp.getY();
                commandPose->pose.position.z = call._position_sp.getZ();

                commandPose->pose.orientation.x = call._position_sp.getOrientation().x();
                commandPose->pose.orientation.y = call._position_sp.getOrientation().y();
                commandPose->pose.orientation.z = call._position_sp.getOrientation().z();
                commandPose->pose.orientation.w = call._position_sp.getOrientation().w();

                std::cout << "command: " << commandPose->pose.position.x << " " << commandPose->pose.position.y << " " <<commandPose->pose.position.z << std::endl;
                pub->publish(commandPose);
            }
            else if(call._position_sp.getType() == MavState::type::VELOCITY) {

                auto commandPose = std::make_shared<geometry_msgs::msg::TwistStamped>();

                handler2.handle();

                commandPose->twist.linear.x =  call._position_sp.getVx();
                commandPose->twist.linear.y =  call._position_sp.getVy();
                commandPose->twist.linear.z =  call._position_sp.getVz();

                std::cout << "commandV: " << commandPose->twist.linear.x << " " << commandPose->twist.linear.y << " " <<commandPose->twist.linear.z << std::endl;
                pub2->publish(commandPose);
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
