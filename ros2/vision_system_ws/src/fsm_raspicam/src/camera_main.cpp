//#include "lcm/lcm-cpp.hpp"
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <ctime>
#include <sys/time.h>
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <raspicam/raspicam.h>
#include "jpeglib.h"

#include <FiniteStateMachine.h>
#include "State.h"
#include "CameraStates.h"
using namespace std;

int status = -1;


void chatterCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
	status = msg->data;
}

int main ( int argc,char **argv )
{
	rclcpp::init(argc, argv);

	auto node = rclcpp::Node::make_shared("Camera_main");
	auto sub = node->create_subscription<std_msgs::msg::Float32>("/drone_state", chatterCallback, 1);

	FiniteStateMachine CameraMachine;

	IdleState IdleS(&CameraMachine);
	LandingState LandS(&CameraMachine, 640, 480, 30);
	//SearchState SearchS(&CameraMachine,1280,960,50);
	SearchState SearchS(&CameraMachine, 640, 480, 20);

	IdleS.nextLandingState_ = &LandS;
	IdleS.nextSearchState_ = &SearchS;
	LandS.nextState_ = &IdleS;
	SearchS.nextState_ = &IdleS; 

	CameraMachine.setState(&IdleS);

	int actualState;

	enum flightMode{LAND,SEARCH};

//	bool init = true;

	struct timeval timer;
	struct timeval end;

	rclcpp::WallRate loop_rate(40);


	while (rclcpp::ok()) {
		gettimeofday(&timer,NULL);
		
		actualState = CameraMachine.getActualID();

		if (status == 1 || status == 0) {
			switch(actualState) {
			case(State::states::IDLE):
				std::cout<<"IDLE"<<std::endl;
				IdleS.setMode(status);
//				if(init) {
					//IdleS.setPrevMode();
//					init = false;
//				}
				IdleS.setParameters();
				break;

			case(State::states::LANDING):
				std::cout<<"LANDING"<<std::endl;
				LandS.setMode(status);
				LandS.setParameters();
				break;
		
			case(State::states::SEARCH):
				std::cout<<"SEARCH"<<std::endl;
				SearchS.setMode(status);
				SearchS.setParameters();
				break;
		
			case(State::states::STOP):
				std::cout<<"STOP"<<std::endl;
				break;
			}
		}

		rclcpp::spin_some(node);


		loop_rate.sleep();
		gettimeofday(&end,NULL);
		std::cout<<"rate: "<< 1000000.0 / (double) ((end.tv_sec - timer.tv_sec) * 1000000 + (end.tv_usec - timer.tv_usec))<<std::endl;
	}

	return 0;
}
