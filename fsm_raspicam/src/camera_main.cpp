#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sys/time.h>
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
	LandingState LandS(&CameraMachine, 640, 480, 20);
	SearchState SearchS(&CameraMachine, 640, 480, 20);
	StopState StopS(&CameraMachine);

	IdleS.nextStopState_ = &StopS;
	IdleS.nextLandingState_ = &LandS;
	IdleS.nextSearchState_ = &SearchS;
	LandS.nextState_ = &IdleS;
	LandS.nextStopState_ = &StopS;
	SearchS.nextState_ = &IdleS;
	SearchS.nextStopState_ = &StopS;
	StopS.nextState_ = &IdleS;
	CameraMachine.setState(&IdleS);

	int actualState;

	struct timeval timer;
	struct timeval end;

	rclcpp::WallRate loop_rate(20);

	int printStateMachine = 0;
	int printRate = 0;

	while (rclcpp::ok()) {
		gettimeofday(&timer,NULL);

		actualState = CameraMachine.getActualID();

		if (status == 1 || status == 0 || status == 2 || status == 3) {
			switch(actualState) {
			case(State::states::IDLE):
				if(printStateMachine >= 10){
					std::cout<<"IDLE"<<std::endl;
					printStateMachine = 0;
				}
				IdleS.setMode(status);
				IdleS.setParameters();
				break;

			case(State::states::LANDING):
				 if(printStateMachine >= 10){
           	std::cout<<"LANDING"<<std::endl;
         	 	printStateMachine = 0;
                                }
				LandS.setMode(status);
				LandS.setParameters();
				break;

			case(State::states::SEARCH):
				if(printStateMachine >= 10){
            std::cout<<"SEARCH"<<std::endl;
            printStateMachine = 0;
                                }
				SearchS.setMode(status);
				SearchS.setParameters();
				break;

			case(State::states::STOP):
				if(printStateMachine >= 10){
            std::cout<<"STOP"<<std::endl;
          	printStateMachine = 0;
				}
				StopS.setMode(status);
				StopS.setParameters();
				break;
			}
		}

		rclcpp::spin_some(node);


		loop_rate.sleep();
		gettimeofday(&end,NULL);

		}
		printRate++;
		printStateMachine++;
	}

	return 0;
}
