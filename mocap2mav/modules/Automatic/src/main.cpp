#include <iostream>
#include "lcm/lcm-cpp.hpp"
#include "mocap/include/Automatic.h"
#include "common/CallbackHandler.hpp"
#include "poll.h"
#include "utils/TimeHelpers.hpp"
#include "common/MavState.h"
//#include "common/Parameters.hpp"

using namespace common;

int main(int argc, char** argv){

	Duration VisionData(1); //Timer of 1 second

	lcm::LCM handler, handler2, handler3, handler4;

	if (!handler.good() && !handler2.good() && !handler3.good() && !handler4.good())
		return 1;

	CallbackHandler call;

	//Load parameters
	//TODO: create param clas and feed it to lander

	Automatic autom;
	Lander lander;

	lcm::Subscription *sub   = handler.subscribe("vision_position_estimate", &CallbackHandler::visionEstimateCallback, &call);

	lcm::Subscription *sub2  = handler2.subscribe("Landing_Site/pose", &CallbackHandler::positionSetpointCallback, &call);
	//topic for platform pose

	lcm::Subscription *sub4  = handler4.subscribe("apriltag_vision_system", &CallbackHandler::ApriltagCallback, &call);
	//topic of the vision system (Apriltag)

	lcm::Subscription *sub3  = handler3.subscribe("actual_task", &CallbackHandler::actualTaskCallback, &call);

	sub ->setQueueCapacity(1);

	sub2->setQueueCapacity(1);

	sub3->setQueueCapacity(1);

	sub4->setQueueCapacity(1);


	struct pollfd fds[3];

	fds[0].fd = handler3.getFileno(); // Actual task
	fds[0].events = POLLIN;

	fds[1].fd = handler2.getFileno(); //platform
	fds[1].events = POLLIN;

	fds[2].fd = handler4.getFileno(); //vision feedback
	fds[2].events = POLLIN;



  bool waiting = true;

	MavState platform;
	MavState visionSystem;

	MavState estim;


	while(0==handler.handle()){

		autom.setState(call._vision_pos); //fill the state of the UAV in the automatic class
  	lander.setState(call._vision_pos);


		int ret = poll(fds,3,0);

		//the following instruction checks with bitwise AND if the two events are the same.
		if(fds[0].revents & POLLIN){

			handler3.handle();
			autom.setTask(call._task);

			std::cout<<  "New task arrived with action: " << printAction(autom._actualTask.action) << std::endl;
			// used just to wait for the first task coming
      waiting = false;
			// calling the function that parses the coming command
      autom.handleCommands();
		}

		if(fds[1].revents & POLLIN){

			handler2.handle();

      platform = call._position_sp;
      //DEBUG
			//std::cout<<"x:"<<platform.getX()<<std::endl;
			//std::cout<<"y:"<<platform.getY()<<std::endl;
			//std::cout<<"z:"<<platform.getZ()<<std::endl;
      autom.setPlatformState(platform);
		}

		if(fds[2].revents & POLLIN){

			handler4.handle();

			VisionData.updateTimer(); //it takes the actual time.
			VisionData._start = VisionData._actualTime;
			//every time I entered here there will be data from vision system.
			//It is necessary to reset the duration of the timer(the dt will be 0.)


			visionSystem = call._relative_pos;
			visionSystem.VisionDataUpdated = true; //the vision data are updated

			//DEBUG
			//std::cout<<"x:"<<visionSystem.getX()<<std::endl;
  		//std::cout<<"y:"<<visionSystem.getY()<<std::endl;
  		//std::cout<<"z:"<<visionSystem.getZ()<<std::endl;
      autom.setVisionFeedback(visionSystem);

		}
		else {

			if(VisionData.isExpired()) {//if It passed one second the vision data are not
				//available

				visionSystem.VisionDataUpdated = false; //the vision data are updated
				autom.setVisionFeedback(visionSystem);
			}



		}

		// if the first task didn't come yet, skip; else keep on the algorithm
    if(!waiting) {

      //Call commands
      autom.executeCommand();

      //Prepare LCM stuff
      geometry::pose command = call.mavState2LcmPose(autom._comm);
      geometry::pose platRobPos;

      platRobPos.position[0] = platform.getX();
      platRobPos.position[1] = platform.getY();
      platRobPos.position[2] = platform.getZ();

      platRobPos.velocity[0] = platform.getVx();
      platRobPos.velocity[1] = platform.getVy();
      platRobPos.velocity[2] = platform.getVz();

      handler.publish("local_position_sp", &command);

			//For gazebo visualization with the marker plugin
			handler.publish("Marker/pose_cmd", &command);

      handler3.publish("platRob", &platRobPos);
    }
	}

	return 0;

}
