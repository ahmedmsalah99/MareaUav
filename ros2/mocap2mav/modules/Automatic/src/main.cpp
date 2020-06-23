#include <iostream>
#include "lcm/lcm-cpp.hpp"
#include "Automatic.h"
#include "common/CallbackHandler.hpp"
#include "poll.h"
#include "Command/TakeOff.hpp"
#include "utils/TimeHelpers.hpp"
#include "common/MavState.h"

using namespace common;
/** \addtogroup Automatic
 * @brief Main function allowing the automatic
 * implementation of actions for drones
 */

int main(int argc, char** argv){

	lcm::LCM handler, handler2, handler3, handler4, handler5, handler6;

	//the following timer are used to check if both the vision and Ultrasonic sensor are not available for some reason.

	Duration VisionData(1); //timer of 1 second

	Duration UltrasonicData(1); //timer of 1 second


	if (!handler.good() && !handler2.good() && !handler3.good() && !handler4.good() && !handler5.good() && !handler6.good())
		return 1;

	CallbackHandler call;

	Automatic autom;

	Lander lander;

	//Subscription to different topics: state of the drone, vision data, ultrasonic sensor, platform pose, current action

	lcm::Subscription *sub   = handler.subscribe("vision_position_estimate", &CallbackHandler::visionEstimateCallback, &call);//pose of the UAV
	lcm::Subscription *sub2  = handler2.subscribe("Landing_Site/pose", &CallbackHandler::positionSetpointCallback, &call); //absolute pose of the platform
	lcm::Subscription *sub3  = handler3.subscribe("actual_task", &CallbackHandler::actualTaskCallback, &call);
	lcm::Subscription *sub4  = handler4.subscribe("apriltag_vision_system", &CallbackHandler::ApriltagCallback, &call); //topic for the vision system.
	lcm::Subscription *sub5  = handler5.subscribe("UltrasonicSensor/platform", &CallbackHandler::UltrasonicCallback, &call); //topic for the ultrasonic sensor.
	lcm::Subscription *sub6 = handler6.subscribe("safety_ack", &CallbackHandler::safetyAckCallback, &call);


	sub ->setQueueCapacity(1);
	sub2->setQueueCapacity(1);
	sub3->setQueueCapacity(1);
	sub4->setQueueCapacity(1);
	sub5->setQueueCapacity(1);
	sub6->setQueueCapacity(1);


	struct pollfd fds[5];

	//the getFileno() function returns a file descriptor that can
	//be used for asynchronous notification of incoming messages
	//POLLIN means: data may be read without blocking

	fds[0].fd = handler3.getFileno();
	fds[0].events = POLLIN;

	fds[1].fd = handler2.getFileno();
	fds[1].events = POLLIN;

	fds[2].fd = handler4.getFileno();
	fds[2].events = POLLIN;

	fds[3].fd = handler5.getFileno();
	fds[3].events = POLLIN;

	fds[4].fd = handler6.getFileno(); // Safety Acknowledgements
	fds[4].events = POLLIN;

  bool waiting = true;

  //filled here the information of the platform, vision data and ultrasonic sensor

	MavState platform;
	MavState visionSystem;
	MavState UltrasonicPos;


	while(0==handler.handle()){

		autom.setState(call._vision_pos);
    lander.setState(call._vision_pos);

    //It waits for one of a set of file descriptors to become ready to perform I/O
		int ret = poll(fds,5,0);

		if(fds[4].revents & POLLIN)
		{
			handler6.handle();
		}

		//It checks with bitwise AND if the two events are the same
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
			//messages of the platform arrived

			handler2.handle();

			platform = call._position_sp;
      autom.setPlatformState(platform);
			//std::cout<<"x:"<<platform.getX()<<std::endl;
			//std::cout<<"y:"<<platform.getY()<<std::endl;
			//std::cout<<"z:"<<platform.getZ()<<std::endl;
			// std::cout<<"yaw: "<<platform.getYawFromQuat()<<std::endl;

		}

		if(fds[2].revents & POLLIN){
			//every time I entered here there will be data from vision system
			//it is necessary to reset the duration of the timer (the dt will be zero)

			handler4.handle();

			VisionData.updateTimer(); //it sets the actual time
			VisionData._start = VisionData._actualTime; //reset timer

			visionSystem = call._relative_pos;
			visionSystem.VisionDataUpdated = true;
      autom.setVisionFeedback(visionSystem);
			// std::cout<<"x:"<<visionSystem.getX()<<std::endl;
			// std::cout<<"y:"<<visionSystem.getY()<<std::endl;
			// std::cout<<"z:"<<visionSystem.getZ()<<std::endl;

		}
		else {

			if( VisionData.isExpired() ) {
				//the timer is expired, no vision data are available since 1 sec

				visionSystem.VisionDataUpdated = false;
				autom.setVisionFeedback(visionSystem);
			}

		}


		if(fds[3].revents & POLLIN){
			//every time I entered here there will be data from ultrasonic sensor
			//it is necessary to reset the duration of the timer (the dt will be zero)

			handler5.handle();

			UltrasonicData.updateTimer();
			UltrasonicData._start = UltrasonicData._actualTime; //reset timer

			UltrasonicPos = call._Ultrasonic_pos; //copy the information from the callback to the main function.
			UltrasonicPos.UltrasonicDataUpdated = true;
			autom.setUltrasonicInfo(UltrasonicPos);// pass them to the automatic function.

			//DEBUG:
			// std::cout<<"Z:"<<UltrasonicPos.getZ()<<std::endl;
			//std::cout<<"Vz:"<<UltrasonicPos.getVz()<<std::endl;
			//std::cout<<"valid? "<<UltrasonicPos.IsValid<<std::endl;


		}
		else{

			if( UltrasonicData.isExpired() ) {
				//the timer is expired, no ultrasonic sensor data are available since 1 sec

				UltrasonicPos.UltrasonicDataUpdated = false;
        autom.setUltrasonicInfo(UltrasonicPos);

			}


		}




    if(!waiting) {

			//Call commands
			// if the MOVE action is the current one, we set the target as the platform's position
			// Handling the case where it rgoes in safe harbor due to failure
			if(call._task.action == actions::MOVE && !call.safety_ack.isgoinginsafeharbor)
			{
				autom._actualTask.x = platform.getX();
				autom._actualTask.y = platform.getY();
				// autom._actualTask.z = call._task.z;
				// std::cout << "X: " << autom.getState().getX() << std::endl;
				// std::cout << "Y: " << autom.getState().getY() << std::endl;
				// std::cout << "Z: " << autom.getState().getZ() << std::endl;
        // std::cout << "X platform: " << autom._actualTask.x << std::endl;
				// std::cout << "Y platform: " << autom._actualTask.y << std::endl;
        // std::cout << "Z desired: " << autom._actualTask.z << std::endl;
        // std::cout << "*******************************************" << std::endl;
			}
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
