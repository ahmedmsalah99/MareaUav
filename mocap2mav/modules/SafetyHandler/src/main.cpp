#include <iostream>
#include <fstream>
#include "lcm/lcm-cpp.hpp"
#include "mocap/include/SafetyHandler.h"
#include "common/CallbackHandler.hpp"
#include "SafetyParser.h"
#include "utils/TimeHelpers.hpp"

using namespace common;

int main(int argc, char** argv)
{
  // declaring objects
  SafetyHandler safetyHandler;
  SafetyParser sp;
  CallbackHandler call;
  //Fetch list path
  std::string listPath;
  std::ifstream listF(".direnv/lists.txt");
  if (listF.is_open())
  {
       getline(listF,listPath);
  }
  else
  {
      std::cout << "Environment paths not set!!" << std::endl;
      return 1;
  }
  // getting ready the file to load
  sp.loadFile(listPath+"/../safety_params.txt");
  std::cout << "Parsing Safety Parameters......." << std::endl;
  // if the parsing is ok
  if(sp.parse())
  {
      std::cout << ".......Parsed Safety Parameters" << std::endl;
      safetyHandler.setParamsStruct(sp.getParamsMapParsed());
  }
  else
  {
      // else we stop execution
      std::cout << "Unable to parse the safety parameters file" << std::endl;
      return 1;
  }

  // defining handlers for topics communication
  lcm::LCM handler, handler2, handler3, handler4, handler5, handler6;

  if(!handler.good())
    return 1;

  // subscribing to needed lcm topics
	lcm::Subscription *sub  = handler.subscribe("vision_position_estimate", &CallbackHandler::visionEstimateCallback, &call);
  lcm::Subscription *sub2 = handler2.subscribe("battery", &CallbackHandler::batteryStatusCallback, &call);
  lcm::Subscription *sub3 = handler3.subscribe("actual_task", &CallbackHandler::actualTaskCallback, &call);
  // and setting the desired queue capacity
	sub -> setQueueCapacity(1);
	sub2 -> setQueueCapacity(1);
	sub3 -> setQueueCapacity(1);
  // defining then the structure used to poll from topics
  struct pollfd fds[3];
  // and the interested topics to poll
	fds[0].fd = handler3.getFileno(); // Actual Task
	fds[0].events = POLLIN;

	fds[1].fd = handler2.getFileno(); // Battery Status
	fds[1].events = POLLIN;
  // here begins the task, choosen to loop infinitely (safety has priority)
  while(true)
  {
    // updating the struct
		int ret = poll(fds,3,0);
    // if some new tasks came to the topic
    if(fds[0].revents & POLLIN)
    {
      // it's getting updated
      handler3.handle();
    }
    // if the battery status got updated
    if(fds[1].revents & POLLIN)
    {
      // we handle it
      handler2.handle();
      // checking then its remaining
      if(safetyHandler.handleBattery(call.battery.remaining))
      {
        // if we are in landing state, just land. else, perform emergency land
        if(call._task.action != actions::LAND)
        {
          // TODO: divide patrolling to reach platofrom, if it patrols, it may
          // need to come back to the platform, else, it performs and emergency land.
          // QUEST: should we consider two thresholds?
          // signaling then to Executioner to change modality

          // TODO: make safety struct static
          exec::safety safety;
          // setting the safety status
          safety.isbatterygone = 1;
          // and signaling it
          handler6.publish("safety", &safety);
        }
      }
    }
  }
  return 0;
}
