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
	lcm::Subscription *sub  = handler.subscribe("camera_info", &CallbackHandler::cameraInfoCallback, &call);
  lcm::Subscription *sub2 = handler2.subscribe("battery", &CallbackHandler::batteryStatusCallback, &call);
  lcm::Subscription *sub3 = handler3.subscribe("actual_task", &CallbackHandler::actualTaskCallback, &call);
  lcm::Subscription *sub4 = handler4.subscribe("gps_status", &CallbackHandler::gpsStatusCallback, &call);
  // and setting the desired queue capacity
	sub -> setQueueCapacity(1);
	sub2 -> setQueueCapacity(1);
	sub3 -> setQueueCapacity(1);
	sub4 -> setQueueCapacity(1);
  // defining then the structure used to poll from topics
  struct pollfd fds[4];
  // and the interested topics to poll
	fds[0].fd = handler3.getFileno(); // Actual Task
	fds[0].events = POLLIN;

	fds[1].fd = handler2.getFileno(); // Battery Status
	fds[1].events = POLLIN;

	fds[2].fd = handler4.getFileno(); // GPS Status
	fds[2].events = POLLIN;

  fds[3].fd = handler.getFileno(); // Camera Data
	fds[3].events = POLLIN;

  // here begins the task, choosen to loop infinitely (safety has priority)
  while(true)
  {
    // updating the struct
		int ret = poll(fds,4,0);
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
          // TODO: divide patrolling to reach platfrom; if it patrols, it may
          // need to come back to the platform, else, it performs and emergency land.
          // QUEST: should we consider two thresholds?
          // signaling then to Executioner to change modality
          safetyHandler.safety.isbatterygone = 1;
          safetyHandler.safety.params[0] = safetyHandler.getParam(0);
          safetyHandler.safety.params[1] = safetyHandler.getParam(1);
          safetyHandler.safety.params[2] = safetyHandler.getParam(2);
          // and signaling it
          handler6.publish("safety", &safetyHandler.safety);
        }
      }
    }
    // checking if Camera timestamp got updated
    if(fds[3].revents & POLLIN)
    {
      handler.handle();
      // in this case the camera is used only in the Landing phase, so it makes sense
      // to check its integrity during that phase only. If it would be gone before
      // that phase, at least the drone would move in the proximity of the landing plat.
      if(call._task.action == actions::LAND && safetyHandler.handleCamera(call.camera_timestamp))
      {
        // in this case, we signal it.
        safetyHandler.safety.iscameragone = 1;
        handler6.publish("safety", &safetyHandler.safety);
      }
    }
    // checking then if GPS status got updated
    if(fds[2].revents & POLLIN)
    {
      handler4.handle();
      // doing the safety check on GPS status: checking first if the topic is dead
      if(call._task.action != actions::LAND && safetyHandler.handleGPS(call.gps.timestamp))
      {
        // in case, we signal it
        safetyHandler.safety.isgpsgone = 1;
        handler6.publish("safety", &safetyHandler.safety);
      }
      else if(call.gps.status == -1) // if -1, it means it has no Satellite signal.
      {
        // TODO: handle this case differently
        // // increment the counter
        // if(safetyHandler.gpsGoneCounter < sizeof(int))
        // {
        //   safetyHandler.gpsGoneCounter++;
        // }
        // // luckily, in LANDing phase, the drone does not need the gps, so no need to signal.
        // if(call._task.action != actions::LAND && safetyHandler.handleGPS())
        // {
        //   // if instead this case happens, we have to signal it.
        //   safetyHandler.safety.isgpsgone = 1;
        //   handler6.publish("safety", &safetyHandler.safety);
        // }
      }
      // TODO: handle this as well
      // else if(call.gps.status != -1 && safetyHandler.safety.isgpsgone == 1)
      // {
      //   // it may happen that, if the gps looses the signal with the satellite, it may
      //   // get it back somewhen. If that happens while the safety task is running, means we
      //   // can get back to the mission, so we signal it.
      //   safetyHandler.safety.isgpsgone = 0;
      //   handler6.publish("safety", &safetyHandler.safety);
      //   // resetting also the gpsgone counter
      //   safetyHandler.gpsGoneCounter = 0;
      // }
    }
  }
  return 0;
}
