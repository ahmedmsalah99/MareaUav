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

  // initializing safety msg.
  exec::safety safety;
  safety.isbatterygone = 0;
  safety.iscameragone = 0;
  safety.isgpsgone = 0;
  safety.issonargone = 0;
  safety.goingsafeharbor = 0;

  // bool first = true;

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

  if(!handler.good() || !handler2.good() || !handler3.good() || !handler4.good() || !handler5.good() || !handler6.good())
    return 1;

  // subscribing to needed lcm topics
	lcm::Subscription *sub  = handler.subscribe("camera_info", &CallbackHandler::cameraInfoCallback, &call);
  lcm::Subscription *sub2 = handler2.subscribe("battery", &CallbackHandler::batteryStatusCallback, &call);
  lcm::Subscription *sub3 = handler3.subscribe("actual_task", &CallbackHandler::actualTaskCallback, &call);
  lcm::Subscription *sub4 = handler4.subscribe("gps_status", &CallbackHandler::gpsStatusCallback, &call);
  lcm::Subscription *sub5 = handler5.subscribe("safety_ack", &CallbackHandler::safetyAckCallback, &call);
  // and setting the desired queue capacity
	sub -> setQueueCapacity(1);
	sub2 -> setQueueCapacity(1);
	sub3 -> setQueueCapacity(1);
	sub4 -> setQueueCapacity(1);
	sub5 -> setQueueCapacity(1);
  // defining then the structure used to poll from topics
  struct pollfd fds[5];
  // and the interested topics to poll
	fds[0].fd = handler3.getFileno(); // Actual Task
	fds[0].events = POLLIN;

	fds[1].fd = handler2.getFileno(); // Battery Status
	fds[1].events = POLLIN;

	fds[2].fd = handler4.getFileno(); // GPS Status
	fds[2].events = POLLIN;

  fds[3].fd = handler.getFileno(); // Camera Data
	fds[3].events = POLLIN;

  fds[4].fd = handler5.getFileno(); // Safety Acknowledgements
	fds[4].events = POLLIN;

  // here begins the task, choosen to loop infinitely (safety has priority)
  while(true)
  {
    // updating the struct
		int ret = poll(fds,5,0);
    // if some new tasks came to the topic
    if(fds[0].revents & POLLIN)
    {
      // it's getting updated
      handler3.handle();
    }

    // checking if we have answers from the Executioner after a fail
    if(fds[4].revents & POLLIN)
    {
      handler5.handle();
    }

    // if the battery status got updated
    if(fds[1].revents & POLLIN)
    {
      // we handle it
      handler2.handle();
      // checking then its remaining
      if(safetyHandler.handleBattery(call.battery.remaining) && !safety.isbatterygone)
      {
        // if we are in landing state, just land. else, perform emergency land
        if(call._task.action != actions::LAND)
        {
          // TODO: divide patrolling to reach platfrom; if it patrols, it may
          // need to come back to the platform, else, it performs and emergency land.
          // QUEST: should we consider two thresholds?
          // signaling then to Executioner to change modality
          safety.isbatterygone = 1;
          safety.params[0] = safetyHandler.getParam(0);
          safety.params[1] = safetyHandler.getParam(1);
          safety.params[2] = safetyHandler.getParam(2);
          // and signaling it
          handler6.publish("safety", &safety);
          std::cout << "CRITIC: LOW BATTERY!!! Performing Emergency Land" << std::endl;
        }
      }
    }


    // checking if Camera timestamp got updated
    if(fds[3].revents & POLLIN)
    {
      handler.handle();
      // if the camera is dead, and it comes back online, we signal it
      // unless we are already going in safe harbor

      // TODO: check on the distance to choose to come back in mission???
      if(safety.iscameragone && !call.safety_ack.isgoinginsafeharbor)
      {
        safety.iscameragone = 0;
        handler6.publish("safety", &safety);
        std::cout << "GOOD NEWS: CAMERA WORKS AGAIN!!! Coming Back To Mission" << std::endl;
      }
    }

    // FOR TEST CAMERA ONLINE AGAIN
    // if(safety.iscameragone && !call.safety_ack.isgoinginsafeharbor && call.safety_ack.iscamerasafetytaskrunning && call.safety_ack.iscameragone_timestamp == 0.1)
    // {
    //   safety.iscameragone = 0;
    //   handler6.publish("safety", &safety);
    //   std::cout << "GOOD NEWS: CAMERA WORKS AGAIN!!! Coming Back To Mission" << std::endl;
    // }

    // in this case the camera is used only in the Landing phase, so it makes sense
    // to check its integrity during that phase only. If it would be gone before
    // that phase, at least the drone would move in the proximity of the landing plat.
//&& first)
    if(call._task.action == actions::LAND && !safety.iscameragone && safetyHandler.handleCamera(call.camera_timestamp))
    {
      // in this case, we signal it.
      safety.iscameragone = 1;
      // filling with desired altitude, alpha and threshold
      safety.params[0] = safetyHandler.getParam(7);
      safety.params[1] = safetyHandler.getParam(9);
      safety.params[2] = safetyHandler.getParam(10);
      handler6.publish("safety", &safety);
      // first = false;
      std::cout << "CRITIC: CAMERA STOPPED WORKING!!! Safety Task Deployed" << std::endl;
    }

    // if the camera didn't come back online after the second prefixed threshold,
    // we go to a safe harbor place.
    if(safety.iscameragone && call.safety_ack.iscamerasafetytaskrunning && !safety.goingsafeharbor && safetyHandler.handleSafeHarbor(call.safety_ack.iscameragone_timestamp))
    {
      // in case, we signal it
      safety.goingsafeharbor = 1;
      safety.params[0] = safetyHandler.getParam(0);
      safety.params[1] = safetyHandler.getParam(1);
      safety.params[2] = safetyHandler.getParam(2);
      safety.safeharbor_coord[0] = safetyHandler.getParam(3);
      safety.safeharbor_coord[1] = safetyHandler.getParam(4);
      safety.safeharbor_coord[2] = safetyHandler.getParam(5);
      safety.params[3] = safetyHandler.getParam(6);
      safety.params[4] = safetyHandler.getParam(8);
      safety.params[5] = safetyHandler.getParam(9);
      handler6.publish("safety", &safety);
      // std::cout << "CRITIC: Going In Safe Harbor!!!" << std::endl;
    }

    // checking then if GPS status got updated
    if(fds[2].revents & POLLIN)
    {
      handler4.handle();

      if(call.gps.status == -1) // if -1, it means it has no Satellite signal.
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
      // else if(call.gps.status != -1 && safety.isgpsgone == 1)
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

    // doing the safety check on GPS status: checking first if the topic is dead
    // if(call._task.action != actions::LAND && safetyHandler.handleGPS(call.gps.timestamp)  && !safety.isgpsgone)
    // {
    //   // in case, we signal it
    //   safety.isgpsgone = 1;
    //   handler6.publish("safety", &safety);
    //   std::cout << "CRITIC: GPS STOPPED WORKING!!! Predicting Position" << std::endl;
    // }

  }
  return 0;
}
