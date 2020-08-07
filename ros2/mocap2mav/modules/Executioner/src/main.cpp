#include <iostream>
#include <fstream>
#include <lcm/lcm-cpp.hpp>
#include "Executioner.h"
#include "common/CallbackHandler.hpp"
#include "lcm_messages/exec/safety_ack.hpp"
#include "Parser.h"
#include <poll.h>
#include "utils/TimeHelpers.hpp"


int main(int argc, char** argv){

    Parser p;
    Executioner e;

    exec::safety_ack safety_ack;
    safety_ack.isbatterysafetytaskrunning = 0;
    safety_ack.isgpssafetytaskrunning = 0;
    safety_ack.iscamerasafetytaskrunning = 0;
    safety_ack.issonarsafetytaskrunning = 0;
    safety_ack.isgoinginsafeharbor = 0;
    safety_ack.iscameragone_timestamp = 0.0;

    //Fetch list path
    std::string listPath;
    std::ifstream listF(".direnv/lists.txt");
    if (listF.is_open()){
         getline(listF,listPath);
    } else {
        std::cout << "Environment paths not set!!" << std::endl;
        return 1;
    }
    p.loadFile(listPath+"/list.txt");
    std::cout << "Parsing ....." << std::endl;

    if(p.parse()){
        std::cout << ".......Parsed" << std::endl;
        e.setList(p.getTaskListParsed());
    }else{

        std::cout << "Unable to parse the file" << std::endl;
        return 1;
    }
    // Call init everytime you have a new list.
    e.init();
    lcm::LCM handler,handler2, handler3, handler4, handler5, handler6;


    if (!handler.good() || !handler2.good() || !handler3.good() || !handler4.good() || !handler5.good() || !handler6.good())
        return 1;


    CallbackHandler call;


    lcm::Subscription *sub = handler.subscribe("vision_position_estimate", &CallbackHandler::visionEstimateCallback, &call);
    lcm::Subscription *sub2 = handler2.subscribe("state", &CallbackHandler::stateCallback, &call);
    lcm::Subscription *sub3 = handler3.subscribe("Landing_Site/pose", &CallbackHandler::positionSetpointCallback, &call);
    lcm::Subscription *sub4 = handler4.subscribe("safety", &CallbackHandler::safetyCallback, &call);
    lcm::Subscription *sub5 = handler6.subscribe("release_payload_feedback", &CallbackHandler::releasePayloadFeedbackCallback, &call);

    sub ->setQueueCapacity(1);

    sub2->setQueueCapacity(1);

    sub3->setQueueCapacity(1);

    sub4->setQueueCapacity(1);

    sub5->setQueueCapacity(1);

    struct pollfd fds[4];

    fds[0].fd = handler2.getFileno(); // state
    fds[0].events = POLLIN;

    fds[1].fd = handler3.getFileno(); // platform position
    fds[1].events = POLLIN;

    fds[2].fd = handler4.getFileno(); // safety conditions
    fds[2].events = POLLIN;

    fds[3].fd = handler6.getFileno(); // release payload feedback
    fds[3].events = POLLIN;

    Spinner sp(5);
    sp.suppressWarnings();
    sp.useMilli();
    while(sp.ok() && (0==handler.handle())){

        //Fetch land messages from mavros

        int ret = poll(fds,4,0);

        if(fds[0].revents & POLLIN)
        {
            handler2.handle();
            e.setStatus(call._armed,call._landed);

        }
        // checking if the current platform position has been updated
        if(fds[1].revents & POLLIN)
        {
          handler3.handle();
          // and if it is, update it.
          e.setPlatformState(call._position_sp);
        }
        // checking if we are getting some feedbacks from the releasing payload package
        if(fds[3].revents & POLLIN)
        {
          handler6.handle();
          // and if it is, update it.
          std::cout << call.relpay_feedback.data << std::endl;

          e.setRelPayFeedback(call.relpay_feedback);
          std::cout << "Feedback Received" << std::endl;
        }
        // if there are news on the safety topic
        if(fds[2].revents & POLLIN)
        {
          // we have to handle it
          handler4.handle();

          // First let's check if the battery Safety task is running, cause if it is,
          // there is no point to execute the others
          if(!e.isBatterySafetyTaskRunning)
          {
            // Second, depending on struct values, we have to take a decision and change task
            // defining then a priority policy, where the battery is the first to be
            // checked, then comes the camera, gps and finally the sonar.
            // depending on where the problem is, a different behaviour is adopted.
            if(call.safety.isbatterygone)
            {
              e.isBatterySafetyTaskRunning = 1;
              e.batteryGoneHandler(call.safety);
              std::cout << "Battery Safety Task Deployed" << std::endl;
              // notifying back the safety handler process
              safety_ack.isbatterysafetytaskrunning = 1;
              handler5.publish("safety_ack", &safety_ack);
            }
            else if(call.safety.isgpsgone && !e.isGPSSafetyTaskRunning) // TODO: if gps does not include tasks, put it outside of if?
            {
              e.isGPSSafetyTaskRunning = 1;
              e.gpsGoneHandler();
              std::cout << "GPS Safety Task Deployed" << std::endl;
              // notifying back the safety handler process
              safety_ack.isgpssafetytaskrunning = 1;
              handler5.publish("safety_ack", &safety_ack);
            }
            else if(call.safety.goingsafeharbor && !e.isGoingInSafeHarbor)
            {
              // if the safe harbor necessity came from a Camera failure,
              // we now notify that the task is not running anymore.
              if(e.isCameraSafetyTaskRunning)

              {
                e.isCameraSafetyTaskRunning = 0;
                safety_ack.iscamerasafetytaskrunning = 0;
              }
              e.isGoingInSafeHarbor = 1;
              e.moveToSafeHarbor(call.safety);
              std::cout << "Safe Harbor Safety Task Deployed" << std::endl;
              // notifying back the safety handler process
              safety_ack.isgoinginsafeharbor = 1;
              // safety_ack.iscameragone_timestamp = 0.1;
              handler5.publish("safety_ack", &safety_ack);
            }
            else if(call.safety.iscameragone && !e.isCameraSafetyTaskRunning && !e.isGoingInSafeHarbor)
            {
              e.isCameraSafetyTaskRunning = 1;
              e.cameraGoneHandler(call.safety);
              std::cout << "Camera Safety Task Deployed" << std::endl;
              // notifying back the safety handler with the timestamp, so he can go in safe harbor
              TimeManager tm;
              safety_ack.iscamerasafetytaskrunning = 1;
              safety_ack.iscameragone_timestamp = (double)tm.getTimeMilliSecond();
              handler5.publish("safety_ack", &safety_ack);
            }
            else if(call.safety.isgoingplatform && !e.isGPSSafetyTaskRunning &&
              !e.isCameraSafetyTaskRunning && !e.isGoingInSafeHarbor)
            {
              e.isGoingToPlatform = 1;
              e.batteryLimitHandler(call.safety);
              std::cout << "Coming To Platform Safety Task Deployed" << std::endl;

              safety_ack.isgoingtoplatform = 1;
              handler5.publish("safety_ack", &safety_ack);
            }
            else if(call.safety.issonargone && !e.isSonarSafetyTaskRunning && !e.isGPSSafetyTaskRunning &&
                    !e.isCameraSafetyTaskRunning && !e.isGoingInSafeHarbor && !e.isGoingToPlatform)
            {
              e.isSonarSafetyTaskRunning = 1;
              e.sonarGoneHandler();
              std::cout << "Sonar Safety Task Deployed" << std::endl;
              // notifying back the safety handler process
              safety_ack.issonarsafetytaskrunning = 1;
              handler5.publish("safety_ack", &safety_ack);
            }


            // TODO: handle the case when the gps starts working again after the
            // safety task has been deployed
            if(e.isGPSSafetyTaskRunning && !call.safety.isgpsgone)
            {
              // TODO
            }

            // handling then the case where the Camera comes back online
            if(e.isCameraSafetyTaskRunning && !call.safety.iscameragone)
            {
              // notifying back the safety handler process
              e.isCameraSafetyTaskRunning = 0;
              safety_ack.iscamerasafetytaskrunning = 0;
              // resetting landing phase
              e.resetLanding();
              handler5.publish("safety_ack", &safety_ack);
            }
          }

        }
        //Run state machine
        e.setState(call._vision_pos);
        e.run();
        //Publish next task
        if (e.readyToPublish()) {

            std::cout << "publishing task" << std::endl;
            handler.publish("actual_task", &e._actualTask);

        }

    }
    return 0;
}
