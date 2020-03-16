#include <iostream>
#include <fstream>
#include <lcm/lcm-cpp.hpp>
#include <poll.h>
#include "mocap/include/Executioner.h"
#include "common/CallbackHandler.hpp"
#include "Parser.h"
#include "utils/TimeHelpers.hpp"


int main(int argc, char** argv){

    Parser p;
    Executioner e;

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
    std::cout << "Parsing......." << std::endl;

    if(p.parse()){
        std::cout << ".......Parsed" << std::endl;
        e.setList(p.getTaskListParsed());
    }else{

        std::cout << "Unable to parse the file" << std::endl;
        return 1;
    }
    // Call init everytime you have a new list.
    e.init();
    lcm::LCM handler,handler2, handler3, handler4;

    if (!handler.good())
        return 1;
    CallbackHandler call;


    lcm::Subscription *sub = handler.subscribe("vision_position_estimate", &CallbackHandler::visionEstimateCallback, &call);
    lcm::Subscription *sub2 = handler2.subscribe("state", &CallbackHandler::stateCallback, &call);
    lcm::Subscription *sub3 = handler3.subscribe("Landing_Site/pose", &CallbackHandler::positionSetpointCallback, &call);
    lcm::Subscription *sub4 = handler4.subscribe("safety", &CallbackHandler::safetyCallback, &call);

    sub ->setQueueCapacity(1);

    sub2->setQueueCapacity(1);

    sub3->setQueueCapacity(1);

    sub4->setQueueCapacity(1);

    struct pollfd fds[3];

    fds[0].fd = handler2.getFileno(); // state
    fds[0].events = POLLIN;

    fds[1].fd = handler3.getFileno(); // platform position
    fds[1].events = POLLIN;

    fds[2].fd = handler4.getFileno(); // safety conditions
    fds[2].events = POLLIN;

    Spinner sp(5);
    sp.suppressWarnings();
    sp.useMilli();
    while(sp.ok() && (0==handler.handle()))
    {

        //Fetch land messages from mavros
        int ret = poll(fds,3,0);

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
        // if there are news on the safety topic
        if(fds[2].revents & POLLIN)
        {
          // we have to handle it
          handler4.handle();
          // depending on struct values, we have to take a decision and change task
          // defining then a priority policy, where the battery is the first to be
          // checked, then comes the camera, gps and finally the sonar.
          // depending on where the problem is, a different behaviour is adopted.

          //TODO: gestire caso dove arriva un'altro fail e una security task sta runnando
          if(call.safety.isbatterygone)
          {
            e.batteryGoneHandler(call.safety);
          }
          else if(call.safety.iscameragone)
          {
            e.cameraGoneHandler();
          }
          else if(call.safety.isgpsgone)
          {
            e.gpsGoneHandler();
          }
          else if(call.safety.issonargone)
          {
            e.sonarGoneHandler();
          }
        }
        //Run state machine
        e.setState(call._vision_pos);
        e.run();
        //Publish next task
        if (e.readyToPublish())
        {

            std::cout << "publishing task" << std::endl;
            handler.publish("actual_task", &e._actualTask);

        }

    }
    return 0;
}
