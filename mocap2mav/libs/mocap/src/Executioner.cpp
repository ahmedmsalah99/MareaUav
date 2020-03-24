#include <iostream>

#include "include/Executioner.h"

#define PI 3.141592653589
#define GUI

int node = 0;
bool skip = false;

using namespace common;

Executioner::Executioner(){

    _actualNode = 0;
    _idle = false;
    _can_run = false;
    _newTask = true;
    _publish_task = false;
    _nodeList.clear();
    _nodeList.shrink_to_fit();
    isBatterySafetyTaskRunning = 0;
    isGPSSafetyTaskRunning = 0;
    isCameraSafetyTaskRunning = 0;
    isSonarSafetyTaskRunning = 0;
}

void Executioner::init(){

    if(_nodeList.size()>0){

        _can_run = true;
        _newTask = true;
        _publish_task = false;
        _idle = false;
    }
    else{

        _nodeList.clear();
        _nodeList.shrink_to_fit();
        std::cout << "WARNING, empty list"<<std::endl;
        _can_run = false;
        _idle = true;
    }

}

void Executioner::setState(MavState s){

    _state = s;

}

void Executioner::setPlatformState(MavState ps){

    _platState = ps;

}

void Executioner::setList(std::deque<exec::task> list) {

    _nodeList = list;

}

bool Executioner::readyToPublish() {

    if(_publish_task) {
        _publish_task = false;
        return true;
    }
    else return false;

}


void Executioner::run(){

    if(_nodeList.size() > 0) {

        loadTask();
        _can_run = true;
        _idle = false;
    }
    else{
        _can_run = false;

        if (!_idle){
            //Publishing last task, next will be idle
            _publish_task = true;
            _idle = true;
            _actualTask.action = actions ::IDLE;


        }else _publish_task = false;


    }

    //Detect a new task arrived
    if(_newTask) {

        bool done = false;
        char in;
        std::cout<<std::endl;
        std::cout << "***************************************"<<std::endl;
        std::cout << "Performing node: " << node++ << " with action: " << common::printAction(_actualTask.action)<<std::endl;

        if( common::printAction(_actualTask.action).compare("Take off") == 0 ){
#ifdef GUI
        std::cout << "Do you want to proceed? y/n"<<std::endl;
#endif
    }
        std::cout << "***************************************"<<std::endl;
        std::cout<<std::endl;


        do
        {
          if( common::printAction(_actualTask.action).compare("Take off") == 0 )
          {
            #ifdef GUI
              std::cin  >> in;
            #else
              in = 'y';
            #endif
          }
          else
            in = 'y';


          switch (in)
          {
              case 'y':
                  _publish_task = true;
                  done = true;
                  skip = false;
                  break;

              case 'n':

                  _publish_task = false;
                  skip = true;
                  std::cout << "Skipping task, moving to the next" << std::endl;
                  done = true;
                  break;

              default :

                  std::cout << "Wrong command, type y or n" << std::endl;
                  done = false;

            }
        }
        while(!done);
    }

    // Check for next task
    if((CheckActions(_actualTask.action) || skip) && _can_run) {

        if(_nodeList.size() > 0){

            _nodeList.pop_front();
            _nodeList.shrink_to_fit();
            std::cout << "popping, size: "<< _nodeList.size() << std::endl;

            if (_nodeList.empty()) _newTask = false;
            else                   _newTask = true;

        }
        else{
            std::cout<<"Empty list, sending last setpoint"<<std::endl;

            _newTask = false;
        }
    }
    else{
        _newTask = false;
    }

    //std::cout << _nodeList.size() << std::endl;
}

bool Executioner::CheckActions(int a)
{
    int c = a;
    switch (c)
    {
            //MOVE
        case actions::MOVE:
            // returns the distance condition wrt the platform coordinates and the prefixed desired altitude
            return (fabs(_state.getX() - _platState.getX()) < _actualTask.params[5] &&
                    fabs(_state.getY() - _platState.getY()) < _actualTask.params[5] &&
                    fabs(_state.getZ() - _nodeList[_actualNode].z) < _actualTask.params[5] );

            //TAKE_OFF
        case actions::TAKE_OFF:

            return (fabs(_state.getZ()) >= (fabs(_actualTask.params[0]) - _actualTask.params[1]));

            //ROTATE
        case actions::ROTATE:

            return (fabs(fabs(_actualTask.yaw) - fabs(_state.getYawFromQuat())) < _actualTask.params[4]);

            // return (fabs(fabs(_actualTask.yaw) - fabs(_state.getYawFromQuat())) < PI/10);


            //LAND
        case actions::LAND:

            if(_actualTask.params[0] == 1){

                return false;//TODO: implement stopping condition

            }
            else{

                return (_landed);

            }

        case actions::EMERG_LAND:
        {
          // in this case the task never finishes, cause the drone has to signal
          // its gps position once landed.

          // TODO: set condition on velocity on Z, and check if IDLE state works
          // if(_state.getVz() < _actualTask.params[2] && fabs(_state.getZ() - _actualTask.z) < _actualTask.params[5])
          //   return true;

          return false;
        }
        default:
            std::cout << "IDLE" << std::endl;
            return true;
    }

}

void Executioner::setLastTask(const exec::task task){

    _nodeList.shrink_to_fit();
    _nodeList.push_back(task);

}

void Executioner::setNextTask(const exec::task task){

    _nodeList.shrink_to_fit();
    _nodeList.push_front(task);

}

void Executioner::clearList(){

    _nodeList.clear();
    _nodeList.shrink_to_fit();

}

void Executioner::abort() {



}

void Executioner::rtl() {

}

void Executioner::loadTask() {

    _actualTask.x         = _nodeList[_actualNode].x;
    _actualTask.y         = _nodeList[_actualNode].y;
    _actualTask.z         = _nodeList[_actualNode].z;
    _actualTask.yaw       = _nodeList[_actualNode].yaw;
    _actualTask.action    = _nodeList[_actualNode].action;
    _actualTask.id        = _nodeList[_actualNode].id;
    _actualTask.params[0] = _nodeList[_actualNode].params[0];
    _actualTask.params[1] = _nodeList[_actualNode].params[1];
    _actualTask.params[2] = _nodeList[_actualNode].params[2];
    _actualTask.params[3] = _nodeList[_actualNode].params[3];
    _actualTask.params[4] = _nodeList[_actualNode].params[4];
    _actualTask.params[5] = _nodeList[_actualNode].params[5];

}

void Executioner::setStatus(bool armed, bool landed) {

    _armed = armed;
    _landed = landed;
}


void Executioner::batteryGoneHandler(exec::safety safety)
{
  // first thing to do is to clear tasks list, so that noone of them gets executed
  clearList();
  // hence, the safety task has to be build
  exec::task tempTask;
  // resetting the index to access the task list
  _actualNode = 0;
  // setting the action to EMERG_LAND
  tempTask.action = actions::EMERG_LAND;
  // with a proper id
  tempTask.id = 100;
  // setting the desired position on x and y as the actual state of the drone,
  // so that it stays on the current position.
  // This happens iff the gps is not gone as well
  if(!safety.isgpsgone)
  {
    tempTask.x = _state.getX();
    tempTask.y = _state.getY();
  }
  else
  {
    //TODO: handle this case.
  }
  // this is the desired position on z the drone has to achieve
  tempTask.z = 0;
  // this one is the first threshold
  tempTask.params[0] = safety.params[0];
  // and this one the corresponding "turn off motors" one
  tempTask.params[1] = safety.params[1];
  // then we have the descendent rate default gain
  tempTask.params[2] = safety.params[2];
  // and the speed threshold on z used to change mode
  // tempTask.params[2] = 0.1;
  // setting the task list then
  setNextTask(tempTask);
  // calling init every time there is a new list
  init();
  // and loading the task
  loadTask();
}

void Executioner::cameraGoneHandler()
{

}

void Executioner::gpsGoneHandler()
{

}

void Executioner::sonarGoneHandler()
{

}
