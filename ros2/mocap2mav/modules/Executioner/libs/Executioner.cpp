#include <iostream>
#include "Executioner.h"


#define PI 3.141592653589
#define GUI

int node = 0;


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
    isGoingInSafeHarbor = 0;

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

void::Executioner::setList(std::deque<exec::task> list) {

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
            	std::cout << "Do you want to proceed? Y/N"<<std::endl;
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

  if(!isGoingInSafeHarbor)
  {
    _nodeList[_actualNode].x = _platState.getX();
    _nodeList[_actualNode].y = _platState.getY();
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
          // in the case the camera safety task runs, the drone has to follow
          // the platform for a while, waiting the system to come back online
          if(isCameraSafetyTaskRunning)
            return false;
          else  // returns the distance condition wrt the platform coordinates and the prefixed desired altitude
            return (fabs(_state.getX() - _nodeList[_actualNode].x) < _actualTask.params[1] &&
                    fabs(_state.getY() - _nodeList[_actualNode].y) < _actualTask.params[1] &&
                    fabs(_state.getZ() - _nodeList[_actualNode].z) < _actualTask.params[1] );
          break;

        //TAKE_OFF
        case actions::TAKE_OFF:

          return (fabs(_state.getZ()) >= (fabs(_actualTask.params[0]) - _actualTask.params[1]));
          break;

        //ROTATE
        case actions::ROTATE:

          return (fabs(fabs(_actualTask.yaw) - fabs(_state.getYawFromQuat())) < _actualTask.params[1]);
          break;

        //LAND
        case actions::LAND:

          if(_actualTask.params[0] == 1){

              return false;//TODO: implement stopping condition

          }
          else{

              return (_landed);

          }
          break;

        //EMERG_LAND
        case actions::EMERG_LAND:
        {
          // in this case the task never finishes, cause the drone has to signal
          // its gps position once landed.

          return false;
          break;
        }

        default:
            std::cout << "IDLE" << std::endl;
            return true;
            break;
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

void Executioner::cameraGoneHandler(exec::safety safety)
{
  // we create the object for the task
  exec::task tempTask;
  // this time we don't clear the task list, cause the camera may come back online,
  // hence the mission could continue
  tempTask.action = actions::MOVE;
  // filling the coordinates to follow, which correspond to platform position
  tempTask.x = _platState.getX();
  tempTask.y = _platState.getY();
  tempTask.z = safety.params[0];
  // setting the alpha as well
  tempTask.params[0] = safety.params[1];
  // and the threshold to satisfy the condition, in case camera would come back online
  tempTask.params[1] = safety.params[2];
  // setting the task list then
  setNextTask(tempTask);
  // calling init every time there is a new list
  init();
  // and loading the task
  loadTask();
}

void Executioner::moveToSafeHarbor(exec::safety safety)
{
  // first thing to do is to clear tasks list, so that noone of them gets executed
  clearList();
  // hence, the safety tasks have to be build
  exec::task tempTask0;
  exec::task tempTask1;
  exec::task tempTask2;
  // resetting the index to access the task list
  _actualNode = 0;
  // starting with rotating the drone in the direction of the safe harbor
  tempTask0.action = actions::ROTATE;
  // desired coordinates
  tempTask0.x = _state.getX();
  tempTask0.y = _state.getY();
  tempTask0.z = _state.getZ();
  // and yaw
  tempTask0.yaw = safety.params[6];
  // an id
  tempTask0.id = 99;
  // in the rotate task, if this is 0, it computes the desired yaw by using the
  // passed coordinates.
  tempTask0.params[0] = 1;
  // the threshold
  tempTask0.params[1] = safety.params[7];

  // once it's aligned, the next task is MOVE
  tempTask1.action = actions::MOVE;
  // setting then the params of both tasks, starting from Move
  // filling the coordinates of the desired safe harbor place
  tempTask1.x = safety.safeharbor_coord[0];
  tempTask1.y = safety.safeharbor_coord[1];
  tempTask1.z = safety.params[4];
  tempTask1.yaw = safety.params[6];
  // the gain to used to reach it
  tempTask1.params[0] = safety.params[5];
  // and the threshold to satisfy the condition
  tempTask1.params[1] = safety.params[3];

  // proceeding with the last task, the Land
  tempTask2.action = actions::LAND;
  // with a proper id
  tempTask2.id = 100;
  // this is the desired position on z the drone has to achieve
  tempTask2.z = safety.safeharbor_coord[2];
  // this one specify that no platform exists, so the drone has to land normally
  tempTask2.params[0] = 0;
  // finally filling it on the execution list, as the last to be executed
  setNextTask(tempTask2);
  // setting the tasks in order
  setNextTask(tempTask1);
  // so that the alignment is the first to get executed
  setNextTask(tempTask0);

  // calling init every time there is a new list
  init();
  // and loading the task
  loadTask();

}

void Executioner::gpsGoneHandler()
{

}

void Executioner::sonarGoneHandler()
{

}

void Executioner::setSkip()
{
  skip = true;
}

void Executioner::resetLanding()
{
  // resetting the Land task, if it's getting executed
  for(int i = 0; i < _nodeList.size(); i++)
  {
    if(_nodeList[i].action == actions::LAND)
    {
      _nodeList[i].params[3] = 127.0;
    }
  }
}
