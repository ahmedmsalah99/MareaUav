#include <iostream>
#include "guidance_system/exec/Executioner.h"


#define PI 3.141592653589
// #define GUI

int node = 0;


using namespace common;

Executioner::Executioner(){

    _currentTaskIndex = 0;
    _idle = false;
    _can_run = false;
    _newTask = true;
    _publish_task = false;
    _payloadReleased = false;
    _nav_mission_finished = false;
    _first_hold = true;
    clearList();
    isBatterySafetyTaskRunning = 0;
    isGPSSafetyTaskRunning = 0;
    isCameraSafetyTaskRunning = 0;
    isSonarSafetyTaskRunning = 0;
    isGoingInSafeHarbor = 0;
    env_sim = (bool)std::getenv("M2M_SIM");

}

void Executioner::init(){

    if(_taskQueue.size()>0){

        _can_run = true;
        _newTask = true;
        _publish_task = false;
        _idle = false;
    }
    else{

        clearList();
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

    _taskQueue = list;

}

void Executioner::setRelPayFeedback(const std::string& feedback) {

    _relpay_feedback.data = feedback;

}

void Executioner::setNavTaskStatus(const ulisse_msgs::msg::VehicleStatus& nav_task_status) {
    if(nav_task_status.vehicle_state == "Hold" && !_first_hold)
    {
        _nav_mission_finished = true;
    }
    else if(nav_task_status.vehicle_state == "Path_Following" || nav_task_status.vehicle_state == "Go_To")
    {
      _first_hold = false;
      _nav_mission_finished = false;
    }
}

bool Executioner::readyToPublish() {

    if(_publish_task) {
        _publish_task = false;
        return true;
    }
    else return false;

}


void Executioner::run(){
    if(_taskQueue.size() > 0) {

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

  if(_actualTask.target == "platform" && env_sim)
  {
    // if we are on simulation, setting at every iteration the platform state
    // as the desired one
    _actualTask.x = _platState.getX();
    _actualTask.y = _platState.getY();
    // and adjusting the yaw as well, to be more precise on orientation
    double coords[3];
    coords[0] = _actualTask.x;
    coords[1] = _actualTask.y;
    coords[2] = _actualTask.z;

    _actualTask.yaw = computeDesiredYaw(_state, coords);

    // std::cout << _actualTask.yaw*180.0/3.14 << std::endl;
  }
    // Check for next task
    if((CheckActions(_actualTask.action) || skip) && _can_run) {

        if(_taskQueue.size() > 0){

            _taskQueue.pop_front();
            _taskQueue.shrink_to_fit();
            std::cout << "popping, size: "<< _taskQueue.size() << std::endl;

            if (_taskQueue.empty()) _newTask = false;
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

    //std::cout << _taskQueue.size() << std::endl;
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
          {
            // std::cout << "_actualTask.x " << _actualTask.x << " _actualTask.y " << _actualTask.y << " _actualTask.z " << _actualTask.z << std::endl;
            return (fabs(_state.getX() - _actualTask.x) < _actualTask.params[1] &&
                    fabs(_state.getY() - _actualTask.y) < _actualTask.params[1] &&
                    fabs(_state.getZ() - _actualTask.z) < _actualTask.params[1]);// && _nav_mission_finished);
          }
          break;

        //TAKE_OFF
        case actions::TAKE_OFF:

          return (fabs(_state.getZ()) >= (fabs(_actualTask.params[0]) - _actualTask.params[1]));
          break;

        //SEARCH
        case actions::SEARCH:
          if(_relpay_feedback.data == "R" && !_payloadReleased)
          {
            std::cout << "PAYLOAD RELEASED, HOLDING ON TARGET" << std::endl;
            _payloadReleased = true;
            // return true;
          }

          if(_payloadReleased && _nav_mission_finished)
          {
            std::cout << "Rendez-vous with Platform" << std::endl;
            return true;
          }
          // computing then the distance between the platform and the drone,
          // and if it is under a certain threshold AND the payload got released, then the drone can come back.
          // if(_payloadReleased && sqrt(pow((_platState.getX() - _state.getX()),2) + pow((_platState.getY() - _state.getY()),2)) < _actualTask.params[2])
          // {
          //   return true;
          // }
          return false;
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

    _taskQueue.shrink_to_fit();
    _taskQueue.push_back(task);

}

void Executioner::setNextTask(const exec::task task){

    _taskQueue.shrink_to_fit();
    _taskQueue.push_front(task);

}

void Executioner::clearList(){

    _taskQueue.clear();
    _taskQueue.shrink_to_fit();

}

void Executioner::abort() {



}

void Executioner::rtl() {

}

void Executioner::loadTask() {

    _actualTask.x         = _taskQueue[_currentTaskIndex].x;
    _actualTask.y         = _taskQueue[_currentTaskIndex].y;
    _actualTask.z         = _taskQueue[_currentTaskIndex].z;
    // std::cout << "_currentTaskIndex " << _currentTaskIndex << " _actualTask.x " << _actualTask.x << " _actualTask.y " << _actualTask.y << " _actualTask.z " << _actualTask.z << " _taskQueue[_currentTaskIndex].z " << _taskQueue[_currentTaskIndex].z << std::endl;
    _actualTask.yaw       = _taskQueue[_currentTaskIndex].yaw;
    _actualTask.target    = _taskQueue[_currentTaskIndex].target;
    _actualTask.action    = _taskQueue[_currentTaskIndex].action;
    _actualTask.id        = _taskQueue[_currentTaskIndex].id;
    _actualTask.params[0] = _taskQueue[_currentTaskIndex].params[0];
    _actualTask.params[1] = _taskQueue[_currentTaskIndex].params[1];
    _actualTask.params[2] = _taskQueue[_currentTaskIndex].params[2];
    _actualTask.params[3] = _taskQueue[_currentTaskIndex].params[3];
    _actualTask.params[4] = _taskQueue[_currentTaskIndex].params[4];

    if(_actualTask.action == actions::MOVE)
    {
      double coords[3];
      coords[0] = _actualTask.x;
      coords[1] = _actualTask.y;
      coords[2] = _actualTask.z;
      double home_coords[3];
      home_coords[0] = _actualTask.params[2];
      home_coords[1] = _actualTask.params[3];
      home_coords[2] = _actualTask.params[4];
      if(_actualTask.target == "rescue")
      {
        globalToLocalCoords(coords, home_coords);
        _actualTask.x = coords[0];
        _actualTask.y = coords[1];
        _actualTask.z = coords[2];
        _actualTask.yaw = computeDesiredYaw(_state, coords);
      }
      else if(_actualTask.target == "platform")
      {
        if(!env_sim)
        {
          globalToLocalCoords(coords, home_coords);
          _actualTask.x = coords[0];
          _actualTask.y = coords[1];
          _actualTask.z = coords[2];
        }
        _actualTask.yaw = computeDesiredYaw(_state, coords);
      }
      // _defaultYaw = _actualTask.yaw;
      // std::cout << _defaultYaw*180.0/3.14 << std::endl;
    }
    // else if(_actualTask.action == actions::SEARCH || _actualTask.action == actions::LAND)
    // {
    //   _actualTask.yaw = _defaultYaw;
    // }

}

void Executioner::setStatus(bool armed, bool landed) {

    _armed = armed;
    _landed = landed;
}


void Executioner::batteryGoneHandler(const ulisse_msgs::msg::Safety& safety)
{
  // first thing to do is to clear tasks list, so that noone of them gets executed
  clearList();
  // hence, the safety task has to be build
  exec::task tempTask;
  // resetting the index to access the task list
  _currentTaskIndex = 0;
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
    tempTask.x = _state.getX();
    tempTask.y = _state.getY();
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

void Executioner::batteryLimitHandler(const ulisse_msgs::msg::Safety& safety)
{
  // defining the tasks to be filled (MOVE to platform and LAND on it)
  exec::task tempTask1;
  exec::task tempTask2;
  // QUEST: Checking for already existing tasks to copy or filling directly?
  // checking if in the list of the tasks to be executed there exist MOVE to platform
  // and LAND already, starting from the current executed task
  // for(int i = _currentTaskIndex; i < _taskQueue.size(); i++)
  // {
  //   if(_taskQueue[i].action == actions::MOVE && _taskQueue[i].target == "platform")
  //   {
  //   }
  // }
  // APPROACH: filling directly; hence, as first step, the task list wil be cleared
  clearList();
  // starting then with first task
  tempTask1.action = actions::MOVE;
  tempTask1.target = "platform";
  // we fill the coordinates
  tempTask1.x = safety.target_coord[0];
  tempTask1.y = safety.target_coord[1];
  tempTask1.z = safety.params[2];
  // the desired yaw
  tempTask1.yaw = safety.params[3];
  // the alpha
  tempTask1.params[0] = safety.params[0];
  // and the threshold
  tempTask1.params[1] = safety.params[1];

  // filling then the LAND task
  tempTask2.action = actions::LAND;
  tempTask2.params[0] = safety.params[4];
  // setting the task list then
  setNextTask(tempTask2);
  setNextTask(tempTask1);
  // calling init every time there is a new list
  init();
  // and loading the task
  loadTask();
}



void Executioner::cameraGoneHandler(const ulisse_msgs::msg::Safety& safety)
{
  // we create the object for the task
  exec::task tempTask;
  // this time we don't clear the task list, cause the camera may come back online,
  // hence the mission could continue
  tempTask.action = actions::MOVE;
  if(_actualTask.action == actions::LAND)
  {
    // filling the coordinates to follow, which correspond to platform position
    tempTask.x = _platState.getX();
    tempTask.y = _platState.getY();
  }
  else
  {
    // in any other case, we keep the current position
    tempTask.x = _state.getX();
    tempTask.y = _state.getY();
  }
  tempTask.z = safety.params[0];
  tempTask.yaw = _state.getYaw();
  tempTask.target = "platform";
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

void Executioner::moveToSafeHarbor(const ulisse_msgs::msg::Safety& safety)
{
  // first thing to do is to clear tasks list, so that noone of them gets executed
  clearList();
  // hence, the safety tasks have to be build
  exec::task tempTask0;
  exec::task tempTask1;
  exec::task tempTask2;
  // resetting the index to access the task list
  _currentTaskIndex = 0;
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
  tempTask1.x = safety.target_coord[0];
  tempTask1.y = safety.target_coord[1];
  tempTask1.z = safety.params[4];
  tempTask1.yaw = safety.params[6];
  // setting the right target as well
  tempTask1.target = "safe_harbor";
  // the gain to used to reach it
  tempTask1.params[0] = safety.params[5];
  // and the threshold to satisfy the condition
  tempTask1.params[1] = safety.params[3];

  // proceeding with the last task, the Land
  tempTask2.action = actions::LAND;
  // with a proper id
  tempTask2.id = 100;
  // this is the desired position on z the drone has to achieve
  tempTask2.z = safety.target_coord[2];
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
  for(int i = 0; i < _taskQueue.size(); i++)
  {
    if(_taskQueue[i].action == actions::LAND)
    {
      _taskQueue[i].target = "reset_land";
    }
  }
}

// float Executioner::deg2rad(float angle_degree)
// {
//   return (angle_degree*_PI)/180.0f;
// }
//
// void Executioner::globalToLocalCoords(exec::task actualTask)
// {
//   // getting target global coordinates
//   double target_lon = actualTask.x;
//   double target_lat = actualTask.y;
//   double target_alt = actualTask.z;
//   // and defining a common constant
//   double earth_radius = 6371000; // (meters)
//   double polar_radius = 6356752; // (meters)
//
//   // computing the distance between home position and safe harbor one using Haversine formula
//   // float a = sinf((sh_lat - _safety_params.home_latitude)/2)*sinf((sh_lat - _safety_params.home_latitude)/2) +
//   //  					cosf(_safety_params.home_latitude)*cosf(sh_lat)*sinf((sh_lon - _safety_params.home_longitude)/2)*sinf((sh_lon - _safety_params.home_longitude)/2);
//   //
//   // float c = 2.0f*atan2f(sqrtf(a), sqrtf(1.0f-a));
//   //
//   // d = earth_radius * c;
//   double cap_radius_sh = earth_radius*cosf(deg2rad(target_lat));
//   double cap_radius_h = earth_radius*cosf(deg2rad(actualTask.params[3]));
//   double mean_cap_radius = (cap_radius_h + cap_radius_sh)/2.0;
//
//   // converting it
//   _actualTask.x = mean_cap_radius * deg2rad(target_lon - actualTask.params[2]);
//
//   _actualTask.y = polar_radius * deg2rad(target_lat - actualTask.params[3]);
//
//   _actualTask.z = target_alt - actualTask.params[4];
//
//   // std::cout << "Longitude: " << _actualTask.x << std::endl;
//   // std::cout << "Latitude: " <<_actualTask.y << std::endl;
//
// }
//
// float Executioner::computeDesiredYaw(MavState pose, exec::task actualTask)
// {
//   // getting the actual yaw angle
//   double rapp = (pose.getX() - actualTask.x)/(pose.getY() - actualTask.y);
//   // as the tangent angle between these two points
//   double alpha = atanf(rapp);
//   // converting it wrt the local reference frame (x on EST, y on NORTH)
//   double alpha_local = _PI/2.0 - alpha;
//   // std::cout << 180.0*alpha/_PI << std::endl;
//   // checking its orientation then: if the point to be reached
//   // is below (wrt local reference frame) the drone's position, then we fix the orientation
//   if(actualTask.y - pose.getY() < 0.0)
//   {
//     alpha_local = alpha_local - _PI;
//   }
//   // returning then the final desired value
//   return  alpha_local;
// }
