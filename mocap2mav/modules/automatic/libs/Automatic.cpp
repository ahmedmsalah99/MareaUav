#include "Automatic.h"
using namespace common;
Automatic::Automatic()
{
  _defaultYaw = 0;
  env_sim = (bool)std::getenv("M2M_SIM");
}

MavState Automatic::getState()
{
    return _state;
}

MavState Automatic::getPlatformState()
{
    return _platformState;
}

exec::task Automatic::getTask()
{
    return _actualTask;
}

void Automatic::setState(MavState rState)
{
    _state = rState;
}

void Automatic::setRelPayFeedback(exec::release_payload feedback)
{
    _relpay_feedback.data = feedback.data;
}

void Automatic::setTask(ulisse_exec_msgs::msg::ActualTask rTask)
{
    _actualTask.x=rTask.x;
    _actualTask.y=rTask.y;
    _actualTask.z=rTask.z;
    _actualTask.yaw=rTask.yaw;
    _actualTask.action=rTask.action;
    _actualTask.target=rTask.target;
    _actualTask.params[0]=rTask.params[0];
    _actualTask.params[1]=rTask.params[1];
    _actualTask.params[2]=rTask.params[2];
    _actualTask.params[3]=rTask.params[3];
    _actualTask.params[4]=rTask.params[4];



    if(_actualTask.action == actions::MOVE)
    {
      _defaultYaw = _actualTask.yaw;
    }
    else if(_actualTask.action == actions::SEARCH || _actualTask.action == actions::LAND)
    {
      _actualTask.yaw = _defaultYaw;
    }
}

void Automatic::handleCommands() {
    /*
    The automatic class has a member 'std::unique_ptr<Command> _actualCommand' which is a template of the class Command.
    In the above line, being Idle, Move , Takeoff and Land derived classes from the command one, I set only the variable of the Command one
    which Idle or Move or Land or Takeoff inherits.

    */

    std::cout << "Command: " << printAction(_actualTask.action) <<std::endl;
    switch (_actualTask.action){
        //based on type of action, I will fill the _actualcommand variable.



        case actions::IDLE:
            _actualCommand = std::unique_ptr<Command>(new Idle(&_state,&_comm, nullptr));
            //passing the state of the UAV, and filling the command class.
            break;

        case actions::MOVE:
            _actualCommand = std::unique_ptr<Command>(new Move(&_state,&_comm,&_actualTask));
            break;

        case actions::SEARCH:
            _actualCommand = std::unique_ptr<Command>(new Search(&_state,&_comm,&_actualTask,&_visionRescuePose,&_relpay_feedback));
            break;

        case actions::TAKE_OFF:
            _actualCommand.reset(new TakeOff(&_state,&_comm,&_actualTask));
           // _actualCommand = std::unique_ptr<Command>(new TakeOff(&_state,&_comm,&_actualTask));
            std::cout << "Actual command: TakeOff" << std::endl;
            break;

        case actions::LAND:
            _actualCommand = std::unique_ptr<Command>(new Land(&_state,&_comm,&_actualTask,&_platformState,&_visionFeedbackPose,&_UltrasonicInfo));
            std::cout << "Actual command: Land" << std::endl;
            break;
        case actions::ROTATE:
            _actualCommand = std::unique_ptr<Command>(new Rotate(&_state, &_comm, &_actualTask));
            std::cout << "Actual command: Rotate" << std::endl;
            break;

        case actions::EMERG_LAND:
            _actualCommand = std::unique_ptr<Command>(new EmergLand(&_state,&_comm,&_actualTask));
            std::cout << "Actual command: Emergency Land" << std::endl;
            break;

        default:

            std::cout << "Unknown action" << std::endl;
            return;

    }

}
void Automatic::executeCommand() {
    this->_actualCommand->execute();
}

/*
Inside the Command class there is a function 'virtual void execute() = 0'.
In this way, adding the specificator virtual, it is possible to guarantee a dynamic association.
The real implementation will have to be performed by the derived classes (Idle, Move, TakeOff, Land)

----IF the action is IDLE:
Basically my drone remains in the same position. So I will save the initial state if we have a new task
and there i fill the MavState* _comm object with the actual state of the UAV.

----IF the action is MOVE:
1)Computing the position error between the state of the UAV and the target one.
2)fill the _comm object with the actual state + a value which is basically the position error divided by the distance just
to reduce the boost:).

----IF the action is TAKEOFF:
1)take the actual pose
2)take the height from the params vector
3) fill the _comm

----IF the action is LAND:
1)take the actual position of the drone and a land parameter(_plat).
2)if _plat == 0 ==> simple landing(without z compensation).
3)if _plat == 1 ==> state machine with z compensation.
*/

void Automatic::setPlatformState(MavState pose) {
    _platformState = pose;
}

void Automatic::setVisionFeedback(MavState pose) {
	_visionFeedbackPose = pose;
}

void Automatic::setVisionRescue(MavState pose) {
	_visionRescuePose = pose;
}

void Automatic::setUltrasonicInfo(MavState pose) {
	_UltrasonicInfo = pose;
}
