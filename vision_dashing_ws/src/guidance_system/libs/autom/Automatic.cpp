#include "guidance_system/autom/Automatic.h"
#include "common/RunMode.h"
#include <cmath>
using namespace common;
Automatic::Automatic()
{
  _defaultYaw = 0;
  env_sim = runModeIsGazebo(runModeFromEnv());
  _fenceParams.loadLanding();
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

void Automatic::setRelPayFeedback(const std::string& feedback)
{
    _relpay_feedback.data = feedback;
}

void Automatic::setTask(ulisse_msgs::msg::ActualTask rTask)
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
            // Remember where we took off: a landing abort returns and lands here.
            _takeoffX = _state.getX(); _takeoffY = _state.getY(); _takeoffZ = _state.getZ();
            _takeoffSet = true;
            std::cout << "Take-off point recorded: (" << _takeoffX << ", " << _takeoffY << ", " << _takeoffZ << ")" << std::endl;
            {
                const Parameters& f = _fenceParams;
                if (f.geofenceEnable && !f.geofence.contains(_takeoffX, _takeoffY)) {
                    std::cout << "[GEOFENCE] WARNING: take-off point (" << _takeoffX << ", " << _takeoffY
                              << ") is OUTSIDE the geofence (lab boundary - " << f.geofenceMargin
                              << " m): setpoints will be clamped, and an abort will land inside the fence instead."
                              << std::endl;
                }
            }
            _actualCommand.reset(new TakeOff(&_state,&_comm,&_actualTask));
           // _actualCommand = std::unique_ptr<Command>(new TakeOff(&_state,&_comm,&_actualTask));
            std::cout << "Actual command: TakeOff" << std::endl;
            break;

        case actions::LAND:
        {
            Land* land = new Land(&_state,&_comm,&_actualTask,&_platformState,&_visionFeedbackPose,&_UltrasonicInfo,&_seaState);
            if (_takeoffSet) land->setTakeoffPosition(_takeoffX, _takeoffY, _takeoffZ);
            _actualCommand = std::unique_ptr<Command>(land);
        }
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
    // The bridge forwards XY velocity as a feed-forward on POSITION setpoints,
    // and _comm is shared by every command. Reset XY to NaN (= position-only)
    // each tick so commands that never touch it (Move, TakeOff, ...) don't
    // publish MavState's default 0 -- an active "hold zero XY velocity" --
    // or a stale value left by a previous command.
    _comm.setVx(NAN);
    _comm.setVy(NAN);
    this->_actualCommand->execute();
    applyGeofence();
}

void Automatic::applyGeofence() {
    const Parameters& p = _fenceParams;
    if (!p.geofenceEnable) return;

    // XY: lab polygon shrunk by GEOFENCE_MARGIN_M. Z: ceiling only, no floor
    // (landing must reach the pad).
    const double px = _state.getX(), py = _state.getY(), pz = _state.getZ();
    bool changed = false, xyClamped = false, zClamped = false;

    if (_comm.getType() == MavState::POSITION) {
        double x = _comm.getX(), y = _comm.getY(), z = _comm.getZ();
        xyClamped = p.geofence.clamp(x, y);
        if (z > p.geofenceCeiling) { z = p.geofenceCeiling; zClamped = true; }
        if (xyClamped || zClamped) { _comm.setPosition(x, y, z); changed = true; }
    }

    // Velocity (a VELOCITY command, or the feed-forward on a POSITION one):
    // never push further out while at or beyond a wall.
    double v[3] = {_comm.getVx(), _comm.getVy(), _comm.getVz()};
    bool vChanged = false;
    if (!std::isnan(v[0]) && !std::isnan(v[1]) && (xyClamped || !p.geofence.contains(px, py))) {
        double nx, ny;
        p.geofence.outwardNormal(px, py, nx, ny);
        const double out = v[0] * nx + v[1] * ny;
        if (out > 0.0) { v[0] -= out * nx; v[1] -= out * ny; vChanged = true; }
    }
    if (!std::isnan(v[2]) && v[2] > 0.0 && (zClamped || pz >= p.geofenceCeiling)) { v[2] = 0.0; vChanged = true; }
    if (vChanged) _comm.setV(v[0], v[1], v[2]);

    if (changed || vChanged) {
        static int throttle = 0;
        if (throttle++ % 25 == 0)
            std::cout << "[GEOFENCE] setpoint limited to the lab boundary (margin " << p.geofenceMargin
                      << " m, ceiling " << p.geofenceCeiling << " m)  drone (" << px << ", " << py << ", " << pz << ")"
                      << std::endl;
    }
}

double Automatic::getPredictionHorizon() const {
    if (auto* land = dynamic_cast<Land*>(_actualCommand.get()))
        return land->getPredictionHorizon();
    return -1.0;
}

LandGoDiagnostics Automatic::getGoDiagnostics() const {
    if (auto* land = dynamic_cast<Land*>(_actualCommand.get()))
        return land->getGoDiagnostics();
    return LandGoDiagnostics();
}

bool Automatic::consumeDisarmRequest() const {
    if (auto* land = dynamic_cast<Land*>(_actualCommand.get()))
        return land->consumeDisarmRequest();
    return false;
}

bool Automatic::consumePayloadRelease() const {
    if (auto* search = dynamic_cast<Search*>(_actualCommand.get()))
        return search->consumePayloadRelease();
    return false;
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

void Automatic::setSeaState(SeaState seaState) {
	_seaState = seaState;
}
