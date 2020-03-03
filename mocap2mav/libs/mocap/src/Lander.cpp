//
// Created by andreanistico on 06/02/17.
//

#include <iostream>
#include "StatesClasses.hpp"
#include "Lander.h"
#include "common/conversions.h"


Lander::Lander()
        : _horizontaErr((double)0)    , _tauHold((double)0), _tauLost((double)0), _tauErr((double)0), _NHold(0),
          _NLost(0),_NComp(0), _initS(&_machine), _holdS(&_machine)  , _asceS(&_machine)  , _descS(&_machine),_compS(&_machine),
          _rtolS(&_machine),_landS(&_machine),_inspeS(&_machine),_homingS(&_machine), _err(0,0,0), _err_int(0,0,0), _err_diff(0,0,0), _dt(0), _prevTime(0), _actualTime(0),
          _actualState(0),_prevState(0), _verticalErr(0), _holdPIDX(0,0,0), _holdPIDY(0,0,0)
{

    //set the state machine
    initStateMachine();

    ////Load parameters
    param.loadConfigFile("Landing_params.txt");

    _holdPIDX.reset();
    _holdPIDY.reset();

    _holdPIDX.setP(param.KpHold);
    _holdPIDX.setI(param.KiHold);
    _holdPIDX.setD(param.KdHold);

    _holdPIDY.setP(param.KpHold);
    _holdPIDY.setI(param.KiHold);
    _holdPIDY.setD(param.KdHold);

    _holdPIDX.setMaxIOutput(param.maxIntValue);
    _holdPIDY.setMaxIOutput(param.maxIntValue);
    //put a saturation for the integral part of the PID

    _holdPIDX.setOutputLimits(param.maxOutput);
    //put a saturation for the whole PID

    _tauHold = param.hold_threshold;
    _tauLost = param.lost_threshold;
    //this two parameters are thresholds for the horizontal error.
////

    //stored starting local position
    starting_x = _state.getX();
    starting_y = _state.getY();
    starting_z = _state.getZ();
    _home = false;


    _actualState = _machine.getActualNodeId();
    //set the actual state of the machine, intially is is INIT

    _err_prev = _err;
}

void Lander::setState(MavState pose) {
    _state = pose;
}

MavState Lander::getState() {
    return _state;
}

MavState Lander::getCommand() {
    return _setPoint;
}


void Lander::loadMachine() {

    param.loadConfigFile("Landing_params.txt");

    _holdPIDX.setP(param.KpHold);
    _holdPIDX.setI(param.KiHold);
    _holdPIDX.setD(param.KdHold);

    _holdPIDY.setP(param.KpHold);
    _holdPIDY.setI(param.KiHold);
    _holdPIDY.setD(param.KdHold);

    _holdPIDX.setMaxIOutput(param.maxIntValue);
    _holdPIDY.setMaxIOutput(param.maxIntValue);
    //put a saturation for the integral part of the PID

    _holdPIDX.setOutputLimits(param.maxOutput);
    //put a saturation for the whole PID

    _tauHold = param.hold_threshold;
    _tauLost = param.lost_threshold;

    //stored starting local position
    starting_x = _state.getX();
    starting_y = _state.getY();
    starting_z = _state.getZ();
    _home = false;

}

void Lander::initStateMachine() {

	  Inspection = false;

    switchSensor = false;

    _resetMachine = false;

    //reset variables
    _horizontaErr = (double)0;
    _tauHold = (double)0;
    _tauLost = (double)0;
    _tauErr = (double)0;
    _NHold = 0;
    _NLost = 0;
    _NComp = 0;

    _err << 0,0,0;
    _err_int << 0,0,0;
    _err_diff << 0,0,0;
    _dt = 0;
    _prevTime = 0;
    _actualTime = 0;
    _verticalErr = 0;
    _actualState = 0;
    _prevState = 0;

    //Link signals
    //assign initialized errors of the Lander class with
    //the one of the LandMachine.
    _machine._horizontaErr =  &_horizontaErr;
    _machine._verticalErr  =  &_verticalErr;
    _machine._tauErr       =  &_tauErr;
    _machine._tauHold      =  &_tauHold;
    _machine._tauLost      =  &_tauLost;
    _machine._NLost        =  &_NLost;
    _machine._NHold        =  &_NHold;
    _machine._NComp        =  &_NComp;
    _machine._state        =  &_state;
    _machine._setPoint     =  &_setPoint;
    _machine._VisionPose   =  &_VisionPose;
    _machine.param         =  &param;
    //New signals
    _machine._holding      =  &_holding;
    _machine._centered     =  &_centered;
    _machine._lost         =  &_lost;

    //Link states
    //keep in mind that _nextState object is inherited from the AbstractLandState class.
    //the following instructions set the machine state.
    _initS._nextState       = &_holdS;
	  _initS._nextInspeState  = &_inspeS;
    _initS._nextHomingState = &_homingS;

	  _inspeS._nextState      = &_initS;

    _homingS._nextState     = &_initS;

    _holdS._nextAscState    = &_asceS;
    _holdS._nextDesState    = &_descS;
    _holdS._nextState       = &_rtolS;
    _holdS._nextInitState   = &_initS;

    _asceS._nextState       = &_holdS;

    _descS._nextState       = &_holdS;

    _rtolS._nextComState    = &_compS;
    _rtolS._nextState       = &_holdS;

    _compS._nextState       = &_asceS;
    _compS._nextLanState    = &_landS;

    _landS._nextState       = &_asceS;

    _machine.setStatePtr(&_initS); //set the state with the INIT one.


    //Print actual state
    std::cout << "Actual state: " << _machine.getActualNodeId() << std::endl;

}

void Lander::setPlatformState(const MavState platformState) {
    _platformState = platformState;
}

void Lander::setVisionPose(const MavState VisionPose) {
    _VisionPose = VisionPose;
}


void Lander::updateSignals() {

    if(_VisionPose.VisionDataUpdated){
        Inspection = false;
        _home = false;
    }


    double dx;
    double dy;
    double dz;

    switchSensor = (fabs( _VisionPose.getZ() ) > 0.5);
    //above half meters use vision system for altitude value
    //otherwise, ultrasonic sensors.

    if(switchSensor)
        dz = _VisionPose.getZ();

    else{
        dz = _platformState.getZ() - _state.getZ() + PLATFORM_OFFSET;
    }

    dx = _VisionPose.getX();
    dy = _VisionPose.getY();

    _err[0] = dx;
    _err[1] = dy;
    _err[2] = 0; //Trick, used to calculate horizontal error

    _horizontaErr = _err.norm();
    _err[2]       = dz;
    _verticalErr  = _err[2];
    //set the horizontal and vertical error.

    //Increment N if needed
    if (_horizontaErr < _tauHold && _VisionPose.VisionDataUpdated) {
        _NLost = 0;
        _NHold++;
    }
    else if (_horizontaErr > _tauLost || !_VisionPose.VisionDataUpdated) {
        _NHold = 0;
        _NComp = 0;
        _NLost++;
    }

    if( switchSensor ) { //in this condition is also taken into account the update vision data

        _holding  = (_NHold > param.NFramesHold && _VisionPose.VisionDataUpdated );//Number of consecutive frames in which tracking is considered valid.

        _lost     = (_NLost > param.NFramesLost || !_VisionPose.VisionDataUpdated );//Number of consecutive frames in which tracking is considered not valid.

        _centered = (_horizontaErr < _tauHold * 0.5 && _VisionPose.VisionDataUpdated );

    }
    else{

        _holding  = (_NHold > param.NFramesHold );//Number of consecutive frames in which tracking is considered valid.

        _lost     = (_NLost > param.NFramesLost );//Number of consecutive frames in which tracking is considered not valid.

        _centered = (_horizontaErr < param.comp_threshold );

    }

    if(_actualState == AbstractLandState::states::R2LA || _actualState == AbstractLandState::states::COMP || _actualState == AbstractLandState::states::LAND){
    //RL2A state is between hold and comp states.

        //Reset NComp
        if(_prevState == AbstractLandState::states::HOLD) _NComp = 0;

        //Check whether we are on place to land
        if (_centered) {
            _NComp++;
        } else{
            _NComp = 0;
            _NHold = 0;
        }
    }
    //Check if we are escaping to ascending
    if(_actualState == AbstractLandState::states::ASCE && (_prevState == AbstractLandState::states::COMP ||
                                                           _prevState == AbstractLandState::states::LAND))
    {
        _NComp = 0;
        _NHold = 0;
    }


    _err_prev = _err;

}

void Lander::printSignals(){

#ifdef DEBUG

    std::cout<<"altezza: "<<param.zMax<<std::endl;

    std::cout << "**********************************" << std::endl;
    std::cout << "STATE: " << _actualState<< std::endl;
    std::cout << "HERRO: " << _horizontaErr<< std::endl;
    std::cout << "HERRV: " << _verticalErr<< std::endl;
    std::cout << "switchSensor" << switchSensor<< std::endl;
    std::cout << "visionData" <<_VisionPose.VisionDataUpdated<<std::endl;
    std::cout << "roll: " <<_VisionPose.getRoll()<<std::endl;
    std::cout << "pitch: " <<_VisionPose.getPitch()<<std::endl;
    std::cout << "NHOLD: " << _NHold<< std::endl;
    std::cout << "NLOST: " << _NLost<< std::endl;
    std::cout << "NCOMP: " << _NComp<< std::endl;
    std::cout <<"inspection: "<<Inspection<<std::endl;


    //std::cout << "INTEX: " << _err_int[0] << std::endl;
    //std::cout << "INTEY: " << _err_int[1] << std::endl;
    std::cout << "**********************************" << std::endl;

#endif

}


void Lander::handleMachine() {

    //isValid = 1 ==> OFFBOARD
    //isValid = 0 ==> ANOTHER FLIGHT MODE
    if(!_state.isValid && !_resetMachine){
        initStateMachine();
        _resetMachine = true;
    }
    else if( _state.isValid ){ //OFFBOARD

        if(_resetMachine){
            loadMachine();
            _resetMachine = false;
        }

        updateSignals();

        //--computes the horizontal and vertical error of the actual state.

        //--check if the horizontal error is under a given threshold(_tauHold and _tauLost)
        //incrementing or nullifying NLost or NHold.

        //--then, st the boolean value _holding, lost, centering based on the value of NLost or Nhold.

        //--if the UAV is centered the variable NComp is incremented.


        _machine.handle();
        //Being such function virtual, will be defined by its derived classes.
        //depending on the actual state a different function is called.



}

    std::cout<<"reset"<<_resetMachine<<std::endl;


printSignals();

}

int Lander::getActualMachineState() {
    return _machine.getActualNodeId();
}

void Lander::run() {

    _prevState = _actualState; //set the previous state with the actual one, initially is INIT.

    handleMachine();

    _actualState = _machine.getActualNodeId();//it obtains the new state

    managetime();

    static bool initDone = false;

    switch (_actualState){

        case (AbstractLandState::states::INIT):

            if(!initDone){
           		std::cout<<"INIT"<<std::endl;

                init();

            	initDone = _VisionPose.VisionDataUpdated;

            }

            break;

        case (AbstractLandState::states::INSPE):
            std::cout<<"INSPE"<<std::endl;

            inspection();

            break;

        case (AbstractLandState::states::HOMING):
            std::cout<<"HOMING"<<std::endl;

            homing();

            break;

        case (AbstractLandState::states::HOLD):
            std::cout<<"HOLD"<<std::endl;

            initDone = false;
            clampZSP();//it takes the Z of the UAV in a interval specified.

            hold();
            //control the x and y error position.
            break;

        case (AbstractLandState::states::DESC):
            std::cout<<"DESC"<<std::endl;

            desc();
            //decrease the z position to 0.1 meters.
            clampZSP();
            break;
        case (AbstractLandState::states::ASCE):
            std::cout<<"ASCE"<<std::endl;

            asce();
            //increase the z position to 0.1 meters.
            clampZSP();

            break;

        case (AbstractLandState::states::R2LA):
            std::cout<<"R2LA"<<std::endl;

            clampZSP();
            hold();
            break;

        case (AbstractLandState::states::COMP):
            std::cout<<"COMP"<<std::endl;

            hold();
            comp();
            //compensation of the altitude
            break;
        case (AbstractLandState::states::LAND):
            std::cout<<"LAND"<<std::endl;

            land();
            //landing phase.
            break;

        default:
            hold();
            break;
    }


}

void Lander::resetSetPoint() {

    _setPoint = _state;
    _setPoint.setType(MavState::type::POSITION);

}

void Lander::updateIntegrals() {

    _err_int[0] +=  _dt * _err[0];
    _err_int[1] +=  _dt * _err[1];

    double tempx,tempy;


    tempx = common::clamp(_err_int[0],param.minIntValue,param.maxIntValue);
    tempy = common::clamp(_err_int[1],param.minIntValue,param.maxIntValue);



    _err_int[0] =  tempx;
    _err_int[1] =  tempy;

}

void Lander::managetime() {

    _actualTime = common::time::getTimeMilliSecond();
    double dt = _prevTime != 0 ? (_actualTime - _prevTime) * (double)MILLI2SECS : 0.0;
    _prevTime = _actualTime;
    _dt = dt;

}

void Lander::resetIntegrals() {

    _err_int[0] = 0;
    _err_int[1] = 0;

}

void Lander::init() {

    //Set point to my position
    resetSetPoint();

    //Go to max tracking height
    //_setPoint.setZ(params_automatic::zMax);
    _setPoint.setZ(param.zMax);

}

void Lander::initInspection() {

	time = 0;
	X0 = _state.getX();
	Y0 = _state.getY();
	Inspection = true;

    RadiusInspection = param.inspeRadius; // radius of 1 meter
    Period = (2*PI*RadiusInspection)/param.inspeLinVel;

}

void Lander::inspection() {
//This function implements a circular trajectory

    if(!Inspection)
        initInspection();


    if(time > Period){
        time = 0;
        RadiusInspection += 0.5;
        Period = (2*PI*RadiusInspection)/param.inspeLinVel;
    }

time+=_dt;

_setPoint.setPosition(RadiusInspection * cos((param.inspeLinVel/RadiusInspection)*time) + X0, RadiusInspection * sin((param.inspeLinVel/RadiusInspection)*time) + Y0,param.zMax);

std::cout<<"Circ:  "<<_setPoint.getX()<<std::endl;
std::cout<<"Time: "<<time<<std::endl;

_setPoint.setType(MavState::POSITION);

}

void Lander::homing(){

    if(!_home){
        _setPoint.setX(starting_x);
        _setPoint.setY(starting_y);
        _setPoint.setType(MavState::POSITION);
        _home = true;
    }

}

void Lander::hold() {

    //This function is purely tracking, nothing more

    /*
     * Tracking is performed by POSITION control, issuing the position setpoint
     * in order to achieve the desired velocity calculated by:
     *
     * Vdes = K * ep + Vplat
     * or
     * Psp = Pplat + K * Vplat
     *
     * where:
     *
     * Vdes = desired velocity, K = proportional gain, ep = position error, Vplat = paltform velocity
     */

    Eigen::Vector2d tempVel(_platformState.getVx(),_platformState.getVy());

    _holdPIDX.setDt(_dt);
    _holdPIDY.setDt(_dt);

    double xTarget = _holdPIDX.getOutput(0,_VisionPose.getX());//0 because is already a relative error.
    double yTarget = _holdPIDY.getOutput(0,_VisionPose.getY());

    Eigen::Vector2d targetVect(_state.getX() + xTarget , _state.getY() + yTarget);

    updateIntegrals();
    //PosSP = PlatPos + K * Vplat
    //targetVect += params_automatic::KpHoldV * tempVel;
    targetVect += param.KpHoldV * tempVel;

    //Fill right fields
    _setPoint.setPosition(targetVect(0),targetVect(1),_setPoint.getZ());

    _setPoint.setType(MavState::POSITION);

}

void Lander::asce() {

    _holdPIDX.reset();
    _holdPIDY.reset();
    _setPoint.setZ(_setPoint.getZ() + 0.10);

}

void Lander::desc() {

	_setPoint.setZ(_setPoint.getZ() - 0.10);

}

void Lander::comp() {

    double dz;
    if(switchSensor)
        dz = _VisionPose.getZ() + PLATFORM_OFFSET;

    else{
        dz = - _state.getZ() + _platformState.getZ() + PLATFORM_OFFSET;
        std::cout<<"UTRASENSOR"<<std::endl;
    }


    //Calculate desired vertical velocity in order to compensate oscillations
    double desc = common::interpolate(fabs(dz), DRATE_MAX, DRATE_MIN, TMAX, TMIN);
    double z_target_v = _platformState.getVz() - desc;
    double err_v = z_target_v - _state.getVz();

    //z_target_v += params_automatic::KPCompV * (err_v);
    z_target_v += param.KPCompV * (err_v);

    //Now we need to transform this velocity in a position setpoint since in Firmware:
    // VelSP = Kp * PosError then PosSP = ( VelSP / Kp ) + RobotPos

    _setPoint.setZ((z_target_v) + _state.getZ());

}

void Lander::clampZSP() {

    double temp;
    //temp = common::clamp(_setPoint.getZ(),params_automatic::zMin,params_automatic::zMax);
    temp = common::clamp(_setPoint.getZ(),param.zMin,param.zMax);

    _setPoint.setZ(temp);

}

void Lander::land() {

    resetSetPoint();
    _setPoint.setZ(_state.getZ()-10);
    _setPoint.setType(MavState::POSITION);

}

void Lander::allign() {

    //Continuosly change yaw value
    double yaw_target = _platformState.getYawFromQuat();
    Eigen::Quaterniond q_interm = yawToQuaternion(yaw_target);

    _setPoint.setOrientation(q_interm);

}
