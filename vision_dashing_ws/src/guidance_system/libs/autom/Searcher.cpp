#include <iostream>
#include "guidance_system/autom/Searcher/Searcher.h"
#include "common/conversions.h"
#include "guidance_system/autom/param.h"

Searcher::Searcher():_horizontaErr((double)0), _tauHold((double)0), _tauLost((double)0),  _NHold(0),_dt(0), _prevTime(0), _actualTime(0),
                    _NLost(0), _initS(&_machine), _inspeS(&_machine), _hoveringS(&_machine), _holdS(&_machine), _relS(&_machine), _err(0,0,0), _err_int(0,0,0),
                    _actualState(0),_prevState(0), _verticalErr(0), _holdPIDX(0,0,0), _holdPIDY(0,0,0), _payloadReleased(false), _releasePayload(false)
{

    //set the state machine
    initStateMachine();
    ////Load parameters
  	param.loadConfigFile("Searching_params.txt");

    _holdPIDX.reset();
  	_holdPIDY.reset();

  	_holdPIDX.setP(param.xyPidKp);
  	_holdPIDX.setI(param.xyPidKi);
  	_holdPIDX.setD(param.xyPidKd);

  	_holdPIDY.setP(param.xyPidKp);
  	_holdPIDY.setI(param.xyPidKi);
  	_holdPIDY.setD(param.xyPidKd);


  	_holdPIDX.setMaxIOutput(param.xyPidIntMax);
    _holdPIDY.setMaxIOutput(param.xyPidIntMax);

    //put a saturation for the whole PID
    _holdPIDX.setOutputLimits(param.xyPidOutputMax);

  	_tauHold = param.centeredConeRadiusMax;
  	_tauLost = param.offCenterConeRadiusMax;

    //stored starting hovering position
    starting_x = _state.getX();
    starting_y = _state.getY();
    starting_z = _state.getZ();

    _hovering = false;

    //set the actual state of the machine, intially is is INIT
    _actualState = _machine.getActualNodeId();

    _err_prev = _err;
}

void Searcher::initStateMachine()
{
  print = 0;

  _inspection = false;
  First_insp = true;
  _payloadReleased = false;
  _releasePayload = false;
	resetMachine = false;
  _hovering = false;
  //reset variables
	_horizontaErr = (double)0;
	_tauHold      = (double)0;
	_tauLost      = (double)0;
	_NHold = 0;
	_NLost = 0;

	_err << 0,0,0;
	_err_int << 0,0,0;
	_dt 		 = 0;
	_prevTime    = 0;
	_actualTime  = 0;
	_verticalErr = 0;
	_actualState = 0;
	_prevState   = 0;

  //Link signals
  //assign initialized errors of the Lander class with
  //the one of the LandMachine.
  _machine._horizontaErr =  &_horizontaErr;
  _machine._verticalErr  =  &_verticalErr;
  _machine._tauHold      =  &_tauHold;
  _machine._tauLost      =  &_tauLost;
  _machine._NLost        =  &_NLost;
  _machine._NHold        =  &_NHold;
  _machine._state        =  &_state;
  _machine._setPoint     =  &_setPoint;
  _machine._VisionPose   =  &_VisionPose;
	_machine.param		     =  &param;
	//New signals
  _machine._holding      =  &_holding;
  _machine._positioned   =  &_positioned;
  _machine._lost         =  &_lost;
  _machine._inspection   =  &_inspection;
  _machine._payloadReleased   =  &_payloadReleased;


  //Link states
  //keep in mind that _nextState object is inherited from the AbstractSearchState class.
  //the following instructions set the machine state.
  _initS._nextState      = &_holdS;
  _initS._nextInspeState = &_inspeS;

  _inspeS._nextState     = &_initS;

  _hoveringS._nextState = &_initS;

  _holdS._nextState      = &_relS;
  _holdS._nextInitState  = &_initS;
	_holdS._nextHoveringState = &_hoveringS;

  _relS._nextState       = &_holdS;

  _machine.setStatePtr(&_initS); //set the state with the INIT one.

  //Print actual state
  std::cout << "Actual state: " << _machine.getActualNodeId() << std::endl;

}
void Searcher::loadMachine()
{

	param.loadConfigFile("Searching_params.txt");

	_holdPIDX.reset();
	_holdPIDY.reset();

	_holdPIDX.setP(param.xyPidKp);
	_holdPIDX.setI(param.xyPidKi);
	_holdPIDX.setD(param.xyPidKd);

	_holdPIDY.setP(param.xyPidKp);
	_holdPIDY.setI(param.xyPidKi);
	_holdPIDY.setD(param.xyPidKd);

	_holdPIDX.setMaxIOutput(param.xyPidIntMax);
  _holdPIDY.setMaxIOutput(param.xyPidIntMax);
  //put a saturation for the integral part of the PID

  _holdPIDX.setOutputLimits(param.xyPidOutputMax);

	_tauHold = param.centeredConeRadiusMax;
	_tauLost = param.offCenterConeRadiusMax;
  //stored starting hovering position
  starting_x = _state.getX();
  starting_y = _state.getY();
  starting_z = _state.getZ();

  _hovering = false;
}

void Searcher::setVisionPose(const MavState VisionPose)
{
	_VisionPose = VisionPose;
}

void Searcher::setState(MavState pose)
{
    _state = pose;
}

void Searcher::setYaw(float yaw)
{
    _yaw_des = yaw;
}

MavState Searcher::getCommand()
{
    return _setPoint;
}

bool Searcher::getPayloadCommand()
{
    return _releasePayload;
}

void Searcher::setRelPayFeedback(exec::release_payload  feedback)
{
    if(feedback.data == "R")
      _payloadReleased = true;
}

void Searcher::managetime()
{

    _actualTime = common::time::getTimeMilliSecond();
    double dt = _prevTime != 0 ? (_actualTime - _prevTime) * (double)MILLI2SECS : 0.0;
    _prevTime = _actualTime;
    _dt = dt;

}

void Searcher::run()
{
  print++;

  _prevState = _actualState;

  handleMachine();

  _actualState = _machine.getActualNodeId(); //It obtains the new state

  managetime();

  static bool initDone = false;

  switch (_actualState)
  {

    case (AbstractSearchState::states::INIT):

      if(!initDone)
      {

				if (print > 30) std::cout<<"INIT"<<std::endl;

        init();
        align(); //controlling the yaw
        initDone = _VisionPose.VisionDataUpdated;

      }
      break;

    case (AbstractSearchState::states::HOVERING):

			if (print > 30) std::cout<<"HOVERING"<<std::endl;

			hovering();
			break;

    case (AbstractSearchState::states::INSPE):

    	if (print > 30) std::cout<<"INSPE"<<std::endl;

      inspection();
      align(); //controlling the yaw

      break;

    case (AbstractSearchState::states::HOLD):

			if (print > 30) std::cout<<"HOLD"<<std::endl;

			initDone = false;

			hold(); //control the position
      align(); //controlling the yaw
      break;

    case (AbstractSearchState::states::RELEASE):

			if (print > 30) std::cout<<"RELEASING PAYLOAD"<<std::endl;

      releasePayload();
			hold(); //control the position
      align(); //controlling the yaw
      break;

    default:

      hold();
      align();
      break;
    }

    if (print > 30)
    {
      printSignals();
      print = 0;
    }
}

void Searcher::handleMachine()
{
  //isValid = 1 ==> OFFBOARD
	//isValid = 0 ==> ANOTHER FLIGHT MODE
	if( !_state.IsValid && !resetMachine )
  {
		initStateMachine();
		resetMachine = true;
	}
	else if( _state.IsValid )
  { //OFFBOARD

		if(resetMachine)
    {
			loadMachine();
			resetMachine = false;
		}

    /*
      --computes the horizontal and vertical error of the actual state

      --check if the horizontal error is under a given threshold(_tauHold and _tauLost)
      incrementing or nullifying NLost or NHold.

      --then, set the boolean value _holding, lost, centering based on the value of NLost or NHold

      --if the UAV is centered the variable NComp is incremented.
      */

    updateSignals();

	   //Being such function vrtual, will be defined by its derived classes.
    //Depending on the actual state a different function is called.
    _machine.handle();

  }
}

void Searcher::updateSignals()
{

	if( _VisionPose.VisionDataUpdated )
    _inspection = false;

  if(_VisionPose.IsValid)
    _hovering = false;

	double dx;
	double dy;
	double dz;

  dx = _VisionPose.getX();

	dy = _VisionPose.getY();

  dz = _VisionPose.getZ();

  _err[0] = dx;
  _err[1] = dy;
  _err[2] = 0; //Trick, used to calculate horizontal error

  _horizontaErr = _err.norm();
  _err[2]       = dz;
  _verticalErr  = _err[2];

  //Increment N if needed
  if (_horizontaErr < _tauHold && _VisionPose.VisionDataUpdated)
  {
      _NLost = 0;
      _NHold++;
  }
  else if (_horizontaErr > _tauLost && _VisionPose.VisionDataUpdated)
  {
      _NHold = 0;
      _NLost++;
  }

	_holding  = (_NHold > param.centeredFramesRequired && _VisionPose.VisionDataUpdated);

  _lost     = (_NLost > param.offCenterFramesToLosePad || !_VisionPose.VisionDataUpdated);

  _positioned = _horizontaErr < _tauHold * 0.5 && _VisionPose.VisionDataUpdated;

  //Inspection state
  if(_actualState == AbstractSearchState::states::INSPE)
  {
      //Reset NHold
      if(_prevState == AbstractSearchState::states::HOLD) _NHold = 0;
  }

  _err_prev = _err;

}

void Searcher::printSignals()
{

	#ifdef DEBUG
  std::cout << "**********************************" << std::endl;
  std::cout << "STATE: " << _actualState << std::endl;
  std::cout << "HERRO: " << _horizontaErr << std::endl;
  // std::cout << "HERRV: " << _verticalErr << std::endl;
  //
	// std::cout << "Vision_Z:" << _VisionPose.getZ() << std::endl;

	std::cout << "VisionDataAvailable: " << _VisionPose.VisionDataUpdated << std::endl;
	std::cout << "NHOLD: " << _NHold<< std::endl;
  std::cout << "NLOST: " << _NLost<< std::endl;
  std::cout << "**********************************" << std::endl;
#endif

}

void Searcher::init()
{

    //Set point to my position
    // resetSetPoint();

  _NHold=0;
  _NLost=0;

  _setPoint.setX(_state.getX());
  _setPoint.setY(_state.getY());
  _setPoint.setZ(_Z0);

}

void Searcher::hold() {

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


  _holdPIDX.setDt(_dt);
  _holdPIDY.setDt(_dt);


  double xTarget = _holdPIDX.getOutput(0, _VisionPose.getX());
  double yTarget = _holdPIDY.getOutput(0, _VisionPose.getY());

  // Eigen::Vector2d targetVect(_state.getX() + xTarget , _state.getY() + yTarget);
	Eigen::Vector2d targetVect(_state.getX() + xTarget/2.0 , _state.getY() + yTarget/2.0);


  updateIntegrals();
  std::cout << "xTarget is " << xTarget << " _state.getX() " << _state.getX() << std::endl;
  std::cout << "final xTarget " << targetVect(0) << " yTarget " << targetVect(1) << std::endl;
  std::cout << "_releasePayload " << _releasePayload << "_payloadReleased " << _payloadReleased << std::endl;
  if(_releasePayload && !_payloadReleased)
    _setPoint.setZ(param.altMaxAbovePad);
  else
    _setPoint.setZ(_Z0);
  //Fill right fields
  _setPoint.setPosition(targetVect(0),targetVect(1),_setPoint.getZ());
  _setPoint.setType(MavState::POSITION);
}

void Searcher::initInspectionParams(double initRadius, double initVelocity, double initZ)
{
  _Z0 = initZ;
  _inspectionRadius = initRadius;
  _inspectionLinearVel = initVelocity;
}

void Searcher::initInspection()
{
	_time = 0;
  // this is a check whenever we are coming from inspection to avoid the drone
  // loosing the target and initializing the research from the last known position
  // if it's the first time we approach the target and its location is not defined,
  // the previous error is 0, so it has no influence on the starting position
  // if it's the first time we inspect, there should be no previous error
  if(First_insp)
  {
    _X0 = _state.getX();
    _Y0 = _state.getY();
    First_insp = false;
  }
  else
  {
    _X0 = _state.getX() + _err_prev[0];
    _Y0 = _state.getY() + _err_prev[1];
  }

	_inspection = true;
  _inspectionPeriod = (2*PI*_inspectionRadius)/_inspectionLinearVel;
}

void Searcher::inspection()
{

  //This function implements a circular trajectory
  if(!_inspection)
      initInspection();

  if(_time > _inspectionPeriod)
  {
      _time = 0;
      _inspectionRadius += 1.0;
      _inspectionPeriod = (2*PI*_inspectionRadius)/_inspectionLinearVel;
  }

  _time+=_dt;

  _setPoint.setPosition(_inspectionRadius * cos((_inspectionLinearVel/_inspectionRadius)*_time) + _X0, _inspectionRadius * sin((_inspectionLinearVel/_inspectionRadius)*_time) + _Y0, _Z0);

  // std::cout<<"Circ: "<<_inspectionRadius<<std::endl;
  // std::cout<<"Time: "<<_time<<std::endl;

  _setPoint.setType(MavState::POSITION);

}

void Searcher::hovering()
{

	if(!_hovering) {
    starting_x = _state.getX();
    starting_y = _state.getY();
		starting_z = _state.getZ();
		_hovering = true;
	}

  _setPoint.setType(MavState::POSITION);
  _setPoint.setPosition(starting_x, starting_y, starting_z);

}

void Searcher::releasePayload()
{
  _releasePayload = true;
}

void Searcher::align()
{
    //
    _setPoint.setYaw(_yaw_des);
    Eigen::Quaterniond q_interm = yawToQuaternion(_yaw_des);
    // std::cout << _yaw_des*180.0/3.14 << std::endl;
    _setPoint.setOrientation(q_interm);

}

void Searcher::updateIntegrals()
{

    _err_int[0] +=  _dt * _err[0];
    _err_int[1] +=  _dt * _err[1];

    double tempx,tempy;

    tempx = common::clamp(_err_int[0],param.xyPidIntMin,param.xyPidIntMax);
    tempy = common::clamp(_err_int[1],param.xyPidIntMin,param.xyPidIntMax);

    _err_int[0] =  tempx;
    _err_int[1] =  tempy;

}
