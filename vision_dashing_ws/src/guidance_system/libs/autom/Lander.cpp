//
// Created by andreanistico on 06/02/17.
//

#include <iostream>
#include "guidance_system/autom/Lander/StatesClasses.hpp"
#include "guidance_system/autom/Lander/Lander.h"
#include "common/conversions.h"
#include "guidance_system/autom/param.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

Lander::Lander()
        : _horizontaErr((double)0)    , _tauHold((double)0), _tauLost((double)0), _tauErr((double)0), _NHold(0),
          _NLost(0),_NComp(0) , _initS(&_machine) , _inspeS(&_machine) , _hoveringS(&_machine) , _holdS(&_machine)  , _asceS(&_machine)  , _descS(&_machine),_compS(&_machine),
          _rtolS(&_machine),_landS(&_machine), _err(0,0,0), _err_int(0,0,0), _err_diff(0,0,0), _dt(0), _prevTime(0), _actualTime(0),
          _actualState(0),_prevState(0), _verticalErr(0), _holdPIDX(0,0,0),
          _holdPIDY(0,0,0)
{
  //set the state machine
  initStateMachine();
  ////Load parameters
  std::string pkg_path = ament_index_cpp::get_package_share_directory("guidance_system");
  std::string params_path = pkg_path + "/params/Landing_params.txt";
	param.loadConfigFile(params_path.c_str());

	// std::cout<<"Params: "<<param.zMax<<std::endl;

	desc_velocity = 0;

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

	//stored starting hovering position
	starting_x = _state.getX();
	starting_y = _state.getY();
	starting_z = _state.getZ();
  // _home = false;

	_hovering = false;

	_actualState = _machine.getActualNodeId();
  //set the actual state of the machine, intially is is INIT

  _err_prev = _err;

}

void Lander::setState(MavState pose) {
    _state = pose;
}

void Lander::setYaw(float yaw)
{
    _yaw_des = yaw;
}

MavState Lander::getState() {
    return _state;
}

MavState Lander::getCommand() {
    return _setPoint;
}

void Lander::loadMachine() {
  std::cout << "parsing landing params " << std::endl;
  std::string pkg_path = ament_index_cpp::get_package_share_directory("guidance_system");
  std::string params_path = pkg_path + "/params/Landing_params.txt";
	param.loadConfigFile(params_path.c_str());

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

	_tauHold = param.hold_threshold;
	_tauLost = param.lost_threshold;

	//stored starting hovering position
	starting_x = _state.getX();
	starting_y = _state.getY();
	starting_z = _state.getZ();
  // _home = false;

	_hovering = false;

}

void Lander::initStateMachine() {

  Inspection = false;
  First_insp = true;
  switchSensor = false;

	print = 0;

  _hovering = false;

	_resetMachine = false;
  //reset variables
	_horizontaErr = (double)0;
	_tauHold      = (double)0;
	_tauLost      = (double)0;
	_tauErr       = (double)0;
	_NHold = 0;
	_NLost = 0;
	_NComp = 0;

	_err << 0,0,0;
	_err_int << 0,0,0;
	_err_diff << 0,0,0;
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
  _machine._tauErr       =  &_tauErr;
  _machine._tauHold      =  &_tauHold;
  _machine._tauLost      =  &_tauLost;
  _machine._NLost        =  &_NLost;
  _machine._NHold        =  &_NHold;
  _machine._NComp        =  &_NComp;
  _machine._state        =  &_state;
  _machine._setPoint     =  &_setPoint;
  _machine._VisionPose   =  &_VisionPose;
  _machine._UltraInfo    =  &_UltraInfo;
	_machine.param		     =  &param;
	//New signals
  _machine._holding      =  &_holding;
  _machine._centered     =  &_centered;
  _machine._lost         =  &_lost;
  _machine._Inspection   =  &Inspection;
  _machine._readyToLand  =  &_readyToLand;

  //Link states
  //keep in mind that _nextState object is inherited from the AbstractLandState class.
  //the following instructions set the machine state.
  _initS._nextState    = &_holdS;
  _initS._nextInspeState  = &_inspeS;
  _initS._nextHoveringState = &_hoveringS;
  // _initS._nextHomingState = &_homingS;

  _inspeS._nextState      = &_initS;

  _hoveringS._nextState = &_initS;

  _holdS._nextAscState     = &_asceS;
  _holdS._nextDesState     = &_descS;
  _holdS._nextState        = &_rtolS;
  _holdS._nextInitState    = &_initS;
  _holdS._nextHoveringState = &_hoveringS;

  _asceS._nextState    = &_holdS;

  _descS._nextState    = &_holdS;

  _rtolS._nextComState = &_compS;
  _rtolS._nextState    = &_holdS;

  _compS._nextState    = &_asceS;
  _compS._nextLanState = &_landS;
	_compS._nextHoveringState = &_hoveringS;

  _landS._nextState    = &_asceS;
  _landS._restartState = &_initS;

  _machine.setStatePtr(&_initS); //set the state with the INIT one.

  //Print actual state
  // std::cout << "Actual state: " << _machine.getActualNodeId() << std::endl;

}

void Lander::setPlatformState(const MavState platformState) {
    _platformState = platformState;
}

void Lander::setVisionPose(const MavState VisionPose){
	_VisionPose = VisionPose;
}


void Lander::setUltrasonicInfo(const MavState UltraInfo) {
	_UltraInfo = UltraInfo;
}


void Lander::updateSignals() {

	if( _VisionPose.VisionDataUpdated )
    Inspection = false;

  if(_VisionPose.IsValid)
    _hovering = false;


	double dx;
	double dy;
	double dz;


	switchSensor = (fabs( _VisionPose.getZ() ) > 0.83); //with 0.1 always vision:)
  //above half meters use vision system for altitude value
  //otherwise, ultrasonic sensors.

  if( switchSensor )
		dz = _VisionPose.getZ();
	else
		dz = (_UltraInfo.getZ()-params_automatic::OffsetUltraSensor);


  dx = _VisionPose.getX();

	dy = _VisionPose.getY();

  _err[0] = dx;
  _err[1] = dy;
  _err[2] = 0; //Trick, used to calculate horizontal error

  _horizontaErr = _err.norm();
  _err[2]       = dz;
  _verticalErr  = _err[2];

  // Cone thresholds: linearly interpolate between min (at z=0) and max (at z=cone_z_ref).
  double cone_t = common::clamp(fabs(_verticalErr) / param.cone_z_ref, 0.0, 1.0);
  double tau_hold_cone = param.hold_threshold_min + (param.hold_threshold - param.hold_threshold_min) * cone_t;
  double tau_lost_cone = param.lost_threshold_min + (param.lost_threshold - param.lost_threshold_min) * cone_t;
  double tau_comp_cone = param.comp_threshold_min + (param.comp_threshold - param.comp_threshold_min) * cone_t;

  //Increment N if needed
  if (_horizontaErr < tau_hold_cone && _VisionPose.VisionDataUpdated) {
      _NLost = 0;
      _NHold++;
  }
  else if (_horizontaErr > tau_lost_cone && _VisionPose.VisionDataUpdated) {
      _NHold = 0;
      _NComp = 0;
      _NLost++;
  }

	_holding  = (_NHold > param.NFramesHold && _VisionPose.VisionDataUpdated );

  _lost     = (_NLost > param.NFramesLost || !_VisionPose.VisionDataUpdated );

  _centered = _horizontaErr < tau_comp_cone && _VisionPose.VisionDataUpdated;

  //R2LA state is between hold and comp state
  if(_actualState == AbstractLandState::states::R2LA || _actualState == AbstractLandState::states::COMP || _actualState == AbstractLandState::states::LAND){


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

void Lander::printSignals() {

	#ifdef DEBUG
  std::cout<<"Descending velocity: "<<desc_velocity<<std::endl;
  std::cout << "**********************************" << std::endl;
  std::cout << "STATE: " << _actualState<< std::endl;
  std::cout << "HERRO: " << _horizontaErr<< std::endl;
  std::cout << "HERRV: " << _verticalErr<< std::endl;

	std::cout<<"Vision_Z:"<<_VisionPose.getZ()<<std::endl;
	std::cout<<"Ultrasonic_Z:"<<(_UltraInfo.getZ() - params_automatic::OffsetUltraSensor)<<std::endl;
	std::cout<<"UltrasonicValid? "<<_UltraInfo.IsValid<<std::endl;

	std::cout << "switchSensor: " << switchSensor << std::endl;
	std::cout << "VisionDataAvailable: " << _VisionPose.VisionDataUpdated << std::endl;
	std::cout << "UltrasonicdataAvailable: " << _UltraInfo.UltrasonicDataUpdated << std::endl;
  std::cout << "Roll: " << _VisionPose.getRoll() << std::endl;
  std::cout << "Pitch: " << _VisionPose.getPitch() << std::endl;
  std::cout << "Yaw: " << _VisionPose.getYaw() << std::endl;
	std::cout << "NHOLD: " << _NHold<< std::endl;
  std::cout << "NLOST: " << _NLost<< std::endl;
  std::cout << "NCOMP: " << _NComp<< std::endl;
  //std::cout << "INTEX: " << _err_int[0] << std::endl;
  //std::cout << "INTEY: " << _err_int[1] << std::endl;
  std::cout << "**********************************" << std::endl;
#endif

}

void Lander::handleMachine() {


	//isValid = 1 ==> OFFBOARD
	//isValid = 0 ==> ANOTHER FLIGHT MODE
  // std::cout << "start _state.IsValid " << _state.IsValid << " _resetMachine " << _resetMachine << std::endl;
	if( !_state.IsValid && !_resetMachine ) {
    // std::cout << "_state.IsValid " << _state.IsValid << " _resetMachine " << _resetMachine << std::endl;

		initStateMachine();
		_resetMachine = true;
	}
	else if( _state.IsValid ) { //OFFBOARD
    // std::cout << "offboard _state.IsValid " << _state.IsValid << " _resetMachine " << _resetMachine << std::endl;
		if(_resetMachine) {
			loadMachine();
			_resetMachine = false;
		}


    updateSignals();

	/*
    --computes the horizontal and vertical error of the actual state

    --check if the horizontal error is under a given threshold(_tauHold and _tauLost)
    incrementing or nullifying NLost or NHold.

    --then, set the boolean value _holding, lost, centering based on the value of NLost or NHold

    --if the UAV is centered the variable NComp is incremented.
    */

    _machine.handle();
	 //Being such function vrtual, will be defined by its derived classes.
    //Depending on the actual state a different function is called.

  }
  // std::cout<<"reset"<<_resetMachine<<std::endl;


// printSignals();

}

int Lander::getActualMachineState() {
    return _machine.getActualNodeId();
}

void Lander::run() {

    //set the previous state with the actual one, initially is INIT.
   print++;


  _prevState = _actualState;
  handleMachine();

  _actualState = _machine.getActualNodeId(); //It obtains the new state

  if(startFromBeginning) // in case the landing should get reinitialized after a Safety Task
  {
    _actualState = AbstractLandState::states::INIT;
  }

  managetime();
  static bool initDone = false;
  switch (_actualState){

    case (AbstractLandState::states::INIT):
      std::cout << "in init "<< " initDone " << initDone << " startFromBeginning " << startFromBeginning << std::endl;
      if(!initDone)
      {

				if (print > 30) std::cout<<"INIT"<<std::endl;

				init();
        align(); //controlling the yaw
				/*
				It is divided in two parts:
					--set the setpoint to the actual drone position.
					--set the z axis to the max one 2.5 meters.
				*/

                //initDone = true;
        // positioning the drone in front of the platform
        _readyToLand = setDroneInFront();
				initDone = _VisionPose.VisionDataUpdated && _readyToLand;
        // once the drone is ready to land, the altitude setpoint changes
        if(initDone)
        {
           _setPoint.setZ(param.zMax);
        }

      }

      if(startFromBeginning)
      {
        if (print > 30) std::cout<<"INIT2"<<std::endl;

        init();
        align(); //controlling the yaw

        // positioning the drone in front of the platform
        _readyToLand = setDroneInFront();
        startFromBeginning = !_readyToLand;
        // once the drone is ready to land, the altitude setpoint changes
        if(!startFromBeginning)
        {
           _setPoint.setZ(param.zMax);
        }
      }
      break;
    case (AbstractLandState::states::INSPE):

      if (print > 30) std::cout<<"INSPE"<<std::endl;

      inspection();
      // align(); //controlling the yaw

      break;

		case ( AbstractLandState::states::HOVERING):

			if (print > 30) std::cout<<"HOVERING"<<std::endl;

			hovering();
			break;

    case (AbstractLandState::states::HOLD):

			if (print > 30) std::cout<<"HOLD"<<std::endl;

			initDone = false;
      clampZSP(); //It keeps the Z of the UAV in a interval specified.

			hold(); //control the x and y position
      // align(); //controlling the yaw
      break;

    case (AbstractLandState::states::DESC):

			if (print > 30) std::cout<<"DESC"<<std::endl;

      desc();
      // align(); //controlling the yaw
      //decrease the z position to 0.1 meters.
      clampZSP();
      break;

    case (AbstractLandState::states::ASCE):

			if (print > 30) std::cout<<"ASCE"<<std::endl;

      asce();
      // align(); //controlling the yaw
      //increase the z position to 0.1 meters.
      clampZSP();
      break;

    case (AbstractLandState::states::R2LA):

			if (print > 30) std::cout<<"R2LA"<<std::endl;

      // align(); //controlling the yaw
      clampZSP();
      hold();
      break;

    case (AbstractLandState::states::COMP):

			if (print > 30) std::cout<<"COMP"<<std::endl;

      // align(); //controlling the yaw
      hold();
      comp();
      //compensation of the altitude
      break;

    case (AbstractLandState::states::LAND):

			if (print > 30) std::cout<<"LAND"<<std::endl;

      // align(); //controlling the yaw
      // initDone = false;
      land();
      //landing phase.
      break;

    default:

      hold();
      break;

  }

	if (print > 30){
		// printSignals();
		print = 0;
    std::cout << "actual state " << _actualState << std::endl;
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
    // resetSetPoint();

    _NHold=0;
    _NComp=0;
    _NLost=0;

    //Go to max tracking height
    // _setPoint.setZ(param.zMax);

}

bool Lander::setDroneInFront()
{
  // this function computes the desired position the drone has to reach before
  // attempting the landing. That position is computed to be in front of the platform.
  double x_des_drone = 0;
  double y_des_drone = 0;
  double xTarget = 0;
  double yTarget = 0;
  // getting the yaw of the platform to know how it's oriented.
  double yaw_plat = _platformState.getYaw();
  // std::cout << "YAW: " << yaw_plat << std::endl;

  // getting also the distance from the center the drone has to keep once positioned,
  // passed from file; it also must be not less than 0.
  double gain = fabs(param.distanceFromCenter);
  // then, depending on how the platform is oriented, the desired position changes;
  // more precisely, the values of sin and cos (sin acting on x cause North in NED == Y in Local Frame)
  // determinate the position around the platform, with a certain distance from the center;
  // adding then the error wrt the platform , so that the drone is in the correct position.
  if(yaw_plat >= -PI && yaw_plat < PI)
  {
    _holdPIDX.setDt(_dt);
    _holdPIDY.setDt(_dt);

    x_des_drone = _VisionPose.getX() + gain*sinf(yaw_plat);
    y_des_drone = _VisionPose.getY() + gain*cosf(yaw_plat);

    xTarget = _holdPIDX.getOutput(0, x_des_drone) + _state.getX();
    yTarget = _holdPIDY.getOutput(0, y_des_drone) + _state.getY();
// xTarget = x_des_drone + _state.getX();
// yTarget = y_des_drone + _state.getY();


    updateIntegrals();
  }
  else
  {
    std::cout << "YAW NOT VALID, CANNOT GENERATE DESIRED POSITION";
    return false;
  }
  std::cout << "x_des_drone: " << x_des_drone << " gain " << gain << " yaw_plat " << yaw_plat << " sinf(yaw_plat) " << sinf(yaw_plat) << std::endl;
  std::cout << "y_des_drone: " << y_des_drone << " cosf(yaw_plat) " << cosf(yaw_plat) << std::endl;
  std::cout << "_state.getX() " << _state.getX() << " _state.getY() " << _state.getY() << std::endl;
  std::cout << "xTarget: " << xTarget << std::endl;
  std::cout << "yTarget: " << yTarget << std::endl;

  std::cout << "_VisionPose.getX(): " << _VisionPose.getX() << std::endl;
  std::cout << "_VisionPose.getY(): " << _VisionPose.getY() << std::endl;
  std::cout << "**********************************" << std::endl;
  // setting the setpoint finally
  _setPoint.setPosition(xTarget, yTarget, param.prelandAltitude);
  _setPoint.setType(MavState::type::POSITION);

  // returning the error condition
  if(fabs(xTarget - _state.getX()) < 0.15 && fabs(yTarget - _state.getY()) < 0.15)
  {
    // setting the slope vector for the next step, simplifying the computations
    // _platformState.getX() - (_platformState.getX() + gain*sinf(yaw_plat))
    _line_direction[0] = -gain*sinf(yaw_plat);
    _line_direction[1] = -gain*cosf(yaw_plat);
    _line_direction[2] = _state.getZ() - (_platformState.getZ() + param.z_margin);
    // and returning
    return true;
  }
  return false;
}

void Lander::initInspection()
{

	time = 0;
  // if it's the first time we inspect, there should be no previous error
  if(First_insp)
  {
    X0 = _state.getX();
    Y0 = _state.getY();
    First_insp = false;
  }
  else
  {
    X0 = _state.getX() + _err_prev[0];
    Y0 = _state.getY() + _err_prev[1];
  }
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

  _setPoint.setPosition(RadiusInspection * cos((param.inspeLinVel/RadiusInspection)*time) + X0, RadiusInspection * sin((param.inspeLinVel/RadiusInspection)*time) + Y0, param.prelandAltitude);

  // std::cout<<"Circ:  "<<_setPoint.getX()<<std::endl;
  // std::cout<<"Time: "<<time<<std::endl;

  _setPoint.setType(MavState::POSITION);

}
//
// void Lander::homing(){
//
//     if(!_home){
//         _setPoint.setX(starting_x);
//         _setPoint.setY(starting_y);
//         _setPoint.setType(MavState::POSITION);
//         _home = true;
//     }
//
// }

void Lander::hovering() {

  if(!_hovering) {
    starting_x = _state.getX();
    starting_y = _state.getY();
		starting_z = param.prelandAltitude;
		_hovering = true;
	}

  _setPoint.setType(MavState::POSITION);
  _setPoint.setPosition(starting_x, starting_y, starting_z);

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

  // Eigen::Vector2d tempVel(_platformState.getVx(),_platformState.getVy());


  _holdPIDX.setDt(_dt);
  _holdPIDY.setDt(_dt);

  // Apply slope Z adjustment only when HOLD is in a neutral state — no ascend
  // or descend transition is pending. Mirrors the condition checks in HoldState::handle().
  {
    bool desc_v        = _holding && (_setPoint.getZ() > param.zMin + 0.1) && _machine.getLandingGoSignal();
    bool asce_v        = _lost    && (_verticalErr < param.zMax - 0.1);
    bool asce_min      = (_setPoint.getZ() < param.zMin);
    bool asce_margin   = _holding && (_verticalErr < param.z_margin) && (_verticalErr < param.zMax - 0.1) && !desc_v;

    // if (_prevState == AbstractLandState::states::INIT)
    if(!desc_v && !asce_v && !asce_min && !asce_margin && !_holding)
    {
      double z_base = _platformState.getZ() + param.z_margin;
      double dot    = _VisionPose.getX() * _line_direction[0] + _VisionPose.getY() * _line_direction[1];
      double norm_sq = _line_direction[0] * _line_direction[0] + _line_direction[1] * _line_direction[1];
      double z_adj  = z_base + _line_direction[2] * dot / norm_sq;
      
      _setPoint.setZ(z_adj);
        std::cout << "hold setting z pos !desc_v " << !desc_v << " !asce_v " << !asce_v
                  << " !asce_min " << !asce_min << " !asce_margin " << !asce_margin
                  << " z_adj " << z_adj << std::endl;
      clampZSP();
    }
  }


  // When centered and close to the platform the tag is viewed nearly head-on,
  // making single-tag pose unobservable. Freeze X/Y setpoint — the drone is
  // already aligned and vision noise at this singularity would push it away.
  double xTarget = 0.0, yTarget = 0.0;
  if (!(_centered && _verticalErr < param.zMin)) {
    xTarget = _holdPIDX.getOutput(0, _VisionPose.getX());
    yTarget = _holdPIDY.getOutput(0, _VisionPose.getY());
  }

	Eigen::Vector2d targetVect(_state.getX() + xTarget , _state.getY() + yTarget);

  std::cout << "t state=" << _actualState
          << " sp=(" << _setPoint.getX() << "," << _setPoint.getY() << "," << _setPoint.getZ() << ")"
          << " vis=(" << _VisionPose.getX() << "," << _VisionPose.getY() << ")"
          << " visUpd=" << _VisionPose.VisionDataUpdated
          << " holding=" << _holding << " lost=" << _lost << " centered=" << _centered
          << " xT=" << xTarget << " yT=" << yTarget << std::endl;

  updateIntegrals();
  //PosSP = PlatPos + K * Vplat
  //targetVect += params_automatic::KpHoldV * tempVel;
  // targetVect += param.KpHoldV * tempVel;


  //Fill right fields
  _setPoint.setPosition(targetVect(0),targetVect(1),_setPoint.getZ());
  _setPoint.setType(MavState::POSITION);

}

void Lander::asce() {

  _holdPIDX.reset();
  _holdPIDY.reset();
  _setPoint.setZ(_setPoint.getZ() + 0.1);

}

void Lander::desc() {

  _setPoint.setZ(_setPoint.getZ() - 0.1);
}

void Lander::comp() {

	double dz;

	if(switchSensor)
  	dz = _VisionPose.getZ() + PLATFORM_OFFSET;
	else{

		dz = (_UltraInfo.getZ() - params_automatic::OffsetUltraSensor);
		//std::cout<<"ULTRASENSOR"<<std::endl;
	}

  //Calculate desired vertical velocity in order to compensate oscillations
  desc_velocity = common::interpolate(fabs(dz), DRATE_MAX, DRATE_MIN, TMAX, TMIN);

	//FIRST
    //double z_target_v = _platformState.getVz() - desc;
    //double err_v = z_target_v - _state.getVz();
    //z_target_v += params_automatic::KPCompV * (err_v);


	// SECOND
	//double Z_target_v = ( _UltraInfo.getVz() + _state.getVz() ) - desc;
	//double Err_v = Z_target_v - _state.getVz();
	//Z_target_v += params_automatic::KPCompV * (Err_v);

	// std::cout<<"Target: "<< _UltraInfo.getVz() << std::endl;
	//std::cout<<"MyTarget: "<< Z_target_v << std::endl;

	// double Err_v = desc_velocity - _UltraInfo.getVz();
	// double Z_target_v = param.KPCompV * (Err_v);
  desc_velocity = 0.1;
  // double Z_target_v = _UltraInfo.getVz() - desc_velocity;
  // double Err_v = Z_target_v - _state.getVz();
  // Z_target_v += param.KPCompV * (Err_v);

  double v_des_q = desc_velocity + (_state.getVz() - _UltraInfo.getVz());
  double e_v =  v_des_q +_state.getVz();
  double v_star_q =  _state.getVz() + param.KPCompV * e_v;





    // std::cout<<"Old: "<< Z_target_v << std::endl;
    // std::cout<<"NEWt: "<<  0.5 * v_star_q<< std::endl;

  //Now we need to transform this velocity in a position setpoint since in Firmware:
  // VelSP = Kp * PosError then PosSP = ( VelSP / Kp ) + RobotPos

  _setPoint.setZ( 0.5 * (v_star_q) + _state.getZ());


  if(_setPoint.getZ() > param.zMax)
  {
    _setPoint.setZ(param.zMax);

  }

  // std::cout << _setPoint.getZ() << std::endl;
}

void Lander::clampZSP() {

    double temp;
    temp = common::clamp(_setPoint.getZ(), param.zMin, _platformState.getZ() + param.zMax);

    _setPoint.setZ(temp);
}

void Lander::land() {

    resetSetPoint();
    _setPoint.setZ(_state.getZ()-20);

}

void Lander::align() {

    //we keep the yaw value the platform had in the init state
    // Eigen::Quaterniond q_interm = yawToQuaternion(_VisionPose.getYawFromQuat());
    // Eigen::Quaterniond q_interm = yawToQuaternion(_VisionPose.getYaw());
    _setPoint.setYaw(_yaw_des);
    Eigen::Quaterniond q_interm = yawToQuaternion(_yaw_des);
    // std::cout << _yaw_des*180.0/3.14 << std::endl;
    _setPoint.setOrientation(q_interm);

}
