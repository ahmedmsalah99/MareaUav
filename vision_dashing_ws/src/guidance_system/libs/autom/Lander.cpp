//
// Created by andreanistico on 06/02/17.
//

#include <iostream>
#include <algorithm>
#include "guidance_system/autom/Lander/StatesClasses.hpp"
#include "guidance_system/autom/Lander/Lander.h"
#include "common/conversions.h"
#include "guidance_system/autom/param.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

Lander::Lander()
        : _horizontaErr((double)0)    , _tauHold((double)0), _tauLost((double)0), _tauErr((double)0), _NHold(0),
          _NLost(0),_NComp(0) , _initS(&_machine) , _inspeS(&_machine) , _hoveringS(&_machine) , _holdS(&_machine)  , _asceS(&_machine)  , _descS(&_machine),_compS(&_machine),
          _rtolS(&_machine),_landS(&_machine),_landingDoneS(&_machine), _err(0,0,0), _err_int(0,0,0), _err_diff(0,0,0), _dt(0), _prevTime(0), _actualTime(0),
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
	_predictionHorizon = 0;

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
	std::cout << "REACTION_LAG_S " << param.reactionLag << std::endl;

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
	_spSeeded    = false;   // re-seed the XY platform-hold filter on (re)init
	_compTracking = true;   // COMP starts in tracking mode
	_landingGo    = false;  // GO/NO-GO window
	_goLatched    = false;
	_goLatchTimer = 0.0;
	_goLogPrev    = false;
	_goVoteHistory.clear();
	_goDiag       = LandGoDiagnostics();
	_touchPrevVz         = 0.0;
	_touchAccelBuf.clear();
	_touchContactTimer   = 0.0;
	_touchTimeSinceSpike = 1e9;
	_touchConfirmTimer   = 0.0;
	_touchedDown         = false;

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
  _machine._seaState     =  &_seaState;
  _machine._landingGo    =  &_landingGo;
  _machine._touchedDown  =  &_touchedDown;
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

  _compS._nextState    = &_holdS;   // gap drifted back above standoff -> HOLD
  _compS._nextLanState = &_landS;
	_compS._nextHoveringState = &_hoveringS;

  _landS._nextState    = &_compS;   // NO-GO or vision lost during landing -> back to COMP
  _landS._restartState = &_initS;
  _landS._nextDoneState = &_landingDoneS;   // touchdown detected -> terminal

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

void Lander::setSeaState(const SeaState seaState) {
	_seaState = seaState;
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

  // Prediction horizon: time to reach platform at current descent velocity, capped.
  double predHorizonMax = param.zMax / param.descVelocity;
  _predictionHorizon = std::min(fabs(_verticalErr) / param.descVelocity, predHorizonMax);

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
  if(!_centered){
    std::cout << "_vertical_Err" << _verticalErr << "_horizontaErr " << _horizontaErr << " tau_comp_cone " << tau_comp_cone << " _VisionPose.VisionDataUpdated " << _VisionPose.VisionDataUpdated << std::endl;
  }
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

  // ── Landing GO/NO-GO window ────────────────────────────────────────────────
  // GO when the SPA prediction and heave estimate are active, the predicted
  // roll/pitch and the roll/pitch rates are within the LANDING limits, and the
  // pad is descending from a peak. The raw (instantaneous) decision is noisy —
  // it's smoothed by a majority vote over the last landVoteWindow seconds before
  // it feeds the latch, so a single bad sample can't flip the landing decision.
  // The latch then holds each (voted) decision for at least landLatchTime before
  // accepting the opposite.
  {
    bool anglesOk = std::fabs(_seaState.spaRollPred)  < param.landRollLimit
                 && std::fabs(_seaState.spaPitchPred) < param.landPitchLimit;
    bool ratesOk  = std::fabs(_seaState.rollRate)  < param.landRollRateLimit
                 && std::fabs(_seaState.pitchRate) < param.landPitchRateLimit;
    bool descFromPeak = (_seaState.heaveVelPred < 0.7);
    bool rawGo = _seaState.spaValid && _seaState.heaveValid
              && anglesOk && ratesOk && descFromPeak;

    // Majority vote over the last landVoteWindow seconds.
    _goVoteHistory.emplace_back(_seaState.dt, rawGo);
    double windowSpan = 0.0;
    for (auto &s : _goVoteHistory) windowSpan += s.first;
    while (_goVoteHistory.size() > 1 && windowSpan > param.landVoteWindow) {
      windowSpan -= _goVoteHistory.front().first;
      _goVoteHistory.pop_front();
    }
    int voteCount = 0;
    for (auto &s : _goVoteHistory) if (s.second) voteCount++;
    double voteFraction = _goVoteHistory.empty() ? 0.0
                         : static_cast<double>(voteCount) / _goVoteHistory.size();
    bool votedGo = voteFraction >= param.landVoteThreshold;

    _goLatchTimer += _seaState.dt;
    if (votedGo != _goLatched && _goLatchTimer >= param.landLatchTime) {
      _goLatched   = votedGo;
      _goLatchTimer = 0.0;
    }
    _landingGo = _goLatched;

    _goDiag.spaValid       = _seaState.spaValid;
    _goDiag.heaveValid     = _seaState.heaveValid;
    _goDiag.spaRollPred    = _seaState.spaRollPred;
    _goDiag.spaPitchPred   = _seaState.spaPitchPred;
    _goDiag.rollRate       = _seaState.rollRate;
    _goDiag.pitchRate      = _seaState.pitchRate;
    _goDiag.heavePos       = _seaState.heavePos;
    _goDiag.heaveVel       = _seaState.heaveVel;
    _goDiag.heaveVelPred   = _seaState.heaveVelPred;
    _goDiag.rollLimit      = param.landRollLimit;
    _goDiag.pitchLimit     = param.landPitchLimit;
    _goDiag.rollRateLimit  = param.landRollRateLimit;
    _goDiag.pitchRateLimit = param.landPitchRateLimit;
    _goDiag.anglesOk       = anglesOk;
    _goDiag.ratesOk        = ratesOk;
    _goDiag.descFromPeak   = descFromPeak;
    _goDiag.rawGo          = rawGo;
    _goDiag.voteFraction   = voteFraction;
    _goDiag.voteWindowSize = static_cast<int>(_goVoteHistory.size());
    _goDiag.votedGo        = votedGo;
    _goDiag.latchTimer     = _goLatchTimer;
    _goDiag.landingGo      = _landingGo;

    if (_landingGo != _goLogPrev) {
      std::cout << "[GO] " << (_landingGo ? "GO" : "NO-GO")
                << " vote=" << voteFraction << "/" << param.landVoteThreshold
                << " spaPred(r=" << _seaState.spaRollPred << ",p=" << _seaState.spaPitchPred << ")"
                << " rates(r=" << _seaState.rollRate << ",p=" << _seaState.pitchRate << ")"
                << " heave(pos=" << _seaState.heavePos << ",vel=" << _seaState.heaveVel << ")"
                << std::endl;
      _goLogPrev = _landingGo;
    }
  }

  // ── Touchdown detection (LAND -> LANDING_DONE) ─────────────────────────────
  // Adapted from PX4's own multicopter land detector (staged hysteresis: contact
  // then confirm), corrected for a moving/heaving pad by checking velocity
  // RELATIVE to the pad rather than in the world frame — a drone resting on the
  // pad still has non-zero world-frame velocity because the pad is moving.
  // Only evaluated while actually committed to landing (LAND); _touchedDown
  // latches permanently once confirmed and is only cleared by initStateMachine().
  if (_actualState == AbstractLandState::states::LAND && !_touchedDown) {
    double vRelZ = _state.getVz() - _seaState.heaveVel;
    bool velSettled = std::fabs(vRelZ)          < param.touchVelRelMax
                    && std::fabs(_state.getVx()) < param.touchVelRelMax
                    && std::fabs(_state.getVy()) < param.touchVelRelMax;

    // Impact deceleration: finite-difference vz, smoothed (a single noisy IMU
    // sample shouldn't count as an impact spike).
    double az = (_seaState.dt > 1e-3) ? (_state.getVz() - _touchPrevVz) / _seaState.dt : 0.0;
    _touchPrevVz = _state.getVz();
    _touchAccelBuf.push_back(az);
    if (_touchAccelBuf.size() > 5) _touchAccelBuf.pop_front();
    double azSum = 0.0;
    for (double v : _touchAccelBuf) azSum += v;
    double azSmoothed = azSum / static_cast<double>(_touchAccelBuf.size());

    bool accelSpike = std::fabs(azSmoothed) > param.touchAccelSpike;
    _touchTimeSinceSpike = accelSpike ? 0.0 : (_touchTimeSinceSpike + _seaState.dt);

    // Stage 1: relative velocity settled, sustained.
    _touchContactTimer = velSettled ? (_touchContactTimer + _seaState.dt) : 0.0;
    bool groundContact = _touchContactTimer >= param.touchContactTime;

    // Stage 2: contact + a recent impact spike, sustained.
    bool spikeRecent = _touchTimeSinceSpike <= param.touchAccelSpikeWindow;
    _touchConfirmTimer = (groundContact && spikeRecent) ? (_touchConfirmTimer + _seaState.dt) : 0.0;
    bool confirmed = _touchConfirmTimer >= param.touchConfirmTime;

    if (confirmed) {
      _touchedDown = true;
      std::cout << "[TOUCHDOWN] confirmed: vRelZ=" << vRelZ << " azSmoothed=" << azSmoothed
                << " groundContact=" << groundContact << std::endl;
    }

    _goDiag.touchVelRelZ       = vRelZ;
    _goDiag.touchAccelZ        = azSmoothed;
    _goDiag.touchGroundContact = groundContact;
    _goDiag.touchConfirmed     = confirmed;
    _goDiag.touchedDown        = _touchedDown;
  }

  _err_prev = _err;

}

void Lander::printSignals() {

	#ifdef DEBUG
  std::cout<<"Descending velocity: "<<desc_velocity<<std::endl;
  std::cout<<"Prediction horizon T: "<<_predictionHorizon<<" s"<<std::endl;
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
  if(!_VisionPose.VisionDataUpdated){
    _holdPIDX.reset();
    _holdPIDY.reset();
  }


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
      hold();   // keep XY centred on the pad during the committed descent
      land();
      //landing phase.
      break;

    case (AbstractLandState::states::LANDING_DONE):

			if (print > 30) std::cout<<"LANDING_DONE"<<std::endl;

      // Terminal: touchdown confirmed. Freeze the setpoint at the current
      // (already-landed) position every tick — no more tracking, no descent,
      // no way back to flying. A fresh landing task (initStateMachine()) is the
      // only way out.
      _setPoint = _state;
      _setPoint.setType(MavState::POSITION);
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

  // if(_VisionPose.getOrientation().w() == 0){
  //   _setPoint.setPosition(_state.getX(),_state.getY(),_setPoint.getZ());
  //   // Vision is invalid (shaking / too fast). Start or extend the freeze timer.
  //   if(!_inInvalidVisionHold) {
  //     _inInvalidVisionHold = true;
  //     _invalidVisionTimer  = 0.0;
  //     std::cout << "[hold] vision invalid — starting " << kInvalidVisionHoldSecs << "s freeze" << std::endl;
  //   }
  //   _invalidVisionTimer += _dt;
  //   _setPoint.setPosition(_state.getX(), _state.getY(), _setPoint.getZ());
  //   _setPoint.setType(MavState::POSITION);
  //   return;
  // }


  // if(_inInvalidVisionHold) {
  //   // Vision recovered. Check if it stayed invalid for the full window.
  //   if(_invalidVisionTimer < kInvalidVisionHoldSecs) {
  //     _invalidVisionTimer += _dt;
  //     // _setPoint.setPosition(_state.getX(), _state.getY(), _setPoint.getZ());
  //     // _setPoint.setType(MavState::POSITION);
  //     std::cout << "time is " << _invalidVisionTimer << std::endl;
  //     return;
  //   }
  //   // If the full 3 seconds elapsed, the state machine will have already handled
  //   // the loss (VisionDataUpdated timeout → INIT); we just reset the flag here.
  //   _inInvalidVisionHold = false;
  //   _invalidVisionTimer  = 0.0;
  // }


  _holdPIDX.setDt(_dt);
  _holdPIDY.setDt(_dt);


  // Apply slope Z adjustment only when HOLD is in a neutral state — no ascend
  // or descend transition is pending. Mirrors the condition checks in HoldState::handle().
  {
    bool desc_v        = _centered && _holding && (_setPoint.getZ() > param.zMin + 0.1) && _machine.safeInitialLanding();
    bool asce_v        = _lost    && (_verticalErr < param.zMax - 0.1);
    bool asce_min      = (_setPoint.getZ() < param.zMin);
    bool asce_margin   = _holding && _centered && (_verticalErr < param.z_margin) && (_verticalErr < param.zMax - 0.1) && !desc_v;

    // if (_prevState == AbstractLandState::states::INIT)
    if(!desc_v && !asce_v && !asce_min && !asce_margin && !_centered)
    {
      double z_base = _platformState.getZ() + param.z_margin;
      double dot    = _VisionPose.getX() * _line_direction[0] + _VisionPose.getY() * _line_direction[1];
      double norm_sq = _line_direction[0] * _line_direction[0] + _line_direction[1] * _line_direction[1];
      double z_adj  = z_base + _line_direction[2] * dot / norm_sq;
      
      _setPoint.setZ(z_adj);
        std::cout << "hold setting z pos !desc_v " << !desc_v << " !asce_v " << !asce_v
                  << " !asce_min " << !asce_min << " !asce_margin " << !asce_margin
                  << " z_adj " << z_adj << " centered " << _centered << std::endl;
      clampZSP();
    }
    _setPoint.setType(MavState::POSITION);
  }


  // ── XY: absolute platform-position hold ──────────────────────────────────
  // Command the platform's estimated WORLD position, not "drone position + gain*
  // vision". vision = platform - drone, so platform_world = state + vision. This
  // is an absolute target (no walk), low-pass filtered and rate-limited so the
  // vision delay/jitter can't drive the sinusoidal limit cycle that unity-gain
  // "state + vision" produced.
  const bool visionOk = _VisionPose.IsValid && _VisionPose.VisionDataUpdated;

  double sp_x, sp_y;
  if (!visionOk) {
    // No trustworthy vision: freeze XY at the current position and re-seed the
    // filter when vision returns.
    sp_x = _state.getX();
    sp_y = _state.getY();
    _spSeeded = false;
  } else {
    double plat_x = _state.getX() + _VisionPose.getX();
    double plat_y = _state.getY() + _VisionPose.getY();

    if (!_spSeeded) {
      _platFiltX = plat_x;  _platFiltY = plat_y;
      _prevSpX   = plat_x;  _prevSpY   = plat_y;
      _spSeeded  = true;
    }

    // EMA low-pass — the platform's horizontal position is quasi-stationary.
    _platFiltX = param.spFilterAlpha * _platFiltX + (1.0 - param.spFilterAlpha) * plat_x;
    _platFiltY = param.spFilterAlpha * _platFiltY + (1.0 - param.spFilterAlpha) * plat_y;

    // Rate limit so a vision glitch can't fling the setpoint.
    double maxStep = param.spMaxVelXY * _dt;
    sp_x = common::clamp(_platFiltX, _prevSpX - maxStep, _prevSpX + maxStep);
    sp_y = common::clamp(_platFiltY, _prevSpY - maxStep, _prevSpY + maxStep);
  }

  _prevSpX = sp_x;
  _prevSpY = sp_y;
  Eigen::Vector2d targetVect(sp_x, sp_y);

  //Fill right fields
  _setPoint.setPosition(targetVect(0),targetVect(1),_setPoint.getZ());
  _setPoint.setType(MavState::POSITION);

}

void Lander::asce() {

  _holdPIDX.reset();
  _holdPIDY.reset();
  double new_z = std::min(_setPoint.getZ() + param.asceVelocity,_state.getZ() + param.asceVelocity);
  _setPoint.setZ(new_z);
  _setPoint.setType(MavState::POSITION);

}

void Lander::desc() {
  std::cout << "In desc " << std::endl;
  double v_hor_calm = std::fabs(_state.getVx()) < 0.1 && std::fabs(_state.getVy()) < 0.1;
  if (!v_hor_calm)
    return;
  double new_z = std::max(_setPoint.getZ() - param.descVelocity,_state.getZ() - param.descVelocity);
  _setPoint.setZ(new_z);
  _setPoint.setType(MavState::POSITION);
}

void Lander::comp() {

  // COMP standoff hold: keep the drone at compHoldDist above the pad while
  // centred (XY is held by hold(), which runs before comp() in the COMP tick).
  //
  // Hysteresis safety: if the gap drops below compMinDist the pad is coming up
  // too close — stop tracking and back off (ascend) until the gap climbs back
  // above compResumeDist, then resume tracking.
  //
  // B2 is reactive: it uses the CURRENT measured gap. C1 adds the predicted
  // pad motion (lead by REACTION_LAG_S) to drive the relative velocity to zero.

  double gap = -_verticalErr;   // measured drone->pad vertical distance

  if (gap < param.compMinDist)    _compTracking = false;   // too close — back off
  if (gap > param.compResumeDist) _compTracking = true;    // safe again — track

  double new_z;
  if (_compTracking) {
    double vff  = _seaState.heaveValid ? _seaState.heaveVelPred : 0.0;
    double lead = (param.compHoldDist - gap) + vff ;
    new_z = _state.getZ() + lead;
  } else {
    // Backing off: ascend (anchored, bounded) until the gap recovers.
    new_z = std::min(_setPoint.getZ() + param.asceVelocity, _state.getZ() + param.asceVelocity);
  }

  std::cout << "comp gap " << gap << " tracking " << _compTracking
            << " heaveVelPred " << _seaState.heaveVelPred << " new_z " << new_z << std::endl;

  _setPoint.setZ(new_z);
  clampZSP();
  _setPoint.setType(MavState::POSITION);
}

void Lander::clampZSP() {

    double temp;
    temp = common::clamp(_setPoint.getZ(), param.zMin, _platformState.getZ() + param.zMax);
    // std::cout << "_platformState.getZ() " << _platformState.getZ() << " _platformState.getZ() + param.zMax " << _platformState.getZ() + param.zMax << std::endl;
    _setPoint.setZ(temp);
    _setPoint.setType(MavState::POSITION);
}

void Lander::land() {

    // Committed aggressive descent toward the pad (GO window). XY is held centred
    // by hold() (runs before land() in the LAND tick); here we only drive Z down,
    // anchored to the drone so the setpoint can't run away. No zMin clamp — we are
    // committing to touch down.

    // double new_z = std::min(_setPoint.getZ() - param.landDescVel, _state.getZ() - param.landDescVel);
    // _setPoint.setZ(new_z);
    // _setPoint.setType(MavState::POSITION);

    _setPoint.setVz(-param.landDescVel);
    _setPoint.setType(MavState::VELOCITY);

}

void Lander::align() {

    //we keep the yaw value the platform had in the init state
    // Eigen::Quaterniond q_interm = yawToQuaternion(_VisionPose.getYawFromQuat());
    // Eigen::Quaterniond q_interm = yawToQuaternion(_VisionPose.getYaw());
    _setPoint.setYaw(_yaw_des);
    Eigen::Quaterniond q_interm = yawToQuaternion(_yaw_des);
    // std::cout << _yaw_des*180.0/3.14 << std::endl;
    _setPoint.setOrientation(q_interm);
    _setPoint.setType(MavState::POSITION);

}
