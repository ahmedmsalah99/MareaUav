//
// Created by andreanistico on 06/02/17.
//

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <cstdlib>
#include <cmath>
#include "guidance_system/autom/Lander/StatesClasses.hpp"
#include "guidance_system/autom/Lander/Lander.h"
#include "common/conversions.h"
#include "guidance_system/autom/param.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

Lander::Lander()
        : _horizontaErr((double)0)    , _tauHold((double)0), _tauLost((double)0), _NHold(0),
          _NLost(0),_NComp(0) , _initS(&_machine) , _inspeS(&_machine) , _hoveringS(&_machine) , _holdS(&_machine)  , _asceS(&_machine)  , _descS(&_machine),_compS(&_machine),
          _rtolS(&_machine),_landS(&_machine),_landingDoneS(&_machine), _err(0,0,0), _err_int(0,0,0), _dt(0), _prevTime(0), _actualTime(0),
          _actualState(0),_prevState(0), _verticalErr(0), _holdPIDX(0,0,0),
          _holdPIDY(0,0,0)
{
  //set the state machine
  initStateMachine();
  ////Load parameters
  std::string pkg_path = ament_index_cpp::get_package_share_directory("guidance_system");
  std::string params_path = pkg_path + "/params/Landing_params.txt";
	param.loadConfigFile(params_path.c_str());

	_envSim = (bool)std::getenv("M2M_SIM");
	std::cout << "[OFFSETS] M2M_SIM " << _envSim
	          << " -> camera " << (_envSim ? param.cameraOffsetSim : param.cameraOffsetHw)
	          << " sonar "    << (_envSim ? param.sonarOffsetSim  : param.sonarOffsetHw)
	          << std::endl;

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
	_NHold = 0;
	_NLost = 0;
	_NComp = 0;

	_err << 0,0,0;
	_err_int << 0,0,0;
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
	_touchAccelBuf.clear();
	_touchAccelZSmoothed = 0.0;
	_padAccelBuf.clear();
	_padAccelZSmoothed   = 0.0;
	_wasInLand           = false;
	_touchContactTimer   = 0.0;
	_touchTimeSinceSpike = 1e9;
	_touchConfirmTimer   = 0.0;
	_touchedDown         = false;
	_touchdownLogged     = false;
	_compValidTimer      = 0.0;
	_landValidTimer      = 0.0;
	_disarmRequest       = false;
	_disarmRetryElapsed  = 0.0;
	_disarmRetrySince    = 0.0;
	_approachSettleTimer = 0.0;
	_approachLogTimer    = 0.0;
	_predHistRoll.clear();
	_predHistPitch.clear();
	_predHistHeavePos.clear();
	_predHistHeaveVel.clear();

  //Link signals
  //assign initialized errors of the Lander class with
  //the one of the LandMachine.
  _machine._horizontaErr =  &_horizontaErr;
  _machine._verticalErr  =  &_verticalErr;
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
  _machine._disarmRequest = &_disarmRequest;
  _machine._compValidTimer = &_compValidTimer;
  _machine._landValidTimer = &_landValidTimer;
  _machine._heaveUpMax    = &_heaveUpMax;
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
  _compS._nextDoneState = &_landingDoneS;   // disarmOnGo -> straight to terminal

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

double Lander::smoothAccel(double rawSample, std::deque<double> &buf) {
  // Clip before averaging: a single sensor glitch/dropout would otherwise
  // dominate the short window and read out as a wildly oversized "acceleration".
  double clipped = common::clamp(rawSample, -kAccelEstimateClip, kAccelEstimateClip);

  buf.push_back(clipped);
  if (buf.size() > kAccelEstimateWindow) buf.pop_front();
  double sum = 0.0;
  for (double v : buf) sum += v;
  return sum / static_cast<double>(buf.size());
}

void Lander::pushPredHistory(std::deque<std::pair<double,double>> &buf, double value, double dt, double maxAge) {
  buf.emplace_back(dt, value);
  double span = 0.0;
  for (auto &s : buf) span += s.first;
  while (buf.size() > 1 && span > maxAge) {
    span -= buf.front().first;
    buf.pop_front();
  }
}

double Lander::lookupPredHistory(const std::deque<std::pair<double,double>> &buf, double lookback, double &achievedLookback) const {
  achievedLookback = 0.0;
  if (buf.empty()) return 0.0;

  double age = 0.0;
  for (auto it = buf.rbegin(); it != buf.rend(); ++it) {
    age += it->first;
    if (age >= lookback) {
      achievedLookback = age;
      return it->second;
    }
  }
  // History doesn't extend far enough back yet — return the oldest we have.
  achievedLookback = age;
  return buf.front().second;
}

void Lander::logTouchdownCsv() {

  // Named by wall-clock creation time (down to the millisecond) so repeated
  // landing attempts/sim runs never clash.
  auto now = std::chrono::system_clock::now();
  std::time_t now_c = std::chrono::system_clock::to_time_t(now);
  auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                  now.time_since_epoch()) % 1000;

  std::ostringstream fname;
  fname << "touchdown_"
        << std::put_time(std::localtime(&now_c), "%Y%m%d_%H%M%S")
        << "_" << std::setfill('0') << std::setw(3) << now_ms.count()
        << ".csv";

  std::string pkg_path   = ament_index_cpp::get_package_share_directory("guidance_system");
  std::string results_dir = pkg_path + "/results";
  std::string file_path   = results_dir + "/" + fname.str();

  std::ofstream csv(file_path);
  if (!csv.is_open()) {
    std::cout << "[TOUCHDOWN] failed to open " << file_path
              << " for the touchdown CSV dump (does " << results_dir << " exist?)" << std::endl;
    return;
  }

  double droneRoll, dronePitch, droneYaw;
  _state.getOrientationRPY(droneRoll, dronePitch, droneYaw);

  // What the SPA/heave predictors said touchPredLookback seconds before this
  // exact touchdown moment — compare against the raw (actual) columns above to
  // see how accurate the prediction pipeline actually was.
  // The four lookups all land at (~) the same age (they're pushed in lockstep
  // every tick) — only the roll one's age is kept, as the representative
  // "how far back did we actually manage to look".
  double rollAge = 0.0, unusedAge = 0.0;
  double rollPredLookback     = lookupPredHistory(_predHistRoll,     param.touchPredLookback, rollAge);
  double pitchPredLookback    = lookupPredHistory(_predHistPitch,    param.touchPredLookback, unusedAge);
  double heavePosPredLookback = lookupPredHistory(_predHistHeavePos, param.touchPredLookback, unusedAge);
  double heaveVelPredLookback = lookupPredHistory(_predHistHeaveVel, param.touchPredLookback, unusedAge);

  csv << "horizontal_error_x_m,horizontal_error_y_m,horizontal_error_norm_m,vertical_error_m,"
      << "drone_x_m,drone_y_m,drone_z_m,drone_vx_mps,drone_vy_mps,drone_vz_mps,"
      << "drone_roll_rad,drone_pitch_rad,drone_yaw_rad,"
      << "platform_roll_rad,platform_pitch_rad,platform_roll_pred_rad,platform_pitch_pred_rad,"
      << "platform_roll_rate_radps,platform_pitch_rate_radps,"
      << "heave_pos_m,heave_vel_mps,heave_pos_pred_m,heave_vel_pred_mps,"
      << "platform_roll_pred_lookback_rad,platform_pitch_pred_lookback_rad,"
      << "heave_pos_pred_lookback_m,heave_vel_pred_lookback_mps,pred_lookback_actual_s,"
      << "ultrasonic_z_m,"
      << "drone_accel_z_mps2,pad_accel_z_mps2,"
      << "touch_contact_timer_s,touch_confirm_timer_s,touch_time_since_spike_s,"
      << "land_vote_fraction,land_latch_timer_s,landing_go\n";

  csv << _VisionPose.getX()   << "," << _VisionPose.getY()   << "," << _horizontaErr << "," << _verticalErr << ","
      << _state.getX()        << "," << _state.getY()        << "," << _state.getZ()        << ","
      << _state.getVx()       << "," << _state.getVy()       << "," << _state.getVz()       << ","
      << droneRoll             << "," << dronePitch           << "," << droneYaw             << ","
      << _seaState.roll        << "," << _seaState.pitch      << ","
      << _seaState.spaRollPred << "," << _seaState.spaPitchPred << ","
      << _seaState.rollRate    << "," << _seaState.pitchRate  << ","
      << _seaState.heavePos    << "," << _seaState.heaveVel   << ","
      << _seaState.heavePosPred << "," << _seaState.heaveVelPred << ","
      << rollPredLookback << "," << pitchPredLookback << ","
      << heavePosPredLookback << "," << heaveVelPredLookback << ","
      << rollAge << ","
      << _UltraInfo.getZ()     << ","
      << _goDiag.touchAccelZ   << "," << _goDiag.touchPadAccelZ << ","
      << _goDiag.touchContactTimer << "," << _goDiag.touchConfirmTimer << "," << _goDiag.touchTimeSinceSpike << ","
      << _goDiag.voteFraction  << "," << _goDiag.latchTimer   << "," << _goDiag.landingGo
      << "\n";

  csv.close();
  std::cout << "[TOUCHDOWN] stats written to " << file_path << std::endl;
}

void Lander::updateSignals() {

  // Keep a rolling history of the SPA/heave PREDICTIONS (touchPredLookback
  // seconds, plus margin), regardless of state, so logTouchdownCsv() can look
  // back into "what the predictor said touchPredLookback seconds before
  // touchdown" once touchdown actually happens.
  {
    double maxAge = param.touchPredLookback + 1.0;
    pushPredHistory(_predHistRoll,     _seaState.spaRollPred,   _seaState.dt, maxAge);
    pushPredHistory(_predHistPitch,    _seaState.spaPitchPred,  _seaState.dt, maxAge);
    pushPredHistory(_predHistHeavePos, _seaState.heavePosPred,  _seaState.dt, maxAge);
    pushPredHistory(_predHistHeaveVel, _seaState.heaveVelPred,  _seaState.dt, maxAge);
  }

  // Rolling PEAK of recent upward heave velocity, feeding comp()'s extra
  // standoff margin (see param.compHeaveMarginWindow / _heaveUpMax's
  // declaration). Pushed every tick (like the history above) so the window
  // is pruned by real elapsed time regardless of sign -- but the value
  // pushed is heaveVel clamped to >= 0, so a downward tick can only ever
  // contribute 0 to the max, never pull it down.
  {
    double v = std::max(_seaState.heaveVel, 0.0);
    pushPredHistory(_heaveUpSamples, v, _seaState.dt, param.compHeaveMarginWindow);
    double peak = 0.0;
    for (auto &s : _heaveUpSamples) peak = std::max(peak, s.second);
    _heaveUpMax = peak;
  }

	if( _VisionPose.VisionDataUpdated )
    Inspection = false;

  if(_VisionPose.IsValid)
    _hovering = false;


	double dx;
	double dy;
	double dz;


	switchSensor = (fabs( _VisionPose.getZ() ) > 0.83); 
  //above half meters use vision system for altitude value
  //otherwise, ultrasonic sensors.

  // Offset per source, SELECTED by environment — the sim and hardware values
  // are alternatives, never summed. The simulated sensors sit at different
  // places relative to the drone reference point than the real ones, so each
  // environment gets its own pair. Same M2M_SIM flag Automatic and Executioner
  // already switch on.
  const double camOffset   = _envSim ? param.cameraOffsetSim : param.cameraOffsetHw;
  const double sonarOffset = _envSim ? param.sonarOffsetSim  : param.sonarOffsetHw;

  // dz is negative-down (downDist = -dz), so a positive offset is SUBTRACTED
  // here to push the reported distance up in both branches.
  if( switchSensor )
		dz = _VisionPose.getZ() - camOffset;
	else
		dz = -(_UltraInfo.getZ() + sonarOffset);

  {
    // Both sources are printed with their offsets applied, so the step at the
    // switchSensor handover is directly visible: compare visionDist against
    // sonarDist on the tick the source flips. They must agree there, or
    // downDist jumps — which moves the landing height gate and injects a step
    // into comp()'s tracking error at the worst possible moment.
    static int sensorLogTick = 0;
    if ((sensorLogTick++ % 25) == 0) {
      const double visionDist = -_VisionPose.getZ() + camOffset;
      const double sonarDist  =  _UltraInfo.getZ()  + sonarOffset;
      // std::cout << "[SENSOR] switchSensor " << switchSensor
      //           << " | visionRaw " << -_VisionPose.getZ()
      //           << " +off " << camOffset << " = " << visionDist
      //           << " | sonarRaw " << _UltraInfo.getZ()
      //           << " +off " << sonarOffset << " = " << sonarDist
      //           << " | handoverStep " << (sonarDist - visionDist)
      //           << " ultraValid " << _UltraInfo.IsValid
      //           << " ultraUpdated " << _UltraInfo.UltrasonicDataUpdated
      //           << " downDist " << -dz << std::endl;
    }
  }


  dx = _VisionPose.getX();

	dy = _VisionPose.getY();

  _err[0] = dx;
  _err[1] = dy;
  _err[2] = 0; //Trick, used to calculate horizontal error

  _horizontaErr = _err.norm();
  _err[2]       = dz;
  _verticalErr  = _err[2];

  updatePlatformVision();

  // Prediction horizon: time to reach platform at current descent velocity, capped.
  double predHorizonMax = param.zMax / param.landDescVel;
  _predictionHorizon = std::min(fabs(_verticalErr) / param.landDescVel, predHorizonMax);

  // Cone thresholds: linearly interpolate between min (at z=0) and max (at z=cone_z_ref).
  // TODO: Change hold_threshold to hold_threshold_max and same for lost and comp
  double cone_th = common::clamp(fabs(_verticalErr) / param.cone_z_ref, 0.0, 1.0);
  double tau_hold_cone = param.hold_threshold_min + (param.hold_threshold - param.hold_threshold_min) * cone_th;
  double tau_lost_cone = param.lost_threshold_min + (param.lost_threshold - param.lost_threshold_min) * cone_th;
  double tau_comp_cone = param.comp_threshold_min + (param.comp_threshold - param.comp_threshold_min) * cone_th;

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

  if(_actualState == AbstractLandState::states::COMP)
    _centered = _horizontaErr < tau_comp_cone && _VisionPose.VisionDataUpdated;
  else
    _centered = _horizontaErr < tau_hold_cone && _VisionPose.VisionDataUpdated;
  // if(!_centered){
  //   std::cout << "_vertical_Err" << _verticalErr << "_horizontaErr " << _horizontaErr << " tau_comp_cone " << tau_comp_cone << " _VisionPose.VisionDataUpdated " << _VisionPose.VisionDataUpdated << std::endl;
  // }
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
  // pad is not rushing upward at the drone. The raw (instantaneous) decision is noisy —
  // it's smoothed by a majority vote over the last landVoteWindow seconds before
  // it feeds the latch, so a single bad sample can't flip the landing decision.
  // The latch then holds each (voted) decision for at least landLatchTime before
  // accepting the opposite.
  {
    bool anglesOk = std::fabs(_seaState.spaRollPred)  < param.landRollLimit
                 && std::fabs(_seaState.spaPitchPred) < param.landPitchLimit;
    bool ratesOk  = std::fabs(_seaState.rollRate)  < param.landRollRateLimit
                 && std::fabs(_seaState.pitchRate) < param.landPitchRateLimit;
    // One-sided ON PURPOSE, and sign-sensitive: heaveVelPred is up-positive, so
    // the pad may descend as fast as it likes (the gap only opens), but
    // committing while it is rushing UP at the drone is what slams the pad into
    // it. Not a fabs() limit — that would also veto the safe half of the cycle.
    bool padNotRushingUp = (_seaState.heaveVelPred < param.landHeaveUpVelLimit);
    bool rawGo = _seaState.spaValid && _seaState.heaveValid
              && anglesOk && ratesOk && padNotRushingUp;

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
    bool votedGo = voteFraction >= param.landVoteThreshold || std::abs(_UltraInfo.getZ() < 0.2);

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
    _goDiag.descFromPeak   = padNotRushingUp;
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
  // then confirm). NOT based on relative vertical velocity (drone vz -
  // pad heaveVel) — PX4's odometry/EKF velocity output degrades once the drone
  // stops moving smoothly, i.e. exactly near/at touchdown, so "velocity ~= 0
  // relative to the pad" is unreliable when it matters most.
  //
  // Both accel signals come from real IMUs, not derivatives of velocity: the pad
  // from its raw IMU (imu/data) and the drone from PX4's own
  // — see SeaState.h. The two sources use DIFFERENT
  // fixed gravity conventions (the pad simulator publishes az_true - g; a real
  // accelerometer reads az_true + g, "specific force", once flipped to
  // up-positive), so each is corrected back to true kinematic acceleration here
  // before comparing — otherwise they'd differ by a constant ~2g regardless of
  // any real motion.
  //
  // Both estimates are smoothed UNCONDITIONALLY, every tick in every state — not
  // just while in LAND — so they're already warm and settled by the time LAND
  // (and its terminal touchdown check) starts.
  // droneAccelZ is already flipped to up-positive specific force in
  // AutomaticNode::onDroneAccel() (raw PX4 xyz[2] is FRD, Z down).
  double droneAccelTrue = _seaState.droneAccelZ - kGravityMs2;
  double padAccelTrue   = _seaState.padAccelZ   + kGravityMs2;
  _touchAccelZSmoothed = smoothAccel(droneAccelTrue, _touchAccelBuf);
  _padAccelZSmoothed   = smoothAccel(padAccelTrue,   _padAccelBuf);

  // Impact-deceleration spike: touching down arrests the descent, producing a
  // brief POSITIVE acceleration bump on the drone (up-positive convention) — not
  // an absolute-value test, a downward spike is just more descending, not contact.
  bool accelSpike = droneAccelTrue > param.touchAccelSpike;
  _touchTimeSinceSpike = accelSpike ? 0.0 : (_touchTimeSinceSpike + _seaState.dt);

  bool inLand = (_actualState == AbstractLandState::states::LAND);
  if (inLand && !_wasInLand) {
    // Fresh entry into LAND (including a GO -> NO-GO -> GO round trip): start the
    // per-attempt stage timers clean, so a stale "recent spike"/partial contact
    // time from a previous attempt (or from flight before LAND even started)
    // can't cause a confused touchdown verdict this time around. The accel
    // estimates themselves are deliberately NOT reset here — they stay warm.
    _touchContactTimer   = 0.0;
    _touchConfirmTimer   = 0.0;
    _touchTimeSinceSpike = param.touchAccelSpikeWindow+0.1;
  }
  _wasInLand = inLand;

  // Only evaluated while actually committed to landing (LAND), and only once
  // both real IMU sources are fresh; _touchedDown latches permanently once
  // confirmed and is only cleared by initStateMachine().
  if (inLand && !_touchedDown) {
    bool sensorsOk   = _seaState.padImuValid && _seaState.droneAccelValid;
    bool accelMatches = sensorsOk
                     && std::fabs(_touchAccelZSmoothed - _padAccelZSmoothed) < param.touchAccelMatchTol;
    bool xyStill      = std::fabs(_state.getVx()) < param.touchVelRelMax
                      && std::fabs(_state.getVy()) < param.touchVelRelMax;
    bool contactCond  =  xyStill;

    // Stage 1: drone/pad accel matched + xy settled, sustained touchContactTime.
    _touchContactTimer = contactCond ? (_touchContactTimer + _seaState.dt) : 0.0;
    bool groundContact = _touchContactTimer >= param.touchContactTime;

    // Stage 2: contact + a recent positive impact spike, sustained touchConfirmTime.
    bool spikeRecent = _touchTimeSinceSpike <= param.touchAccelSpikeWindow;
    _touchConfirmTimer = (groundContact && spikeRecent) ? (_touchConfirmTimer + _seaState.dt) : 0.0;
    bool confirmed = _touchConfirmTimer >= param.touchConfirmTime;

    if (confirmed) {
      _touchedDown = true;
      std::cout << "[TOUCHDOWN] confirmed: droneAz=" << _touchAccelZSmoothed
                << " padAz=" << _padAccelZSmoothed << " groundContact=" << groundContact << std::endl;
    }

    _goDiag.touchAccelZ        = droneAccelTrue;
    _goDiag.touchPadAccelZ     = _padAccelZSmoothed;
    _goDiag.touchGroundContact = groundContact;
    _goDiag.touchConfirmed     = confirmed;
    _goDiag.touchedDown        = _touchedDown;
    _goDiag.touchContactTimer   = _touchContactTimer;
    _goDiag.touchConfirmTimer   = _touchConfirmTimer;
    _goDiag.touchTimeSinceSpike = _touchTimeSinceSpike;

    if (_touchedDown && !_touchdownLogged) {
      logTouchdownCsv();
      _touchdownLogged = true;
    }
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
	std::cout<<"Ultrasonic_Z:"<<(_UltraInfo.getZ() + (_envSim ? param.sonarOffsetSim : param.sonarOffsetHw))<<std::endl;
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
  // Reset the feed-forward velocity every tick — only comp() (and land()/
  // LANDING_DONE, which set their own Vz) should ever drive it non-zero, so
  // any state that falls through without touching Vz publishes zero
  // feed-forward instead of leaking a stale value from a previous tick.
  _setPoint.setVz(0.0);
  static bool initDone = false;
  switch (_actualState){

    case (AbstractLandState::states::INIT):
      std::cout << "in init "<< " initDone " << initDone << " startFromBeginning " << startFromBeginning << std::endl;
      if(!initDone)
      {

				if (print > 30) std::cout<<"INIT"<<std::endl;

				init();
				/*
				It is divided in two parts:
					--set the setpoint to the actual drone position.
					--set the z axis to the max one 2.5 meters.
				*/

                //initDone = true;
        // positioning the drone in front of the platform; also updates
        // _yaw_des to the platform's live heading, so align() (below) uses
        // this tick's value rather than a tick-stale one.
        _readyToLand = setDroneInFront();
        align(); //controlling the yaw
				initDone = _VisionPose.VisionDataUpdated && _readyToLand;
        std::cout << "_VisionPose.VisionDataUpdated " << _VisionPose.VisionDataUpdated << "_readyToLand" << _readyToLand << std::endl;
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

        // positioning the drone in front of the platform; also updates
        // _yaw_des to the platform's live heading, so align() (below) uses
        // this tick's value rather than a tick-stale one.
        _readyToLand = setDroneInFront();
        align(); //controlling the yaw
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

      // Terminal state. A fresh landing task (initStateMachine()) is the only
      // way out.
      _setPoint.setVx(0);
      _setPoint.setVy(0);

      if (param.disarmOnGo) {
        // Disarm mode arrives here still FLYING (just above the pad), not
        // already landed, so the push-down below must not run: it would command
        // a landDescVel descent under offboard velocity control and land the
        // drone the very way disarm mode exists to skip. Hold zero vertical
        // velocity and let the disarm stop the motors.
        _setPoint.setVz(0.0);

        // Re-send the disarm for a bounded window — the command goes out
        // best-effort, so one dropped message would leave the drone armed and
        // hovering here indefinitely.
        if (_disarmRetryElapsed < 2.0) {
          _disarmRetryElapsed += _dt;
          _disarmRetrySince   += _dt;
          if (_disarmRetrySince >= 0.1) {
            _disarmRetrySince = 0.0;
            _disarmRequest    = true;
          }
        }
      } else {
        // Normal path: touchdown is already confirmed, so keep pushing down to
        // stay planted until PX4's own land detector disarms.
        _setPoint.setVz(-param.landDescVel);
      }

      _setPoint.setType(MavState::VELOCITY);
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

void Lander::updatePlatformVision() {

  const bool visionOk = _VisionPose.IsValid && _VisionPose.VisionDataUpdated;

  // Position. The apriltag node already rotates its output into world-aligned
  // axes (correctToPlatformFrame()), so the vision offset is a plain
  // drone->platform vector in the same frame as _state — no rotation needed.
  // Z deliberately uses _verticalErr rather than _VisionPose.getZ(): that is
  // the offset-corrected, sensor-selected vertical, so the platform Z derived
  // here is exactly consistent with downDist.
  const double px = _state.getX() + _VisionPose.getX();
  const double py = _state.getY() + _VisionPose.getY();
  const double pz = _state.getZ() + _verticalErr;

  // Velocity. vision = platform - drone, so d(vision)/dt = plat_vel - drone_vel
  // and plat_vel = drone_vel + d(vision)/dt. The drone's own velocity carries
  // the bulk of the signal and the differentiated vision term is a small
  // correction — which is what makes differentiating a noisy measurement
  // workable here. Still low-passed, since d/dt amplifies jitter by 1/dt.
  if (!visionOk || _dt <= 1e-6) {
    // Nothing trustworthy to differentiate: hold the last estimate and re-seed
    // when vision returns, so a dropout cannot inject a huge false derivative.
    _platVisSeeded = false;
  } else {
    if (!_platVisSeeded) {
      _prevVisX = _VisionPose.getX();
      _prevVisY = _VisionPose.getY();
      _platVisFiltVx = _state.getVx();
      _platVisFiltVy = _state.getVy();
      _platVisSeeded = true;
    }

    const double rawVx = _state.getVx() + (_VisionPose.getX() - _prevVisX) / _dt;
    const double rawVy = _state.getVy() + (_VisionPose.getY() - _prevVisY) / _dt;
    _prevVisX = _VisionPose.getX();
    _prevVisY = _VisionPose.getY();

    const double a = param.platVisionVelAlpha;
    _platVisFiltVx = a * _platVisFiltVx + (1.0 - a) * rawVx;
    _platVisFiltVy = a * _platVisFiltVy + (1.0 - a) * rawVy;
  }

  // Yaw. _VisionPose's quaternion is the platform's orientation relative to the
  // drone body, so the platform's heading is the drone's plus that relative
  // term. Stored via setYaw() because MavState::setOrientation() does NOT touch
  // the _yaw scalar that getYaw() returns — which is exactly why the old
  // _platformState.getYaw() was always 0 (mavStateFromNavOdom only ever calls
  // setOrientation), pinning the approach slope to the Y axis.
  double platYaw = _state.getYawFromQuat() + _VisionPose.getYawFromQuat();
  while (platYaw >   PI) platYaw -= 2.0 * PI;
  while (platYaw <= -PI) platYaw += 2.0 * PI;

  _platformVision.setX(px);
  _platformVision.setY(py);
  _platformVision.setZ(pz);
  _platformVision.setVx(_platVisFiltVx);
  _platformVision.setVy(_platVisFiltVy);
  _platformVision.setYaw(platYaw);
  _platformVision.IsValid = visionOk;

  // Vision estimate vs the ground-truth topic, side by side, for verification.
  // _platformState is Landing_Site/pose, which in simulation is the Gazebo
  // plugin echoing the pose it wrote onto the model. Expect a CONSTANT offset
  // on X/Y/Z (the two live in different frame origins) — what matters is that
  // the two track the same shape and that the velocities agree in magnitude.
  {
    static int platLogTick = 0;
    if ((platLogTick++ % 10) == 0) {
      // std::cout << "[PLAT] vision pos (" << px << "," << py << "," << pz << ")"
      //           << " vel (" << _platVisFiltVx << "," << _platVisFiltVy << ")"
      //           << " yaw " << platYaw
      //           << " | truth pos (" << _platformState.getX() << "," << _platformState.getY()
      //           << "," << _platformState.getZ() << ")"
      //           << " vel (" << _platformState.getVx() << "," << _platformState.getVy() << ")"
      //           << " yaw " << _platformState.getYaw()
      //           << " | dPos (" << (px - _platformState.getX())
      //           << "," << (py - _platformState.getY())
      //           << "," << (pz - _platformState.getZ()) << ")"
      //           << " dVel (" << (_platVisFiltVx - _platformState.getVx())
      //           << "," << (_platVisFiltVy - _platformState.getVy()) << ")"
      //           << " visionOk " << visionOk << std::endl;
    }
  }
}

bool Lander::setDroneInFront()
{
  // this function computes the desired position the drone has to reach before
  // attempting the landing. That position is computed to be in front of the platform.
  _slopeActive = true;
  // getting the yaw of the platform to know how it's oriented.
  // GROUND TRUTH (Landing_Site/pose) — kept commented for reference:
  // double yaw_plat = _platformState.getYaw();
  // NOTE this always returned 0: mavStateFromNavOdom() only calls
  // setOrientation(), and MavState::setOrientation() does not update the _yaw
  // scalar that getYaw() reads. The slope direction was therefore pinned to the
  // Y axis regardless of the platform's real heading.
  double yaw_plat = _platformVision.getYaw();

  if (!(yaw_plat >= -PI && yaw_plat < PI))
  {
    std::cout << "YAW NOT VALID, CANNOT GENERATE DESIRED POSITION" << std::endl;
    return false;
  }

  // Track the platform's live heading instead of whatever static yaw the
  // mission task carries — Land.hpp's setYaw(_actualTask->yaw) is a fixed
  // config value, unrelated to the pad's actual orientation.
  //
  // Sign: yaw_plat (like all getYawFromQuat() readings) is PX4's raw native
  // yaw (0 = North, +ve = North->East) because bridge_node's onPx4Odom swaps
  // NED->ENU on POSITION only, leaving the orientation quaternion untouched.
  // _yaw_des instead feeds yawToQuaternion() -> bridge_node's
  // setpoint.yaw = -yawFromQuaternion(...), which is a pure sign flip (exact
  // inverse of yawToQuaternion). So the native yaw PX4 actually receives is
  // -_yaw_des; to converge the drone's native yaw on yaw_plat we must command
  // the negation.
  setYaw(-yaw_plat);

  // getting also the distance from the center the drone has to keep once positioned,
  // passed from file; it also must be not less than 0.
  double gain = fabs(param.distanceFromCenter);

  // The platform's current absolute world position, recovered from the
  // drone's own pose plus the (relative) vision measurement — same trick
  // trackXY() uses. Led by the platform's own measured velocity
  // (feed-forward): without this, a moving platform makes the desired
  // standoff point perpetually recede — a pure position-error law with no
  // leading term settles to a nonzero steady-state tracking lag against a
  // steadily drifting target and never actually closes the gap.
  // GROUND TRUTH — kept commented for reference:
  // double plat_x = _state.getX() + _VisionPose.getX() + _platformState.getVx() * param.reactionLag;
  // double plat_y = _state.getY() + _VisionPose.getY() + _platformState.getVy() * param.reactionLag;
  double plat_x = _platformVision.getX() + _platformVision.getVx() * param.reactionLag;
  double plat_y = _platformVision.getY() + _platformVision.getVy() * param.reactionLag;

  // then, depending on how the platform is oriented, the desired position changes.
  // yaw_plat is PX4's raw native yaw (0 = North, +ve = North->East) — see the
  // sign note above setYaw(-yaw_plat). This local (x, y) position frame is
  // ENU (x = East, y = North; onPx4Odom swaps position axes), so the
  // platform's forward unit vector in (x, y) is (sin(yaw_plat), cos(yaw_plat)):
  // North-component cos(yaw_plat) lands on y, East-component sin(yaw_plat) on x.
  double xTarget = plat_x + gain*sinf(yaw_plat);
  double yTarget = plat_y + gain*cosf(yaw_plat);

  // setting the setpoint finally
  _setPoint.setPosition(xTarget, yTarget, param.prelandAltitude);
  _setPoint.setType(MavState::type::POSITION);

  // Genuine physical proximity to the standoff point — NOT the magnitude of
  // a controller output (which only meant "close" while the platform was
  // stationary; against a moving platform a PID's steady-state output stays
  // nonzero indefinitely and this check would never pass).
  double errX = xTarget - _state.getX();
  double errY = yTarget - _state.getY();
  bool closeEnough = fabs(errX) < param.initApproachPosThld
                   && fabs(errY) < param.initApproachPosThld;

  // Require it to hold CONTINUOUSLY for initApproachWaitTime seconds before
  // actually releasing — same continuous-timer pattern as the HOLD -> COMP
  // and COMP -> LAND gates, so a brief flicker into "close enough" isn't
  // enough. Reset (not just pause) the instant it drops.
  if (closeEnough) {
    _approachSettleTimer += _dt;
  } else {
    _approachSettleTimer = 0.0;
  }
  bool ready = closeEnough && (_approachSettleTimer >= param.initApproachWaitTime);
  std::cout << "closeEnough " << closeEnough << " _approachSettleTimer " << _approachSettleTimer << std::endl;
  std::cout << "_platformVision.getVx() " << _platformVision.getVx() << " _platformVision.getX() " << _platformVision.getX() << std::endl;
  // Diagnostic log, throttled to ~1 Hz (this runs at the full control-loop
  // rate, e.g. 50 Hz) — enough to see whether the gap is shrinking, holding
  // steady, or growing, and whether the threshold or the settle timer is
  // what's currently blocking release, without flooding the console. Also
  // prints the platform's own measured velocity, since a platform drifting
  // faster than the bridge's position-setpoint slew limit (kMaxVelocity in
  // bridge_node.cpp) can never actually be closed on regardless of what
  // target this function computes — worth comparing against that limit.
  _approachLogTimer += _dt;
  if (_approachLogTimer >= 1.0) {
    _approachLogTimer = 0.0;
    std::cout << "[setDroneInFront] target=(" << xTarget << "," << yTarget << ")"
              << " state=(" << _state.getX() << "," << _state.getY() << ")"
              << " err=(" << errX << "," << errY << ")"
              << " platVel=(" << _platformVision.getVx() << "," << _platformVision.getVy() << ")"
              << " yawPlat " << yaw_plat
              << " yawDes " << _yaw_des
              << " droneYawActual " << _state.getYawFromQuat()
              << " spYaw " << _setPoint.getYaw()
              << " closeEnough " << closeEnough
              << " settleTimer " << _approachSettleTimer << "/" << param.initApproachWaitTime
              << " ready " << ready << std::endl;
  }

  if (ready)
  {
    // setting the slope vector for the next step, simplifying the computations
    // _platformState.getX() - (_platformState.getX() + gain*sinf(yaw_plat))
    _line_direction[0] = -gain*sinf(yaw_plat);
    _line_direction[1] = -gain*cosf(yaw_plat);
    // GROUND TRUTH — kept commented for reference:
    // _line_direction[2] = _state.getZ() - (_platformState.getZ() + param.z_margin);
    _line_direction[2] = _state.getZ() - (_platformVision.getZ() + param.z_margin);
    _approachSettleTimer = 0.0;   // consumed — start clean for next time
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
    if(_slopeActive)
    {
      // GROUND TRUTH — kept commented for reference:
      // double z_base = _platformState.getZ() + param.z_margin;
      // The vision-derived Z is also in the DRONE's frame, where the setpoint is
      // commanded; _platformState is in the simulator's world frame, so the old
      // form mixed frames as well as using truth.
      double z_base = _platformVision.getZ() + param.z_margin;
      double dot    = _VisionPose.getX() * _line_direction[0] + _VisionPose.getY() * _line_direction[1];
      double norm_sq = _line_direction[0] * _line_direction[0] + _line_direction[1] * _line_direction[1];
      double z_adj = z_base + _line_direction[2] * dot / norm_sq;

      // Feed-forward the descent rate implied by the line instead of relying
      // on PX4's position loop to infer it from a moving setpoint. z_adj
      // slides every tick as dot closes -- purely commanding the position
      // makes PX4 chase a receding target and lag behind, which is what the
      // (removed) position-feedback term was trying, unstably, to paper
      // over. Differentiate z_adj analytically via the chain rule instead
      // (dot's own rate is the horizontal closing velocity projected onto
      // the line direction) -- same feed-forward technique desc()/comp()
      // already use for their own Vz.
      double closingVx = _platformVision.getVx() - _state.getVx();
      double closingVy = _platformVision.getVy() - _state.getVy();
      double dDotDt = closingVx * _line_direction[0] + closingVy * _line_direction[1];
      double zDotFF = _line_direction[2] * dDotDt / norm_sq;

      std::cout << "z_base "  << z_base << " z_adj " << z_adj << " zDotFF " << zDotFF << " state z "<< _state.getZ() << " _line_direction[2] " << _line_direction[2]
      << " dot " << dot << " norm_sq " << norm_sq << " dot / norm_sq " << dot / norm_sq  << " _VisionPose.getX() " << (_VisionPose.getX()) <<
      " _VisionPose.getY() " << _VisionPose.getY() << " _state.getX() " <<_state.getX() << std::endl;
      _setPoint.setVz(zDotFF);
      _setPoint.setZ(z_adj);
        // std::cout << "hold setting z pos !desc_v " << !desc_v << " !asce_v " << !asce_v
        //           << " !asce_min " << !asce_min << " !asce_margin " << !asce_margin
        //           << " z_adj " << z_adj << " centered " << _centered << std::endl;
      clampZSP();
      if(std::fabs(_VisionPose.getX()) < 0.1 && std::fabs(_VisionPose.getY()) < 0.1){
        std::cout << "disabling slope" << std::endl;
        _slopeActive = false;
      }
    }
    _setPoint.setType(MavState::POSITION);
  }


  trackXY();

}

void Lander::trackXY() {

  const bool visionOk = _VisionPose.IsValid && _VisionPose.VisionDataUpdated;

  // Platform velocity feed-forward — Vdes = K*ep + Vplat, per hold()'s own
  // doc comment above, which described this law but never actually
  // implemented the +Vplat term. Without it, either branch below is a pure
  // reactive law chasing the CURRENT vision offset with no leading term: against
  // a genuinely drifting platform that's a ramp-tracking problem — the
  // controller has to sustain a persistent nonzero error just to keep pace,
  // and if the platform outruns what it can produce, the tracking error
  // grows without ever converging (the drone stays trailing behind,
  // "attached to the tail" of the platform, instead of centering on it).
  // Same fix, and same lead convention (param.reactionLag), as
  // setDroneInFront()'s equivalent feed-forward.
  // GROUND TRUTH — kept commented for reference:
  // double ffX = visionOk ? _platformState.getVx() * param.reactionLag : 0.0;
  // double ffY = visionOk ? _platformState.getVy() * param.reactionLag : 0.0;
  double ffX = visionOk ? _platformVision.getVx()  : 0.0;
  double ffY = visionOk ? _platformVision.getVy() : 0.0;

  // Captured for the diagnostic below (mode 1 only; stays 0 in direct mode).
  double pidOutX = 0.0, pidOutY = 0.0;

  if (param.xyControlMode == 1) {
    // ── XY: PID on the horizontal vision error ─────────────────────────────
    // Drives the drone->platform vision error (_VisionPose.getX()/getY()) to
    // zero: setpoint = current position + PID(error) + platform velocity lead.
    // Reset the integrator whenever vision drops out so it can't wind up
    // while blind.
    _holdPIDX.setDt(_dt);
    _holdPIDY.setDt(_dt);

    if (!visionOk) {
      _holdPIDX.reset();
      _holdPIDY.reset();
      _setPoint.setPosition(_state.getX(), _state.getY(), _setPoint.getZ());
    } else {
      pidOutX = _holdPIDX.getOutput(0, _VisionPose.getX());
      pidOutY = _holdPIDY.getOutput(0, _VisionPose.getY());
      double targetX = _state.getX() + pidOutX + ffX;
      double targetY = _state.getY() + pidOutY + ffY;
      _setPoint.setPosition(targetX, targetY, _setPoint.getZ());
    }
  } else {
    // ── XY: absolute platform-position hold (default) ──────────────────────
    // Command the platform's estimated WORLD position, not "drone position + gain*
    // vision". vision = platform - drone, so platform_world = state + vision. This
    // is an absolute target (no walk), low-pass filtered and rate-limited so the
    // vision delay/jitter can't drive the sinusoidal limit cycle that unity-gain
    // "state + vision" produced. Led by the platform's own velocity before
    // filtering, for the same reason as the PID branch above.
    double sp_x, sp_y;
    if (!visionOk) {
      // No trustworthy vision: freeze XY at the current position and re-seed the
      // filter when vision returns.
      sp_x = _state.getX();
      sp_y = _state.getY();
      _spSeeded = false;
    } else {
      double plat_x = _state.getX() + _VisionPose.getX() + ffX;
      double plat_y = _state.getY() + _VisionPose.getY() + ffY;

      if (!_spSeeded) {
        _platFiltX = plat_x;  _platFiltY = plat_y;
        _prevSpX   = plat_x;  _prevSpY   = plat_y;
        _spSeeded  = true;
      }

      // EMA low-pass — the platform's horizontal position is quasi-stationary.
      _platFiltX = param.spFilterAlpha * _platFiltX + (1.0 - param.spFilterAlpha) * plat_x;
      _platFiltY = param.spFilterAlpha * _platFiltY + (1.0 - param.spFilterAlpha) * plat_y;

      // Rate limit so a vision glitch can't fling the setpoint.
      // spMaxVelXY is a SPEED [m/s], so the per-tick position step is
      // speed * dt. It was being applied as a raw per-tick step, which at the
      // 50 Hz control loop meant 3.0 m per tick (~150 m/s) — the limiter never
      // clamped anything, leaving the EMA above as the only damping, where the
      // comment on this branch assumes both are protecting the loop.
      double maxStep = param.spMaxVelXY * _dt;
      sp_x = common::clamp(_platFiltX, _prevSpX - maxStep, _prevSpX + maxStep);
      sp_y = common::clamp(_platFiltY, _prevSpY - maxStep, _prevSpY + maxStep);
    }

    _prevSpX = sp_x;
    _prevSpY = sp_y;
    // _setPoint.setPosition(sp_x, sp_y, _setPoint.getZ());
    //Fill right fields
    if(param.xyControlMode == 1)
      _setPoint.setPosition(sp_x, sp_y, _setPoint.getZ());
    else
       _setPoint.setPosition(sp_x + 0.5*(sp_x - _state.getX()), sp_y + 0.5*(sp_y - _state.getY()), _setPoint.getZ());
  }

  // XY tracking diagnostic, throttled to ~5 Hz.
  //
  // The horizontal error is a 2-vector but the existing slope log prints only
  // _state.getX(), so a divergence that is mostly in Y is invisible. This
  // prints both axes of everything, plus the pieces the commanded target is
  // built from, so a run where the drone falls behind can be attributed:
  //   * platVel large vs the bridge's 0.3 m/s setpoint slew cap
  //         -> the platform is genuinely outrunning the achievable speed
  //   * platVel small but ff ~ 0
  //         -> the velocity feed-forward is not arriving (stale platform state)
  //   * cmdOffset not growing with err
  //         -> the controller is not asking for more as it falls behind
  //           (PID output saturated at maxOutput, or the error is not reaching it)
  {
    static int xyLogTick = 0;
    if ((xyLogTick++ % 10) == 0) {
      const double errNorm = std::sqrt(_VisionPose.getX() * _VisionPose.getX()
                                     + _VisionPose.getY() * _VisionPose.getY());
      // std::cout << "[XY] mode " << param.xyControlMode
      //           << " visionOk " << visionOk
      //           << " err (" << _VisionPose.getX() << "," << _VisionPose.getY() << ") |err| " << errNorm
      //           << " state (" << _state.getX() << "," << _state.getY() << ")"
      //           << " platVel (" << _platformVision.getVx() << "," << _platformVision.getVy() << ")"
      //           << " ff (" << ffX << "," << ffY << ")"
      //           << " pidOut (" << pidOutX << "," << pidOutY << ")"
      //           << " cmd (" << _setPoint.getX() << "," << _setPoint.getY() << ")"
      //           << " cmdOffset (" << (_setPoint.getX() - _state.getX())
      //           << "," << (_setPoint.getY() - _state.getY()) << ")"
      //           << " maxOutput " << param.maxOutput
      //           << std::endl;
    }
  }

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
  double v_hor_calm = std::fabs(_state.getVx()) < 0.3 && std::fabs(_state.getVy()) < 0.3;
  if (!v_hor_calm)
    return;
  double new_z = std::max(_setPoint.getZ() - param.descVelocity,_state.getZ() - param.descVelocity);
  _setPoint.setZ(new_z);
  // Feed the intended descent rate forward on the position command's
  // velocity field (same mechanism comp() already uses for the heave
  // feed-forward — see bridge_node.cpp's POSITION branch), instead of
  // relying purely on the stepped-down position target. Without this the
  // actual descent rate is bottlenecked by the bridge's own
  // position-setpoint slew limit (kMaxVelocity) chasing a target that only
  // moves once every other tick (DescState always bounces back to HOLD
  // after one call), which can end up far slower than descVelocity.
  _setPoint.setVz(-param.descVelocity);
  _setPoint.setType(MavState::POSITION);
}

void Lander::comp() {

  // XY is held by hold() (runs before comp() in the COMP tick) but call
  // trackXY() explicitly here too, so comp() applies the configured XY control
  // mode (direct/PID) on its own and doesn't just inherit it implicitly from
  // run()'s call order. Same-tick recompute — harmless, same inputs.
  trackXY();

  // COMP standoff hold: keep the drone at compHoldDist above the pad while
  // centred.
  //
  // Hysteresis safety: if the gap drops below compMinDist the pad is coming up
  // too close — stop tracking and back off (ascend) until the gap climbs back
  // above compResumeDist, then resume tracking.
  //
  // Uses the CURRENT measured gap plus the predicted pad velocity to drive the
  // relative velocity to zero. The prediction here is led by the drone's own
  // control+communication delay (heaveVelPredComp, from heave_estimator's
  // control_comm_delay_s), NOT by the distance-based landing-decision horizon
  // (heaveVelPred) — comp() needs "where will the pad be by the time my command
  // takes effect", which is a fixed system latency, not a function of how far
  // the drone still is from the platform.

  double gap = -_verticalErr;   // measured drone->pad vertical distance

  // Extra standoff margin, added directly (no gain): calm seas (no recent
  // upward heave) hold at plain compHoldDist; a rougher sea (a nonzero
  // _heaveUpMax) pushes the held distance back out. The CAP applies to the
  // resulting TOTAL standoff distance (compHeaveMarginMax), not to the
  // margin in isolation -- an unbounded standoff would back the drone
  // arbitrarily far off in a rough enough sea, and capping the total is what
  // actually bounds that, not capping how much gets added to whatever
  // compHoldDist already is. Computed once, before the tracking branch, so
  // both the standoff below and the throttled diagnostic log agree on the
  // exact same value.
  double effectiveHoldDist = std::min(param.compHoldDist + _heaveUpMax, param.compHeaveMarginMax);

  // compMinDist/compResumeDist are configured as offsets FROM compHoldDist
  // (how much closer than the hold distance triggers back-off / resume).
  // When _heaveUpMax pushes the effective hold distance out, these two
  // hysteresis bounds must follow it by the same amount so they keep the
  // same physical offset from wherever the drone is actually holding —
  // otherwise a large standoff margin would leave compMinDist/compResumeDist
  // sitting at their configured-for-calm-seas values, far too close to the
  // (now much larger) effective hold distance.
  double effectiveMinDist    = effectiveHoldDist - (param.compHoldDist - param.compMinDist);
  double effectiveResumeDist = effectiveHoldDist - (param.compHoldDist - param.compResumeDist);

  if (gap < effectiveMinDist)    _compTracking = false;   // too close — back off
  if (gap > effectiveResumeDist) _compTracking = true;    // safe again — track

  double new_z;
  if (_compTracking) {
    double vff  = _seaState.heaveValid ? _seaState.heaveVelPredComp : 0.0;
    double lead = (effectiveHoldDist - gap);
    new_z = _state.getZ() + 3.0*lead;

    // Two modes, so the position and velocity halves of the setpoint stop
    // fighting each other.
    //
    //   MATCHING  heaveVelPredComp > compVelMatchThld: the pad is climbing fast
    //             enough that matching its vertical rate is the point, so Vz is
    //             the pad rate, nothing else added. Signed, not |.|: the pad
    //             dropping away is not a case that needs matching.
    //   POSITION  otherwise: command position ONLY.
    //
    // NOTE the position-only branch sends NaN, not 0. They are NOT the same
    // thing — a velocity field of 0 is still a commanded setpoint ("hold zero
    // vertical velocity") that PX4 tracks and that therefore fights the
    // position target. NaN is what tells PX4 the axis is not velocity
    // commanded, which is exactly how velocity[0]/[1] are already sent in
    // bridge_node's POSITION branch. It propagates untouched: getVz() ->
    // twist.linear.z -> "velocity[2] = -twist.linear.z", and -NaN is NaN.
    _compVelMatching = vff > param.compVelMatchThld;
    _setPoint.setVz(vff);
    // if (_compVelMatching) {
    //   _setPoint.setVz(vff);
    // } else {
    //   _setPoint.setVz(NAN);
    // }

  } else {
    // Backing off: ascend (anchored, bounded) until the gap recovers.
    new_z = std::min(_setPoint.getZ() + param.asceVelocity, _state.getZ() + param.asceVelocity);
    _setPoint.setVz(0.0);
  }

  _setPoint.setZ(new_z);
  clampZSP();
  _setPoint.setType(MavState::POSITION);

  // Throttled to ~10 Hz — enough to read the descent rate without flooding at
  // the full loop rate. Prints what is needed to tell apart the three ways the
  // descent can stall:
  //   * commanded but not followed  -> cmdZ stays well below stateZ, stateVz ~0
  //   * not commanded               -> cmdZ converges onto stateZ (gap already
  //                                    at compHoldDist, or gap not updating)
  //   * clamped                     -> clamped flag set, cmdZ pinned at zMin
  {
    static int compLogTick = 0;
    if ((compLogTick++ % 5) == 0) {
      const double cmdZ = _setPoint.getZ();
      std::cout << "comp gap " << gap << "/" << effectiveHoldDist
                << " (base " << param.compHoldDist << " +heaveUpMax " << _heaveUpMax
                << " capped@" << param.compHeaveMarginMax << ")"
                << " minDist " << effectiveMinDist << " resumeDist " << effectiveResumeDist
                << " tracking " << _compTracking
                << " mode " << (_compTracking ? (_compVelMatching ? "VEL" : "POS") : "BACKOFF")
                << " vff " << (_seaState.heaveValid ? _seaState.heaveVelPredComp : 0.0)
                << "/" << param.compVelMatchThld
                << " cmdVz " << _setPoint.getVz()
                << " heaveVelPredComp " << _seaState.heaveVelPredComp
                << " | stateZ " << _state.getZ()
                << " stateVz " << _state.getVz()
                << " newZ " << new_z
                << " cmdZ " << cmdZ
                << " cmdMinusState " << (cmdZ - _state.getZ())
                << " clamped " << (std::fabs(cmdZ - new_z) > 1e-9)
                << " zMin " << param.zMin
                << " platZ " << _platformVision.getZ()
                << std::endl;
    }
  }
}

void Lander::clampZSP() {

    double temp;
    // GROUND TRUTH — kept commented for reference:
    // temp = common::clamp(_setPoint.getZ(), param.zMin, _platformState.getZ() + param.zMax);
    temp = common::clamp(_setPoint.getZ(), param.zMin, _platformVision.getZ() + param.zMax);
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
