//
// Created by andreanistico on 06/02/17.
//

#ifndef MOCAP2MAV_LANDER_H
#define MOCAP2MAV_LANDER_H

#include <deque>
#include <utility>
#include "StateMachine/include/Machine.h"
#include "common/MavState.h"
#include "common/SeaState.h"
#include "StatesClasses.hpp"
#include "common/common.h"
#include "miniPID/MiniPID.h"
#include "Parameters.h"

#define DEBUG
#define PI              3.141592653589
#define Kland           1
#define THRE            0.25
#define DRATE_MIN       0.1
#define DRATE_MAX       0.3
#define TMAX            1.6
#define TMIN            0.7
#define PLATFORM_OFFSET 0.0

// Touchdown-detector accel-estimate smoothing (smoothAccel()): clip each raw IMU
// sample to this magnitude [m/s^2] before it enters the moving average — no real
// drone/pad acceleration should exceed this, so a clipped value only ever means
// "a sensor glitch/dropout", not a real event.
#define kAccelEstimateClip   20.0
#define kAccelEstimateWindow 5

// Standard gravity [m/s^2], used to remove each accel source's own known
// constant gravity-reaction offset before comparing drone vs pad acceleration
// (see updateSignals()'s touchdown detector and SeaState.h for the two
// different raw conventions involved).
#define kGravityMs2 9.81

/**
 * @brief All the variables behind the landing GO/NO-GO decision, snapshotted every
 * tick in Lander::updateSignals() for logging/plotting (see getGoDiagnostics()).
 */
struct LandGoDiagnostics {
    // Inputs
    bool   spaValid       = false;
    bool   heaveValid     = false;
    double spaRollPred    = 0.0;
    double spaPitchPred   = 0.0;
    double rollRate       = 0.0;
    double pitchRate      = 0.0;
    double heavePos       = 0.0;
    double heaveVel       = 0.0;
    double heaveVelPred   = 0.0;

    // Limits applied to the inputs above (LAND_* params, logged so plots are
    // self-contained — no need to cross-reference the param file).
    double rollLimit      = 0.0;
    double pitchLimit     = 0.0;
    double rollRateLimit  = 0.0;
    double pitchRateLimit = 0.0;

    // Per-condition verdicts
    bool anglesOk      = false;
    bool ratesOk       = false;
    bool descFromPeak  = false;

    // Decision pipeline: instantaneous -> majority vote over a window -> latch
    bool   rawGo         = false;   // this tick's instantaneous decision
    double voteFraction  = 0.0;     // fraction of the vote window that was GO
    int    voteWindowSize = 0;      // number of samples in the vote window
    bool   votedGo        = false;  // majority vote over goVoteWindow
    double latchTimer     = 0.0;    // time since the latch last flipped
    bool   landingGo      = false;  // final latched GO/NO-GO (== getLandingGoSignal())

    // Touchdown detection (only evaluated in LAND — see updateSignals()). Based on
    // matching the drone's and pad's vertical acceleration (both from real IMU
    // sources, gravity-corrected to true kinematic accel), not relative velocity
    // (PX4's odometry velocity output is unreliable near/at touchdown).
    double touchAccelZ    = 0.0;    // drone: smoothed true vertical accel [m/s^2]
    double touchPadAccelZ = 0.0;    // pad: smoothed true vertical accel [m/s^2]
    bool   touchGroundContact = false;  // stage 1: drone/pad accel matched + xy velocity settled
    bool   touchConfirmed     = false;  // stage 2: contact + a recent positive impact spike
    bool   touchedDown        = false;  // latched final verdict (== getTouchedDown())
    // Timers driving the two stages above (seconds) — how close each is to tripping.
    double touchContactTimer   = 0.0;  // time accel-matched + xy-settled has held (-> touchContactHold)
    double touchConfirmTimer   = 0.0;  // time contact+recent-spike has held (-> touchConfirmHold)
    double touchTimeSinceSpike = 0.0;  // time since the last positive impact spike (vs touchImpactWindow)
};


/** @ingroup Automatic
 * @brief Planner of the Landing procedure
 * @details Such class implements the landing
 * on a moving platform using a state machine.
 *
 */
class Lander {

public:

    /**
     * @brief Constructor of the class.
     * @details It initializes the members of the class (errors, values of the PID,
     * initialization of the state machine and it links each state of this class with
     * the LandMachine class.
     */
    Lander();

    /**
     * @brief It allows to take information coming from vision system
     * (apriltag_vision_system topic)
     * @param VisionPose Pose of the Robot respect to the platform
     */
    void setVisionPose(const MavState VisionPose);

    /**
     * @brief It allows to take info coming from platform/pose topic
     * @param platformState Absolute pose of the platform respect to the world frame
     */

    void setPlatformState(const MavState platformState);

    /**
     * @brief It allows to take info coming from UltrasonicSensor/platform topic
     * @param UltraInfo Partial pose of the drone using Ultrasonic sensor
     */

    void setUltrasonicInfo(const MavState UltraInfo);

    /**
     * @brief It feeds the aggregated sea-state estimate (SPA prediction + heave)
     * used by the calm-sea landing gate.
     * @param seaState SPA/heave freshness, heave velocity and roll/pitch rates
     */

    void setSeaState(const SeaState seaState);

    /**
     * @brief It initialize the state Machine for the landing procedure
     * @details It builds the state machine linking properly the states.
     * It links members of the LandMachine class instance with the Lander ones;
     * just because more than one state machine can exist at the same time.
     */

    void initStateMachine();

    /**
     * @brief It allows to take the absolute pose of the drone, vision_position_estimate
     * @param pose Full pose of the drone respect to the world frame
     */

    void setState(MavState pose);

	/**
	 * @brief It allows to set the desired yaw during seaching phase
	 * @param desired yaw coming from task struct
	 */

	void setYaw(float yaw);

    /**
     * @brief It controls the actual state of the machine and of the drone
     * @details It calls updateSignals() and handle() methods
     */

    void handleMachine();

  	/**
       * @brief It loads the new machine updating the parameter list
     */

  	void loadMachine();

  	/**
       * @brief It prints state machine signals
     */

    void printSignals();

    /**
     * @brief It computes the horizontal and vertical error of the drone,
     * incrementing properly some boolean members(_holding, _lost, _centered)
     * which allows to set the next state.
     */

    void updateSignals();
    /**
     * @brief It sends a set of setpoints to the drone in order to position
     * @return The actual state of the machine
     */

    bool setDroneInFront();
    /**
     * @brief It gets the actual state of the machine
     * @return The actual state of the machine
     */
	
    int getActualMachineState();

    /**
     * @brief It gets the pose of the drone
     * @return The pose of the drone respect to the world frame
     */

    MavState getState();

    /**
     * @brief It returns the command to send to the motor
     * @return The setPoint position
     */

    MavState getCommand();

    /**
     * @brief Returns the Go/NoGo prediction horizon T = min(|vertErr|/descVel, cap)
     * @return Prediction horizon in seconds, updated every tick in updateSignals()
     */

    double getPredictionHorizon() const { return _predictionHorizon; }

    /**
     * @brief Snapshot of all the variables behind the landing GO/NO-GO decision,
     * updated every tick in updateSignals(). Published for plotting/debugging.
     */

    LandGoDiagnostics getGoDiagnostics() const { return _goDiag; }

    /**
     * @brief Touchdown detection: true once the drone has landed on the pad
     * (only evaluated in LAND — see updateSignals()). Latched: never resets on
     * its own, only initStateMachine() clears it.
     */

    bool getTouchedDown() const { return _touchedDown; }

    /**
     * @brief Consumes (returns and clears) the pending disarm-on-GO request,
     * set by CompState::handle() when param.landDisarmOnGo is enabled and the GO
     * signal arrives. Edge-triggered so the caller (AutomaticNode) sends the
     * disarm command exactly once.
     */

    bool consumeDisarmRequest() {
        bool req = _disarmRequest;
        _disarmRequest = false;
        return req;
    }

    /**
     * @brief The core function of the landing procedure. Mainly, it is
     * a switch based on the actual state.
     */

    void run();

    /**
     * Used to switch between vision system and ultrasonic sensor.
     */

    bool switchSensor;

  	/**
       * The following variables are used to implement an hovering action:
       */

  	/**
       * X hovering coordinate.
       */

  	double starting_x;

  	/**
       * Y hovering coordinate.
       */

  	double starting_y;

  	/**
       * Z hovering coordinate.
       */

  	double starting_z;

  	/**
       * To check if the state is set to hovering
       */

  	bool _hovering;

  	/**
       * To check if the machine has been reset
       */

  	bool _resetMachine;
	
	/**
	 * desired yaw
	 */

	float _yaw_des;


  	/**
       * To print signals every seconds
       */

  	int print;

  	double desc_velocity;

    // T = min(|verticalErr| / holdDescendStep, predHorizonMax) — updated every tick
    double _predictionHorizon;

    bool startFromBeginning;

    // perform inspection
     double time;
     double X0;
     double Y0;
     bool Inspection;
     bool First_insp;
     double RadiusInspection;
     double Period;

private:

    LandMachine  _machine;

    /**
     * The full state of the drone respect to the world frame
     */

    MavState _state;

    /**
     * The position setPoint to accomplish the landing
     */

    MavState _setPoint;

    /**
     * Platform pose respect to the world frame
     */

    MavState _platformState;

    /**
     * Relative pose between drone and platform using vision system
     */

    MavState _VisionPose;

    /**
     * Aggregated sea-state estimate (SPA prediction + heave) fed from the
     * guidance node and shared with the LandMachine for the calm-sea gate.
     */

    SeaState _seaState;

    /**
     * Relative z position and velocity of the drone respect to the platform
     */

    MavState _UltraInfo;

    /**
     * PID for the x axis used in the hold state filled in the
     * constructor with variables in params_automatic namespace
     */

    MiniPID  _holdPIDX;

    /**
     * PID for the y axis used in the hold state filled in the
     * constructor with variables in params_automatic namespace
     */

    MiniPID  _holdPIDY;

    // ── XY platform-position hold state (hold()) ─────────────────────────────
    // Low-pass-filtered platform XY setpoint and the previously commanded XY
    // setpoint (for rate limiting). Seeded on entry / after a vision dropout.
    double _platFiltX = 0.0;
    double _platFiltY = 0.0;
    double _prevSpX   = 0.0;
    double _prevSpY   = 0.0;
    bool   _spSeeded  = false;

    // COMP vertical mode this tick: true = velocity matching (pad moving),
    // false = pure position (pad quiet, Vz commanded 0). Logged so the mode
    // the drone was actually in is visible per tick.
    bool   _compVelMatching = false;

    // COMP standoff hysteresis: true = track compHeightBase, false = backing off
    // (ascending) because the gap dropped below compBackoffBelow.
    bool   _compTracking = true;

    // Landing GO/NO-GO window (computed each tick in updateSignals()).
    bool   _landingGo   = false;   // latched GO signal
    bool   _goLatched   = false;   // internal latch state
    double _goLatchTimer = 0.0;    // time [s] since the last GO<->NO-GO switch
    bool   _goLogPrev   = false;   // for edge-triggered logging

    // Set by CompState::handle() when param.landDisarmOnGo is enabled and the GO
    // signal arrives — consumed (and cleared) via consumeDisarmRequest().
    // True when M2M_MODE is 0 (sim), cached at construction — selects the
    // simulated vs hardware altitude sensor offsets. Same flag Automatic/Executioner use.
    // ── Platform state derived from VISION (not the ground-truth topic) ───────
    // _platformState comes from Landing_Site/pose, which in simulation is the
    // Gazebo plugin publishing the pose it just wrote onto the model — i.e.
    // simulator truth, not a measurement. Feeding that into the control path
    // makes sim behaviour unreachable on hardware, so the guidance now builds
    // its own platform estimate from what it can actually measure:
    //
    //   position  = drone state + vision offset   (vision is already rotated
    //               into world-aligned axes by the apriltag node's
    //               correctToPlatformFrame(), so this is a plain sum)
    //   z         = drone state + _verticalErr    (the offset-corrected,
    //               sensor-selected vertical, so it matches downDist exactly)
    //   velocity  = d(x, y)/dt, updated only on new apriltag frames over the
    //               real inter-frame interval, low-passed (padEstVelFilterTau)
    //               and clamped (padEstVelMax)
    //   yaw       = absolute heading published by the apriltag node
    //               (+ padEstYawOffset)
    MavState _platformVision;

    double   _platVisFiltVx = 0.0;   // low-passed platform Vx estimate [m/s]
    double   _platVisFiltVy = 0.0;
    double   _prevPadX      = 0.0;   // pad world XY at the previous frame
    double   _prevPadY      = 0.0;
    long int _prevVisStamp  = 0;     // timestamp [us] of the previous frame
    bool     _platVisSeeded = false;
    long int _platVelSamples = 0;    // frames folded into the velocity estimate (warm-up)

    // Highest platform Z seen so far this landing attempt (world/drone frame,
    // same convention as _platformVision.getZ()), updated every tick in
    // updatePlatformVision() and reset in initStateMachine(). Used instead of
    // the live/current platform Z for setDroneInFront()'s INIT standoff
    // target, so the initial approach altitude doesn't track the pad's
    // instantaneous heave and only ever gets safer (higher) as a better peak
    // is observed, never lower.
    double _platformMaxZ = 0.0;
    bool   _platformMaxZSeeded = false;

    /**
     * @brief Rebuilds _platformVision from the drone state + vision offset.
     * Called every tick from updateSignals(), after _verticalErr is computed.
     */
    void updatePlatformVision();

    bool   _envSim = false;

    bool   _disarmRequest = false;

    // Disarm mode re-sends the disarm for a short window after entering
    // LANDING_DONE: /fmu/in/vehicle_command is published best-effort, so a
    // single dropped command would otherwise leave the drone armed and holding
    // in the terminal state. Bounded rather than forever, so it cannot fight an
    // operator trying to re-arm afterwards.
    double _disarmRetryElapsed = 0.0;   // time [s] since entering LANDING_DONE
    double _disarmRetrySince   = 0.0;   // time [s] since the last re-send

    // Rolling window of the raw (instantaneous) GO decision, used to majority-vote
    // a smoothed decision before it feeds the latch — a single noisy sample must
    // not be able to flip the landing decision. Each entry is (dt, rawGo) for that
    // tick; the window covers the last goVoteWindow seconds.
    std::deque<std::pair<double,bool>> _goVoteHistory;

    // Snapshot of the GO decision variables, published for plotting/debugging.
    LandGoDiagnostics _goDiag;

    // ── Touchdown detection (LAND -> LANDING_DONE) ────────────────────────────
    // Staged hysteresis (contact -> confirm — see Parameters.h touch* fields for
    // the rationale). Both accel estimates (from real IMU sources, see SeaState.h)
    // are smoothed every tick regardless of state, so they're already warm and
    // settled by the time LAND starts — only the stage timers below are
    // LAND/per-attempt scoped.
    std::deque<double> _touchAccelBuf;     // small moving-average window for the drone accel estimate
    double _touchAccelZSmoothed = 0.0;     // latest smoothed drone true vertical accel [m/s^2]

    std::deque<double> _padAccelBuf;       // small moving-average window for the pad accel estimate
    double _padAccelZSmoothed = 0.0;       // latest smoothed pad true vertical accel [m/s^2]

    bool   _wasInLand = false;             // for detecting a fresh entry into LAND
    double _touchContactTimer = 0.0;       // time [s] drone/pad accel have matched + xy settled
    double _touchTimeSinceSpike = 1e9;     // time [s] since the last positive impact spike
    double _touchConfirmTimer = 0.0;       // time [s] contact+recent-spike has held
    bool   _touchedDown = false;           // latched final verdict — never resets on its own
    bool   _touchdownLogged = false;       // guards logTouchdownCsv() to fire once per attempt

    // Continuous time [s] compValid has held (HoldState::handle()) — the
    // HOLD -> COMP wait gate. Reset to 0 the instant compValid goes false, and
    // on any full state-machine reset (initStateMachine()).
    double _compValidTimer = 0.0;

    // Continuous time [s] the COMP -> LAND transition condition has held
    // (CompState::handle()) — the COMP -> LAND wait gate. Reset to 0 the
    // instant the condition goes false, and on any full state-machine reset
    // (initStateMachine()).
    double _landValidTimer = 0.0;

    // Continuous time [s] setDroneInFront()'s standoff-position error has
    // stayed under threshold — the INIT approach settle gate. Reset to 0 the
    // instant the drone drifts back out of tolerance, and on any full
    // state-machine reset (initStateMachine()).
    double _approachSettleTimer = 0.0;

    // Rolling window of recent heave velocity, (dt-since-previous-sample, value)
    // pairs pushed every tick in updateSignals() (same pushPredHistory helper
    // and real-elapsed-time pruning as the SPA/heave prediction history
    // above) -- but the value stored is _seaState.heaveVel clamped to >= 0,
    // since downward heave velocity is our friend (the pad moving away from
    // the drone needs no extra standoff margin) and must not affect the peak
    // computed from this window at all.
    std::deque<std::pair<double, double>> _heaveUpSamples;

    // Peak (max) of _heaveUpSamples' values over the window (0.0 when the
    // pad hasn't moved up at all recently, e.g. a genuinely calm or
    // currently-descending sea) -- feeds comp()'s extra standoff margin:
    // calm seas let the drone hold closer; a recent upward excursion pushes
    // the standoff back out by that excursion's peak speed, and the margin
    // relaxes again once that peak ages out of the window. Recomputed
    // alongside _heaveUpSamples in updateSignals().
    double _heaveUpMax = 0.0;

    // Throttles setDroneInFront()'s diagnostic log to ~1 Hz instead of the
    // full control-loop rate.
    double _approachLogTimer = 0.0;


     bool _slopeActive = true;
    // Rolling history of the SPA/heave PREDICTIONS (not the raw estimates),
    // tracked unconditionally every tick regardless of state, so
    // logTouchdownCsv() can look up "what the predictor said logPredLookback
    // seconds before touchdown" for comparison against the raw value it logs at
    // touchdown itself. Each entry is (dt, value) for that tick.
    std::deque<std::pair<double,double>> _predHistRoll;
    std::deque<std::pair<double,double>> _predHistPitch;
    std::deque<std::pair<double,double>> _predHistHeavePos;
    std::deque<std::pair<double,double>> _predHistHeaveVel;

private:

	/**
	 * Load parameters
	 */
	Parameters param;

    /**
     *  Init state of the machine linked to the hold state
     */

    InitState       _initS;

    /**
     * Holding state linked to the comp, asce and desc states
     */

    HoldState       _holdS;

    /**
     * Ascending state linked to the comp, hold and land states
     */

    AsceState       _asceS;

    /**
     * Descending state linked to the hold state
     */

    DescState       _descS;

    /**
     * RToLand state linked is between the comp and land states
     */

    RToLandState    _rtolS;

    /**
     * Compensation state linked to the asce, land and hold states
     */

    CompState       _compS;

    /**
     * Landing state is the final one, linked to the comp and asce states
     */

    LandState       _landS;

    /**
     * Terminal state reached once touchdown is detected. No transitions out.
     */

    LandingDoneState _landingDoneS;

    InspectionState _inspeS;    //its constructor set its ID to INSPE
  	/**
       * state for hovering action
       */

  	HoveringState _hoveringS;

    /**
     * Actual state of the machine state
     */

    int _actualState;

    /**
     * Previous state of the machine state
     */

    int _prevState;

    /**
     * Differential time
     */

    double _dt;

    /**
     * Actual time
     */

    uint64_t _actualTime;

    /**
     * Previous time
     */

    uint64_t _prevTime;

    /**
     * Horizontal error between drone and platform using vision system
     */

    double _horizontaErr;

    /**
     * Vertical error between drone and platform using vision system
     * and ultrasonic sensor
     */

    double _verticalErr;

    /**
     * Threshold value for the horizontal error to allow the hold state
     */

    double _tauHold;

    /**
     * Threshold value for the horizontal error to check if the platform is lost
     */

    double _tauLost;

    /**
     * It keeps how many times the _horizontaErr is under a given threshold (_tauLost)
     */

    int    _NHold;

    /**
     * It keeps how many times the _horizontaErr is above a given threshold (_tauLost)
     */

    int    _NLost;

    /**
     * It keeps how many times the drone is on place to accomplish the landing
     */

    int    _NComp;

    /**
     * It checks if the drone is on place respect to the platform
     */

    bool _centered;

    /**
     * It checks if the drone is holding (horizontal error) the platform
     */

    bool _holding;

    /**
     * It checks if the drone has lost the platform
     */

    bool _lost;    

    /**
     * It checks if the drone is aligned with the platform and ready to start the landing
     */

    bool _readyToLand;

    /**
     * Integral error of the drone respect to the platform
     */

    Eigen::Vector3d _err_int;

    /**
     * Actual error of the drone respect to the platform
     */

    Eigen::Vector3d _err;

    /**
     * Previous error of the drone respect to the platform
     */

    Eigen::Vector3d _err_prev;

    // ── Time-parameterized INIT->HOLD slope (hold(), _slopeActive) ───────────
    // Replaces the old horizontal-convergence-fraction ("dot/norm_sq") slope:
    // that law made the Z command a direct function of instantaneous
    // horizontal vision error, so any stall/wobble in XY convergence stalled
    // or reversed Z too (see the session history this superseded).
    //
    // This one is driven by elapsed TIME and defined in the PAD's frame:
    // captured once, at slope start (setDroneInFront()'s `ready` block) --
    //   rel0 = drone position - pad position at that instant (t=0)
    // -- and every tick thereafter, with s = clamp(elapsed/slopeDuration, 0, 1):
    //   rel(t) = (1-s)*rel0 + s*(0, 0, holdHeight)
    //   cmd(t) = padLive(t) + rel(t)
    // i.e. at every moment the drone is commanded to a specific position
    // RELATIVE to the pad, transformed to world using the pad's live
    // position -- so the whole path rides along with a moving/heaving pad.
    // At s=1 the slope hands off to normal hold() tracking (trackXY() + live
    // Z) by clearing _slopeActive.
    //
    // If the slope is interrupted (state machine resets back through INIT),
    // setDroneInFront()'s `ready` block re-captures rel0/elapsed fresh --
    // no partial resume, the slope restarts from t=0.
    double _slopeStartRelX = 0.0;
    double _slopeStartRelY = 0.0;
    double _slopeStartRelZ = 0.0;
    double _slopeElapsed = 0.0;
    // Integral of the slope's XY tracking error (slopeXYKi), reset at slope start.
    double _slopeIntX = 0.0;
    double _slopeIntY = 0.0;

    /**
     * @brief Logic and signals helpers
     * @details It updates the integral error and
     * it checks if is in a defined interval
     */

    void updateIntegrals();

    /**
     * @brief Logic and signals helpers
     * @details It resets the integral error
     */

    void resetIntegrals();

    /**
     * @brief Logic and signals helpers
     * @details It computes the _dt based on
     * the _actualTime
     */

    void managetime();

    /**
     * @brief It clamps the setPoint on z axis in a specific interval
     */

    void clampZSP();

    /**
     * @brief It resets position setpoint to actual position
     */

    void resetSetPoint();

    /**
     * @brief Initialize stuff here
     * @details It calls resetSetPoint() method
     */

    // Align yaw with platform
    void align();

    void init();

    /**
     * @brief Tracking logic defined here
     * @details  Tracking is performed by POSITION control, issuing the position setpoint
     * in order to achieve the desired velocity calculated by:
     *
     * Vdes = K * ep + Vplat
     * or
     * Psp = Pplat + K * Vplat
     *
     * where:
     *
     * Vdes = desired velocity, K = proportional gain, ep = position error,
     *
     * Vplat = paltform velocity
     */

    void hold();

    /**
     * @brief Drives the XY setpoint toward the platform. Configurable via
     * param.xyControlMode: direct (absolute EMA-filtered/rate-limited platform-
     * position hold) or pid (MiniPID on the horizontal vision error). Called
     * from both hold() and comp() so the chosen mode applies in either state.
     */

    void trackXY();

    /**
     * @brief Go up every time of 10 cm in case we miss, reset PID values
     */

    void asce();

    /**
     * @brief Get closer to the target, every time of 10 cm
     */

    void desc();

    /**
     * @brief Compensate altitude oscillation before landing
     * @details It chooses between vision system or ultrasonic sensor based on
     * the actual height and then it calculates desired vertical velocity
     *  in order to compensate oscillations.
     */

    void comp();

    /**
     * @brief Aggressive land procedure
     */

    void land();

    /**
     * @brief Hovering action
     */

	void hovering();

  	// circular trajectory for inspection. It tries with a circular
  	// trajectory to find the platform
  	void inspection();
   	void initInspection(); //initialize variables to perform inspection

    /**
     * @brief Clips a raw IMU acceleration sample to a sane range and smooths it
     * with a moving-average window. A single sensor glitch/dropout would
     * otherwise dominate the short window and read out as a wildly oversized
     * "acceleration" — a clipped sample only ever means "a glitch", never a real
     * reading. Used for both the drone's and the pad's vertical-acceleration
     * estimate in updateSignals().
     * @param rawSample Current raw (already gravity-corrected) accel sample.
     * @param buf Moving-average window (updated in place).
     */

    double smoothAccel(double rawSample, std::deque<double> &buf);

    /**
     * @brief Dumps a one-row CSV snapshot of the touchdown-moment stats (x/y
     * error, roll/pitch, heave, touchdown-detector state, ...) into the
     * package's installed share/results directory, named by the wall-clock time
     * it's created so repeated runs never clash. Called once, right when
     * touchdown is confirmed (see updateSignals()).
     */

    void logTouchdownCsv();

    /**
     * @brief Pushes (dt, value) onto a rolling history buffer, trimming entries
     * older than maxAge seconds — used to keep logPredLookback seconds (plus
     * margin) of the SPA/heave prediction signals available for
     * logTouchdownCsv() to look back into.
     */

    void pushPredHistory(std::deque<std::pair<double,double>> &buf, double value, double dt, double maxAge);

    /**
     * @brief Looks up the value from `lookback` seconds before the most recent
     * sample in `buf` (walking backward from the newest entry). Returns the
     * oldest available sample if the buffer doesn't extend that far back yet.
     * @param achievedLookback Set to the actual age [s] of the returned sample.
     */

    double lookupPredHistory(const std::deque<std::pair<double,double>> &buf, double lookback, double &achievedLookback) const;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

#endif //MOCAP2MAV_LANDER_H
