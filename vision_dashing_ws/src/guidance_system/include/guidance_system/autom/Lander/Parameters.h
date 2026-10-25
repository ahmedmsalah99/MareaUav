//
// Created by andreanistico on 15/12/17.
//



#ifndef MOCAP2MAV_PARAMETERS_H
#define MOCAP2MAV_PARAMETERS_H
//
// Created by andreanistico on 24/10/17.
//

#include <string>
#include <iostream>
#include <fstream>
#include <algorithm>


class Parameters {

public:
    Parameters();

    void loadConfigFile(const char* config_file);

private:

    bool loaded;

    void loadParamFile(const char* config_file);

public:

    //PARAMETERS FIELDS
    
	//if 1 ==>inspection, if 0 it implements an hovering action
	bool search;

	//if 1 ==>disarm immediately on the GO signal in COMP and jump straight to
	//LANDING_DONE, skipping LAND's descent and touchdown detection entirely
	bool disarmOnGo;

    //init value of radius for inspection
    double inspeRadius;

    //linear velocity of the drone during inspection
    double inspeLinVel;

    //Number of consecutive frames in which tracking is considered valid
    int    NFramesHold;

    //Number of consecutive frames in which tracking is considered not valid
    int    NFramesLost;

    //Number of consecutive frames in which tracking is considered valid
    //and robot is ready for compensation
    int    NFramesComp;

    //horizontal error allowed during holding phase
    double hold_threshold;

    //horizontal error allowed during compensation phase
    double comp_threshold;

    //vertical error allowed
    double lost_threshold;

    // Drone must stay at or above platformZ + z_margin when not actively descending.
    double z_margin;

    // Minimum (tightest) values of the above thresholds, applied at z=0.
    // At any relative z the threshold is linearly interpolated between
    // the _min value (z=0) and the max value (z=cone_z_ref).
    double hold_threshold_min;
    double comp_threshold_min;
    double lost_threshold_min;

    // Relative Z at which thresholds reach their maximum (cone base radius).
    double cone_z_ref;

    //Max altitude for landing procedure
    double zMax;

    //Minimum altitude for landing procedure (before compensating, this value should be above the maximum platform altitude)
    double zMin;

    //Proportional gain times horizontal error
    double KpHold; // 0.5

    //Differential gain times horizontal error
    double KdHold;

    //Integral gain times integral horizontal error
    double KiHold; //0.1

    //Integral clamping values
    double maxIntValue;
    double minIntValue;

    //Max total PID output
    double maxOutput;

    // Distance from the center of the platform where to initialize landing

    double distanceFromCenter;

    // Fixed absolute altitude used by inspection()/searching mode. NOT used by
    // setDroneInFront()'s INIT standoff target -- that Z is now computed as
    // platformZ + standoffDistance + gain (see Lander::setDroneInFront()),
    // not this constant.
    double prelandAltitude;

    // setDroneInFront()'s standoff-position error (X and Y) must stay under
    // initApproachPosThld CONTINUOUSLY for this long [s] before INIT
    // releases into the landing state machine — same continuous-timer
    // pattern as compWaitTime/compToLandWaitTime.
    double initApproachWaitTime;

    // Max X/Y position error [m] from the standoff point for
    // setDroneInFront() to consider the drone "close enough" — a coarse
    // initial-approach gate, not final landing precision (that's
    // landXYErrThld). Against a moving platform this can never fully reach
    // 0 (the bridge's own position-setpoint slew limit bounds how fast the
    // gap can close), so keep this looser than landXYErrThld.
    double initApproachPosThld;

    // Max Z position error [m] from the standoff point for
    // setDroneInFront() to consider the drone "close enough" in altitude.
    // Kept separate from (and tighter than) initApproachPosThld because
    // the slope's point0 (see _slopeStartRelZ) is captured from state.getZ()
    // at slope-start time -- a loose Z tolerance here seeds the slope's
    // starting point with more residual error than intended.
    double initApproachZPosThld;

    // Total duration [s] of the time-parameterized INIT->HOLD slope (see
    // hold(), _slopeActive, _slopeStartRelX/Y/Z, _slopeElapsed). The commanded
    // pad-relative position interpolates from the drone's offset to the pad
    // at slope-start (t=0) to (0, 0, standoffDistance) at t=slopeDuration,
    // and is placed in world using the pad's live position. Tune to the
    // achievable descent/closing rate for the actual standoff distance --
    // too short demands an unrealistic velocity from the position
    // controller, too long leaves the drone loitering above the pad longer
    // than necessary before COMP can take over.
    double slopeDuration;

    // Integral gain [1/s] and clamp [m] on the slope's XY tracking error
    // (commanded slope position - drone position). Removes the steady
    // trailing offset PX4 settles into behind a moving command without
    // differentiating anything; reset at every slope start. 0 disables it.
    double slopeKi;
    double slopeIntMax;

    // Descent velocity (m per tick) applied in desc()
    double descVelocity;

    // Ascent velocity (m per tick) applied in asce()
    double asceVelocity;

    // ── Calm-sea gate (safeInitialLanding) ───────────────────────────────────
    // Max platform roll angle [rad] (estimation) allowed for the sea to be "calm".
    double rollLimit;
    // Max platform pitch angle [rad] (estimation) allowed for the sea to be "calm".
    double pitchLimit;
    // Max platform heave velocity [m/s] allowed for the sea to be "calm".
    double heaveVelLimit;
    // Max platform roll rate [rad/s] allowed for the sea to be "calm".
    double rollRateLimit;
    // Max platform pitch rate [rad/s] allowed for the sea to be "calm".
    double pitchRateLimit;
    // Time [s] the calm conditions must hold continuously before landing is allowed.
    double calmDuration;

    // ── XY platform-position hold (trackXY(), used by hold() and comp()) ─────
    // Which control law drives the XY setpoint:
    //   0 = direct  — absolute platform-position hold: EMA-filtered + rate-
    //       limited target at the platform's estimated world position.
    //   1 = pid     — MiniPID (KpHold/KiHold/KdHold) driving the horizontal
    //       vision error to zero, output added onto the drone's current position.
    int xyControlMode;
    // EMA smoothing factor [0..1) for the platform XY setpoint. Higher = smoother
    // / slower (rejects vision jitter and the delay-driven limit cycle). Direct mode only.
    double spFilterAlpha;
    // Max speed [m/s] the XY setpoint is allowed to move (rate limit). Direct mode only.
    double spMaxVelXY;

    // ── Heave-predictive landing ─────────────────────────────────────────────
    // Drone vertical reaction lag [s]: how far ahead to lead the pad using the
    // heave prediction. Should match the heave_estimator's prediction_horizon_s.
    // Fixed for now; estimated online in a later step.
    double reactionLag;

    // Drone->pad vertical distance [m] to descend to and hold before starting
    // the compensated / aggressive landing (the "standoff"). Also the height
    // hold()'s INIT slope glides down to (in place of a separately-configured
    // z_margin), so the slope hands off to the same height the HOLD state
    // machine's descend/COMP thresholds (standoffCompThld/standoffDescThld
    // below) are already built around. Nominal reference distance for those
    // two thresholds, which are what HOLD/COMP actually gate transitions on.
    double standoffDistance;

    // Hysteresis band around standoffDistance for the HOLD <-> DESC/COMP/ASCE
    // transitions (HoldState::handle(), CompState::handle()). Against a
    // heaving pad, downDist crosses a single threshold every heave cycle;
    // splitting it into an inner (standoffCompThld) and outer
    // (standoffDescThld) threshold gives a dead band in between where HOLD
    // just holds level instead of flip-flopping DESC/ASCE every crossing,
    // and where compValidTimer can actually accumulate compWaitTime seconds
    // of continuous validity instead of resetting every crossing.
    //   downDist > standoffDescThld  -> descend (HOLD's descValid; also
    //                                   where COMP exits back to HOLD)
    //   downDist < standoffCompThld  -> eligible for COMP (once sustained +
    //                                   safe) / ascend if not yet cleared
    //   in between                   -> just hold level
    double standoffCompThld;
    double standoffDescThld;

    // ── COMP standoff (comp()) ───────────────────────────────────────────────
    // Drone->pad distance [m] COMP holds while compensating (below standoffDistance).
    double compHoldDist;
    // Pad vertical speed [m/s] above which comp() switches to VELOCITY MATCHING.
    // Compared SIGNED against heaveVelPredComp (up-positive), so only a pad
    // climbing faster than this engages matching. Below it comp() commands
    // position only, with Vz sent as NaN so PX4 does not velocity-control the
    // axis at all.
    double compVelMatchThld;

    // Window [s] over which comp() tracks the PEAK recent upward heave
    // velocity (see Lander::_heaveUpSamples/_heaveUpMax) to grow the
    // standoff distance in rougher seas and let it shrink back once that
    // peak ages out of the window. Added directly (no gain) to compHoldDist
    // -- see comp().
    double compHeaveMarginWindow;

    // Cap [m] on the TOTAL effective COMP standoff (compHoldDist +
    // _heaveUpMax), not on _heaveUpMax's contribution in isolation -- an
    // unbounded standoff would back the drone arbitrarily far off in a
    // sufficiently rough sea, and capping the total is what actually bounds
    // how far out COMP will ever hold, regardless of what compHoldDist
    // itself is set to.
    double compHeaveMarginMax;

    // Cap [m] on the TOTAL effective landHeightThld's gate in
    // CompState::handle() (see heightOk there) -- landHeightThld +
    // _heaveUpMax, capped, mirroring compHeaveMarginMax above. Uses the SAME
    // _heaveUpMax value comp() uses for its own standoff, so the landing
    // height gate widens in step with the (capped) standoff instead of
    // staying fixed while the standoff grows out from under it, which would
    // otherwise mean COMP never gets close enough to satisfy heightOk and
    // the drone never lands. Capped separately (and smaller, by default)
    // from compHeaveMarginMax: this gate is the LAST check before
    // committing to land, so it should stay tighter than the mid-COMP
    // standoff cap.
    double landHeightMarginMax;

    // If the gap drops below this [m], COMP stops tracking and backs off (ascends).
    double compMinDist;
    // COMP resumes tracking once the gap climbs back above this [m] (hysteresis).
    double compResumeDist;

    // ── HOLD -> COMP wait (HoldState::handle()) ───────────────────────────────
    // compValid must hold CONTINUOUSLY for this long [s] before HOLD actually
    // transitions toward COMP (via R2LA). The timer resets to 0 the instant
    // compValid goes false even momentarily (continuous, not cumulative time),
    // and on any full state-machine reset back to INIT.
    double compWaitTime;

    // ── COMP -> LAND wait (CompState::handle()) ───────────────────────────────
    // The COMP->LAND transition condition must hold CONTINUOUSLY for this long
    // [s] before COMP actually transitions to LAND. Same continuous-timer
    // pattern as compWaitTime, but a distinct parameter/timer.
    double compToLandWaitTime;

    // COMP -> LAND also requires the X and Y position errors (not just the
    // combined horizontal norm) to each individually be under this [m].
    double landXYErrThld;

    // COMP -> LAND also requires the drone-to-pad vertical gap (downDist) to
    // be under this [m] — i.e. already close enough in height before
    // committing to landing.
    double landHeightThld;

    // ── Landing GO/NO-GO window (aggressive-landing trigger) ─────────────────
    // These are the LANDING limits (distinct from the calm-sea limits above).
    // Max SPA predicted roll/pitch [rad] for a GO.
    double landRollLimit;
    double landPitchLimit;
    // Max roll/pitch rate [rad/s] for a GO.
    double landRollRateLimit;
    double landPitchRateLimit;
    // Latch time [s]: after switching GO<->NO-GO, hold that decision at least this
    // long before accepting the opposite (debounce, 0.25 s).
    double landLatchTime;

    // Majority-vote window [s] applied to the raw (instantaneous) GO decision
    // before it feeds the latch, so a single noisy sample can't flip the landing
    // decision — smooths/robustifies the discrete signal the latch reacts to.
    double landVoteWindow;
    // Fraction (0..1] of the vote window that must be GO for the vote to be GO.
    double landVoteThreshold;

    // Setpoint lead [m] for the committed (aggressive) descent in land().
    double landDescVel;

    // Low-pass time constant [s] for the vision-derived platform velocity
    // estimate (updated once per apriltag frame, so frame-rate independent).
    // The boat moves slowly and steadily, so this can be long: the estimate
    // differentiates vision, which amplifies jitter, and it now feeds PX4
    // directly as the slope's XY velocity feed-forward.
    double platVisionVelTau;

    // Clamp [m/s] on the magnitude of that estimate -- anything above a
    // plausible boat speed is treated as vision noise.
    double platVisionVelMax;

    // ── Altitude sensor offsets [m] ───────────────────────────────────────────
    // Added to the raw reading of each altitude source to give the drone->pad
    // distance (downDist). All four push in the SAME direction: a larger offset
    // means the drone is reported as FURTHER from the pad for the same raw
    // reading. (Stored as a positive magnitude and added, where the original
    // sonar constant was negative and subtracted — same result, but it lets all
    // four read consistently.)
    //
    // SIM and HW are ALTERNATIVES, not terms to be summed: exactly one pair is
    // used per run, chosen by the M2M_SIM environment variable (the same flag
    // Automatic and Executioner already switch on). The simulated camera and
    // sonar sit at different places relative to the drone reference point than
    // the real ones do, so each environment needs its own pair.
    //
    // NOTE the camera and sonar values must be referenced to the SAME plane, or
    // downDist steps at the switchSensor handover — which shows up as a jump in
    // both the landing height gate and comp()'s tracking error.
    double cameraOffsetSim;
    double cameraOffsetHw;
    double sonarOffsetSim;
    double sonarOffsetHw;

    // Max UPWARD platform heave velocity [m/s] allowed to commit to landing.
    // One-sided on purpose: the pad may descend as fast as it likes (the gap
    // only opens), but committing while it is rushing UP at the drone is what
    // slams the pad into it. Compared against the PREDICTED heave velocity
    // (up-positive), so it is sign-sensitive — unlike the calm-sea gate's
    // heaveVelLimit, which is a symmetric fabs() limit.
    double landHeaveUpVelLimit;

    // ── Touchdown detection (LAND -> LANDING_DONE) ────────────────────────────
    // Adapted from PX4's own multicopter land detector (staged hysteresis:
    // contact then confirm). NOT based on relative vertical velocity (drone vz -
    // pad heaveVel) — PX4's odometry/EKF velocity output degrades once the drone
    // stops moving smoothly (near/at touchdown), so "velocity ~= 0 relative to
    // the pad" is unreliable exactly when it matters most. Instead both
    // conditions below are driven from real IMU accelerations (pad: imu/data:
    // drone: — see SeaState.h), not derivatives of
    // velocity, which stays meaningful even when the raw velocity is noisy.
    //
    // Stage 1 (contact): |drone vertical accel - pad vertical accel| (both
    // converted to true kinematic acceleration — see updateSignals()) below
    // touchAccelMatchTol, AND horizontal velocity below touchVelRelMax,
    // sustained touchContactTime.
    double touchAccelMatchTol;
    // Max horizontal (x/y) velocity [m/s] for stage 1.
    double touchVelRelMax;
    double touchContactTime;
    // Stage 2 (confirm): stage 1 held AND a POSITIVE impact-deceleration spike on
    // the drone's own vertical acceleration (touching down arrests the descent,
    // producing a brief positive acceleration bump in the up-positive
    // convention) exceeded touchAccelSpike within the last touchAccelSpikeWindow
    // seconds — sustained touchConfirmTime.
    double touchAccelSpike;
    double touchAccelSpikeWindow;
    double touchConfirmTime;

    // How far back [s] logTouchdownCsv() looks up the SPA/heave PREDICTIONS that
    // were current before touchdown, so they can be compared against the actual
    // raw roll/pitch/heave logged at touchdown itself — i.e. validate "what the
    // predictor said touchPredLookback seconds ago" against "what really
    // happened", rather than the (much less useful) prediction made at the
    // touchdown instant itself.
    double touchPredLookback;

};


#endif //MOCAP2MAV_PARAMETERS_H

    
