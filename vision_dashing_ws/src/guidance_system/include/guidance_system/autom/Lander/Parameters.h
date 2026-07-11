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

    //init value of radius for inspection
    double inspeRadius;

    //linear velocity of the drone during inspection
    double inspeLinVel;

    //roll upper limit to land
    double RollThreshold;

    //pitch upper limit to land
    double PitchThreshold;

    //Number of consecutive frames in which tracking is considered valid
    int    NFramesHold;

    //Number of consecutive frames in which tracking is considered not valid
    int    NFramesLost;

    //Number of consecutive frames in which tracking is considered valid
    //and robot is ready for compensation
    int    NFramesComp;

    //Platform dimension
    double platformLenght;

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

    //switch off the motors
    double land_threshold;
    //Max altitude for landing procedure
    double zMax;

    //Minimum altitude for landing procedure (before compensating, this value should be above the maximum platform altitude)
    double zMin;

    //Proportional gain times platform velocity
    double KpHoldV;

    //Proportional gain times horizontal error
    double KpHold; // 0.5

    //Differential gain times horizontal error
    double KdHold;

    //Integral gain times integral horizontal error
    double KiHold; //0.1

    //Proportional gain for velocity tracking
    double KPCompV;

    //Integral clamping values
    double maxIntValue;
    double minIntValue;

    //Max total PID output
    double maxOutput;

    // Distance from the center of the platform where to initialize landing

    double distanceFromCenter;

    // Drone altitude when initializing the landing

    double prelandAltitude;

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

    // ── XY platform-position hold (hold()) ───────────────────────────────────
    // EMA smoothing factor [0..1) for the platform XY setpoint. Higher = smoother
    // / slower (rejects vision jitter and the delay-driven limit cycle).
    double spFilterAlpha;
    // Max speed [m/s] the XY setpoint is allowed to move (rate limit).
    double spMaxVelXY;

    // ── Heave-predictive landing ─────────────────────────────────────────────
    // Drone vertical reaction lag [s]: how far ahead to lead the pad using the
    // heave prediction. Should match the heave_estimator's prediction_horizon_s.
    // Fixed for now; estimated online in a later step.
    double reactionLag;

    // Drone->pad vertical distance [m] to descend to and hold before starting
    // the compensated / aggressive landing (the "standoff"). Also the gap at
    // which HOLD hands off to COMP.
    double standoffDistance;

    // ── COMP standoff (comp()) ───────────────────────────────────────────────
    // Drone->pad distance [m] COMP holds while compensating (below standoffDistance).
    double compHoldDist;
    // If the gap drops below this [m], COMP stops tracking and backs off (ascends).
    double compMinDist;
    // COMP resumes tracking once the gap climbs back above this [m] (hysteresis).
    double compResumeDist;

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

    // ── Touchdown detection (LAND -> LANDING_DONE) ────────────────────────────
    // Adapted from PX4's own multicopter land detector (ground_contact / maybe_landed
    // / landed staged-hysteresis state machine), corrected for a moving/heaving pad:
    // velocity is checked RELATIVE to the pad (drone vz - pad heaveVel), not in the
    // world frame, since a drone resting on the pad still moves with it.
    //
    // Stage 1 (contact): |relative vertical velocity| and horizontal velocity below
    // this [m/s], sustained touchContactTime.
    double touchVelRelMax;
    // Stage 2 (confirm): stage 1 held AND an impact deceleration spike (vertical
    // acceleration magnitude, finite-differenced from vz) above this [m/s^2] was
    // seen within touchAccelSpikeWindow seconds — sustained touchConfirmTime.
    double touchAccelSpike;
    double touchAccelSpikeWindow;
    double touchContactTime;
    double touchConfirmTime;

};


#endif //MOCAP2MAV_PARAMETERS_H

    
