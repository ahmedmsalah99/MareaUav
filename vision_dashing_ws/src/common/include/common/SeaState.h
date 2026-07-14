#ifndef MOCAP2MAV_SEASTATE_H
#define MOCAP2MAV_SEASTATE_H

/**
 * @brief Aggregated sea-state/sensor feed for the landing state machine.
 * @details Populated by the guidance node from the SPA predictor
 * (spa/estimate, spa/prediction), the heave estimator (heave/position,
 * heave/velocity), the pad's raw IMU (imu/data) and the drone's own PX4
 * acceleration estimate . Used by
 * LandMachine::safeInitialLanding() to decide whether the sea is calm enough
 * to begin the landing, and by Lander's touchdown detector (accel matching).
 */
struct SeaState {

    // SPA prediction pipeline is producing fresh output (not outdated).
    bool spaValid = false;

    // Heave estimator is producing fresh output (not outdated).
    bool heaveValid = false;

    // Heave vertical velocity of the platform [m/s].
    double heaveVel = 0.0;

    // Heave vertical position of the platform [m].
    double heavePos = 0.0;

    // Forward-predicted heave (heave_estimator's prediction_horizon_s ahead, T =
    // l/v to the platform): where the pad will be, used by the landing GO/NO-GO
    // decision (see Lander::updateSignals()).
    double heaveVelPred = 0.0;
    double heavePosPred = 0.0;

    // Second, INDEPENDENT forward-predicted heave (heave_estimator's
    // control_comm_delay_s ahead — the drone's own control+communication
    // latency, not the time-to-platform above): used by the compensation-phase
    // vertical tracker (Lander::comp()) to lead the pad by "how far ahead the
    // command actually lands" rather than by distance-to-platform.
    double heaveVelPredComp = 0.0;
    double heavePosPredComp = 0.0;

    // Roll/pitch angular rates [rad/s], finite-differenced from spa/estimate
    // and smoothed with a moving average.
    double rollRate = 0.0;
    double pitchRate = 0.0;

    // Latest SPA roll/pitch estimate [rad].
    double roll = 0.0;
    double pitch = 0.0;

    // SPA predicted roll/pitch [rad] (spa/prediction, prediction_horizon ahead)
    // — used by the landing GO/NO-GO window.
    double spaRollPred = 0.0;
    double spaPitchPred = 0.0;

    // Control-loop period [s] measured by the guidance node. Used by the
    // calm-sea gate to accumulate its "sea has been calm for N seconds" timer,
    // so the timing does not depend on the Lander's internal dt bookkeeping.
    double dt = 0.0;

    // ── Raw IMU accelerations, for the touchdown detector's accel-matching test ──
    // Pad: raw imu/data linear_acceleration.z, whatever sign/gravity convention
    // that source uses (see the landingplat plugin — it happens to publish
    // az_true - g). Drone: sign-flipped
    // from PX4's FRD (Z down) to an up-positive convention (still includes the
    // real +g gravity reaction of a physical accelerometer). Both are raw —
    // Lander::updateSignals() removes each convention's known additive gravity
    // constant before comparing them.
    double padAccelZ = 0.0;
    bool   padImuValid = false;
    double droneAccelZ = 0.0;
    bool   droneAccelValid = false;
};

#endif //MOCAP2MAV_SEASTATE_H
