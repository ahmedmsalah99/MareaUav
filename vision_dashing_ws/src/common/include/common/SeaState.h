#ifndef MOCAP2MAV_SEASTATE_H
#define MOCAP2MAV_SEASTATE_H

/**
 * @brief Aggregated sea-state estimate fed to the landing state machine.
 * @details Populated by the guidance node from the SPA predictor
 * (spa/estimate, spa/prediction) and the heave estimator (heave/position,
 * heave/velocity). Used by LandMachine::safeInitialLanding() to decide whether
 * the sea is calm enough to begin the landing.
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

    // Roll/pitch angular rates [rad/s], finite-differenced from spa/estimate
    // and smoothed with a moving average.
    double rollRate = 0.0;
    double pitchRate = 0.0;

    // Latest SPA roll/pitch estimate [rad].
    double roll = 0.0;
    double pitch = 0.0;

    // Control-loop period [s] measured by the guidance node. Used by the
    // calm-sea gate to accumulate its "sea has been calm for N seconds" timer,
    // so the timing does not depend on the Lander's internal dt bookkeeping.
    double dt = 0.0;
};

#endif //MOCAP2MAV_SEASTATE_H
