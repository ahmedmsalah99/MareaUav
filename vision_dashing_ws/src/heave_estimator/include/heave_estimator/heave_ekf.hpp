#pragma once

#include <vector>
#include <Eigen/Dense>
#include "heave_estimator/fft_identifier.hpp"

namespace heave_estimator {

// Extended Kalman Filter for heave estimation (Küchler et al., IFAC 2011).
//
// State per mode j (3 states): x_j = [z_j, zdot_j, omega_j]^T
// omega_j modelled as random walk so the EKF can track slow frequency changes.
// Plus one offset state: x_off  (absorbs gravity + sensor bias)
// Total state dim = 3*Nm + 1
//
// Measurement (scalar): az_imu = sum_j(-omega_j^2 * z_j) + x_off
//
// Covariance update uses the symmetric Joseph form to maintain positive-
// definiteness and prevent numerical divergence.
class HeaveEKF {
public:
    HeaveEKF(double dt, double q_pos, double q_vel, double q_omega,
             double q_off, double r_meas,
             double omega_max = 3.0, double nis_gate = 25.0,
             double vel_limit = 5.0);

    // (Re-)initialize with identified modes. Reuses existing states when possible.
    // measured_offset is the DC mean of the acceleration window the modes were
    // identified from (FFTIdentifier::lastMean()); it seeds the offset state on
    // the first call, so the filter does not assume a particular IMU z-axis
    // sign or a bias-free accelerometer.
    void initialize(const std::vector<HeaveMode>& modes, double measured_offset);

    // Discard all state so the next initialize() starts fresh (no state reuse).
    // Used to recover cleanly after a divergence.
    void reset() { initialized_ = false; healthy_ = true; }

    // EKF predict + update step. Returns false if not yet initialized.
    bool update(double az_imu);

    double heavePosision() const { return heave_pos_; }
    double heaveVelocity() const { return heave_vel_; }
    bool isInitialized() const { return initialized_; }

    // Predict the heave position and velocity T seconds into the future by
    // propagating the identified harmonic modes forward in closed form
    // (same z̈ + ω²z = 0 solution as the one-step transition, with dt -> T).
    // Outputs 0 and does nothing if not yet initialized.
    void predict(double T, double& pos_pred, double& vel_pred) const;

    // False when the filter has diverged this step (output non-finite or beyond
    // physical limits). The node uses this to withhold the estimate and force a
    // re-identification instead of publishing garbage.
    bool isHealthy() const { return healthy_; }

    // Sum of the identified mode amplitudes [m] behind the current state --
    // the scale the estimate is expected to stay within. Reported alongside a
    // divergence so it is obvious whether the filter outgrew its own model.
    double identifiedAmplitude() const { return amp_sum_; }

private:
    double dt_;
    double q_pos_, q_vel_, q_omega_, q_off_, r_meas_;

    // Physical / robustness bounds.
    double omega_max_;   // max mode frequency [rad/s] — swell is well under this
    double nis_gate_;    // reject a measurement if innov^2/S exceeds this
    double vel_limit_;   // |heave velocity| beyond this [m/s] => diverged
    bool   healthy_ = true;

    // Sum of the amplitudes of the modes the filter was last initialised with:
    // the largest heave the identification actually supports. The estimate is
    // physically not allowed to exceed it by much, so it doubles as a tight
    // divergence check -- much tighter than the fixed 5 m/s velocity limit,
    // which a filter tracking accelerometer noise stays comfortably inside
    // while still being several times too large.
    double amp_sum_ = 0.0;

    bool initialized_ = false;
    int nm_        = 0;
    int state_dim_ = 0;

    Eigen::VectorXd x_;   // state estimate
    Eigen::MatrixXd P_;   // covariance (kept s.p.d. via Joseph form)

    double heave_pos_ = 0.0;
    double heave_vel_ = 0.0;

    Eigen::Vector3d modeTransition(const Eigen::Vector3d& xj) const;
    Eigen::Matrix3d modeJacobian(const Eigen::Vector3d& xj) const;
    Eigen::MatrixXd buildQ() const;
    void            checkObservability();

    int modeOffset(int j) const { return j * 3; }
};

}  // namespace heave_estimator
