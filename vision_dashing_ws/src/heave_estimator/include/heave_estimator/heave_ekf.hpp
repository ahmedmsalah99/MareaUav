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
             double q_off, double r_meas);

    // (Re-)initialize with identified modes. Reuses existing states when possible.
    void initialize(const std::vector<HeaveMode>& modes);

    // EKF predict + update step. Returns false if not yet initialized.
    bool update(double az_imu);

    double heavePosision() const { return heave_pos_; }
    double heaveVelocity() const { return heave_vel_; }
    bool isInitialized() const { return initialized_; }

private:
    double dt_;
    double q_pos_, q_vel_, q_omega_, q_off_, r_meas_;

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
