#pragma once

#include <vector>
#include <Eigen/Dense>
#include "heave_estimator/fft_identifier.hpp"

namespace heave_estimator {

// Extended Kalman Filter estimating heave position and velocity from IMU acceleration.
//
// State per mode j (3 states): x_j = [z_j, zdot_j, omega_j]^T
// Plus one offset state: x_off (absorbs gravity + sensor bias)
// Total state dim = 3*Nm + 1
//
// Measurement model (scalar): y = sum_j(-omega_j^2 * z_j) + x_off = az_imu
class HeaveEKF {
public:
    HeaveEKF(double dt, double q_pos, double q_vel, double q_omega, double q_off, double r_meas);

    // (Re-)initialize with identified modes. Preserves existing mode states when possible.
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
    int nm_ = 0;                         // number of modes
    int state_dim_ = 0;                  // 3*nm_ + 1

    Eigen::VectorXd x_;                  // state vector
    Eigen::MatrixXd P_;                  // covariance matrix

    double heave_pos_ = 0.0;
    double heave_vel_ = 0.0;

    // Discrete transition for mode j given current state
    Eigen::Vector3d modeTransition(const Eigen::Vector3d& xj) const;

    // Jacobian of modeTransition w.r.t. xj
    Eigen::Matrix3d modeJacobian(const Eigen::Vector3d& xj) const;

    // Build full process noise matrix Q
    Eigen::MatrixXd buildQ() const;

    void checkObservability();

    // idx of first element of mode j in state vector
    int modeOffset(int j) const { return j * 3; }
};

}  // namespace heave_estimator
