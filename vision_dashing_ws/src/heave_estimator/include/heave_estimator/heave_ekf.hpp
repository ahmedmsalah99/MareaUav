#pragma once

#include <vector>
#include <Eigen/Dense>
#include "heave_estimator/fft_identifier.hpp"

namespace heave_estimator {

// Linear steady-state Kalman observer for heave position / velocity.
//
// Frequencies are fixed from FFT identification (not estimated).
// State per mode j (2 states): [z_j, zdot_j]
// Plus one offset state: x_off  (absorbs gravity + sensor bias)
// Total state dim = 2*Nm + 1
//
// Transition: block-diagonal harmonic oscillator (matrix exponential, exact)
// Measurement (scalar): az_imu = sum_j(-omega_j^2 * z_j) + x_off
//
// DARE is solved once at initialization to get the steady-state Kalman gain.
class HeaveEKF {
public:
    HeaveEKF(double dt, double q_scale, double q_off, double r_meas);

    void initialize(const std::vector<HeaveMode>& modes);

    // Observer predict + update step. Returns false if not initialized.
    bool update(double az_imu);

    double heavePosision() const { return heave_pos_; }
    double heaveVelocity() const { return heave_vel_; }
    bool isInitialized() const { return initialized_; }

private:
    double dt_;
    double q_scale_, q_off_, r_meas_;

    bool initialized_ = false;
    int nm_ = 0;
    int state_dim_ = 0;

    Eigen::MatrixXd    Psi_;   // full block-diagonal transition matrix
    Eigen::RowVectorXd C_;     // measurement row: [-w1² 0 -w2² 0 ... 1]
    Eigen::VectorXd    L_;     // steady-state Kalman gain (predictor form)
    Eigen::VectorXd    x_hat_; // state estimate
    std::vector<double> omegas_;

    double heave_pos_ = 0.0;
    double heave_vel_ = 0.0;

    Eigen::MatrixXd    buildPsi(const std::vector<HeaveMode>& modes) const;
    Eigen::RowVectorXd buildC(const std::vector<HeaveMode>& modes) const;
    Eigen::MatrixXd    buildQ(const std::vector<HeaveMode>& modes) const;
    Eigen::VectorXd    solveDARE(const Eigen::MatrixXd& Psi,
                                  const Eigen::MatrixXd& Q,
                                  const Eigen::RowVectorXd& C) const;
};

}  // namespace heave_estimator
