#include "heave_estimator/heave_ekf.hpp"

#include <cmath>
#include <algorithm>

namespace heave_estimator {

HeaveEKF::HeaveEKF(double dt, double q_pos, double q_vel, double q_omega, double q_off,
                   double r_meas)
    : dt_(dt), q_pos_(q_pos), q_vel_(q_vel), q_omega_(q_omega), q_off_(q_off), r_meas_(r_meas)
{
}

void HeaveEKF::initialize(const std::vector<HeaveMode>& modes)
{
    int new_nm = static_cast<int>(modes.size());
    int new_dim = 3 * new_nm + 1;

    Eigen::VectorXd x_new = Eigen::VectorXd::Zero(new_dim);
    Eigen::MatrixXd P_new = Eigen::MatrixXd::Identity(new_dim, new_dim);

    // Initialize offset state: x_off = -g (gravity acts negative on z-down sensor)
    static constexpr double g = 9.81;
    x_new(new_dim - 1) = -g;
    P_new(new_dim - 1, new_dim - 1) = 1.0;

    for (int j = 0; j < new_nm; ++j) {
        const HeaveMode& m = modes[j];
        int idx = modeOffset(j);

        // Initial z_j(t0) = A_j * cos(phi_j), zdot_j(t0) = -A_j * omega_j * sin(phi_j)
        double z0 = m.amplitude * std::cos(m.phase);
        double zd0 = -m.amplitude * m.frequency * std::sin(m.phase);

        // Try to reuse current estimate if this mode existed before
        bool reused = false;
        if (initialized_) {
            for (int i = 0; i < nm_; ++i) {
                double omega_est = x_(modeOffset(i) + 2);
                if (std::abs(omega_est - m.frequency) < 0.05) {  // 0.05 rad/s tolerance
                    x_new.segment<3>(idx) = x_.segment<3>(modeOffset(i));
                    P_new.block<3, 3>(idx, idx) = P_.block<3, 3>(modeOffset(i), modeOffset(i));
                    reused = true;
                    break;
                }
            }
        }

        if (!reused) {
            x_new(idx)     = z0;
            x_new(idx + 1) = zd0;
            x_new(idx + 2) = m.frequency;
            // Position/velocity uncertainty based on amplitude; omega uncertainty small
            P_new.block<3, 3>(idx, idx) = Eigen::DiagonalMatrix<double, 3>(
                m.amplitude * m.amplitude, m.amplitude * m.amplitude * m.frequency * m.frequency,
                0.01).toDenseMatrix();
        }
    }

    // Preserve offset state if already running
    if (initialized_) {
        x_new(new_dim - 1) = x_(state_dim_ - 1);
        P_new(new_dim - 1, new_dim - 1) = P_(state_dim_ - 1, state_dim_ - 1);
    }

    nm_        = new_nm;
    state_dim_ = new_dim;
    x_         = x_new;
    P_         = P_new;
    initialized_ = true;
}

// Discrete transition for a single mode: [z, zdot, omega] -> [z', zdot', omega]
Eigen::Vector3d HeaveEKF::modeTransition(const Eigen::Vector3d& xj) const
{
    double z  = xj(0);
    double zd = xj(1);
    double w  = xj(2);  // omega

    if (std::abs(w) < 1e-6) {
        // Degenerate: treat as constant velocity
        return Eigen::Vector3d(z + zd * dt_, zd, w);
    }

    double c = std::cos(w * dt_);
    double s = std::sin(w * dt_);

    Eigen::Vector3d xj_next;
    xj_next(0) = z * c + zd * s / w;
    xj_next(1) = -z * w * s + zd * c;
    xj_next(2) = w;  // random walk: stays constant in predict step
    return xj_next;
}

// Jacobian of modeTransition w.r.t. xj (3x3)
Eigen::Matrix3d HeaveEKF::modeJacobian(const Eigen::Vector3d& xj) const
{
    double z  = xj(0);
    double zd = xj(1);
    double w  = xj(2);

    if (std::abs(w) < 1e-6) {
        Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
        F(0, 1) = dt_;
        return F;
    }

    double c = std::cos(w * dt_);
    double s = std::sin(w * dt_);

    Eigen::Matrix3d F;
    // d(z')/d(z, zd, w)
    F(0, 0) = c;
    F(0, 1) = s / w;
    F(0, 2) = -z * dt_ * s + zd * (dt_ * c / w - s / (w * w));

    // d(zd')/d(z, zd, w)
    F(1, 0) = -w * s;
    F(1, 1) = c;
    F(1, 2) = -z * (s + w * dt_ * c) - zd * dt_ * s;

    // d(w')/d(...)
    F(2, 0) = 0.0;
    F(2, 1) = 0.0;
    F(2, 2) = 1.0;

    return F;
}

Eigen::MatrixXd HeaveEKF::buildQ() const
{
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(state_dim_, state_dim_);
    for (int j = 0; j < nm_; ++j) {
        int idx = modeOffset(j);
        double omega = x_(idx + 2);
        // Higher frequency modes get higher pos/vel noise
        double freq_scale = std::max(1.0, omega / (2.0 * M_PI));
        Q(idx,     idx)     = q_pos_ * freq_scale;
        Q(idx + 1, idx + 1) = q_vel_ * freq_scale;
        Q(idx + 2, idx + 2) = q_omega_;
    }
    Q(state_dim_ - 1, state_dim_ - 1) = q_off_;
    return Q;
}

bool HeaveEKF::update(double az_imu)
{
    if (!initialized_) return false;

    // ---- Predict ----
    Eigen::VectorXd x_pred = Eigen::VectorXd::Zero(state_dim_);
    Eigen::MatrixXd F = Eigen::MatrixXd::Zero(state_dim_, state_dim_);

    for (int j = 0; j < nm_; ++j) {
        int idx = modeOffset(j);
        Eigen::Vector3d xj = x_.segment<3>(idx);
        x_pred.segment<3>(idx) = modeTransition(xj);
        F.block<3, 3>(idx, idx) = modeJacobian(xj);
    }
    // Offset state: identity (random walk)
    x_pred(state_dim_ - 1) = x_(state_dim_ - 1);
    F(state_dim_ - 1, state_dim_ - 1) = 1.0;

    Eigen::MatrixXd Q = buildQ();
    Eigen::MatrixXd P_pred = F * P_ * F.transpose() + Q;

    // ---- Update ----
    // Measurement: y = sum_j(-omega_j^2 * z_j) + x_off
    double y_pred = 0.0;
    Eigen::RowVectorXd H = Eigen::RowVectorXd::Zero(state_dim_);

    for (int j = 0; j < nm_; ++j) {
        int idx = modeOffset(j);
        double z_j = x_pred(idx);
        double w_j = x_pred(idx + 2);
        y_pred += -w_j * w_j * z_j;

        H(idx)     = -w_j * w_j;      // d(y)/d(z_j)
        H(idx + 1) = 0.0;
        H(idx + 2) = -2.0 * w_j * z_j; // d(y)/d(omega_j)
    }
    H(state_dim_ - 1) = 1.0;  // d(y)/d(x_off)
    y_pred += x_pred(state_dim_ - 1);

    double innov = az_imu - y_pred;
    double S = (H * P_pred * H.transpose())(0, 0) + r_meas_;
    Eigen::VectorXd K = P_pred * H.transpose() / S;  // Kalman gain

    x_ = x_pred + K * innov;
    P_ = (Eigen::MatrixXd::Identity(state_dim_, state_dim_) - K * H) * P_pred;

    checkObservability();

    // Aggregate heave position and velocity
    heave_pos_ = 0.0;
    heave_vel_ = 0.0;
    for (int j = 0; j < nm_; ++j) {
        int idx = modeOffset(j);
        heave_pos_ += x_(idx);
        heave_vel_ += x_(idx + 1);
    }

    return true;
}

void HeaveEKF::checkObservability()
{
    // Ensure each mode's omega > 0, and all omegas are distinct.
    // If omega approaches zero, clamp it.
    // If two omegas are too close, collapse the smaller-amplitude one.
    static constexpr double omega_min = 0.01;   // rad/s
    static constexpr double omega_tol = 0.05;   // rad/s

    for (int j = 0; j < nm_; ++j) {
        double& w = x_(modeOffset(j) + 2);
        if (w < omega_min) w = omega_min;
    }

    // Enforce distinct frequencies (simple: clamp close ones slightly apart)
    for (int j = 0; j < nm_; ++j) {
        for (int i = j + 1; i < nm_; ++i) {
            double& wi = x_(modeOffset(i) + 2);
            double& wj = x_(modeOffset(j) + 2);
            if (std::abs(wi - wj) < omega_tol) {
                // Separate them by pushing the higher-index one up
                wi = wj + omega_tol;
            }
        }
    }
}

}  // namespace heave_estimator
