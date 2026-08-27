#include "heave_estimator/heave_ekf.hpp"

#include <cmath>
#include <algorithm>

namespace heave_estimator {

HeaveEKF::HeaveEKF(double dt, double q_pos, double q_vel, double q_omega,
                   double q_off, double r_meas,
                   double omega_max, double nis_gate,
                   double vel_limit)
    : dt_(dt), q_pos_(q_pos), q_vel_(q_vel), q_omega_(q_omega),
      q_off_(q_off), r_meas_(r_meas),
      omega_max_(omega_max), nis_gate_(nis_gate),
      vel_limit_(vel_limit)
{
}

// ---- Discrete mode dynamics ------------------------------------------------

// Exact solution of z̈ + ω²z = 0 over one dt step.
// State: [z, ẑ, ω]; ω treated as constant within the step.
Eigen::Vector3d HeaveEKF::modeTransition(const Eigen::Vector3d& xj) const
{
    double z  = xj(0);
    double zd = xj(1);
    double w  = xj(2);

    if (std::abs(w) < 1e-6) {
        return Eigen::Vector3d(z + zd * dt_, zd, w);
    }

    double c = std::cos(w * dt_);
    double s = std::sin(w * dt_);

    return Eigen::Vector3d(
         z * c + zd * s / w,
        -z * w * s + zd * c,
         w);
}

// Jacobian of modeTransition w.r.t. [z, ẑ, ω]
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
    F(0, 0) =  c;
    F(0, 1) =  s / w;
    F(0, 2) = -z * dt_ * s + zd * (dt_ * c / w - s / (w * w));

    F(1, 0) = -w * s;
    F(1, 1) =  c;
    F(1, 2) = -z * (s + w * dt_ * c) - zd * dt_ * s;

    F(2, 0) = 0.0;
    F(2, 1) = 0.0;
    F(2, 2) = 1.0;

    return F;
}

Eigen::MatrixXd HeaveEKF::buildQ() const
{
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(state_dim_, state_dim_);
    for (int j = 0; j < nm_; ++j) {
        int    idx        = modeOffset(j);
        double omega      = x_(idx + 2);
        double freq_scale = std::max(1.0, omega / (2.0 * M_PI));
        Q(idx,     idx)     = q_pos_ * freq_scale;
        Q(idx + 1, idx + 1) = q_vel_ * freq_scale;
        Q(idx + 2, idx + 2) = q_omega_;
    }
    Q(state_dim_ - 1, state_dim_ - 1) = q_off_;
    return Q;
}

// ---- Initialization --------------------------------------------------------

void HeaveEKF::initialize(const std::vector<HeaveMode>& modes, double measured_offset)
{
    if (modes.empty()) return;

    int new_nm  = static_cast<int>(modes.size());
    int new_dim = 3 * new_nm + 1;

    Eigen::VectorXd x_new = Eigen::VectorXd::Zero(new_dim);
    Eigen::MatrixXd P_new = Eigen::MatrixXd::Identity(new_dim, new_dim);

    for (int j = 0; j < new_nm; ++j) {
        const HeaveMode& m = modes[j];
        int idx = modeOffset(j);

        bool reused = false;
        if (initialized_) {
            for (int i = 0; i < nm_; ++i) {
                if (std::abs(x_(modeOffset(i) + 2) - m.frequency) < 0.05) {
                    x_new.segment<3>(idx) = x_.segment<3>(modeOffset(i));
                    P_new.block<3, 3>(idx, idx) = P_.block<3, 3>(modeOffset(i), modeOffset(i));
                    reused = true;
                    break;
                }
            }
        }

        if (!reused) {
            // z_j(t) = A cos(ωt + φ); t=0 is the start of the FFT window
            double phi = m.phase;
            x_new(idx)     = m.amplitude * std::cos(phi);
            x_new(idx + 1) = -m.amplitude * m.frequency * std::sin(phi);
            x_new(idx + 2) = m.frequency;

            double A2 = m.amplitude * m.amplitude;
            P_new.block<3, 3>(idx, idx) = Eigen::DiagonalMatrix<double, 3>(
                A2,
                A2 * m.frequency * m.frequency,
                0.01).toDenseMatrix();
        }
    }

    // Preserve the offset across re-identifications; on the first call seed it
    // with the MEASURED DC mean of the identification window rather than a
    // hard-coded -g. The measurement model is
    //     az = sum_j(-omega_j^2 * z_j) + offset
    // so the offset has to absorb the whole constant term (gravity projected on
    // the IMU z axis, plus accelerometer bias). Assuming -9.81 breaks whenever
    // the IMU reports +g on z (the usual ROS z-up convention) or carries a
    // noticeable bias: the ~2g constant mismatch cannot be taken up by the
    // near-frozen offset state (q_off is tiny), so it is forced into the
    // oscillator states instead, inflating heave position/velocity far past the
    // identified mode amplitude and driving the filter into a limit cycle.
    x_new(new_dim - 1) = initialized_ ? x_(state_dim_ - 1) : measured_offset;
    P_new(new_dim - 1, new_dim - 1) = initialized_
        ? P_(state_dim_ - 1, state_dim_ - 1) : 1.0;

    amp_sum_ = 0.0;
    for (const HeaveMode& m : modes) amp_sum_ += std::abs(m.amplitude);

    nm_          = new_nm;
    state_dim_   = new_dim;
    x_           = x_new;
    P_           = P_new;
    initialized_ = true;
}

// ---- EKF update step -------------------------------------------------------

bool HeaveEKF::update(double az_imu)
{
    if (!initialized_) return false;

    // ---- Predict ----
    Eigen::VectorXd x_pred = Eigen::VectorXd::Zero(state_dim_);
    Eigen::MatrixXd F      = Eigen::MatrixXd::Zero(state_dim_, state_dim_);

    for (int j = 0; j < nm_; ++j) {
        int idx = modeOffset(j);
        Eigen::Vector3d xj = x_.segment<3>(idx);
        x_pred.segment<3>(idx)      = modeTransition(xj);
        F.block<3, 3>(idx, idx)     = modeJacobian(xj);
    }
    x_pred(state_dim_ - 1) = x_(state_dim_ - 1);
    F(state_dim_ - 1, state_dim_ - 1) = 1.0;

    Eigen::MatrixXd P_pred = F * P_ * F.transpose() + buildQ();

    // ---- Update ----
    // Linearise measurement at x_pred:  y = sum_j(-omega_j^2 * z_j) + x_off
    double y_pred = x_pred(state_dim_ - 1);
    Eigen::RowVectorXd H = Eigen::RowVectorXd::Zero(state_dim_);
    H(state_dim_ - 1) = 1.0;

    for (int j = 0; j < nm_; ++j) {
        int    idx = modeOffset(j);
        double z_j = x_pred(idx);
        double w_j = x_pred(idx + 2);
        y_pred    += -w_j * w_j * z_j;
        H(idx)     = -w_j * w_j;
        H(idx + 2) = -2.0 * w_j * z_j;
    }

    double innov = az_imu - y_pred;
    double S     = (H * P_pred * H.transpose())(0, 0) + r_meas_;

    // Innovation (NIS) gate: reject an implausible measurement so a single bad
    // IMU sample cannot kick the filter into divergence. Predict-only this step.
    double nis = (S > 0.0) ? (innov * innov / S) : 0.0;
    if (nis > nis_gate_) {
        x_ = x_pred;
        P_ = P_pred;
    } else {
        Eigen::VectorXd K = P_pred * H.transpose() / S;
        x_ = x_pred + K * innov;

        // Joseph form update to keep P symmetric positive-definite
        Eigen::MatrixXd I_KH = Eigen::MatrixXd::Identity(state_dim_, state_dim_) - K * H;
        P_ = I_KH * P_pred * I_KH.transpose() + r_meas_ * K * K.transpose();
    }

    checkObservability();

    heave_pos_ = 0.0;
    heave_vel_ = 0.0;
    for (int j = 0; j < nm_; ++j) {
        int idx = modeOffset(j);
        heave_pos_ += x_(idx);
        heave_vel_ += x_(idx + 1);
    }

    // Divergence guard: flag when the output is non-finite or the heave velocity
    // is beyond any physically plausible value. Heave position is left unbounded
    // (it is not range-limited). The node withholds the estimate and forces a
    // re-identification when this trips.
    // The estimate cannot legitimately be much larger than the modes it was
    // built from: |z| <= sum(A_j) and |z_dot| <= sum(A_j * omega_j) <=
    // amp_sum * omega_max. Allow a generous factor for genuine growth between
    // re-identifications, plus a small floor so a near-zero identification
    // cannot latch the filter permanently unhealthy.
    static constexpr double kAmpSlack = 3.0;
    static constexpr double kAmpFloor = 0.05;   // [m]
    double pos_limit = kAmpSlack * amp_sum_ + kAmpFloor;

    healthy_ = std::isfinite(heave_pos_) && std::isfinite(heave_vel_)
            && std::fabs(heave_vel_) <= vel_limit_
            && std::fabs(heave_pos_) <= pos_limit;

    return true;
}

// ---- Forward prediction ----------------------------------------------------

void HeaveEKF::predict(double T, double& pos_pred, double& vel_pred) const
{
    pos_pred = 0.0;
    vel_pred = 0.0;
    if (!initialized_) return;

    // Propagate each harmonic mode T seconds forward: z(T) = z·cos(ωT) + (ż/ω)·sin(ωT)
    for (int j = 0; j < nm_; ++j) {
        int    idx = modeOffset(j);
        double z   = x_(idx);
        double zd  = x_(idx + 1);
        double w   = x_(idx + 2);

        if (std::abs(w) < 1e-6) {
            pos_pred += z + zd * T;
            vel_pred += zd;
        } else {
            double c = std::cos(w * T);
            double s = std::sin(w * T);
            pos_pred += z * c + zd * s / w;
            vel_pred += -z * w * s + zd * c;
        }
    }
}

// ---- Observability guard ---------------------------------------------------

void HeaveEKF::checkObservability()
{
    static constexpr double omega_min = 0.01;
    static constexpr double omega_sep = 0.05;

    // Clamp each mode frequency to a physical band. Without the upper bound a
    // drifting omega makes the -omega^2 measurement terms explode and the filter
    // diverges (the "heave velocity > 200" failure).
    for (int j = 0; j < nm_; ++j) {
        double& w = x_(modeOffset(j) + 2);
        if (w < omega_min)  w = omega_min;
        if (w > omega_max_) w = omega_max_;
    }

    // Ensure distinct frequencies
    for (int j = 0; j < nm_; ++j) {
        for (int i = j + 1; i < nm_; ++i) {
            double& wj = x_(modeOffset(j) + 2);
            double& wi = x_(modeOffset(i) + 2);
            if (std::abs(wi - wj) < omega_sep) {
                wi = wj + omega_sep;
            }
        }
    }
}

}  // namespace heave_estimator
