#include "heave_estimator/heave_ekf.hpp"

#include <cmath>

namespace heave_estimator {

HeaveEKF::HeaveEKF(double dt, double q_scale, double q_off, double r_meas)
    : dt_(dt), q_scale_(q_scale), q_off_(q_off), r_meas_(r_meas)
{
}

// ---- Matrix builders -------------------------------------------------------

Eigen::MatrixXd HeaveEKF::buildPsi(const std::vector<HeaveMode>& modes) const
{
    int dim = 2 * static_cast<int>(modes.size()) + 1;
    Eigen::MatrixXd Psi = Eigen::MatrixXd::Zero(dim, dim);

    for (int i = 0; i < static_cast<int>(modes.size()); ++i) {
        double omega = modes[i].frequency;   // already rad/s from FFT
        double c = std::cos(omega * dt_);
        double s = std::sin(omega * dt_);
        int idx = 2 * i;

        // Exact matrix exponential of [[0,1],[-omega²,0]] over dt
        Psi(idx,     idx)     =  c;
        Psi(idx,     idx + 1) =  s / omega;
        Psi(idx + 1, idx)     = -omega * s;
        Psi(idx + 1, idx + 1) =  c;
    }
    Psi(dim - 1, dim - 1) = 1.0;   // offset: slow random walk
    return Psi;
}

Eigen::RowVectorXd HeaveEKF::buildC(const std::vector<HeaveMode>& modes) const
{
    // az_imu = sum_j(-omega_j^2 * z_j) + x_off
    int dim = 2 * static_cast<int>(modes.size()) + 1;
    Eigen::RowVectorXd C = Eigen::RowVectorXd::Zero(dim);
    for (int i = 0; i < static_cast<int>(modes.size()); ++i) {
        double omega = modes[i].frequency;
        C(2 * i) = -omega * omega;  // coefficient of z_j in measurement
    }
    C(dim - 1) = 1.0;   // x_off
    return C;
}

Eigen::MatrixXd HeaveEKF::buildQ(const std::vector<HeaveMode>& modes) const
{
    int dim = 2 * static_cast<int>(modes.size()) + 1;
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(dim, dim);
    for (int i = 0; i < static_cast<int>(modes.size()); ++i) {
        Q(2 * i,     2 * i)     = q_scale_;
        Q(2 * i + 1, 2 * i + 1) = q_scale_;
    }
    Q(dim - 1, dim - 1) = q_off_;
    return Q;
}

// Iterative DARE solver — same structure as SPAObserver::solveDARE.
Eigen::VectorXd HeaveEKF::solveDARE(const Eigen::MatrixXd& Psi,
                                      const Eigen::MatrixXd& Q,
                                      const Eigen::RowVectorXd& C) const
{
    Eigen::MatrixXd P = Q;

    for (int iter = 0; iter < 500; ++iter) {
        double S = (C * P * C.transpose())(0, 0) + r_meas_;
        Eigen::VectorXd K  = P * C.transpose() / S;
        Eigen::MatrixXd P_new = Psi * (P - K * C * P) * Psi.transpose() + Q;

        if ((P_new - P).norm() < 1e-12) { P = P_new; break; }
        P = P_new;
    }

    double S = (C * P * C.transpose())(0, 0) + r_meas_;
    return Psi * P * C.transpose() / S;
}

// ---- Public interface -------------------------------------------------------

void HeaveEKF::initialize(const std::vector<HeaveMode>& modes)
{
    if (modes.empty()) return;

    int new_nm  = static_cast<int>(modes.size());
    int new_dim = 2 * new_nm + 1;

    Eigen::MatrixXd    Psi_new = buildPsi(modes);
    Eigen::RowVectorXd C_new   = buildC(modes);
    Eigen::MatrixXd    Q_new   = buildQ(modes);
    Eigen::VectorXd    L_new   = solveDARE(Psi_new, Q_new, C_new);

    // Build initial state from FFT-identified parameters.
    // Position convention: z_j(t) = A_j cos(omega_j * t + phi_j)
    // So z_j(0) = A cos(phi), zdot_j(0) = -A*omega*sin(phi)
    // phase from FFT is the position phase at t=0 of the buffer window.
    Eigen::VectorXd x_new = Eigen::VectorXd::Zero(new_dim);

    for (int i = 0; i < new_nm; ++i) {
        double omega = modes[i].frequency;
        bool reused = false;

        // Reuse existing state if a matching frequency is found
        if (initialized_) {
            for (int j = 0; j < nm_; ++j) {
                if (std::abs(omegas_[j] - omega) < 0.05) {
                    x_new.segment<2>(2 * i) = x_hat_.segment<2>(2 * j);
                    reused = true;
                    break;
                }
            }
        }

        if (!reused) {
            double phi = modes[i].phase;
            x_new(2 * i)     = modes[i].amplitude * std::cos(phi);
            x_new(2 * i + 1) = -modes[i].amplitude * omega * std::sin(phi);
        }
    }

    // Preserve offset (absorbs gravity); init to -g on first run.
    static constexpr double g = 9.81;
    x_new(new_dim - 1) = initialized_ ? x_hat_(state_dim_ - 1) : -g;

    Psi_   = Psi_new;
    C_     = C_new;
    L_     = L_new;
    x_hat_ = x_new;

    omegas_.resize(new_nm);
    for (int i = 0; i < new_nm; ++i) omegas_[i] = modes[i].frequency;

    nm_          = new_nm;
    state_dim_   = new_dim;
    initialized_ = true;
}

bool HeaveEKF::update(double az_imu)
{
    if (!initialized_) return false;

    double s_hat = (C_ * x_hat_)(0);
    double innov = az_imu - s_hat;

    // Observer step: x̂_{k+1} = Ψ x̂_k + L (y_k − ŷ_k)
    x_hat_ = Psi_ * x_hat_ + L_ * innov;

    // Aggregate heave position and velocity over all modes
    heave_pos_ = 0.0;
    heave_vel_ = 0.0;
    for (int j = 0; j < nm_; ++j) {
        heave_pos_ += x_hat_(2 * j);
        heave_vel_ += x_hat_(2 * j + 1);
    }

    return true;
}

}  // namespace heave_estimator
