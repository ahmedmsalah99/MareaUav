#include "spa_predictor/spa_observer.hpp"

#include <cmath>

namespace spa_predictor {

SPAObserver::SPAObserver(double dt, double q_scale, double r_meas)
    : dt_(dt), q_scale_(q_scale), r_meas_(r_meas)
{
}

// ---- Helpers ---------------------------------------------------------------

Eigen::MatrixXd SPAObserver::buildPsi(const std::vector<Mode>& modes) const
{
    int dim = 2 * static_cast<int>(modes.size()) + 1;
    Eigen::MatrixXd Psi = Eigen::MatrixXd::Zero(dim, dim);

    for (int i = 0; i < static_cast<int>(modes.size()); ++i) {
        double omega = 2.0 * M_PI * modes[i].frequency;
        double c = std::cos(omega * dt_);
        double s = std::sin(omega * dt_);
        int idx = 2 * i;

        // Matrix exponential of Aᵢ = [0, 1; -ω², 0] over ΔT (closed form)
        Psi(idx,     idx)     =  c;
        Psi(idx,     idx + 1) =  s / omega;
        Psi(idx + 1, idx)     = -omega * s;
        Psi(idx + 1, idx + 1) =  c;
    }

    // Offset state: identity (random walk — stays constant in predict step)
    Psi(dim - 1, dim - 1) = 1.0;

    return Psi;
}

Eigen::RowVectorXd SPAObserver::buildC(int n_modes) const
{
    // C = [1, 0, 1, 0, ..., 1, 0, 1]
    // Selects x_{i,1} from each mode pair, plus x_off at the end.
    int dim = 2 * n_modes + 1;
    Eigen::RowVectorXd C = Eigen::RowVectorXd::Zero(dim);
    for (int i = 0; i < n_modes; ++i) C(2 * i) = 1.0;
    C(dim - 1) = 1.0;
    return C;
}

Eigen::MatrixXd SPAObserver::buildQ(const std::vector<Mode>& modes) const
{
    int dim = 2 * static_cast<int>(modes.size()) + 1;
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(dim, dim);

    for (int i = 0; i < static_cast<int>(modes.size()); ++i) {
        double freq_scale = std::max(1.0, modes[i].frequency);
        Q(2 * i,     2 * i)     = q_scale_ * freq_scale;   // signal state
        Q(2 * i + 1, 2 * i + 1) = q_scale_ * freq_scale;   // rate state
    }
    // Offset varies very slowly
    Q(dim - 1, dim - 1) = q_scale_ * 1e-4;

    return Q;
}

// Solves the DARE iteratively and returns the steady-state Kalman gain L.
//
// DARE (predictor form):
//   P⁻ = Ψ (P⁻ − P⁻ Cᵀ (C P⁻ Cᵀ + R)⁻¹ C P⁻) Ψᵀ + Q
//
// Since C P Cᵀ is a scalar (scalar measurement), the gain simplifies to:
//   L = Ψ P⁻ Cᵀ / (C P⁻ Cᵀ + R)
Eigen::VectorXd SPAObserver::solveDARE(const Eigen::MatrixXd& Psi,
                                        const Eigen::MatrixXd& Q,
                                        const Eigen::RowVectorXd& C) const
{
    Eigen::MatrixXd P = Q;   // initialise with process noise

    for (int iter = 0; iter < 500; ++iter) {
        double S    = (C * P * C.transpose())(0, 0) + r_meas_;   // scalar
        Eigen::VectorXd K  = P * C.transpose() / S;              // corrector gain
        Eigen::MatrixXd P_new = Psi * (P - K * C * P) * Psi.transpose() + Q;

        if ((P_new - P).norm() < 1e-12) { P = P_new; break; }
        P = P_new;
    }

    double S = (C * P * C.transpose())(0, 0) + r_meas_;
    return Psi * P * C.transpose() / S;   // L = Ψ P⁻ Cᵀ (C P⁻ Cᵀ + R)⁻¹
}

// ---- Public interface -------------------------------------------------------

void SPAObserver::initialize(const std::vector<Mode>& modes, double t0)
{
    if (modes.empty()) return;

    int new_n   = static_cast<int>(modes.size());
    int new_dim = 2 * new_n + 1;

    Eigen::MatrixXd  Psi_new = buildPsi(modes);
    Eigen::RowVectorXd C_new = buildC(new_n);
    Eigen::MatrixXd  Q_new   = buildQ(modes);
    Eigen::VectorXd  L_new   = solveDARE(Psi_new, Q_new, C_new);

    // Build initial state from FFT-identified parameters.
    // x_{i,1}(t0) = Aᵢ sin(2πfᵢ t0 + φᵢ)
    // x_{i,2}(t0) = Aᵢ 2πfᵢ cos(2πfᵢ t0 + φᵢ)
    Eigen::VectorXd x_new = Eigen::VectorXd::Zero(new_dim);

    for (int i = 0; i < new_n; ++i) {
        double omega = 2.0 * M_PI * modes[i].frequency;
        double arg   = omega * t0 + modes[i].phase;

        // Try to reuse existing state for a matching frequency
        bool reused = false;
        if (initialized_) {
            for (int j = 0; j < n_modes_; ++j) {
                if (std::abs(freqs_[j] - modes[i].frequency) < 0.01) {
                    x_new.segment<2>(2 * i) = x_hat_.segment<2>(2 * j);
                    reused = true;
                    break;
                }
            }
        }

        if (!reused) {
            x_new(2 * i)     = modes[i].amplitude * std::sin(arg);
            x_new(2 * i + 1) = modes[i].amplitude * omega * std::cos(arg);
        }
    }

    // Preserve offset state if already running
    x_new(new_dim - 1) = initialized_ ? x_hat_(state_dim_ - 1) : 0.0;

    Psi_   = Psi_new;
    C_     = C_new;
    L_     = L_new;
    x_hat_ = x_new;

    freqs_.resize(new_n);
    for (int i = 0; i < new_n; ++i) freqs_[i] = modes[i].frequency;

    n_modes_     = new_n;
    state_dim_   = new_dim;
    t_current_   = t0;
    initialized_ = true;
}

bool SPAObserver::update(double s_measured, double t_current)
{
    if (!initialized_) return false;

    t_current_ = t_current;

    // Innovation
    double s_hat  = (C_ * x_hat_)(0);
    double innov  = s_measured - s_hat;

    // Observer step (Eq. 6): x̂_{k+1} = Ψ x̂_k + L (s_k − ŝ_k)
    x_hat_ = Psi_ * x_hat_ + L_ * innov;

    estimate_ = (C_ * x_hat_)(0);

    return true;
}

double SPAObserver::predict(double T_pred) const
{
    if (!initialized_) return 0.0;

    double t_pred = t_current_ + T_pred;

    // Recover adapted amplitude and phase for each mode (Eqs. 7-8),
    // then evaluate the sum at t + T_pred (Eq. 9).
    double s_pred = x_hat_(state_dim_ - 1);   // x_off

    for (int i = 0; i < n_modes_; ++i) {
        double fi    = freqs_[i];
        double omega = 2.0 * M_PI * fi;
        double xi1   = x_hat_(2 * i);
        double xi2   = x_hat_(2 * i + 1);

        // Eq. 7: φ_obs = arctan(2πfᵢ x̂_{i,1} / x̂_{i,2}) − 2πfᵢ tₖ
        double phi_obs = std::atan2(omega * xi1, xi2) - omega * t_current_;

        // Eq. 8: A_obs = x̂_{i,1} / sin(2πfᵢ tₖ + φ_obs)
        double sin_val = std::sin(omega * t_current_ + phi_obs);
        double A_obs   = (std::abs(sin_val) > 1e-6) ? xi1 / sin_val
                                                      : std::hypot(xi1, xi2 / omega);

        // Eq. 9 contribution
        s_pred += A_obs * std::sin(omega * t_pred + phi_obs);
    }

    return s_pred;
}

}  // namespace spa_predictor
