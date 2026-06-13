#pragma once

#include <vector>
#include <Eigen/Dense>
#include "spa_predictor/fft_identifier.hpp"

namespace spa_predictor {

// Linear steady-state Kalman observer for signal prediction (SPA).
//
// State-space model (paper Eqs. 3-5):
//   Each mode i:  Aᵢ = [ 0,  1; -(2πfᵢ)², 0 ]
//   Full model:   ẋ = block_diag(A₁,...,Aₙ, 0) x   (0 for x_off random walk)
//   Output:       s = C x  where C = [1,0, 1,0, ..., 1,0, 1]
//
// State vector x ∈ ℝ^(2N+1):
//   [x_{1,1}, x_{1,2}, x_{2,1}, x_{2,2}, ..., x_{N,1}, x_{N,2}, x_off]
//   x_{i,1} = mode-i signal component  = Aᵢ sin(2πfᵢ t + φᵢ)
//   x_{i,2} = mode-i rate component    = Aᵢ 2πfᵢ cos(2πfᵢ t + φᵢ)
//
// Observer (Eq. 6):  x̂_{k+1} = Ψ x̂_k + L (s_k − ŝ_k),  ŝ_k = C x̂_k
// Gain L is the steady-state Kalman gain solved via DARE.
//
// Prediction (Eq. 9):  s_pred(t+T) = Σ A_obs,i sin(2πfᵢ(t+T) + φ_obs,i) + x_off
class SPAObserver {
public:
    // q_scale : process noise scaling (tune: larger → trust measurement more)
    // r_meas  : measurement noise variance
    SPAObserver(double dt, double q_scale, double r_meas);

    // (Re-)initialise with identified modes. Reuses existing state when frequencies match.
    void initialize(const std::vector<Mode>& modes, double t0);

    // Observer update step. Returns false if not yet initialised.
    bool update(double s_measured, double t_current);

    // Current signal estimate (sum of all modes + offset).
    double estimate() const { return estimate_; }

    // Predicted signal at t_current + T_pred seconds (Eq. 9).
    double predict(double T_pred) const;

    bool isInitialized() const { return initialized_; }

private:
    double dt_;
    double q_scale_;
    double r_meas_;

    bool   initialized_ = false;
    int    n_modes_     = 0;
    int    state_dim_   = 0;   // 2*n_modes_ + 1

    Eigen::VectorXd x_hat_;    // observer state
    Eigen::MatrixXd Psi_;      // discrete state transition matrix
    Eigen::RowVectorXd C_;     // output matrix
    Eigen::VectorXd L_;        // steady-state Kalman gain

    std::vector<double> freqs_;  // fᵢ [Hz] for each mode (for prediction)
    double t_current_ = 0.0;
    double estimate_  = 0.0;

    // Builds Ψ (block-diagonal matrix exponential) for the current modes.
    Eigen::MatrixXd buildPsi(const std::vector<Mode>& modes) const;

    // Builds C (output row vector) for n modes.
    Eigen::RowVectorXd buildC(int n_modes) const;

    // Builds process noise Q: larger for signal states of higher-freq modes.
    Eigen::MatrixXd buildQ(const std::vector<Mode>& modes) const;

    // Solves the DARE iteratively for the steady-state prediction covariance.
    // Returns L = Ψ P⁻ Cᵀ (C P⁻ Cᵀ + R)⁻¹
    Eigen::VectorXd solveDARE(const Eigen::MatrixXd& Psi,
                               const Eigen::MatrixXd& Q,
                               const Eigen::RowVectorXd& C) const;
};

}  // namespace spa_predictor
