#include "heave_estimator/fft_identifier.hpp"

#include <algorithm>
#include <numeric>
#include <stdexcept>

namespace heave_estimator {

FFTIdentifier::FFTIdentifier(int window_size, double sample_rate,
                             double peak_threshold, double min_omega)
    : sample_rate_(sample_rate), peak_threshold_(peak_threshold), min_omega_(min_omega)
{
    // Round down to nearest power of 2 (required by Cooley-Tukey FFT)
    window_size_ = 1;
    while (window_size_ * 2 <= window_size) window_size_ *= 2;
    buffer_.reserve(window_size_);
}

void FFTIdentifier::addSample(double az_imu)
{
    buffer_.push_back(az_imu);
    // Keep only the latest window_size_ samples
    if (static_cast<int>(buffer_.size()) > window_size_) {
        buffer_.erase(buffer_.begin());
    }
}

// Cooley-Tukey FFT (N must be power of 2)
void FFTIdentifier::fft(std::vector<std::complex<double>>& x)
{
    const int N = static_cast<int>(x.size());
    if (N <= 1) return;

    // Bit-reversal permutation
    for (int i = 1, j = 0; i < N; ++i) {
        int bit = N >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) std::swap(x[i], x[j]);
    }

    // FFT butterfly
    for (int len = 2; len <= N; len <<= 1) {
        double angle = -2.0 * M_PI / len;
        std::complex<double> wlen(std::cos(angle), std::sin(angle));
        for (int i = 0; i < N; i += len) {
            std::complex<double> w(1.0, 0.0);
            for (int j = 0; j < len / 2; ++j) {
                std::complex<double> u = x[i + j];
                std::complex<double> v = x[i + j + len / 2] * w;
                x[i + j] = u + v;
                x[i + j + len / 2] = u - v;
                w *= wlen;
            }
        }
    }
}

bool FFTIdentifier::identify(std::vector<HeaveMode>& modes)
{
    if (!isReady()) return false;

    const int N = window_size_;

    // Subtract mean before windowing to prevent the gravity DC offset (-9.81 m/s²)
    // from leaking into frequency bins via the Hann window sidelobes.
    double mean = 0.0;
    for (int i = 0; i < N; ++i) mean += buffer_[i];
    mean /= N;

    // Apply Hann window to reduce spectral leakage
    std::vector<std::complex<double>> X(N);
    for (int i = 0; i < N; ++i) {
        double w = 0.5 * (1.0 - std::cos(2.0 * M_PI * i / (N - 1)));
        X[i] = std::complex<double>((buffer_[i] - mean) * w, 0.0);
    }

    fft(X);

    // Frequency resolution [rad/s]
    double df_hz = sample_rate_ / N;          // Hz per bin
    double df_rad = 2.0 * M_PI * df_hz;       // rad/s per bin

    // Only use positive frequencies (bins 1 .. N/2-1)
    // Convert acceleration spectrum -> position spectrum: A_pos = A_acc / omega^2
    int n_pos = N / 2;
    std::vector<double> amp_pos(n_pos, 0.0);
    std::vector<double> phase_pos(n_pos, 0.0);

    for (int k = 1; k < n_pos; ++k) {
        double omega = k * df_rad;
        double amp_acc = (2.0 / N) * std::abs(X[k]);  // two-sided -> one-sided
        amp_pos[k] = amp_acc / (omega * omega);         // eq. (2) of paper
        phase_pos[k] = std::arg(X[k]) - M_PI;           // eq. (2) of paper
    }

    modes = detectPeaks(amp_pos, phase_pos, df_rad);
    return !modes.empty();
}

std::vector<HeaveMode> FFTIdentifier::detectPeaks(const std::vector<double>& amp_pos,
                                                    const std::vector<double>& phase_pos,
                                                    double df_rad)
{
    std::vector<HeaveMode> modes;
    if (amp_pos.empty()) return modes;

    double max_amp = *std::max_element(amp_pos.begin(), amp_pos.end());
    double threshold = peak_threshold_ * max_amp;
    last_threshold_amp_ = threshold;

    // Track the single strongest local-max candidate regardless of the
    // min_omega cutoff, so a caller can tell "there IS a real peak, it's
    // just below min_omega" apart from "there's genuinely no signal".
    last_candidate_freq_ = 0.0;
    last_candidate_amp_  = 0.0;

    int n = static_cast<int>(amp_pos.size());
    for (int k = 1; k < n - 1; ++k) {
        double omega_k = k * df_rad;
        bool is_local_max = amp_pos[k] > amp_pos[k - 1] && amp_pos[k] > amp_pos[k + 1];

        if (is_local_max && amp_pos[k] > threshold && amp_pos[k] > last_candidate_amp_) {
            last_candidate_freq_ = omega_k;
            last_candidate_amp_  = amp_pos[k];
        }

        if (omega_k < min_omega_) continue;   // reject DC/drift artifacts below wave range

        if (amp_pos[k] > threshold && is_local_max)
        {
            HeaveMode m;
            m.amplitude = amp_pos[k];
            m.frequency = omega_k;
            m.phase     = phase_pos[k];
            modes.push_back(m);
        }
    }

    // Sort by amplitude descending, keep top 6 to bound state size
    std::sort(modes.begin(), modes.end(),
              [](const HeaveMode& a, const HeaveMode& b) {
                  return a.amplitude > b.amplitude;
              });
    if (static_cast<int>(modes.size()) > 6) modes.resize(6);

    return modes;
}

}  // namespace heave_estimator
