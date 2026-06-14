#include "spa_predictor/fft_identifier.hpp"

#include <algorithm>

namespace spa_predictor {

FFTIdentifier::FFTIdentifier(int window_size, double sample_rate,
                             double peak_threshold, double min_freq_hz)
    : sample_rate_(sample_rate), peak_threshold_(peak_threshold), min_freq_hz_(min_freq_hz)
{
    // Round down to nearest power of 2 (required by Cooley-Tukey FFT)
    window_size_ = 1;
    while (window_size_ * 2 <= window_size) window_size_ *= 2;
    buffer_.reserve(window_size_);
}

void FFTIdentifier::addSample(double s)
{
    buffer_.push_back(s);
    if (static_cast<int>(buffer_.size()) > window_size_) {
        buffer_.erase(buffer_.begin());
    }
}

// Cooley-Tukey in-place FFT (N must be power of 2)
void FFTIdentifier::fft(std::vector<std::complex<double>>& x)
{
    const int N = static_cast<int>(x.size());
    if (N <= 1) return;

    for (int i = 1, j = 0; i < N; ++i) {
        int bit = N >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) std::swap(x[i], x[j]);
    }

    for (int len = 2; len <= N; len <<= 1) {
        double angle = -2.0 * M_PI / len;
        std::complex<double> wlen(std::cos(angle), std::sin(angle));
        for (int i = 0; i < N; i += len) {
            std::complex<double> w(1.0, 0.0);
            for (int j = 0; j < len / 2; ++j) {
                std::complex<double> u = x[i + j];
                std::complex<double> v = x[i + j + len / 2] * w;
                x[i + j]           = u + v;
                x[i + j + len / 2] = u - v;
                w *= wlen;
            }
        }
    }
}

bool FFTIdentifier::identify(std::vector<Mode>& modes)
{
    if (!isReady()) return false;

    const int N = window_size_;

    // Subtract mean before windowing to prevent any DC bias from leaking
    // into frequency bins via the Hann window sidelobes.
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

    // Frequency resolution [Hz/bin]
    double df = sample_rate_ / N;

    // Build one-sided amplitude and phase spectra (bins 1 .. N/2-1)
    int n_pos = N / 2;
    std::vector<double> amp(n_pos, 0.0);
    std::vector<double> phase(n_pos, 0.0);

    for (int k = 1; k < n_pos; ++k) {
        amp[k]   = (2.0 / N) * std::abs(X[k]);   // two-sided → one-sided
        phase[k] = std::arg(X[k]);                 // [rad], no ω² conversion needed
    }

    modes = detectPeaks(amp, phase, df);
    return !modes.empty();
}

std::vector<Mode> FFTIdentifier::detectPeaks(const std::vector<double>& amp,
                                               const std::vector<double>& phase,
                                               double df_hz)
{
    std::vector<Mode> modes;
    if (amp.empty()) return modes;

    double max_amp  = *std::max_element(amp.begin(), amp.end());
    double threshold = peak_threshold_ * max_amp;

    int n = static_cast<int>(amp.size());
    for (int k = 1; k < n - 1; ++k) {
        if (k * df_hz < min_freq_hz_) continue;   // reject DC/drift artifacts
        if (amp[k] > threshold &&
            amp[k] > amp[k - 1] &&
            amp[k] > amp[k + 1])
        {
            Mode m;
            m.amplitude = amp[k];
            m.frequency = k * df_hz;   // Hz
            m.phase     = phase[k];
            modes.push_back(m);
        }
    }

    // Keep top 6 by amplitude
    std::sort(modes.begin(), modes.end(),
              [](const Mode& a, const Mode& b) { return a.amplitude > b.amplitude; });
    if (static_cast<int>(modes.size()) > 6) modes.resize(6);

    return modes;
}

}  // namespace spa_predictor
