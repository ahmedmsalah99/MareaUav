#pragma once

#include <vector>
#include <complex>
#include <cmath>

namespace spa_predictor {

struct Mode {
    double amplitude;   // Aᵢ  [same units as input signal]
    double frequency;   // fᵢ  [Hz]
    double phase;       // φᵢ  [rad]
};

// Identifies dominant sinusoidal modes in a buffered scalar signal via FFT.
// Unlike the heave estimator, no ω² conversion is needed — the signal is
// the angle itself, so the amplitude spectrum gives Aᵢ directly.
class FFTIdentifier {
public:
    // window_size    : number of samples per FFT (rounded down to power of 2)
    // sample_rate    : Hz
    // peak_threshold : fraction of the max amplitude bin to consider a peak
    // min_freq_hz    : discard peaks below this frequency [Hz] (rejects DC/drift artifacts)
    FFTIdentifier(int window_size, double sample_rate,
                  double peak_threshold = 0.05, double min_freq_hz = 0.05);

    void addSample(double s);
    bool isReady() const { return static_cast<int>(buffer_.size()) >= window_size_; }

    // Runs FFT + peak detection. Returns true if at least one mode found.
    bool identify(std::vector<Mode>& modes);

private:
    int    window_size_;
    double sample_rate_;
    double peak_threshold_;
    double min_freq_hz_;
    std::vector<double> buffer_;

    void fft(std::vector<std::complex<double>>& x);
    std::vector<Mode> detectPeaks(const std::vector<double>& amp,
                                   const std::vector<double>& phase,
                                   double df_hz);
};

}  // namespace spa_predictor
