#pragma once

#include <vector>
#include <complex>
#include <cmath>

namespace heave_estimator {

struct HeaveMode {
    double amplitude;   // Aj [m]
    double frequency;   // omega_j [rad/s]
    double phase;       // phi_j [rad]
};

// Identifies dominant heave modes from a buffered IMU acceleration signal via FFT.
// Converts acceleration spectrum -> position spectrum, then peak-detects modes.
class FFTIdentifier {
public:
    // window_size    : number of samples per FFT (should be power of 2)
    // sample_rate    : Hz
    // peak_threshold : fraction of max amplitude to consider a peak significant
    // min_omega      : discard peaks below this frequency [rad/s] (rejects DC/drift artifacts)
    FFTIdentifier(int window_size, double sample_rate,
                  double peak_threshold = 0.05, double min_omega = 0.5);

    void addSample(double az_imu);
    bool isReady() const { return static_cast<int>(buffer_.size()) >= window_size_; }

    // Warm-up progress, for diagnostics: how many samples buffered vs. how
    // many identify() needs (== window_size_ once rounded to a power of 2).
    int bufferedSamples() const { return static_cast<int>(buffer_.size()); }
    int windowSize() const { return window_size_; }

    // Runs identification. Returns true if at least one mode found.
    bool identify(std::vector<HeaveMode>& modes);

    // Diagnostics from the most recent identify() call, regardless of
    // whether it found any accepted mode — lets a caller see e.g. "the
    // strongest candidate peak exists, but its frequency is below
    // min_omega" instead of just "found nothing."
    double lastCandidateFreqRadS() const { return last_candidate_freq_; }
    double lastCandidateAmp() const { return last_candidate_amp_; }
    double lastPeakThresholdAmp() const { return last_threshold_amp_; }

private:
    int    window_size_;
    double sample_rate_;
    double peak_threshold_;
    double min_omega_;
    std::vector<double> buffer_;

    double last_candidate_freq_ = 0.0;
    double last_candidate_amp_  = 0.0;
    double last_threshold_amp_  = 0.0;

    // Cooley-Tukey in-place FFT (N must be power of 2)
    void fft(std::vector<std::complex<double>>& x);

    std::vector<HeaveMode> detectPeaks(const std::vector<double>& amp_pos,
                                        const std::vector<double>& phase_pos,
                                        double freq_res);
};

}  // namespace heave_estimator
