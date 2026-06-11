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
    // window_size: number of samples per FFT (should be power of 2)
    // sample_rate: Hz
    // peak_threshold: fraction of max amplitude to consider a peak significant
    FFTIdentifier(int window_size, double sample_rate, double peak_threshold = 0.05);

    void addSample(double az_imu);
    bool isReady() const { return static_cast<int>(buffer_.size()) >= window_size_; }

    // Runs identification. Returns true if at least one mode found.
    bool identify(std::vector<HeaveMode>& modes);

private:
    int window_size_;
    double sample_rate_;
    double peak_threshold_;
    std::vector<double> buffer_;

    // Cooley-Tukey in-place FFT (N must be power of 2)
    void fft(std::vector<std::complex<double>>& x);

    std::vector<HeaveMode> detectPeaks(const std::vector<double>& amp_pos,
                                        const std::vector<double>& phase_pos,
                                        double freq_res);
};

}  // namespace heave_estimator
