#include <cmath>
#include <mutex>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <ulisse_msgs/msg/nav_filter_data.hpp>

#include "heave_estimator/fft_identifier.hpp"
#include "heave_estimator/heave_ekf.hpp"

using namespace heave_estimator;

class HeaveEstimatorNode : public rclcpp::Node {
public:
    HeaveEstimatorNode() : Node("heave_estimator")
    {
        // Processing rate of the wall timer (Hz).
        // This is the effective sample rate seen by the FFT and EKF —
        // independent of however fast the nav filter publishes. The latest
        // received velocity is (re)sampled on every tick, so the FFT time base
        // really is this rate; previously a tick with no new data returned early
        // and skipped the sample entirely, so the true sample rate silently
        // differed from this value and every identified frequency was scaled
        // by that ratio.
        double timer_rate     = declare_parameter("sample_rate",                30.0);
        int    fft_window     = declare_parameter("fft_window_size",           1024);
        // Minimum peak height, as a fraction of the strongest peak, for a mode
        // to be accepted. At 0.05 the identifier also picks up a spurious
        // SUBHARMONIC at exactly half the true frequency: the Hann-windowed main
        // peak leaks downward, and the velocity->position conversion (A/omega)
        // then amplifies whatever lands at omega/2 by a further 2x, which is
        // enough to clear a 5% threshold. Being at half the wave frequency it
        // shows up downstream as a slow baseline wander rather than as an
        // obviously wrong mode. Rejecting it halves the position error and makes
        // the forward prediction essentially exact.
        double peak_threshold = declare_parameter("peak_threshold",            0.15);
        double min_omega      = declare_parameter("min_omega_rad_s",           0.5);
        double reident_period = declare_parameter("reidentification_period_s", 40.0);
        // Filter tuning, in VELOCITY units now that the measurement is m/s
        // rather than m/s^2.
        //
        // r_meas is the variance of the nav filter's vertical-velocity noise.
        // Observed jitter on /ulisse/nav_filter/data is of order 1e-3 m/s
        // against a heave amplitude of order 1e-2 m/s, so 1e-6 (std 1e-3 m/s).
        // To retune from data: log bodyframe_linear_velocity[2] with the
        // platform still and take its variance.
        //
        // The mode dynamics are an EXACT model of a stationary sinusoid, so the
        // process noise only has to cover how fast the sea state itself drifts
        // (tens of seconds), not the oscillation. Rule of thumb: keep
        // sqrt(q_vel * rate) well under the heave velocity amplitude -- here
        // sqrt(1e-8 * 10) = 3e-4 m/s per second, so about 1e-2 m/s of drift
        // allowed over a 40 s re-identification period.
        double q_pos          = declare_parameter("q_pos",                     1e-10);
        double q_vel          = declare_parameter("q_vel",                     1e-8);
        double q_omega        = declare_parameter("q_omega",                   1e-9);
        // The offset state absorbs any bias in the measured velocity, or a
        // genuine slow rise/fall of the platform. The oscillator states are
        // zero-mean by construction, so a constant has nowhere else to go; if
        // this is too small the offset freezes and the constant is forced into
        // the oscillators, which cannot represent it, and the amplitude grows.
        // Slow enough not to absorb the heave itself.
        double q_off          = declare_parameter("q_off",                     1e-9);
        double r_meas         = declare_parameter("r_meas",                    1e-6);
        // Divergence safeguards (see HeaveEKF): physical frequency cap, innovation
        // gate, and output sanity limits.
        double omega_max      = declare_parameter("omega_max_rad_s",           3.0);
        double nis_gate       = declare_parameter("nis_gate",                  25.0);
        double vel_limit      = declare_parameter("heave_vel_limit_mps",       5.0);
        // Forward-prediction horizon [s] for heave/prediction_* (used downstream
        // to lead the pad by the drone's reaction lag). Static fallback, used
        // until the guidance node starts publishing a dynamic one on
        // spa/prediction_horizon (same T = l/v signal spa_predictor already
        // consumes for its own prediction window).
        pred_horizon_       = declare_parameter("prediction_horizon_s",        0.3);
        // Second, INDEPENDENT prediction horizon: the drone's own control +
        // communication delay (setpoint compute + link + inner-loop response),
        // used only by the compensation-phase vertical tracking (comp()) so it
        // leads the pad by "how far ahead the command actually lands", not by
        // the (unrelated) time-to-platform used for the landing GO/NO-GO
        // decision. Static — this is a system latency, not something that
        // shrinks as the drone approaches the pad.
        control_comm_delay_ = declare_parameter("control_comm_delay_s",        0.15);
        // Sign applied to bodyframe_linear_velocity[2] to bring it into the
        // UP-positive convention that comp() expects of the published heave.
        //
        // +1 is correct for this system, and the landingplat plugin settles it:
        // it dead-reckons the platform's GAZEBO WORLD position straight from
        // this field, with no sign flip anywhere —
        //     v_body(lin_vel_filt[0], lin_vel_filt[1], lin_vel_filt[2]);
        //     _sim_position += (orientation * v_body) * _dt;
        // and Gazebo's world frame is z-UP, so a positive value moves the pad
        // up. (This defaulted to -1 on the assumption that a marine nav filter
        // would use a z-down body frame. It does not here, and the published
        // heave came out inverted — right in magnitude, opposite in sign, which
        // makes comp() lead the pad the wrong way and ADD to the relative
        // motion instead of cancelling it.)
        heave_sign_         = declare_parameter("heave_sign",                  1.0);

        dt_             = 1.0 / timer_rate;
        reident_period_ = reident_period;
        min_omega_      = min_omega;

        identifier_ = std::make_unique<FFTIdentifier>(fft_window, timer_rate, peak_threshold, min_omega);
        ekf_        = std::make_unique<HeaveEKF>(dt_, q_pos, q_vel, q_omega, q_off, r_meas,
                                                 omega_max, nis_gate, vel_limit);

        // Subscription: hold the latest vertical velocity — no processing here.
        // Same source and pattern spa_predictor uses for attitude: the nav
        // filter has already fused the IMU, removed gravity and compensated
        // attitude, so this is the pad's actual vertical motion rather than a
        // raw body-frame accelerometer axis.
        nav_filter_sub_ = create_subscription<ulisse_msgs::msg::NavFilterData>(
            "/ulisse/nav_filter/data", rclcpp::SensorDataQoS(),
            [this](const ulisse_msgs::msg::NavFilterData::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(buf_mutex_);
                latest_vel_    = heave_sign_ * msg->bodyframe_linear_velocity[2];
                have_vel_      = true;
            });

        // Dynamic prediction horizon T = l/v published by the guidance node.
        // When fresh, it overrides the static prediction_horizon_s parameter —
        // same signal/pattern as spa_predictor's own horizon_sub_.
        horizon_sub_ = create_subscription<std_msgs::msg::Float64>(
            "spa/prediction_horizon", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(buf_mutex_);
                if (msg->data >= 0.0) pred_horizon_ = msg->data;
            });

        pos_pub_ = create_publisher<std_msgs::msg::Float64>("heave/position", 10);
        vel_pub_ = create_publisher<std_msgs::msg::Float64>("heave/velocity", 10);
        pred_pos_pub_ = create_publisher<std_msgs::msg::Float64>("heave/prediction_position", 10);
        pred_vel_pub_ = create_publisher<std_msgs::msg::Float64>("heave/prediction_velocity", 10);
        // Comp-phase prediction, led by control_comm_delay_ instead of pred_horizon.
        pred_pos_comp_pub_ = create_publisher<std_msgs::msg::Float64>("heave/prediction_position_comp", 10);
        pred_vel_comp_pub_ = create_publisher<std_msgs::msg::Float64>("heave/prediction_velocity_comp", 10);

        // Wall timer drives all processing at `timer_rate` Hz.
        auto period = std::chrono::nanoseconds(static_cast<int64_t>(1e9 / timer_rate));
        timer_ = create_wall_timer(period,
            std::bind(&HeaveEstimatorNode::timerCallback, this));

        RCLCPP_INFO(get_logger(),
            "HeaveEstimatorNode: processing at %.1f Hz | FFT window %d samples (%.1f s) | "
            "static prediction horizon %.2f s (overridden by spa/prediction_horizon when fresh) | "
            "comp-phase (control+comm delay) horizon %.2f s",
            timer_rate, fft_window, fft_window / timer_rate, pred_horizon_, control_comm_delay_);
    }

private:
    void timerCallback()
    {
        // Take a consistent snapshot of the latest velocity and of the
        // (possibly dynamically-updated) prediction horizon, without holding the
        // lock across any processing.
        double heave_vel;
        double pred_horizon;
        bool   have_vel;
        {
            std::lock_guard<std::mutex> lk(buf_mutex_);
            heave_vel    = latest_vel_;
            pred_horizon = pred_horizon_;
            have_vel     = have_vel_;
        }

        // Nothing has arrived yet — there is no sample to take.
        if (!have_vel) return;

        // Re-sample the latest value on every tick, even if the nav filter has
        // not published since the last one. That keeps the FFT time base and the
        // EKF's dt exactly at timer_rate; skipping ticks instead would make the
        // real sample rate depend on the publisher and scale every identified
        // frequency by that ratio. Set sample_rate at or below the nav filter
        // rate so this stays a mild zero-order hold rather than heavy repetition.
        identifier_->addSample(heave_vel);

        double now = this->now().seconds();

        // Warm-up progress — throttled to ~1 Hz, only while the FFT window
        // isn't full yet (isReady() false means identify() can't run at all).
        if (!identifier_->isReady() && (now - last_diag_log_time_) >= 1.0) {
            last_diag_log_time_ = now;
            RCLCPP_INFO(get_logger(), "Heave warm-up: %d/%d samples buffered.",
                identifier_->bufferedSamples(), identifier_->windowSize());
        }

        // Re-identification trigger
        if (!ekf_->isInitialized() || (now - last_reident_time_) >= reident_period_) {
            std::vector<HeaveMode> modes;
            if (identifier_->identify(modes)) {
                ekf_->initialize(modes, identifier_->lastMean());
                last_reident_time_ = now;
                RCLCPP_INFO(get_logger(),
                    "Identified %zu heave mode(s). Velocity DC mean %.5f m/s "
                    "(seeds the EKF offset state on first init).",
                    modes.size(), identifier_->lastMean());
                for (size_t i = 0; i < modes.size(); ++i)
                    RCLCPP_INFO(get_logger(),
                        "  Mode %zu: A=%.4f m, omega=%.4f rad/s, phi=%.4f rad",
                        i, modes[i].amplitude, modes[i].frequency, modes[i].phase);
            } else if (identifier_->isReady() && (now - last_diag_log_time_) >= 2.0) {
                // Window is full but nothing passed both the amplitude
                // threshold and the min_omega cutoff — report the strongest
                // candidate that DID pass the threshold (if any), so it's
                // clear whether a real peak exists but is being rejected for
                // being too slow (below min_omega_rad_s), versus there being
                // no signal above the amplitude threshold at all.
                last_diag_log_time_ = now;
                if (identifier_->lastCandidateAmp() > 0.0) {
                    double f_hz = identifier_->lastCandidateFreqRadS() / (2.0 * M_PI);
                    RCLCPP_WARN(get_logger(),
                        "Heave identify() found 0 modes, but a candidate peak exists: "
                        "omega=%.4f rad/s (%.4f Hz, %.1fs period), amp=%.4f — "
                        "rejected because it's below min_omega_rad_s (currently %.4f rad/s). "
                        "Amplitude threshold was %.6f.",
                        identifier_->lastCandidateFreqRadS(), f_hz,
                        f_hz > 0.0 ? 1.0 / f_hz : 0.0,
                        identifier_->lastCandidateAmp(), min_omega_, identifier_->lastPeakThresholdAmp());
                } else {
                    RCLCPP_WARN(get_logger(),
                        "Heave identify() found 0 modes and no candidate peak passed the "
                        "amplitude threshold (%.6f) at all — no discernible periodic signal "
                        "in the buffered velocity data.",
                        identifier_->lastPeakThresholdAmp());
                }
            }
        }

        if (!ekf_->isInitialized()) {
            // The window is full but no wave mode was found: nothing periodic to
            // model or predict. Publish the measurement itself (a still platform,
            // e.g. the mocap pad, reads as heave 0 / velocity ~0).
            if (identifier_->isReady()) {
                publishMeasured(heave_vel);
                if ((now - last_calm_log_time_) >= 5.0) {
                    last_calm_log_time_ = now;
                    RCLCPP_INFO(get_logger(), "No heave mode: publishing the measured velocity %.4f m/s (position 0) as estimate and prediction.", heave_vel);
                }
            }
            return;
        }

        if (!ekf_->update(heave_vel)) return;

        // Divergence self-heal: if the EKF output is implausible, do NOT publish
        // (downstream freshness turns heaveValid false), and force an immediate
        // re-identification instead of waiting for the periodic one.
        if (!ekf_->isHealthy()) {
            RCLCPP_WARN(get_logger(),
                "Heave EKF diverged (pos=%.3f vel=%.3f, identified amplitude %.3f m) — "
                "withholding estimate and re-identifying.",
                ekf_->heavePosision(), ekf_->heaveVelocity(), ekf_->identifiedAmplitude());
            ekf_->reset();                               // discard diverged state (no reuse)
            last_reident_time_ = now - reident_period_;  // trip the re-ident branch next tick
            return;
        }

        std_msgs::msg::Float64 pos_msg, vel_msg;
        pos_msg.data = ekf_->heavePosision();
        vel_msg.data = ekf_->heaveVelocity();
        pos_pub_->publish(pos_msg);
        vel_pub_->publish(vel_msg);

        // Forward prediction: where the pad heave will be pred_horizon s from now.
        double pred_pos, pred_vel;
        ekf_->predict(pred_horizon, pred_pos, pred_vel);
        std_msgs::msg::Float64 pred_pos_msg, pred_vel_msg;
        pred_pos_msg.data = pred_pos;
        pred_vel_msg.data = pred_vel;
        pred_pos_pub_->publish(pred_pos_msg);
        pred_vel_pub_->publish(pred_vel_msg);

        // Second forward prediction, at the (static) control+comm delay horizon,
        // for the compensation-phase vertical tracker.
        double pred_pos_comp, pred_vel_comp;
        ekf_->predict(control_comm_delay_, pred_pos_comp, pred_vel_comp);
        std_msgs::msg::Float64 pred_pos_comp_msg, pred_vel_comp_msg;
        pred_pos_comp_msg.data = pred_pos_comp;
        pred_vel_comp_msg.data = pred_vel_comp;
        pred_pos_comp_pub_->publish(pred_pos_comp_msg);
        pred_vel_comp_pub_->publish(pred_vel_comp_msg);
    }

    // No wave mode: heave position 0, velocity = measured, and the predictions
    // equal the current values (nothing periodic to extrapolate).
    void publishMeasured(double heave_vel)
    {
        std_msgs::msg::Float64 pos, vel;
        pos.data = 0.0;
        vel.data = heave_vel;
        pos_pub_->publish(pos);           vel_pub_->publish(vel);
        pred_pos_pub_->publish(pos);      pred_vel_pub_->publish(vel);
        pred_pos_comp_pub_->publish(pos); pred_vel_comp_pub_->publish(vel);
    }

    rclcpp::Subscription<ulisse_msgs::msg::NavFilterData>::SharedPtr nav_filter_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr   horizon_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr    pos_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr    vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr    pred_pos_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr    pred_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr    pred_pos_comp_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr    pred_vel_comp_pub_;
    rclcpp::TimerBase::SharedPtr                            timer_;

    std::unique_ptr<FFTIdentifier> identifier_;
    std::unique_ptr<HeaveEKF>      ekf_;

    std::mutex buf_mutex_;
    double     latest_vel_ = 0.0;   // latest nav-filter vertical velocity [m/s], up-positive
    bool       have_vel_   = false;

    double dt_                  = 1.0 / 30.0;
    double reident_period_      = 40.0;
    double last_reident_time_   = 0.0;
    double pred_horizon_        = 0.3;
    double control_comm_delay_  = 0.15;
    double min_omega_           = 0.5;
    double heave_sign_          = 1.0;
    double last_diag_log_time_  = 0.0;
    double last_calm_log_time_  = 0.0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HeaveEstimatorNode>());
    rclcpp::shutdown();
    return 0;
}
