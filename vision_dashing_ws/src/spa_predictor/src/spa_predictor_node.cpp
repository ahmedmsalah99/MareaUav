#include <mutex>
#include <array>

#include <rclcpp/rclcpp.hpp>
#include <ulisse_msgs/msg/nav_filter_data.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <cmath>

#include "spa_predictor/fft_identifier.hpp"
#include "spa_predictor/spa_observer.hpp"

using namespace spa_predictor;

class SPAPredictorNode : public rclcpp::Node {
public:
    SPAPredictorNode() : Node("spa_predictor")
    {
        double timer_rate     = declare_parameter("sample_rate",                  30.0);
        int    fft_window     = declare_parameter("fft_window_size",             1024);
        double peak_threshold = declare_parameter("peak_threshold",              0.05);
        double min_freq_hz    = declare_parameter("min_freq_hz",                 0.05);
        double reident_period = declare_parameter("reidentification_period_s",   60.0);
        double q_scale        = declare_parameter("q_scale",                     1e-4);
        double r_meas         = declare_parameter("r_meas",                      1e-4);
        // Static fallback horizon, used until the guidance node starts publishing
        // a dynamic one on spa/prediction_horizon.
        T_pred_               = declare_parameter("prediction_horizon_s",        1.0);
        // Calm-sea fallback, per axis: when the FFT window is full, no mode is
        // found and the buffered angle's std dev is below this [rad], the axis
        // is treated as still: estimate = prediction = the measured angle (for a
        // tilted static pad, its tilt). No in-code default: set it from the launch file.
        calm_angle_std_max_   = declare_parameter<double>("calm_angle_std_max_rad");

        dt_             = 1.0 / timer_rate;
        reident_period_ = reident_period;

        for (int i = 0; i < 3; ++i) {
            identifiers_.emplace_back(fft_window, timer_rate, peak_threshold, min_freq_hz);
            observers_.emplace_back(dt_, q_scale, r_meas);
        }

        // Subscription: just buffer the latest roll/pitch/yaw — no processing here.
        // Real nav-filter attitude output (not the landingplat plugin's simulated
        // IMU) — already Euler angles, no quaternion conversion needed.
        nav_filter_sub_ = create_subscription<ulisse_msgs::msg::NavFilterData>(
            "/ulisse/nav_filter/data", rclcpp::SensorDataQoS(),
            [this](const ulisse_msgs::msg::NavFilterData::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(buf_mutex_);
                buf_rpy_ = {msg->bodyframe_angular_position.roll,
                            msg->bodyframe_angular_position.pitch,
                            msg->bodyframe_angular_position.yaw};
                buf_has_data_ = true;
            });

        // Dynamic prediction horizon T = l/v published by the guidance node.
        // When fresh, it overrides the static prediction_horizon_s parameter.
        horizon_sub_ = create_subscription<std_msgs::msg::Float64>(
            "spa/prediction_horizon", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(buf_mutex_);
                if (msg->data >= 0.0) T_pred_ = msg->data;
            });

        estimate_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "spa/estimate", 10);
        predict_pub_  = create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "spa/prediction", 10);

        // Wall timer drives all processing at timer_rate Hz.
        auto period = std::chrono::nanoseconds(static_cast<int64_t>(1e9 / timer_rate));
        timer_ = create_wall_timer(period,
            std::bind(&SPAPredictorNode::timerCallback, this));

        RCLCPP_INFO(get_logger(),
            "SPAPredictorNode: processing at %.1f Hz | FFT window %d samples (%.1f s) | "
            "min_freq %.3f Hz | prediction horizon %.2f s",
            timer_rate, fft_window, fft_window / timer_rate, min_freq_hz, T_pred_);
    }

private:
    void timerCallback()
    {
        std::array<double, 3> rpy;
        double T_pred;
        {
            std::lock_guard<std::mutex> lk(buf_mutex_);
            if (!buf_has_data_) return;
            rpy = buf_rpy_;
            T_pred = T_pred_;
        }

        double angles[3] = {rpy[0], rpy[1], rpy[2]};
        double now = this->now().seconds();

        for (int i = 0; i < 3; ++i)
            identifiers_[i].addSample(angles[i]);

        // Re-identification trigger
        bool do_reident = !observers_[0].isInitialized()
                          || (now - last_reident_time_) >= reident_period_;

        if (do_reident) {
            const char* names[3] = {"roll", "pitch", "yaw"};
            for (int i = 0; i < 3; ++i) {
                std::vector<Mode> modes;
                if (identifiers_[i].identify(modes)) {
                    observers_[i].initialize(modes, now);
                    RCLCPP_INFO(get_logger(), "%s: identified %zu mode(s)",
                                names[i], modes.size());
                    // Per-mode breakdown — hitting the 6-mode cap repeatedly
                    // usually means peak_threshold is too permissive and
                    // real sensor noise is being picked up as extra "modes"
                    // instead of just the 1-2 genuine dominant frequencies.
                    for (size_t m = 0; m < modes.size(); ++m)
                        RCLCPP_INFO(get_logger(),
                            "  Mode %zu: A=%.5f rad, f=%.4f Hz (%.1fs period), phi=%.4f rad",
                            m, modes[m].amplitude, modes[m].frequency,
                            modes[m].frequency > 0.0 ? 1.0 / modes[m].frequency : 0.0,
                            modes[m].phase);
                }
            }
            last_reident_time_ = now;
        }

        bool any_ready = false;
        bool calm[3]   = {false, false, false};
        double est[3]  = {0, 0, 0};
        double pred[3] = {0, 0, 0};

        for (int i = 0; i < 3; ++i) {
            if (observers_[i].update(angles[i], now)) {
                est[i]  = observers_[i].estimate();
                pred[i] = observers_[i].predict(T_pred);
                any_ready = true;
            } else if (!observers_[i].isInitialized() && identifiers_[i].isReady()
                       && identifiers_[i].bufferStdDev() < calm_angle_std_max_) {
                est[i]  = angles[i];   // still axis: nothing periodic to predict
                pred[i] = angles[i];
                any_ready = true;
                calm[i] = true;
            }
        }

        if (!any_ready) return;

        if ((calm[0] || calm[1] || calm[2]) && (now - last_calm_log_time_) >= 5.0) {
            last_calm_log_time_ = now;
            RCLCPP_INFO(get_logger(), "Calm sea (no mode, quiet signal) on roll=%d pitch=%d yaw=%d -> publishing the measured angles.",
                        calm[0], calm[1], calm[2]);
        }

        auto stamp = this->now();

        auto est_msg = geometry_msgs::msg::Vector3Stamped();
        est_msg.header.stamp    = stamp;
        est_msg.header.frame_id = "vessel";
        est_msg.vector.x = est[0];
        est_msg.vector.y = est[1];
        est_msg.vector.z = est[2];
        estimate_pub_->publish(est_msg);

        auto pred_msg = geometry_msgs::msg::Vector3Stamped();
        pred_msg.header.stamp    = stamp;
        pred_msg.header.frame_id = "vessel";
        pred_msg.vector.x = pred[0];
        pred_msg.vector.y = pred[1];
        pred_msg.vector.z = pred[2];
        predict_pub_->publish(pred_msg);
    }

    rclcpp::Subscription<ulisse_msgs::msg::NavFilterData>::SharedPtr nav_filter_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr horizon_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr estimate_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr predict_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double calm_angle_std_max_ = 0.0;
    double last_calm_log_time_ = 0.0;
    std::vector<FFTIdentifier> identifiers_;
    std::vector<SPAObserver>   observers_;

    std::mutex              buf_mutex_;
    std::array<double, 3>   buf_rpy_{0.0, 0.0, 0.0};
    bool                    buf_has_data_ = false;

    double dt_                = 1.0 / 30.0;
    double reident_period_    = 60.0;
    double last_reident_time_ = 0.0;
    double T_pred_            = 1.0;   // active horizon (dynamic once guidance publishes)
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SPAPredictorNode>());
    rclcpp::shutdown();
    return 0;
}
