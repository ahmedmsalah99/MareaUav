#include <mutex>
#include <array>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <cmath>

#include "spa_predictor/fft_identifier.hpp"
#include "spa_predictor/spa_observer.hpp"

using namespace spa_predictor;

static void quatToRPY(double w, double x, double y, double z,
                      double& roll, double& pitch, double& yaw)
{
    double sinr_cosp = 2.0 * (w * x + y * z);
    double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    double sinp = 2.0 * (w * y - z * x);
    pitch = (std::abs(sinp) >= 1.0) ? std::copysign(M_PI / 2.0, sinp) : std::asin(sinp);

    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    yaw = std::atan2(siny_cosp, cosy_cosp);
}

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

        dt_             = 1.0 / timer_rate;
        reident_period_ = reident_period;

        for (int i = 0; i < 3; ++i) {
            identifiers_.emplace_back(fft_window, timer_rate, peak_threshold, min_freq_hz);
            observers_.emplace_back(dt_, q_scale, r_meas);
        }

        // Subscription: just buffer the latest IMU orientation — no processing here.
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "imu/data", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(buf_mutex_);
                buf_quat_ = {msg->orientation.w, msg->orientation.x,
                             msg->orientation.y, msg->orientation.z};
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
        std::array<double, 4> q;
        double T_pred;
        {
            std::lock_guard<std::mutex> lk(buf_mutex_);
            if (!buf_has_data_) return;
            q = buf_quat_;
            T_pred = T_pred_;
        }

        double roll, pitch, yaw;
        quatToRPY(q[0], q[1], q[2], q[3], roll, pitch, yaw);

        double angles[3] = {roll, pitch, yaw};
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
                }
            }
            last_reident_time_ = now;
        }

        bool any_ready = false;
        double est[3]  = {0, 0, 0};
        double pred[3] = {0, 0, 0};

        for (int i = 0; i < 3; ++i) {
            if (observers_[i].update(angles[i], now)) {
                est[i]  = observers_[i].estimate();
                pred[i] = observers_[i].predict(T_pred);
                any_ready = true;
            }
        }

        if (!any_ready) return;

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

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr horizon_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr estimate_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr predict_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<FFTIdentifier> identifiers_;
    std::vector<SPAObserver>   observers_;

    std::mutex              buf_mutex_;
    std::array<double, 4>   buf_quat_{1.0, 0.0, 0.0, 0.0};
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
