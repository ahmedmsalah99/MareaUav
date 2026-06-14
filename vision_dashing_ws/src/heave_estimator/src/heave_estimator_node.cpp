#include <mutex>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>

#include "heave_estimator/fft_identifier.hpp"
#include "heave_estimator/heave_ekf.hpp"

using namespace heave_estimator;

class HeaveEstimatorNode : public rclcpp::Node {
public:
    HeaveEstimatorNode() : Node("heave_estimator")
    {
        // Processing rate of the wall timer (Hz).
        // This is the effective sample rate seen by the FFT and EKF —
        // independent of however fast the IMU topic is published.
        double timer_rate     = declare_parameter("sample_rate",                30.0);
        int    fft_window     = declare_parameter("fft_window_size",           1024);
        double peak_threshold = declare_parameter("peak_threshold",            0.05);
        double reident_period = declare_parameter("reidentification_period_s", 40.0);
        double q_pos          = declare_parameter("q_pos",                     1e-4);
        double q_vel          = declare_parameter("q_vel",                     1e-3);
        double q_omega        = declare_parameter("q_omega",                   1e-6);
        double q_off          = declare_parameter("q_off",                     1e-8);
        double r_meas         = declare_parameter("r_meas",                    1e-4);

        dt_             = 1.0 / timer_rate;
        reident_period_ = reident_period;

        identifier_ = std::make_unique<FFTIdentifier>(fft_window, timer_rate, peak_threshold);
        ekf_        = std::make_unique<HeaveEKF>(dt_, q_pos, q_vel, q_omega, q_off, r_meas);

        // Subscription: just buffer incoming az readings — no processing here.
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "imu/data", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(buf_mutex_);
                buf_.push_back(msg->linear_acceleration.z);
            });

        pos_pub_ = create_publisher<std_msgs::msg::Float64>("heave/position", 10);
        vel_pub_ = create_publisher<std_msgs::msg::Float64>("heave/velocity", 10);

        // Wall timer drives all processing at `timer_rate` Hz.
        auto period = std::chrono::nanoseconds(static_cast<int64_t>(1e9 / timer_rate));
        timer_ = create_wall_timer(period,
            std::bind(&HeaveEstimatorNode::timerCallback, this));

        RCLCPP_INFO(get_logger(),
            "HeaveEstimatorNode: processing at %.1f Hz | FFT window %d samples (%.1f s)",
            timer_rate, fft_window, fft_window / timer_rate);
    }

private:
    void timerCallback()
    {
        // Swap the live buffer out under the lock so the subscriber
        // can keep writing without blocking.
        std::vector<double> samples;
        {
            std::lock_guard<std::mutex> lk(buf_mutex_);
            samples.swap(buf_);
        }

        if (samples.empty()) return;

        // Average all IMU readings that arrived since the last timer fire.
        // This downsamples to timer_rate Hz while reducing noise by sqrt(N).
        double az = 0.0;
        for (double s : samples) az += s;
        az /= static_cast<double>(samples.size());

        identifier_->addSample(az);

        // Re-identification trigger
        double now = this->now().seconds();
        if (!ekf_->isInitialized() || (now - last_reident_time_) >= reident_period_) {
            std::vector<HeaveMode> modes;
            if (identifier_->identify(modes)) {
                ekf_->initialize(modes);
                last_reident_time_ = now;
                RCLCPP_INFO(get_logger(), "Identified %zu heave mode(s).", modes.size());
                for (size_t i = 0; i < modes.size(); ++i)
                    RCLCPP_INFO(get_logger(),
                        "  Mode %zu: A=%.4f m, omega=%.4f rad/s, phi=%.4f rad",
                        i, modes[i].amplitude, modes[i].frequency, modes[i].phase);
            }
        }

        if (!ekf_->update(az)) return;

        std_msgs::msg::Float64 pos_msg, vel_msg;
        pos_msg.data = ekf_->heavePosision();
        vel_msg.data = ekf_->heaveVelocity();
        pos_pub_->publish(pos_msg);
        vel_pub_->publish(vel_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr    pos_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr    vel_pub_;
    rclcpp::TimerBase::SharedPtr                            timer_;

    std::unique_ptr<FFTIdentifier> identifier_;
    std::unique_ptr<HeaveEKF>      ekf_;

    std::mutex          buf_mutex_;
    std::vector<double> buf_;

    double dt_                  = 1.0 / 30.0;
    double reident_period_      = 40.0;
    double last_reident_time_   = 0.0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HeaveEstimatorNode>());
    rclcpp::shutdown();
    return 0;
}
