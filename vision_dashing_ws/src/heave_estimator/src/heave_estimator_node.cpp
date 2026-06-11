#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/float64.hpp>

#include "heave_estimator/fft_identifier.hpp"
#include "heave_estimator/heave_ekf.hpp"

using namespace heave_estimator;

class HeaveEstimatorNode : public rclcpp::Node {
public:
    HeaveEstimatorNode()
        : Node("heave_estimator")
    {
        // Parameters
        double sample_rate     = declare_parameter("sample_rate", 100.0);         // Hz
        int fft_window         = declare_parameter("fft_window_size", 1024);
        double peak_threshold  = declare_parameter("peak_threshold", 0.05);
        double reident_period  = declare_parameter("reidentification_period_s", 60.0);
        double q_pos           = declare_parameter("q_pos", 1e-4);
        double q_vel           = declare_parameter("q_vel", 1e-3);
        double q_omega         = declare_parameter("q_omega", 1e-6);
        double q_off           = declare_parameter("q_off", 1e-8);
        double r_meas          = declare_parameter("r_meas", 1e-4);

        dt_ = 1.0 / sample_rate;
        reident_period_ = reident_period;

        identifier_ = std::make_unique<FFTIdentifier>(fft_window, sample_rate, peak_threshold);
        ekf_ = std::make_unique<HeaveEKF>(dt_, q_pos, q_vel, q_omega, q_off, r_meas);

        // Subscriptions
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "imu/data", 10,
            std::bind(&HeaveEstimatorNode::imuCallback, this, std::placeholders::_1));

        // Publishers
        pos_pub_  = create_publisher<std_msgs::msg::Float64>("heave/position", 10);
        vel_pub_  = create_publisher<std_msgs::msg::Float64>("heave/velocity", 10);

        RCLCPP_INFO(get_logger(),
                    "HeaveEstimatorNode started. Collecting %d samples before first identification.",
                    fft_window);
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        double az = msg->linear_acceleration.z;
        identifier_->addSample(az);

        // Re-identification trigger
        double now = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        if (now - last_reident_time_ >= reident_period_ || !ekf_->isInitialized()) {
            std::vector<HeaveMode> modes;
            if (identifier_->identify(modes)) {
                ekf_->initialize(modes);
                last_reident_time_ = now;
                RCLCPP_INFO(get_logger(), "Identified %zu heave mode(s).", modes.size());
                for (size_t i = 0; i < modes.size(); ++i) {
                    RCLCPP_DEBUG(get_logger(),
                                 "  Mode %zu: A=%.4f m, omega=%.4f rad/s, phi=%.4f rad",
                                 i, modes[i].amplitude, modes[i].frequency, modes[i].phase);
                }
            }
        }

        if (!ekf_->update(az)) return;

        auto pos_msg = std_msgs::msg::Float64();
        auto vel_msg = std_msgs::msg::Float64();
        pos_msg.data = ekf_->heavePosision();
        vel_msg.data = ekf_->heaveVelocity();

        pos_pub_->publish(pos_msg);
        vel_pub_->publish(vel_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pos_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vel_pub_;

    std::unique_ptr<FFTIdentifier> identifier_;
    std::unique_ptr<HeaveEKF> ekf_;

    double dt_ = 0.01;
    double reident_period_ = 60.0;
    double last_reident_time_ = 0.0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HeaveEstimatorNode>());
    rclcpp::shutdown();
    return 0;
}
