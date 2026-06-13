#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <cmath>

#include "spa_predictor/fft_identifier.hpp"
#include "spa_predictor/spa_observer.hpp"

using namespace spa_predictor;

// Converts quaternion (w,x,y,z) to roll/pitch/yaw [rad].
static void quatToRPY(double w, double x, double y, double z,
                      double& roll, double& pitch, double& yaw)
{
    // Roll (x-axis rotation)
    double sinr_cosp = 2.0 * (w * x + y * z);
    double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    double sinp = 2.0 * (w * y - z * x);
    pitch = (std::abs(sinp) >= 1.0) ? std::copysign(M_PI / 2.0, sinp) : std::asin(sinp);

    // Yaw (z-axis rotation)
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    yaw = std::atan2(siny_cosp, cosy_cosp);
}

class SPAPredictorNode : public rclcpp::Node {
public:
    SPAPredictorNode() : Node("spa_predictor")
    {
        double sample_rate    = declare_parameter("sample_rate",             100.0);
        int    fft_window     = declare_parameter("fft_window_size",         1024);
        double peak_threshold = declare_parameter("peak_threshold",          0.05);
        double reident_period = declare_parameter("reidentification_period_s", 60.0);
        double q_scale        = declare_parameter("q_scale",                 1e-4);
        double r_meas         = declare_parameter("r_meas",                  1e-4);
        T_pred_               = declare_parameter("prediction_horizon_s",    1.0);

        dt_             = 1.0 / sample_rate;
        reident_period_ = reident_period;

        // One FFT identifier and one SPA observer per angle
        for (int i = 0; i < 3; ++i) {
            identifiers_.emplace_back(fft_window, sample_rate, peak_threshold);
            observers_.emplace_back(dt_, q_scale, r_meas);
        }

        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "imu/data", 10,
            std::bind(&SPAPredictorNode::imuCallback, this, std::placeholders::_1));

        estimate_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "spa/estimate", 10);
        predict_pub_  = create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "spa/prediction", 10);

        RCLCPP_INFO(get_logger(),
                    "spa_predictor started — prediction horizon: %.2f s", T_pred_);
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        double roll, pitch, yaw;
        quatToRPY(msg->orientation.w, msg->orientation.x,
                  msg->orientation.y, msg->orientation.z,
                  roll, pitch, yaw);

        double angles[3] = {roll, pitch, yaw};
        double now = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

        // Feed each angle into its identifier and observer
        for (int i = 0; i < 3; ++i) {
            identifiers_[i].addSample(angles[i]);
        }

        // Re-identification trigger
        bool do_reident = (now - last_reident_time_ >= reident_period_)
                          || !observers_[0].isInitialized();

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

        // Observer update + publish
        bool any_ready = false;
        double est[3] = {0, 0, 0};
        double pred[3] = {0, 0, 0};

        for (int i = 0; i < 3; ++i) {
            if (observers_[i].update(angles[i], now)) {
                est[i]  = observers_[i].estimate();
                pred[i] = observers_[i].predict(T_pred_);
                any_ready = true;
            }
        }

        if (!any_ready) return;

        auto stamp = msg->header.stamp;

        auto est_msg = geometry_msgs::msg::Vector3Stamped();
        est_msg.header.stamp    = stamp;
        est_msg.header.frame_id = "vessel";
        est_msg.vector.x = est[0];   // roll
        est_msg.vector.y = est[1];   // pitch
        est_msg.vector.z = est[2];   // yaw
        estimate_pub_->publish(est_msg);

        auto pred_msg = geometry_msgs::msg::Vector3Stamped();
        pred_msg.header.stamp    = stamp;
        pred_msg.header.frame_id = "vessel";
        pred_msg.vector.x = pred[0];  // roll
        pred_msg.vector.y = pred[1];  // pitch
        pred_msg.vector.z = pred[2];  // yaw
        predict_pub_->publish(pred_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr estimate_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr predict_pub_;

    std::vector<FFTIdentifier> identifiers_;
    std::vector<SPAObserver>   observers_;

    double dt_             = 0.01;
    double reident_period_ = 60.0;
    double last_reident_time_ = 0.0;
    double T_pred_         = 1.0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SPAPredictorNode>());
    rclcpp::shutdown();
    return 0;
}
