// px4_global_origin: set PX4's global position origin once, then exit.
//
// Flying on motion capture, PX4 knows its local position (external vision)
// but has no global reference, which some checks and modes want (home
// position, global setpoints, ...). This node sends
// VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN with the configured lat/lon/alt, resending
// until PX4 acknowledges it, then exits.
//
// Adapted from the lab's time_synchro_gps_sys. Its other job, setting the
// computer's clock from the PX4 GPS time, is dropped: indoors there is no GPS
// fix (it never completes), and NTP keeps the clock right.

#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>

using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleCommandAck;

class Px4GlobalOrigin : public rclcpp::Node {
public:
    Px4GlobalOrigin() : Node("px4_global_origin") {
        // No in-code defaults (config/px4_global_origin.yaml): a missing key throws.
        lat_          = declare_parameter<double>("latitude_deg");
        lon_          = declare_parameter<double>("longitude_deg");
        alt_          = declare_parameter<double>("altitude_m");
        const auto pf = declare_parameter<std::string>("fmu_prefix");
        targetSystem_ = static_cast<int>(declare_parameter<int64_t>("target_system"));
        const double period = declare_parameter<double>("resend_period_s");
        giveUpAfter_  = declare_parameter<double>("give_up_after_s");

        auto px4Qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
        cmdPub_ = create_publisher<VehicleCommand>(pf + "/fmu/in/vehicle_command", 10);
        ackSub_ = create_subscription<VehicleCommandAck>(pf + "/fmu/out/vehicle_command_ack", px4Qos,
            [this](VehicleCommandAck::SharedPtr ack) { onAck(*ack); });

        start_ = now();
        timer_ = create_wall_timer(std::chrono::duration<double>(period), [this] { send(); });

        RCLCPP_INFO(get_logger(), "Setting PX4 global origin to lat %.8f lon %.8f alt %.2f (system %d, %s/fmu/...)",
                    lat_, lon_, alt_, targetSystem_, pf.c_str());
    }

private:
    void send() {
        if ((now() - start_).seconds() > giveUpAfter_) {
            RCLCPP_ERROR(get_logger(), "PX4 never accepted SET_GPS_GLOBAL_ORIGIN within %.0f s: giving up", giveUpAfter_);
            rclcpp::shutdown();
            return;
        }
        VehicleCommand msg{};
        msg.timestamp        = get_clock()->now().nanoseconds() / 1000;
        msg.param5           = lat_;
        msg.param6           = lon_;
        msg.param7           = static_cast<float>(alt_);
        msg.command          = VehicleCommand::VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN;
        msg.target_system    = targetSystem_;
        msg.target_component = 1;
        msg.source_system    = 1;
        msg.source_component = 1;
        msg.confirmation     = 1;
        msg.from_external    = true;
        cmdPub_->publish(msg);
    }

    void onAck(const VehicleCommandAck& ack) {
        if (ack.command != VehicleCommand::VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN) return;
        switch (ack.result) {
            case VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED:
                RCLCPP_INFO(get_logger(), "PX4 accepted the global origin: done");
                rclcpp::shutdown();
                break;
            case VehicleCommandAck::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED:
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "SET_GPS_GLOBAL_ORIGIN temporarily rejected, retrying");
                break;
            default:
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "SET_GPS_GLOBAL_ORIGIN result %u, retrying", ack.result);
                break;
        }
    }

    double lat_, lon_, alt_, giveUpAfter_;
    int targetSystem_;
    rclcpp::Time start_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr cmdPub_;
    rclcpp::Subscription<VehicleCommandAck>::SharedPtr ackSub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Px4GlobalOrigin>());
    if (rclcpp::ok()) rclcpp::shutdown();
    return 0;
}
