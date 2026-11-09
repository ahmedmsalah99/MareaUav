// mocap_bridge: motion-capture poses -> PX4 external vision + pad "sea state".
//
// Inputs (geometry_msgs/PoseStamped, ENU world, Z-up, body FLU), one per rigid
// body, as published by natnet_ros2 in the lab or by the landingplat plugin's
// fake mocap in M2M_MODE=3:
//   drone_pose_topic -> /fmu/in/vehicle_visual_odometry  (PX4 flies on mocap)
//   pad_pose_topic   -> /ulisse/nav_filter/data           (heave_estimator, spa_predictor, bridge)
//                    -> /adnav_node/imu_raw               (touchdown detector)
//
// The pad outputs mimic the catamaran's own sensors, so the guidance code runs
// unchanged: a static pad simply reads as a calm sea.

#include <array>
#include <cmath>
#include <memory>
#include <algorithm>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <ulisse_msgs/msg/nav_filter_data.hpp>

namespace {

constexpr double kG   = 9.81;

struct Quat { double w, x, y, z; };

Quat mul(const Quat& a, const Quat& b) {
    return {a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
            a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
            a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
            a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w};
}

// Roll/pitch/yaw (ZYX) of a body-to-world quaternion.
std::array<double, 3> toRpy(const Quat& q) {
    const double roll  = std::atan2(2.0 * (q.w*q.x + q.y*q.z), 1.0 - 2.0 * (q.x*q.x + q.y*q.y));
    const double sp    = std::clamp(2.0 * (q.w*q.y - q.z*q.x), -1.0, 1.0);
    const double pitch = std::asin(sp);
    const double yaw   = std::atan2(2.0 * (q.w*q.z + q.x*q.y), 1.0 - 2.0 * (q.y*q.y + q.z*q.z));
    return {roll, pitch, yaw};
}

double wrap(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

}  // namespace

class MocapBridge : public rclcpp::Node {
public:
    MocapBridge() : Node("mocap_bridge") {
        // No in-code defaults (config/mocap_bridge.yaml): a missing key throws.
        const auto droneTopic = declare_parameter<std::string>("drone_pose_topic");
        const auto padTopic   = declare_parameter<std::string>("pad_pose_topic");
        worldYaw_        = declare_parameter<double>("world_yaw_offset_rad");
        posVar_          = std::pow(declare_parameter<double>("position_std_m"), 2);
        oriVar_          = std::pow(declare_parameter<double>("orientation_std_rad"), 2);
        timeout_         = declare_parameter<double>("pose_timeout_s");
        const double navRate = declare_parameter<double>("pad_nav_rate_hz");
        const double imuRate = declare_parameter<double>("pad_imu_rate_hz");
        velTau_          = declare_parameter<double>("pad_velocity_tau_s");

        auto sensorQos = rclcpp::SensorDataQoS();
        droneSub_ = create_subscription<geometry_msgs::msg::PoseStamped>(droneTopic, sensorQos,
            [this](geometry_msgs::msg::PoseStamped::SharedPtr m) { onDrone(*m); });
        padSub_ = create_subscription<geometry_msgs::msg::PoseStamped>(padTopic, sensorQos,
            [this](geometry_msgs::msg::PoseStamped::SharedPtr m) { onPad(*m); });

        visOdomPub_ = create_publisher<px4_msgs::msg::VehicleOdometry>(
            "/fmu/in/vehicle_visual_odometry", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
        navPub_ = create_publisher<ulisse_msgs::msg::NavFilterData>(
            "/ulisse/nav_filter/data", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
        imuPub_ = create_publisher<sensor_msgs::msg::Imu>("/adnav_node/imu_raw", sensorQos);

        navTimer_ = create_wall_timer(std::chrono::duration<double>(1.0 / navRate), [this] { publishNav(); });
        imuTimer_ = create_wall_timer(std::chrono::duration<double>(1.0 / imuRate), [this] { publishImu(); });

        RCLCPP_INFO(get_logger(), "mocap_bridge: drone %s -> /fmu/in/vehicle_visual_odometry, pad %s -> "
                    "/ulisse/nav_filter/data (%.0f Hz) + /adnav_node/imu_raw (%.0f Hz), world yaw offset %.3f rad",
                    droneTopic.c_str(), padTopic.c_str(), navRate, imuRate, worldYaw_);
    }

private:
    // Mocap world -> ENU: rotate about Z by world_yaw_offset_rad.
    void toEnu(const geometry_msgs::msg::Pose& p, std::array<double, 3>& pos, Quat& q) const {
        const double c = std::cos(worldYaw_), s = std::sin(worldYaw_);
        pos = {c * p.position.x - s * p.position.y, s * p.position.x + c * p.position.y, p.position.z};
        const Quat qz{std::cos(worldYaw_ / 2), 0, 0, std::sin(worldYaw_ / 2)};
        q = mul(qz, {p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z});
    }

    uint64_t nowUs() { return static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000); }

    // ---------------- Drone -> PX4 external vision ----------------
    void onDrone(const geometry_msgs::msg::PoseStamped& m) {
        std::array<double, 3> p;
        Quat q;
        toEnu(m.pose, p, q);

        // ENU/FLU -> NED/FRD: position (x, y, z)_ENU -> (y, x, -z)_NED;
        // attitude q_NED_FRD = q(ENU->NED) * q_ENU_FLU * q(FLU->FRD).
        static const Quat kEnuToNed{0.0, M_SQRT1_2, M_SQRT1_2, 0.0};   // 180 deg about (1,1,0)/sqrt2
        static const Quat kFluToFrd{0.0, 1.0, 0.0, 0.0};               // 180 deg about X
        const Quat qNed = mul(mul(kEnuToNed, q), kFluToFrd);

        px4_msgs::msg::VehicleOdometry o{};
        o.timestamp        = nowUs();
        o.timestamp_sample = o.timestamp;
        o.pose_frame       = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
        o.position         = {static_cast<float>(p[1]), static_cast<float>(p[0]), static_cast<float>(-p[2])};
        o.q                = {static_cast<float>(qNed.w), static_cast<float>(qNed.x),
                              static_cast<float>(qNed.y), static_cast<float>(qNed.z)};
        o.velocity_frame   = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;
        o.velocity         = {NAN, NAN, NAN};          // position-only: EKF2 differentiates itself
        o.angular_velocity = {NAN, NAN, NAN};
        o.position_variance    = {static_cast<float>(posVar_), static_cast<float>(posVar_), static_cast<float>(posVar_)};
        o.orientation_variance = {static_cast<float>(oriVar_), static_cast<float>(oriVar_), static_cast<float>(oriVar_)};
        o.velocity_variance    = {NAN, NAN, NAN};
        o.reset_counter = 0;
        o.quality       = 100;
        visOdomPub_->publish(o);
        lastDrone_ = now();
    }

    // ---------------- Pad -> catamaran-style sea state ----------------
    void onPad(const geometry_msgs::msg::PoseStamped& m) {
        std::array<double, 3> p;
        Quat q;
        toEnu(m.pose, p, q);
        const auto rpy = toRpy(q);
        const rclcpp::Time t = now();

        if (padSeen_) {
            const double dt = (t - lastPad_).seconds();
            if (dt > 1e-4) {
                const double a = std::exp(-dt / std::max(velTau_, 1e-3));
                const double vz = (p[2] - padPos_[2]) / dt;
                const double az = (vz - padVz_) / dt;
                std::array<double, 3> rate;
                for (int i = 0; i < 3; ++i) rate[i] = wrap(rpy[i] - padRpy_[i]) / dt;
                padVz_ = a * padVz_ + (1 - a) * vz;
                padAz_ = a * padAz_ + (1 - a) * az;
                for (int i = 0; i < 3; ++i) padRate_[i] = a * padRate_[i] + (1 - a) * rate[i];
            }
        }
        padPos_ = p;
        padRpy_ = rpy;
        padQ_   = q;
        lastPad_ = t;
        padSeen_ = true;
    }

    bool padFresh() {
        if (!padSeen_) return false;
        if ((now() - lastPad_).seconds() > timeout_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "pad pose not received for > %.2f s", timeout_);
            return false;
        }
        return true;
    }

    // Same fields heave_estimator (bodyframe_linear_velocity[2]) and
    // spa_predictor (bodyframe_angular_position) read from the real boat.
    void publishNav() {
        if (!padFresh()) return;
        ulisse_msgs::msg::NavFilterData n{};
        n.stamp = now();
        n.bodyframe_angular_position.roll  = padRpy_[0];
        n.bodyframe_angular_position.pitch = padRpy_[1];
        n.bodyframe_angular_position.yaw   = padRpy_[2];
        n.bodyframe_linear_velocity  = {0.0, 0.0, padVz_};      // heave velocity, up-positive
        n.bodyframe_angular_velocity = {padRate_[0], padRate_[1], padRate_[2]};
        n.imu_received = true;
        navPub_->publish(n);
    }

    // Pad IMU in the convention the touchdown detector currently assumes for
    // /adnav_node/imu_raw: linear_acceleration.z = true vertical accel - g
    // (see SeaState.h; padAccelTrue = padAccelZ + g). At rest: -9.81.
    void publishImu() {
        if (!padFresh()) return;
        sensor_msgs::msg::Imu imu;
        imu.header.stamp    = now();
        imu.header.frame_id = "pad";
        imu.orientation.w = padQ_.w; imu.orientation.x = padQ_.x;
        imu.orientation.y = padQ_.y; imu.orientation.z = padQ_.z;
        imu.angular_velocity.x = padRate_[0];
        imu.angular_velocity.y = padRate_[1];
        imu.angular_velocity.z = padRate_[2];
        imu.linear_acceleration.x = 0.0;
        imu.linear_acceleration.y = 0.0;
        imu.linear_acceleration.z = padAz_ - kG;
        imuPub_->publish(imu);
    }

    double worldYaw_, posVar_, oriVar_, timeout_, velTau_;

    bool padSeen_ = false;
    rclcpp::Time lastPad_, lastDrone_;
    std::array<double, 3> padPos_{}, padRpy_{}, padRate_{};
    Quat padQ_{1, 0, 0, 0};
    double padVz_ = 0.0, padAz_ = 0.0;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr droneSub_, padSub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr visOdomPub_;
    rclcpp::Publisher<ulisse_msgs::msg::NavFilterData>::SharedPtr navPub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPub_;
    rclcpp::TimerBase::SharedPtr navTimer_, imuTimer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MocapBridge>());
    rclcpp::shutdown();
    return 0;
}
