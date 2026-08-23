// bridge_node.cpp
//
// PX 4 <-> ROS 2 bridge, pure ROS 2 (LCM fully removed).
//
// The Gazebo plugins (landingPlat / Payload) now talk ROS 2 directly, so the
// old LCM plugin-input/output mirrors are gone. What remains is the PX 4 glue:
//
// What this node does:
//   * Republishes /fmu/out/vehicle_odometry as ROS 2 "vision_position_estimate"
//     (with NED->ENU axis swap). App nodes and the Gazebo plugins subscribe to
//     this; any other source (mocap, custom estimator) can publish to the same
//     topic to override.
//   * Republishes /ulisse/nav_filter/data as ROS 2 "nav_pose"
//     (with global->local lat/lon conversion using a captured home).
//   * Translates ROS 2 "local_position_sp" (nav_msgs/Odometry) to PX 4
//     /fmu/in/trajectory_setpoint, with ENU->NED swap and the slew-rate filter.
//     POSITION vs VELOCITY is read from header.frame_id ("POSITION"|"VELOCITY").
//   * Publishes /fmu/in/offboard_control_mode at 50 ms, with position/velocity
//     flags tracking the last local_position_sp type.
//   * Translates "actual_task" action enum to /drone_state (Float32) for the
//     RPi-side fsm_raspicam package.
//
// Moved out of the bridge:
//   * The landingPlat/Payload plugins now subscribe to ROS 2 directly
//     (vision_position_estimate, nav_pose, Landing_Site/pose_cmd,
//      release_payload_feedback, /fmu/out/vehicle_gps_position) and publish
//      Landing_Site/pose themselves — no LCM mirror needed.
//   * battery / camera_info / state / nav_task_status: apps read PX 4 / sensor
//     topics directly.

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

// PX 4
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>

// Standard / ulisse
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <ulisse_msgs/msg/nav_filter_data.hpp>
#include <ulisse_msgs/msg/actual_task.hpp>

// Project utilities
#include "common/conversions.h"   // globalToLocalCoords, yawFromQuaternion
#include "common/common.h"        // common::actions::*

using namespace std::chrono_literals;
using std::placeholders::_1;

class BridgeNode : public rclcpp::Node
{
public:
  BridgeNode()
  : rclcpp::Node("ros2_px4_bridge")
  {
    auto px4_qos      = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

    // ==================== ROS 2 publishers ====================

    // Republisher outputs.
    vision_position_estimate_pub_ = create_publisher<px4_msgs::msg::VehicleOdometry>(
      "vision_position_estimate", px4_qos);
    nav_pose_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "nav_pose", reliable_qos);

    // PX 4 command outputs.
    trajectory_setpoint_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      "/fmu/in/trajectory_setpoint", 1);
    gt_setpoint_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      "/bridge/gt_trajectory_setpoint", 1);
    offboard_control_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
      "/fmu/in/offboard_control_mode", 1);

    // RPi-facing translator output.
    drone_state_pub_ = create_publisher<std_msgs::msg::Float32>("/drone_state", 1);

    // ==================== ROS 2 subscriptions ====================

    // -- Republisher sources --
    odom_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
      "/fmu/out/vehicle_odometry", px4_qos,
      std::bind(&BridgeNode::onPx4Odom, this, _1));
    nav_filter_sub_ = create_subscription<ulisse_msgs::msg::NavFilterData>(
      "/ulisse/nav_filter/data", reliable_qos,
      std::bind(&BridgeNode::onNavFilter, this, _1));

    // -- Cached state used by other paths --
    land_detected_sub_ = create_subscription<px4_msgs::msg::VehicleLandDetected>(
      "/fmu/out/vehicle_land_detected", px4_qos,
      std::bind(&BridgeNode::onLandDetected, this, _1));

    // -- Command translator --
    local_position_sp_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "local_position_sp", reliable_qos,
      std::bind(&BridgeNode::onLocalPositionSp, this, _1));

    // -- actual_task -> /drone_state translator --
    actual_task_sub_ = create_subscription<ulisse_msgs::msg::ActualTask>(
      "actual_task", reliable_qos,
      std::bind(&BridgeNode::onActualTask, this, _1));

    // ==================== Timers ====================

    // Offboard heartbeat (50 ms, matches old bridge).
    offboard_timer_ = create_wall_timer(
      50ms, std::bind(&BridgeNode::publishOffboardMode, this));

    RCLCPP_INFO(get_logger(), "Bridge node ready");
  }

private:
  // ===================== Republisher: PX 4 odom -> ENU =====================

  void onPx4Odom(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
  {
    px4_msgs::msg::VehicleOdometry out = *msg;
    // NED -> ENU axis swap.
    out.position[0] =  msg->position[1];
    out.position[1] =  msg->position[0];
    out.position[2] = -msg->position[2];
    out.velocity[0] =  msg->velocity[1];
    out.velocity[1] =  msg->velocity[0];
    out.velocity[2] = -msg->velocity[2];
    // Orientation: the original bridge passed q[] through unchanged when writing
    // to LCM geometry::pose. Preserve that behavior — if you want a proper
    // frame-rotation on the quaternion, do it here.
    vision_position_estimate_pub_->publish(out);

    last_drone_pose_enu_ = out;  // cache for gt_setpoint
  }

  // ===================== Republisher: nav_filter -> nav_pose ===================

  void onNavFilter(const ulisse_msgs::msg::NavFilterData::SharedPtr msg)
  {
    if (first_nav_state_) {
      nav_home_lat_ = msg->inertialframe_linear_position.latitude;
      nav_home_lon_ = msg->inertialframe_linear_position.longitude;
      RCLCPP_INFO(get_logger(), "NAV home lat=%.7f lon=%.7f", nav_home_lat_, nav_home_lon_);
      first_nav_state_ = false;
    }

    double coords[3] = {
      msg->inertialframe_linear_position.longitude,
      msg->inertialframe_linear_position.latitude,
      0.0
    };
    double home[3] = {nav_home_lon_, nav_home_lat_, 0.0};
    globalToLocalCoords(coords, home);

    nav_msgs::msg::Odometry out;
    out.header.stamp = this->now();
    out.header.frame_id = "local";
    out.pose.pose.position.x = coords[0];
    out.pose.pose.position.y = coords[1];
    out.pose.pose.position.z = 0.0;

    out.twist.twist.linear.x = msg->bodyframe_linear_velocity[0];
    out.twist.twist.linear.y = msg->bodyframe_linear_velocity[1];
    out.twist.twist.linear.z = msg->bodyframe_linear_velocity[2];

    Eigen::Quaternionf q =
      Eigen::AngleAxisf(msg->bodyframe_angular_position.roll,  Eigen::Vector3f::UnitX()) *
      Eigen::AngleAxisf(msg->bodyframe_angular_position.pitch, Eigen::Vector3f::UnitY()) *
      Eigen::AngleAxisf(msg->bodyframe_angular_position.yaw,   Eigen::Vector3f::UnitZ());
    out.pose.pose.orientation.w = q.w();
    out.pose.pose.orientation.x = q.x();
    out.pose.pose.orientation.y = q.y();
    out.pose.pose.orientation.z = q.z();

    nav_pose_pub_->publish(out);
  }

  // ===================== Cached state callbacks =====================

  void onLandDetected(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg)
  {
    landed_ = msg->landed;
  }

  // ===================== Command translator =====================
  // ROS 2 nav_msgs/Odometry on "local_position_sp" -> PX 4 TrajectorySetpoint.
  // header.frame_id selects POSITION vs VELOCITY. ENU -> NED on the way out.

  void onLocalPositionSp(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    px4_msgs::msg::TrajectorySetpoint setpoint;
    setpoint.timestamp = this->now().nanoseconds() / 1000;

    if (msg->header.frame_id == "POSITION") {
      const rclcpp::Time current_time = get_clock()->now();
      // ENU -> NED target.
      double target[3];
      target[0] =  msg->pose.pose.position.y;
      target[1] =  msg->pose.pose.position.x;
      target[2] = -msg->pose.pose.position.z;

      if (first_pos_sp_) {
        pos_sp_last_time_ = current_time;
        prev_pos_sp_[0] = target[0];
        prev_pos_sp_[1] = target[1];
        prev_pos_sp_[2] = target[2];
        first_pos_sp_ = false;
      }

      const double dt = (current_time - pos_sp_last_time_).seconds();
      pos_sp_last_time_ = current_time;

      if (dt <= 0.0 || dt > 0.5) {
        // Big dt jump (or first frame): snap.
        for (int i = 0; i < 3; ++i) {
          setpoint.position[i] = target[i];
          prev_pos_sp_[i]      = target[i];
        }
      } else {
        const double max_step = kMaxVelocity * dt;
        for (int i = 0; i < 3; ++i) {
          double error = target[i] - prev_pos_sp_[i];
          if      (error >  max_step) error =  max_step;
          else if (error < -max_step) error = -max_step;
          setpoint.position[i] = prev_pos_sp_[i] + error;
          prev_pos_sp_[i]      = setpoint.position[i];
        }
      }

      Eigen::Quaterniond q(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z);
      setpoint.yaw = -yawFromQuaternion(q);
      // XY feed-forward stays NaN (position-only tracking there); Z carries
      // the heave feed-forward velocity (0 outside COMP — see Lander::run()).
      setpoint.velocity[0] = NAN;
      setpoint.velocity[1] = NAN;
      setpoint.velocity[2] = -msg->twist.twist.linear.z;  // ENU -> NED

      position_enabled_ = true;
      velocity_enabled_ = false;
      trajectory_setpoint_pub_->publish(setpoint);

      // gt_setpoint: drone-position-plus-offset, preserved verbatim from the
      // original. NOTE: position[2] uses position[1] in the source, which looks
      // like a typo (should probably be -position[2]); kept as-is.
      px4_msgs::msg::TrajectorySetpoint gt;
      gt.position[0] =  last_drone_pose_enu_.position[1] + 1.0;
      gt.position[1] =  last_drone_pose_enu_.position[0];
      gt.position[2] = -last_drone_pose_enu_.position[1];  // FIXME (probably should be -position[2])
      gt_setpoint_pub_->publish(gt);
    }
    else if (msg->header.frame_id == "VELOCITY") {
      // ENU -> NED on velocity.
      setpoint.velocity[0] =  msg->twist.twist.linear.y;
      setpoint.velocity[1] =  msg->twist.twist.linear.x;
      setpoint.velocity[2] = -msg->twist.twist.linear.z;
      setpoint.position = {NAN, NAN, NAN};

      // Unlike the POSITION branch above, this was previously leaving
      // setpoint.yaw at its default-constructed value — causing a sudden yaw
      // snap whenever a POSITION setpoint (which does set yaw) was followed
      // by a VELOCITY one (e.g. HOLD/COMP -> LAND). Carry the commanded yaw
      // through here too.
      Eigen::Quaterniond qv(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z);
      setpoint.yaw = -yawFromQuaternion(qv);

      position_enabled_ = false;
      velocity_enabled_ = true;
      trajectory_setpoint_pub_->publish(setpoint);
    }
    else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "local_position_sp: unknown frame_id '%s' (expected POSITION or VELOCITY)",
        msg->header.frame_id.c_str());
    }
  }

  // ===================== actual_task -> /drone_state =====================

  void onActualTask(const ulisse_msgs::msg::ActualTask::SharedPtr msg)
  {
    std_msgs::msg::Float32 ds;
    if      (msg->action == common::actions::LAND)        ds.data = 0;
    else if (msg->action == common::actions::SEARCH)      ds.data = 1;
    else if (msg->action == common::actions::EMERG_LAND || landed_) ds.data = 2;
    else                                                  ds.data = 3;
    drone_state_pub_->publish(ds);
  }

  // ===================== Offboard heartbeat =====================

  void publishOffboardMode()
  {
    px4_msgs::msg::OffboardControlMode msg;
    msg.timestamp    = this->now().nanoseconds() / 1000;
    msg.position     = position_enabled_;
    msg.velocity     = velocity_enabled_;
    msg.acceleration = false;
    msg.attitude     = false;
    msg.body_rate    = false;
    offboard_control_mode_pub_->publish(msg);
  }

  // ===================== State =====================

  // Latest ENU drone pose (republisher output) used for gt_setpoint.
  px4_msgs::msg::VehicleOdometry last_drone_pose_enu_ {};
  // Latest flags used by other paths.
  bool landed_ {true};

  // Nav filter home (captured on first message).
  bool   first_nav_state_ {true};
  double nav_home_lat_    {0.0};
  double nav_home_lon_    {0.0};

  // Slew-rate state for the position command translator.
  static constexpr double kMaxVelocity = 0.3;  // m/s
  bool         first_pos_sp_ {true};
  rclcpp::Time pos_sp_last_time_;
  double       prev_pos_sp_[3] {0.0, 0.0, 0.0};

  // Offboard mode flags reflect the type of the last accepted local_position_sp.
  bool position_enabled_ {true};
  bool velocity_enabled_ {false};

  // ---- Publishers ----
  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr      vision_position_estimate_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr             nav_pose_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr   trajectory_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr   gt_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr  offboard_control_mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr              drone_state_pub_;

  // ---- Subscriptions ----
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr      odom_sub_;
  rclcpp::Subscription<ulisse_msgs::msg::NavFilterData>::SharedPtr     nav_filter_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr  land_detected_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr             local_position_sp_sub_;
  rclcpp::Subscription<ulisse_msgs::msg::ActualTask>::SharedPtr        actual_task_sub_;

  // ---- Timers ----
  rclcpp::TimerBase::SharedPtr offboard_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<BridgeNode>());
  } catch (const std::exception & e) {
    std::cerr << "Fatal: " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
