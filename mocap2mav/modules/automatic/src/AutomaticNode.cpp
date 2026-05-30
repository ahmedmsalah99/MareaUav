// automatic_node.cpp
//
// ROS 2 port of the original LCM-based Automatic main loop.
//
// Migration notes:
//   * No LCM is used here.
//   * Drone pose ("vision_position_estimate") is a ROS 2 topic of the same name,
//     type px4_msgs/VehicleOdometry. Already ENU thanks to the bridge.
//   * Platform pose ("Landing_Site/pose") is a Gazebo plugin output. The bridge
//     mirrors it to a ROS 2 topic of the same name with type nav_msgs/Odometry.
//   * Platform pose command ("Landing_Site/pose_cmd") is a Gazebo plugin input.
//     We publish it as ROS 2 (nav_msgs/Odometry) and the bridge mirrors to LCM.
//   * Ultrasonic ("UltrasonicSensor/platform") is no longer a Gazebo plugin
//     topic — it's a regular ROS 2 sensor topic published by the migrated
//     mocap2mav node (type ultrasonic_sensor/msg/UltrasonicMsg). The LCM tail
//     of that node is being removed.
//   * AprilTag platform detections come from the AprilTag2Node's ROS 2 publisher
//     /platform_center (geometry_msgs/TransformStamped). The LCM tail
//     (apriltag_vision_system) of that node is being removed.
//     NOTE: /platform_center carries the raw transform only. The filtered
//     roll/pitch/yaw that used to live on the LCM geometry::vision message are
//     no longer produced. If Automatic actually consumes them, AprilTag2Node
//     needs a richer ROS 2 publication. See TODO in onPlatformCenter().
//   * AprilTag rescue detections come from /rescue
//     (geometry_msgs/TransformStamped) — same story as /platform_center.
//   * GPS is read directly from /fmu/out/vehicle_gps_position (px4_msgs/SensorGps).
//     The bridge's gps_status scaling is reproduced here (lon/lat × 1e-7,
//     alt × 1e-3 — verify the altitude scaling against your px4_msgs version).
//   * Release payload feedback is now native ROS 2 (std_msgs/String).
//   * Marker/pose_cmd publication is commented out (per the design notes).
//   * Command publishing:
//       - "local_position_sp" -> ROS 2 nav_msgs/Odometry. The bridge subscribes,
//         applies the slew-rate filter, and publishes /fmu/in/trajectory_setpoint.
//         The MavState::type::{POSITION,VELOCITY} flag is encoded in
//         header.frame_id ("POSITION" or "VELOCITY"). Update the bridge to
//         switch on frame_id instead of getType().
//       - "platRob" -> ROS 2 nav_msgs/Odometry. No known consumer; preserved.
//
// TODOs in the supporting headers (Automatic.h / Automatic.cpp):
//   * Change `Automatic::_actualTask` from `exec::actual_task` to
//     `ulisse_exec_msgs::msg::ActualTask`. Field names preserved.
//   * Adapt `Automatic::setTask()`, `setVisionFeedback()`, `setVisionRescue()`,
//     `setUltrasonicInfo()`, `setPlatformState()`, `setRelPayFeedback()` to
//     take the ROS 2 equivalents (or pass `const MavState&` already built here).
//   * `printAction()` should accept the ROS 2 ActualTask::action field type.

#include <chrono>
#include <iostream>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

// PX 4 / sensor ROS 2 messages
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>

// Other ROS 2 messages
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <ultrasonic_sensor/msg/ultrasonic_msg.hpp>

// Custom ROS 2 messages (mirrors of the old LCM exec/* structs)
#include <ulisse_exec_msgs/msg/actual_task.hpp>
#include <ulisse_exec_msgs/msg/safety_ack.hpp>

// Unchanged application code
#include "Automatic.h"
#include "Command/TakeOff.hpp"
#include "common/MavState.h"
#include "common/conversions.h"

using namespace common;
using namespace std::chrono_literals;

// ---------- helpers ----------

// Build a MavState from a px4_msgs/VehicleOdometry that has already been
// NED→ENU-swapped by the bridge.
static void mavStateFromOdom(const px4_msgs::msg::VehicleOdometry & in, MavState & out)
{
  out.setX(in.position[0]);
  out.setY(in.position[1]);
  out.setZ(in.position[2]);
  out.setVx(in.velocity[0]);
  out.setVy(in.velocity[1]);
  out.setVz(in.velocity[2]);
  Eigen::Quaterniond q(in.q[0], in.q[1], in.q[2], in.q[3]);
  out.setOrientation(q);
}

// Build a MavState from a nav_msgs/Odometry.
static void mavStateFromNavOdom(const nav_msgs::msg::Odometry & in, MavState & out)
{
  out.setX(in.pose.pose.position.x);
  out.setY(in.pose.pose.position.y);
  out.setZ(in.pose.pose.position.z);
  out.setVx(in.twist.twist.linear.x);
  out.setVy(in.twist.twist.linear.y);
  out.setVz(in.twist.twist.linear.z);
  Eigen::Quaterniond q(
    in.pose.pose.orientation.w,
    in.pose.pose.orientation.x,
    in.pose.pose.orientation.y,
    in.pose.pose.orientation.z);
  out.setOrientation(q);
}

// Convert a MavState back to a nav_msgs/Odometry, encoding the type
// (POSITION/VELOCITY) in header.frame_id for the bridge to switch on.
static nav_msgs::msg::Odometry odomFromMavState(const MavState & s)
{
  nav_msgs::msg::Odometry msg;
  msg.header.frame_id = (s.getType() == MavState::type::POSITION) ? "POSITION" : "VELOCITY";
  msg.pose.pose.position.x = s.getX();
  msg.pose.pose.position.y = s.getY();
  msg.pose.pose.position.z = s.getZ();
  Eigen::Quaterniond q = s.getOrientation();
  msg.pose.pose.orientation.w = q.w();
  msg.pose.pose.orientation.x = q.x();
  msg.pose.pose.orientation.y = q.y();
  msg.pose.pose.orientation.z = q.z();
  msg.twist.twist.linear.x = s.getVx();
  msg.twist.twist.linear.y = s.getVy();
  msg.twist.twist.linear.z = s.getVz();
  return msg;
}


class AutomaticNode : public rclcpp::Node
{
public:
  AutomaticNode()
  : rclcpp::Node("automatic")
  {
    auto px4_qos      = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

    // -------- Publishers --------
    local_position_sp_pub_ =
      create_publisher<nav_msgs::msg::Odometry>("local_position_sp", reliable_qos);
    landing_site_pose_cmd_pub_ =
      create_publisher<nav_msgs::msg::Odometry>("Landing_Site/pose_cmd", reliable_qos);
    plat_rob_pub_ =
      create_publisher<nav_msgs::msg::Odometry>("platRob", reliable_qos);
    // Marker visualization publisher: removed (was Gazebo Marker plugin, not used).
    // marker_pose_cmd_pub_ =
    //   create_publisher<nav_msgs::msg::Odometry>("Marker/pose_cmd", reliable_qos);

    // -------- Subscriptions --------
    // Drone pose. This callback is also the master tick (matches the old
    // main loop, which blocked on handler.handle() for this topic).
    vision_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
      "vision_position_estimate", px4_qos,
      std::bind(&AutomaticNode::onVisionPos, this, std::placeholders::_1));

    // Platform pose (Gazebo plugin output via bridge mirror).
    platform_pose_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "Landing_Site/pose", reliable_qos,
      std::bind(&AutomaticNode::onPlatformPose, this, std::placeholders::_1));

    // Task from Executioner.
    actual_task_sub_ = create_subscription<ulisse_exec_msgs::msg::ActualTask>(
      "actual_task", reliable_qos,
      std::bind(&AutomaticNode::onActualTask, this, std::placeholders::_1));

    // AprilTag platform detection (from AprilTag2Node).
    platform_center_sub_ = create_subscription<geometry_msgs::msg::TransformStamped>(
      "/platform_center", reliable_qos,
      std::bind(&AutomaticNode::onPlatformCenter, this, std::placeholders::_1));

    // AprilTag rescue detection (from AprilTag2RescueNode).
    rescue_sub_ = create_subscription<geometry_msgs::msg::TransformStamped>(
      "/rescue", reliable_qos,
      std::bind(&AutomaticNode::onRescue, this, std::placeholders::_1));

    // Ultrasonic sensor (from migrated mocap2mav node).
    ultrasonic_sub_ = create_subscription<ultrasonic_sensor::msg::UltrasonicMsg>(
      "UltrasonicSensor/platform", reliable_qos,
      std::bind(&AutomaticNode::onUltrasonic, this, std::placeholders::_1));

    // Safety ack (drained but not actively read in old main; preserved).
    safety_ack_sub_ = create_subscription<ulisse_exec_msgs::msg::SafetyAck>(
      "safety_ack", reliable_qos,
      std::bind(&AutomaticNode::onSafetyAck, this, std::placeholders::_1));

    // GPS direct from PX 4 (replaces bridge-translated "gps_status").
    gps_sub_ = create_subscription<px4_msgs::msg::SensorGps>(
      "/fmu/out/vehicle_gps_position", px4_qos,
      std::bind(&AutomaticNode::onGps, this, std::placeholders::_1));

    // Release payload feedback (already ROS 2 native).
    rel_pay_fb_sub_ = create_subscription<std_msgs::msg::String>(
      "release_payload_feedback", reliable_qos,
      std::bind(&AutomaticNode::onReleasePayloadFeedback, this, std::placeholders::_1));

    // Initial freshness timestamps (very old so isExpired() is true until first message).
    rclcpp::Time epoch(0, 0, get_clock()->get_clock_type());
    last_vision_time_     = epoch;
    last_rescue_time_     = epoch;
    last_ultrasonic_time_ = epoch;

    RCLCPP_INFO(get_logger(), "Automatic node ready");
  }

private:
  // ===================== Callbacks =====================

  void onVisionPos(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
  {
    // Update drone pose.
    mavStateFromOdom(*msg, drone_pose_);
    autom_.setState(drone_pose_);

    // Sensor-freshness checks (matches the original "else { if isExpired() }" branches).
    checkSensorFreshness();

    if (waiting_) {
      return;
    }

    // MOVE-to-platform: overwrite the task target with the live platform pose (sim only).
    if (current_action_ == actions::MOVE
        && current_target_ == "platform"
        && autom_.env_sim)
    {
      autom_._actualTask.x = platform_state_.getX();
      autom_._actualTask.y = platform_state_.getY();
      // autom_._actualTask.z left untouched, matching the original.
    }

    // Compute and publish the next command.
    autom_.executeCommand();

    auto command = odomFromMavState(autom_._comm);
    local_position_sp_pub_->publish(command);
    // marker_pose_cmd_pub_->publish(command);  // commented out

    // platRob: echo the platform pose (no known consumer in current code).
    auto plat_rob = odomFromMavState(platform_state_);
    plat_rob_pub_->publish(plat_rob);
  }

  void onPlatformPose(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    mavStateFromNavOdom(*msg, platform_state_);
    autom_.setPlatformState(platform_state_);
  }

  void onActualTask(const ulisse_exec_msgs::msg::ActualTask::SharedPtr msg)
  {
    // TODO: update Automatic::setTask signature to take the ROS 2 ActualTask.
    autom_.setTask(*msg);
    current_action_ = msg->action;
    current_target_ = msg->target;

    std::cout << "New task arrived with action: " << printAction(msg->action) << std::endl;
    waiting_ = false;
    autom_.handleCommands();
  }

  void onPlatformCenter(const geometry_msgs::msg::TransformStamped::SharedPtr msg)
  {
    // Build a MavState from the raw transform.
    // TODO: filtered roll/pitch/yaw used to live on the old LCM geometry::vision
    // message (filtered inside AprilTag2Node via movingFilter+toEulerAngle).
    // /platform_center carries only the raw transform, so the filtered euler is
    // gone here. Either:
    //   (a) move the filtering into this node (re-implement movingFilter),
    //   (b) extend AprilTag2Node to publish a richer ROS 2 message, or
    //   (c) confirm Automatic only needs the orientation quaternion (in which
    //       case nothing else is needed and this TODO can be deleted).
    vision_system_.setX(msg->transform.translation.x);
    vision_system_.setY(msg->transform.translation.y);
    vision_system_.setZ(msg->transform.translation.z);
    vision_system_.setVx(0.0);
    vision_system_.setVy(0.0);
    vision_system_.setVz(0.0);
    Eigen::Quaterniond q(
      msg->transform.rotation.w,
      msg->transform.rotation.x,
      msg->transform.rotation.y,
      msg->transform.rotation.z);
    vision_system_.setOrientation(q);

    vision_system_.VisionDataUpdated = true;
    autom_.setVisionFeedback(vision_system_);
    last_vision_time_ = this->now();
  }

  void onRescue(const geometry_msgs::msg::TransformStamped::SharedPtr msg)
  {
    // Same filtering caveat as onPlatformCenter applies here.
    vision_rescue_.setX(msg->transform.translation.x);
    vision_rescue_.setY(msg->transform.translation.y);
    vision_rescue_.setZ(msg->transform.translation.z);
    vision_rescue_.setVx(0.0);
    vision_rescue_.setVy(0.0);
    vision_rescue_.setVz(0.0);
    Eigen::Quaterniond q(
      msg->transform.rotation.w,
      msg->transform.rotation.x,
      msg->transform.rotation.y,
      msg->transform.rotation.z);
    vision_rescue_.setOrientation(q);

    vision_rescue_.VisionDataUpdated = true;
    autom_.setVisionRescue(vision_rescue_);
    last_rescue_time_ = this->now();

    // In SEARCH state, notify the platform of the rescue target's coordinates.
    if (current_action_ == actions::SEARCH) {
      publishRescueCoords();
    }
  }

  void onUltrasonic(const ultrasonic_sensor::msg::UltrasonicMsg::SharedPtr msg)
  {
    // TODO: confirm the field names on UltrasonicMsg match those used here.
    // The old LCM geometry::UltrasonicPosition had Z_Position, Z_Velocity, isValid.
    ultrasonic_pos_.setZ(msg->z_position);
    ultrasonic_pos_.setVz(msg->z_velocity);
    ultrasonic_pos_.IsValid = msg->is_valid;

    ultrasonic_pos_.UltrasonicDataUpdated = true;
    autom_.setUltrasonicInfo(ultrasonic_pos_);
    last_ultrasonic_time_ = this->now();
  }

  void onSafetyAck(const ulisse_exec_msgs::msg::SafetyAck::SharedPtr msg)
  {
    // Original main drained this topic but never read it. Preserve the subscription
    // (in case downstream code grows to need it) and cache for that future.
    last_safety_ack_ = *msg;
  }

  void onGps(const px4_msgs::msg::SensorGps::SharedPtr msg)
  {
    // Same scaling the bridge used; verify altitude scaling against your px4_msgs version.
    gps_lon_ = 1.0e-7 * msg->longitude_deg;
    gps_lat_ = 1.0e-7 * msg->latitude_deg;
    gps_alt_ = 1.0e-3 * msg->altitude_msl_m;
  }

  void onReleasePayloadFeedback(const std_msgs::msg::String::SharedPtr msg)
  {
    // TODO: update Automatic::setRelPayFeedback signature to const std::string&.
    autom_.setRelPayFeedback(msg->data);
  }

  // ===================== Helpers =====================

  void checkSensorFreshness()
  {
    const rclcpp::Time now = this->now();
    constexpr double kStaleSec = 1.0;

    if ((now - last_vision_time_).seconds() > kStaleSec) {
      vision_system_.VisionDataUpdated = false;
      autom_.setVisionFeedback(vision_system_);
    }

    if ((now - last_rescue_time_).seconds() > kStaleSec) {
      vision_rescue_.VisionDataUpdated = false;
      autom_.setVisionRescue(vision_rescue_);
    }

    if ((now - last_ultrasonic_time_).seconds() > kStaleSec) {
      ultrasonic_pos_.UltrasonicDataUpdated = false;
      autom_.setUltrasonicInfo(ultrasonic_pos_);
    }
  }

  // Called from onRescue() when current_action_ == SEARCH.
  void publishRescueCoords()
  {
    nav_msgs::msg::Odometry rescue;
    rescue.header.stamp    = this->now();
    rescue.header.frame_id = "rescue_target";

    if (autom_.env_sim) {
      // Local-frame coordinates of the drone (NED-local ENU).
      rescue.pose.pose.position.x = autom_.getState().getX();
      rescue.pose.pose.position.y = autom_.getState().getY();
      rescue.pose.pose.position.z = autom_.getState().getZ();

      // Yaw: drone-to-platform plus a 90° offset because the platform's local
      // frame is rotated 90° wrt NED. Preserved from the original verbatim.
      double platform_pos[3] = {
        platform_state_.getX(), platform_state_.getY(), platform_state_.getZ()};
      double yaw_temp = computeDesiredYaw(autom_.getState(), platform_pos) + 1.57;

      Eigen::Quaterniond orient = getQuatFromYaw(yaw_temp);
      rescue.pose.pose.orientation.w = orient.w();
      rescue.pose.pose.orientation.x = orient.x();
      rescue.pose.pose.orientation.y = orient.y();
      rescue.pose.pose.orientation.z = orient.z();
    } else {
      // Real test: global GPS coordinates. The ship computes its own orientation.
      rescue.pose.pose.position.x = gps_lon_;
      rescue.pose.pose.position.y = gps_lat_;
      rescue.pose.pose.position.z = gps_alt_;
    }

    landing_site_pose_cmd_pub_->publish(rescue);
  }

  // ===================== State =====================

  Automatic                              autom_;

  MavState                               drone_pose_;
  MavState                               platform_state_;
  MavState                               vision_system_;
  MavState                               vision_rescue_;
  MavState                               ultrasonic_pos_;

  bool                                   waiting_         {true};
  int                                    current_action_  {actions::NONE};
  std::string                            current_target_  {""};

  // Cached GPS (already scaled to deg / m).
  double                                 gps_lon_ {0.0};
  double                                 gps_lat_ {0.0};
  double                                 gps_alt_ {0.0};

  ulisse_exec_msgs::msg::SafetyAck       last_safety_ack_ {};

  rclcpp::Time                           last_vision_time_;
  rclcpp::Time                           last_rescue_time_;
  rclcpp::Time                           last_ultrasonic_time_;

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr   local_position_sp_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr   landing_site_pose_cmd_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr   plat_rob_pub_;
  // rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr   marker_pose_cmd_pub_;  // dropped

  // Subscriptions
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr        vision_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr               platform_pose_sub_;
  rclcpp::Subscription<ulisse_exec_msgs::msg::ActualTask>::SharedPtr     actual_task_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr  platform_center_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr  rescue_sub_;
  rclcpp::Subscription<ultrasonic_sensor::msg::UltrasonicMsg>::SharedPtr ultrasonic_sub_;
  rclcpp::Subscription<ulisse_exec_msgs::msg::SafetyAck>::SharedPtr      safety_ack_sub_;
  rclcpp::Subscription<px4_msgs::msg::SensorGps>::SharedPtr              gps_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr                 rel_pay_fb_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<AutomaticNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    std::cerr << "Fatal: " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}