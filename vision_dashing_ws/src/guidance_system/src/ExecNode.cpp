// executioner_node.cpp
//
// ROS 2 port of the original LCM-based Executioner main loop.
//
// Migration notes:
//   * No LCM is used here.
//   * Drone pose ("vision_position_estimate") is a ROS 2 topic of the same name,
//     type px4_msgs/VehicleOdometry. The bridge republishes /fmu/out/vehicle_odometry
//     onto this topic with the NED→ENU axis swap applied, so apps consume ENU.
//   * The combined LCM "state" (armed+landed) is split into two direct PX 4
//     subscriptions: /fmu/out/vehicle_control_mode for armed, and
//     /fmu/out/vehicle_land_detected for landed. The bridge's old
//     "differs from first observed" semantics are dropped — we pass the raw flags.
//     This is equivalent for the normal startup case (drone starts landed,
//     disarmed). Verify against whatever Executioner::setStatus() expects.
//   * "Landing_Site/pose" is a Gazebo plugin output. The bridge mirrors it from
//     LCM to a ROS 2 topic of the same name with type nav_msgs/Odometry. We
//     subscribe to the ROS 2 side and build a MavState in the callback.
//   * Internal app topics that were LCM become custom ROS 2 messages:
//       - "actual_task" -> ulisse_msgs/msg/ActualTask   (published)
//       - "safety"      -> ulisse_msgs/msg/Safety       (subscribed)
//       - "safety_ack"  -> ulisse_msgs/msg/SafetyAck    (published)
//   * "nav_task_status" used to be wrapped by the bridge from
//     /ulisse/ctrl/vehicle_status into a small LCM struct. We subscribe directly
//     to /ulisse/ctrl/vehicle_status now. The bridge stamping step is gone.
//   * Original loop was a `Spinner(5).useMilli()` busy loop that called
//     setState/run/publish every iteration. Replaced by a 5 ms wall timer.
//
// TODOs in the supporting headers (Executioner.h / Executioner.cpp):
//   * Change `Executioner::_actualTask` from `exec::actual_task` to
//     `ulisse_msgs::msg::ActualTask`. Field names are preserved so the
//     code inside Executioner.cpp that assigns to it does not change.
//   * Adapt `Executioner::setRelPayFeedback()` to take `const std::string&`
//     instead of the LCM exec::release_payload struct.
//   * Adapt `Executioner::setNavTaskStatus()` to take
//     `const ulisse_msgs::msg::VehicleStatus&` (or copy the two fields
//     it currently reads).
//   * Adapt `Executioner::batteryGoneHandler / cameraGoneHandler /
//     moveToSafeHarbor / batteryLimitHandler` to take
//     `const ulisse_msgs::msg::Safety&` instead of the LCM exec::safety.
//     Field names are preserved.

#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

// PX 4 / sensor ROS 2 messages
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>

// Other ROS 2 messages
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <ulisse_msgs/msg/vehicle_status.hpp>

// Custom ROS 2 messages mirroring the old LCM exec/* structs.
#include <ulisse_msgs/msg/actual_task.hpp>
#include <ulisse_msgs/msg/safety.hpp>
#include <ulisse_msgs/msg/safety_ack.hpp>

// Unchanged application code
#include "guidance_system/exec/Executioner.h"
#include "guidance_system/exec/Parser.h"
#include "common/MavState.h"
#include "utils/TimeHelpers.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

using namespace std::chrono_literals;

class ExecutionerNode : public rclcpp::Node
{
public:
  ExecutionerNode()
  : rclcpp::Node("executioner")
  {
    std::string pkg_path = ament_index_cpp::get_package_share_directory("guidance_system");
    std::string list_path = pkg_path + "/lists/list.txt";

    p_.loadFile(list_path);
    RCLCPP_INFO(get_logger(), "Parsing.....");
    if (!p_.parse()) {
      RCLCPP_FATAL(get_logger(), "Unable to parse the task list");
      throw std::runtime_error("task list parse failed");
    }
    RCLCPP_INFO(get_logger(), ".......Parsed");
    e_.setList(p_.getTaskListParsed());
    e_.init();

    // -------- Initialize safety_ack --------
    safety_ack_msg_.isbatterysafetytaskrunning = 0;
    safety_ack_msg_.isgpssafetytaskrunning     = 0;
    safety_ack_msg_.iscamerasafetytaskrunning  = 0;
    safety_ack_msg_.issonarsafetytaskrunning   = 0;
    safety_ack_msg_.isgoinginsafeharbor        = 0;
    safety_ack_msg_.iscameragone_timestamp     = 0.0;
    safety_ack_msg_.isgoingtoplatform          = 0;

    // -------- QoS --------
    auto px4_qos      = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

    // -------- Publishers --------
    actual_task_pub_ = create_publisher<ulisse_msgs::msg::ActualTask>("actual_task", reliable_qos);
    safety_ack_pub_  = create_publisher<ulisse_msgs::msg::SafetyAck>("safety_ack", reliable_qos);

    // -------- Subscriptions --------
    // Drone pose (already ENU thanks to the bridge republisher).
    vision_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
      "vision_position_estimate", px4_qos,
      std::bind(&ExecutionerNode::onVisionPos, this, std::placeholders::_1));

    // Armed flag (was part of the combined LCM "state").
    control_mode_sub_ = create_subscription<px4_msgs::msg::VehicleControlMode>(
      "/fmu/out/vehicle_control_mode", px4_qos,
      std::bind(&ExecutionerNode::onControlMode, this, std::placeholders::_1));

    // Landed flag (was part of the combined LCM "state").
    land_detected_sub_ = create_subscription<px4_msgs::msg::VehicleLandDetected>(
      "/fmu/out/vehicle_land_detected", px4_qos,
      std::bind(&ExecutionerNode::onLandDetected, this, std::placeholders::_1));

    // Platform pose (Gazebo plugin output, bridge-mirrored from LCM).
    platform_pose_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "Landing_Site/pose", reliable_qos,
      std::bind(&ExecutionerNode::onLandingSitePose, this, std::placeholders::_1));

    // Safety msg from SafetyHandler.
    safety_sub_ = create_subscription<ulisse_msgs::msg::Safety>(
      "safety", reliable_qos,
      std::bind(&ExecutionerNode::onSafety, this, std::placeholders::_1));

    // Release payload feedback (now native ROS 2).
    rel_pay_fb_sub_ = create_subscription<std_msgs::msg::String>(
      "release_payload_feedback", reliable_qos,
      std::bind(&ExecutionerNode::onReleasePayloadFeedback, this, std::placeholders::_1));

    // Nav task status: direct from ulisse, no bridge stamping.
    nav_task_status_sub_ = create_subscription<ulisse_msgs::msg::VehicleStatus>(
      "/ulisse/ctrl/vehicle_status", reliable_qos,
      std::bind(&ExecutionerNode::onNavTaskStatus, this, std::placeholders::_1));

    // -------- Periodic tick (5 ms, matching the original Spinner(5).useMilli()) --------
    // Old main always called setState/run and conditionally published every iteration.
    tick_timer_ = create_wall_timer(5ms, std::bind(&ExecutionerNode::tick, this));

    RCLCPP_INFO(get_logger(), "Executioner node ready");
  }

private:
  // ===================== Callbacks =====================

  void onVisionPos(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
  {
    // Already ENU thanks to the bridge.
    current_pose_.setX(msg->position[0]);
    current_pose_.setY(msg->position[1]);
    current_pose_.setZ(msg->position[2]);
    current_pose_.setVx(msg->velocity[0]);
    current_pose_.setVy(msg->velocity[1]);
    current_pose_.setVz(msg->velocity[2]);
    Eigen::Quaterniond q(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
    current_pose_.setOrientation(q);
  }

  void onControlMode(const px4_msgs::msg::VehicleControlMode::SharedPtr msg)
  {
    // NOTE: We pass the raw armed flag. The old bridge encoded "differs from
    // first observed", which is equivalent if the drone starts disarmed.
    armed_ = msg->flag_armed ? 1 : 0;
    e_.setStatus(armed_, landed_);
  }

  void onLandDetected(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg)
  {
    // NOTE: Raw landed flag. The bridge's old logic returned 1 when the
    // current state matched the *first observed* state — equivalent if the
    // drone starts landed.
    landed_ = msg->landed ? 1 : 0;
    e_.setStatus(armed_, landed_);
  }

  void onLandingSitePose(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    MavState platform;
    platform.setX(msg->pose.pose.position.x);
    platform.setY(msg->pose.pose.position.y);
    platform.setZ(msg->pose.pose.position.z);

    platform.setVx(msg->twist.twist.linear.x);
    platform.setVy(msg->twist.twist.linear.y);
    platform.setVz(msg->twist.twist.linear.z);

    Eigen::Quaterniond q(
      msg->pose.pose.orientation.w,
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z);
    platform.setOrientation(q);

    e_.setPlatformState(platform);
  }

  void onSafety(const ulisse_msgs::msg::Safety::SharedPtr msg)
  {
    // Battery has top priority — if its task is running, skip everything else.
    if (e_.isBatterySafetyTaskRunning) {
      return;
    }

    // Priority chain: battery, gps, safe_harbor, camera, going_platform, sonar.
    if (msg->isbatterygone) {
      e_.isBatterySafetyTaskRunning = 1;
      e_.batteryGoneHandler(*msg);
      RCLCPP_WARN(get_logger(), "Battery Safety Task Deployed");
      safety_ack_msg_.isbatterysafetytaskrunning = 1;
      safety_ack_pub_->publish(safety_ack_msg_);
    }
    else if (msg->isgpsgone && !e_.isGPSSafetyTaskRunning) {
      e_.isGPSSafetyTaskRunning = 1;
      e_.gpsGoneHandler();
      RCLCPP_WARN(get_logger(), "GPS Safety Task Deployed");
      safety_ack_msg_.isgpssafetytaskrunning = 1;
      safety_ack_pub_->publish(safety_ack_msg_);
    }
    else if (msg->goingsafeharbor && !e_.isGoingInSafeHarbor) {
      // If safe-harbor was triggered by camera failure, also clear the camera task.
      if (e_.isCameraSafetyTaskRunning) {
        e_.isCameraSafetyTaskRunning = 0;
        safety_ack_msg_.iscamerasafetytaskrunning = 0;
      }
      e_.isGoingInSafeHarbor = 1;
      e_.moveToSafeHarbor(*msg);
      RCLCPP_WARN(get_logger(), "Safe Harbor Safety Task Deployed");
      safety_ack_msg_.isgoinginsafeharbor = 1;
      safety_ack_pub_->publish(safety_ack_msg_);
    }
    else if (msg->iscameragone && !e_.isCameraSafetyTaskRunning && !e_.isGoingInSafeHarbor) {
      e_.isCameraSafetyTaskRunning = 1;
      e_.cameraGoneHandler(*msg);
      RCLCPP_WARN(get_logger(), "Camera Safety Task Deployed");
      TimeManager tm;
      safety_ack_msg_.iscamerasafetytaskrunning = 1;
      safety_ack_msg_.iscameragone_timestamp    = static_cast<double>(tm.getTimeMilliSecond());
      safety_ack_pub_->publish(safety_ack_msg_);
    }
    else if (msg->isgoingplatform
             && !e_.isGPSSafetyTaskRunning
             && !e_.isCameraSafetyTaskRunning
             && !e_.isGoingInSafeHarbor)
    {
      e_.isGoingToPlatform = 1;
      e_.batteryLimitHandler(*msg);
      RCLCPP_WARN(get_logger(), "Coming To Platform Safety Task Deployed");
      safety_ack_msg_.isgoingtoplatform = 1;
      safety_ack_pub_->publish(safety_ack_msg_);
    }
    else if (msg->issonargone
             && !e_.isSonarSafetyTaskRunning
             && !e_.isGPSSafetyTaskRunning
             && !e_.isCameraSafetyTaskRunning
             && !e_.isGoingInSafeHarbor
             && !e_.isGoingToPlatform)
    {
      e_.isSonarSafetyTaskRunning = 1;
      e_.sonarGoneHandler();
      RCLCPP_WARN(get_logger(), "Sonar Safety Task Deployed");
      safety_ack_msg_.issonarsafetytaskrunning = 1;
      safety_ack_pub_->publish(safety_ack_msg_);
    }

    // ---- Recovery transitions ----
    // GPS back online (still TODO in original).
    if (e_.isGPSSafetyTaskRunning && !msg->isgpsgone) {
      // TODO: implement GPS recovery handler.
    }
    // Camera back online — re-enable mission, reset landing state.
    if (e_.isCameraSafetyTaskRunning && !msg->iscameragone) {
      e_.isCameraSafetyTaskRunning = 0;
      safety_ack_msg_.iscamerasafetytaskrunning = 0;
      e_.resetLanding();
      safety_ack_pub_->publish(safety_ack_msg_);
    }
  }

  void onReleasePayloadFeedback(const std_msgs::msg::String::SharedPtr msg)
  {
    std::cout << msg->data << std::endl;
    // TODO: update Executioner::setRelPayFeedback signature to const std::string&.
    e_.setRelPayFeedback(msg->data);
    std::cout << "Feedback Received" << std::endl;
  }

  void onNavTaskStatus(const ulisse_msgs::msg::VehicleStatus::SharedPtr msg)
  {
    // TODO: update Executioner::setNavTaskStatus to take
    // const ulisse_msgs::msg::VehicleStatus& (it only reads stamp.sec and vehicle_state).
    e_.setNavTaskStatus(*msg);
  }

  // ===================== Periodic tick =====================

  void tick()
  {
    e_.setState(current_pose_);
    e_.run();
    if (e_.readyToPublish()) {
      std::cout << "publishing task" << std::endl;
      // TODO: change `Executioner::_actualTask` type to
      // ulisse_msgs::msg::ActualTask (field names preserved).
      actual_task_pub_->publish(e_._actualTask);
    }
  }

  // ===================== State =====================

  Executioner e_;
  Parser      p_;

  MavState                              current_pose_;
  int                                   armed_  {0};
  int                                   landed_ {1};   // assume drone starts landed
  ulisse_msgs::msg::SafetyAck      safety_ack_msg_ {};

  // Publishers
  rclcpp::Publisher<ulisse_msgs::msg::ActualTask>::SharedPtr  actual_task_pub_;
  rclcpp::Publisher<ulisse_msgs::msg::SafetyAck>::SharedPtr   safety_ack_pub_;

  // Subscriptions
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr      vision_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr   control_mode_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr  land_detected_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr             platform_pose_sub_;
  rclcpp::Subscription<ulisse_msgs::msg::Safety>::SharedPtr       safety_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr               rel_pay_fb_sub_;
  rclcpp::Subscription<ulisse_msgs::msg::VehicleStatus>::SharedPtr     nav_task_status_sub_;

  // Timer
  rclcpp::TimerBase::SharedPtr tick_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<ExecutionerNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    std::cerr << "Fatal: " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}