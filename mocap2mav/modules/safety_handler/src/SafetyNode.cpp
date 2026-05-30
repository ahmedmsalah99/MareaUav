// safety_handler_node.cpp
//
// ROS 2 port of the original LCM-based SafetyHandler main loop.
//
// Migration notes:
//   * No LCM is used here.
//   * Drone pose ("vision_position_estimate" in LCM) is now read from a ROS 2 topic
//     of the same name, type px4_msgs/VehicleOdometry. The bridge republishes
//     /fmu/out/vehicle_odometry onto this topic by default. Any alternative source
//     (mocap, custom estimator) can publish to "vision_position_estimate" and
//     override the source without changes here.
//   * The other PX 4 / sensor inputs (battery, gps, camera_info,
//     release_payload_feedback) are read directly from their native ROS 2 topics.
//     The bridge translation step for those topics can be removed once Executioner
//     and Automatic are also ported.
//   * Three internal LCM topics become custom ROS 2 messages:
//       - "safety"      -> ulisse_exec_msgs/msg/Safety
//       - "safety_ack"  -> ulisse_exec_msgs/msg/SafetyAck
//       - "actual_task" -> ulisse_exec_msgs/msg/ActualTask
//     Field names are kept identical to the old LCM structs so SafetyHandler.h
//     and SafetyParser.h need no rewrites.
//   * No Gazebo plugin topics are used by SafetyHandler, so no LCM-mirror
//     subscriptions are needed in this node.
//   * Original loop was a `Spinner(5).useMilli()` busy loop. Replaced by:
//       - event-driven callbacks (where the original key-off was a POLLIN), and
//       - one 5 ms wall timer for the periodic state-machine checks that
//         weren't strictly tied to one incoming message.
//   * SafetyHandler::checkRelPayFeedback() currently takes the LCM
//     exec::release_payload struct. Adapt it to take a `const std::string&`
//     (the `.data` field) -- see TODO below.

#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

// PX4 / sensor ROS 2 messages (replace the bridge-translated LCM equivalents)
#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/string.hpp>

// Custom ROS 2 messages mirroring the old LCM exec/* structs.
// (Define these in a new ulisse_exec_msgs package with identical field names.)
#include <ulisse_exec_msgs/msg/safety.hpp>
#include <ulisse_exec_msgs/msg/safety_ack.hpp>
#include <ulisse_exec_msgs/msg/actual_task.hpp>

// Unchanged application code
#include "SafetyHandler.h"
#include "SafetyParser.h"
#include "common/MavState.h"
#include "common/conversions.h"

using namespace common;
using namespace std::chrono_literals;

class SafetyHandlerNode : public rclcpp::Node
{
public:
  SafetyHandlerNode()
  : rclcpp::Node("safety_handler")
  {
    // -------- Load safety parameters (same logic as the old main) --------
    std::string listPath;
    std::ifstream listF(".direnv/lists.txt");
    if (!listF.is_open()) {
      RCLCPP_FATAL(get_logger(), "Environment paths not set (.direnv/lists.txt missing)");
      throw std::runtime_error("environment not configured");
    }
    std::getline(listF, listPath);

    sp_.loadFile(listPath + "/../safety_params.txt");
    RCLCPP_INFO(get_logger(), "Parsing safety parameters...");
    if (!sp_.parse()) {
      RCLCPP_FATAL(get_logger(), "Unable to parse the safety parameters file");
      throw std::runtime_error("safety params parse failed");
    }
    handler_.setParamsStruct(sp_.getParamsMapParsed());
    RCLCPP_INFO(get_logger(), "Target dispersed: %d", static_cast<int>(handler_.target_dispersed));

    // Compute safe-harbor and platform-rdv local coords (same logic as old main)
    initLocalCoords();

    // -------- Initialize the outgoing safety message --------
    safety_msg_.isbatterygone    = 0;
    safety_msg_.iscameragone     = 0;
    safety_msg_.isgpsgone        = 0;
    safety_msg_.issonargone      = 0;
    safety_msg_.goingsafeharbor  = 0;
    safety_msg_.isgoingplatform  = 0;

    // -------- QoS --------
    auto px4_qos      = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

    // -------- Publisher --------
    // Was: handler6.publish("safety", &safety);
    safety_pub_ = create_publisher<ulisse_exec_msgs::msg::Safety>("safety", reliable_qos);

    // -------- Subscriptions --------
    // Battery: was LCM "battery" (bridge-translated from PX4). Now direct from PX4.
    battery_sub_ = create_subscription<px4_msgs::msg::BatteryStatus>(
      "/fmu/out/battery_status", px4_qos,
      std::bind(&SafetyHandlerNode::onBattery, this, std::placeholders::_1));

    // GPS: was LCM "gps_status" (bridge-translated from PX4). Now direct from PX4.
    gps_sub_ = create_subscription<px4_msgs::msg::SensorGps>(
      "/fmu/out/vehicle_gps_position", px4_qos,
      std::bind(&SafetyHandlerNode::onGps, this, std::placeholders::_1));

    // Camera info: was LCM "camera_info" (bridge-stamped). Now direct from the camera info topic.
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "/apriltag/image/camera_info", 1,
      std::bind(&SafetyHandlerNode::onCameraInfo, this, std::placeholders::_1));

    // Drone pose: was LCM "vision_position_estimate".
    // We subscribe to a ROS 2 topic of the same name (px4_msgs/VehicleOdometry).
    // The bridge republishes /fmu/out/vehicle_odometry as "vision_position_estimate"
    // by default, but any other source (mocap, custom estimator) can publish to this
    // topic and override the source without touching app nodes.
    odom_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
      "vision_position_estimate", px4_qos,
      std::bind(&SafetyHandlerNode::onOdometry, this, std::placeholders::_1));

    // Internal: actual_task from Executioner.
    actual_task_sub_ = create_subscription<ulisse_exec_msgs::msg::ActualTask>(
      "actual_task", reliable_qos,
      std::bind(&SafetyHandlerNode::onActualTask, this, std::placeholders::_1));

    // Internal: safety_ack from Executioner.
    safety_ack_sub_ = create_subscription<ulisse_exec_msgs::msg::SafetyAck>(
      "safety_ack", reliable_qos,
      std::bind(&SafetyHandlerNode::onSafetyAck, this, std::placeholders::_1));

    // release_payload_feedback: already a std_msgs/String in the ROS 2 world (the bridge
    // just wrapped it back into LCM). Now we read it natively.
    rel_pay_fb_sub_ = create_subscription<std_msgs::msg::String>(
      "release_payload_feedback", reliable_qos,
      std::bind(&SafetyHandlerNode::onReleasePayloadFeedback, this, std::placeholders::_1));

    // -------- Periodic tick (5 ms, matching the original Spinner(5).useMilli()) --------
    // The original main loop checked time-driven conditions (camera-still-dead,
    // camera-dead-detection during LAND/SEARCH) on every iteration regardless of which
    // topic had data. Keep that semantics by evaluating them in a wall timer.
    tick_timer_ = create_wall_timer(5ms, std::bind(&SafetyHandlerNode::tick, this));

    RCLCPP_INFO(get_logger(), "SafetyHandler node ready");
  }

private:
  // ===================== Callbacks =====================

  void onBattery(const px4_msgs::msg::BatteryStatus::SharedPtr msg)
  {
    const float remaining = msg->remaining;

    // Exactly mirrors the original "if battery POLLIN" branch in the old main loop.
    if (handler_.handleBattery(remaining) && !safety_msg_.isbatterygone) {
      // If we're already landing, let landing finish; otherwise emergency-land.
      if (current_task_action_ != actions::LAND) {
        safety_msg_.isbatterygone = 1;
        safety_msg_.params[0] = handler_.getParam(0);
        safety_msg_.params[1] = handler_.getParam(1);
        safety_msg_.params[2] = handler_.getParam(2);
        safety_pub_->publish(safety_msg_);
        RCLCPP_WARN(get_logger(), "CRITIC: LOW BATTERY!!! Performing emergency land");
      }
    } else if ((!handler_.target_dispersed || handler_.payload_released)
               && !safety_msg_.isgoingplatform
               && !safety_msg_.isbatterygone) {
      if (handler_.handleBatteryReturn(remaining)
          && current_task_action_ != actions::LAND
          && current_task_target_ != "platform"
          && current_task_target_ != "safe_harbor")
      {
        safety_msg_.isgoingplatform = 1;
        safety_msg_.params[0] = handler_.getParam(9);
        safety_msg_.params[1] = handler_.getParam(12);
        safety_msg_.params[2] = handler_.getParam(13);
        safety_msg_.target_coord[0] = handler_.plat_local_coord[0];
        safety_msg_.target_coord[1] = handler_.plat_local_coord[1];
        safety_msg_.target_coord[2] = handler_.plat_local_coord[2];
        safety_msg_.params[3] = computeDesiredYaw(current_pose_, handler_.plat_local_coord);
        safety_msg_.params[4] = handler_.platform;
        safety_pub_->publish(safety_msg_);
        RCLCPP_WARN(get_logger(), "BATTERY LIMIT!!! Drone coming back to platform");
      }
    }
  }

  void onGps(const px4_msgs::msg::SensorGps::SharedPtr msg)
  {
    // In the original main the GPS-status branch is almost entirely commented-out
    // TODOs. We just cache the fix_type for any future logic, same as the bridge did.
    last_gps_fix_type_ = msg->fix_type;
  }

  void onCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr /*msg*/)
  {
    // Update freshness timestamp. SafetyHandler::handleCamera() compares this against
    // "now" to decide whether the camera is dead.
    last_camera_msg_time_ms_ = nowMs();

    // The old main's "camera POLLIN" branch also did the "camera came back online"
    // recovery transition right here. Replicate that to preserve original behavior:
    // the recovery fires only when a fresh frame arrives.
    if (safety_msg_.iscameragone && !safety_ack_.isgoinginsafeharbor) {
      safety_msg_.iscameragone = 0;
      safety_pub_->publish(safety_msg_);
      RCLCPP_INFO(get_logger(), "GOOD NEWS: CAMERA WORKS AGAIN!!! Coming back to mission");
    }
  }

  void onOdometry(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
  {
    // vision_position_estimate is already ENU here: the bridge does the
    // NED→ENU axis swap once when it republishes /fmu/out/vehicle_odometry.
    // TODO: verify the setter names on MavState (setX/setY/setZ/setVx/setVy/setVz/setOrientation).
    current_pose_.setX(msg->position[0]);
    current_pose_.setY(msg->position[1]);
    current_pose_.setZ(msg->position[2]);

    current_pose_.setVx(msg->velocity[0]);
    current_pose_.setVy(msg->velocity[1]);
    current_pose_.setVz(msg->velocity[2]);

    Eigen::Quaterniond q(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
    current_pose_.setOrientation(q);
  }

  void onActualTask(const ulisse_exec_msgs::msg::ActualTask::SharedPtr msg)
  {
    current_task_action_ = msg->action;
    current_task_target_ = msg->target;
  }

  void onSafetyAck(const ulisse_exec_msgs::msg::SafetyAck::SharedPtr msg)
  {
    safety_ack_ = *msg;
  }

  void onReleasePayloadFeedback(const std_msgs::msg::String::SharedPtr msg)
  {
    // TODO: update SafetyHandler::checkRelPayFeedback() to take `const std::string&`
    // instead of the LCM exec::release_payload struct. The struct only had a `.data`
    // string field, so the change is trivial.
    handler_.checkRelPayFeedback(msg->data);
  }

  // ===================== Periodic tick =====================

  void tick()
  {
    // ---- Camera-dead detection (only meaningful during LAND or SEARCH) ----
    // Matches the original: every loop iteration, regardless of which topic had data.
    if ((current_task_action_ == actions::LAND || current_task_action_ == actions::SEARCH)
        && !safety_msg_.iscameragone
        && handler_.handleCamera(last_camera_msg_time_ms_))
    {
      safety_msg_.iscameragone = 1;
      safety_msg_.params[0] = handler_.getParam(7);
      safety_msg_.params[1] = handler_.getParam(9);
      safety_msg_.params[2] = handler_.getParam(10);
      safety_pub_->publish(safety_msg_);
      RCLCPP_WARN(get_logger(), "CRITIC: CAMERA STOPPED WORKING!!! Safety task deployed");
    }

    // ---- Camera still dead past the second threshold -> go safe harbor ----
    if (safety_msg_.iscameragone
        && safety_ack_.iscamerasafetytaskrunning
        && !safety_msg_.goingsafeharbor
        && handler_.handleSafeHarbor(safety_ack_.iscameragone_timestamp)
        && !safety_msg_.isbatterygone)
    {
      safety_msg_.goingsafeharbor = 1;
      safety_msg_.params[0] = handler_.getParam(0);
      safety_msg_.params[1] = handler_.getParam(1);
      safety_msg_.params[2] = handler_.getParam(2);
      safety_msg_.target_coord[0] = handler_.sh_local_coord[0];
      safety_msg_.target_coord[1] = handler_.sh_local_coord[1];
      safety_msg_.target_coord[2] = handler_.sh_local_coord[2];
      safety_msg_.params[3] = handler_.getParam(6);
      safety_msg_.params[4] = handler_.getParam(8);
      safety_msg_.params[5] = handler_.getParam(9);
      safety_msg_.params[6] = computeDesiredYaw(current_pose_, handler_.sh_local_coord);
      safety_msg_.params[7] = deg2rad(handler_.getParam(11));
      safety_pub_->publish(safety_msg_);
      RCLCPP_WARN(get_logger(), "CRITIC: Going in safe harbor!!!");
    }
  }

  // ===================== Helpers =====================

  void initLocalCoords()
  {
    double target_global[3];
    double home[3];

    // Safe-harbor target
    target_global[0] = handler_.getParam(3);
    target_global[1] = handler_.getParam(4);
    target_global[2] = handler_.getParam(5);
    home[0] = handler_.home_longitude;
    home[1] = handler_.home_latitude;
    home[2] = handler_.home_altitude;
    globalToLocalCoords(target_global, home);
    handler_.sh_local_coord[0] = target_global[0];
    handler_.sh_local_coord[1] = target_global[1];
    handler_.sh_local_coord[2] = target_global[2];

    // Platform rendezvous target
    target_global[0] = handler_.platform_rdv_lon;
    target_global[1] = handler_.platform_rdv_lat;
    target_global[2] = handler_.platform_rdv_alt;
    globalToLocalCoords(target_global, home);
    handler_.plat_local_coord[0] = target_global[0];
    handler_.plat_local_coord[1] = target_global[1];
    handler_.plat_local_coord[2] = target_global[2];
  }

  static double nowMs()
  {
    return static_cast<double>(
      std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1));
  }

  // ===================== State =====================

  SafetyHandler handler_;
  SafetyParser  sp_;

  // Cached state (was held in CallbackHandler `call` in the LCM version)
  MavState                              current_pose_;
  int                                   current_task_action_   {actions::NONE}; // adjust default to whatever your enum uses
  std::string                           current_task_target_   {""};
  int                                   last_gps_fix_type_     {0};
  double                                last_camera_msg_time_ms_ {0.0};
  ulisse_exec_msgs::msg::SafetyAck      safety_ack_            {};
  ulisse_exec_msgs::msg::Safety         safety_msg_            {};

  // Publishers
  rclcpp::Publisher<ulisse_exec_msgs::msg::Safety>::SharedPtr           safety_pub_;

  // Subscriptions
  rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr         battery_sub_;
  rclcpp::Subscription<px4_msgs::msg::SensorGps>::SharedPtr             gps_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr         camera_info_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr       odom_sub_;
  rclcpp::Subscription<ulisse_exec_msgs::msg::ActualTask>::SharedPtr    actual_task_sub_;
  rclcpp::Subscription<ulisse_exec_msgs::msg::SafetyAck>::SharedPtr     safety_ack_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr                rel_pay_fb_sub_;

  // Timer
  rclcpp::TimerBase::SharedPtr tick_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<SafetyHandlerNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    std::cerr << "Fatal: " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}