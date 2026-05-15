// bridge_node.cpp
//
// LCM <-> ROS 2 bridge after the application migration.
//
// What this node does:
//   * Republishes /fmu/out/vehicle_odometry as ROS 2 "vision_position_estimate"
//     (with NED->ENU axis swap). App nodes subscribe to this; any other source
//     (mocap, custom estimator) can publish to the same topic to override.
//   * Republishes /ulisse/nav_filter/data as ROS 2 "nav_pose"
//     (with global->local lat/lon conversion using a captured home).
//   * Plugin-input mirrors (ROS 2 -> LCM, fed to Gazebo plugins):
//       vision_position_estimate, nav_pose, Landing_Site/pose_cmd,
//       release_payload_feedback, gps_status
//   * Plugin-output mirror (LCM -> ROS 2, from landingPlat plugin):
//       Landing_Site/pose
//   * Translates ROS 2 "local_position_sp" (nav_msgs/Odometry) to PX 4
//     /fmu/in/trajectory_setpoint, with ENU->NED swap and the slew-rate filter.
//     POSITION vs VELOCITY is read from header.frame_id ("POSITION"|"VELOCITY").
//   * Publishes /fmu/in/offboard_control_mode at 50 ms, with position/velocity
//     flags tracking the last local_position_sp type.
//   * Translates "actual_task" action enum to /drone_state (Float32) for the
//     RPi-side fsm_raspicam package.
//
// Dropped from the old bridge:
//   * /fmu/out/battery_status -> LCM "battery"           (apps read PX 4 direct)
//   * /apriltag/image/camera_info -> LCM "camera_info"    (apps read direct)
//   * /fmu/out/vehicle_control_mode + vehicle_land_detected -> LCM "state"
//                                                       (apps read direct)
//   * /ulisse/ctrl/vehicle_status -> LCM "nav_task_status" (apps read direct)
//   * LCM "release_payload" -> ROS 2 /release_payload    (Automatic.cpp should
//        publish to ROS 2 /release_payload directly; downstream reads native).

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <lcm/lcm-cpp.hpp>

// PX 4
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>

// Standard / ulisse
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <ulisse_msgs/msg/nav_filter_data.hpp>

// Custom app messages
#include <ulisse_msgs/msg/actual_task.hpp>

// LCM message types (unchanged from before).
#include <lcm_messages/geometry/pose.hpp>
#include <lcm_messages/exec/gps_status.hpp>
#include <lcm_messages/exec/release_payload.hpp>

// Project utilities
#include "common/MavState.h"      // common::actions::*
#include "common/conversions.h"   // globalToLocalCoords, yawFromQuaternion
#include "common/common.h"
#include "common/CallbackHandler.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class BridgeNode : public rclcpp::Node
{
public:
  BridgeNode()
  : rclcpp::Node("ros2lcm_bridge")
  {
    if (!lcm_.good()) {
      RCLCPP_FATAL(get_logger(), "Failed to initialize LCM");
      throw std::runtime_error("lcm init failed");
    }

    auto px4_qos      = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

    // ==================== ROS 2 publishers ====================

    // Republisher outputs.
    vision_position_estimate_pub_ = create_publisher<px4_msgs::msg::VehicleOdometry>(
      "vision_position_estimate", px4_qos);
    nav_pose_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "nav_pose", reliable_qos);

    // Plugin-output mirror.
    landing_site_pose_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "Landing_Site/pose", reliable_qos);

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
    control_mode_sub_ = create_subscription<px4_msgs::msg::VehicleControlMode>(
      "/fmu/out/vehicle_control_mode", px4_qos,
      std::bind(&BridgeNode::onControlMode, this, _1));
    land_detected_sub_ = create_subscription<px4_msgs::msg::VehicleLandDetected>(
      "/fmu/out/vehicle_land_detected", px4_qos,
      std::bind(&BridgeNode::onLandDetected, this, _1));

    // -- Plugin-input mirrors (ROS 2 -> LCM) --
    // NOTE: these subscribe to topics this same node also publishes (vision_position_estimate
    // and nav_pose). ROS 2 delivers self-publishes back to the same node, so the mirror picks
    // up the republisher output. If a different node publishes the topic, the mirror still works.
    vis_pos_for_lcm_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
      "vision_position_estimate", px4_qos,
      std::bind(&BridgeNode::onVisionPosForLcm, this, _1));
    nav_pose_for_lcm_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "nav_pose", reliable_qos,
      std::bind(&BridgeNode::onNavPoseForLcm, this, _1));
    landing_site_pose_cmd_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "Landing_Site/pose_cmd", reliable_qos,
      std::bind(&BridgeNode::onLandingSitePoseCmd, this, _1));
    rel_pay_fb_sub_ = create_subscription<std_msgs::msg::String>(
      "release_payload_feedback", reliable_qos,
      std::bind(&BridgeNode::onReleasePayloadFb, this, _1));
    gps_sub_ = create_subscription<px4_msgs::msg::SensorGps>(
      "/fmu/out/vehicle_gps_position", px4_qos,
      std::bind(&BridgeNode::onGpsForLcm, this, _1));

    // -- Command translator --
    local_position_sp_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "local_position_sp", reliable_qos,
      std::bind(&BridgeNode::onLocalPositionSp, this, _1));

    // -- actual_task -> /drone_state translator --
    actual_task_sub_ = create_subscription<ulisse_msgs::msg::ActualTask>(
      "actual_task", reliable_qos,
      std::bind(&BridgeNode::onActualTask, this, _1));

    // ==================== LCM subscriptions ====================

    lcm_sub_landing_pose_ = lcm_.subscribe(
      "Landing_Site/pose", &BridgeNode::onLcmLandingSitePose, this);
    lcm_sub_landing_pose_->setQueueCapacity(1);

    // ==================== Timers ====================

    // Offboard heartbeat (50 ms, matches old bridge).
    offboard_timer_ = create_wall_timer(
      50ms, std::bind(&BridgeNode::publishOffboardMode, this));

    // Drain LCM at 1 ms — non-blocking, no thread required.
    lcm_poll_timer_ = create_wall_timer(1ms, [this]() {
      while (lcm_.handleTimeout(0) > 0) { /* keep draining */ }
    });

    RCLCPP_INFO(get_logger(), "Bridge node ready");
  }

  ~BridgeNode() override
  {
    if (lcm_sub_landing_pose_ != nullptr) {
      lcm_.unsubscribe(lcm_sub_landing_pose_);
    }
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
      nav_home_lat_ = msg->inertialframe_linear_position.latlong.latitude;
      nav_home_lon_ = msg->inertialframe_linear_position.latlong.longitude;
      RCLCPP_INFO(get_logger(), "NAV home lat=%.7f lon=%.7f", nav_home_lat_, nav_home_lon_);
      first_nav_state_ = false;
    }

    double coords[3] = {
      msg->inertialframe_linear_position.latlong.longitude,
      msg->inertialframe_linear_position.latlong.latitude,
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

  void onControlMode(const px4_msgs::msg::VehicleControlMode::SharedPtr msg)
  {
    offboard_enabled_ = msg->flag_control_offboard_enabled;
  }

  void onLandDetected(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg)
  {
    landed_ = msg->landed;
  }

  // ===================== Plugin-input mirrors (ROS 2 -> LCM) =====================

  void onVisionPosForLcm(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
  {
    // Already ENU (this topic is the republisher's output, or any equivalent ENU source).
    geometry::pose p {};
    p.position[0] = msg->position[0];
    p.position[1] = msg->position[1];
    p.position[2] = msg->position[2];
    p.velocity[0] = msg->velocity[0];
    p.velocity[1] = msg->velocity[1];
    p.velocity[2] = msg->velocity[2];
    p.orientation[0] = msg->q[0];
    p.orientation[1] = msg->q[1];
    p.orientation[2] = msg->q[2];
    p.orientation[3] = msg->q[3];
    p.isValid = offboard_enabled_;
    lcm_.publish("vision_position_estimate", &p);
  }

  void onNavPoseForLcm(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    geometry::pose p {};
    p.position[0] = msg->pose.pose.position.x;
    p.position[1] = msg->pose.pose.position.y;
    p.position[2] = msg->pose.pose.position.z;
    p.velocity[0] = msg->twist.twist.linear.x;
    p.velocity[1] = msg->twist.twist.linear.y;
    p.velocity[2] = msg->twist.twist.linear.z;
    p.orientation[0] = msg->pose.pose.orientation.w;
    p.orientation[1] = msg->pose.pose.orientation.x;
    p.orientation[2] = msg->pose.pose.orientation.y;
    p.orientation[3] = msg->pose.pose.orientation.z;
    lcm_.publish("nav_pose", &p);
  }

  void onLandingSitePoseCmd(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    geometry::pose p {};
    p.position[0] = msg->pose.pose.position.x;
    p.position[1] = msg->pose.pose.position.y;
    p.position[2] = msg->pose.pose.position.z;
    p.velocity[0] = msg->twist.twist.linear.x;
    p.velocity[1] = msg->twist.twist.linear.y;
    p.velocity[2] = msg->twist.twist.linear.z;
    p.orientation[0] = msg->pose.pose.orientation.w;
    p.orientation[1] = msg->pose.pose.orientation.x;
    p.orientation[2] = msg->pose.pose.orientation.y;
    p.orientation[3] = msg->pose.pose.orientation.z;
    // If geometry::pose has a separate yaw field, derive it from the quaternion.
    // TODO: confirm signature of yawFromQuaternion (Eigen::Quaterniond vs raw double[4]).
    Eigen::Quaterniond q(
      p.orientation[0], p.orientation[1], p.orientation[2], p.orientation[3]);
    p.yaw = yawFromQuaternion(q);
    lcm_.publish("Landing_Site/pose_cmd", &p);
  }

  void onReleasePayloadFb(const std_msgs::msg::String::SharedPtr msg)
  {
    exec::release_payload fb {};
    fb.data = msg->data;
    lcm_.publish("release_payload_feedback", &fb);
  }

  void onGpsForLcm(const px4_msgs::msg::SensorGps::SharedPtr msg)
  {
    exec::gps_status gs {};
    gs.lon = 1.0e-7 * msg->longitude_deg;
    gs.lat = 1.0e-7 * msg->latitude_deg;
    // TODO: confirm altitude_msl_m units against your px4_msgs version
    // (the original bridge used a 1e-3 scaling, which is suspicious for "_m").
    gs.alt = 1.0e-3 * msg->altitude_msl_m;
    gs.status = msg->fix_type;
    gs.timestamp = static_cast<double>(
      std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1));
    lcm_.publish("gps_status", &gs);
  }

  // ===================== Plugin-output mirror (LCM -> ROS 2) =====================

  void onLcmLandingSitePose(const lcm::ReceiveBuffer * /*rbuf*/,
                            const std::string & /*chan*/,
                            const geometry::pose * msg)
  {
    nav_msgs::msg::Odometry out;
    out.header.stamp = this->now();
    out.header.frame_id = "Landing_Site";
    out.pose.pose.position.x = msg->position[0];
    out.pose.pose.position.y = msg->position[1];
    out.pose.pose.position.z = msg->position[2];
    out.twist.twist.linear.x = msg->velocity[0];
    out.twist.twist.linear.y = msg->velocity[1];
    out.twist.twist.linear.z = msg->velocity[2];
    out.pose.pose.orientation.w = msg->orientation[0];
    out.pose.pose.orientation.x = msg->orientation[1];
    out.pose.pose.orientation.y = msg->orientation[2];
    out.pose.pose.orientation.z = msg->orientation[3];
    landing_site_pose_pub_->publish(out);
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
      setpoint.velocity = {NAN, NAN, NAN};

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

  lcm::LCM            lcm_;
  lcm::Subscription * lcm_sub_landing_pose_ {nullptr};

  // Latest ENU drone pose (republisher output) used for gt_setpoint.
  px4_msgs::msg::VehicleOdometry last_drone_pose_enu_ {};
  // Latest flags used by other paths.
  bool offboard_enabled_ {false};
  bool landed_ {true};

  // Nav filter home (captured on first message).
  bool   first_nav_state_ {true};
  double nav_home_lat_    {0.0};
  double nav_home_lon_    {0.0};

  // Slew-rate state for the position command translator.
  static constexpr double kMaxVelocity = 1.0;  // m/s
  bool         first_pos_sp_ {true};
  rclcpp::Time pos_sp_last_time_;
  double       prev_pos_sp_[3] {0.0, 0.0, 0.0};

  // Offboard mode flags reflect the type of the last accepted local_position_sp.
  bool position_enabled_ {true};
  bool velocity_enabled_ {false};

  // ---- Publishers ----
  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr      vision_position_estimate_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr             nav_pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr             landing_site_pose_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr   trajectory_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr   gt_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr  offboard_control_mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr              drone_state_pub_;

  // ---- Subscriptions ----
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr      odom_sub_;
  rclcpp::Subscription<ulisse_msgs::msg::NavFilterData>::SharedPtr     nav_filter_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr   control_mode_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr  land_detected_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr      vis_pos_for_lcm_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr             nav_pose_for_lcm_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr             landing_site_pose_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr               rel_pay_fb_sub_;
  rclcpp::Subscription<px4_msgs::msg::SensorGps>::SharedPtr            gps_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr             local_position_sp_sub_;
  rclcpp::Subscription<ulisse_msgs::msg::ActualTask>::SharedPtr   actual_task_sub_;

  // ---- Timers ----
  rclcpp::TimerBase::SharedPtr offboard_timer_;
  rclcpp::TimerBase::SharedPtr lcm_poll_timer_;
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