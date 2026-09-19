// automatic_node.cpp — ROS 2 port with timer-driven control loop

#include <chrono>
#include <cmath>
#include <deque>
#include <iostream>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <ultrasonic_sensor/msg/ultrasonic_msg.hpp>
#include <ulisse_msgs/msg/actual_task.hpp>
#include <ulisse_msgs/msg/safety_ack.hpp>

#include "guidance_system/autom/Automatic.h"
#include "guidance_system/autom/Command/TakeOff.hpp"
#include "common/MavState.h"
#include "common/conversions.h"

using namespace common;
using namespace std::chrono_literals;

// ─────────────────────────── helpers ────────────────────────────────────────

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
  out.IsValid = true;
}

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
  out.IsValid = true;
}

static nav_msgs::msg::Odometry odomFromMavState(const MavState & s)
{
  nav_msgs::msg::Odometry msg;
  msg.header.frame_id =
    (s.getType() == MavState::type::POSITION) ? "POSITION" : "VELOCITY";
  msg.pose.pose.position.x    = s.getX();
  msg.pose.pose.position.y    = s.getY();
  msg.pose.pose.position.z    = s.getZ();
  Eigen::Quaterniond q        = s.getOrientation();
  msg.pose.pose.orientation.w = q.w();
  msg.pose.pose.orientation.x = q.x();
  msg.pose.pose.orientation.y = q.y();
  msg.pose.pose.orientation.z = q.z();
  msg.twist.twist.linear.x    = s.getVx();
  msg.twist.twist.linear.y    = s.getVy();
  msg.twist.twist.linear.z    = s.getVz();
  return msg;
}

// ─────────────────────────── node ───────────────────────────────────────────

class AutomaticNode : public rclcpp::Node
{
public:
  AutomaticNode()
  : rclcpp::Node("automatic")
  {
    auto px4_qos      = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

    // ── Publishers ──────────────────────────────────────────────────────────
    local_position_sp_pub_     =
      create_publisher<nav_msgs::msg::Odometry>("local_position_sp",       reliable_qos);
    vehicle_command_pub_       =
      create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", px4_qos);
    landing_site_pose_cmd_pub_ =
      create_publisher<nav_msgs::msg::Odometry>("Landing_Site/pose_cmd",   reliable_qos);
    plat_rob_pub_              =
      create_publisher<nav_msgs::msg::Odometry>("platRob",                 reliable_qos);
    pred_horizon_pub_          =
      create_publisher<std_msgs::msg::Float64>("spa/prediction_horizon",   reliable_qos);
    landing_diag_pub_          =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>("landing/diagnostics", reliable_qos);

    // ── Subscriptions ───────────────────────────────────────────────────────

    // Drone pose — state update ONLY; no longer the control tick.
    vision_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
      "vision_position_estimate", px4_qos,
      std::bind(&AutomaticNode::onVisionPos, this, std::placeholders::_1));

    platform_pose_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "Landing_Site/pose", reliable_qos,
      std::bind(&AutomaticNode::onPlatformPose, this, std::placeholders::_1));

    actual_task_sub_ = create_subscription<ulisse_msgs::msg::ActualTask>(
      "actual_task", reliable_qos,
      std::bind(&AutomaticNode::onActualTask, this, std::placeholders::_1));

    platform_center_sub_ = create_subscription<geometry_msgs::msg::TransformStamped>(
      "/platform_center", reliable_qos,
      std::bind(&AutomaticNode::onPlatformCenter, this, std::placeholders::_1));

    rescue_sub_ = create_subscription<geometry_msgs::msg::TransformStamped>(
      "/rescue", reliable_qos,
      std::bind(&AutomaticNode::onRescue, this, std::placeholders::_1));

    ultrasonic_sub_ = create_subscription<ultrasonic_sensor::msg::UltrasonicMsg>(
      "UltrasonicSensor/platform", reliable_qos,
      std::bind(&AutomaticNode::onUltrasonic, this, std::placeholders::_1));

    safety_ack_sub_ = create_subscription<ulisse_msgs::msg::SafetyAck>(
      "safety_ack", reliable_qos,
      std::bind(&AutomaticNode::onSafetyAck, this, std::placeholders::_1));

    gps_sub_ = create_subscription<px4_msgs::msg::SensorGps>(
      "/fmu/out/vehicle_gps_position", px4_qos,
      std::bind(&AutomaticNode::onGps, this, std::placeholders::_1));

    rel_pay_fb_sub_ = create_subscription<std_msgs::msg::String>(
      "release_payload_feedback", reliable_qos,
      std::bind(&AutomaticNode::onReleasePayloadFeedback, this, std::placeholders::_1));

    // ── Sea-state (SPA predictor + heave estimator) ──────────────────────────
    spa_estimate_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "spa/estimate", rclcpp::QoS(rclcpp::KeepLast(1)),
      std::bind(&AutomaticNode::onSpaEstimate, this, std::placeholders::_1));

    spa_prediction_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "spa/prediction", rclcpp::QoS(rclcpp::KeepLast(1)),
      std::bind(&AutomaticNode::onSpaPrediction, this, std::placeholders::_1));

    heave_pos_sub_ = create_subscription<std_msgs::msg::Float64>(
      "heave/position", rclcpp::QoS(rclcpp::KeepLast(1)),
      std::bind(&AutomaticNode::onHeavePosition, this, std::placeholders::_1));

    heave_vel_sub_ = create_subscription<std_msgs::msg::Float64>(
      "heave/velocity", rclcpp::QoS(rclcpp::KeepLast(1)),
      std::bind(&AutomaticNode::onHeaveVelocity, this, std::placeholders::_1));

    heave_pred_pos_sub_ = create_subscription<std_msgs::msg::Float64>(
      "heave/prediction_position", rclcpp::QoS(rclcpp::KeepLast(1)),
      std::bind(&AutomaticNode::onHeavePredPosition, this, std::placeholders::_1));

    heave_pred_vel_sub_ = create_subscription<std_msgs::msg::Float64>(
      "heave/prediction_velocity", rclcpp::QoS(rclcpp::KeepLast(1)),
      std::bind(&AutomaticNode::onHeavePredVelocity, this, std::placeholders::_1));

    // Comp-phase heave prediction, led by the drone's control+comm delay
    // instead of distance-to-platform — see SeaState.h.
    heave_pred_pos_comp_sub_ = create_subscription<std_msgs::msg::Float64>(
      "heave/prediction_position_comp", rclcpp::QoS(rclcpp::KeepLast(1)),
      std::bind(&AutomaticNode::onHeavePredPositionComp, this, std::placeholders::_1));

    heave_pred_vel_comp_sub_ = create_subscription<std_msgs::msg::Float64>(
      "heave/prediction_velocity_comp", rclcpp::QoS(rclcpp::KeepLast(1)),
      std::bind(&AutomaticNode::onHeavePredVelocityComp, this, std::placeholders::_1));

    // Pad's raw IMU (real IMU, not the landingplat plugin's simulated one —
    // same source heave_estimator now consumes) — used by the touchdown
    // detector's accel-matching test.
    pad_imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/adnav_node/imu_raw", rclcpp::SensorDataQoS(),
      std::bind(&AutomaticNode::onPadImu, this, std::placeholders::_1));

    // Drone's own PX4 acceleration estimate — same purpose.
    drone_accel_sub_ = create_subscription<px4_msgs::msg::SensorCombined>(
      "/fmu/out/sensor_combined", px4_qos,
      std::bind(&AutomaticNode::onDroneAccel, this, std::placeholders::_1));

    // ── Sensor-staleness timer (1 Hz — matches the original 1-second Duration) ──
    //
    // In the LCM loop the "else { if isExpired() }" branches ran on every loop
    // iteration (~ vision rate, typically 30–100 Hz). Checking at 1 Hz is enough
    // because the threshold itself is 1 second; checking more often wastes CPU
    // but checking less often would delay the stale notification.
    staleness_timer_ = create_wall_timer(
      1000ms,
      std::bind(&AutomaticNode::checkSensorFreshness, this));

    // ── Control-loop timer ───────────────────────────────────────────────────
    //
    // This is the direct replacement for the original LCM while(0==handler.handle())
    // loop.  The old loop ran at whatever rate vision_position_estimate arrived
    // (typically 30–100 Hz for PX4 EKF output).  50 Hz (20 ms) is a safe match
    // that keeps latency low without saturating the flight controller's setpoint
    // queue.  Tune to match your actual odometry publication rate if needed.
    //
    // Key insight: the original code called executeCommand() on *every* loop
    // iteration regardless of whether any sensor data was fresh. The timer
    // faithfully reproduces that behaviour. Commands like Land that run a
    // multi-phase state machine (descend → vision-guided → ultrasonic → disarm)
    // need this unconditional tick to advance their internal FSM even when some
    // sensors are temporarily absent.
    control_timer_ = create_wall_timer(
      20ms,
      std::bind(&AutomaticNode::controlLoop, this));

    // Mark sensor timestamps as ancient so isExpired() fires immediately.
    rclcpp::Time epoch(0, 0, get_clock()->get_clock_type());
    last_vision_time_     = epoch;
    last_rescue_time_     = epoch;
    last_ultrasonic_time_ = epoch;
    last_spa_time_        = epoch;
    last_heave_time_      = epoch;
    last_spa_est_stamp_   = epoch;
    last_loop_time_       = epoch;
    last_pad_imu_time_    = epoch;
    last_drone_accel_time_ = epoch;

    RCLCPP_INFO(get_logger(), "Automatic node ready (control loop 50 Hz)");
  }

private:
  // ══════════════════════════════════════════════════════════════════════════
  // Control loop  — replaces while(0 == handler.handle()) { ... executeCommand() }
  // ══════════════════════════════════════════════════════════════════════════
  void controlLoop()
  {
    if (waiting_) {
      // No task has arrived yet — mirror the original "if(!waiting)" guard.
      return;
    }

    // MOVE-to-platform: keep the task target locked to the live platform pose
    // (simulation only), exactly as the original did on each loop iteration.
    if (current_action_ == actions::MOVE
        && current_target_ == "platform"
        && autom_.env_sim)
    {
      // std::cout << "in move" << std::endl;
      autom_._actualTask.x = platform_state_.getX();
      autom_._actualTask.y = platform_state_.getY();
    }
    // Refresh sea-state every control tick (50 Hz) and hand it to the guidance
    // for the calm-sea landing gate:
    //   - spaValid / heaveValid are recomputed live from message freshness, so
    //     they flip to false as soon as either estimator stops publishing.
    //   - dt is the measured control-loop period, used by the calm-sea timer.
    {
      const rclcpp::Time now = this->now();
      sea_state_.spaValid   = (now - last_spa_time_).seconds()   < kSeaStale;
      sea_state_.heaveValid = (now - last_heave_time_).seconds() < kSeaStale;
      sea_state_.padImuValid     = (now - last_pad_imu_time_).seconds()     < kSeaStale;
      sea_state_.droneAccelValid = (now - last_drone_accel_time_).seconds() < kSeaStale;
      sea_state_.dt = (last_loop_time_.nanoseconds() == 0)
                        ? 0.0 : (now - last_loop_time_).seconds();
      last_loop_time_ = now;
      autom_.setSeaState(sea_state_);

      // A2 verification: confirm predicted heave arrives and is fresh.
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "[sea] heaveValid=%d pos=%.3f vel=%.3f  pred(pos=%.3f vel=%.3f)",
        sea_state_.heaveValid, sea_state_.heavePos, sea_state_.heaveVel,
        sea_state_.heavePosPred, sea_state_.heaveVelPred);
    }

    // std::cout << "executing commands" << std::endl;
    // Tick the active command's state machine.
    autom_.executeCommand();

    // disarmOnGo: send a forced component-disarm to PX4 whenever the guidance
    // asks. The Lander re-raises the request a few times over a bounded window
    // after entering LANDING_DONE, since this topic is published best-effort
    // and a single dropped command would leave the drone armed and hovering.
    if (autom_.consumeDisarmRequest()) {
      px4_msgs::msg::VehicleCommand cmd{};
      cmd.timestamp = this->get_clock()->now().nanoseconds() / 1000;
      cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
      cmd.param1 = 0.0;  // 0 = disarm
      // param2 = 21196 is MAVLink's "force" magic value. Without it PX4's
      // commander refuses to disarm an airborne vehicle and defers the command
      // until its own land detector reports landed -- which is why the disarm
      // appeared late: it only took effect after the drone had already been put
      // down by the LANDING_DONE descent. Disarm mode exists precisely to cut
      // the motors while still just above the pad, so the force flag is
      // required for it to mean anything.
      cmd.param2 = 21196.0;
      cmd.target_system = 1;
      cmd.target_component = 1;
      cmd.source_system = 1;
      cmd.source_component = 1;
      cmd.from_external = true;
      vehicle_command_pub_->publish(cmd);
      RCLCPP_WARN(get_logger(), "[disarmOnGo] sending FORCED disarm command");
    }

    // Publish the resulting setpoint.
    auto command = odomFromMavState(autom_._comm);
    command.header.stamp = this->now();
    local_position_sp_pub_->publish(command);

    // Echo platform pose for any downstream consumer of platRob.
    auto plat_rob = odomFromMavState(platform_state_);
    plat_rob.header.stamp = this->now();
    plat_rob_pub_->publish(plat_rob);

    // Publish the Go/NoGo prediction horizon T = l/v so the SPA predictor can
    // adjust its prediction window to how far we are from the platform.
    // Negative sentinel means no Land command is active — skip publishing then.
    double horizon = autom_.getPredictionHorizon();
    if (horizon >= 0.0) {
      std_msgs::msg::Float64 horizon_msg;
      horizon_msg.data = horizon;
      pred_horizon_pub_->publish(horizon_msg);
    }

    // Publish every variable behind the landing GO/NO-GO decision, so it can be
    // logged/plotted (e.g. in PlotJuggler, which parses DiagnosticArray natively).
    // Only meaningful while a Land command is active.
    if (horizon >= 0.0) {
      LandGoDiagnostics d = autom_.getGoDiagnostics();

      diagnostic_msgs::msg::DiagnosticStatus status;
      status.name    = "landing_go_nogo";
      status.message = d.landingGo ? "GO" : "NO-GO";
      status.level   = d.landingGo ? diagnostic_msgs::msg::DiagnosticStatus::OK
                                    : diagnostic_msgs::msg::DiagnosticStatus::WARN;

      auto kv = [](const std::string & key, double value) {
        diagnostic_msgs::msg::KeyValue e;
        e.key   = key;
        e.value = std::to_string(value);
        return e;
      };
      status.values = {
        kv("spa_valid",         d.spaValid),
        kv("heave_valid",       d.heaveValid),
        kv("spa_roll_pred",     d.spaRollPred),
        kv("spa_pitch_pred",    d.spaPitchPred),
        kv("roll_rate",         d.rollRate),
        kv("pitch_rate",        d.pitchRate),
        kv("heave_pos",         d.heavePos),
        kv("heave_vel",         d.heaveVel),
        kv("heave_vel_pred",    d.heaveVelPred),
        kv("roll_limit",        d.goRollLimit),
        kv("pitch_limit",       d.goPitchLimit),
        kv("roll_rate_limit",   d.goRollRateLimit),
        kv("pitch_rate_limit",  d.goPitchRateLimit),
        kv("angles_ok",         d.anglesOk),
        kv("rates_ok",          d.ratesOk),
        kv("desc_from_peak",    d.descFromPeak),
        kv("raw_go",            d.rawGo),
        kv("vote_fraction",     d.voteFraction),
        kv("vote_window_size",  d.voteWindowSize),
        kv("voted_go",          d.votedGo),
        kv("latch_timer_s",     d.latchTimer),
        kv("landing_go",        d.landingGo),
        kv("touch_accel_z",         d.touchAccelZ),
        kv("touch_pad_accel_z",     d.touchPadAccelZ),
        kv("touch_ground_contact",  d.touchGroundContact),
        kv("touch_confirmed",       d.touchConfirmed),
        kv("touched_down",          d.touchedDown),
        kv("touch_contact_timer_s",     d.touchContactTimer),
        kv("touch_confirm_timer_s",     d.touchConfirmTimer),
        kv("touch_time_since_spike_s",  d.touchTimeSinceSpike),
      };

      diagnostic_msgs::msg::DiagnosticArray diag_msg;
      diag_msg.header.stamp = this->now();
      diag_msg.status.push_back(status);
      landing_diag_pub_->publish(diag_msg);
    }
  }

  // ══════════════════════════════════════════════════════════════════════════
  // Sensor callbacks — update state only; no command execution here
  // ══════════════════════════════════════════════════════════════════════════

  void onVisionPos(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
  {
    mavStateFromOdom(*msg, drone_pose_);
    // Push fresh drone pose into Automatic every time it arrives, just as the
    // original did with autom_.setState() at the top of its loop.
    autom_.setState(drone_pose_);
  }

  void onPlatformPose(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    mavStateFromNavOdom(*msg, platform_state_);
    autom_.setPlatformState(platform_state_);
  }

  void onActualTask(const ulisse_msgs::msg::ActualTask::SharedPtr msg)
  {
    autom_.setTask(*msg);
    current_action_ = msg->action;
    current_target_ = msg->target;

    RCLCPP_INFO(get_logger(), "New task arrived with action: %s",
                printAction(msg->action).c_str());

    waiting_ = false;
    // handleCommands() parses the incoming command and initialises the FSM
    // for the chosen action (e.g. sets up the Landing state machine).
    // executeCommand() will then be called repeatedly by the control timer.
    autom_.handleCommands();
  }

  void onPlatformCenter(const geometry_msgs::msg::TransformStamped::SharedPtr msg)
  {
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
    // std::cout << "onPlatformCenter VisionDataUpdated set to true " << std::endl; 
    vision_system_.VisionDataUpdated = true;
    vision_system_.IsValid = true;
    autom_.setVisionFeedback(vision_system_);
    last_vision_time_ = this->now();
  }

  void onRescue(const geometry_msgs::msg::TransformStamped::SharedPtr msg)
  {
    // Same filtering caveat as onPlatformCenter.
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
    vision_rescue_.IsValid = true;
    autom_.setVisionRescue(vision_rescue_);
    last_rescue_time_ = this->now();

    if (current_action_ == actions::SEARCH) {
      publishRescueCoords();
    }
  }

  void onUltrasonic(const ultrasonic_sensor::msg::UltrasonicMsg::SharedPtr msg)
  {
    ultrasonic_pos_.setZ(msg->z_position);
    ultrasonic_pos_.setVz(msg->z_velocity);
    ultrasonic_pos_.IsValid = msg->is_valid;
    // std::cout << "valid ultrasonic " << ultrasonic_pos_.IsValid << std::endl;
    ultrasonic_pos_.UltrasonicDataUpdated = true;
    autom_.setUltrasonicInfo(ultrasonic_pos_);
    last_ultrasonic_time_ = this->now();
  }

  void onSafetyAck(const ulisse_msgs::msg::SafetyAck::SharedPtr msg)
  {
    last_safety_ack_ = *msg;
  }

  void onGps(const px4_msgs::msg::SensorGps::SharedPtr msg)
  {
    // PX4 GPS fields are scaled integers: 1e-7 degrees, 1e-3 meters
    gps_lon_ = 1.0e-7 * msg->longitude_deg;
    gps_lat_ = 1.0e-7 * msg->latitude_deg;
    gps_alt_ = 1.0e-3 * msg->altitude_msl_m;
  }

  void onReleasePayloadFeedback(const std_msgs::msg::String::SharedPtr msg)
  {
    autom_.setRelPayFeedback(msg->data);
  }

  // ── Sea-state callbacks (SPA predictor + heave estimator) ───────────────────

  // spa/estimate carries the current roll/pitch/yaw estimate. We finite-difference
  // roll and pitch here to obtain the angular rates used by the calm-sea gate.
  void onSpaEstimate(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
  {
    const double roll  = msg->vector.x;
    const double pitch = msg->vector.y;
    const rclcpp::Time stamp(msg->header.stamp);

    if (spa_est_init_) {
      const double dt = (stamp - last_spa_est_stamp_).seconds();
      if (dt > 1e-3) {
        // Raw finite-difference rates are noisy — smooth each with a moving average.
        sea_state_.rollRate  = movingAverage(roll_rate_buf_,  (roll  - prev_spa_roll_)  / dt);
        sea_state_.pitchRate = movingAverage(pitch_rate_buf_, (pitch - prev_spa_pitch_) / dt);
      }
    }
    spa_est_init_       = true;
    prev_spa_roll_      = roll;
    prev_spa_pitch_     = pitch;
    last_spa_est_stamp_ = stamp;

    sea_state_.roll  = roll;
    sea_state_.pitch = pitch;
  }

  // Push a sample into a fixed-length window and return its mean.
  double movingAverage(std::deque<double> & buf, double sample)
  {
    buf.push_back(sample);
    if (buf.size() > kRateWindow) buf.pop_front();
    double sum = 0.0;
    for (double v : buf) sum += v;
    return sum / static_cast<double>(buf.size());
  }

  // spa/prediction marks the SPA pipeline as active; used for the freshness check.
  void onSpaPrediction(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
  {
    sea_state_.spaRollPred  = msg->vector.x;   // predicted roll  [rad]
    sea_state_.spaPitchPred = msg->vector.y;   // predicted pitch [rad]
    last_spa_time_ = this->now();
  }

  void onHeavePosition(const std_msgs::msg::Float64::SharedPtr msg)
  {
    sea_state_.heavePos = msg->data;
    last_heave_time_    = this->now();
  }

  void onHeaveVelocity(const std_msgs::msg::Float64::SharedPtr msg)
  {
    sea_state_.heaveVel = msg->data;
    last_heave_time_    = this->now();
  }

  // Forward-predicted heave (published in lockstep with heave/velocity, so its
  // freshness is already covered by heaveValid).
  void onHeavePredPosition(const std_msgs::msg::Float64::SharedPtr msg)
  {
    sea_state_.heavePosPred = msg->data;
  }

  void onHeavePredVelocity(const std_msgs::msg::Float64::SharedPtr msg)
  {
    sea_state_.heaveVelPred = msg->data;
  }

  // Comp-phase heave prediction (control+comm delay horizon) — same freshness
  // reasoning as above (published in lockstep with heave/velocity).
  void onHeavePredPositionComp(const std_msgs::msg::Float64::SharedPtr msg)
  {
    sea_state_.heavePosPredComp = msg->data;
  }

  void onHeavePredVelocityComp(const std_msgs::msg::Float64::SharedPtr msg)
  {
    sea_state_.heaveVelPredComp = msg->data;
  }

  // Pad's raw IMU — same topic/field heave_estimator already consumes. Raw,
  // whatever gravity/sign convention the source uses (see SeaState.h) — the
  // touchdown detector removes the known constant before comparing.
  void onPadImu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    sea_state_.padAccelZ = msg->linear_acceleration.z;
    last_pad_imu_time_   = this->now();
  }

  // Drone's PX4 acceleration estimate. xyz is FRD (Z down), "bias corrected
  // including gravity" per px4_msgs — flip to up-positive so it at least has
  // the same sign sense as a conventional accelerometer; the touchdown detector
  // removes the +g reaction before comparing to the pad.
  void onDroneAccel(const px4_msgs::msg::SensorCombined::SharedPtr msg)
  {
    sea_state_.droneAccelZ = -msg->accelerometer_m_s2[2];
    last_drone_accel_time_ = this->now();
  }

  // ══════════════════════════════════════════════════════════════════════════
  // Staleness checks — runs at 1 Hz via staleness_timer_
  // ══════════════════════════════════════════════════════════════════════════
  void checkSensorFreshness()
  {
    const rclcpp::Time now      = this->now();
    constexpr double   kStale   = 1.0;   // seconds — matches original Duration(1)

    if ((now - last_vision_time_).seconds() > kStale) {
      // std::cout << "VisionDataUpdated timed out it was " << vision_system_.VisionDataUpdated << std::endl;
      vision_system_.VisionDataUpdated = false;
      autom_.setVisionFeedback(vision_system_);
    }

    if ((now - last_rescue_time_).seconds() > kStale) {
      vision_rescue_.VisionDataUpdated = false;
      autom_.setVisionRescue(vision_rescue_);
    }

    if ((now - last_ultrasonic_time_).seconds() > kStale) {
      ultrasonic_pos_.UltrasonicDataUpdated = false;
      autom_.setUltrasonicInfo(ultrasonic_pos_);
    }
  }

  // ══════════════════════════════════════════════════════════════════════════
  // SEARCH-state rescue coordinate publication
  // ══════════════════════════════════════════════════════════════════════════
  void publishRescueCoords()
  {
    nav_msgs::msg::Odometry rescue;
    rescue.header.stamp    = this->now();
    rescue.header.frame_id = "rescue_target";

    if (autom_.env_sim) {
      rescue.pose.pose.position.x = autom_.getState().getX();
      rescue.pose.pose.position.y = autom_.getState().getY();
      rescue.pose.pose.position.z = autom_.getState().getZ();

      double platform_pos[3] = {
        platform_state_.getX(), platform_state_.getY(), platform_state_.getZ()};
      double yaw_temp = computeDesiredYaw(autom_.getState(), platform_pos) + 1.57; // +π/2: 90° offset to align camera forward direction with target

      Eigen::Quaterniond orient = getQuatFromYaw(yaw_temp);
      rescue.pose.pose.orientation.w = orient.w();
      rescue.pose.pose.orientation.x = orient.x();
      rescue.pose.pose.orientation.y = orient.y();
      rescue.pose.pose.orientation.z = orient.z();
    } else {
      rescue.pose.pose.position.x = gps_lon_;
      rescue.pose.pose.position.y = gps_lat_;
      rescue.pose.pose.position.z = gps_alt_;
    }

    landing_site_pose_cmd_pub_->publish(rescue);
  }

  // ══════════════════════════════════════════════════════════════════════════
  // State
  // ══════════════════════════════════════════════════════════════════════════

  Automatic   autom_;

  MavState    drone_pose_;
  MavState    platform_state_;
  MavState    vision_system_;
  MavState    vision_rescue_;
  MavState    ultrasonic_pos_;
  SeaState    sea_state_;

  // Sea-state freshness window [s]: SPA/heave data older than this is "outdated".
  static constexpr double kSeaStale = 0.5;
  // Moving-average window for the (noisy) finite-differenced roll/pitch rates.
  static constexpr std::size_t kRateWindow = 10;
  bool             spa_est_init_   {false};
  double           prev_spa_roll_  {0.0};
  double           prev_spa_pitch_ {0.0};
  rclcpp::Time     last_spa_est_stamp_;
  rclcpp::Time     last_loop_time_;
  std::deque<double> roll_rate_buf_;
  std::deque<double> pitch_rate_buf_;

  bool        waiting_        {true};
  int         current_action_ {actions::IDLE};
  std::string current_target_ {""};

  double      gps_lon_ {0.0};
  double      gps_lat_ {0.0};
  double      gps_alt_ {0.0};

  ulisse_msgs::msg::SafetyAck last_safety_ack_ {};

  rclcpp::Time last_vision_time_;
  rclcpp::Time last_rescue_time_;
  rclcpp::Time last_ultrasonic_time_;
  rclcpp::Time last_spa_time_;
  rclcpp::Time last_heave_time_;
  rclcpp::Time last_pad_imu_time_;
  rclcpp::Time last_drone_accel_time_;

  // Timers
  rclcpp::TimerBase::SharedPtr control_timer_;    ///< 50 Hz — main control tick
  rclcpp::TimerBase::SharedPtr staleness_timer_;  ///<  1 Hz — sensor freshness

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr local_position_sp_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr landing_site_pose_cmd_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr plat_rob_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr  pred_horizon_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr landing_diag_pub_;

  // Subscriptions
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr       vision_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr              platform_pose_sub_;
  rclcpp::Subscription<ulisse_msgs::msg::ActualTask>::SharedPtr         actual_task_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr platform_center_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr rescue_sub_;
  rclcpp::Subscription<ultrasonic_sensor::msg::UltrasonicMsg>::SharedPtr ultrasonic_sub_;
  rclcpp::Subscription<ulisse_msgs::msg::SafetyAck>::SharedPtr          safety_ack_sub_;
  rclcpp::Subscription<px4_msgs::msg::SensorGps>::SharedPtr             gps_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr                rel_pay_fb_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr   spa_estimate_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr   spa_prediction_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr               heave_pos_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr               heave_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr               heave_pred_pos_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr               heave_pred_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr               heave_pred_pos_comp_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr               heave_pred_vel_comp_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr                pad_imu_sub_;
  rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr   drone_accel_sub_;
};

// ─────────────────────────── main ───────────────────────────────────────────

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