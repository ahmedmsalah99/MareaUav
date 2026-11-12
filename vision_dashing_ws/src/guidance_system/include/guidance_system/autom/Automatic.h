#ifndef AUTOMATIC_H
#define AUTOMATIC_H
#include <iostream>
#include <memory>
#include "lcm_messages/geometry/pose.hpp"
#include "lcm_messages/exec/task.hpp"
#include "lcm_messages/exec/release_payload.hpp"
#include "common/MavState.h"
#include "common/common.h"
#include "common/conversions.h"
#include "guidance_system/autom/Lander/Lander.h"
#include "guidance_system/autom/Command/TakeOff.hpp"
#include "guidance_system/autom/Command/Land.hpp"
#include "guidance_system/autom/Command/Move.hpp"
#include "guidance_system/autom/Command/Rotate.hpp"
#include "guidance_system/autom/Command/Idle.hpp"
#include "guidance_system/autom/Command/EmergLand.hpp"
#include "guidance_system/autom/Command/Search.hpp"

// PX 4 / sensor ROS 2 messages
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>

// Other ROS 2 messages
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <ultrasonic_sensor/msg/ultrasonic_msg.hpp>

// Custom ROS 2 messages (mirrors of the old LCM exec/* structs)
#include <ulisse_msgs/msg/actual_task.hpp>
#include <ulisse_msgs/msg/safety_ack.hpp>

/** @ingroup Automatic
 *
 * @brief Primary class of such module
 * @details Used to implement autonomous task for the drone
 *
 */

class Automatic
{

public:

  /**
   * @brief Constructor of the class
   */
  Automatic();

  /**
   * @brief Such method sets the actual state of the drone
   * @param pose MavState object
   */

  void setState(MavState pose);

  /**
   * @brief Such method sets the actual state of the platform
   * @param pose MavState Object
   */

  void setPlatformState(MavState pose); //set platform state

  /**
   * @brief Such method sets the actual state of the drone respect to the platform using vision system
   * @param pose MavState Object
   */

  void setVisionFeedback(MavState pose);

  /**
   * @brief Such method sets the actual state of the drone respect to the rescue position using vision system
   * @param pose MavState Object
   */

  void setVisionRescue(MavState pose);

  /**
   * @brief Such method sets the actual state of the drone using ultrasonic sensor
   * @param pose MavState Object
   */

  void setUltrasonicInfo(MavState pose);

  /**
   * @brief Such method sets the aggregated sea-state estimate (SPA prediction + heave)
   * @param seaState SeaState Object
   */

  void setSeaState(SeaState seaState);

  /**
   * @brief Such method sets the feedback coming from the release payload node
   * @param string feedback
   */

  void setRelPayFeedback(const std::string& feedback);

  /**
   * @brief Such method sets the actual task.
   * @param task task Object
   */

  void setTask(ulisse_msgs::msg::ActualTask rTask);

  /**
   * @brief Such method returns the actual task.
   * @return task Object
   */

  exec::task getTask();

  /**
   * @brief Such method returns the MavState Object _state
   * @return MavState Object _state
   */

  MavState getState();



  MavState getPlatformState();

  /**
   * @brief Actual command to provide to the drone
   */

  MavState _comm;

  /**
   * @brief Keeps every outgoing setpoint inside the configured box
   * (GEOFENCE_* and ALT_MAX_WORLD_M in landing_<mode>.yaml). Position targets
   * are clamped into the box; velocity components that would push the drone
   * further out while it is at/over a wall are zeroed. No-op when
   * GEOFENCE_ENABLE is 0.
   */
  void applyGeofence();

  /// Take-off point (where the last TAKE_OFF task started), handed to the
  /// Lander so a landing abort can return and land there.
  bool   _takeoffSet = false;
  double _takeoffX = 0.0, _takeoffY = 0.0, _takeoffZ = 0.0;

  /// Landing parameters (only the geofence keys are used here).
  Parameters _fenceParams;

  /**
   * @brief Actual task
   */

  exec::task _actualTask;

  /**
   * @brief The automatic class has a member 'std::unique_ptr<Command> _actualCommand'
   * which is a template of the class Command. The class Command is the base one.
   * The derived classes are: Idle,Move,Takeoff and Land. Based on the actual task arrived,
   * such function sets a particular derived class.
   */

  void handleCommands();

  /**
   * @brief Such function will call the virtual function 'execute()' of the Command class
   */

  void executeCommand();

  /**
   * @brief Returns the active command's Go/NoGo prediction horizon.
   * @return Horizon in seconds, or a negative sentinel when no Land command is active.
   */

  double getPredictionHorizon() const;

  /**
   * @brief Returns the active command's landing GO/NO-GO decision variables.
   * @return LandGoDiagnostics, or a default-constructed (all-false/zero) one when
   * no Land command is active — check landingGo/spaValid/heaveValid before use.
   */

  LandGoDiagnostics getGoDiagnostics() const;

  /**
   * @brief Consumes (returns and clears) the active Land command's pending
   * disarm-on-GO request, set when param.landDisarmOnGo is enabled and the GO
   * signal arrives in COMP. False when no Land command is active or none
   * is pending.
   */

  bool consumeDisarmRequest() const;

  /**
   * @brief Returns and clears the active Search command's pending
   * payload-release request (AutomaticNode publishes it on "release_payload").
   */
  bool consumePayloadRelease() const;

  /**
   * @brief True when Gazebo is running (M2M_MODE 0 = sim or 3 = mocap in sim)
   */

  bool env_sim;

private:

    /**
     *  State of the drone
     */

	MavState _state;

    /**
     *  State of the platform
     */

	MavState _platformState;

    /**
     * Relative state of the platform respect to the drone using vision system
     */

  	MavState _visionFeedbackPose;

    /**
     * Relative state of the rescue target respect to the drone using vision system
     */

  	MavState _visionRescuePose;

    /**
     * partial state of the drone using ultrasonic sensor
     */

	MavState _UltrasonicInfo;

    /**
     * aggregated sea-state estimate (SPA prediction + heave)
     */

	SeaState _seaState;

    /**
     * feedback values from the release payload node
     */

	exec::release_payload _relpay_feedback;

    /**
     * @brief Template of Command class
     * @details The template is used to diversificate from the derived class
     *
     */

	std::unique_ptr<Command> _actualCommand;

    /**
     * yaw used during land and search tasks (must be the same as move one)
     */

	float _defaultYaw;

};

#endif // AUTOMATIC_H
