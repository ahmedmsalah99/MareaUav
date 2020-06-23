#ifndef AUTOMATIC_H
#define AUTOMATIC_H
#include <iostream>
#include <memory>
#include "lcm_messages/geometry/pose.hpp"
#include "lcm_messages/exec/task.hpp"
#include "common/MavState.h"
#include "common/common.h"
#include "common/conversions.h"
#include "include/Lander/Lander.h"
#include "Command/TakeOff.hpp"
#include "Command/Land.hpp"
#include "Command/Move.hpp"
#include "Command/Rotate.hpp"
#include "Command/Idle.hpp"
#include "Command/EmergLand.hpp"

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
   * @brief Such method sets the actual state of the drone using ultrasonic sensor
   * @param pose MavState Object
   */

	void setUltrasonicInfo(MavState pose);

  /**
   * @brief Such method sets the actual task.
   * @param task task Object
   */

  void setTask(exec::task task);

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
     * partial state of the drone using ultrasonic sensor
     */

	MavState _UltrasonicInfo;

    /**
     * @brief Template of Command class
     * @details The template is used to diversificate from the derived class
     *
     */

	std::unique_ptr<Command> _actualCommand;

};

#endif // AUTOMATIC_H
