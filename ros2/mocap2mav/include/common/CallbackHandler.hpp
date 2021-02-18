
#ifndef MOCAP2MAV_CALLBACKHANDLER_HPP
#define MOCAP2MAV_CALLBACKHANDLER_HPP

#include "MavState.h"
#include "lcm/lcm-cpp.hpp"
#include "lcm_messages/geometry/pose.hpp"
#include "lcm_messages/geometry/vision.hpp"
#include "lcm_messages/geometry/UltrasonicPosition.hpp" //for the ultrasonic message
#include "lcm_messages/exec/task.hpp"
#include "lcm_messages/exec/state.hpp"
#include "lcm_messages/exec/battery.hpp"
#include "lcm_messages/exec/gps_status.hpp"
#include "lcm_messages/exec/safety.hpp"
#include "lcm_messages/exec/safety_ack.hpp"
#include "lcm_messages/exec/camera_info.hpp"
#include "lcm_messages/exec/release_payload.hpp"

#include <iostream>

/** @ingroup Automatic
 *
 * @brief Class used to set the actual state of the drone from different sources: GPS, camera, ultrasonic sensor..
 * @details Such class uses callbacks of lcm topics.
 *
 */
class CallbackHandler {

public:

    /**
     *  State of the drone obtained by motion Capture
     */
    MavState _vision_pos;

    /**
     *  State of the platform obtained by motion Capture
     */

    MavState _position_sp;

    exec::task _task;
    exec::safety safety;
    exec::safety_ack safety_ack;
    exec::battery battery;
    exec::gps_status gps;
    exec::release_payload release_payload;
    exec::release_payload relpay_feedback;
    /**
     *  State of the drone respect to the platform obtained by vision system
     */

	  MavState _relative_pos;

    /**
     *  State of the drone respect to the rescue target obtained by vision system
     */

	  MavState _rescue_pos;

    /**
     *  State of the drone obtained by ultrasonic sensor
     */

	  MavState _Ultrasonic_pos;

    /**
     *  Bool variable to check if the drone is landed
     */

    bool _landed;

    /**
     *  Bool variable to check if the drone is armed
     */

    bool _armed;

    /**
     *  Set to true after that the callback of the motion capture or GPS is called (absolute frame)
     */

    bool _estimate_ready;

    /**
     *  Bool variable to check if there is a position setPoint
     */

    bool _position_sp_ready;

    /**
     *  Camera timestamp for lcm message
     */

    double camera_timestamp;

    /**
     * @brief Constructor of the class
     * @details It sets initial values of the class
     */
    CallbackHandler(){

        _position_sp.setPosition(0,0,0);
        _position_sp_ready = false;
        _position_sp.setOrientation(1,0,0,0);
        _position_sp.setYaw(0);
        _position_sp.setType(MavState::type::POSITION);

        _vision_pos.setPosition(0,0,0);
        _estimate_ready = false;
        _vision_pos.setOrientation(1,0,0,0);
        _vision_pos.setYaw(0);
        _vision_pos.setType(MavState::type::POSITION);


        safety_ack.isbatterysafetytaskrunning = 0;
        safety_ack.isgpssafetytaskrunning = 0;
        safety_ack.iscamerasafetytaskrunning = 0;
        safety_ack.issonarsafetytaskrunning = 0;
        safety_ack.isgoinginsafeharbor = 0;
        safety_ack.isgoingtoplatform = 0;
        safety_ack.iscameragone_timestamp = 0.0;

        safety.isbatterygone = 0;
        safety.isgpsgone = 0;
        safety.iscameragone = 0;
        safety.issonargone = 0;
        safety.goingsafeharbor = 0;
        safety.isgoingplatform = 0;

        _armed = false;
        _landed = true;

    }

    /**
     * @brief Callback used to take the messages of the aboslute state of the drone (motion capture or GPS)
     * @details It sets the position, velocity, orientation and the type of command
     */

    void visionEstimateCallback(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const geometry::pose* msg){

        _vision_pos.setPosition((double)msg->position[0],(double)msg->position[1],(double)msg->position[2]);

        _vision_pos.setV((double)msg->velocity[0], (double)msg->velocity[1], (double)msg->velocity[2]);

        _vision_pos.setOrientation((double)msg->orientation[0],(double)msg->orientation[1],(double)msg->orientation[2],(double)msg->orientation[3]);

        _vision_pos.setYaw((float)msg->yaw);

		    _vision_pos.IsValid = (bool)msg->isValid;

        if (msg->type == 0){
            _vision_pos.setType(MavState::type::POSITION);
        }
        else if(msg->type == 1){
            _vision_pos.setType(MavState::type::VELOCITY);
        }
        else{
            _vision_pos.setType(MavState::type::POSITION);
        }

        _estimate_ready = true;

    }

    /**
     * @brief Callback used to take the messages of the absolute state of the platform
     * @details It sets the position, velocity, orientation and the type of command
     */

    void positionSetpointCallback(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const geometry::pose* msg){

        _position_sp.setPosition((float)msg->position[0],(float)msg->position[1],(float)msg->position[2]);

        _position_sp.setV((float)msg->velocity[0],(float)msg->velocity[1],(float)msg->velocity[2]);

        _position_sp.setOrientation((float)msg->orientation[0],(float)msg->orientation[1],(float)msg->orientation[2],(float)msg->orientation[3]);

        _position_sp.setYaw((float)msg->yaw);

        if (msg->type == 0){
            _position_sp.setType(MavState::type::POSITION);
        }
        else if(msg->type == 1){
            _position_sp.setType(MavState::type::VELOCITY);
        }
        else{
            _position_sp.setType(MavState::type::POSITION);
        }

    }

    // callback function for the battery status
    void batteryStatusCallback(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const exec::battery* msg)
    {
      battery.voltage = msg->voltage;
      battery.current = msg->current;
      battery.remaining = msg->remaining;
    }

    // the one for gps
    void gpsStatusCallback(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const exec::gps_status* msg)
    {
      gps.lon = msg->lon;
      gps.lat = msg->lat;
      gps.alt = msg->alt;
      gps.status = msg->status;
      gps.timestamp = msg->timestamp;
    }

    // and for camera
    void cameraInfoCallback(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const exec::camera_info* msg)
    {
      camera_timestamp = msg->timestamp;
    }

    // this is the callback for the safety message instead
    void safetyCallback(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const exec::safety* msg)
    {
      safety.isbatterygone = msg->isbatterygone;
      safety.isgpsgone = msg->isgpsgone;
      safety.issonargone = msg->issonargone;
      safety.iscameragone = msg->iscameragone;
      safety.goingsafeharbor = msg->goingsafeharbor;
      safety.isgoingplatform = msg->isgoingplatform;
      safety.target_coord[0] = msg->target_coord[0];
      safety.target_coord[1] = msg->target_coord[1];
      safety.target_coord[2] = msg->target_coord[2];
      safety.params[0] = msg->params[0];
      safety.params[1] = msg->params[1];
      safety.params[2] = msg->params[2];
      safety.params[3] = msg->params[3];
      safety.params[4] = msg->params[4];
      safety.params[5] = msg->params[5];
      safety.params[6] = msg->params[6];
      safety.params[7] = msg->params[7];
    }

    // and this is the one for its acknowledgments
    void safetyAckCallback(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const exec::safety_ack* msg)
    {
      safety_ack.isbatterysafetytaskrunning = msg->isbatterysafetytaskrunning;
      safety_ack.isgpssafetytaskrunning = msg->isgpssafetytaskrunning;
      safety_ack.iscamerasafetytaskrunning = msg->iscamerasafetytaskrunning;
      safety_ack.issonarsafetytaskrunning = msg->issonarsafetytaskrunning;
      safety_ack.isgoinginsafeharbor = msg->isgoinginsafeharbor;
      safety_ack.isgoingtoplatform = msg->isgoingtoplatform;
      safety_ack.iscameragone_timestamp = msg->iscameragone_timestamp;
    }

    /**
     * @brief Callback used to take the messages coming from release payload topic
     * @details It sets a boolean.
     */

    void releasePayloadCallback(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const exec::release_payload* msg)
    {
      release_payload.data = msg->data;
    }

    /**
     * @brief Callback used to take the messages coming from release payload topic
     * @details It sets a boolean.
     */

    void releasePayloadFeedbackCallback(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const exec::release_payload* msg)
    {
      relpay_feedback.data = msg->data;
    }

    /**
     * @brief Callback used to take the messages coming from the vision system
     * @details It sets the position, velocity, orientation
     */

	void ApriltagCallback(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const geometry::vision* msg)
  {
	   //callback of the vision system.

		_relative_pos.setPosition((float)msg->position[0],(float)msg->position[1],(float)msg->position[2]);

  	_relative_pos.setV((float)msg->velocity[0],(float)msg->velocity[1],(float)msg->velocity[2]);

  	_relative_pos.setOrientation((float)msg->orientation[0],(float)msg->orientation[1],(float)msg->orientation[2],(float)msg->orientation[3]);

		_relative_pos.setRoll( (float)msg->roll );

		_relative_pos.setPitch( (float)msg->pitch );

		_relative_pos.setYaw( (float)msg->yaw );

    _relative_pos.IsValid = (bool)msg->isValid;

	}

  /**
   * @brief Callback used to take the messages coming from the vision system
   * @details It sets the position, velocity, orientation
   */

  void RescueCallback(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const geometry::vision* msg)
  {
     //callback of the vision system.

    _rescue_pos.setPosition((float)msg->position[0],(float)msg->position[1],(float)msg->position[2]);

    _rescue_pos.setV((float)msg->velocity[0],(float)msg->velocity[1],(float)msg->velocity[2]);

    _rescue_pos.setOrientation((float)msg->orientation[0],(float)msg->orientation[1],(float)msg->orientation[2],(float)msg->orientation[3]);

    _rescue_pos.setRoll( (float)msg->roll );

    _rescue_pos.setPitch( (float)msg->pitch );

    _rescue_pos.setYaw( (float)msg->yaw );

    _rescue_pos.IsValid = (bool)msg->isValid;

  }
    /**
     * @brief Callback used to take the messages coming from the ultrasonic sensor
     * @details It sets the Z position, the estimated Z velocity and set the validity of the message
     */

	void UltrasonicCallback(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const geometry::UltrasonicPosition* msg){
	//callback of the vision system.

		_Ultrasonic_pos.setZ((float)msg->Z_Position);

		_Ultrasonic_pos.setVz((float)msg->Z_Velocity);

		_Ultrasonic_pos.IsValid = (bool)msg->isValid;

	}

    /**
     * @brief Callback used to take the actual task coming from the executioner module
     * @details It sets the type of action, its param and the core of the message
     */

    void actualTaskCallback(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const exec::task* msg){

        _task.action = msg->action;
        _task.id = msg->id;

        _task.params[0] = msg->params[0];
        _task.params[1] = msg->params[1];
        _task.params[2] = msg->params[2];
        _task.params[3] = msg->params[3];
        _task.params[4] = msg->params[4];

        _task.x = msg->x;
        _task.y = msg->y;
        _task.z = msg->z;

        _task.yaw = msg->yaw;

        _task.target = msg->target;


    }

    /**
     * @brief Callback used to set bool varibles
     * @details It sets if the drone is landed and if the drone is armed
     */

    void stateCallback(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const exec::state* msg){

        if (msg->landed == (uint8_t)1) _landed = true;
        else _landed = false;

        if(msg->armed == 1)   _armed = true;
        else _armed = false;

    }

    /**
     * @brief Function to convert lcm message to MavState class
     * @param lcmPose Object of pose class
     * @return Filled MavState object
     */

    MavState lcmPose2MavState(const geometry::pose lcmPose){

        MavState temp;

        temp.setPosition((float)lcmPose.position[0],(float)lcmPose.position[1],(float)lcmPose.position[2]);
        temp.setV((float)lcmPose.velocity[0],(float)lcmPose.velocity[1],(float)lcmPose.velocity[2]);
        temp.setOrientation((float)lcmPose.orientation[0],(float)lcmPose.orientation[1],(float)lcmPose.orientation[2],(float)lcmPose.orientation[3]);
        temp.setYaw(lcmPose.yaw);

        temp.timestamp = lcmPose.timestamp;

        if (lcmPose.type == 0){
            temp.setType(MavState::type::POSITION);
        }
        else if(lcmPose.type == 1){
            temp.setType(MavState::type::VELOCITY);
        }
        else{
            temp.setType(MavState::type::POSITION);
        }

        return temp;

    }

    /**
     * @brief Function to convert MavState class to lcm message
     * @param mavPose Object of MavState class
     * @return Filled lcm message
     */

    geometry::pose mavState2LcmPose(MavState mavPose){

        geometry::pose temp;

        temp.position[0] = mavPose.getX();
        temp.position[1] = mavPose.getY();
        temp.position[2] = mavPose.getZ();

        temp.velocity[0] = mavPose.getVx();
        temp.velocity[1] = mavPose.getVy();
        temp.velocity[2] = mavPose.getVz();

        temp.yaw = mavPose.getYaw();

        temp.orientation[0] = mavPose.getOrientation().w();
        temp.orientation[1] = mavPose.getOrientation().x();
        temp.orientation[2] = mavPose.getOrientation().y();
        temp.orientation[3] = mavPose.getOrientation().z();

        temp.timestamp = mavPose.timestamp;

        temp.type = mavPose.getType();

        return temp;

    }

};

#endif //MOCAP2MAV_CALLBACKHANDLER_HPP
