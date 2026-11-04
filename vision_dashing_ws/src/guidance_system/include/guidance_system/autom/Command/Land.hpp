//
// Created by andreanistico on 13/06/17.
//

#ifndef MOCAP2MAV_LAND_HPP
#define MOCAP2MAV_LAND_HPP

#include "Command.hpp"
#include "common/conversions.h"
#include "guidance_system/autom/Lander/Lander.h"
#include "guidance_system/autom/SimpleLanding.hpp"

/** @ingroup Automatic
 * @brief Derived class of the Command class.
 * @details Such class implements the Land Action.
 * Such function implements two types of Landing:
 * Basic landing procedure (the drone lands from the actual position)
 *
 * Landing procedure on a moving platform (using vision system and ultrasonic sensor ).
 */

class Land : public Command{

private:

    /**
     * Actual X position of the drone
     */

    double _xin;

    /**
     * Actual Y position of the drone
     */

    double _yin;

    /**
     * Actual yaw angle of the drone
     */

    double _yawin;

    /**
     *  To diversificate from blind or improved landing
     */

    int    _plat;

    /**
     *  Pose of the platform respect to the absolute reference frame
     */

    MavState* _platformPose;

    /**
     *  Relative pose of the platform respect to the drone
     */

    MavState* _VisionPose;

    /**
     *  Partial state of the drone using the ultrasonic sensor.
     */

    MavState* _UltrasonicInfo;

    /**
     *  Aggregated sea-state estimate (SPA prediction + heave)
     */

    SeaState* _seaState;

    /**
     * @brief Lander object
     */

    Lander _lander;

    bool first = true;

    /**
     * @brief Basic landing: lands at (x_target, y_target) on the ground height h,
     * without the pad or vision. The algorithm is simple_landing::step()
     * (SimpleLanding.hpp), shared with the Lander's abort.
     */

    void simpleLanding(float x_target, float y_target, float h) {
        simple_landing::step(*_state, x_target, y_target, h, _lander.getParams(), *_comm);
    }

    /**
     * @brief Such method is called by the execute method
     * @details Based on the value of _plat, it can distinguish
     * the two landing procedures
     */

    void land(){

        //Save initial state if we have a new task
        if (_newTask) {
            _xin   = _state->getX();
            _yin   = _state->getY();
            _yawin = _state->getYawFromQuat();
            _plat = (int)_actualTask->params[0];
            _newTask = false;
        }

        if (_plat == 0){
            std::cout << "simpleLanding " << std::endl;
            simpleLanding((float)_xin,(float)_yin,0);
        } else {

            // this is used to reset the lander if needed
            if(_actualTask->target == "reset_land" && first)
            {
              first = false;
              _lander.startFromBeginning = true;
            }

            //set the state of the UAV

            _lander.setState(*_state);

            //set the platform pose

            _lander.setPlatformState(*_platformPose);

            //passing the information coming from the vision system to the LanderStateMachine.

    	    	_lander.setVisionPose(*_VisionPose);

            //passing the information coming from the ultrasonic sensor to the LanderStateMachine.

      			_lander.setUltrasonicInfo(*_UltrasonicInfo);

            //passing the aggregated sea-state estimate (SPA prediction + heave).

            _lander.setSeaState(*_seaState);

            // passing the desired yaw before landing
            _lander.setYaw(_actualTask->yaw);
      			//here, we can divide the function run() to obtain the actual state of the machine

            _lander.run();

            *_comm = _lander.getCommand();
        }

    }

public:
    /** Where the drone took off: a landing abort returns and lands here. */
    void setTakeoffPosition(double x, double y, double z) { _lander.setTakeoffPosition(x, y, z); }

    Land(MavState *_state, MavState *_comm,exec::task *_actualTask, MavState* _platform, MavState* _Vision, MavState* _Ultrasonic, SeaState* _Sea) : Command(_state, _comm, _actualTask) , _plat(_plat), _platformPose(_platform), _VisionPose(_Vision),_UltrasonicInfo(_Ultrasonic),_seaState(_Sea){}

    /**
     * @brief Method inherited from the base class
     */

    void execute() override {
        // std::cout << "executing landing " << std::endl;
        land();
    }

    double getPredictionHorizon() const {
        return _lander.getPredictionHorizon();
    }

    LandGoDiagnostics getGoDiagnostics() const {
        return _lander.getGoDiagnostics();
    }

    bool consumeDisarmRequest() {
        return _lander.consumeDisarmRequest();
    }
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

#endif //MOCAP2MAV_ROTATE_HPP
