//
// Created by andreanistico on 13/06/17.
//

#ifndef MOCAP2MAV_LAND_HPP
#define MOCAP2MAV_LAND_HPP

#include "Command.hpp"
#include "common/conversions.h"
#include "Lander/Lander.h"

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
     * @brief Lander object
     */

    Lander _lander;

    bool first = true;

    /**
     * @brief Such method calculates the descend rate profile (linear) through y = mx + q
     * @param dz vertical distance between robot and the platform
     * @param drate_max Maximum rate
     * @param drate_min Minimum rate
     * @param tmax Maximum time
     * @param tmin Minimum time
     * @return setpoint position on Z axis
     */

    double calculateDescendRate(double dz,double drate_max,double drate_min, double tmax, double tmin){

        //where x is vertical distance between robot and platform
        if(dz > tmax)      return drate_max;
        else if(dz < tmin) return drate_min;
        else{

            double m = (drate_max - drate_min) / (tmax - tmin);
            double q = drate_min - m * tmin;
            return m * dz + q;

        }
    }

    /**
     * @brief The core method for simple landing action
     * @details Such method allows to land from the current X and Y position.
     *
     * @param x_target Actual X position of the drone
     * @param y_target Actual Y position of the drone
     * @param h Height from which the land should start
     */

    void simpleLanding(float x_target, float y_target, float h) {

        //Calculate difference

        double dx = - _state->getX() + x_target;
        double dy = - _state->getY() + y_target;
        double dz = - _state->getZ() + h;

        // Be sure that we are on top of the target

        double x_target_v = Kland * (dx);
        double y_target_v = Kland * (dy);

        _comm->setVx(x_target_v);
        _comm->setVy(y_target_v);
        _comm->setType(MavState::VELOCITY);
        //TODO: add security checks on vz
        if(fabs(dx) <= THRE && fabs(dy) <= THRE){
            //Descending is safe, is it?
            double desc = calculateDescendRate(fabs(dz), DRATE_MAX, DRATE_MIN, TMAX, TMIN);
            _comm->setVz(-desc);

            if (fabs(dz) < THRE){
                _comm->setX(x_target);
                _comm->setY(y_target);
                _comm->setZ(h-10);
                _comm->setType(MavState::POSITION);
            }

        }
        else _comm->setVz(0);

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

            // passing the desired yaw before landing
            _lander.setYaw(_actualTask->yaw);
      			//here, we can divide the function run() to obtain the actual state of the machine

            _lander.run();

            *_comm = _lander.getCommand();
        }

    }

public:
    Land(MavState *_state, MavState *_comm,exec::task *_actualTask, MavState* _platform, MavState* _Vision, MavState* _Ultrasonic) : Command(_state, _comm, _actualTask) , _plat(_plat), _platformPose(_platform), _VisionPose(_Vision),_UltrasonicInfo(_Ultrasonic){}

    /**
     * @brief Method inherited from the base class
     */

    void execute() override {
        land();
    }
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

#endif //MOCAP2MAV_ROTATE_HPP
