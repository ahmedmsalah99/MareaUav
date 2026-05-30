
#ifndef MOCAP2MAV_IDLE_HPP
#define MOCAP2MAV_IDLE_HPP


#include "Command.hpp"
#include "common/conversions.h"

/** @ingroup Automatic
 * @brief Derived class of the Command class.
 * @details Such class implements the Idle Action.
 * Basically, it allows the drone to maintain the actual pose.
 * 
 */

class Idle : public Command{

private:

    /**
     * X desired position
     */

    double _xin;

    /**
     * Y desired position
     */

    double _yin;
    
    /**
     * Z desired position
     */

    double _zin;
    
    /**
     * Yaw desired position
     */

    double _yawin;

    /**
     * @brief Such method is called inside execute function.
     * @details It sends the command with the actual pose of the drone
     */

    void idle() {
        //Save initial state if we have a new task
        if (_newTask) {
            _xin = _state->getX();
            _yin = _state->getY();
            _zin = _state->getZ();
            _yawin = _state->getYawFromQuat();
            _newTask = false;
        }

        _comm->setType(MavState::type::POSITION);
        _comm->setX((float)_xin);
        _comm->setY((float)_yin);
        _comm->setZ((float)_zin);

        Eigen::Quaterniond q = getQuatFromYaw(_yawin);

        _comm->setOrientation((float)q.w(),(float)q.x(),(float)q.y(),(float)q.z());
    }

public:

    Idle(MavState *_state, MavState *_comm,exec::task *_actualTask) :
    Command(_state, _comm, _actualTask){}

    /**
     * @brief Method inherited from the base class
     */

    void execute() override {
        idle();
    }

};


#endif //MOCAP2MAV_IDLE_HPP
