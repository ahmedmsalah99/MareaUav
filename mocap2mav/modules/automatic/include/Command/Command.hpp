#ifndef MOCAP2MAV_COMMAND_HPP
#define MOCAP2MAV_COMMAND_HPP

#include "common/MavState.h"
#include "lcm_messages/exec/task.hpp"

/**@ingroup Automatic
 * @brief Base class of the commands to provide to the drone
 */

class Command {

protected:

    /**
     * @brief State of the drone
     */
    
    MavState* _state;

    /**
     * @brief command to provide to the drone
     */

    MavState* _comm;

    /**
     * @brief actual task which has to satisfy
     */

    exec::task* _actualTask;
    bool _newTask;


public:
    Command(){}
    Command(MavState *_state, MavState *_comm,exec::task *_actualTask) : _state(_state), _comm(_comm),
                                                                         _actualTask(_actualTask), _newTask(true) {}

    virtual ~Command() {}

    /**
     * @brief Virtual function
     * @details Adding the specificator virtual, it is possible to guarantee a dynamic association.
     * In this case, being the Command class the base one, it is important that it defines a method for the execution of the command.
     * The real implementation of such method has to be performed by the derived classes (Idle, Move, TakeOff, Land).
     */

    virtual void execute() = 0;
    

    void init(MavState* s, MavState* c){
        this->_state = s;
        this->_comm = c;
    }

};




#endif //MOCAP2MAV_COMMAND_HPP
