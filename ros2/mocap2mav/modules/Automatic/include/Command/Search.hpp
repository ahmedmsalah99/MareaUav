
#ifndef MOCAP2MAV_SEARCH_HPP
#define MOCAP2MAV_SEARCH_HPP

#include "Command.hpp"
#include "Searcher/Searcher.h"
#include "lcm/lcm-cpp.hpp"
#include "lcm_messages/exec/release_payload.hpp"


/** @ingroup Automatic
 * @brief Derived class of the Command class
 * @details Such class implements the Move Action
 * Basically, it allows the drone to reach a desired position
 */

class Search : public Command{

private:

    // vision data coming from camera

    MavState*_visionRescuePose;

    // Searcher object

    Searcher _searcher;

    // lcm handler, to send payload message

    lcm::LCM _handler;

    // release payload message to fill

    exec::release_payload _releasePayload;

    // feedback from the release payload module

    exec::release_payload* _relpay_feedback;

    // ignited function

    void search()
    {
        _comm->setType(MavState::type::POSITION);

        // set Uav state
        _searcher.setState(*_state);

        //set vision info
        _searcher.setVisionPose(*_visionRescuePose);

        _searcher.initInspectionParams(_actualTask->params[0], _actualTask->params[1], _actualTask->z);

        _searcher.setYaw(_actualTask->yaw);

        _searcher.setRelPayFeedback(*_relpay_feedback);

        // finally run
        _searcher.run();

        // getting then the computed command
        *_comm = _searcher.getCommand();
        // and the feedback to know if release the payload or not
        if(_searcher.getPayloadCommand())
        {
          // filling the message
          _releasePayload.data = "T";
          // and then sending it
          _handler.publish("release_payload", &_releasePayload);
        }
    }

public:
    Search(MavState *_state, MavState *_comm,exec::task *_actualTask, MavState *_visionRescuePose, exec::release_payload *_relpay_feedback) : Command(_state, _comm, _actualTask), _visionRescuePose(_visionRescuePose), _relpay_feedback(_relpay_feedback)
    {
      this->_relpay_feedback->data = _relpay_feedback->data;
    }

    /**
     * @brief Method inherited from the base class
     */

    void execute() override {
        search();
    }

};
#endif //MOCAP2MAV_SEARCH_HPP
