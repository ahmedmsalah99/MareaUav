#ifndef MOCAP2MAV_EMERGLAND_HPP
#define MOCAP2MAV_EMERGLAND_HPP

#include "Command.hpp"
#include "math.h"
#include "guidance_system/autom/Lander/Parameters.h"

class EmergLand : public Command
{
  private:

    Parameters _param;

    void emergLand()
    {
      // Compute difference between current position and desired one
      double dz = _state->getZ() - _actualTask->z;

      // setting desired velocities on x and y to 0
      _comm->setVx(0);
      _comm->setVy(0);
      _comm->setType(MavState::VELOCITY);
      // then, if the error distance is greater than the "turn off motors" threshold
      if(fabs(dz) >= _actualTask->params[1])
      {
          // computing a descending rate based on the distance itself
          double desc = computeDescendRate(fabs(dz));
          // and setting it as desired velocity on z
          _comm->setVz(-desc);
      } // else means we are already under the threshold, so we can cut motors, by giving a negative desired velocity;
      // the drone will then disarm itself once touched the ground, according to its safety parameters
      else _comm->setVz(-_param.emergLandFinalDescSpeed);
    }

    double computeDescendRate(double dz)
    {
      // function that computes descending rate for the drone
      // if the error is greater than a first threshold, the descending rate
      // can be equal to a certain fixed parameter
      if(dz > _actualTask->params[0])
        return _actualTask->params[2];
      else // else, the rate descreases exponentially wrt the distance error.
        return exp(dz - _actualTask->params[0]);
    }

  public:

    EmergLand(MavState *_state, MavState *_comm,exec::task *_actualTask) : Command(_state, _comm, _actualTask)
    {
      _param.loadLanding();
    }

    void execute() override
    {
      emergLand();
    }

};

#endif //MOCAP2MAV_EMERGLAND_HPP
