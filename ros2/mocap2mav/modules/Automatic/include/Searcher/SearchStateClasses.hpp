#ifndef MOCAP2MAV_SEARCHSTATESCLASSES_HPP
#define MOCAP2MAV_SEARCHSTATESCLASSES_HPP

#include <iostream>
#include "StateMachine/include/Machine.h"
#include "common/MavState.h"
#include "Lander/Parameters.h"

/** @ingroup Automatic
 * @brief Derived class of the Machine one.
 *
 * @details Being the type of access public,
 * the private members of the base class are
 * hidden to the derivated one.
 * Its public members are used to share
 * the actual state of the drone, members of the Searcher class,
 * with the ones of the searchMachine.
 */

 class SearchMachine : public Machine
 {
  public:

    /**
     * Horizontal error between drone and platform using vision system
     */

    double *_horizontaErr;

    /**
     *  Vertical error between drone and platform using vision system
     * and ultrasonic sensor
     */

    double *_verticalErr;

    /**
     * Threshold value for the horizontal error to allow the hold state
     */

    double *_tauHold;

    /**
     * Threshold value for the horizontal error to check if the platform is lost
     */

    double *_tauLost;

    /**
     * It keeps how many times the _horizontaErr is under a given threshold (_tauLost)
     */

    int *_NHold;

    /**
     * It keeps how many times the _horizontaErr is above a given threshold (_tauLost)
     */

    int *_NLost;

    /**
     * The full state of the drone respect to the world frame
     */

    MavState *_state;

    /**
     * The position setPoint to accomplish the landing
     */

    MavState *_setPoint;

    /**
     * Relative z position and velocity of the drone respect to the target
     */

    MavState *_VisionPose;

    /**
     * It checks if the drone is holding (horizontal error) the target
     */

    bool *_holding;

    /**
     * It checks if the drone is on place with respect to the target
     */

    bool *_positioned;

    /**
     * It checks if the drone has lost the target
     */

    bool *_lost;

    /**
     * It checks if the drone needs to perform an Inspection to find the target
     */

    bool *_inspection;

    /**
     * It checks if the drone released the payload during its rescue mission
     */

    bool *_payloadReleased;

  	Parameters *param;
 };

 /** @ingroup Automatic
  * @brief Derived class of the AbstractState.
  * @details Being the type of access public,
  * the private members of the base class are
  * hidden to the derivated one.
  */

  class AbstractSearchState : public AbstractState
  {
    public:
      /**
       * It is a pointer to the next state.
       * It is useful to link each state and to build the
       * state machine.
       */

      AbstractState* _nextState;

      /**
       * Enumeration with represents the possible state of the search state machine
       */

      enum states
      {
          INIT,
          INSPE,
          HOVERING,
          HOLD,
          RELEASE
      };

      /**
       * @brief Constructor of the derived class.
       * @param context Pointer to the SearchMachine class
       */

      AbstractSearchState(SearchMachine *context) : AbstractState(context)
      {
          _contextL = context;
      }

      /**
       * @brief Such method allows to connect some members of the LandMachine class
       * with the ones of each state.
       * @details More in details, the Lander class has some members that it wants to share
       * with the LandMachine class, then the LandMachine class will share them also with
       * those states that want to know the actual situation.
       */

      void getSignals()
      {

           _horizontaErr = *(_contextL->_horizontaErr);
           _verticalErr  = *(_contextL->_verticalErr);
           _tauHold      = *(_contextL->_tauHold);
           _tauLost      = *(_contextL->_tauLost);
           _NHold        = *(_contextL->_NHold);
           _NLost        = *(_contextL->_NLost);
           _state        = *(_contextL->_state);
           _setPoint     = *(_contextL->_setPoint);
           _VisionPose   = *(_contextL->_VisionPose);
           param         = *(_contextL->param);
           //New Signals
           _holding      = *(_contextL->_holding);
           _positioned   = *(_contextL->_positioned);
           _lost         = *(_contextL->_lost);
           _inspection   = *(_contextL->_inspection);
           _payloadReleased   = *(_contextL->_payloadReleased);

      }

      /**
       * @brief Such method prints the actual ID of the current state
       */

      void printStateTransition()
      {
          std::cout << "Actual state: " << _contextL->getActualNodeId() << std::endl;
      }

    protected:

        /**
         * Horizontal error between drone and platform using vision system
         */

        double   _horizontaErr;

        /**
         * Vertical error between drone and platform using vision system
         * and ultrasonic sensor
         */

        double   _verticalErr;

        /**
         * Threshold value for the horizontal error to allow the hold state
         */

        double   _tauHold;

        /**
         * Threshold value for the horizontal error to check if the platform is lost
         */

        double   _tauLost;


        /**
         * It keeps how many times the _horizontaErr is under a given threshold (_tauLost)
         */

        int      _NHold;

        /**
         * It keeps how many times the _horizontaErr is above a given threshold (_tauLost)
         */

        int      _NLost;

         /**
         * The full state of the drone respect to the world frame
         */

        MavState _state;

        /**
         * The position setPoint
         */

        MavState _setPoint;

        /**
         * Relative pose between drone and target using vision system
         */

        MavState _VisionPose;


        /**
         * Pointer to the SearchMachine class.
         */

        SearchMachine* _contextL;


        /**
         * It checks if the drone is holding (horizontal error) the target
         */

        bool _holding;

        /**
         * It checks if the drone is on place with respect to the target
         */

        bool _positioned;

        /**
         * It checks if the drone has lost the target
         */

        bool _lost;


        /**
         * It checks if the drone needs to perform an Inspection to find the target
         */

        bool _inspection;

        /**
         * It checks if the drone released the payload during its rescue mission
         */

        bool _payloadReleased;

      	Parameters param;

  };

  /** @ingroup Automatic
   * @brief Derived class of the AbstractSearchState
   * Such class represents the init state.
   * This state will be linked with the SearchMachine class via
   * the protected member of the AbstractSearchState class(_contextL)
   */

class InitSearchState : public AbstractSearchState
{
  public:

    AbstractSearchState* _nextInspeState;

    /**
     * @brief Constructor of the derived class
     */

    InitSearchState(SearchMachine *context) :  AbstractSearchState(context)
    {
        setId();
    }

    /**
     * @brief It sets the ID to the INIT one.
     */

    void setId() override
    {

        _id = INIT;

    }

    /**
     * @brief Such method allows to check the current state of the drone
     * and it allows to set the next state of the search state machine.
     * @details In this case, the handle() method for InitState sets the next state(hold)
     * of the state machine.
     */

    void handle();
};

/** @ingroup Automatic
 * @brief Derived class of the AbstractSearchState
 * Such class represents the inspection state.
 */

class InspectionSearchState : public AbstractSearchState
{
  public:

    InspectionSearchState(SearchMachine *context) :  AbstractSearchState(context)
    {
        setId();
    }

    void setId() override
    {
        _id = INSPE;
    }

    void handle();
};


/** @ingroup Automatic
 * @brief Derived class of the AbstractSearchState
 * Such class represents the hovering state.
 * This state will be linked with the SearchMachine class via
 * the protected member of the AbstractSearchState class(_contextL)
 */

class HoveringSearchState : public AbstractSearchState {

public:

	HoveringSearchState(SearchMachine *context) : AbstractSearchState(context) {
        setId();
    }

	/**
     * @brief It sets the ID to the HOVERING one
     */

    void setId() override {

        _id = HOVERING;
    }

    /**
     * @brief Such method allows to check the current state of the drone
     * and it allows to set the next state of the land state machine.
     * @details In this case, the handle() method for Hovering sets the next
     * state of the machine to the init one; the hovering() method has been performed.
     */

    void handle();


};


/** @ingroup Automatic
 * @brief Derived class of the AbstractSearchState
 * Such class represents the hold state.
 * This state will be linked with the SearchMachine class via
 * the protected member of the AbstractSearchState class(_contextL)
 */

class HoldSearchState : public AbstractSearchState
{

  public:


    /**
     * The hold state is linked with the init state
     */

	  AbstractSearchState* _nextInitState;

    /**
     * The hold state is linked with the hovering state
     */

    AbstractSearchState* _nextHoveringState;

    /**
     * @brief Constructor of the derived class
     */

    HoldSearchState(SearchMachine *context) : AbstractSearchState(context)
    {
        setId();
    }

    /**
     * @brief It sets the ID to the HOLD one
     */

    void setId() override
    {
        _id = HOLD;
    }

    /**
     * @brief Such method allows to check the current state of the drone
     * and it allows to set the next state of the search state machine.
     * @details In this case, the handle() method for HoldState checks different boolean variables
     * (descValid, InitValid, asceValid, compValid) and based on those it sets the next
     * state of the machine
     */

    void handle();
};

/** @ingroup Automatic
 * @brief Derived class of the AbstractSearchState
 * Such class represents the payload releasing state.
 * This state will be linked with the SearchMachine class via
 * the protected member of the AbstractSearchState class(_contextL)
 */

class ReleaseState : public AbstractSearchState
{

  public:

    /**
     * @brief Constructor of the derived class
     */

    ReleaseState(SearchMachine *context) : AbstractSearchState(context)
    {
        setId();
    }

    /**
     * @brief It sets the ID to the RELEASE one
     */

    void setId() override
    {
        _id = RELEASE;
    }

    void handle();
};


#endif //MOCAP2MAV_SEARCHSTATESCLASSES_HPP
