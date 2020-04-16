//
// Created by andreanistico on 08/02/17.
//

#ifndef MOCAP2MAV_STATESCLASSES_HPP
#define MOCAP2MAV_STATESCLASSES_HPP

#include <iostream>
#include "StateMachine/include/Machine.h"
#include "common/MavState.h"
#include "Parameters.h"


class LandMachine : public Machine {

public:
    double *_horizontaErr;
    double *_verticalErr;
    double *_tauHold;
    double *_tauLost;
    double *_tauErr;
    int *_NHold;
    int *_NLost;
    int      *_NComp;
    MavState *_state;
    MavState *_setPoint;
    MavState *_VisionPose;
    
    //New Signals
    Parameters *param;
    bool *_holding;
    bool *_lost;
    bool *_centered;
    bool *_Inspection;

};

class AbstractLandState : public AbstractState {

public:
    AbstractState* _nextState;
    enum states{

        INIT,
        INSPE,
        HOMING,
        HOLD,
        DESC,
        ASCE,
        R2LA,
        COMP,
        LAND

    };

    AbstractLandState(LandMachine *context) : AbstractState(context){
        _contextL = context;
    }

    void getSignals(){
        //copy the value of the LandMachine class into the ones of 
        //AbstractLandState class.

         _horizontaErr = *(_contextL->_horizontaErr);
         _verticalErr  = *(_contextL->_verticalErr);
         _tauHold      = *(_contextL->_tauHold);
         _tauLost      = *(_contextL->_tauLost);
         _tauErr       = *(_contextL->_tauErr);
         _NHold        = *(_contextL->_NHold);
         _NLost        = *(_contextL->_NLost);
         _NComp        = *(_contextL->_NComp);
         _state        = *(_contextL->_state);
         _setPoint     = *(_contextL->_setPoint);
         _VisionPose   = *(_contextL->_VisionPose);
         param         = *(_contextL->param);

        //New Signals
        _holding       = *(_contextL->_holding);
        _centered      = *(_contextL->_centered);
        _lost          = *(_contextL->_lost);

    }
    void printStateTransition(){
        std::cout << "Actual state: " << _contextL->getActualNodeId() << std::endl;
    }

protected:

    double   _horizontaErr;
    double   _verticalErr;
    double   _tauHold;
    double   _tauLost;
    double   _tauErr;
    int      _NHold;
    int      _NLost;
    int      _NComp;
    MavState _state;
    MavState _setPoint;
    MavState _VisionPose;
    LandMachine* _contextL;

    Parameters param;
    //New Signals
    bool _centered;
    bool _holding;
    bool _lost;


};

class InitState : public AbstractLandState {
public:
    AbstractLandState* _nextInspeState;
    AbstractLandState* _nextHomingState;

    InitState(LandMachine *context) :  AbstractLandState(context){
        setId();
    }
    void setId() override {

        _id = INIT;

    }
    void handle();
};

class InspectionState : public AbstractLandState {
public:

    InspectionState(LandMachine *context) :  AbstractLandState(context){
        setId();
    }
    void setId() override {

        _id = INSPE;

    }
    void handle();
};

class HomingState : public AbstractLandState {
public:

    HomingState(LandMachine *context) :  AbstractLandState(context){
        setId();
    }
    void setId() override {

        _id = HOMING;

    }
    void handle();
};

class HoldState : public AbstractLandState {
public:
    AbstractLandState* _nextAscState;
    AbstractLandState* _nextDesState;
    AbstractLandState* _nextInitState;


    HoldState(LandMachine *context) : AbstractLandState(context) {
        setId();
    }
    void setId() override {

        _id = HOLD;

    }
    void handle();
};

class DescState : public AbstractLandState {
public:
    DescState(LandMachine *context) : AbstractLandState(context) {
        setId();
    }
    void setId() override {

        _id = DESC;

    }
    void handle();
};

class AsceState : public AbstractLandState {
public:
    AsceState(LandMachine *context) : AbstractLandState(context) {
        setId();
    }

    void setId() override {

        _id = ASCE;

    }
    void handle();
};
class RToLandState : public AbstractLandState {
public:
    AbstractLandState* _nextComState;
    RToLandState(LandMachine *context) : AbstractLandState(context) {
        setId();
    }

    void setId() override {

        _id = R2LA;

    }
    void handle();
};
class CompState : public AbstractLandState {
public:
    AbstractLandState* _nextLanState;
    CompState(LandMachine *context) : AbstractLandState(context) {
        setId();
    }

    void setId() override {

        _id = COMP;

    }
    void handle();
};

class LandState : public AbstractLandState {
public:
    LandState(LandMachine *context) : AbstractLandState(context) {
        setId();
    }

    void setId() override {

        _id = LAND;

    }
    void handle();
};



#endif //MOCAP2MAV_STATESCLASSES_HPP
