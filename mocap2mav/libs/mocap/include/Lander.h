//
// Created by andreanistico on 06/02/17.
//

#ifndef MOCAP2MAV_LANDER_H
#define MOCAP2MAV_LANDER_H

#include "StateMachine/include/Machine.h"
#include "common/MavState.h"
#include "StatesClasses.hpp"
#include "common/common.h"
#include "miniPID/MiniPID.h"
#include "Parameters.h"

#define DEBUG
#define PI              3.141592653589
#define Kland           1
#define THRE            0.15
#define DRATE_MIN       0.2
#define DRATE_MAX       0.8
#define VMAX            1.5
#define TMAX            1.5
#define TMIN            0.7
#define PLATFORM_OFFSET 0.0

class Lander {

public:
    Lander();

    void setVisionPose(const MavState VisionPose);
    void setPlatformState(const MavState platformState);
    void initStateMachine();
    void setState(MavState pose);
    void handleMachine();
    void loadMachine();
    void updateSignals();
    void printSignals();
    int getActualMachineState();
    MavState getState();
    MavState getCommand();
    void run();

    bool switchSensor; //used to switch between vision system and ultrasonic sensor.

    bool _resetMachine; //used to reset the whole machine state.

    //perform inspection
    double time;
    double X0;
    double Y0;
    bool Inspection;
    double RadiusInspection;
    double Period;

    //used to implement an hovering action for homing
    double starting_x;
    double starting_y;
    double starting_z;
    bool _home;



private:

    LandMachine  _machine; //derived from machine one.

    MavState _state;
    MavState _setPoint;
    MavState _platformState;
    MavState _VisionPose;
    MiniPID  _holdPIDX;
    MiniPID  _holdPIDY;

private:

    //load parameter
    Parameters param;

    //Create states
    //each states are derived from AbstractLandState class.
    //The class AbstractLandState is derived from AbstractState.

    InitState       _initS;     //its constructor set its ID to INIT
    HoldState       _holdS;     //its constructor set its ID to HOLD
    AsceState       _asceS;     //its constructor set its ID to ASCE
    DescState       _descS;     //its constructor set its ID to DESC
    RToLandState    _rtolS;     //its constructor set its ID to R2LA
    CompState       _compS;     //its constructor set its ID to COMP
    LandState       _landS;     //its constructor set its ID to LAND
    InspectionState _inspeS;    //its constructor set its ID to INSPE
    HomingState     _homingS;   //its constructor set its ID to HOMING

    int _actualState;
    int _prevState;

    //Time helpers
    double _dt;
    uint64_t _actualTime;
    uint64_t _prevTime;

    //Signals
    double _horizontaErr;
    double _verticalErr;
    double _tauHold;
    double _tauLost;
    double _tauErr;
    int    _NHold;
    int    _NLost;
    int    _NComp;

    //NewSignals
    bool _centered;
    bool _holding;
    bool _lost;

    //Errors
    Eigen::Vector3d _err_int;
    Eigen::Vector3d _err;
    Eigen::Vector3d _err_prev;
    Eigen::Vector3d _err_diff;

    //Logic and signals helpers
    void updateIntegrals();
    void resetIntegrals();
    void managetime();

    //Clamp z SP
    void clampZSP();

    //Reset position setpoint to actual position
    void resetSetPoint();

    //Align yaw with platform
    void allign();
    //Initialize stuff here
    void init();
    //Tracking logic defined here
    void hold();
    //Go up in case we miss, reset integrals.
    void asce();
    //Get closer to the target
    void desc();
    //Compensate altitude oscillation before landing
    void comp();
    //Aggressive land
    void land();
    //circular trajectory for inspection. It tries with a circular 
    //trajectory to find the platform
    void inspection();
    //homing position action
    void homing();

    void initInspection(); //initialise variables to perform inspection 

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

#endif //MOCAP2MAV_LANDER_H
