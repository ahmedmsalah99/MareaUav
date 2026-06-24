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
#define THRE            0.25
#define DRATE_MIN       0.1
#define DRATE_MAX       0.3
#define TMAX            1.6
#define TMIN            0.7
#define PLATFORM_OFFSET 0.0


/** @ingroup Automatic
 * @brief Planner of the Landing procedure
 * @details Such class implements the landing
 * on a moving platform using a state machine.
 *
 */
class Lander {

public:

    /**
     * @brief Constructor of the class.
     * @details It initializes the members of the class (errors, values of the PID,
     * initialization of the state machine and it links each state of this class with
     * the LandMachine class.
     */
    Lander();

    /**
     * @brief It allows to take information coming from vision system
     * (apriltag_vision_system topic)
     * @param VisionPose Pose of the Robot respect to the platform
     */
    void setVisionPose(const MavState VisionPose);

    /**
     * @brief It allows to take info coming from platform/pose topic
     * @param platformState Absolute pose of the platform respect to the world frame
     */

    void setPlatformState(const MavState platformState);

    /**
     * @brief It allows to take info coming from UltrasonicSensor/platform topic
     * @param UltraInfo Partial pose of the drone using Ultrasonic sensor
     */

    void setUltrasonicInfo(const MavState UltraInfo);

    /**
     * @brief It initialize the state Machine for the landing procedure
     * @details It builds the state machine linking properly the states.
     * It links members of the LandMachine class instance with the Lander ones;
     * just because more than one state machine can exist at the same time.
     */

    void initStateMachine();

    /**
     * @brief It allows to take the absolute pose of the drone, vision_position_estimate
     * @param pose Full pose of the drone respect to the world frame
     */

    void setState(MavState pose);

	/**
	 * @brief It allows to set the desired yaw during seaching phase
	 * @param desired yaw coming from task struct
	 */

	void setYaw(float yaw);

    /**
     * @brief It controls the actual state of the machine and of the drone
     * @details It calls updateSignals() and handle() methods
     */

    void handleMachine();

  	/**
       * @brief It loads the new machine updating the parameter list
     */

  	void loadMachine();

  	/**
       * @brief It prints state machine signals
     */

    void printSignals();

    /**
     * @brief It computes the horizontal and vertical error of the drone,
     * incrementing properly some boolean members(_holding, _lost, _centered)
     * which allows to set the next state.
     */

    void updateSignals();
    /**
     * @brief It sends a set of setpoints to the drone in order to position
     * @return The actual state of the machine
     */

    bool setDroneInFront();
    /**
     * @brief It gets the actual state of the machine
     * @return The actual state of the machine
     */
	
    int getActualMachineState();

    /**
     * @brief It gets the pose of the drone
     * @return The pose of the drone respect to the world frame
     */

    MavState getState();

    /**
     * @brief It returns the command to send to the motor
     * @return The setPoint position
     */

    MavState getCommand();

    /**
     * @brief Returns the Go/NoGo prediction horizon T = min(|vertErr|/descVel, cap)
     * @return Prediction horizon in seconds, updated every tick in updateSignals()
     */

    double getPredictionHorizon() const { return _predictionHorizon; }

    /**
     * @brief The core function of the landing procedure. Mainly, it is
     * a switch based on the actual state.
     */

    void run();

    /**
     * Used to switch between vision system and ultrasonic sensor.
     */

    bool switchSensor;

  	/**
       * The following variables are used to implement an hovering action:
       */

  	/**
       * X hovering coordinate.
       */

  	double starting_x;

  	/**
       * Y hovering coordinate.
       */

  	double starting_y;

  	/**
       * Z hovering coordinate.
       */

  	double starting_z;

  	/**
       * To check if the state is set to hovering
       */

  	bool _hovering;

  	/**
       * To check if the machine has been reset
       */

  	bool _resetMachine;
	
	/**
	 * desired yaw
	 */

	float _yaw_des;


  	/**
       * To print signals every seconds
       */

  	int print;

  	double desc_velocity;

    // T = min(|verticalErr| / descVelocity, predHorizonMax) — updated every tick
    double _predictionHorizon;

    bool startFromBeginning;


    // bool positionedInFront;

    // perform inspection
     double time;
     double X0;
     double Y0;
     bool Inspection;
     bool First_insp;
     double RadiusInspection;
     double Period;
    //
    // //used to implement an hovering action for homing
    // bool _home;

private:

    LandMachine  _machine;

    /**
     * The full state of the drone respect to the world frame
     */

    MavState _state;

    /**
     * The position setPoint to accomplish the landing
     */

    MavState _setPoint;

    /**
     * Platform pose respect to the world frame
     */

    MavState _platformState;

    /**
     * Relative pose between drone and platform using vision system
     */

    MavState _VisionPose;

    static constexpr double kInvalidVisionHoldSecs = 3.0;
    double _invalidVisionTimer {0.0};  // seconds elapsed since vision went invalid
    bool   _inInvalidVisionHold {false};

    /**
     * Relative z position and velocity of the drone respect to the platform
     */

    MavState _UltraInfo;

    /**
     * PID for the x axis used in the hold state filled in the
     * constructor with variables in params_automatic namespace
     */

    MiniPID  _holdPIDX;

    /**
     * PID for the y axis used in the hold state filled in the
     * constructor with variables in params_automatic namespace
     */

    MiniPID  _holdPIDY;

private:

	/**
	 * Load parameters
	 */
	Parameters param;

    /**
     *  Init state of the machine linked to the hold state
     */

    InitState       _initS;

    /**
     * Holding state linked to the comp, asce and desc states
     */

    HoldState       _holdS;

    /**
     * Ascending state linked to the comp, hold and land states
     */

    AsceState       _asceS;

    /**
     * Descending state linked to the hold state
     */

    DescState       _descS;

    /**
     * RToLand state linked is between the comp and land states
     */

    RToLandState    _rtolS;

    /**
     * Compensation state linked to the asce, land and hold states
     */

    CompState       _compS;

    /**
     * Landing state is the final one, linked to the comp and asce states
     */

    LandState       _landS;

    InspectionState _inspeS;    //its constructor set its ID to INSPE
    // HomingState     _homingS;   //its constructor set its ID to HOMING
  	/**
       * state for hovering action
       */

  	HoveringState _hoveringS;

    /**
     * Actual state of the machine state
     */

    int _actualState;

    /**
     * Previous state of the machine state
     */

    int _prevState;

    /**
     * Differential time
     */

    double _dt;

    /**
     * Actual time
     */

    uint64_t _actualTime;

    /**
     * Previous time
     */

    uint64_t _prevTime;

    /**
     * Horizontal error between drone and platform using vision system
     */

    double _horizontaErr;

    /**
     * Vertical error between drone and platform using vision system
     * and ultrasonic sensor
     */

    double _verticalErr;

    /**
     * Threshold value for the horizontal error to allow the hold state
     */

    double _tauHold;

    /**
     * Threshold value for the horizontal error to check if the platform is lost
     */

    double _tauLost;

    /**
     * It seems not used
     */

    double _tauErr;

    /**
     * It keeps how many times the _horizontaErr is under a given threshold (_tauLost)
     */

    int    _NHold;

    /**
     * It keeps how many times the _horizontaErr is above a given threshold (_tauLost)
     */

    int    _NLost;

    /**
     * It keeps how many times the drone is on place to accomplish the landing
     */

    int    _NComp;

    /**
     * It checks if the drone is on place respect to the platform
     */

    bool _centered;

    /**
     * It checks if the drone is holding (horizontal error) the platform
     */

    bool _holding;

    /**
     * It checks if the drone has lost the platform
     */

    bool _lost;    

    /**
     * It checks if the drone is aligned with the platform and ready to start the landing
     */

    bool _readyToLand;

    /**
     * Integral error of the drone respect to the platform
     */

    Eigen::Vector3d _err_int;

    /**
     * Actual error of the drone respect to the platform
     */

    Eigen::Vector3d _err;

    /**
     * Previous error of the drone respect to the platform
     */

    Eigen::Vector3d _err_prev;

    /**
     * It seems not used
     */

    Eigen::Vector3d _err_diff;

    /**
     * direction of the line the drone has to follow to descend
     */

    Eigen::Vector3d _line_direction;

    /**
     * @brief Logic and signals helpers
     * @details It updates the integral error and
     * it checks if is in a defined interval
     */

    void updateIntegrals();

    /**
     * @brief Logic and signals helpers
     * @details It resets the integral error
     */

    void resetIntegrals();

    /**
     * @brief Logic and signals helpers
     * @details It computes the _dt based on
     * the _actualTime
     */

    void managetime();

    /**
     * @brief It clamps the setPoint on z axis in a specific interval
     */

    void clampZSP();

    /**
     * @brief It resets position setpoint to actual position
     */

    void resetSetPoint();

    /**
     * @brief Initialize stuff here
     * @details It calls resetSetPoint() method
     */

    // Align yaw with platform
    void align();

    void init();

    /**
     * @brief Tracking logic defined here
     * @details  Tracking is performed by POSITION control, issuing the position setpoint
     * in order to achieve the desired velocity calculated by:
     *
     * Vdes = K * ep + Vplat
     * or
     * Psp = Pplat + K * Vplat
     *
     * where:
     *
     * Vdes = desired velocity, K = proportional gain, ep = position error,
     *
     * Vplat = paltform velocity
     */

    void hold();

    /**
     * @brief Go up every time of 10 cm in case we miss, reset PID values
     */

    void asce();

    /**
     * @brief Get closer to the target, every time of 10 cm
     */

    void desc();

    /**
     * @brief Compensate altitude oscillation before landing
     * @details It chooses between vision system or ultrasonic sensor based on
     * the actual height and then it calculates desired vertical velocity
     *  in order to compensate oscillations.
     */

    void comp();

    /**
     * @brief Aggressive land procedure
     */

    void land();

    /**
     * @brief Hovering action
     */

	void hovering();

  	// circular trajectory for inspection. It tries with a circular
  	// trajectory to find the platform
  	void inspection();
  	// //homing position action
  	// void homing();
   	void initInspection(); //initialize variables to perform inspection



    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

#endif //MOCAP2MAV_LANDER_H
