#ifndef MOCAP2MAV_SEARCHER_H
#define MOCAP2MAV_SEARCHER_H

#include "StateMachine/include/Machine.h"
#include "common/MavState.h"
#include "common/common.h"
#include "SearchStateClasses.hpp"
#include "lcm_messages/exec/release_payload.hpp"
#include "miniPID/MiniPID.h"
#include "Lander/Parameters.h"


#define DEBUG
#define PI              3.141592653589

class Searcher
{
	private:

   		SearchMachine  _machine;

		/**
		 * The full state of the drone respect to the world frame
		 */

		MavState _state;

		/**
		 * The position setPoint to accomplish the inspection
		 */

		MavState _setPoint;


		/**
		 * Relative pose between drone and rescue target using vision system
		 */

		MavState _VisionPose;

		/**
		 *  Init state of the machine linked to the hold state
		 */

		InitSearchState       _initS;

		/**
		 * Holding state
		 */

		HoldSearchState       _holdS;

		/**
       		* state for hovering action
      		*/

  		HoveringSearchState _hoveringS;

		/**
		 * Inspection state
		 */

		InspectionSearchState _inspeS;

		/**
		 * Inspection state
		 */

		ReleaseState _relS;

		/**
		 * Starting position on x for Inspection
		 */

		double _X0;

		/**
		 * Starting position on y for Inspection
		 */

		double _Y0;

		/**
		 * Starting position on z for Inspection
		 */

		double _Z0;

		/**
		 * Inspection Radius
		 */

		double _inspectionRadius;


		/**
		 * Inspection Desired Linear Velocity
		 */

		double _inspectionLinearVel;

		/**
		 * Inspection task running bool
		 */

     		bool _inspection;

		/*
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
		 * bool variable for Hovering
		 */

		bool _hovering;

		/**
		 * payload released feedback check
		 */

     		bool _payloadReleased;

		/**
		 * desired yaw
		 */

     		float _yaw_des;

		/**
		 * Inspection radius change time
		 */

     		double _time;

		/**
		 * Inspection Period
		 */

		double _inspectionPeriod;

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
		 * Horizontal error between drone and target using vision system
		 */

		double _horizontaErr;

		/**
		 * Vertical error between drone and target using vision system
		 */

		double _verticalErr;

		/**
		 * Threshold value for the horizontal error to allow the hold state
		 */

		double _tauHold;

		/**
		 * Threshold value for the horizontal error to check if the target is lost
		 */

		double _tauLost;

		/**
		 * It keeps how many times the _horizontaErr is under a given threshold (_tauLost)
		 */

		int    _NHold;

		/**
		 * It keeps how many times the _horizontaErr is above a given threshold (_tauLost)
		 */

		int    _NLost;


		/**
		 * It checks if the drone is on place respect to the target
		 */

		bool _positioned;

		/**
		 * It checks if the drone is holding (horizontal error) the target
		 */

		bool _holding;

		/**
		 * It checks if the drone has lost the target
		 */

		bool _lost;

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
		 * PID for the x axis used in the hold state filled in the
		 * constructor with variables in params_automatic namespace
		 */

		MiniPID  _holdPIDX;

		/**
		 * PID for the y axis used in the hold state filled in the
		 * constructor with variables in params_automatic namespace
		 */

		MiniPID  _holdPIDY;

	   	/**
		 * @brief Logic and signals helpers
		 * @details It updates the integral error and
		 * it checks if is in a defined interval
		 */

		void updateIntegrals();


		/**
		 * Load parameters
		 */
		Parameters param;

		/**
		 * variable to communicate the intention to release the payload to extern functions
		 */

		bool _releasePayload;

    		/**
     		* @brief Hovering action
     		*/

		void hovering();

		/**
		 * @brief circular trajectory for inspection. It tries with a circular trajectory to find the target to
		 * rescue
		 */

 		void inspection();

		/**
		 * @brief initializes variables to perform inspection
		 */

   		void initInspection();

		/**
		 * @brief Logic and signals helpers
		 * @details It computes the _dt based on
		 * the _actualTime
		 */

		void managetime();

		/**
		  * Aligning with the target, if needed
		  */

		void align();

		/**
		  * Init state handler
		  */

		void init();

		/**
		  * Hold state handler
		  */

		void hold();

    		/**
		  * @brief It computes the horizontal and vertical error of the drone,
		  * incrementing properly some boolean members
		  * which allows to set the next state.
		  */

		void updateSignals();

		/**
		  * Prints data on the screen
		  */

		void printSignals();

		/**
		  * Release payload state handler
		  */

		void releasePayload();

	public:


		Searcher();

		/**
		 * @brief It allows to take information coming from vision system
		 * (apriltag_vision_system topic)
		 * @param VisionPose Pose of the Robot respect to the rescue target
		 */
    		void setVisionPose(const MavState VisionPose);

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
		 * @brief It returns the value of _releasePayload
		 */

		bool getPayloadCommand();

		/**
		 * @brief It returns the command to send to the motor
		 * @return The setPoint position
		 */

		MavState getCommand();

	    	/**
		 * @brief It initialize the state Machine for the searching procedure
		 * @details It builds the state machine linking properly the states.
		 * It links members of the SearchMachine class instance with the Searcher ones;
		 * just because more than one state machine can exist at the same time.
		 */

		void initStateMachine();

	  	/**
		  * @brief It loads the new machine updating the parameter list
		  */

	  	void loadMachine();

		/**
		 * @brief initializes parameters to perform inspection, passed throught the task object
		 */

   		void initInspectionParams(double initRadius, double initVelocity, double initZ);

		/**
		 * @brief sets the feedback coming from the release payload node
		 */

   		void setRelPayFeedback(exec::release_payload  feedback);

		/**
		 * @brief The core function of the searching procedure. Mainly, it is
		 * a switch based on the actual state.
		 */

		void run();

		/**
		 * @brief It controls the actual state of the drone
		 */

		void handleMachine();

	  	/**
	   	  * To check if the machine has been reset
	   	  */

		bool resetMachine;
		bool First_insp;

	  	/**
		  * To print signals
		  */

	  	int print;

};


#endif //MOCAP2MAV_SEARCHER_H
