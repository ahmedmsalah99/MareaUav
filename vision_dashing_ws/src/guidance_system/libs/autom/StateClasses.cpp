//
// Created by andreanistico on 08/02/17.
//

#include "guidance_system/autom/Lander/StatesClasses.hpp"
#include "guidance_system/autom/param.h"
//Define actual states

void InitState::handle(){
	//If the state of machine is INIT...

    //Got everything (assume it)
    getSignals();
    static int wait = 0;
    std::cout << "start param.search " << param.search << std::endl;
    //Wait 20 iterations
    if( wait++ > 20 && _VisionPose.VisionDataUpdated && _readyToLand) {
        this->_contextL->setStatePtr(_nextState);
        printStateTransition(); //print the actual state
        wait=0;
    }
	  else if(wait++ > 20 && !_VisionPose.VisionDataUpdated) {
        std::cout << "param.search " << param.search << std::endl;
      if(param.search){
          this->_contextL->setStatePtr(_nextInspeState);
          printStateTransition(); //print the actual state.
      }
      else{
  		    this->_contextL->setStatePtr(_nextHoveringState);
          printStateTransition(); //print the actual state
      }
      wait=0;
	}


}

void InspectionState::handle() {
    getSignals();

    if(_VisionPose.VisionDataUpdated)
    {
      this->_contextL->setStatePtr(_nextState); //set next state (InitState)
      printStateTransition();
    }
}

void HoveringState::handle() {

	getSignals();

	if(_VisionPose.IsValid && _VisionPose.VisionDataUpdated )
  {
    this->_contextL->setStatePtr(_nextState);
    printStateTransition(); //print the actual state
  }

}

void HoldState::handle(){

    static int waitComp = 0;
    //copy the values of the LandMachine class into the ones of the AbstractLandState class.

    getSignals();

    // Descend only when holding AND go signal is active AND above zMin floor.
    bool descValid = _holding && (_setPoint.getZ() > param.zMin + 0.1)
                    && _contextL->getLandingGoSignal();

    // Ascend if platform is lost and we are still below zMax (original case).
    bool asceValid = _lost && (_setPoint.getZ() < param.zMax - 0.1);

    // Ascend if setpoint has drifted below zMin (safety floor).
    bool asceUnderMin = (_setPoint.getZ() < param.zMin);

    //if the vision data are not available and the drone is at or above zMax, go back to init.
    bool InitValid = !_VisionPose.VisionDataUpdated && !(_setPoint.getZ() < param.zMax - 0.1);

    //comp is a middle state between land state and hold state.
    //if the UAV is holding ad is centered on the platform and is close to the platform(20 cm), will be accomplish.

    // bool compValid = _holding && (fabs( _UltraInfo.getZ()) + param.zMin < 0.2 ) && _centered;
    bool compValid = _holding && (fabs(param.zMin - fabs( _UltraInfo.getZ())) > 0.1 ) && _centered;
    // bool compValid = _holding && (fabs( _UltraInfo.getZ()) < 0.80 ) && _centered;
    // std::cout << compValid << std::endl;

    if(!_VisionPose.IsValid){
      this->_contextL->setStatePtr(_nextHoveringState); //set the next state (InitState)
      printStateTransition();
      return;
    }

    if(asceUnderMin){
        this->_contextL->setStatePtr(_nextAscState); // below zMin floor — ascend
        printStateTransition();
        return;
    }

    if(descValid){
        this->_contextL->setStatePtr(_nextDesState); //set the next state (DescState)
        printStateTransition();
        return;
    }

    if (asceValid){
        this->_contextL->setStatePtr(_nextAscState); //set the next state (AsceState)
        printStateTransition();
        return;
    }

    if(compValid && waitComp++ > 10){
        this->_contextL->setStatePtr(_nextState); //set the next state (RToLandState)
        printStateTransition();
        waitComp = 0;
        return;
    }

    if(InitValid){
	      this->_contextL->setStatePtr(_nextInitState); //set the next state (InitState)
        printStateTransition();
        return;
    }



}

void DescState::handle() {
    getSignals();
    this->_contextL->setStatePtr(_nextState); //set the next state (HoldState)
    printStateTransition();
}

void AsceState::handle() {
    getSignals();
    this->_contextL->setStatePtr(_nextState); //set the next state (HoldState)
    printStateTransition();
}

void CompState::handle() {

    getSignals();

    /*
    NComp is incremented every time the variable centered is true.
    NframesComp is the number of consecutive frames in which tracking is considered valid
    and robot is ready for compensation.
    */

    bool onTarget = _NComp > param.NFramesComp;

    if(!_VisionPose.IsValid){
      this->_contextL->setStatePtr(_nextHoveringState); //set the next state (InitState)
      printStateTransition();
    }
    else if (!onTarget || !_centered){
        //if is not on the target or if is not centered..coming back!

        this->_contextL->setStatePtr(_nextState); //set the next state(AsceState)
        printStateTransition();

	}else if ( !_UltraInfo.UltrasonicDataUpdated ){
        //if the ultrasonic sensor data are not available..

        this->_contextL->setStatePtr(_nextState); //set the next state(AsceState)
        printStateTransition();

	}else if ( fabs(_VisionPose.getRoll()) > param.RollThreshold || fabs(_VisionPose.getPitch()) > param.PitchThreshold ){

		this->_contextL->setStatePtr(_nextState); //set the next state(AsceState)
        printStateTransition();

	}


  // std::cout << "VERRRRRRRR: " << _verticalErr << std::endl;

    if(onTarget && _centered && (fabs(_verticalErr) < param.land_threshold)){
	   //if is on the target and is centered and the vertical error is under 25cm, set next state to LandState.

        this->_contextL->setStatePtr(_nextLanState);
        printStateTransition();
    }

}

void RToLandState::handle() {

    getSignals();
    if (_NComp > param.NFramesComp){
        this->_contextL->setStatePtr(_nextComState); //set the next state(CompState)
        printStateTransition();
        return;
    } else if (!_centered){
        this->_contextL->setStatePtr(_nextState); //set the next state(HoldState)
        printStateTransition();
        return;
    }

}


void LandState::handle() {

    getSignals();
    static int wait = 0;
    bool onTarget = _NComp > param.NFramesComp;

    if (!onTarget || !_centered){ //set the next state(AsceState)
        //this->_contextL->setStatePtr(_nextState);
        printStateTransition();
        wait = 0;
    }

    //Restart the procedure
    //if(wait++ > 100) {
     //   this->_contextL->setStatePtr(_restartState); //set the next state(InitState)
       // printStateTransition();
        //wait=0;
    //}

}
