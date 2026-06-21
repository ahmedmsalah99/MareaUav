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

    // Ascend if platform is lost and we are still below zMax (relative to pad).
    bool asceValid = _lost && (_verticalErr < param.zMax - 0.1);

    // Ascend if setpoint has drifted below zMin (absolute hard floor).
    bool asceUnderMin = (_setPoint.getZ() < param.zMin);

    // Ascend if drone is closer to the platform than z_margin and not actively descending.
    // Only fires when vision is available to avoid blind ascent on stale verticalErr.
    bool asceUnderMargin = (_verticalErr < param.z_margin) && !descValid
                           && _VisionPose.VisionDataUpdated;

    // If vision is lost, go back to init when drone is still below zMax above the pad.
    bool InitValid = !_VisionPose.VisionDataUpdated && (_verticalErr < param.zMax - 0.1);

    // Enter compensation once the ultrasonic reads closer than zMax - 0.2 m to the pad.
    bool compValid = _holding && (_UltraInfo.getZ() < param.zMax - 0.2) && _centered;
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

    if (asceUnderMargin){
        this->_contextL->setStatePtr(_nextAscState); // below z_margin above pad — ascend
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
