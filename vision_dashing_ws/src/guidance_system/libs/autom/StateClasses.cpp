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
    double downDist = -_verticalErr;
    
    double ultra_limit = 0.83 - 0.2;
    // Old close-range (ultrasonic) condition, kept under a new name for the final
    // landing phase.
    bool ultraCompValid = _holding && (fabs(_UltraInfo.getZ()) < ultra_limit+0.1) && _centered;
    (void)ultraCompValid;   // reserved for the final landing phase

    // Enter COMP once we are within the standoff distance (2 m) of the pad,
    // centred, holding and on a calm sea. COMP then holds compHoldDist (1.5 m).
    bool compValid = _holding && _centered && (downDist < param.standoffDistance)
                    && _contextL->safeInitialLanding();
    // B1: descend only while farther than the standoff distance (2 m) from the
    // pad. Once within the standoff we stop descending and hold there; the
    // compensated/aggressive landing from the standoff comes in later phases.
    bool descValid = _centered && _holding && (_setPoint.getZ() > param.zMin) && (downDist > param.standoffDistance)
                    && _contextL->safeInitialLanding();

    // Ascend if platform is lost and we are still below zMax (relative to pad).
    bool asceValid = _lost && (downDist < param.zMax - 0.1);

    // Ascend if setpoint has drifted below zMin (absolute hard floor).
    bool asceUnderMin = (_setPoint.getZ() < param.zMin);

    // Ascend if the drone is closer than the standoff distance and not actively
    // descending — together with descValid this parks the drone at the standoff.
    bool asceUnderMargin = _centered && _holding &&  (downDist > param.standoffDistance) && (downDist < param.zMax - 0.1) && !descValid;

    //if the vision data are not available and the drone is at or above zMax, go back to init.
    // bool InitValid = !_VisionPose.VisionDataUpdated && !(_setPoint.getZ() < param.zMax - 0.1);
    bool InitValid = !_VisionPose.VisionDataUpdated;

    //comp is a middle state between land state and hold state.
    //if the UAV is holding ad is centered on the platform and is close to the platform(20 cm), will be accomplish.

    
    // std::cout << compValid << std::endl;
    
    // std::cout << "downDist " << downDist << "compValid " << compValid << " InitValid " << InitValid << " descValid " << descValid << " asceUnderMargin is " << asceUnderMargin << " and asceUnderMin " << asceUnderMin << " and asceValid " << asceValid << std::endl;
    
    if(InitValid){
      this->_contextL->setStatePtr(_nextInitState); //set the next state (InitState)
      printStateTransition();
      return;
    }

    if(asceUnderMin){
        this->_contextL->setStatePtr(_nextAscState); // below zMin floor — ascend
        printStateTransition();
        std::cout << "asceUnderMin " << " _setPoint.getZ() " << _setPoint.getZ() << std::endl;
        return;
    }
    if(compValid){
        this->_contextL->setStatePtr(_nextState); //set the next state (RToLandState)
        printStateTransition();
        waitComp = 0;
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
        std::cout << "asceValid " << asceValid << std::endl;
        return;
    }

    if (asceUnderMargin){
        this->_contextL->setStatePtr(_nextAscState); // below z_margin above pad — ascend
        printStateTransition();
        std::cout << "asceUnderMargin " << asceUnderMargin << std::endl;
        // std::cout << "_centered " << _centered << " _holding " << _holding << " z cond " << (_setPoint.getZ() > param.zMin + 0.1) << " _setPoint.getZ() "
        // << _setPoint.getZ() << " param.zMin + 0.1 " << param.zMin + 0.1 << " safe landing "<<
        //              _contextL->safeInitialLanding();
        return;
    }

    

    // if(InitValid){
	//       this->_contextL->setStatePtr(_nextInitState); //set the next state (InitState)
    //     printStateTransition();
    //     return;
    // }



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
    double downDist = -_verticalErr;
    bool onTarget = _NComp > param.NFramesComp;

    // COMP holds compHoldDist (1.5 m) above the pad and compensates the heave.
    // Rough platform attitude does NOT leave COMP (that's the aggressive-landing
    // abort, a later phase). Only a lost platform fix, or drifting back above the
    // standoff, leaves COMP.

    if(!_VisionPose.IsValid || !_VisionPose.VisionDataUpdated){
      // Lost a trustworthy platform fix — recover via hovering.
      this->_contextL->setStatePtr(_nextHoveringState);
      printStateTransition();
      std::cout << "out of comp: vision not valid/updated" << std::endl;
      return;
    }

    if(downDist > param.standoffDistance){
      // Drifted back above the standoff — return to HOLD.
      this->_contextL->setStatePtr(_nextState);
      printStateTransition();
      std::cout << "out of comp: gap above standoff, back to hold" << std::endl;
      return;
    }
    bool xyStill = std::fabs(_state.getVx()) < param.touchVelRelMax
                      && std::fabs(_state.getVy()) < param.touchVelRelMax;
    if(_contextL->getLandingGoSignal() && _centered && xyStill){
      // Landing window is open — commit to the (aggressive) landing.
      this->_contextL->setStatePtr(_nextLanState);
      printStateTransition();
      std::cout << "COMP -> LAND: GO window open" << std::endl;
      return;
    }

    // otherwise stay in COMP.
    (void)onTarget;

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
    bool onTarget = _NComp > param.NFramesComp;

    // Touchdown takes priority over everything else — once detected we commit
    // permanently to LANDING_DONE, even if the GO window happens to close or
    // vision drops the same tick (that's expected once sitting on the pad).
    if(_contextL->getTouchedDown()){
      this->_contextL->setStatePtr(_nextDoneState);   // -> LANDING_DONE (terminal)
      printStateTransition();
      std::cout << "LAND -> LANDING_DONE: touchdown detected" << std::endl;
      return;
    }

    // Abort the landing and go back to COMP if the GO window closes (NO-GO) or we
    // lose the platform fix. Otherwise stay in LAND — the committed descent is
    // issued in Lander::land() (XY stays centred via hold()).
    if(!_contextL->getLandingGoSignal() || !_centered){
      this->_contextL->setStatePtr(_nextState);   // -> COMP
      printStateTransition();
      std::cout << "LAND -> COMP: " << "on target " << onTarget << " _centered " << _centered 
                << (!_contextL->getLandingGoSignal() ? "NO-GO" : "vision lost") << std::endl;
      return;
    }

    // otherwise stay in LAND (committed descent).

}

void LandingDoneState::handle() {
    // Terminal: no transitions out. The drone stays put (Lander::run() freezes
    // the setpoint at the current position for this state) until an external
    // reset reinitialises the whole state machine.
}
