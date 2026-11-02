//
// Created by andreanistico on 08/02/17.
//

#include <algorithm>
#include "guidance_system/autom/Lander/StatesClasses.hpp"
#include "guidance_system/autom/param.h"
//Define actual states

void InitState::handle(){
	//If the state of machine is INIT...

    //Got everything (assume it)
    getSignals();
    static int wait = 0;
    std::cout << "start param.inspecEnable " << param.inspecEnable << std::endl;
    //Wait 20 iterations
    if( wait++ > param.initWaitTicks && _VisionPose.VisionDataUpdated && _readyToLand) {
        this->_contextL->setStatePtr(_nextState);
        printStateTransition(); //print the actual state
        wait=0;
    }
	  else if(wait++ > param.initWaitTicks && !_VisionPose.VisionDataUpdated) {
        std::cout << "param.inspecEnable " << param.inspecEnable << std::endl;
      if(param.inspecEnable){
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
    

    // Enter COMP once we are within holdDescendAbove of the pad, centred,
    // holding and on a calm sea. COMP then holds compHeightBase (1.5 m).
    //
    // Uses the same (outer) threshold descValid stops descending at, not the
    // inner holdCompBelow: descent only ever brings downDist down to just
    // under holdDescendAbove before stopping, so gating compValid on the
    // tighter holdCompBelow left a gap nothing ever crossed -- the drone
    // would stop descending in the dead band and sit there forever, never
    // satisfying compValid, so COMP was never reached at all.
    bool compValid = _holding && _centered && (downDist < param.holdDescendAbove)
                    && _contextL->safeInitialLanding();


    // B1: descend only while farther than the (outer) descend threshold from
    // the pad. Once inside it we stop descending; the compensated/aggressive
    // landing from the standoff comes in later phases.
    bool descValid = _centered && _holding && (_setPoint.getZ() > param.altMinWorld) && (downDist > param.holdDescendAbove)
                    && _contextL->safeInitialLanding();

    // Ascend if platform is lost and we are still below zMax (relative to pad).
    bool asceValid = _lost && (downDist < param.altMaxAbovePad - param.altMaxAbovePadMargin);

    // Ascend if setpoint has drifted below zMin (absolute hard floor).
    bool asceUnderMin = (_setPoint.getZ() < param.altMinWorld);

    // Ascend if the drone is closer than the (inner) COMP threshold and not
    // actively descending -- i.e. too close to be cleared to hold there
    // (compValid false, e.g. sea not calm) but not far enough out to need a
    // fresh descent either. Uses the same inner threshold as compValid so
    // there's no gap between "not close enough to ascend" and "close enough
    // for COMP".
    bool asceUnderMargin = _centered && _holding &&  (downDist < param.holdCompBelow) && (downDist < param.altMaxAbovePad - param.altMaxAbovePadMargin) && !descValid;

    //if the vision data are not available and the drone is at or above zMax, go back to init.
    // bool InitValid = !_VisionPose.VisionDataUpdated && !(_setPoint.getZ() < param.altMaxAbovePad - param.altMaxAbovePadMargin);
    bool InitValid = !_VisionPose.VisionDataUpdated;

    //comp is a middle state between land state and hold state.
    //if the UAV is holding ad is centered on the platform and is close to the platform(20 cm), will be accomplish.

    
    // Throttled diagnostic (~1 Hz, HoldState runs at the full control-loop
    // rate) — shows which of compValid's/descValid's constituent conditions
    // is actually blocking the HOLD -> COMP transition. safeInitialLanding()
    // itself already logs which of ITS conditions (spa/heave freshness,
    // attitude limits, rates, heave velocity) is failing when called below.
    {
      static int logTick = 0;
      if ((logTick++ % 50) == 0) {
        std::cout << "[HOLD] downDist " << downDist
                  << " holdCompBelow " << param.holdCompBelow << " holdDescendAbove " << param.holdDescendAbove
                  << " compValid " << compValid << " descValid " << descValid
                  << " _holding " << _holding << " _centered " << _centered
                  << " InitValid " << InitValid
                  << " asceUnderMargin " << asceUnderMargin
                  << " asceUnderMin " << asceUnderMin
                  << " asceValid " << asceValid
                  << " compValidTimer " << *_contextL->_compValidTimer << "/" << param.holdToCompWait
                  << std::endl;
      }
    }

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

    // While the INIT slope is still driving Z (hold()'s _slopeActive), don't
    // let DESC/ASCE/COMP steal alternating ticks from it -- DescState/AsceState
    // always bounce straight back to HoldState after one call, so every tick
    // one of those fires instead of hold() is a tick the slope's own Z control
    // (position target + Vz feed-forward, both computed from the geometric
    // line) doesn't run at all. Just stay in HOLD and let the slope finish.
    if (*_contextL->_slopeActive) {
        return;
    }

    // Require compValid to hold for close to holdToCompWait seconds before
    // actually transitioning — a brief flicker into "valid" shouldn't be
    // enough on its own. The timer DECAYS (at the same rate it accumulates)
    // rather than hard-resetting to 0 the instant compValid drops, so a
    // single-tick dropout (e.g. safeInitialLanding() blipping false for one
    // heave-driven sample) only costs back roughly what it cost to earn,
    // instead of wiping out the whole accumulated window and forcing a full
    // holdToCompWait restart. A genuinely sustained loss of validity still
    // drains the timer back to 0 over about the same time it took to build
    // up. A full state-machine reset back to INIT still clears it outright
    // (Lander::initStateMachine()).
    if (compValid) {
      *_contextL->_compValidTimer += _contextL->_seaState->dt;
    } else {
      *_contextL->_compValidTimer = std::max(0.0, *_contextL->_compValidTimer - _contextL->_seaState->dt);
    }
    bool compValidSustained = compValid && (*_contextL->_compValidTimer >= param.holdToCompWait);
    if(compValidSustained){
        this->_contextL->setStatePtr(_nextState); //set the next state (RToLandState)
        printStateTransition();
        waitComp = 0;
        *_contextL->_compValidTimer = 0.0;   // consumed — start clean for next time
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
        // std::cout << "_centered " << _centered << " _holding " << _holding << " z cond " << (_setPoint.getZ() > param.altMinWorld + 0.1) << " _setPoint.getZ() "
        // << _setPoint.getZ() << " param.altMinWorld + 0.1 " << param.altMinWorld + 0.1 << " safe landing "<<
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
    bool onTarget = _NComp > param.compCenteredFramesRequired;

    // COMP holds compHeightBase above the pad and compensates the heave.
    // Rough platform attitude does NOT leave COMP (that's the aggressive-landing
    // abort, a later phase). Only a lost platform fix, or drifting back above the
    // standoff, leaves COMP.

    if(!_VisionPose.IsValid || !_VisionPose.VisionDataUpdated){
      // Lost a trustworthy platform fix — recover via hovering.
      this->_contextL->setStatePtr(_nextHoveringState);
      printStateTransition();
      *_contextL->_landValidTimer = 0.0;   // leaving COMP — reset the dwell timer
      std::cout << "out of comp: vision not valid/updated" << std::endl;
      return;
    }

    if(downDist > param.holdDescendAbove){
      // Drifted back above the (outer) standoff threshold — return to HOLD.
      // Uses the same outer threshold HOLD's descValid re-engages descent at,
      // so COMP doesn't get kicked back to HOLD by the same small heave blip
      // that HOLD's dead band is meant to absorb.
      this->_contextL->setStatePtr(_nextState);
      printStateTransition();
      *_contextL->_landValidTimer = 0.0;   // leaving COMP — reset the dwell timer
      std::cout << "out of comp: gap above standoff, back to hold" << std::endl;
      return;
    }


    // Minimum time spent in COMP before LAND is even considered — counts
    // plain dwell time in COMP, independent of the landing condition below.
    // Reset to 0 whenever COMP is left (the two early returns above) or on a
    // full state-machine reset (Lander::initStateMachine()).
    *_contextL->_landValidTimer += _contextL->_seaState->dt;
    bool compDwellElapsed = *_contextL->_landValidTimer >= param.goHold;

    bool xyStill = std::fabs(_state.getVx()) < param.touchXYSpeedMax
                      && std::fabs(_state.getVy()) < param.touchXYSpeedMax;
    bool xyPrecise = std::fabs(_VisionPose.getX()) < param.goXYErrMax
                      && std::fabs(_VisionPose.getY()) < param.goXYErrMax;
    // Same _heaveUpMax comp() widens its own standoff by (see comp()'s
    // effectiveHoldDist), capped separately here (goHeightCap) since
    // this gate is the LAST check before committing to land and should stay
    // tighter than the mid-COMP standoff cap. Without this, a rough sea
    // that's widened comp()'s standoff would leave downDist permanently
    // above a fixed goHeightBase -- heightOk would never pass and the
    // drone would never land. The cap applies to the TOTAL effective
    // threshold, not to the margin in isolation -- same reasoning as
    // comp()'s effectiveHoldDist.
    double effectiveHeightThld = std::min(param.goHeightBase + *_contextL->_heaveUpMax, param.goHeightCap);
    bool heightOk = downDist < effectiveHeightThld;
    bool goSignal = _contextL->getLandingGoSignal();
    bool landCondition = goSignal && _centered && xyStill && xyPrecise && heightOk;

    // Which condition is actually holding the landing up.
    //
    // landCondition is an instantaneous AND of five noisy terms, so a slow
    // commit is usually not one term being permanently false — it is several
    // being true only intermittently and rarely coinciding. A per-tick snapshot
    // cannot show that, so report the FRACTION of the last second each term
    // held true: the bottleneck is whichever fraction is low. The previous log
    // omitted goSignal and _centered entirely (2 of the 5 terms), so when either
    // was the blocker it printed all-ones and explained nothing.
    {
      static double win = 0.0;
      static double tGo = 0.0, tCentered = 0.0, tStill = 0.0, tPrecise = 0.0, tHeight = 0.0;
      const double dt = _contextL->_seaState->dt;
      win += dt;
      if (goSignal)  tGo       += dt;
      if (_centered) tCentered += dt;
      if (xyStill)   tStill    += dt;
      if (xyPrecise) tPrecise  += dt;
      if (heightOk)  tHeight   += dt;

      if (win >= 1.0) {
        std::cout << "[COMP] held-fraction over " << win << "s:"
                  << " go "        << (tGo       / win)
                  << " centered "  << (tCentered / win)
                  << " xyStill "   << (tStill    / win)
                  << " xyPrecise " << (tPrecise  / win)
                  << " heightOk "  << (tHeight   / win)
                  << " | dwell "   << *_contextL->_landValidTimer << "/" << param.goHold
                  << " visionXY ("  << _VisionPose.getX() << "," << _VisionPose.getY()
                  << ") thld "      << param.goXYErrMax
                  << " downDist "   << downDist << "/" << effectiveHeightThld
                  << " (base " << param.goHeightBase << " +heaveUpMax " << *_contextL->_heaveUpMax
                  << " capped@" << param.goHeightCap << ")"
                  << " spaValid "   << _contextL->_seaState->spaValid
                  << " heaveValid " << _contextL->_seaState->heaveValid
                  << std::endl;
        win = 0.0;
        tGo = tCentered = tStill = tPrecise = tHeight = 0.0;
      }
    }
    if(landCondition && compDwellElapsed){
      // Landing window is open, drone is precisely centred, and at least
      // goHold has been spent in COMP — commit to landing.
      if(param.landDisarmOnGo){
        // Disarm mode: skip LandState entirely — its committed descent and the
        // touchdown detector — and disarm straight into the terminal state.
        // NOTE: this deliberately shares the SAME preconditions as the normal
        // transition. Disarm mode changes what happens once the drone is
        // committed, not how it gets there: it still has to have descended to
        // within goHeightBase of the pad, be centred and still, and have held
        // COMP for goHold. Testing only the GO signal here (as this
        // first did) fires the instant COMP is entered, which is still up at the
        // standoff distance — the drone was dropping from ~1.4 m, far outside
        // the ultrasonic's ~0.83 m range, so it never even saw the pad.
        this->_contextL->requestDisarm();
        this->_contextL->setStatePtr(_nextDoneState);
        printStateTransition();
        *_contextL->_landValidTimer = 0.0;
        std::cout << "COMP -> LANDING_DONE: landDisarmOnGo, downDist " << downDist << std::endl;
        return;
      }
      this->_contextL->setStatePtr(_nextLanState);
      printStateTransition();
      *_contextL->_landValidTimer = 0.0;   // consumed — start clean for next time
      std::cout << "COMP -> LAND: GO window open" << std::endl;
      return;
    }

    // otherwise stay in COMP.
    (void)onTarget;

}

void RToLandState::handle() {

    getSignals();
    if (_NComp > param.compCenteredFramesRequired){
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
    bool onTarget = _NComp > param.compCenteredFramesRequired;

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
