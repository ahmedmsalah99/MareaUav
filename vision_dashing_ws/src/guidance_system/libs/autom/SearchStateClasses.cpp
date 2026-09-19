#include "guidance_system/autom/Searcher/SearchStateClasses.hpp"

void InitSearchState::handle()
{
	//If the state of machine is INIT...

  //Got everything (assume it)
  getSignals();
  static int wait = 0;

  //Wait 10 iterations
  if( wait++ > 10 && _VisionPose.VisionDataUpdated)
  {
      this->_contextL->setStatePtr(_nextState);
      printStateTransition(); //print the actual state
      wait=0;
  }
	else if(wait++ > 10 && !_VisionPose.VisionDataUpdated)
  {
    this->_contextL->setStatePtr(_nextInspeState);
    printStateTransition(); //print the actual state
    wait=0;
	}
}

void InspectionSearchState::handle()
{
    getSignals();
    if(_VisionPose.VisionDataUpdated)
    {
        std::cout << "_VisionPose.VisionDataUpdated " << _VisionPose.VisionDataUpdated << std::endl;
        this->_contextL->setStatePtr(_nextState); //set next state (InitState)
        printStateTransition();
    }
}

void HoveringSearchState::handle() {

	getSignals();

	if(_VisionPose.VisionDataUpdated && _VisionPose.IsValid)
  {
    this->_contextL->setStatePtr(_nextState);
    printStateTransition(); //print the actual state
  }

}

void HoldSearchState::handle()
{

  static int wait = 0;
  // copy the values of the SearchMachine class into the ones of the AbstractSearchMachine class.

  getSignals();

  // if the vision data are not available, coming back to the initState

  bool InitValid = !_VisionPose.VisionDataUpdated && !_payloadReleased;
  //std::cout<<"InitValid: "<<InitValid<<std::endl;

  // then, if the drone is ready to release the payload (holding the target for
  // a certain time and is positioned wrt the target), change to ReleaseState
  bool ReleaseValid = _holding && _positioned && !_payloadReleased;

  if(!_VisionPose.IsValid){
    this->_contextL->setStatePtr(_nextHoveringState); //set the next state (HoveringState)
    printStateTransition();
    return;
  }

  if(ReleaseValid)
  {
    this->_contextL->setStatePtr(_nextState); //set the next state (ReleaseState)
    printStateTransition();
    return;
  }


  if(InitValid && wait++ > 20){
    this->_contextL->setStatePtr(_nextInitState); //set the next state (InitState)
    printStateTransition();
    wait=0;
    return;
  }
}

void ReleaseState::handle()
{

    getSignals();
    // static int wait = 0;
    // bool onTarget = _NComp > param.nFramesComp;
    //
    // if (!onTarget || !_centered)
    // { //set the next state(AsceState)
    //     //this->_contextL->setStatePtr(_nextState);
    //     printStateTransition();
    //     wait = 0;
    // }

    // if the payload got released, we come back to holding state, while waiting
    // for the relief efforts.
    if(_payloadReleased)
    {
      this->_contextL->setStatePtr(_nextState); //set the next state(HoldState)
      printStateTransition();
      return;
    }

    // printStateTransition();

}
