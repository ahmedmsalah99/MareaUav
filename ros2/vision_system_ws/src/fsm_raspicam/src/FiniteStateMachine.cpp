#include "FiniteStateMachine.h"

FiniteStateMachine::FiniteStateMachine(){}

State* FiniteStateMachine::getState(){
	
	
	return CurrentState;
	}
	
FiniteStateMachine::~FiniteStateMachine(){}
	
void FiniteStateMachine::setState(State *ActualState){
	
	CurrentState=ActualState;
	}

int FiniteStateMachine::getActualID(){
	
	return getState()->getID();
	}
