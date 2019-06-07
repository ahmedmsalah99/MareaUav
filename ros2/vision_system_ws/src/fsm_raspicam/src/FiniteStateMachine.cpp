#include "FiniteStateMachine.h"

FiniteStateMachine::FiniteStateMachine()
{
}

State* FiniteStateMachine::getState()
{
	return currentState_;
}

FiniteStateMachine::~FiniteStateMachine()
{
}

void FiniteStateMachine::setState(State *actualState)
{
	currentState_ = actualState;
}

int FiniteStateMachine::getActualID()
{
	return getState()->getID();
}
