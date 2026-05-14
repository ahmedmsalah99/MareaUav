#ifndef FINITESTATEMACHINE_H
#define FINITESTATEMACHINE_H

#include "State.h"

class State;

class FiniteStateMachine {

public:
	FiniteStateMachine();
	~FiniteStateMachine();

	State *getState();
	void setState(State *ActualState);

	int getActualID();

private:
	State *currentState_;
};

#endif
