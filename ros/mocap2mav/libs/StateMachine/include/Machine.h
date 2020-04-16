//
// Created by andreanistico on 07/02/17.
//

#ifndef STATEMACHINE_MACHINE_H
#define STATEMACHINE_MACHINE_H

#include "AbstractState.h"

class AbstractState;
class Machine {


public:

    Machine();
    Machine(AbstractState* init);
    ~Machine();

    AbstractState* getStatePtr();
    int getActualNodeId();

    void handle();
    void setStatePtr(AbstractState *state) ;


private:
    AbstractState* _state;
};


#endif //STATEMACHINE_MACHINE_H
