//
// Created by andreanistico on 07/02/17.
//

#include "AbstractState.h"

AbstractState::AbstractState(Machine *context) : _context(context) {}

int AbstractState::getId() const {
    return _id;
}

AbstractState::~AbstractState() {

}



