#ifndef SAFETYHANDLER_H
#define SAFETYHANDLER_H
#include "lcm_messages/exec/safety.hpp"
#include <iostream>
#include <poll.h>
#include "common/common.h"
#include <map>

class SafetyHandler
{
  private:

    struct
    {
      float battery_threshold;
      double params[3];
    } _safety_params {};

  public:
    SafetyHandler();
    bool handleBattery(float remaining);
    void setParamsStruct(std::map<std::string, float> map);
    double getParam(int i);
    exec::safety safety;
};

#endif
