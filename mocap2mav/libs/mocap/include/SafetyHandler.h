#ifndef SAFETYHANDLER_H
#define SAFETYHANDLER_H
#include "lcm_messages/exec/safety.hpp"
#include <iostream>
#include <poll.h>
#include "common/common.h"
#include "utils/TimeHelpers.hpp"
#include <map>

class SafetyHandler
{
  private:

    struct
    {
      float battery_threshold;
      double cameragone_threshold;
      double gpsgone_threshold;
      double params[3];
    } _safety_params {};

	
    TimeManager tm;

  public:
    SafetyHandler();
    bool handleBattery(float remaining);
    bool handleCamera(double camera_timestamp);
    bool handleGPS(double gps_timestamp);
    void setParamsStruct(std::map<std::string, float> map);
    double getParam(int i);
    void setParam(int i, double value);
    
    exec::safety safety;
    int gpsGoneCounter;
};

#endif
