#ifndef SAFETYHANDLER_H
#define SAFETYHANDLER_H
#include "lcm_messages/exec/safety.hpp"
#include "lcm_messages/exec/safety_ack.hpp"
#include <iostream>
#include <math.h>
#include <poll.h>
#include "common/common.h"
#include "common/MavState.h"
#include "utils/TimeHelpers.hpp"
#include <map>

class SafetyHandler
{
  private:

    struct
    {
	float battery_threshold;
	double cameragone_threshold;
	double gpsgone_threshold1;
	double gpsgone_threshold2;
	double cameratosafe_threshold;
	double home_longitude;
	double home_latitude;
	double home_altitude;
	double params[12];
    } _safety_params {};

	
    double _PI = 3.141592653589;
    TimeManager tm;

  public:
    SafetyHandler();
    bool handleBattery(float remaining);
    bool handleCamera(double camera_timestamp);
    bool handleGPS(double gps_timestamp);
    bool handleSafeHarbor(double cameragone_timestamp);
    void setParamsStruct(std::map<std::string, float> map);
    double getParam(int i);
    void setParam(int i, double value);
    void globalToLocalCoords();
    float computeDesiredYaw(MavState pose);
    float deg2rad(float angle_degree);
    
    exec::safety_ack safety_ack;
    int gpsGoneCounter;
    double sh_local_coord[3];
};

#endif
