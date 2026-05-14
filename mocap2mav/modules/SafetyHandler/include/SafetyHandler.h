#ifndef SAFETYHANDLER_H
#define SAFETYHANDLER_H
#include "lcm_messages/exec/safety.hpp"
#include "lcm_messages/exec/safety_ack.hpp"
#include "lcm_messages/exec/release_payload.hpp"
#include <iostream>
#include <math.h>
#include <poll.h>
#include "common/common.h"
#include "common/conversions.h"
#include "common/MavState.h"
#include "utils/TimeHelpers.hpp"
#include <map>

class SafetyHandler
{
  private:

    struct
    {
	float battery_threshold1;
	float battery_threshold2;
	double cameragone_threshold;
	double gpsgone_threshold1;
	double gpsgone_threshold2;
	double cameratosafe_threshold;
	double params[14];
    } _safety_params {};

	
    double _PI = 3.141592653589;
    TimeManager tm;

  public:
    SafetyHandler();
    bool handleBattery(float remaining);
    bool handleBatteryReturn(float remaining);
    bool handleCamera(double camera_timestamp);
    bool handleGPS(double gps_timestamp);
    bool handleSafeHarbor(double cameragone_timestamp);
    void setParamsStruct(std::map<std::string, float> map);
    double getParam(int i);
    void setParam(int i, double value);
    void checkRelPayFeedback(exec::release_payload feedback);
    
    exec::safety_ack safety_ack;
    int gpsGoneCounter;
    bool target_dispersed;
    double home_longitude;
    double home_latitude;
    double home_altitude;
    double platform;
    double platform_rdv_lon;
    double platform_rdv_lat;
    double platform_rdv_alt;
    double sh_local_coord[3];
    double plat_local_coord[3];
    bool payload_released;
};

#endif
