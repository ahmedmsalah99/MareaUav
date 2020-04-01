#include "SafetyHandler.h"

SafetyHandler::SafetyHandler()
{
  gpsGoneCounter = 0;
}

void SafetyHandler::setParamsStruct(std::map<std::string, float> map)
{
  // assigning the corresponding parameters on the struct
  _safety_params.battery_threshold = map.at("battery_threshold");
  _safety_params.gpsgone_threshold1 = map.at("gpsgone_threshold1");
  _safety_params.gpsgone_threshold2 = map.at("gpsgone_threshold2");
  _safety_params.cameragone_threshold = map.at("cameragone_threshold");
  _safety_params.cameratosafe_threshold = map.at("cameratosafe_threshold");
  _safety_params.params[0] = map.at("emergland_threshold1");
  _safety_params.params[1] = map.at("emergland_threshold2");
  _safety_params.params[2] = map.at("emergland_gain");
  _safety_params.params[3] = map.at("safeharbor_lon");
  _safety_params.params[4] = map.at("safeharbor_lat");
  _safety_params.params[5] = map.at("safeharbor_alt");
  _safety_params.params[6] = map.at("safeharbor_threshold");
  _safety_params.params[7] = map.at("cameragone_altitude");
  _safety_params.params[8] = map.at("movetosafeharbor_altitude");
  _safety_params.params[9] = map.at("move_alpha");
}

double SafetyHandler::getParam(int i)
{
  return _safety_params.params[i];
}

void SafetyHandler::setParam(int i, double value)
{
  _safety_params.params[i] = value;
}

bool SafetyHandler::handleBattery(float remaining)
{
  // if the battery level passes the prefixed threshold, some actions has to be taken
  if(remaining <= _safety_params.battery_threshold)
  {
    return true;
  }
  // else the execution of the algorithm can proeed normally
  return false;
}

bool SafetyHandler::handleCamera(double camera_timestamp)
{
  // if the timestamps difference is greater than a certain threshold, means that
  // it passed too much time since the last camera publication, aka camera doesn't
  // publish, aka it's dead
  if(((double)tm.getTimeMilliSecond() - camera_timestamp)*0.001 > _safety_params.cameragone_threshold)
    return true;

  return false;
}

bool SafetyHandler::handleGPS(double gps_timestamp)
{
  // same as camera check.
  if(((double)tm.getTimeMilliSecond() - gps_timestamp)*0.001 > _safety_params.gpsgone_threshold1)
    return true;

  return false;
}

bool SafetyHandler::handleSafeHarbor(double cameragone_timestamp)
{
  if(cameragone_timestamp == 0.0)
    return false;
  else if(((double)tm.getTimeMilliSecond() - cameragone_timestamp)*0.001 > _safety_params.cameratosafe_threshold)
    return true;

  return false;
}

// TODO: handle
// bool SafetyHandler::handleGPS()
// {
//   // if the number of consecutive times the GPS has no signal passes the prefixed
//   // threshold, returning true.
//   if(gpsGoneCounter >= _safety_params.gpsgone_threshold)
//     return true;
//
//   return false;
// }
