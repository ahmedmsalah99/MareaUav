#include "SafetyHandler.h"

SafetyHandler::SafetyHandler()
{
  safety.isbatterygone = 0;
  safety.iscameragone = 0;
  safety.isgpsgone = 0;
  safety.issonargone = 0;
}

void SafetyHandler::setParamsStruct(std::map<std::string, float> map)
{
  // assigning the corresponding parameters on the struct
  _safety_params.battery_threshold = map.at("battery_threshold");
  _safety_params.params[0] = map.at("emergland_threshold1");
  _safety_params.params[1] = map.at("emergland_threshold2");
  _safety_params.params[2] = map.at("emergland_gain");
}

double SafetyHandler::getParam(int i)
{
  return _safety_params.params[i];
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
