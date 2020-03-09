#include "SafetyHandler.h"

SafetyHandler::SafetyHandler()
{

}

void SafetyHandler::setParamsStruct(std::map<std::string, float> map)
{
  // assigning the corresponding parameters on the struct
  _safety_params.battery_threshold = map.at("battery_threshold");
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
