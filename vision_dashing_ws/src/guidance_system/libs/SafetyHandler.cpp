#include "guidance_system/safety/SafetyHandler.h"

SafetyHandler::SafetyHandler()
{
  payload_released = false;
}

void SafetyHandler::setParamsStruct(std::map<std::string, float> map)
{
  // assigning the corresponding parameters on the struct
  _safety_params.battery_threshold1 = map.at("battery_threshold1");
  _safety_params.battery_threshold2 = map.at("battery_threshold2");
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
  _safety_params.params[10] = map.at("cameraonline_threshold");
  _safety_params.params[11] = map.at("rotation_threshold");
  _safety_params.params[12] = map.at("reachplatform_threshold");
  _safety_params.params[13] = map.at("reachplatfrom_altitude");
  // and one as a public variable
  target_dispersed =(bool)map.at("target_dispersed");
  home_longitude = map.at("home_longitude");
  home_latitude = map.at("home_latitude");
  home_altitude = map.at("home_altitude");
  platform_rdv_lon = map.at("platform_rdv_lon");
  platform_rdv_lat = map.at("platform_rdv_lat");
  platform_rdv_alt = map.at("platform_rdv_alt");
  platform = map.at("platform");
}

double SafetyHandler::getParam(int i)
{
  return _safety_params.params[i];
}

void SafetyHandler::setParam(int i, double value)
{
  _safety_params.params[i] = value;
}

void SafetyHandler::checkRelPayFeedback(const std::string& feedback)
{
    if(feedback == "R")
    {
      payload_released = true;
    }
}

bool SafetyHandler::handleBattery(float remaining)
{
  // if the battery level passes the prefixed threshold, some actions has to be taken
  if(remaining <= _safety_params.battery_threshold1)
  {
    return true;
  }
  // else the execution of the algorithm can proeed normally
  return false;
}

bool SafetyHandler::handleBatteryReturn(float remaining)
{
  // second check on the battery in case we are not rescuing anyone
  if(remaining <= _safety_params.battery_threshold2)
    return true;
  return false;
}

bool SafetyHandler::handleCamera(double camera_timestamp)
{
  // if the timestamps difference is greater than a certain threshold, means that
  // it passed too much time since the last camera publication, aka camera doesn't
  // publish, aka it's dead
  if(((double)tm.getTimeMilliSecond() - camera_timestamp)*0.001 > _safety_params.cameragone_threshold) // *0.001: convert ms difference to seconds
    return true;

  return false;
}

bool SafetyHandler::handleGPS(double gps_timestamp)
{
  // same as camera check.
  if(((double)tm.getTimeMilliSecond() - gps_timestamp)*0.001 > _safety_params.gpsgone_threshold1) // *0.001: convert ms difference to seconds
    return true;

  return false;
}

bool SafetyHandler::handleSafeHarbor(double cameragone_timestamp)
{
  if(cameragone_timestamp == 0.0)
    return false;
  else if(((double)tm.getTimeMilliSecond() - cameragone_timestamp)*0.001 > _safety_params.cameratosafe_threshold) // *0.001: convert ms difference to seconds
    return true;

  return false;
}

	// void SafetyHandler::globalToLocalCoords()
  // {
  //   // getting Safe Harbor global coordinates
  //   double sh_lon = getParam(3);
  //   double sh_lat = getParam(4);
  //   double sh_alt = getParam(5);
  //   // and defining a common constant
  //   double earth_radius = 6371000; // (meters)
  //   double polar_radius = 6356752; // (meters)
  //
  //   // computing the distance between home position andsafe harbor one using Haversine formula
  //   // float a = sinf((sh_lat - _safety_params.home_latitude)/2)*sinf((sh_lat - _safety_params.home_latitude)/2) +
  // 	//  					cosf(_safety_params.home_latitude)*cosf(sh_lat)*sinf((sh_lon - _safety_params.home_longitude)/2)*sinf((sh_lon - _safety_params.home_longitude)/2);
  //   //
  // 	// float c = 2.0f*atan2f(sqrtf(a), sqrtf(1.0f-a));
  //   //
  //   // d = earth_radius * c;
  //   double cap_radius_sh = earth_radius*cosf(deg2rad(sh_lat));
  //   double cap_radius_h = earth_radius*cosf(deg2rad(_safety_params.home_latitude));
  //   double mean_cap_radius = (cap_radius_h + cap_radius_sh)/2.0;
  //
  //   // converting it
  //   sh_local_coord[0] = mean_cap_radius * deg2rad(sh_lon - _safety_params.home_longitude);
  //
  //   sh_local_coord[1] = polar_radius * deg2rad(sh_lat - _safety_params.home_latitude);
  //
  //   sh_local_coord[2] =  sh_alt - _safety_params.home_altitude;
  //
  //   // std::cout << "Longitude: " << sh_local_coord[0] << std::endl;
  //   // std::cout << "Latitude: " << sh_local_coord[1] << std::endl;
  //
  // }
  //
  // float SafetyHandler::computeDesiredYaw(MavState pose)
  // {
  //   // getting the actual yaw angle
  //   double rapp = (pose.getX() - sh_local_coord[0])/(pose.getY() - sh_local_coord[1]);
  //   // as the tangent angle between these two points
  //   double alpha = atanf(rapp);
  //   // converting it wrt the local reference frame (x on EST, y on NORTH)
  //   double alpha_local = _PI/2.0 - alpha;
  //   // std::cout << 180.0*alpha/_PI << std::endl;
  //   // checking its orientation then: if the point to be reached
  //   // is below (wrt local reference frame) the drone's position, then we fix the orientation
  //   if(sh_local_coord[1] - pose.getY() < 0.0)
  //   {
  //     alpha_local = alpha_local - _PI;
  //   }
  //   // returning then the final desired value
  //   return  alpha_local;
  // }
  //
  // float SafetyHandler::deg2rad(float angle_degree)
  // {
  // 	return (angle_degree*_PI)/180.0f;
  // }
