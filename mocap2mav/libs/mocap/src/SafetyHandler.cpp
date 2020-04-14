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
  _safety_params.home_longitude = map.at("home_longitude");
  _safety_params.home_latitude = map.at("home_latitude");
  _safety_params.home_altitude = map.at("home_altitude");
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


	void SafetyHandler::globalToLocalCoords()
  {
    // getting Safe Harbor global coordinates
    double sh_lon = getParam(3);
    double sh_lat = getParam(4);
    double sh_alt = getParam(5);
    // and defining a common constant
    double earth_radius = 6371000; // (meters)
    double polar_radius = 6356752; // (meters)

    // computing the distance between home position andsafe harbor one using Haversine formula
    // float a = sinf((sh_lat - _safety_params.home_latitude)/2)*sinf((sh_lat - _safety_params.home_latitude)/2) +
  	//  					cosf(_safety_params.home_latitude)*cosf(sh_lat)*sinf((sh_lon - _safety_params.home_longitude)/2)*sinf((sh_lon - _safety_params.home_longitude)/2);
    //
  	// float c = 2.0f*atan2f(sqrtf(a), sqrtf(1.0f-a));
    //
    // d = earth_radius * c;
    double cap_radius_sh = earth_radius*cosf(deg2rad(sh_lat));
    double cap_radius_h = earth_radius*cosf(deg2rad(_safety_params.home_latitude));
    double mean_cap_radius = (cap_radius_h + cap_radius_sh)/2.0;

    // converting it in nautical miles and then in meters
    sh_local_coord[0] = mean_cap_radius * deg2rad(sh_lon - _safety_params.home_longitude);

    sh_local_coord[1] = polar_radius * deg2rad(sh_lat - _safety_params.home_latitude);

    sh_local_coord[2] =  sh_alt - _safety_params.home_altitude;

    // std::cout << "Longitude: " << sh_local_coord[0] << std::endl;
    // std::cout << "Latitude: " << sh_local_coord[1] << std::endl;

  }

  float SafetyHandler::computeDesiredYaw(MavState pose)
  {
    // getting Safe Harbor local coordinates
    double sh_lon = sh_local_coord[0];
    double sh_lat = sh_local_coord[1];
    // and drone ones
    double drone_lon = pose.getX();
    double drone_lat = pose.getY();
    // getting the actual yaw angle
    double rapp = (pose.getX() - sh_local_coord[0])/(pose.getY() - sh_local_coord[1]);

    double alfa = atanf(rapp);
    //
    // float x = cosf(sh_lat)*sinf(sh_lon - home_lon);
  	// float y = cosf(home_lat)*sinf(sh_lat) -
  	// 					sinf(home_lat)*cosf(sh_lat)*cosf(sh_lon - home_lon);

    // converting it wrt the local reference frame (x on EST, y on NORTH)
    return  deg2rad(90) - alfa;

    // float heading = bearing - euler_angles.psi();
    // // adjusting the heading wrt NED frame.
    // if(heading > _PI)
    // {
    //   heading = heading - 2.0f*_PI;
    // }
    // if(heading < -_PI)
    // {
    //   heading = heading + 2.0f*_PI;
    // }
  }

  float SafetyHandler::deg2rad(float angle_degree)
  {
  	return (angle_degree*_PI)/180.0f;
  }
