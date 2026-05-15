#ifndef CONVERSIONS_H
#define CONVERSIONS_H
#include <cmath>
#include <cstdint>
#include <Eigen/Geometry>
#include "MavState.h"


#define PI 3.141592653589

inline static void quaternion_to_dcm(const float quaternion[4], float dcm[3][3])
{
    float a = quaternion[0];
    float b = quaternion[1];
    float c = quaternion[2];
    float d = quaternion[3];
    float aSq = a * a;
    float bSq = b * b;
    float cSq = c * c;
    float dSq = d * d;
    dcm[0][0] = aSq + bSq - cSq - dSq;
    dcm[0][1] = 2.0 * (b * c - a * d);
    dcm[0][2] = 2.0 * (a * c + b * d);
    dcm[1][0] = 2.0 * (b * c + a * d);
    dcm[1][1] = aSq - bSq + cSq - dSq;
    dcm[1][2] = 2.0 * (c * d - a * b);
    dcm[2][0] = 2.0 * (b * d - a * c);
    dcm[2][1] = 2.0 * (a * b + c * d);
    dcm[2][2] = aSq - bSq - cSq + dSq;
}

inline static void dcm_to_euler(const float dcm[3][3], float* roll, float* pitch, float* yaw)
{
    float phi, theta, psi;
    theta = asin(-dcm[2][0]);

    if (fabsf(theta - (float)M_PI_2) < 1.0e-3f) {
        phi = 0.0f;
        psi = (atan2f(dcm[1][2] - dcm[0][1],
                dcm[0][2] + dcm[1][1]) + phi);

    } else if (fabsf(theta + (float)M_PI_2) < 1.0e-3f) {
        phi = 0.0f;
        psi = atan2f(dcm[1][2] - dcm[0][1],
                  dcm[0][2] + dcm[1][1] - phi);

    } else {
        phi = atan2f(dcm[2][1], dcm[2][2]);
        psi = atan2f(dcm[1][0], dcm[0][0]);
    }

    *roll = phi;
    *pitch = theta;
    *yaw = psi;
}

inline static void dcm_to_quaternion(const float dcm[3][3], float quaternion[4])
{
    quaternion[0] = (0.5 * sqrt(1.0 +
            (double)(dcm[0][0] + dcm[1][1] + dcm[2][2])));
    quaternion[1] = (0.5 * sqrt(1.0 +
            (double)(dcm[0][0] - dcm[1][1] - dcm[2][2])));
    quaternion[2] = (0.5 * sqrt(1.0 +
            (double)(-dcm[0][0] + dcm[1][1] - dcm[2][2])));
    quaternion[3] = (0.5 * sqrt(1.0 +
            (double)(-dcm[0][0] - dcm[1][1] + dcm[2][2])));
}

inline double yawFromQuaternion(const Eigen::Quaterniond& q) {
    return atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

inline Eigen::Quaterniond yawToQuaternion(const double yaw) {
    Eigen::Quaterniond quaterniond;

    quaterniond.w() = cos(yaw/2);
    quaterniond.x() = 0;
    quaterniond.y() = 0;
    quaterniond.z() = sin(yaw/2);

    return quaterniond;
}


inline static void mul(float q1[4],float q2[4],float res[4]) {
    res[1] =  q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2] + q1[0] * q2[1];
    res[2] = -q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1] + q1[0] * q2[2];
    res[3] =  q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0] + q1[0] * q2[3];
    res[0] = -q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3] + q1[0] * q2[0];
}

inline Eigen::Quaterniond getQuatFromYaw(double yaw){

    Eigen::Quaterniond quaterniond;

    quaterniond.w() = cos(yaw/2);
    quaterniond.x() = 0;
    quaterniond.y() = 0;
    quaterniond.z() = sin(yaw/2);

    return quaterniond;

}

inline float deg2rad(float angle_degree)
{
  return (angle_degree*PI)/180.0f;
}

inline void globalToLocalCoords(double coords[3], double start_ref[3])
{
  // getting target global coordinates
  double target_lon = coords[0];
  double target_lat = coords[1];
  double target_alt = coords[2];
  // and defining a common constant
  double earth_radius = 6371000; // (meters)
  double polar_radius = 6356752; // (meters)


  double cap_radius_sh = earth_radius*cosf(deg2rad(target_lat));
  double cap_radius_h = earth_radius*cosf(deg2rad(start_ref[1]));
  double mean_cap_radius = (cap_radius_h + cap_radius_sh)/2.0;

  // converting it
  coords[0] = mean_cap_radius * deg2rad(target_lon - start_ref[0]);

  coords[1] = polar_radius * deg2rad(target_lat - start_ref[1]);

  coords[2] = target_alt - start_ref[2];

  // std::cout << "Longitude: " << _actualTask.x << std::endl;
  // std::cout << "Latitude: " <<_actualTask.y << std::endl;

}

inline float computeDesiredYaw(MavState pose, double coords[3])
{
  // getting the actual yaw angle
  double rapp = (pose.getX() - coords[0])/(pose.getY() - coords[1]);
  // as the tangent angle between these two points
  double alpha = atanf(rapp);
  // converting it wrt the local reference frame (x on EST, y on NORTH)
  double alpha_local = PI/2.0 - alpha;
  // std::cout << 180.0*alpha/_PI << std::endl;
  // checking its orientation then: if the point to be reached
  // is below (wrt local reference frame) the drone's position, then we fix the orientation
  if(coords[1] - pose.getY() < 0.0)
  {
    alpha_local = alpha_local - PI;
  }
  // returning then the final desired value
  return  alpha_local;
}

#endif
