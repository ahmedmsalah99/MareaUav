#ifndef LANDINGPLATFORM_H
#define LANDINGPLATFORM_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

#define PI 3.14159265

struct Platform_tag {

Eigen::Matrix4d TAG0_T_TAG4;

Eigen::Matrix4d TAG1_T_TAG4;

Eigen::Matrix4d TAG2_T_TAG4;

Eigen::Matrix4d TAG3_T_TAG4;

Eigen::Matrix4d robot_T_cam; // transformation matrix between camera and robot

Eigen::Matrix3d R_z;// rotation around z

Eigen::Matrix3d R_y; 

Eigen::Matrix3d R_x;

Eigen::Matrix3d R_rpy; //rotation matrix of roll pitch and yaw

double yaw;
double roll;
double pitch;

//inside the sdf file of the iris.sdf, the position of the physical camera(in meters) is put:
double x;
double y; 
double z;
};

void SetMatrix(struct Platform_tag *TAGS);


void SetMatrix(struct Platform_tag *TAGS){

TAGS->TAG0_T_TAG4 << 1,0,0,0.24925,
		     0,1,0,0.24925,
		     0,0,1,0,
		     0,0,0,1;

TAGS->TAG1_T_TAG4 << 1,0,0,-0.24925,
		     0,1,0,0.24925,
		     0,0,1,0,
		     0,0,0,1;   

TAGS->TAG2_T_TAG4 << 1,0,0,0.24925,
		     0,1,0,-0.24925,
		     0,0,1,0,
		     0,0,0,1;  


TAGS->TAG3_T_TAG4 << 1,0,0,-0.24925,
		     0,1,0,-0.24925,
		     0,0,1,0,
		     0,0,0,1; 




TAGS->yaw = -PI/2; 
TAGS->pitch = PI;

TAGS->x = 0.06; 
TAGS->y = 0; 
TAGS->z = -0.03;



//computing the rotation matrices..

TAGS->R_z << cos(TAGS->yaw),-sin(TAGS->yaw),0,
	     sin(TAGS->yaw),cos(TAGS->yaw), 0,	
             0             ,0             , 1;

TAGS->R_x << 1,0,0,
             0,1,0,
   	     0,0,1;

TAGS->R_y << 1,0               ,0               ,
	     0,cos(TAGS->pitch),-sin(TAGS->pitch),
	     0,sin(TAGS->pitch),cos(TAGS->pitch) ;	
             
//complessive rotation matrix..
TAGS->R_rpy = TAGS->R_z * TAGS->R_y * TAGS->R_x;

TAGS->robot_T_cam << TAGS->R_rpy(0,0),TAGS->R_rpy(0,1),TAGS->R_rpy(0,2),TAGS->x,
		     TAGS->R_rpy(1,0),TAGS->R_rpy(1,1),TAGS->R_rpy(1,2),TAGS->y,
		     TAGS->R_rpy(2,0),TAGS->R_rpy(2,1),TAGS->R_rpy(2,2),TAGS->z,
		     0               ,0               ,0               ,1      ;


}



#endif
