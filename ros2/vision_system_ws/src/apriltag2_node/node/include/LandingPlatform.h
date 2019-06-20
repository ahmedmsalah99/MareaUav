#ifndef LANDINGPLATFORM_H
#define LANDINGPLATFORM_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>

#define PI 3.14159265

struct Platform_tag {
	Eigen::Matrix4d TAG1_T_TAG5;
	Eigen::Matrix4d TAG2_T_TAG5;
	Eigen::Matrix4d TAG3_T_TAG5;
	Eigen::Matrix4d TAG4_T_TAG5;
	Eigen::Matrix4d TAG6_T_TAG5;
	Eigen::Matrix4d TAG7_T_TAG5;
	Eigen::Matrix4d TAG8_T_TAG5;
	Eigen::Matrix4d TAG9_T_TAG5;
	Eigen::Matrix4d robot_T_cam; //transformation matrix between camera and robot.
	Eigen::Matrix3d R_z;         //rotation around z axis
	Eigen::Matrix3d R_y;         //rotation around y axis
	Eigen::Matrix3d R_x;         //rotation around x axis
	Eigen::Matrix3d R_rpy;       //roll pitch yaw rotation matrix.
	//The following parameters are defined between the drone frame and camera frame.
	double yaw;
	double roll;
	double pitch;
	double x;
	double y;
	double z;
};

void SetMatrix(struct Platform_tag *TAGS);

void SetMatrix(struct Platform_tag *TAGS) {
	TAGS->TAG1_T_TAG5 << 1,0,0,0.243182,
						0,1,0,0.245111,
						0,0,1,0,
						0,0,0,1;
	TAGS->TAG2_T_TAG5 << 1,0,0,-0.242693,
						0,1,0,0.244019,
						0,0,1,0,
						0,0,0,1;   
	TAGS->TAG3_T_TAG5 << 1,0,0,-0.240737,
						0,1,0,-0.248717,
						0,0,1,0,
						0,0,0,1;  
	TAGS->TAG4_T_TAG5 << 1,0,0,0.24212,
						0,1,0,-0.248299,
						0,0,1,0,
						0,0,0,1; 
	TAGS->TAG6_T_TAG5 << 1,0,0,0.163,
						0,1,0,0.004,
						0,0,1,0,
						0,0,0,1; 
	TAGS->TAG7_T_TAG5 << 1,0,0,-0.009,
						0,1,0,0.165,
						0,0,1,0,
						0,0,0,1; 
	TAGS->TAG8_T_TAG5 << 1,0,0,-0.165,
						0,1,0,0.0045,
						0,0,1,0,
						0,0,0,1; 
	TAGS->TAG9_T_TAG5 << 1,0,0,-0.017,
						0,1,0,-0.174,
						0,0,1,0,
						0,0,0,1;
	//The following values are computed using motion capture
	TAGS->yaw = -PI/2;
	TAGS->pitch = PI;
	TAGS->x = 0.0;
	TAGS->y = 0.0;
	TAGS->z = -0.09;

	//computing the rotation matrix..
	TAGS->R_z << cos(TAGS->yaw), -sin(TAGS->yaw), 0,
		     sin(TAGS->yaw),  cos(TAGS->yaw), 0,
				  0,               0, 1;
	TAGS->R_x << 1,0,0,
				0,1,0,
				0,0,1;
	TAGS->R_y << 1,                0,                 0,
		     0, cos(TAGS->pitch), -sin(TAGS->pitch),
	             0, sin(TAGS->pitch),  cos(TAGS->pitch);
	//Complessive rotation matrix..
	TAGS->R_rpy = TAGS->R_z * TAGS->R_y * TAGS->R_x;
	TAGS->robot_T_cam << TAGS->R_rpy(0,0), TAGS->R_rpy(0,1), TAGS->R_rpy(0,2), TAGS->x,
			     TAGS->R_rpy(1,0), TAGS->R_rpy(1,1), TAGS->R_rpy(1,2), TAGS->y,
	        	     TAGS->R_rpy(2,0), TAGS->R_rpy(2,1), TAGS->R_rpy(2,2), TAGS->z,
			                    0,	              0,	        0,       1;
}

#endif
