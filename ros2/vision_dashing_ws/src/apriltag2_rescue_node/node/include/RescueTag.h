#ifndef RESCUETAG_H
#define RESCUETAG_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>

#define PI 3.14159265

struct Rescue_tag {
	Eigen::Matrix4d TAG1;
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

void SetMatrix(struct Rescue_tag *TAGS);

void SetMatrix(struct Rescue_tag *TAGS) {
	/*TAGS->TAG1 << 1,0,0,0,
			     0,1,0,0,
			     0,0,1,0,
			     0,0,0,1;*/
	//The following values are computed using motion capture
	TAGS->yaw = -PI/2;
	TAGS->pitch = PI;
	TAGS->x = 0.06;
	TAGS->y = 0.0;
	TAGS->z = -0.03;

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
