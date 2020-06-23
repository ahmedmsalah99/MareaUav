//
// Created by andreanistico on 15/12/17.
//



#ifndef MOCAP2MAV_PARAMETERS_H
#define MOCAP2MAV_PARAMETERS_H
//
// Created by andreanistico on 24/10/17.
//

#include <string>
#include <iostream>
#include <fstream>
#include <algorithm>


class Parameters {

public:
    Parameters();

    void loadConfigFile(const char* config_file);

private:

    bool loaded;

    void loadParamFile(const char* config_file);

public:

    //PARAMETERS FIELDS
    
	//if 1 ==>inspection, if 0 it implements an hovering action
	bool search;

    //init value of radius for inspection
    double inspeRadius;

    //linear velocity of the drone during inspection
    double inspeLinVel;

    //roll upper limit to land
    double RollThreshold;

    //pitch upper limit to land
    double PitchThreshold;

    //Number of consecutive frames in which tracking is considered valid
    int    NFramesHold;

    //Number of consecutive frames in which tracking is considered not valid
    int    NFramesLost;

    //Number of consecutive frames in which tracking is considered valid
    //and robot is ready for compensation
    int    NFramesComp;

    //Platform dimension
    double platformLenght;

    //horizontal error allowed during holding phase
    double hold_threshold;

    //horizontal error allowed during compensation phase
    double comp_threshold;

    //vertical error allowed
    double lost_threshold;

    //switch off the motors
    double land_threshold;
    //Max altitude for landing procedure
    double zMax;

    //Minimum altitude for landing procedure (before compensating, this value should be above the maximum platform altitude)
    double zMin;

    //Proportional gain times platform velocity
    double KpHoldV;

    //Proportional gain times horizontal error
    double KpHold; // 0.5

    //Differential gain times horizontal error
    double KdHold;

    //Integral gain times integral horizontal error
    double KiHold; //0.1

    //Proportional gain for velocity tracking
    double KPCompV;

    //Integral clamping values
    double maxIntValue;
    double minIntValue;

    //Max total PID output
    double maxOutput;

};


#endif //MOCAP2MAV_PARAMETERS_H

    
