//
// Created by andreanistico on 21/02/17.
//

#ifndef MOCAP2MAV_PARAM_H
#define MOCAP2MAV_PARAM_H

namespace params_automatic {
    //init value of radius for inspection
    static const double inspeRadius = 1;
    //linear velocity of the drone during inspection
    static const double inspeLinVel = 0.3;
    //roll upper limit to land
    static const double RollThreshold = 0.26;

    //pitch upper limit to land
    static const double PitchThreshold = 0.26;

    //Number of consecutive frames in which tracking is considered valid
    static const int    NFramesHold     = 120;

    //Number of consecutive frames in which tracking is considered not valid
    static const int    NFramesLost     = 70;

    //Number of consecutive frames in which tracking is considered valid
    //and robot is ready for compensation
    static const int    NFramesComp     = 20;

    //Platform dimension
    static const double platformLenght  = 0.7;

    //Max altitude for landing procedure
    static const double zMax            = 5;

    //Minimum altitude for landing procedure (before compensating, this value should be above the maximum platform altitude)
    static double       zMin            = 3;

    //Proportional gain times platform velocity
    static const double KpHoldV          = 0.9;

    //Proportional gain times horizontal error
    static const double KpHold           = 0.8;//1.35; //0.65; 

    //Differential gain times horizontal error
    static const double KdHold           = 0.0;

    //Integral gain times integral horizontal error
    static const double KiHold          = 0.0005; //0.0001;

    //Proportional gain for velocity tracking
    static const double KPCompV          = 0.6;

    //Integral clamping values
    static const double maxIntValue     = 5;
    static const double minIntValue     = -maxIntValue;

    //Max total PID output
    static const double maxOutput       = 10;

}

/*
Kp limit found : 1.45

oscilation time: 3.24 sec

===>
Kp = 1.16
Ki = 0.89


*/


#endif //MOCAP2MAV_PARAM_H
