//
// Created by andreanistico on 24/10/17.
//


#include "guidance_system/autom/Lander/Parameters.h"

Parameters::Parameters() {
    disarmOnGo         = false;
    z_margin           = 1.0;
    hold_threshold_min = 0.60;
    comp_threshold_min = 0.30;
    lost_threshold_min = 0.80;
    cone_z_ref         = 5.0;
    descVelocity       = 0.1;
    asceVelocity       = 0.1;
    rollLimit          = 0.7;
    pitchLimit         = 0.7;
    heaveVelLimit      = 2.0;
    rollRateLimit      = 0.1;
    pitchRateLimit     = 0.1;
    calmDuration       = 5.0;
    xyControlMode      = 0;
    spFilterAlpha      = 0.9;
    spMaxVelXY         = 0.3;
    reactionLag        = 0.3;
    standoffDistance   = 2.0;
    standoffCompThld   = 1.7;
    standoffDescThld   = 2.3;
    compHoldDist       = 1.5;
    compMinDist        = 1.0;
    compResumeDist     = 1.3;
    compVelMatchThld   = 0.1;
    compHeaveMarginWindow = 7.5;
    compHeaveMarginMax = 0.7;
    landHeightMarginMax = 0.5;
    compWaitTime       = 1.0;
    compToLandWaitTime = 3.0;
    landXYErrThld      = 0.05;
    landHeightThld     = 0.5;
    initApproachWaitTime = 2.0;
    initApproachPosThld  = 0.5;
    initApproachZPosThld = 0.15;
    slopeDuration        = 8.0;
    slopeKi              = 0.3;
    slopeIntMax          = 1.0;
    landRollLimit      = 0.20;
    landPitchLimit     = 0.20;
    landRollRateLimit  = 0.10;
    landPitchRateLimit = 0.10;
    landLatchTime      = 0.25;
    landVoteWindow     = 1.5;
    landVoteThreshold  = 0.6;
    landDescVel       = 2.5;
    landHeaveUpVelLimit = 0.7;
    platVisionVelTau   = 8.0;
    platVisionVelMax   = 1.0;
    platformYawOffset  = 0.0;
    cameraOffsetSim    = 0.03;
    cameraOffsetHw     = 0.0;
    sonarOffsetSim     = 0.04;
    sonarOffsetHw      = 0.082;   // was params_automatic::OffsetUltraSensor
    touchAccelMatchTol    = 1.0;
    touchVelRelMax        = 1.0;
    touchContactTime      = 0.15;
    touchAccelSpike       = 5.0;
    touchAccelSpikeWindow = 0.5;
    touchConfirmTime      = 0.3;
    touchPredLookback     = 0.5;
}

void Parameters::loadConfigFile(const char *config_file) {

    loadParamFile(config_file);
    }

void Parameters::loadParamFile(const char *config_file) {

    std::cout << "config_file " << config_file << std::endl;
    std::ifstream cFile (config_file);
    if( cFile.is_open() ){

        std::string line;

        while( getline(cFile, line) ){
            line.erase( std::remove_if(line.begin(),line.end(), isspace), line.end() );

            if( line[0] == '#' || line.empty() ) continue;

            auto delimiterPos = line.find("=");
            std::string param = line.substr(0, delimiterPos);
            double value = std::stod( line.substr(delimiterPos+1) );


			if ( !param.compare("SEARCH") )
                search = value;

            if ( !param.compare("DISARM_ON_GO") )
                disarmOnGo = value;

            if ( !param.compare("INSPECTION_RADIUS") )
                inspeRadius = value;

            else if( !param.compare("VELOCITY_INSPECTION") )
                inspeLinVel = value;

            else if( !param.compare("FRAMES_FOR_HOLDING") )
                NFramesHold = value;

            else if( !param.compare("FRAMES_FOR_LOST") )
                NFramesLost = value;

            else if( !param.compare("HOLD_THRESHOLD") )
                hold_threshold = value;

            else if( !param.compare("COMP_THRESHOLD") )
                comp_threshold = value;

            else if( !param.compare("LOST_THRESHOLD") )
                lost_threshold = value;

            else if( !param.compare("Z_MARGIN") )
                z_margin = value;

            else if( !param.compare("HOLD_THRESHOLD_MIN") )
                hold_threshold_min = value;

            else if( !param.compare("COMP_THRESHOLD_MIN") )
                comp_threshold_min = value;

            else if( !param.compare("LOST_THRESHOLD_MIN") )
                lost_threshold_min = value;

            else if( !param.compare("CONE_Z_REF") )
                cone_z_ref = value;

            else if( !param.compare("FRAMES_FOR_COMP") )
                NFramesComp = value;

            else if( !param.compare("MAX_ALTITUDE") )
                zMax = value;

            else if( !param.compare("MIN_ALTITUDE") )
                zMin = value;

            else if( !param.compare("KP_HOLD") )
                KpHold = value;

            else if( !param.compare("KD_HOLD") )
                KdHold = value;

            else if( !param.compare("KI_HOLD") )
                KiHold = value;

            else if( !param.compare("MAX_INT_VALUE") )
                maxIntValue = value;

            else if( !param.compare("MIN_INT_VALUE") )
                minIntValue = value;

            else if( !param.compare("MAX_OUTPUT") )
                maxOutput = value;

            else if( !param.compare("DISTANCE_FROM_CENTER") )
                distanceFromCenter = value;

            else if( !param.compare("PRELAND_ALTITUDE") )
                prelandAltitude = value;

            else if( !param.compare("DESC_VELOCITY") )
                descVelocity = value;

            else if( !param.compare("ASCE_VELOCITY") )
                asceVelocity = value;

            else if( !param.compare("ROLL_LIMIT") )
                rollLimit = value;

            else if( !param.compare("PITCH_LIMIT") )
                pitchLimit = value;

            else if( !param.compare("HEAVE_VEL_LIMIT") )
                heaveVelLimit = value;

            else if( !param.compare("ROLL_RATE_LIMIT") )
                rollRateLimit = value;

            else if( !param.compare("PITCH_RATE_LIMIT") )
                pitchRateLimit = value;

            else if( !param.compare("CALM_DURATION_S") )
                calmDuration = value;

            else if( !param.compare("XY_CONTROL_MODE") )
                xyControlMode = value;

            else if( !param.compare("SP_XY_FILTER_ALPHA") )
                spFilterAlpha = value;

            else if( !param.compare("SP_XY_MAX_VEL") )
                spMaxVelXY = value;

            else if( !param.compare("REACTION_LAG_S") )
                reactionLag = value;

            else if( !param.compare("STANDOFF_DISTANCE") )
                standoffDistance = value;

            else if( !param.compare("STANDOFF_COMP_THLD") )
                standoffCompThld = value;

            else if( !param.compare("STANDOFF_DESC_THLD") )
                standoffDescThld = value;

            else if( !param.compare("COMP_HOLD_DIST") )
                compHoldDist = value;

            else if( !param.compare("COMP_MIN_DIST") )
                compMinDist = value;

            else if( !param.compare("COMP_RESUME_DIST") )
                compResumeDist = value;

            else if( !param.compare("COMP_VEL_MATCH_THLD") )
                compVelMatchThld = value;

            else if( !param.compare("COMP_HEAVE_MARGIN_WINDOW_S") )
                compHeaveMarginWindow = value;

            else if( !param.compare("COMP_HEAVE_MARGIN_MAX") )
                compHeaveMarginMax = value;

            else if( !param.compare("LAND_HEIGHT_MARGIN_MAX") )
                landHeightMarginMax = value;

            else if( !param.compare("COMP_WAIT_TIME_S") )
                compWaitTime = value;

            else if( !param.compare("COMP_TO_LAND_WAIT_TIME_S") )
                compToLandWaitTime = value;

            else if( !param.compare("LAND_XY_ERR_THLD") )
                landXYErrThld = value;

            else if( !param.compare("LAND_HEIGHT_THLD") )
                landHeightThld = value;

            else if( !param.compare("INIT_APPROACH_WAIT_TIME_S") )
                initApproachWaitTime = value;

            else if( !param.compare("INIT_APPROACH_POS_THLD") )
                initApproachPosThld = value;

            else if( !param.compare("INIT_APPROACH_Z_POS_THLD") )
                initApproachZPosThld = value;

            else if( !param.compare("SLOPE_DURATION_S") )
                slopeDuration = value;

            else if( !param.compare("SLOPE_KI") )
                slopeKi = value;

            else if( !param.compare("SLOPE_INT_MAX") )
                slopeIntMax = value;

            else if( !param.compare("LAND_ROLL_LIMIT") )
                landRollLimit = value;

            else if( !param.compare("LAND_PITCH_LIMIT") )
                landPitchLimit = value;

            else if( !param.compare("LAND_ROLL_RATE_LIMIT") )
                landRollRateLimit = value;

            else if( !param.compare("LAND_PITCH_RATE_LIMIT") )
                landPitchRateLimit = value;

            else if( !param.compare("LAND_LATCH_TIME") )
                landLatchTime = value;

            else if( !param.compare("LAND_VOTE_WINDOW_S") )
                landVoteWindow = value;

            else if( !param.compare("LAND_VOTE_THRESHOLD") )
                landVoteThreshold = value;

            else if( !param.compare("LAND_DESC_VEL") )
                landDescVel = value;

            else if( !param.compare("LAND_HEAVE_UP_VEL_LIMIT") )
                landHeaveUpVelLimit = value;

            else if( !param.compare("PLAT_VISION_VEL_TAU_S") )
                platVisionVelTau = value;

            else if( !param.compare("PLAT_VISION_VEL_MAX") )
                platVisionVelMax = value;

            else if( !param.compare("PLATFORM_YAW_OFFSET") )
                platformYawOffset = value;

            else if( !param.compare("CAMERA_OFFSET_SIM") )
                cameraOffsetSim = value;

            else if( !param.compare("CAMERA_OFFSET_HW") )
                cameraOffsetHw = value;

            else if( !param.compare("SONAR_OFFSET_SIM") )
                sonarOffsetSim = value;

            else if( !param.compare("SONAR_OFFSET_HW") )
                sonarOffsetHw = value;

            else if( !param.compare("TOUCH_ACCEL_MATCH_TOL") )
                touchAccelMatchTol = value;

            else if( !param.compare("TOUCH_VEL_REL_MAX") )
                touchVelRelMax = value;

            else if( !param.compare("TOUCH_CONTACT_TIME") )
                touchContactTime = value;

            else if( !param.compare("TOUCH_ACCEL_SPIKE") )
                touchAccelSpike = value;

            else if( !param.compare("TOUCH_ACCEL_SPIKE_WINDOW") )
                touchAccelSpikeWindow = value;

            else if( !param.compare("TOUCH_CONFIRM_TIME") )
                touchConfirmTime = value;

            else if( !param.compare("TOUCH_PRED_LOOKBACK_S") )
                touchPredLookback = value;

        }
		cFile.close();



    }
    else {
        std::cerr<<"Not able to open the configuration file"<<std::endl;
    }



    }
