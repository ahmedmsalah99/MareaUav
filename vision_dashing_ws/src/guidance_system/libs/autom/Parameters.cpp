//
// Created by andreanistico on 24/10/17.
//


#include "guidance_system/autom/Lander/Parameters.h"

Parameters::Parameters() {
    landDisarmOnGo         = false;
    altMinAbovePad           = 1.0;
    centeredConeRadiusMin = 0.60;
    compCenteredConeRadiusMin = 0.30;
    offCenterConeRadiusMin = 0.80;
    centeredConeHeight         = 5.0;
    holdDescendStep       = 0.1;
    holdAscendStep       = 0.1;
    seaCalmRollMax          = 0.7;
    seaCalmPitchMax         = 0.7;
    seaCalmHeaveSpeedMax      = 2.0;
    seaCalmRollRateMax      = 0.1;
    seaCalmPitchRateMax     = 0.1;
    seaCalmHold       = 5.0;
    xyControlMode      = 0;
    xyDirectFilterAlpha      = 0.9;
    xyDirectMaxSpeed         = 0.3;
    compLeadTime        = 0.3;
    holdHeight   = 2.0;
    holdCompBelow   = 1.7;
    holdDescendAbove   = 2.3;
    compHeightBase       = 1.5;
    compBackoffBelow        = 1.0;
    compResumeAbove     = 1.3;
    compVelMatchAbove   = 0.1;
    compHeavePeakWindow = 7.5;
    compHeightCap = 0.7;
    goHeightCap = 0.5;
    holdToCompWait       = 1.0;
    goHold = 3.0;
    goXYErrMax      = 0.05;
    goHeightBase     = 0.5;
    initStartPointHold = 2.0;
    initStartPointXYTol  = 0.5;
    initStartPointZTol = 0.15;
    slopeDuration        = 8.0;
    slopeXYKi              = 0.3;
    slopeXYIntMax          = 1.0;
    goRollMax      = 0.20;
    goPitchMax     = 0.20;
    goRollRateMax  = 0.10;
    goPitchRateMax = 0.10;
    goLatch      = 0.25;
    goVoteWindow     = 1.5;
    goVoteFraction  = 0.6;
    landDescendSpeed       = 2.5;
    goHeaveUpSpeedMax = 0.7;
    padEstVelFilterTau   = 8.0;
    padEstVelMax   = 1.0;
    padEstYawOffset  = 0.0;
    cameraToDroneCenterZSim    = 0.03;
    cameraToDroneCenterZHw     = 0.0;
    sonarToDroneCenterZSim     = 0.04;
    sonarToDroneCenterZHw      = 0.082;   // was params_automatic::OffsetUltraSensor
    touchAccelMatchTol    = 1.0;
    touchXYSpeedMax        = 1.0;
    touchContactHold      = 0.15;
    touchImpactAccel       = 5.0;
    touchImpactWindow = 0.5;
    touchConfirmHold      = 0.3;
    logPredLookback     = 0.5;
    inspecCircleRadiusGrowth = 0.5;
    altMaxAbovePadMargin = 0.1;
    xyDirectOvershootGain = 0.5;
    padEstVelSkipIfFrameGapAbove = 0.5;
    sensorSwitchHeight = 0.83;
    initWaitTicks = 20;
    holdDescendAllowedMaxXYSpeed = 0.3;
    compHeightPGain = 3.0;
    landDisarmResendInterval = 0.1;
    landDisarmResendFor = 2.0;
    touchAccelClip = 20.0;
    touchAccelAvgSamples = 5;
    simpleLandXYGain = 1.0;
    simpleLandXYTol = 0.25;
    simpleLandDescSpeedMax = 0.3;
    simpleLandDescSpeedMaxAbove = 1.6;
    simpleLandDescSpeedMin = 0.1;
    simpleLandDescSpeedMinBelow = 0.7;
    simpleLandFinalHeight = 0.25;
    simpleLandPushDown = 10.0;
    emergLandFinalDescSpeed = 1.0;
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


			if ( !param.compare("INSPEC_ENABLE") )
                inspecEnable = value;

            if ( !param.compare("LAND_DISARM_ON_GO") )
                landDisarmOnGo = value;

            if ( !param.compare("INSPEC_CIRCLE_RADIUS_M") )
                inspecCircleRadius = value;

            else if( !param.compare("INSPEC_CIRCLE_SPEED_MPS") )
                inspecCircleSpeed = value;

            else if( !param.compare("CENTERED_FRAMES_REQUIRED") )
                centeredFramesRequired = value;

            else if( !param.compare("OFF_CENTER_FRAMES_TO_LOSE_PAD") )
                offCenterFramesToLosePad = value;

            else if( !param.compare("CENTERED_CONE_RADIUS_MAX_M") )
                centeredConeRadiusMax = value;

            else if( !param.compare("COMP_CENTERED_CONE_RADIUS_MAX_M") )
                compCenteredConeRadiusMax = value;

            else if( !param.compare("OFF_CENTER_CONE_RADIUS_MAX_M") )
                offCenterConeRadiusMax = value;

            else if( !param.compare("ALT_MIN_ABOVE_PAD_M") )
                altMinAbovePad = value;

            else if( !param.compare("CENTERED_CONE_RADIUS_MIN_M") )
                centeredConeRadiusMin = value;

            else if( !param.compare("COMP_CENTERED_CONE_RADIUS_MIN_M") )
                compCenteredConeRadiusMin = value;

            else if( !param.compare("OFF_CENTER_CONE_RADIUS_MIN_M") )
                offCenterConeRadiusMin = value;

            else if( !param.compare("CENTERED_CONE_HEIGHT_M") )
                centeredConeHeight = value;

            else if( !param.compare("COMP_CENTERED_FRAMES_REQUIRED") )
                compCenteredFramesRequired = value;

            else if( !param.compare("ALT_MAX_ABOVE_PAD_M") )
                altMaxAbovePad = value;

            else if( !param.compare("ALT_MIN_WORLD_M") )
                altMinWorld = value;

            else if( !param.compare("XY_PID_KP") )
                xyPidKp = value;

            else if( !param.compare("XY_PID_KD") )
                xyPidKd = value;

            else if( !param.compare("XY_PID_KI") )
                xyPidKi = value;

            else if( !param.compare("XY_PID_INT_MAX") )
                xyPidIntMax = value;

            else if( !param.compare("XY_PID_INT_MIN") )
                xyPidIntMin = value;

            else if( !param.compare("XY_PID_OUTPUT_MAX_M") )
                xyPidOutputMax = value;

            else if( !param.compare("INIT_START_POINT_XY_DIST_M") )
                initStartPointXYDist = value;

            else if( !param.compare("INSPEC_ALTITUDE_M") )
                inspecAltitude = value;

            else if( !param.compare("HOLD_DESCEND_STEP_M") )
                holdDescendStep = value;

            else if( !param.compare("HOLD_ASCEND_STEP_M") )
                holdAscendStep = value;

            else if( !param.compare("SEA_CALM_ROLL_MAX_RAD") )
                seaCalmRollMax = value;

            else if( !param.compare("SEA_CALM_PITCH_MAX_RAD") )
                seaCalmPitchMax = value;

            else if( !param.compare("SEA_CALM_HEAVE_SPEED_MAX_MPS") )
                seaCalmHeaveSpeedMax = value;

            else if( !param.compare("SEA_CALM_ROLL_RATE_MAX_RADPS") )
                seaCalmRollRateMax = value;

            else if( !param.compare("SEA_CALM_PITCH_RATE_MAX_RADPS") )
                seaCalmPitchRateMax = value;

            else if( !param.compare("SEA_CALM_HOLD_S") )
                seaCalmHold = value;

            else if( !param.compare("XY_CONTROL_MODE") )
                xyControlMode = value;

            else if( !param.compare("XY_DIRECT_FILTER_ALPHA") )
                xyDirectFilterAlpha = value;

            else if( !param.compare("XY_DIRECT_MAX_SPEED_MPS") )
                xyDirectMaxSpeed = value;

            else if( !param.compare("COMP_LEAD_TIME_S") )
                compLeadTime = value;

            else if( !param.compare("HOLD_HEIGHT_M") )
                holdHeight = value;

            else if( !param.compare("HOLD_COMP_BELOW_M") )
                holdCompBelow = value;

            else if( !param.compare("HOLD_DESCEND_ABOVE_M") )
                holdDescendAbove = value;

            else if( !param.compare("COMP_HEIGHT_BASE_M") )
                compHeightBase = value;

            else if( !param.compare("COMP_BACKOFF_BELOW_M") )
                compBackoffBelow = value;

            else if( !param.compare("COMP_RESUME_ABOVE_M") )
                compResumeAbove = value;

            else if( !param.compare("COMP_VEL_MATCH_ABOVE_MPS") )
                compVelMatchAbove = value;

            else if( !param.compare("COMP_HEAVE_PEAK_WINDOW_S") )
                compHeavePeakWindow = value;

            else if( !param.compare("COMP_HEIGHT_CAP_M") )
                compHeightCap = value;

            else if( !param.compare("GO_HEIGHT_CAP_M") )
                goHeightCap = value;

            else if( !param.compare("HOLD_TO_COMP_WAIT_S") )
                holdToCompWait = value;

            else if( !param.compare("GO_HOLD_S") )
                goHold = value;

            else if( !param.compare("GO_XY_ERR_MAX_M") )
                goXYErrMax = value;

            else if( !param.compare("GO_HEIGHT_BASE_M") )
                goHeightBase = value;

            else if( !param.compare("INIT_START_POINT_HOLD_S") )
                initStartPointHold = value;

            else if( !param.compare("INIT_START_POINT_XY_TOL_M") )
                initStartPointXYTol = value;

            else if( !param.compare("INIT_START_POINT_Z_TOL_M") )
                initStartPointZTol = value;

            else if( !param.compare("SLOPE_DURATION_S") )
                slopeDuration = value;

            else if( !param.compare("SLOPE_XY_KI") )
                slopeXYKi = value;

            else if( !param.compare("SLOPE_XY_INT_MAX_M") )
                slopeXYIntMax = value;

            else if( !param.compare("GO_ROLL_MAX_RAD") )
                goRollMax = value;

            else if( !param.compare("GO_PITCH_MAX_RAD") )
                goPitchMax = value;

            else if( !param.compare("GO_ROLL_RATE_MAX_RADPS") )
                goRollRateMax = value;

            else if( !param.compare("GO_PITCH_RATE_MAX_RADPS") )
                goPitchRateMax = value;

            else if( !param.compare("GO_LATCH_S") )
                goLatch = value;

            else if( !param.compare("GO_VOTE_WINDOW_S") )
                goVoteWindow = value;

            else if( !param.compare("GO_VOTE_FRACTION") )
                goVoteFraction = value;

            else if( !param.compare("LAND_DESCEND_SPEED_MPS") )
                landDescendSpeed = value;

            else if( !param.compare("GO_HEAVE_UP_SPEED_MAX_MPS") )
                goHeaveUpSpeedMax = value;

            else if( !param.compare("PAD_EST_VEL_FILTER_TAU_S") )
                padEstVelFilterTau = value;

            else if( !param.compare("PAD_EST_VEL_MAX_MPS") )
                padEstVelMax = value;

            else if( !param.compare("PAD_EST_YAW_OFFSET_RAD") )
                padEstYawOffset = value;

            else if( !param.compare("CAMERA_TO_DRONE_CENTER_Z_SIM_M") )
                cameraToDroneCenterZSim = value;

            else if( !param.compare("CAMERA_TO_DRONE_CENTER_Z_HW_M") )
                cameraToDroneCenterZHw = value;

            else if( !param.compare("SONAR_TO_DRONE_CENTER_Z_SIM_M") )
                sonarToDroneCenterZSim = value;

            else if( !param.compare("SONAR_TO_DRONE_CENTER_Z_HW_M") )
                sonarToDroneCenterZHw = value;

            else if( !param.compare("TOUCH_ACCEL_MATCH_TOL_MPS2") )
                touchAccelMatchTol = value;

            else if( !param.compare("TOUCH_XY_SPEED_MAX_MPS") )
                touchXYSpeedMax = value;

            else if( !param.compare("TOUCH_CONTACT_HOLD_S") )
                touchContactHold = value;

            else if( !param.compare("TOUCH_IMPACT_ACCEL_MPS2") )
                touchImpactAccel = value;

            else if( !param.compare("TOUCH_IMPACT_WINDOW_S") )
                touchImpactWindow = value;

            else if( !param.compare("TOUCH_CONFIRM_HOLD_S") )
                touchConfirmHold = value;

            else if( !param.compare("LOG_PRED_LOOKBACK_S") )
                logPredLookback = value;

            else if( !param.compare("INSPEC_CIRCLE_RADIUS_GROWTH_M") )
                inspecCircleRadiusGrowth = value;

            else if( !param.compare("ALT_MAX_ABOVE_PAD_MARGIN_M") )
                altMaxAbovePadMargin = value;

            else if( !param.compare("XY_DIRECT_OVERSHOOT_GAIN") )
                xyDirectOvershootGain = value;

            else if( !param.compare("PAD_EST_VEL_SKIP_IF_FRAME_GAP_ABOVE_S") )
                padEstVelSkipIfFrameGapAbove = value;

            else if( !param.compare("SENSOR_SWITCH_HEIGHT_M") )
                sensorSwitchHeight = value;

            else if( !param.compare("INIT_WAIT_TICKS") )
                initWaitTicks = value;

            else if( !param.compare("HOLD_DESCEND_ALLOWED_MAX_XY_SPEED_MPS") )
                holdDescendAllowedMaxXYSpeed = value;

            else if( !param.compare("COMP_HEIGHT_P_GAIN") )
                compHeightPGain = value;

            else if( !param.compare("LAND_DISARM_RESEND_INTERVAL_S") )
                landDisarmResendInterval = value;

            else if( !param.compare("LAND_DISARM_RESEND_FOR_S") )
                landDisarmResendFor = value;

            else if( !param.compare("TOUCH_ACCEL_CLIP_MPS2") )
                touchAccelClip = value;

            else if( !param.compare("TOUCH_ACCEL_AVG_SAMPLES") )
                touchAccelAvgSamples = value;

            else if( !param.compare("SIMPLE_LAND_XY_GAIN") )
                simpleLandXYGain = value;

            else if( !param.compare("SIMPLE_LAND_XY_TOL_M") )
                simpleLandXYTol = value;

            else if( !param.compare("SIMPLE_LAND_DESC_SPEED_MAX_MPS") )
                simpleLandDescSpeedMax = value;

            else if( !param.compare("SIMPLE_LAND_DESC_SPEED_MAX_ABOVE_M") )
                simpleLandDescSpeedMaxAbove = value;

            else if( !param.compare("SIMPLE_LAND_DESC_SPEED_MIN_MPS") )
                simpleLandDescSpeedMin = value;

            else if( !param.compare("SIMPLE_LAND_DESC_SPEED_MIN_BELOW_M") )
                simpleLandDescSpeedMinBelow = value;

            else if( !param.compare("SIMPLE_LAND_FINAL_HEIGHT_M") )
                simpleLandFinalHeight = value;

            else if( !param.compare("SIMPLE_LAND_PUSH_DOWN_M") )
                simpleLandPushDown = value;

            else if( !param.compare("EMERG_LAND_FINAL_DESC_SPEED_MPS") )
                emergLandFinalDescSpeed = value;

        }
		cFile.close();



    }
    else {
        std::cerr<<"Not able to open the configuration file"<<std::endl;
    }



    }
