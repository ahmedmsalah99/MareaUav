//
// Created by andreanistico on 24/10/17.
//


#include "guidance_system/autom/Lander/Parameters.h"
#include "common/RunMode.h"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <map>
#include <stdexcept>
#include <vector>

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
    cameraToDroneCenterZ = 0.0;
    sonarToDroneCenterZ = 0.082;
    touchAccelMatchTol    = 1.0;
    touchXYSpeedMax        = 1.0;
    touchContactHold      = 0.15;
    touchImpactAccel       = 5.0;
    touchImpactWindow = 0.5;
    touchConfirmHold      = 0.3;
    logPredLookback     = 0.5;
    goForceIfSonarBelow = 0.2;
    simpleLandFinalHeight = 0.25;
    holdToCompTimeout = 60.0;
    abortAltitude = 6.0;
    abortArrivedTol = 0.3;
    geofenceEnable = false;
    geofenceXMin = -1e9;
    geofenceXMax =  1e9;
    geofenceYMin = -1e9;
    geofenceYMax =  1e9;
    altMaxWorld  =  1e9;
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


            setKey(param, value);   // unknown keys are ignored here (lenient)
        }
		cFile.close();



    }
    else {
        std::cerr<<"Not able to open the configuration file"<<std::endl;
    }



    }

bool Parameters::setKey(const std::string& param, double value) {
			if ( !param.compare("INSPEC_ENABLE") )
                inspecEnable = value;

            else if ( !param.compare("LAND_DISARM_ON_GO") )
                landDisarmOnGo = value;

            else if ( !param.compare("INSPEC_CIRCLE_RADIUS_M") )
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

            else if( !param.compare("GEOFENCE_ENABLE") )
                geofenceEnable = value;

            else if( !param.compare("GEOFENCE_X_MIN_M") )
                geofenceXMin = value;

            else if( !param.compare("GEOFENCE_X_MAX_M") )
                geofenceXMax = value;

            else if( !param.compare("GEOFENCE_Y_MIN_M") )
                geofenceYMin = value;

            else if( !param.compare("GEOFENCE_Y_MAX_M") )
                geofenceYMax = value;

            else if( !param.compare("ALT_MAX_WORLD_M") )
                altMaxWorld = value;

            else if( !param.compare("HOLD_TO_COMP_TIMEOUT_S") )
                holdToCompTimeout = value;

            else if( !param.compare("ABORT_ALTITUDE_M") )
                abortAltitude = value;

            else if( !param.compare("ABORT_ARRIVED_TOL_M") )
                abortArrivedTol = value;

            else if( !param.compare("SIMPLE_LAND_FINAL_HEIGHT_M") )
                simpleLandFinalHeight = value;

            else if( !param.compare("GO_FORCE_IF_SONAR_BELOW_M") )
                goForceIfSonarBelow = value;

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

            else if( !param.compare("CAMERA_TO_DRONE_CENTER_Z_M") )
                cameraToDroneCenterZ = value;

            else if( !param.compare("SONAR_TO_DRONE_CENTER_Z_M") )
                sonarToDroneCenterZ = value;

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



            else if( !param.compare("EMERG_LAND_FINAL_DESC_SPEED_MPS") )
                emergLandFinalDescSpeed = value;

            else
                return false;
            return true;
}

const std::set<std::string>& Parameters::perModeKeys() {
    // Keys that differ between sim, mocap and real hardware. They must be in
    // landing_<mode>.yaml and must NOT be in landing_common.yaml.
    static const std::set<std::string> keys = {
        "ALT_MAX_ABOVE_PAD_M",
        "ALT_MIN_WORLD_M",
        "PAD_EST_VEL_FILTER_TAU_S",
        "PAD_EST_VEL_MAX_MPS",
        "PAD_EST_YAW_OFFSET_RAD",
        "CAMERA_TO_DRONE_CENTER_Z_M",
        "SONAR_TO_DRONE_CENTER_Z_M",
        "SENSOR_SWITCH_HEIGHT_M",
        "SEA_CALM_ROLL_MAX_RAD",
        "SEA_CALM_PITCH_MAX_RAD",
        "SEA_CALM_ROLL_RATE_MAX_RADPS",
        "SEA_CALM_PITCH_RATE_MAX_RADPS",
        "SEA_CALM_HEAVE_SPEED_MAX_MPS",
        "SEA_CALM_HOLD_S",
        "GO_ROLL_MAX_RAD",
        "GO_PITCH_MAX_RAD",
        "GO_ROLL_RATE_MAX_RADPS",
        "GO_PITCH_RATE_MAX_RADPS",
        "GO_HEAVE_UP_SPEED_MAX_MPS",
        "LAND_DISARM_RESEND_INTERVAL_S",
        "LAND_DISARM_RESEND_FOR_S",
        "TOUCH_ACCEL_MATCH_TOL_MPS2",
        "TOUCH_XY_SPEED_MAX_MPS",
        "TOUCH_CONTACT_HOLD_S",
        "TOUCH_IMPACT_ACCEL_MPS2",
        "TOUCH_IMPACT_WINDOW_S",
        "TOUCH_CONFIRM_HOLD_S",
        "TOUCH_ACCEL_CLIP_MPS2",
        "TOUCH_ACCEL_AVG_SAMPLES",
        "EMERG_LAND_FINAL_DESC_SPEED_MPS",
        // Airframe tuning and room size (mocap is confined).
        "HOLD_TO_COMP_TIMEOUT_S",
        "ABORT_ALTITUDE_M",
        "GEOFENCE_ENABLE",
        "GEOFENCE_X_MIN_M",
        "GEOFENCE_X_MAX_M",
        "GEOFENCE_Y_MIN_M",
        "GEOFENCE_Y_MAX_M",
        "ALT_MAX_WORLD_M",
        "INSPEC_ENABLE",
        "INSPEC_CIRCLE_RADIUS_M",
        "INSPEC_CIRCLE_SPEED_MPS",
        "INSPEC_CIRCLE_RADIUS_GROWTH_M",
        "INSPEC_ALTITUDE_M",
        "CENTERED_CONE_HEIGHT_M",
        "CENTERED_CONE_RADIUS_MIN_M",
        "CENTERED_CONE_RADIUS_MAX_M",
        "COMP_CENTERED_CONE_RADIUS_MIN_M",
        "COMP_CENTERED_CONE_RADIUS_MAX_M",
        "OFF_CENTER_CONE_RADIUS_MIN_M",
        "OFF_CENTER_CONE_RADIUS_MAX_M",
        "XY_PID_KP",
        "XY_PID_KI",
        "XY_PID_KD",
        "INIT_START_POINT_XY_DIST_M",
        "INIT_START_POINT_XY_TOL_M",
        "INIT_START_POINT_Z_TOL_M",
        "SLOPE_XY_KI",
        "HOLD_HEIGHT_M",
        "HOLD_DESCEND_ABOVE_M",
        "HOLD_COMP_BELOW_M",
        "COMP_HEIGHT_P_GAIN",
        "COMP_LEAD_TIME_S",
        "SIMPLE_LAND_XY_GAIN",
        "SIMPLE_LAND_DESC_SPEED_MAX_ABOVE_M",
    };
    return keys;
}

void Parameters::loadLanding() {
    const std::string dir = ament_index_cpp::get_package_share_directory("guidance_system") + "/params";
    std::string mode;
    switch (common::runModeFromEnv()) {
        case common::RunMode::Sim:   mode = "sim";   break;
        case common::RunMode::Mocap: mode = "mocap"; break;
        case common::RunMode::Hw:    mode = "hw";    break;
        case common::RunMode::SimMocap: mode = "sim_mocap"; break;
    }
    loadLanding(dir, mode);
}

void Parameters::loadLanding(const std::string& dir, const std::string& mode) {
    const std::string commonFile = dir + "/landing_common.yaml";
    const std::string modeFile   = dir + "/landing_" + mode + ".yaml";
    const auto& perMode = perModeKeys();

    // Collect every problem first, then fail once with the full list.
    std::vector<std::string> errors;
    std::map<std::string, double> values;
    std::map<std::string, std::string> where;   // key -> "file:line" it was set at

    Parameters probe;   // scratch object: only used to ask setKey() whether a key exists

    // File layout: top-level sections (only for grouping) holding KEY: value.
    auto readFile = [&](const std::string& path, bool isModeFile) {
        std::cout << "config_file " << path << std::endl;
        YAML::Node root;
        try { root = YAML::LoadFile(path); }
        catch (const std::exception& e) { errors.push_back("cannot read " + path + ": " + e.what()); return; }
        if (!root.IsMap()) { errors.push_back(path + ": top level must be sections (a map)"); return; }
        for (const auto& section : root) {
            const std::string sectionName = section.first.as<std::string>();
            if (!section.second.IsMap()) {
                errors.push_back(path + ":" + std::to_string(section.first.Mark().line + 1) + ": '" + sectionName +
                                 "' must be a section holding KEY: value entries");
                continue;
            }
            for (const auto& entry : section.second) {
                const std::string key = entry.first.as<std::string>();
                const std::string at  = path + ":" + std::to_string(entry.first.Mark().line + 1);
                double value;
                try { value = entry.second.as<double>(); }
                catch (const std::exception&) { errors.push_back(at + ": bad number for " + key); continue; }

                if (!probe.setKey(key, value)) { errors.push_back(at + ": unknown key " + key); continue; }
                const bool keyIsPerMode = perMode.count(key) > 0;
                if (keyIsPerMode != isModeFile) {
                    errors.push_back(at + ": " + key + " belongs in " + (keyIsPerMode ? modeFile : commonFile));
                    continue;
                }
                if (where.count(key)) {
                    errors.push_back(at + ": duplicate key " + key + " (already set at " + where[key] + ")");
                    continue;
                }
                where[key]  = at;
                values[key] = value;
            }
        }
    };
    readFile(commonFile, false);
    readFile(modeFile, true);

    // Missing: every key setKey() knows must be set exactly once, per-mode
    // keys in the mode file and the rest in landing_common.yaml.
    for (const auto& key : allKeys())
        if (!values.count(key))
            errors.push_back("missing key " + key + " (expected in " +
                             (perMode.count(key) ? modeFile : commonFile) + ")");

    if (!errors.empty()) {
        std::string msg = "Landing parameter errors:";
        for (const auto& e : errors) msg += "\n  " + e;
        throw std::runtime_error(msg);
    }
    for (const auto& kv : values) setKey(kv.first, kv.second);
}

const std::vector<std::string>& Parameters::allKeys() {
    // Every key setKey() accepts (keep in sync with setKey()).
    static const std::vector<std::string> keys = {
        "INSPEC_ENABLE",
        "LAND_DISARM_ON_GO",
        "INSPEC_CIRCLE_RADIUS_M",
        "INSPEC_CIRCLE_SPEED_MPS",
        "CENTERED_FRAMES_REQUIRED",
        "OFF_CENTER_FRAMES_TO_LOSE_PAD",
        "CENTERED_CONE_RADIUS_MAX_M",
        "COMP_CENTERED_CONE_RADIUS_MAX_M",
        "OFF_CENTER_CONE_RADIUS_MAX_M",
        "ALT_MIN_ABOVE_PAD_M",
        "CENTERED_CONE_RADIUS_MIN_M",
        "COMP_CENTERED_CONE_RADIUS_MIN_M",
        "OFF_CENTER_CONE_RADIUS_MIN_M",
        "CENTERED_CONE_HEIGHT_M",
        "COMP_CENTERED_FRAMES_REQUIRED",
        "ALT_MAX_ABOVE_PAD_M",
        "ALT_MIN_WORLD_M",
        "XY_PID_KP",
        "XY_PID_KD",
        "XY_PID_KI",
        "XY_PID_INT_MAX",
        "XY_PID_INT_MIN",
        "XY_PID_OUTPUT_MAX_M",
        "INIT_START_POINT_XY_DIST_M",
        "INSPEC_ALTITUDE_M",
        "HOLD_DESCEND_STEP_M",
        "HOLD_ASCEND_STEP_M",
        "SEA_CALM_ROLL_MAX_RAD",
        "SEA_CALM_PITCH_MAX_RAD",
        "SEA_CALM_HEAVE_SPEED_MAX_MPS",
        "SEA_CALM_ROLL_RATE_MAX_RADPS",
        "SEA_CALM_PITCH_RATE_MAX_RADPS",
        "SEA_CALM_HOLD_S",
        "XY_CONTROL_MODE",
        "XY_DIRECT_FILTER_ALPHA",
        "XY_DIRECT_MAX_SPEED_MPS",
        "COMP_LEAD_TIME_S",
        "HOLD_HEIGHT_M",
        "HOLD_COMP_BELOW_M",
        "HOLD_DESCEND_ABOVE_M",
        "COMP_HEIGHT_BASE_M",
        "COMP_BACKOFF_BELOW_M",
        "COMP_RESUME_ABOVE_M",
        "COMP_VEL_MATCH_ABOVE_MPS",
        "COMP_HEAVE_PEAK_WINDOW_S",
        "COMP_HEIGHT_CAP_M",
        "GO_HEIGHT_CAP_M",
        "HOLD_TO_COMP_WAIT_S",
        "GO_HOLD_S",
        "GO_XY_ERR_MAX_M",
        "GO_HEIGHT_BASE_M",
        "INIT_START_POINT_HOLD_S",
        "INIT_START_POINT_XY_TOL_M",
        "INIT_START_POINT_Z_TOL_M",
        "SLOPE_DURATION_S",
        "SLOPE_XY_KI",
        "SLOPE_XY_INT_MAX_M",
        "GO_ROLL_MAX_RAD",
        "GO_PITCH_MAX_RAD",
        "GO_ROLL_RATE_MAX_RADPS",
        "GO_PITCH_RATE_MAX_RADPS",
        "GEOFENCE_ENABLE",
        "GEOFENCE_X_MIN_M",
        "GEOFENCE_X_MAX_M",
        "GEOFENCE_Y_MIN_M",
        "GEOFENCE_Y_MAX_M",
        "ALT_MAX_WORLD_M",
        "HOLD_TO_COMP_TIMEOUT_S",
        "ABORT_ALTITUDE_M",
        "ABORT_ARRIVED_TOL_M",
        "SIMPLE_LAND_FINAL_HEIGHT_M",
        "GO_FORCE_IF_SONAR_BELOW_M",
        "GO_LATCH_S",
        "GO_VOTE_WINDOW_S",
        "GO_VOTE_FRACTION",
        "LAND_DESCEND_SPEED_MPS",
        "GO_HEAVE_UP_SPEED_MAX_MPS",
        "PAD_EST_VEL_FILTER_TAU_S",
        "PAD_EST_VEL_MAX_MPS",
        "PAD_EST_YAW_OFFSET_RAD",
        "CAMERA_TO_DRONE_CENTER_Z_M",
        "SONAR_TO_DRONE_CENTER_Z_M",
        "TOUCH_ACCEL_MATCH_TOL_MPS2",
        "TOUCH_XY_SPEED_MAX_MPS",
        "TOUCH_CONTACT_HOLD_S",
        "TOUCH_IMPACT_ACCEL_MPS2",
        "TOUCH_IMPACT_WINDOW_S",
        "TOUCH_CONFIRM_HOLD_S",
        "LOG_PRED_LOOKBACK_S",
        "INSPEC_CIRCLE_RADIUS_GROWTH_M",
        "ALT_MAX_ABOVE_PAD_MARGIN_M",
        "XY_DIRECT_OVERSHOOT_GAIN",
        "PAD_EST_VEL_SKIP_IF_FRAME_GAP_ABOVE_S",
        "SENSOR_SWITCH_HEIGHT_M",
        "INIT_WAIT_TICKS",
        "HOLD_DESCEND_ALLOWED_MAX_XY_SPEED_MPS",
        "COMP_HEIGHT_P_GAIN",
        "LAND_DISARM_RESEND_INTERVAL_S",
        "LAND_DISARM_RESEND_FOR_S",
        "TOUCH_ACCEL_CLIP_MPS2",
        "TOUCH_ACCEL_AVG_SAMPLES",
        "SIMPLE_LAND_XY_GAIN",
        "SIMPLE_LAND_XY_TOL_M",
        "SIMPLE_LAND_DESC_SPEED_MAX_MPS",
        "SIMPLE_LAND_DESC_SPEED_MAX_ABOVE_M",
        "SIMPLE_LAND_DESC_SPEED_MIN_MPS",
        "SIMPLE_LAND_DESC_SPEED_MIN_BELOW_M",
        "EMERG_LAND_FINAL_DESC_SPEED_MPS"
    };
    return keys;
}
