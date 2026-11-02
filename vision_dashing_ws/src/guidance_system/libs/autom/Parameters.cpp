//
// Created by andreanistico on 24/10/17.
//


#include "guidance_system/autom/Lander/Parameters.h"

#include <stdexcept>
#include <vector>

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
    cameraOffset       = 0.0;
    sonarOffset        = 0.082;   // was params_automatic::OffsetUltraSensor
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

namespace {

// Strip whitespace; return false for blank / comment lines.
bool parseLine(std::string line, std::string& key, double& value) {
    line.erase(std::remove_if(line.begin(), line.end(), ::isspace), line.end());
    if (line.empty() || line[0] == '#') return false;
    auto pos = line.find('=');
    if (pos == std::string::npos)
        throw std::runtime_error("malformed line (no '='): " + line);
    key = line.substr(0, pos);
    value = std::stod(line.substr(pos + 1));
    return true;
}

}  // namespace

std::map<std::string, std::function<void(double)>> Parameters::keyTable() {
    return {
        {"SEARCH", [this](double v){ search = v; }},
        {"DISARM_ON_GO", [this](double v){ disarmOnGo = v; }},
        {"INSPECTION_RADIUS", [this](double v){ inspeRadius = v; }},
        {"VELOCITY_INSPECTION", [this](double v){ inspeLinVel = v; }},
        {"FRAMES_FOR_HOLDING", [this](double v){ NFramesHold = v; }},
        {"FRAMES_FOR_LOST", [this](double v){ NFramesLost = v; }},
        {"HOLD_THRESHOLD", [this](double v){ hold_threshold = v; }},
        {"COMP_THRESHOLD", [this](double v){ comp_threshold = v; }},
        {"LOST_THRESHOLD", [this](double v){ lost_threshold = v; }},
        {"Z_MARGIN", [this](double v){ z_margin = v; }},
        {"HOLD_THRESHOLD_MIN", [this](double v){ hold_threshold_min = v; }},
        {"COMP_THRESHOLD_MIN", [this](double v){ comp_threshold_min = v; }},
        {"LOST_THRESHOLD_MIN", [this](double v){ lost_threshold_min = v; }},
        {"CONE_Z_REF", [this](double v){ cone_z_ref = v; }},
        {"FRAMES_FOR_COMP", [this](double v){ NFramesComp = v; }},
        {"MAX_ALTITUDE", [this](double v){ zMax = v; }},
        {"MIN_ALTITUDE", [this](double v){ zMin = v; }},
        {"KP_HOLD", [this](double v){ KpHold = v; }},
        {"KD_HOLD", [this](double v){ KdHold = v; }},
        {"KI_HOLD", [this](double v){ KiHold = v; }},
        {"MAX_INT_VALUE", [this](double v){ maxIntValue = v; }},
        {"MIN_INT_VALUE", [this](double v){ minIntValue = v; }},
        {"MAX_OUTPUT", [this](double v){ maxOutput = v; }},
        {"DISTANCE_FROM_CENTER", [this](double v){ distanceFromCenter = v; }},
        {"PRELAND_ALTITUDE", [this](double v){ prelandAltitude = v; }},
        {"DESC_VELOCITY", [this](double v){ descVelocity = v; }},
        {"ASCE_VELOCITY", [this](double v){ asceVelocity = v; }},
        {"ROLL_LIMIT", [this](double v){ rollLimit = v; }},
        {"PITCH_LIMIT", [this](double v){ pitchLimit = v; }},
        {"HEAVE_VEL_LIMIT", [this](double v){ heaveVelLimit = v; }},
        {"ROLL_RATE_LIMIT", [this](double v){ rollRateLimit = v; }},
        {"PITCH_RATE_LIMIT", [this](double v){ pitchRateLimit = v; }},
        {"CALM_DURATION_S", [this](double v){ calmDuration = v; }},
        {"XY_CONTROL_MODE", [this](double v){ xyControlMode = v; }},
        {"SP_XY_FILTER_ALPHA", [this](double v){ spFilterAlpha = v; }},
        {"SP_XY_MAX_VEL", [this](double v){ spMaxVelXY = v; }},
        {"REACTION_LAG_S", [this](double v){ reactionLag = v; }},
        {"STANDOFF_DISTANCE", [this](double v){ standoffDistance = v; }},
        {"STANDOFF_COMP_THLD", [this](double v){ standoffCompThld = v; }},
        {"STANDOFF_DESC_THLD", [this](double v){ standoffDescThld = v; }},
        {"COMP_HOLD_DIST", [this](double v){ compHoldDist = v; }},
        {"COMP_MIN_DIST", [this](double v){ compMinDist = v; }},
        {"COMP_RESUME_DIST", [this](double v){ compResumeDist = v; }},
        {"COMP_VEL_MATCH_THLD", [this](double v){ compVelMatchThld = v; }},
        {"COMP_HEAVE_MARGIN_WINDOW_S", [this](double v){ compHeaveMarginWindow = v; }},
        {"COMP_HEAVE_MARGIN_MAX", [this](double v){ compHeaveMarginMax = v; }},
        {"LAND_HEIGHT_MARGIN_MAX", [this](double v){ landHeightMarginMax = v; }},
        {"COMP_WAIT_TIME_S", [this](double v){ compWaitTime = v; }},
        {"COMP_TO_LAND_WAIT_TIME_S", [this](double v){ compToLandWaitTime = v; }},
        {"LAND_XY_ERR_THLD", [this](double v){ landXYErrThld = v; }},
        {"LAND_HEIGHT_THLD", [this](double v){ landHeightThld = v; }},
        {"INIT_APPROACH_WAIT_TIME_S", [this](double v){ initApproachWaitTime = v; }},
        {"INIT_APPROACH_POS_THLD", [this](double v){ initApproachPosThld = v; }},
        {"INIT_APPROACH_Z_POS_THLD", [this](double v){ initApproachZPosThld = v; }},
        {"SLOPE_DURATION_S", [this](double v){ slopeDuration = v; }},
        {"SLOPE_KI", [this](double v){ slopeKi = v; }},
        {"SLOPE_INT_MAX", [this](double v){ slopeIntMax = v; }},
        {"LAND_ROLL_LIMIT", [this](double v){ landRollLimit = v; }},
        {"LAND_PITCH_LIMIT", [this](double v){ landPitchLimit = v; }},
        {"LAND_ROLL_RATE_LIMIT", [this](double v){ landRollRateLimit = v; }},
        {"LAND_PITCH_RATE_LIMIT", [this](double v){ landPitchRateLimit = v; }},
        {"LAND_LATCH_TIME", [this](double v){ landLatchTime = v; }},
        {"LAND_VOTE_WINDOW_S", [this](double v){ landVoteWindow = v; }},
        {"LAND_VOTE_THRESHOLD", [this](double v){ landVoteThreshold = v; }},
        {"LAND_DESC_VEL", [this](double v){ landDescVel = v; }},
        {"LAND_HEAVE_UP_VEL_LIMIT", [this](double v){ landHeaveUpVelLimit = v; }},
        {"PLAT_VISION_VEL_TAU_S", [this](double v){ platVisionVelTau = v; }},
        {"PLAT_VISION_VEL_MAX", [this](double v){ platVisionVelMax = v; }},
        {"PLATFORM_YAW_OFFSET", [this](double v){ platformYawOffset = v; }},
        {"TOUCH_ACCEL_MATCH_TOL", [this](double v){ touchAccelMatchTol = v; }},
        {"TOUCH_VEL_REL_MAX", [this](double v){ touchVelRelMax = v; }},
        {"TOUCH_CONTACT_TIME", [this](double v){ touchContactTime = v; }},
        {"TOUCH_ACCEL_SPIKE", [this](double v){ touchAccelSpike = v; }},
        {"TOUCH_ACCEL_SPIKE_WINDOW", [this](double v){ touchAccelSpikeWindow = v; }},
        {"TOUCH_CONFIRM_TIME", [this](double v){ touchConfirmTime = v; }},
        {"TOUCH_PRED_LOOKBACK_S", [this](double v){ touchPredLookback = v; }},
        {"CAMERA_OFFSET", [this](double v){ cameraOffset = v; }},
        {"SONAR_OFFSET", [this](double v){ sonarOffset = v; }},
    };
}

const std::set<std::string>& Parameters::perModeKeys() {
    // Agreed per-mode set: anything that depends on the airframe/sensors or on
    // how much we trust the environment. Everything else is common.
    static const std::set<std::string> keys = {
        "MAX_ALTITUDE", "MIN_ALTITUDE",
        "ROLL_LIMIT", "PITCH_LIMIT", "HEAVE_VEL_LIMIT",
        "ROLL_RATE_LIMIT", "PITCH_RATE_LIMIT", "CALM_DURATION_S",
        "LAND_ROLL_LIMIT", "LAND_PITCH_LIMIT",
        "LAND_ROLL_RATE_LIMIT", "LAND_PITCH_RATE_LIMIT", "LAND_HEAVE_UP_VEL_LIMIT",
        "PLAT_VISION_VEL_TAU_S", "PLAT_VISION_VEL_MAX", "PLATFORM_YAW_OFFSET",
        "CAMERA_OFFSET", "SONAR_OFFSET",
        "TOUCH_ACCEL_MATCH_TOL", "TOUCH_VEL_REL_MAX", "TOUCH_CONTACT_TIME",
        "TOUCH_ACCEL_SPIKE", "TOUCH_ACCEL_SPIKE_WINDOW", "TOUCH_CONFIRM_TIME",
        "TOUCH_PRED_LOOKBACK_S",
    };
    return keys;
}

void Parameters::loadParamFile(const char *config_file) {
    std::cout << "config_file " << config_file << std::endl;
    std::ifstream cFile(config_file);
    if (!cFile.is_open()) {
        std::cerr << "Not able to open the configuration file" << std::endl;
        return;
    }
    auto table = keyTable();
    std::string line, key;
    double value;
    while (getline(cFile, line)) {
        if (!parseLine(line, key, value)) continue;
        auto it = table.find(key);
        if (it != table.end()) it->second(value);   // unknown keys ignored (lenient)
    }
}

void Parameters::loadLandingParams(const std::string& dir, const std::string& mode) {
    const std::string commonFile = dir + "/landing_common.txt";
    const std::string modeFile   = dir + "/landing_" + mode + ".txt";
    auto table = keyTable();
    const auto& perMode = perModeKeys();

    std::vector<std::string> errors;
    std::map<std::string, double> values;
    std::map<std::string, std::string> origin;   // key -> file it came from

    auto readFile = [&](const std::string& path, bool isModeFile) {
        std::cout << "config_file " << path << std::endl;
        std::ifstream f(path);
        if (!f.is_open()) { errors.push_back("cannot open " + path); return; }
        std::string line, key;
        double value;
        int lineNo = 0;
        while (getline(f, line)) {
            ++lineNo;
            const std::string where = path + ":" + std::to_string(lineNo);
            try {
                if (!parseLine(line, key, value)) continue;
            } catch (const std::exception& e) {
                errors.push_back(where + ": " + e.what());
                continue;
            }
            if (!table.count(key)) {
                errors.push_back(where + ": unknown key " + key);
                continue;
            }
            const bool keyIsPerMode = perMode.count(key) > 0;
            if (keyIsPerMode != isModeFile) {
                errors.push_back(where + ": " + key + " belongs in " +
                                 (keyIsPerMode ? modeFile : commonFile));
                continue;
            }
            if (origin.count(key)) {
                errors.push_back(where + ": duplicate key " + key +
                                 " (first in " + origin[key] + ")");
                continue;
            }
            origin[key] = where;
            values[key] = value;
        }
    };
    readFile(commonFile, false);
    readFile(modeFile, true);

    for (const auto& kv : table)
        if (!values.count(kv.first))
            errors.push_back("missing key " + kv.first + " (expected in " +
                             (perMode.count(kv.first) ? modeFile : commonFile) + ")");

    if (!errors.empty()) {
        std::string msg = "Landing parameter errors:";
        for (const auto& e : errors) msg += "\n  " + e;
        throw std::runtime_error(msg);
    }
    for (const auto& kv : values) table[kv.first](kv.second);
}
