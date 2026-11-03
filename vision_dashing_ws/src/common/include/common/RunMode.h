#ifndef COMMON_RUNMODE_H
#define COMMON_RUNMODE_H

#include <cstdlib>
#include <stdexcept>
#include <string>

namespace common {

// Which environment the stack is running in, from the M2M_MODE environment
// variable:
//   0 = simulation (Gazebo, boat motion from the recorded bag)
//   1 = motion-capture lab (real drone, OptiTrack)
//   2 = real hardware at sea
//   3 = mocap in sim: Gazebo with the sim sensors, but flown as in the mocap
//       lab (static pad, poses from the plugin's fake mocap, mocap room limits)
enum class RunMode { Sim = 0, Mocap = 1, Hw = 2, SimMocap = 3 };

// Reads M2M_MODE. Throws if it is unset or not 0/1/2/3, so a missing or mistyped
// mode stops the node at startup instead of silently defaulting to one.
inline RunMode runModeFromEnv() {
    const char* v = std::getenv("M2M_MODE");
    const std::string s = v ? v : "";
    if (s == "0") return RunMode::Sim;
    if (s == "1") return RunMode::Mocap;
    if (s == "2") return RunMode::Hw;
    if (s == "3") return RunMode::SimMocap;
    throw std::runtime_error(
        "M2M_MODE must be set to 0 (sim), 1 (mocap), 2 (hw) or 3 (mocap in sim); got '" + s + "'");
}

// True when Gazebo is running (modes 0 and 3): sim sensors, sim camera, and
// the simulation-only ground-truth topics exist.
inline bool runModeIsGazebo(RunMode m) {
    return m == RunMode::Sim || m == RunMode::SimMocap;
}

inline const char* runModeName(RunMode m) {
    switch (m) {
        case RunMode::Sim:   return "sim";
        case RunMode::Mocap: return "mocap";
        case RunMode::Hw:    return "hw";
        case RunMode::SimMocap: return "sim_mocap";
    }
    return "unknown";
}

}  // namespace common

#endif  // COMMON_RUNMODE_H
