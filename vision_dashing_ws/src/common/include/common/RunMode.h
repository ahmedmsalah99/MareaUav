#ifndef COMMON_RUNMODE_H
#define COMMON_RUNMODE_H

#include <cstdlib>
#include <stdexcept>
#include <string>

namespace common {

// Which environment the stack is running in, from the M2M_MODE environment
// variable: 0 = simulation, 1 = motion-capture lab, 2 = real hardware.
enum class RunMode { Sim = 0, Mocap = 1, Hw = 2 };

// Reads M2M_MODE. Throws if it is unset or not 0/1/2, so a missing or mistyped
// mode stops the node at startup instead of silently defaulting to one.
inline RunMode runModeFromEnv() {
    const char* v = std::getenv("M2M_MODE");
    const std::string s = v ? v : "";
    if (s == "0") return RunMode::Sim;
    if (s == "1") return RunMode::Mocap;
    if (s == "2") return RunMode::Hw;
    throw std::runtime_error(
        "M2M_MODE must be set to 0 (sim), 1 (mocap) or 2 (hw); got '" + s + "'");
}

inline const char* runModeName(RunMode m) {
    switch (m) {
        case RunMode::Sim:   return "sim";
        case RunMode::Mocap: return "mocap";
        case RunMode::Hw:    return "hw";
    }
    return "unknown";
}

}  // namespace common

#endif  // COMMON_RUNMODE_H
