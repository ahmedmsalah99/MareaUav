#ifndef GUIDANCE_SYSTEM_SIMPLE_LANDING_HPP
#define GUIDANCE_SYSTEM_SIMPLE_LANDING_HPP

#include <chrono>
#include <cmath>
#include <iostream>

#include "common/MavState.h"
#include "guidance_system/autom/Lander/Parameters.h"

// Simple landing (no pad, no vision), shared by the LAND task's basic mode
// (Land::simpleLanding()) and the Lander's abort (Lander::abortReturn()).
// One control step towards the ground point (x_target, y_target, h):
//   1. fly over the point (XY velocity = SIMPLE_LAND_XY_GAIN * error), no descent
//      until within SIMPLE_LAND_XY_TOL_M on X and Y;
//   2. descend at a height-dependent speed (SIMPLE_LAND_DESC_SPEED_* profile,
//      velocity mode all the way down);
//   3. within SIMPLE_LAND_FINAL_HEIGHT_M of the ground, stop and request a
//      disarm (re-sent every LAND_DISARM_RESEND_INTERVAL_S for
//      LAND_DISARM_RESEND_FOR_S): the drone drops the last few centimetres.
namespace simple_landing {

// Descent speed for a height dz above the target: max above tmax, min below
// tmin, linear in between.
inline double descendRate(double dz, double drate_max, double drate_min, double tmax, double tmin)
{
    if (dz > tmax)      return drate_max;
    else if (dz < tmin) return drate_min;
    const double m = (drate_max - drate_min) / (tmax - tmin);
    const double q = drate_min - m * tmin;
    return m * dz + q;
}

// Re-sends the disarm request at a fixed interval for a bounded time (the
// command goes out best-effort, so one lost message must not leave the drone
// armed on the ground).
class DisarmResender {
public:
    // True on the ticks where a disarm request should be raised.
    bool tick(const Parameters& p) {
        const auto now = std::chrono::steady_clock::now();
        if (!started_) { started_ = true; start_ = now; last_ = now; return true; }
        const double elapsed = std::chrono::duration<double>(now - start_).count();
        const double since   = std::chrono::duration<double>(now - last_).count();
        if (elapsed > p.landDisarmResendFor || since < p.landDisarmResendInterval) return false;
        last_ = now;
        return true;
    }
    bool logged = false;   // the "disarming" line is printed once per landing

private:
    bool started_ = false;
    std::chrono::steady_clock::time_point start_, last_;
};

// One step. Returns true on the ticks where the caller must raise a disarm
// request (the drone is within SIMPLE_LAND_FINAL_HEIGHT_M of the ground).
inline bool step(const MavState& state, double x_target, double y_target, double h,
                 const Parameters& p, MavState& comm, DisarmResender& disarm)
{
    const double dx = -state.getX() + x_target;
    const double dy = -state.getY() + y_target;
    const double dz = -state.getZ() + h;

    // Close enough to the ground: stop commanding a descent and cut the motors.
    if (std::fabs(dz) < p.simpleLandFinalHeight) {
        comm.setType(MavState::VELOCITY);
        comm.setVx(0.0);
        comm.setVy(0.0);
        comm.setVz(0.0);
        const bool req = disarm.tick(p);
        if (req && !disarm.logged) {
            disarm.logged = true;
            std::cout << "[SIMPLE LAND] " << std::fabs(dz) << " m above the ground (< "
                      << p.simpleLandFinalHeight << "): disarming" << std::endl;
        }
        return req;
    }

    // Be sure that we are on top of the target
    comm.setVx(p.simpleLandXYGain * dx);
    comm.setVy(p.simpleLandXYGain * dy);
    comm.setType(MavState::VELOCITY);
    //TODO: add security checks on vz
    if (std::fabs(dx) <= p.simpleLandXYTol && std::fabs(dy) <= p.simpleLandXYTol) {
        //Descending is safe, is it?
        const double desc = descendRate(std::fabs(dz), p.simpleLandDescSpeedMax, p.simpleLandDescSpeedMin,
                                        p.simpleLandDescSpeedMaxAbove, p.simpleLandDescSpeedMinBelow);
        comm.setVz(-desc);
    }
    else comm.setVz(0);
    return false;
}

}  // namespace simple_landing

#endif  // GUIDANCE_SYSTEM_SIMPLE_LANDING_HPP
