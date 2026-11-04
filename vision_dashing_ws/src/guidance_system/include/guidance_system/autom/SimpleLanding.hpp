#ifndef GUIDANCE_SYSTEM_SIMPLE_LANDING_HPP
#define GUIDANCE_SYSTEM_SIMPLE_LANDING_HPP

#include <cmath>

#include "common/MavState.h"
#include "guidance_system/autom/Lander/Parameters.h"

// Simple landing (no pad, no vision), shared by the LAND task's basic mode
// (Land::simpleLanding()) and the Lander's abort (Lander::abortReturn()).
// One control step towards the ground point (x_target, y_target, h):
//   1. fly over the point (XY velocity = SIMPLE_LAND_XY_GAIN * error), no descent
//      until within SIMPLE_LAND_XY_TOL_M on X and Y;
//   2. descend at a height-dependent speed (SIMPLE_LAND_DESC_SPEED_* profile);
//   3. below SIMPLE_LAND_FINAL_HEIGHT_M, switch to a position setpoint
//      SIMPLE_LAND_PUSH_DOWN_M under the ground so the drone keeps pressing
//      down until PX4 detects the landing.
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

inline void step(const MavState& state, double x_target, double y_target, double h,
                 const Parameters& p, MavState& comm)
{
    const double dx = -state.getX() + x_target;
    const double dy = -state.getY() + y_target;
    const double dz = -state.getZ() + h;

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

        if (std::fabs(dz) < p.simpleLandFinalHeight) {
            comm.setX(x_target);
            comm.setY(y_target);
            comm.setZ(h - p.simpleLandPushDown);
            comm.setVx(NAN);   // position-only: don't leak the velocity-mode Vx/Vy above
            comm.setVy(NAN);
            comm.setType(MavState::POSITION);
        }
    }
    else comm.setVz(0);
}

}  // namespace simple_landing

#endif  // GUIDANCE_SYSTEM_SIMPLE_LANDING_HPP
