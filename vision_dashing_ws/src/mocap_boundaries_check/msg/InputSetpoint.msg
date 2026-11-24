# Replaces ros2_muavp_interface/InputSetpoint (that package isn't in this repo).
# Only the fields mocap_boundaries_check uses.
uint64 timestamp      # [us]
float64[3] setpoint   # closest safe point, NED [m]
bool out_of_mocap     # drone is outside (or within the edge margin of) the mocap area
bool safety_kill
