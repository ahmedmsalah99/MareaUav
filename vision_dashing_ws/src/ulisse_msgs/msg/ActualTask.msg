# Mirror of the LCM exec::actual_task struct. Field names kept identical so
# the existing C++ code does not need rewriting.
#
# Published by:   executioner_node
# Subscribed by:  automatic_node, safety_handler_node, bridge_node
#                 (the bridge consumes this only to derive /drone_state for the
#                 RPi-side fsm_raspicam package)

# Task action. Values come from the existing C++ enum common::actions::*
# (LAND, SEARCH, EMERG_LAND, MOVE, etc.). Kept as int8 to match the LCM type;
# no enum constants are duplicated into this msg -- use the C++ enum directly
# at the call sites.
int8 action
int8 id
# Target name, e.g. "platform", "safe_harbor".
string target

# Target coordinates for movement-style tasks (MOVE etc.).
# NOTE: in automatic_node, x and y are overwritten with the live platform
# pose when action == MOVE && target == "platform" && env_sim.
float64 x
float64 y
float64 z

float64 yaw
float64[5] params