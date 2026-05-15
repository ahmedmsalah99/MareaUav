# Mirror of the LCM exec::safety_ack struct. Field names kept identical so
# the existing C++ code does not need rewriting.
#
# Published by:   executioner_node
# Subscribed by:  safety_handler_node (also drained by automatic_node)

bool isbatterysafetytaskrunning
bool isgpssafetytaskrunning
bool iscamerasafetytaskrunning
bool issonarsafetytaskrunning
bool isgoinginsafeharbor
bool isgoingtoplatform

# Time (ms since epoch) at which the camera-gone safety task was deployed.
# SafetyHandler uses this to decide when to escalate to safe harbor.
float64 iscameragone_timestamp
