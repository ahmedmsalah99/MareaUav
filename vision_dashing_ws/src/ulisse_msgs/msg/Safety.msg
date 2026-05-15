# Mirror of the LCM exec::safety struct. Field names kept identical so the
# existing C++ code (SafetyHandler internals, Executioner handlers) does not
# need rewriting.
#
# Published by:   safety_handler_node
# Subscribed by:  executioner_node

bool isbatterygone
bool iscameragone
bool isgpsgone
bool issonargone
bool goingsafeharbor
bool isgoingplatform

# Variable parameters consumed by Executioner's safety-task handlers.
# Index meaning depends on which task is being deployed (see SafetyHandler
# for the assignments). The original LCM struct sizes this at 8 floats.
float64[8] params

# Target coordinates (x, y, z) for tasks that move the drone somewhere
# specific: safe_harbor or platform rendezvous.
float64[3] target_coord
