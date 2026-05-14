Time stamp                 # [1/200 s] since EES power-on
uint8 id                   # 0->left 1->right
uint64 timestamp_485       # [1/200 s] since sw485 power-on
uint64 timestamp_satellite # [1/200 s] since satellite power-on
uint16 voltage             # [1/10 V]
int16 current              # [1/10 A] negative->discharge
uint16 charge_percent       # [%]
uint16 temperature         # [° celsius]
uint16 equalisation_cells
uint16 command_state
uint16 alarm_state          # bits: low voltage | high voltage | discharge overcurrent | charge overcurrent | low SOC | low temperature | high temperature
float32[14] cells         # [mV] (cells 0|1|2|3|9|10|11)
