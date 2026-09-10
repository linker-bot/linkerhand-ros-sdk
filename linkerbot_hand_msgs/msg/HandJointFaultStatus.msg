# One joint's decoded fault byte. ``raw`` is always the exact protocol byte, so
# a subscriber can re-check the bits without trusting the decoded flags.
uint8 raw
bool hand_layer_stall
bool actuator_layer_stall
bool over_current
bool over_temperature
bool actuator_abnormal
bool actuator_offline
uint8 reserved_bits
