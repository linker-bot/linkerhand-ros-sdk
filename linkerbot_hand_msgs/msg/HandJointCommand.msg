# Angle target in capability joint order. The representation is explicit.
# client_id and user_sequence form the command correlation key.
uint64 client_id
uint64 user_sequence
builtin_interfaces/Time publisher_stamp
uint8 representation
float64[] values

uint8 REPRESENTATION_PERCENT=1
uint8 REPRESENTATION_RAW=2
