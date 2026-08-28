# Protocol-native target or configuration values in capability channel order.
# client_id and user_sequence form the command correlation key.
uint64 client_id
uint64 user_sequence
builtin_interfaces/Time publisher_stamp
float64[] raw
