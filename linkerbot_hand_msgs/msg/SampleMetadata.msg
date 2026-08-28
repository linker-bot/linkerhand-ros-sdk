# The SDK monotonic collection time is projected into the active ROS clock.
builtin_interfaces/Time collected_at
uint64 sequence
float64 age_s
bool valid
bool stale
uint64 connection_generation
uint8 origin
bool within_declared_range

uint8 ORIGIN_NONE=0
uint8 ORIGIN_DEVICE_RESPONSE=1
uint8 ORIGIN_DEVICE_REPORT=2
uint8 ORIGIN_DEVICE_CONFIG_RESPONSE=3
uint8 ORIGIN_DEVICE_ECHO=4
uint8 ORIGIN_COMMAND_CACHE=5
