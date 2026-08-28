uint64 user_sequence
uint64 command_id
string family
uint8 stage
uint8 ack_support
uint32 sent_frames
string error_code
string error_message
HandCommandTransition[] transitions

uint8 STAGE_ACCEPTED=1
uint8 STAGE_DISPATCHED=2
uint8 STAGE_ACKNOWLEDGED=3
uint8 STAGE_FAILED=4
uint8 STAGE_SUPERSEDED=5

uint8 ACK_UNSUPPORTED=0
uint8 ACK_SUPPORTED=1
