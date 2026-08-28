# linkerbot_hand_msgs

`linkerbot_hand_msgs` contains transport-neutral ROS 2 interfaces for the
Linkerbot Hand SDK 1.0 contract. It has no dependency on `rclpy`, the Python
Hand SDK, or a specific hand model.

The value representation is always explicit:

- `HandJointCommand` selects `REPRESENTATION_PERCENT` or
  `REPRESENTATION_RAW`.
- `HandJointState` keeps protocol raw, percent, and optional physical radians
  in separate fields with validity masks.
- `HandTelemetry` identifies configured and measured channels through its
  semantic and capability metadata.
- `HandTactileState` represents one complete multi-finger tactile transaction.
- `HandCommandStatus` distinguishes accepted, dispatched, acknowledged,
  failed, and superseded commands. A non-zero `client_id` combined with
  `user_sequence` identifies one publisher's command without colliding with
  another publisher, while publisher and driver-receive timestamps expose the
  ROS admission latency.

Every state sample carries `SampleMetadata`: collection time, per-channel
sequence, age, validity, staleness, connection generation, origin, and range
status. A disabled SDK polling channel therefore produces no synthetic ROS
sample.

`ORIGIN_DEVICE_COMMAND_FEEDBACK` means current state returned in response to a
command; it is neither a target echo nor an acknowledgement stage.

`HandCapabilities` is the ROS projection of the SDK capability object. ROS
clients must use it instead of maintaining a separate model specification.
Its `firmware_unsupported_telemetry` field reports optional queries disabled
for the current connection generation while retaining their protocol metadata.
`OperationCapabilities.save_parameters` explicitly gates the corresponding
management service.

中文约定：普通 `raw` 字段始终表示设备协议原值，不做物理单位换算；`percent` 仅表示
SDK capability 定义的 `0..100` 映射。屈伸轴为 `0=弯曲，100=伸直`，侧摆和旋转轴的
两端方向由对应 `PercentMapping` 明确给出。
