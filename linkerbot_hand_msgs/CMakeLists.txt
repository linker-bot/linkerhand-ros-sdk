cmake_minimum_required(VERSION 3.8)
project(linkerbot_hand_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CommandCapability.msg"
  "msg/HandCapabilities.msg"
  "msg/HandCommandStatus.msg"
  "msg/HandCommandStatusBatch.msg"
  "msg/HandCommandTransition.msg"
  "msg/HandDeviceInfo.msg"
  "msg/HandNormalForceResultant.msg"
  "msg/HandJointCommand.msg"
  "msg/HandJointState.msg"
  "msg/HandRawCommand.msg"
  "msg/HandStallProtectionState.msg"
  "msg/HandTactileField.msg"
  "msg/HandTactileSensor.msg"
  "msg/HandTactileSensorState.msg"
  "msg/HandTactileState.msg"
  "msg/HandTelemetry.msg"
  "msg/IdentityCapability.msg"
  "msg/JointCapability.msg"
  "msg/NumericRange.msg"
  "msg/OperationCapabilities.msg"
  "msg/PercentMapping.msg"
  "msg/PhysicalAngleMapping.msg"
  "msg/SampleMetadata.msg"
  "msg/StallFieldCapability.msg"
  "msg/StallProtectionCapability.msg"
  "msg/TactileMatrix.msg"
  "msg/TelemetryCapability.msg"
  "msg/TransportCapability.msg"
  "msg/VersionField.msg"
  "srv/GetDeviceInfo.srv"
  "srv/GetStallProtection.srv"
  "srv/Recover.srv"
  "srv/SetRawValues.srv"
  "srv/SubmitCommand.srv"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

install(FILES README.md CHANGELOG.rst DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(interface_contract test/test_interfaces.py)
endif()

ament_package()
