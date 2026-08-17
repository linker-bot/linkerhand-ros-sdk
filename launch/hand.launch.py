from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("model", default_value="l6"),
        DeclareLaunchArgument("side", default_value="left"),
        DeclareLaunchArgument("transport_options_json", default_value="{}"),
        DeclareLaunchArgument("interface_name", default_value="can0"),
        DeclareLaunchArgument("interface_type", default_value="socketcan"),
        DeclareLaunchArgument("library_path", default_value=""),
        DeclareLaunchArgument("command_rate_hz", default_value="100.0"),
        DeclareLaunchArgument("auto_reconnect", default_value="false"),
        DeclareLaunchArgument("publish_rate_hz", default_value="60.0"),
        DeclareLaunchArgument("angles_hz", default_value="60.0"),
        DeclareLaunchArgument("speeds_hz", default_value="0.0"),
        DeclareLaunchArgument("torques_hz", default_value="0.0"),
        DeclareLaunchArgument("force_hz", default_value="0.0"),
        DeclareLaunchArgument("temperature_hz", default_value="1.0"),
        DeclareLaunchArgument("current_hz", default_value="10.0"),
        DeclareLaunchArgument("faults_hz", default_value="5.0"),
        DeclareLaunchArgument("voltage_hz", default_value="0.0"),
        DeclareLaunchArgument("acceleration_hz", default_value="0.0"),
        DeclareLaunchArgument("motion_time_hz", default_value="0.0"),
        DeclareLaunchArgument("diagnostic_rate", default_value="1.0"),
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("autostart", default_value="true"),
    ]
    node = LifecycleNode(
        package="linkerbot_hand_ros",
        executable="hand_node",
        name="linkerbot_hand",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        autostart=LaunchConfiguration("autostart"),
        parameters=[
            {
                "model": LaunchConfiguration("model"),
                "side": LaunchConfiguration("side"),
                "transport_options_json": ParameterValue(
                    LaunchConfiguration("transport_options_json"), value_type=str
                ),
                "interface_name": LaunchConfiguration("interface_name"),
                "interface_type": LaunchConfiguration("interface_type"),
                "library_path": LaunchConfiguration("library_path"),
                "command_rate_hz": ParameterValue(
                    LaunchConfiguration("command_rate_hz"), value_type=float
                ),
                "auto_reconnect": ParameterValue(
                    LaunchConfiguration("auto_reconnect"), value_type=bool
                ),
                "publish_rate_hz": ParameterValue(
                    LaunchConfiguration("publish_rate_hz"), value_type=float
                ),
                "angles_hz": ParameterValue(
                    LaunchConfiguration("angles_hz"), value_type=float
                ),
                "speeds_hz": ParameterValue(
                    LaunchConfiguration("speeds_hz"), value_type=float
                ),
                "torques_hz": ParameterValue(
                    LaunchConfiguration("torques_hz"), value_type=float
                ),
                "force_hz": ParameterValue(
                    LaunchConfiguration("force_hz"), value_type=float
                ),
                "temperature_hz": ParameterValue(
                    LaunchConfiguration("temperature_hz"), value_type=float
                ),
                "current_hz": ParameterValue(
                    LaunchConfiguration("current_hz"), value_type=float
                ),
                "faults_hz": ParameterValue(
                    LaunchConfiguration("faults_hz"), value_type=float
                ),
                "voltage_hz": ParameterValue(
                    LaunchConfiguration("voltage_hz"), value_type=float
                ),
                "acceleration_hz": ParameterValue(
                    LaunchConfiguration("acceleration_hz"), value_type=float
                ),
                "motion_time_hz": ParameterValue(
                    LaunchConfiguration("motion_time_hz"), value_type=float
                ),
                "diagnostic_rate": ParameterValue(
                    LaunchConfiguration("diagnostic_rate"), value_type=float
                ),
            }
        ],
    )
    return LaunchDescription([*arguments, node])
