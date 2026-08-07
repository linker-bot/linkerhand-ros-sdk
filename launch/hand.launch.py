from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("model", default_value="l6"),
        DeclareLaunchArgument("side", default_value="left"),
        DeclareLaunchArgument("interface_name", default_value="can0"),
        DeclareLaunchArgument("interface_type", default_value="socketcan"),
        DeclareLaunchArgument("library_path", default_value=""),
        DeclareLaunchArgument("poll_rate", default_value="30.0"),
        DeclareLaunchArgument("timeout_ms", default_value="100.0"),
        DeclareLaunchArgument("optional_state_rate", default_value="1.0"),
    ]
    node = Node(
        package="linkerbot_hand_ros",
        executable="hand_node",
        name="linkerbot_hand",
        output="screen",
        parameters=[
            {
                "model": LaunchConfiguration("model"),
                "side": LaunchConfiguration("side"),
                "interface_name": LaunchConfiguration("interface_name"),
                "interface_type": LaunchConfiguration("interface_type"),
                "library_path": LaunchConfiguration("library_path"),
                "poll_rate": ParameterValue(
                    LaunchConfiguration("poll_rate"), value_type=float
                ),
                "timeout_ms": ParameterValue(
                    LaunchConfiguration("timeout_ms"), value_type=float
                ),
                "optional_state_rate": ParameterValue(
                    LaunchConfiguration("optional_state_rate"), value_type=float
                ),
            }
        ],
    )
    return LaunchDescription([*arguments, node])
