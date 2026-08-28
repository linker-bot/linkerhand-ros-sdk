"""Launch multiple Hand lifecycle drivers in one Python process."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    default_config = PathJoinSubstitution(
        [FindPackageShare("linkerbot_hand_ros"), "config", "multi_hand.yaml"]
    )
    config = LaunchConfiguration("config")
    container = Node(
        package="linkerbot_hand_ros",
        executable="hand_container",
        output="screen",
        parameters=[ParameterFile(config, allow_substs=True)],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=default_config,
                description="ROS parameter file containing the container and hand nodes",
            ),
            container,
        ]
    )
