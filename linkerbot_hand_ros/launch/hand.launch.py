from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.parameter_descriptions import ParameterValue

_ARGUMENTS = (
    ("model", "l6", str),
    ("side", "left", str),
    ("transport_json", '{"backend":"socketcan","channel":"can0"}', str),
    ("command_rate_hz", "250.0", float),
    ("discrete_queue_capacity", "16", int),
    ("mailbox_capacity", "256", int),
    ("route_queue_capacity", "256", int),
    ("identity_timeout_s", "0.2", float),
    ("transaction_timeout_s", "0.1", float),
    ("shutdown_timeout_s", "2.0", float),
    ("auto_reconnect", "false", bool),
    ("reconnect_initial_delay_s", "0.1", float),
    ("reconnect_max_delay_s", "2.0", float),
    ("publish_rate_hz", "100.0", float),
    ("diagnostic_rate_hz", "1.0", float),
    ("angles_hz", "100.0", float),
    ("speeds_hz", "0.0", float),
    ("force_hz", "0.0", float),
    ("temperature_hz", "0.0", float),
    ("current_hz", "0.0", float),
    ("faults_hz", "0.0", float),
    ("torques_hz", "0.0", float),
    ("voltage_hz", "0.0", float),
)


def generate_launch_description() -> LaunchDescription:
    arguments = [
        DeclareLaunchArgument(name, default_value=default)
        for name, default, _value_type in _ARGUMENTS
    ]
    arguments.extend(
        (
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("autostart", default_value="true"),
        )
    )
    parameters = {
        name: ParameterValue(LaunchConfiguration(name), value_type=value_type)
        for name, _default, value_type in _ARGUMENTS
    }
    node = LifecycleNode(
        package="linkerbot_hand_ros",
        executable="hand_node",
        name="linkerbot_hand",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        autostart=LaunchConfiguration("autostart"),
        parameters=[parameters],
    )
    return LaunchDescription([*arguments, node])
