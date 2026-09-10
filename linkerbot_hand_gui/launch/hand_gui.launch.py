"""Launch O6 URDF sliders, target/actual TF trees and the ROS bridge."""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from linkerbot_hand_gui.asset_resolver import load_o6_urdf, resolve_o6_assets


def _launch_preview_nodes(description: str, root_link: str):
    """Return URDF-only nodes without importing or contacting the SDK."""

    description_topic = "/linkerbot_hand_gui/preview/robot_description"
    joint_states_topic = "/linkerbot_hand_gui/preview_joint_states"
    return [
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="o6_preview_sliders",
            output="screen",
            additional_env={
                "QT_QPA_PLATFORM": "xcb",
                "QT_SCREEN_SCALE_FACTORS": "1",
                "QT_AUTO_SCREEN_SCALE_FACTOR": "0",
                "QT_WAYLAND_DISABLE_WINDOWDECORATION": "1",
                "QT_X11_NO_MITSHM": "1",
            },
            parameters=[{"rate": 60, "use_mimic_tags": True}],
            remappings=[
                ("joint_states", joint_states_topic),
                ("robot_description", description_topic),
            ],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="o6_preview_state_publisher",
            output="screen",
            parameters=[
                {"robot_description": ParameterValue(description, value_type=str)}
            ],
            remappings=[
                ("joint_states", joint_states_topic),
                ("robot_description", description_topic),
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="o6_preview_world_anchor",
            output="screen",
            arguments=[
                "--x",
                "0",
                "--y",
                "0",
                "--z",
                "0",
                "--roll",
                "0",
                "--pitch",
                "0",
                "--yaw",
                "0",
                "--frame-id",
                "world",
                "--child-frame-id",
                root_link,
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="o6_preview_rviz",
            output="screen",
            arguments=[
                "-d",
                PathJoinSubstitution(
                    [
                        FindPackageShare("linkerbot_hand_gui"),
                        "config",
                        "o6_preview.rviz",
                    ]
                ),
            ],
        ),
    ]


def _launch_nodes(context):
    side = LaunchConfiguration("side").perform(context)
    assets_root = LaunchConfiguration("assets_root").perform(context) or None
    assets = resolve_o6_assets(side, assets_root)
    driver_name = LaunchConfiguration("driver_name").perform(context).rstrip("/")
    root_link = "lh_hand_base_link" if side == "left" else "rh_hand_base_link"
    if LaunchConfiguration("preview_only").perform(context).lower() in {
        "1",
        "true",
        "yes",
    }:
        # Preview and live mode share the same resolver.  Complete visual GLB
        # files are preferred and materialized for RViz; if an older asset
        # checkout contains only OBJ visuals, the resolver keeps those files
        # usable rather than aborting the GUI before the slider window opens.
        description = load_o6_urdf(assets, require_complete_visual=False)
        return _launch_preview_nodes(description, root_link)
    description = load_o6_urdf(assets)
    description_topic = "/linkerbot_hand_gui/target/robot_description"
    # Keep the slider window on the primary display in multi-monitor desktop
    # sessions.  Qt honours this geometry only for the GUI process; RViz is
    # left to the user's saved layout.
    gui_env = {
        "QT_QPA_PLATFORM": "xcb",
        "QT_SCREEN_SCALE_FACTORS": "1",
        "QT_AUTO_SCREEN_SCALE_FACTOR": "0",
        # GNOME remembers a previous monitor placement for this application;
        # force the first window onto the primary monitor for a predictable
        # first-run experience.  Users can move it afterwards.
        "QT_WAYLAND_DISABLE_WINDOWDECORATION": "1",
        # Prevent a remembered desktop placement from putting the narrow
        # slider window on a disconnected monitor.  Qt's offscreen position
        # is still user-adjustable after startup.
        "QT_X11_NO_MITSHM": "1",
    }
    return [
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="o6_target_sliders",
            output="screen",
            # Qt may start successfully under Wayland but fail to expose a
            # top-level window on systems where the ROS launch environment is
            # inherited from a nested desktop session.  XCB is supported by
            # both Qt and RViz and makes the slider window visible reliably.
            additional_env=gui_env,
            # robot_description is published by robot_state_publisher below;
            # the slider node subscribes to that latched topic.  Passing the
            # XML as a parameter here would not configure the upstream ROS
            # node because joint_state_publisher uses the topic as its source.
            parameters=[
                {
                    "rate": 60,
                    "use_mimic_tags": True,
                    # Initialise all sliders from the latest valid physical
                    # hand state before control is armed.  The bridge emits
                    # one seed sample only; continuous actual feedback must
                    # not overwrite a target while the user is dragging.
                    "source_list": ["/linkerbot_hand_gui/initial_joint_states"],
                }
            ],
            remappings=[
                ("joint_states", "/linkerbot_hand_gui/raw_target_joint_states"),
                # joint_state_publisher_gui reads the URDF from the
                # ``robot_description`` topic rather than its parameter.  The
                # target robot_state_publisher below republishes the same
                # description on this private topic.
                (
                    "robot_description",
                    description_topic,
                ),
            ],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="o6_actual_state_publisher",
            output="screen",
            parameters=[
                {
                    "robot_description": ParameterValue(description, value_type=str),
                    "frame_prefix": "actual/",
                }
            ],
            remappings=[
                ("joint_states", "/linkerbot_hand_gui/actual_joint_states"),
                ("robot_description", "/linkerbot_hand_gui/actual/robot_description"),
            ],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="o6_target_state_publisher",
            output="screen",
            parameters=[
                {
                    "robot_description": ParameterValue(description, value_type=str),
                    "frame_prefix": "target/",
                }
            ],
            remappings=[
                ("joint_states", "/linkerbot_hand_gui/target_joint_states"),
                ("robot_description", description_topic),
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="o6_actual_world_anchor",
            output="screen",
            arguments=[
                "--x",
                "0.22",
                "--y",
                "0",
                "--z",
                "0",
                "--roll",
                "0",
                "--pitch",
                "0",
                "--yaw",
                "0",
                "--frame-id",
                "world",
                "--child-frame-id",
                f"actual/{root_link}",
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="o6_target_world_anchor",
            output="screen",
            arguments=[
                "--x",
                "0",
                "--y",
                "0",
                "--z",
                "0",
                "--roll",
                "0",
                "--pitch",
                "0",
                "--yaw",
                "0",
                "--frame-id",
                "world",
                "--child-frame-id",
                f"target/{root_link}",
            ],
        ),
        Node(
            package="linkerbot_hand_gui",
            executable="o6_gui_bridge",
            name="o6_gui_bridge",
            output="screen",
            parameters=[
                {
                    "driver_name": driver_name,
                    "side": side,
                    "assets_root": assets_root or "",
                    "control_enabled": LaunchConfiguration("control_enabled"),
                    "command_rate_hz": LaunchConfiguration("command_rate_hz"),
                }
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="o6_gui_rviz",
            output="screen",
            condition=IfCondition(LaunchConfiguration("rviz")),
            arguments=[
                "-d",
                PathJoinSubstitution(
                    [FindPackageShare("linkerbot_hand_gui"), "config", "o6.rviz"]
                ),
            ],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("side", default_value="right"),
            DeclareLaunchArgument("assets_root", default_value=""),
            DeclareLaunchArgument("driver_name", default_value="/linkerbot_hand"),
            DeclareLaunchArgument("control_enabled", default_value="false"),
            DeclareLaunchArgument("command_rate_hz", default_value="30.0"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("preview_only", default_value="false"),
            OpaqueFunction(function=_launch_nodes),
        ]
    )


__all__ = ["generate_launch_description"]
