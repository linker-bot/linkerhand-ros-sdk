import ast
from pathlib import Path


def test_create_service_uses_keyword_only_callback_group():
    source = Path("linkerbot_hand_ros/node.py").read_text(encoding="utf-8")
    tree = ast.parse(source)
    calls = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "create_service"
    ]

    assert calls
    for call in calls:
        assert len(call.args) == 3
        assert any(keyword.arg == "callback_group" for keyword in call.keywords)


def test_entry_points_use_idempotent_rclpy_shutdown():
    entry_points = (
        Path("linkerbot_hand_ros/node.py"),
        Path("linkerbot_hand_ros/examples/angle_command.py"),
        Path("linkerbot_hand_ros/examples/angle_controller.py"),
        Path("linkerbot_hand_ros/examples/state_monitor.py"),
    )

    for path in entry_points:
        source = path.read_text(encoding="utf-8")
        assert "rclpy.try_shutdown()" in source
        assert "rclpy.shutdown()" not in source


def test_launch_exposes_every_driver_parameter():
    source = Path("launch/hand.launch.py").read_text(encoding="utf-8")
    tree = ast.parse(source)
    declared = {
        call.args[0].value
        for call in ast.walk(tree)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Name)
        and call.func.id == "DeclareLaunchArgument"
        and call.args
        and isinstance(call.args[0], ast.Constant)
    }
    configured = {
        call.args[0].value
        for call in ast.walk(tree)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Name)
        and call.func.id == "LaunchConfiguration"
        and call.args
        and isinstance(call.args[0], ast.Constant)
    }
    expected = {
        "model",
        "side",
        "interface_name",
        "interface_type",
        "library_path",
        "poll_rate",
        "timeout_ms",
        "publish_optional_state",
        "optional_state_rate",
        "diagnostic_rate",
    }

    assert declared == expected
    assert configured == expected
