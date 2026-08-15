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
        "transport_options_json",
        "interface_name",
        "interface_type",
        "library_path",
        "command_rate_hz",
        "auto_reconnect",
        "publish_rate_hz",
        "angles_hz",
        "speeds_hz",
        "torques_hz",
        "force_hz",
        "temperature_hz",
        "current_hz",
        "faults_hz",
        "diagnostic_rate",
        "namespace",
        "autostart",
    }

    assert declared == expected
    assert configured == expected


def test_launch_uses_jazzy_lifecycle_node_with_autostart():
    source = Path("launch/hand.launch.py").read_text(encoding="utf-8")
    tree = ast.parse(source)
    lifecycle_nodes = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "LifecycleNode"
    ]

    assert len(lifecycle_nodes) == 1
    assert any(keyword.arg == "autostart" for keyword in lifecycle_nodes[0].keywords)


def test_ros_driver_contains_no_direct_blocking_sdk_reads():
    source = "\n".join(
        Path(path).read_text(encoding="utf-8")
        for path in (
            "linkerbot_hand_ros/adapter.py",
            "linkerbot_hand_ros/node.py",
        )
    )

    assert "get_blocking" not in source
    assert "_PriorityIoLock" not in source
    assert "get_snapshot()" in source
    assert "start_polling(" in source


def test_package_depends_on_split_hand_sdk():
    setup_source = Path("setup.py").read_text(encoding="utf-8")

    assert '"linkerbot-hand>=0.1.0.dev1,<0.2.0"' in setup_source
    assert '"linkerbot>=0.5.4"' not in setup_source
    assert '"linkerbot-transport' not in setup_source


def test_ros_source_uses_only_the_unified_hand_import_path():
    source = "\n".join(
        path.read_text(encoding="utf-8")
        for path in Path("linkerbot_hand_ros").rglob("*.py")
    )

    assert "from linkerbot_hand" in source
    assert "from linkerbot.hand" not in source
    assert "from linkerbot_transport" not in source
    assert "MODEL_SPECS" not in source


def test_package_declares_runtime_parameter_message_dependency():
    package_source = Path("package.xml").read_text(encoding="utf-8")

    assert "<exec_depend>rcl_interfaces</exec_depend>" in package_source


def test_python_and_ros_package_versions_match():
    setup_source = Path("setup.py").read_text(encoding="utf-8")
    package_source = Path("package.xml").read_text(encoding="utf-8")
    module_source = Path("linkerbot_hand_ros/__init__.py").read_text(encoding="utf-8")

    assert 'version="0.3.0"' in setup_source
    assert "<version>0.3.0</version>" in package_source
    assert '__version__ = "0.3.0"' in module_source
