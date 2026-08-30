"""Run multiple Hand lifecycle drivers in one ROS 2 process."""

from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import dataclass
from typing import Any, Protocol

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.validate_namespace import InvalidNamespaceException, validate_namespace

from .node import LinkerbotHandNode

CONTAINER_NODE_NAME = "linkerbot_hand_container"
EXECUTOR_SHUTDOWN_TIMEOUT_S = 2.0


class ManagedHandNode(Protocol):
    """Lifecycle surface required by the in-process driver group."""

    def get_fully_qualified_name(self) -> str: ...

    def trigger_configure(self) -> TransitionCallbackReturn: ...

    def trigger_activate(self) -> TransitionCallbackReturn: ...

    def trigger_deactivate(self) -> TransitionCallbackReturn: ...

    def trigger_cleanup(self) -> TransitionCallbackReturn: ...

    def destroy_node(self) -> None: ...


class ErrorLogger(Protocol):
    """Minimal ROS logger surface used during process cleanup."""

    def error(self, message: str) -> None: ...


HandNodeFactory = Callable[..., ManagedHandNode]


@dataclass(frozen=True, slots=True)
class MultiDeviceConfiguration:
    """Validated process-level configuration for a group of hand nodes."""

    device_namespaces: tuple[str, ...]
    autostart: bool


class MultiDeviceLifecycleError(RuntimeError):
    """Report an atomic multi-device lifecycle transition failure."""


class MultiHandContainerNode(Node):
    """Expose process-level parameters without owning device I/O."""

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(CONTAINER_NODE_NAME, **kwargs)
        self.declare_parameter("device_namespaces", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("autostart", True)

    def read_configuration(self) -> MultiDeviceConfiguration:
        """Return normalized container parameters without constructing a hand."""

        namespaces = _normalize_namespaces(
            self.get_parameter("device_namespaces").value
        )
        autostart = self.get_parameter("autostart").value
        if not isinstance(autostart, bool):
            raise TypeError("autostart must be a boolean")
        return MultiDeviceConfiguration(
            device_namespaces=namespaces,
            autostart=autostart,
        )


class HandLifecycleGroup:
    """Apply configure/activate and rollback as one ordered device group."""

    def __init__(self, nodes: Sequence[ManagedHandNode]) -> None:
        if not nodes:
            raise ValueError("at least one hand node is required")
        self._nodes = tuple(nodes)
        self._configured: list[ManagedHandNode] = []
        self._active: list[ManagedHandNode] = []

    def start(self) -> None:
        """Configure every node, then activate every node, or roll all back."""

        if self._configured or self._active:
            raise MultiDeviceLifecycleError("hand lifecycle group is already started")
        try:
            for node in self._nodes:
                self._transition(node, "configure", node.trigger_configure)
                self._configured.append(node)
            for node in self._nodes:
                self._transition(node, "activate", node.trigger_activate)
                self._active.append(node)
        except Exception as error:
            rollback_errors = self.stop()
            message = str(error)
            if rollback_errors:
                message = f"{message}; rollback failed: {'; '.join(rollback_errors)}"
            raise MultiDeviceLifecycleError(message) from error

    def stop(self) -> tuple[str, ...]:
        """Best-effort deactivate and cleanup in reverse startup order."""

        errors: list[str] = []
        for node in reversed(self._active):
            error = self._try_transition(node, "deactivate", node.trigger_deactivate)
            if error is not None:
                errors.append(error)
        self._active.clear()
        for node in reversed(self._configured):
            error = self._try_transition(node, "cleanup", node.trigger_cleanup)
            if error is not None:
                errors.append(error)
        self._configured.clear()
        return tuple(errors)

    @staticmethod
    def _transition(
        node: ManagedHandNode,
        operation: str,
        callback: Callable[[], TransitionCallbackReturn],
    ) -> None:
        result = callback()
        if result != TransitionCallbackReturn.SUCCESS:
            raise MultiDeviceLifecycleError(
                f"{node.get_fully_qualified_name()} {operation} returned {result.name}"
            )

    @classmethod
    def _try_transition(
        cls,
        node: ManagedHandNode,
        operation: str,
        callback: Callable[[], TransitionCallbackReturn],
    ) -> str | None:
        try:
            cls._transition(node, operation, callback)
        except BaseException as error:  # noqa: BLE001 - best-effort rollback boundary
            return str(error)
        return None


def create_hand_nodes(
    configuration: MultiDeviceConfiguration,
    *,
    node_factory: HandNodeFactory = LinkerbotHandNode,
) -> tuple[ManagedHandNode, ...]:
    """Construct one driver per namespace and clean up partial construction."""

    nodes: list[ManagedHandNode] = []
    try:
        for namespace in configuration.device_namespaces:
            nodes.append(node_factory(namespace=namespace))
    except BaseException:
        destroy_hand_nodes(nodes)
        raise
    return tuple(nodes)


def destroy_hand_nodes(nodes: Sequence[ManagedHandNode]) -> tuple[str, ...]:
    """Destroy all nodes in reverse order while preserving every failure."""

    errors: list[str] = []
    for node in reversed(nodes):
        try:
            node.destroy_node()
        except BaseException as error:  # noqa: BLE001 - final resource boundary
            errors.append(f"{node.get_fully_qualified_name()} destroy failed: {error}")
    return tuple(errors)


def _normalize_namespaces(value: object) -> tuple[str, ...]:
    if not isinstance(value, (list, tuple)) or not value:
        raise ValueError("device_namespaces must be a non-empty string array")
    normalized: list[str] = []
    for item in value:
        if not isinstance(item, str) or not item.strip():
            raise ValueError("device_namespaces must contain non-empty strings")
        namespace = item.strip()
        if not namespace.startswith("/"):
            namespace = f"/{namespace}"
        namespace = namespace.rstrip("/")
        if not namespace:
            raise ValueError("the root namespace cannot identify a hand device")
        try:
            validate_namespace(namespace)
        except InvalidNamespaceException as error:
            raise ValueError(f"invalid hand namespace {item!r}: {error}") from error
        if namespace in normalized:
            raise ValueError(f"duplicate hand namespace: {namespace}")
        normalized.append(namespace)
    return tuple(normalized)


def main(args: list[str] | None = None) -> None:
    """Run all configured hand drivers in one shared Python process."""

    rclpy.init(args=args)
    container: MultiHandContainerNode | None = None
    nodes: tuple[ManagedHandNode, ...] = ()
    lifecycle: HandLifecycleGroup | None = None
    executor: SingleThreadedExecutor | None = None
    try:
        container = MultiHandContainerNode()
        configuration = container.read_configuration()
        nodes = create_hand_nodes(configuration)
        lifecycle = HandLifecycleGroup(nodes)
        executor = SingleThreadedExecutor()
        executor.add_node(container)
        for node in nodes:
            executor.add_node(node)
        if configuration.autostart:
            lifecycle.start()
        container.get_logger().info(
            f"running {len(nodes)} hand lifecycle nodes in one process: "
            f"{', '.join(configuration.device_namespaces)}"
        )
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        logger = None if container is None else container.get_logger()
        if lifecycle is not None and rclpy.ok():
            try:
                for error in lifecycle.stop():
                    _log_cleanup_error(logger, error)
            except BaseException as error:  # noqa: BLE001 - final resource boundary
                _log_cleanup_error(logger, f"lifecycle group stop failed: {error}")
        if executor is not None:
            try:
                for node in nodes:
                    executor.remove_node(node)
                if container is not None:
                    executor.remove_node(container)
            except BaseException as error:  # noqa: BLE001 - final resource boundary
                _log_cleanup_error(logger, f"executor node removal failed: {error}")
            try:
                if not executor.shutdown(timeout_sec=EXECUTOR_SHUTDOWN_TIMEOUT_S):
                    _log_cleanup_error(
                        logger,
                        "executor shutdown exceeded "
                        f"{EXECUTOR_SHUTDOWN_TIMEOUT_S:g} seconds",
                    )
            except BaseException as error:  # noqa: BLE001 - final resource boundary
                _log_cleanup_error(logger, f"executor shutdown failed: {error}")
        for error in destroy_hand_nodes(nodes):
            _log_cleanup_error(logger, error)
        try:
            if container is not None:
                container.destroy_node()
        except BaseException as error:  # noqa: BLE001 - final resource boundary
            _log_cleanup_error(logger, f"container destruction failed: {error}")
        finally:
            rclpy.try_shutdown()


def _log_cleanup_error(logger: ErrorLogger | None, message: str) -> None:
    if logger is None:
        return
    try:
        logger.error(message)
    except Exception:  # noqa: BLE001 - cleanup logging must not block shutdown
        return


__all__ = [
    "CONTAINER_NODE_NAME",
    "EXECUTOR_SHUTDOWN_TIMEOUT_S",
    "HandLifecycleGroup",
    "MultiDeviceConfiguration",
    "MultiDeviceLifecycleError",
    "MultiHandContainerNode",
    "create_hand_nodes",
    "destroy_hand_nodes",
    "main",
]


if __name__ == "__main__":
    main()
