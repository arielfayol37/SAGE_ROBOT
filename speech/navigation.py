"""
ROS 2 / Nav2 lifecycle manager for SAGE.

Encapsulates ``rclpy.init()``, the ``MultiThreadedExecutor``, and the
:class:`nav2async.Nav2AsyncBridge` node so the rest of the application
never has to import ``rclpy`` directly.
"""

from __future__ import annotations

import threading
from typing import Any, Callable, Dict, Optional

import logger

_log = logger.get("robot")


class NavManager:
    """Thread-safe, lazy-start wrapper around the Nav2 action-client node.

    Call :meth:`ensure_started` before any navigation call; it is
    idempotent and safe from any thread.
    """

    def __init__(self, enqueue_arrival: Callable[[str], None]) -> None:
        self._enqueue_arrival = enqueue_arrival
        self._node: Optional[Any] = None      # Nav2AsyncBridge
        self._executor: Optional[Any] = None  # MultiThreadedExecutor
        self._started = False
        self._lock = threading.Lock()

    # -- public API ----------------------------------------------------

    @property
    def node(self) -> Any:
        """The :class:`Nav2AsyncBridge` node (raises if not started)."""
        if self._node is None:
            raise RuntimeError("NavManager has not been started yet")
        return self._node

    def ensure_started(self) -> None:
        """Idempotently initialise rclpy + spin the executor in a thread."""
        if self._started:
            return
        with self._lock:
            if self._started:
                return
            self._start_locked()

    def status(self) -> Dict[str, Any]:
        """Return a snapshot of the current navigation state."""
        if self._node is None:
            return {"status": "offline"}
        return {
            "status": self._node.status,
            "target_name": getattr(self._node, "current_target", None),
            "feedback": self._node.last_feedback,
        }

    def shutdown(self) -> None:
        """Tear down the executor and rclpy (best-effort, idempotent)."""
        import rclpy  # local import to avoid top-level ROS dep

        with self._lock:
            if self._executor and self._node:
                try:
                    self._executor.remove_node(self._node)
                except Exception:
                    pass
            if self._executor:
                try:
                    self._executor.shutdown()
                except Exception:
                    pass
            if rclpy.ok():
                try:
                    rclpy.shutdown()
                except Exception:
                    pass
            self._started = False
            self._node = None
            self._executor = None

    # -- internals -----------------------------------------------------

    def _start_locked(self) -> None:
        """Must be called while holding ``self._lock``."""
        import rclpy
        from rclpy.executors import MultiThreadedExecutor
        from nav2async import Nav2AsyncBridge

        if not rclpy.ok():
            rclpy.init()

        self._node = Nav2AsyncBridge(enqueue_arrival=self._enqueue_arrival)
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self._node)

        spin_thread = threading.Thread(
            target=self._executor.spin, daemon=True, name="ros2-spin",
        )
        spin_thread.start()
        self._started = True
        _log.info("Nav2 bridge started (spin thread: %s)", spin_thread.name)
