"""
ROS 2 / Nav2 lifecycle manager for SAGE.

Owns rclpy lifecycle, the executor, and both action-client nodes
(Nav2 and opennav_docking). Provides a single entry point
``set_goal_by_name`` that abstracts dock/undock from the LLM layer.
"""
from __future__ import annotations

import json
import os
import threading
from pathlib import Path
from typing import Any, Callable, Dict, Optional
from dock_state import DockStateTracker, DockState
import logger

_log = logger.get("robot")

DOCK_WAYPOINT_NAME = "DOCKING_STATION"


class NavManager:
    """Thread-safe lazy-start wrapper around Nav2 + docking action clients.

    Call :meth:`ensure_started` before any navigation/docking call.
    """

    def __init__(
        self,
        enqueue_arrival: Callable[[str], None],
        enqueue_failure: Callable[[str, str], None],
        waypoints: Dict[str, Dict[str, Any]],
    ) -> None:
        self._enqueue_arrival = enqueue_arrival
        self._enqueue_failure = enqueue_failure
        self._waypoints = waypoints

        self._nav_node: Optional[Any] = None         # Nav2AsyncBridge
        self._dock_node: Optional[Any] = None        # OpenNavDockingBridge
        self._executor: Optional[Any] = None
        self._started = False
        self._lock = threading.Lock()

        self._dock_tracker: Optional[DockStateTracker] = None

    # -- public API ----------------------------------------------------

    @property
    def nav_node(self) -> Any:
        if self._nav_node is None:
            raise RuntimeError("NavManager has not been started yet")
        return self._nav_node

    @property
    def dock_node(self) -> Any:
        if self._dock_node is None:
            raise RuntimeError("NavManager has not been started yet")
        return self._dock_node

    @property
    def dock_state(self) -> DockState:
        """Current dock state. Synthesized live, not stored as a flag."""
        if self._dock_tracker is None:
            return DockState.UNKNOWN
        return self._dock_tracker.query()

    def ensure_started(self) -> None:
        if self._started:
            return
        with self._lock:
            if self._started:
                return
            self._start_locked()

    def status(self) -> Dict[str, Any]:
        """Snapshot of nav + dock state."""
        if not self._started:
            return {"status": "offline"}
        return {
            "nav_status": self._nav_node.status,
            "dock_status": self._dock_node.status,
            "dock_state": self._dock_tracker.query().value,
            "target_name": getattr(self._nav_node, "current_target", None),
            "feedback": self._nav_node.last_feedback,
        }

    def set_goal_by_name(self, location: str) -> str:
        """Single entry point for the LLM. Handles waypoints + dock cases."""
        self.ensure_started()

        state = self._dock_tracker.query()

        # Special case: dock
        if location == DOCK_WAYPOINT_NAME:
            if state == DockState.DOCKED:
                return "Already docked."
            if state in (DockState.DOCKING, DockState.UNDOCKING):
                return f"Currently {state.value}; try again in a moment."
            self._dock_tracker.declare_attempting_dock()
            return self._dock_node.dock(navigate_to_staging=True)

        # Normal Nav2 goal
        if location not in self._waypoints or location.startswith("_"):
            valid = ", ".join(k for k in self._waypoints.keys() if not k.startswith("_"))
            return f"Unknown location '{location}'. Valid: {valid}"

        if state == DockState.DOCKED:
            return self._undock_then_navigate(location)

        if state in (DockState.DOCKING, DockState.UNDOCKING):
            return f"Currently {state.value}; try again in a moment."

        if state == DockState.UNKNOWN:
            # We might be docked. Try undocking but don't fail catastrophically
            # if it turns out we weren't.
            _log.info("Dock state is UNKNOWN; attempting undock-then-nav as a precaution")
            return self._undock_then_navigate(location, allow_undock_failure=True)

        return self._send_nav_goal(location)

    def cancel_active(self) -> str:
        """Cancel whichever action is currently in flight."""
        if not self._started:
            return "No active goal."

        # Try cancelling whichever is busy
        cancelled = []
        if self._dock_node.is_busy():
            cancelled.append(self._dock_node.cancel())
        cancelled.append(self._nav_node.cancel_goal())
        return " ".join(cancelled)

    def shutdown(self) -> None:
        import rclpy
        with self._lock:
            if self._executor:
                for node in (self._nav_node, self._dock_node):
                    if node:
                        try:
                            self._executor.remove_node(node)
                        except Exception:
                            pass
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
            self._nav_node = None
            self._dock_node = None
            self._executor = None

    # -- internals: action sequencing ----------------------------------

    def _send_nav_goal(self, location: str) -> str:
        wp = self._waypoints[location]
        return self._nav_node.set_goal(
            frame_id=wp.get("frame_id", "map"),
            x=wp["x"], y=wp["y"],
            ox=wp["ox"], oy=wp["oy"], oz=wp["oz"], ow=wp["ow"],
            location_name=location,
        )

    def _undock_then_navigate(self, location: str, allow_undock_failure: bool = False) -> str:
        with self._lock:
            self._pending_after_undock = location
            self._undock_failure_ok = allow_undock_failure
        self._dock_tracker.declare_attempting_undock()
        msg = self._dock_node.undock()
        return f"Undocking first, then heading to {location}. ({msg})"

    # -- internals: callbacks from dock bridge -------------------------

    def _on_dock_success(self) -> None:
        self._dock_tracker.declare_dock_succeeded()
        self._enqueue_arrival(DOCK_WAYPOINT_NAME)

    def _on_undock_success(self) -> None:
        self._dock_tracker.declare_undock_succeeded()

        with self._lock:
            pending = getattr(self, "_pending_after_undock", None)
            self._pending_after_undock = None
            self._undock_failure_ok = False

        if pending:
            try:
                self._send_nav_goal(pending)
            except Exception as exc:
                _log.exception("Pending nav after undock failed")
                self._enqueue_failure(pending, f"Couldn't start navigation after undocking: {exc}")

    def _on_dock_failure(self, target: str, reason: str) -> None:
        if target == "DOCKING_STATION":
            self._dock_tracker.declare_dock_failed(reason)
        else:  # UNDOCK
            self._dock_tracker.declare_undock_failed(reason)

        with self._lock:
            pending = getattr(self, "_pending_after_undock", None)
            allow_failure = getattr(self, "_undock_failure_ok", False)
            self._pending_after_undock = None
            self._undock_failure_ok = False

        if target == "UNDOCK" and allow_failure and pending:
            # We weren't sure we were docked; undock failed; just try the nav anyway.
            _log.info("Undock failed but we weren't sure we were docked; proceeding to %s", pending)
            try:
                self._send_nav_goal(pending)
            except Exception as exc:
                self._enqueue_failure(pending, f"Couldn't start navigation: {exc}")
            return

        if pending and target == "UNDOCK":
            self._enqueue_failure(pending, f"Couldn't undock: {reason}")
        else:
            self._enqueue_failure(target, reason)

    # -- internals: lifecycle ------------------------------------------

    def _start_locked(self) -> None:
        import rclpy
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from nav2async import Nav2AsyncBridge
        from opennav_async import OpenNavDockingBridge

        if not rclpy.ok():
            rclpy.init()

        self._nav_node = Nav2AsyncBridge(
            enqueue_arrival=self._enqueue_arrival,
            enqueue_failure=self._enqueue_failure,
        )
        self._dock_node = OpenNavDockingBridge(
            on_dock_success=self._on_dock_success,
            on_undock_success=self._on_undock_success,
            on_failure=self._on_dock_failure,
        )

        # Dock state tracker — needs the docked physical pose, not staging
        docked_wp = self._waypoints.get("_DOCKED_POSE")
        if docked_wp is None:
            raise RuntimeError(
                "_DOCKED_POSE waypoint missing from waypoints.py. "
                "Required for DockStateTracker proximity check."
            )
        self._dock_tracker = DockStateTracker(
            docked_pose_xy=(docked_wp["x"], docked_wp["y"])
        )

        # Tracker needs a ROS node for TF + battery — give it its own
        self._tracker_node = Node("dock_state_tracker_node")
        self._dock_tracker.attach(self._tracker_node)

        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self._nav_node)
        self._executor.add_node(self._dock_node)
        self._executor.add_node(self._tracker_node)

        spin_thread = threading.Thread(
            target=self._executor.spin, daemon=True, name="ros2-spin",
        )
        spin_thread.start()
        self._started = True
        _log.info("Nav + docking + dock-state bridges started")