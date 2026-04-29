"""
Dock state tracker for SAGE.

Synthesizes a single dock-state answer from multiple inputs that can
disagree:

  - Recent intent (from action callbacks, persisted to disk as event log)
  - Robot localization (TF: map -> base_link)
  - Charging signal (from /battery_state when the charger is built)

Exposes a single ``query()`` method. Callers don't read variables
directly — the tracker computes the answer fresh each time.

The query() method is the only allowed read. All updates go through
declare_*() methods. State is intentionally event-driven and synthesized,
not a single boolean stored anywhere.
"""

from __future__ import annotations

import json
import math
import threading
import time
from enum import Enum
from pathlib import Path
from typing import Any, Dict, Optional

import logger

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
import tf2_ros

_log = logger.get("dock_state")

# Where the event log lives
LOG_FILE = Path.home() / ".sage" / "dock_events.jsonl"

# How recent must intent be before we trust it as authoritative
INTENT_MAX_AGE_S = 24 * 60 * 60       # 24 hours

# Geometric proximity threshold for "near the dock"
NEAR_DOCK_RADIUS_M = 0.30
NEAR_DOCK_HYSTERESIS_M = 0.10         # extra margin once already declared near

# How fresh charging signal must be
CHARGING_MAX_AGE_S = 5.0

# How fresh TF must be to use it
TF_MAX_AGE_S = 2.0


class DockState(Enum):
    UNDOCKED  = "undocked"
    DOCKED    = "docked"
    DOCKING   = "docking"
    UNDOCKING = "undocking"
    UNKNOWN   = "unknown"


class DockStateTracker:
    """Synthesizes dock state from intent log + localization + charging.

    Owns its own ROS node (a TF listener and a /battery_state subscriber).
    The node should be added to the same executor that NavManager uses.
    """

    def __init__(self, docked_pose_xy: tuple[float, float]) -> None:
        self._dock_x, self._dock_y = docked_pose_xy
        self._lock = threading.Lock()

        # Charging signal cache
        self._latest_charging: Optional[bool] = None
        self._latest_charging_ts: float = 0.0

        # Hysteresis: track previous proximity result
        self._was_near_dock: bool = False

        # ROS plumbing — initialized in attach()
        self._node: Optional[Node] = None
        self._tf_buffer: Optional[tf2_ros.Buffer] = None
        self._tf_listener: Optional[tf2_ros.TransformListener] = None

        # Read last event from log if available
        self._last_event = self._read_last_event()

    # ------------------------------------------------------------------
    # ROS lifecycle
    # ------------------------------------------------------------------

    def attach(self, node: Node) -> None:
        """Attach to a ROS node so we can listen to TF and battery state."""
        self._node = node
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, node)
        node.create_subscription(
            BatteryState, "/battery_state", self._on_battery, 10,
        )

    # ------------------------------------------------------------------
    # Public API: declare intent
    # ------------------------------------------------------------------

    def declare_attempting_dock(self) -> None:
        self._append_event("attempting_dock")

    def declare_dock_succeeded(self) -> None:
        self._append_event("dock_succeeded")

    def declare_dock_failed(self, reason: str = "") -> None:
        self._append_event("dock_failed", reason=reason)

    def declare_attempting_undock(self) -> None:
        self._append_event("attempting_undock")

    def declare_undock_succeeded(self) -> None:
        self._append_event("undock_succeeded")

    def declare_undock_failed(self, reason: str = "") -> None:
        self._append_event("undock_failed", reason=reason)

    # ------------------------------------------------------------------
    # Public API: query state
    # ------------------------------------------------------------------

    def query(self) -> DockState:
        """Return the current best-effort dock state."""
        with self._lock:
            return self._compute_state()

    def explain(self) -> Dict[str, Any]:
        """Diagnostic info — what inputs the tracker is seeing right now."""
        with self._lock:
            near, dist = self._near_dock_with_distance()
            return {
                "state": self._compute_state().value,
                "last_event": self._last_event,
                "near_dock": near,
                "distance_to_dock_m": dist,
                "charging": self._fresh_charging(),
            }

    # ------------------------------------------------------------------
    # Internals: state synthesis
    # ------------------------------------------------------------------

    def _compute_state(self) -> DockState:
        """Synthesize state from inputs. Held under self._lock."""
        # In-flight transitions take priority — the action is mid-flight
        intent = self._fresh_intent()
        if intent == "attempting_dock":
            return DockState.DOCKING
        if intent == "attempting_undock":
            return DockState.UNDOCKING

        near, _ = self._near_dock_with_distance()
        charging = self._fresh_charging()

        # Charging is the gold standard if we have it
        if charging is True:
            if near:
                return DockState.DOCKED
            # Charging but far from dock — sensor lying or AMCL lost.
            # Lean toward "trust the charging signal" since it's physical.
            return DockState.DOCKED

        # No charging signal (yet) — synthesize from intent + position
        if intent == "dock_succeeded":
            if near:
                return DockState.DOCKED
            # Last action said docked but we're far away → something
            # changed (robot moved, dock moved, AMCL drift). Don't pretend.
            return DockState.UNKNOWN

        if intent == "undock_succeeded":
            if not near:
                return DockState.UNDOCKED
            # Said undocked but still at the dock pose — undock didn't
            # actually move us, or pose is ambiguous.
            return DockState.UNKNOWN

        if intent == "dock_failed" or intent == "undock_failed":
            # An attempt failed. Trust position alone.
            return DockState.DOCKED if near else DockState.UNDOCKED

        # No fresh intent (cold boot or stale log)
        if near:
            # Booted near the dock; provisionally call it DOCKED.
            # When the charger reports current, this hardens. Without
            # charging info, this is a best guess.
            return DockState.DOCKED
        return DockState.UNDOCKED

    def _near_dock_with_distance(self) -> tuple[bool, Optional[float]]:
        """Return (is_near, distance_m). distance is None if TF unavailable."""
        pose = self._lookup_robot_xy()
        if pose is None:
            return self._was_near_dock, None  # keep last answer

        rx, ry = pose
        dist = math.hypot(rx - self._dock_x, ry - self._dock_y)

        threshold = (
            NEAR_DOCK_RADIUS_M + NEAR_DOCK_HYSTERESIS_M
            if self._was_near_dock else NEAR_DOCK_RADIUS_M
        )
        is_near = dist <= threshold
        self._was_near_dock = is_near
        return is_near, dist

    def _lookup_robot_xy(self) -> Optional[tuple[float, float]]:
        if self._tf_buffer is None or self._node is None:
            return None
        try:
            t = self._tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time(),
                timeout=Duration(seconds=0.1),
            )
            return (t.transform.translation.x, t.transform.translation.y)
        except Exception:
            return None

    def _fresh_charging(self) -> Optional[bool]:
        """Return True/False if we have fresh charging data, else None."""
        if self._latest_charging is None:
            return None
        if time.time() - self._latest_charging_ts > CHARGING_MAX_AGE_S:
            return None
        return self._latest_charging

    def _fresh_intent(self) -> Optional[str]:
        """Return the kind of the most recent event if it's not too stale."""
        if self._last_event is None:
            return None
        age = time.time() - self._last_event.get("ts", 0.0)
        if age > INTENT_MAX_AGE_S:
            return None
        return self._last_event.get("kind")

    # ------------------------------------------------------------------
    # Internals: callbacks
    # ------------------------------------------------------------------

    def _on_battery(self, msg: BatteryState) -> None:
        # Until charger reports POWER_SUPPLY_STATUS_CHARGING, this stays None
        # (the field defaults to UNKNOWN, which we treat as "no signal").
        status = int(msg.power_supply_status)
        if status == BatteryState.POWER_SUPPLY_STATUS_CHARGING:
            self._latest_charging = True
            self._latest_charging_ts = time.time()
        elif status in (
            BatteryState.POWER_SUPPLY_STATUS_DISCHARGING,
            BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING,
        ):
            self._latest_charging = False
            self._latest_charging_ts = time.time()
        # POWER_SUPPLY_STATUS_UNKNOWN → don't update; we have no signal

    # ------------------------------------------------------------------
    # Internals: persistence
    # ------------------------------------------------------------------

    def _append_event(self, kind: str, **extra: Any) -> None:
        event: Dict[str, Any] = {"ts": time.time(), "kind": kind}
        if extra:
            event.update(extra)
        with self._lock:
            self._last_event = event
        try:
            LOG_FILE.parent.mkdir(parents=True, exist_ok=True)
            with LOG_FILE.open("a") as f:
                f.write(json.dumps(event) + "\n")
        except Exception:
            _log.warning("Failed to persist dock event %s", kind, exc_info=True)

    def _read_last_event(self) -> Optional[Dict[str, Any]]:
        """Return the most recent event from the log, or None."""
        if not LOG_FILE.exists():
            return None
        try:
            # Read last non-empty line — log is append-only and small
            with LOG_FILE.open() as f:
                lines = [ln.strip() for ln in f if ln.strip()]
            if not lines:
                return None
            return json.loads(lines[-1])
        except Exception:
            _log.warning("Failed to read dock event log", exc_info=True)
            return None