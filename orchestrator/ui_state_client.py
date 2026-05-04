"""
Minimal helper to publish SAGE UI state over ROS 2.

Publishes JSON on ``/sage/ui_state_json`` (``std_msgs/String``)::

    {"phase":"speaking","energy":0.42,"viseme":7}

Phases: ``idle | listening | thinking | speaking | searching | error``
"""

from __future__ import annotations

import json
import logging
import threading
import time
from typing import Any, Dict, Literal, Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String as RosString

_log = logging.getLogger("sage.ui")

Phase = Literal["idle", "listening", "thinking", "speaking", "searching", "error"]


class _UIStateNode(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("sage_ui_state_client")
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.pub = self.create_publisher(RosString, topic, qos)


class UIStatePublisher:
    """Publish UI phase transitions over a ROS 2 topic.

    Usage::

        ui = UIStatePublisher()
        ui.listening()
        ui.thinking()
        ui.speaking()
        ui.idle()
        ui.close()
    """

    def __init__(
        self,
        topic: str = "/sage/ui_state_json",
        energy_hz_max: float = 20.0,
    ) -> None:
        self._energy_min_dt = 1.0 / max(1e-6, energy_hz_max)
        self._last_energy_ts: float = 0.0

        # Idempotent ROS 2 init
        self._owned_init = False
        if not rclpy.ok():
            rclpy.init()
            self._owned_init = True

        self._node = _UIStateNode(topic)
        self._exec = SingleThreadedExecutor()
        self._exec.add_node(self._node)

        self._spin_thread = threading.Thread(
            target=self._exec.spin, daemon=True, name="ui-state-spin",
        )
        self._spin_thread.start()

        self._lock = threading.Lock()
        self._last_phase: Phase = "idle"

    # -- core send -----------------------------------------------------

    def _send(self, payload: Dict[str, Any]) -> None:
        msg = RosString()
        msg.data = json.dumps(payload, separators=(",", ":"))
        with self._lock:
            self._node.pub.publish(msg)

    # -- public API ----------------------------------------------------

    @property
    def last_phase(self) -> Phase:
        return self._last_phase

    def set_phase(
        self,
        phase: Phase,
        *,
        error: Optional[str] = None,
        viseme: Optional[int] = None,
    ) -> None:
        """Set the current UI phase with optional one-off hints."""
        self._last_phase = phase
        data: Dict[str, Any] = {"phase": phase}
        if viseme is not None:
            data["viseme"] = int(viseme)
        if phase == "error" and error:
            data["err"] = True
        self._send(data)

    # Convenience shorthands
    def idle(self) -> None:
        self.set_phase("idle")

    def listening(self) -> None:
        self.set_phase("listening")

    def thinking(self) -> None:
        self.set_phase("thinking")

    def speaking(self) -> None:
        self.set_phase("speaking")

    def searching(self) -> None:
        self.set_phase("searching")

    def error(self) -> None:
        self.set_phase("error")

    def speaking_energy(self, energy: float, *, viseme: Optional[int] = None) -> None:
        """Send energy (0…1) while speaking.  Rate-limited."""
        now = time.time()
        if (now - self._last_energy_ts) < self._energy_min_dt:
            return
        self._last_energy_ts = now
        payload: Dict[str, Any] = {
            "phase": "speaking",
            "energy": max(0.0, min(1.0, float(energy))),
        }
        if viseme is not None:
            payload["viseme"] = int(viseme)
        self._send(payload)

    def phase_scope(self, phase: Phase) -> "_PhaseScope":
        """Context manager: ``with ui.phase_scope('thinking'): …``"""
        return _PhaseScope(self, phase)

    def close(self) -> None:
        """Clean shutdown (optional if the process is exiting)."""
        for action in (
            lambda: self._exec.remove_node(self._node),
            lambda: self._node.destroy_node(),
            lambda: self._exec.shutdown(),
        ):
            try:
                action()
            except Exception:
                pass
        if self._owned_init and rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


class _PhaseScope:
    """RAII-style phase lifetime helper."""

    _TRANSIENT_PHASES: frozenset[Phase] = frozenset(
        {"thinking", "searching", "listening", "speaking"}
    )

    def __init__(self, publisher: UIStatePublisher, phase: Phase) -> None:
        self._pub = publisher
        self._phase = phase

    def __enter__(self) -> None:
        self._pub.set_phase(self._phase)

    def __exit__(self, *_: object) -> None:
        if self._phase in self._TRANSIENT_PHASES:
            self._pub.idle()
