#!/usr/bin/env python3
"""
ui_state_client.py — minimal helper to publish SAGE UI state for the status interface.

Publishes JSON strings on /sage/ui_state_json (std_msgs/String), e.g.:
  {"phase":"speaking","energy":0.42,"viseme":7}

Phases: 'idle'|'listening'|'thinking'|'speaking'|'searching'|'error'
Only include what's needed to drive animations. No transcript text here.
"""

from __future__ import annotations
import json, time, threading
from typing import Optional, Literal, Dict

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String as RosString

Phase = Literal['idle','listening','thinking','speaking','searching','error']

class _UIStateNode(Node):
    def __init__(self, topic: str):
        super().__init__('sage_ui_state_client')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.pub = self.create_publisher(RosString, topic, qos)

class UIStatePublisher:
    """
    Usage:
        ui = UIStatePublisher()                  # starts a tiny ROS2 node in background
        ui.set_phase('listening')
        ui.set_phase('thinking')
        ui.set_phase('speaking')
        ui.speaking_energy(0.35)                # call at ~10-30 Hz if you want
        ui.set_phase('idle')
        ui.close()                               # optional (on program exit)
    """
    def __init__(self, topic: str = "/sage/ui_state_json",
                 energy_hz_max: float = 20.0):
        self._topic = topic
        self._energy_min_dt = 1.0 / max(1e-6, energy_hz_max)
        self._last_energy_ts = 0.0

        # ROS2 bring-up (idempotent)
        self._owned_init = False
        if not rclpy.ok():
            rclpy.init()
            self._owned_init = True

        self._node = _UIStateNode(topic)
        self._exec = SingleThreadedExecutor()
        self._exec.add_node(self._node)

        self._spin_thread = threading.Thread(target=self._exec.spin, daemon=True)
        self._spin_thread.start()

        self._lock = threading.Lock()
        self._last_phase: Phase = 'idle'

    # ------------- core send -------------
    def _send(self, payload: Dict):
        msg = RosString()
        msg.data = json.dumps(payload, separators=(',', ':'))
        # publish is threadsafe in rclpy; guard anyway to order messages
        with self._lock:
            self._node.pub.publish(msg)

    def _now(self) -> float:
        return time.time()

    # ------------- public API -------------

    def set_phase(self, phase: Phase, *, error: Optional[str] = None, viseme: Optional[int] = None):
        """Set the current UI phase; include optional one-off hints."""
        self._last_phase = phase
        data: Dict = {"phase": phase}
        if viseme is not None:
            data["viseme"] = int(viseme)
        if phase == 'error' and error:
            # Keep it minimal; hub/face can flash error animation without text
            data["err"] = True
        self._send(data)

    # Convenience shorthands
    def idle(self):       self.set_phase('idle')
    def listening(self):  self.set_phase('listening')
    def thinking(self):   self.set_phase('thinking')
    def speaking(self):   self.set_phase('speaking')
    def searching(self):  self.set_phase('searching')
    def error(self):      self.set_phase('error')

    def speaking_energy(self, energy: float, *, viseme: Optional[int] = None):
        """
        Send energy (0..1) and optional viseme while in 'speaking'.
        Throttled to energy_hz_max to avoid spamming.
        """
        now = self._now()
        if (now - self._last_energy_ts) < self._energy_min_dt:
            return
        self._last_energy_ts = now
        e = max(0.0, min(1.0, float(energy)))
        payload: Dict = {"phase": "speaking", "energy": e}
        if viseme is not None:
            payload["viseme"] = int(viseme)
        self._send(payload)

    # Optional context managers for clean phase lifetimes
    def phase_scope(self, phase: Phase):
        """with ui.phase_scope('thinking'): ..."""
        class _Scope:
            def __init__(s, outer: UIStatePublisher, p: Phase):
                s._outer = outer; s._p = p
            def __enter__(s):
                s._outer.set_phase(s._p)
            def __exit__(s, exc_type, exc, tb):
                # if we were thinking/searching, default back to idle
                if s._p in ('thinking','searching','listening','speaking'):
                    s._outer.idle()
        return _Scope(self, phase)

    def close(self):
        """Clean shutdown (optional if process is exiting)."""
        try:
            self._exec.remove_node(self._node)
        except Exception:
            pass
        try:
            self._node.destroy_node()
        except Exception:
            pass
        try:
            self._exec.shutdown()
        except Exception:
            pass
        if self._owned_init and rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass
