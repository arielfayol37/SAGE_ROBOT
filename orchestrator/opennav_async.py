"""
Asynchronous opennav_docking action-client bridge for SAGE.

Wraps ``DockRobot`` and ``UndockRobot`` actions with non-blocking goal
management and result callbacks that feed back into the event dispatcher.
"""

from __future__ import annotations

import logging
from typing import Any, Callable, Optional

from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from opennav_docking_msgs.action import DockRobot, UndockRobot
import threading
import time
from geometry_msgs.msg import Twist

_log = logging.getLogger("sage.dock")


class OpenNavDockingBridge(Node):
    """ROS 2 node that talks to the opennav_docking action servers."""

    STATUS_IDLE       = "idle"
    STATUS_DOCKING    = "docking"
    STATUS_UNDOCKING  = "undocking"
    STATUS_DOCKED     = "docked"
    STATUS_FAILED     = "failed"
    STATUS_CANCELLING = "cancelling"

    def __init__(
        self,
        on_dock_success: Callable[[], None],
        on_undock_success: Callable[[], None],
        on_failure: Callable[[str, str], None],
    ) -> None:
        super().__init__("opennav_docking_bridge_async")
        self._dock_client = ActionClient(self, DockRobot, "dock_robot")
        self._undock_client = ActionClient(self, UndockRobot, "undock_robot")
        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self._goal_handle: Optional[Any] = None
        self._active_action: Optional[str] = None

        # Simple-undock state
        self._simple_undock_thread: Optional[threading.Thread] = None
        self._simple_undock_cancel = threading.Event()

        self.status: str = self.STATUS_IDLE
        self._on_dock_success = on_dock_success
        self._on_undock_success = on_undock_success
        self._on_failure = on_failure

    # -- public API ----------------------------------------------------

    def dock(self, dock_id: str = "sage_charger", navigate_to_staging: bool = True) -> str:
        """Send a DockRobot goal. Returns a short status string."""
        if self._goal_handle is not None:
            self._cancel_active()

        if not self._dock_client.wait_for_server(timeout_sec=0.5):
            self.status = self.STATUS_FAILED
            self._on_failure("DOCKING_STATION", "Docking action server unavailable.")
            return "Docking server unavailable."

        self.status = self.STATUS_DOCKING
        self._active_action = "dock"

        goal = DockRobot.Goal()
        goal.use_dock_id = True
        goal.dock_id = dock_id
        goal.navigate_to_staging_pose = navigate_to_staging

        send_future = self._dock_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_response)
        return "Dock goal accepted."

    def undock(self, distance_m: float = 0.6, speed: float = 0.1) -> str:
        """Custom undock: drive in reverse for a fixed distance, then stop.

        Bypasses opennav_docking's undock action entirely. The opennav controller
        tries to reach the staging *pose* (position + yaw) using a graceful
        controller, which spins aggressively to correct yaw — bad in tight
        docks. We just need 'back up a meter, stop, hand off to Nav2.'
        """
        if self.is_busy():
            return "Already busy with a dock/undock action."

        self.status = self.STATUS_UNDOCKING
        self._active_action = "undock"
        self._simple_undock_cancel.clear()

        duration_s = distance_m / abs(speed)

        def _run():
            rate_hz = 20.0
            period = 1.0 / rate_hz
            start = time.monotonic()
            twist = Twist()
            twist.linear.x = -abs(speed)   # always reverse

            try:
                while not self._simple_undock_cancel.is_set():
                    if (time.monotonic() - start) >= duration_s:
                        break
                    self._cmd_vel_pub.publish(twist)
                    time.sleep(period)

                # Stop the robot regardless of how we exited
                stop = Twist()
                for _ in range(5):
                    self._cmd_vel_pub.publish(stop)
                    time.sleep(0.05)

                if self._simple_undock_cancel.is_set():
                    self.status = self.STATUS_IDLE
                    self._active_action = None
                    _log.info("Simple undock cancelled")
                    self._on_failure("UNDOCK", "Undock cancelled.")
                else:
                    self.status = self.STATUS_IDLE
                    self._active_action = None
                    _log.info("Simple undock complete (%.2f m at %.2f m/s)",
                            distance_m, speed)
                    self._on_undock_success()

            except Exception as exc:
                self.status = self.STATUS_FAILED
                self._active_action = None
                _log.error("Simple undock error", exc_info=True)
                self._on_failure("UNDOCK", f"Undock error: {exc}")

        self._simple_undock_thread = threading.Thread(
            target=_run, daemon=True, name="simple-undock",
        )
        self._simple_undock_thread.start()
        return f"Undocking ({distance_m:.2f} m back at {speed:.2f} m/s)."

    def cancel(self) -> str:
        """Cancel current dock or undock."""
        cancelled = []
        if self._simple_undock_thread and self._simple_undock_thread.is_alive():
            self._simple_undock_cancel.set()
            cancelled.append("Simple undock cancel requested.")
        if self._goal_handle is not None:
            try:
                self.status = self.STATUS_CANCELLING
                self._goal_handle.cancel_goal_async()
                cancelled.append("Dock action cancel requested.")
            except Exception:
                _log.warning("Failed to cancel dock goal", exc_info=True)
        return " ".join(cancelled) or "No active dock/undock."

    def is_busy(self) -> bool:
        if self._goal_handle is not None:
            return True
        if self._simple_undock_thread and self._simple_undock_thread.is_alive():
            return True
        return False

    # -- internal callbacks --------------------------------------------

    def _cancel_active(self) -> None:
        try:
            self.status = self.STATUS_CANCELLING
            self._goal_handle.cancel_goal_async()
        except Exception:
            _log.warning("Failed to cancel dock/undock goal", exc_info=True)

    def _on_goal_response(self, future: Any) -> None:
        try:
            goal_handle = future.result()
            if not goal_handle or not goal_handle.accepted:
                self.status = self.STATUS_FAILED
                action = self._active_action or "dock"
                self._active_action = None
                _log.warning("%s goal rejected", action)
                target = "DOCKING_STATION" if action == "dock" else "UNDOCK"
                self._on_failure(target, f"{action.capitalize()} goal was rejected.")
                return
            self._goal_handle = goal_handle
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self._on_result)
        except Exception:
            self.status = self.STATUS_FAILED
            self._active_action = None
            _log.error("Dock goal response error", exc_info=True)

    def _on_result(self, future: Any) -> None:
        action = self._active_action
        self._active_action = None
        self._goal_handle = None

        try:
            result_wrapper = future.result()
            status_code = result_wrapper.status
            result = result_wrapper.result

            success = bool(getattr(result, "success", False))

            if status_code == GoalStatus.STATUS_SUCCEEDED and success:
                if action == "dock":
                    self.status = self.STATUS_DOCKED
                    _log.info("Docking succeeded")
                    self._on_dock_success()
                else:
                    self.status = self.STATUS_IDLE
                    _log.info("Undocking succeeded")
                    self._on_undock_success()

            elif status_code == GoalStatus.STATUS_CANCELED:
                self.status = self.STATUS_IDLE
                _log.info("%s cancelled", action)

            else:
                self.status = self.STATUS_FAILED
                error_code = getattr(result, "error_code", None)
                reason = self._failure_reason(action, error_code, status_code)
                _log.warning("%s failed: %s", action, reason)
                target = "DOCKING_STATION" if action == "dock" else "UNDOCK"
                self._on_failure(target, reason)

        except Exception as exc:
            self.status = self.STATUS_FAILED
            _log.error("Dock result callback error", exc_info=True)
            target = "DOCKING_STATION" if action == "dock" else "UNDOCK"
            self._on_failure(target, f"Internal error processing dock result: {exc}")

    @staticmethod
    def _failure_reason(action: Optional[str], error_code: Any, status_code: int) -> str:
        verb = "Docking" if action == "dock" else "Undocking"
        if error_code is not None and error_code != 0:
            return f"{verb} failed (error code {error_code})."
        return f"{verb} did not complete (Nav2 status {status_code})."