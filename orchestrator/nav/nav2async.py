"""
Asynchronous Nav2 action-client bridge for SAGE.

Wraps the ``NavigateToPose`` action with non-blocking goal management,
feedback tracking, and an arrival callback that feeds back into the
event dispatcher.
"""

from __future__ import annotations

import logging
import time
from typing import Any, Callable, Dict, Optional

from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
import rclpy

_log = logging.getLogger("sage.robot")


class Nav2AsyncBridge(Node):
    """ROS 2 node that talks to the Nav2 ``navigate_to_pose`` action server.

    Parameters
    ----------
    enqueue_arrival:
        Callback invoked (from the ROS executor thread) when a goal
        reaches ``STATUS_SUCCEEDED``.
    """

    # Valid status strings exposed to the rest of the app
    STATUS_IDLE       = "idle"
    STATUS_NAVIGATING = "navigating"
    STATUS_ARRIVED    = "arrived"
    STATUS_FAILED     = "failed"
    STATUS_CANCELLING = "cancelling"

    def __init__(
        self,
        enqueue_arrival: Callable[[str], None],
        enqueue_failure: Callable[[str, str], None],  
    ) -> None:  
        super().__init__("nav2_llm_bridge_async")
        self._action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._goal_handle: Optional[Any] = None

        self.status: str = self.STATUS_IDLE
        self.current_target: Optional[str] = None
        self.last_feedback: Dict[str, Any] = {}
        self._enqueue_arrival = enqueue_arrival
        self._enqueue_failure = enqueue_failure
        self._goal_start_time: float = 0.0
        self._timeout_cancel: bool = False
        self._nav_timeout_s: float = 60.0   # beyond this the TF buffer is stale
        self._pending_goal: Optional[NavigateToPose.Goal] = None
        self._pending_location: Optional[str] = None
        self._goal_retry_count: int = 0
        self._max_goal_retries: int = 2
    # -- public API ----------------------------------------------------

    def set_goal(
        self,
        *,
        frame_id: str,
        x: float, y: float,
        ox: float, oy: float, oz: float, ow: float,
        location_name: str,
    ) -> str:
        """Send a new navigation goal, preempting any active one.

        Returns a short status string suitable for tool-call results.
        """
        self.current_target = location_name
        self.status = self.STATUS_NAVIGATING
        self._goal_start_time = time.monotonic()
        self._timeout_cancel = False

        # Cancel any in-flight goal
        if self._goal_handle is not None:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:
                _log.warning("Failed to cancel previous goal", exc_info=True)

        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self.status = self.STATUS_FAILED
            return "Nav2 action server not available."

        pose = self._build_pose(frame_id, x, y, ox, oy, oz, ow)
        goal = NavigateToPose.Goal()
        goal.pose = pose

        self._pending_goal = goal
        self._pending_location = location_name
        self._goal_retry_count = 0

        send_future = self._action_client.send_goal_async(
            goal, feedback_callback=self._on_feedback,
        )
        send_future.add_done_callback(self._on_goal_response)
        return "Goal accepted (navigating)."

    def cancel_goal(self) -> str:
        """Cancel the current navigation goal, if any."""
        if self._goal_handle is None:
            return "No active goal."
        self.status = self.STATUS_CANCELLING
        future = self._goal_handle.cancel_goal_async()
        future.add_done_callback(self._on_cancel_done)
        return "Cancel requested."

    # -- internal callbacks --------------------------------------------

    def _on_feedback(self, feedback_msg: Any) -> None:
        try:
            fb = feedback_msg.feedback
            self.last_feedback = {
                "distance_remaining": getattr(fb, "distance_remaining", 0.0),
                "recoveries": getattr(fb, "number_of_recoveries", 0),
            }
            # Guard against TF extrapolation: goal timestamp becomes stale after
            # the TF buffer window (10s). Cancel before that causes Nav2 to seize.
            if (self.status == self.STATUS_NAVIGATING
                    and not self._timeout_cancel
                    and time.monotonic() - self._goal_start_time > self._nav_timeout_s):
                self._timeout_cancel = True
                _log.warning(
                    "Navigation timeout (%.0fs) to %s — cancelling to avoid TF staleness",
                    self._nav_timeout_s, self.current_target,
                )
                self.cancel_goal()
        except Exception:
            _log.debug("Feedback parse error", exc_info=True)

    def _on_goal_response(self, future: Any) -> None:
        try:
            goal_handle = future.result()
            if not goal_handle or not goal_handle.accepted:
                if self._goal_retry_count < self._max_goal_retries and self._pending_goal is not None:
                    self._goal_retry_count += 1
                    _log.warning(
                        "Nav2 rejected goal to %s — retry %d/%d in 1.5 s",
                        self._pending_location, self._goal_retry_count, self._max_goal_retries,
                    )
                    import threading
                    threading.Timer(1.5, self._retry_send_goal).start()
                else:
                    self.status = self.STATUS_FAILED
                    _log.warning("Nav2 rejected goal to %s (all retries exhausted)", self._pending_location)
                    try:
                        self._enqueue_failure(
                            self._pending_location or "unknown",
                            "Nav2 rejected the navigation goal — the server may still be resetting.",
                        )
                    except Exception:
                        _log.error("enqueue_failure callback failed", exc_info=True)
                return
            self._goal_handle = goal_handle
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self._on_result)
        except Exception:
            self.status = self.STATUS_FAILED
            _log.error("Goal response error", exc_info=True)

    def _retry_send_goal(self) -> None:
        if self._pending_goal is None or self.status != self.STATUS_NAVIGATING:
            return
        _log.info("Retrying goal to %s", self._pending_location)
        send_future = self._action_client.send_goal_async(
            self._pending_goal, feedback_callback=self._on_feedback,
        )
        send_future.add_done_callback(self._on_goal_response)

    def _on_result(self, future: Any) -> None:
        self._goal_handle = None
        try:
            result = future.result()
            status_code = result.status

            if status_code == GoalStatus.STATUS_SUCCEEDED:
                self.status = self.STATUS_ARRIVED
                _log.info("Arrived at %s", self.current_target)
                try:
                    self._enqueue_arrival(self.current_target)
                except Exception:
                    _log.error("enqueue_arrival callback failed", exc_info=True)

            elif status_code == GoalStatus.STATUS_CANCELED:
                if self._timeout_cancel:
                    self._timeout_cancel = False
                    self.status = self.STATUS_FAILED
                    _log.warning("Goal to %s timed out", self.current_target)
                    try:
                        self._enqueue_failure(
                            self.current_target or "unknown",
                            "Navigation timed out — robot may be stuck or the path is too long.",
                        )
                    except Exception:
                        _log.error("enqueue_failure callback failed", exc_info=True)
                else:
                    self.status = self.STATUS_IDLE
                    _log.info("Goal to %s was cancelled", self.current_target)

            else:
                self.status = self.STATUS_FAILED
                reason = self._status_reason(status_code)
                _log.warning("Goal to %s finished with status %d (%s)",
                            self.current_target, status_code, reason)
                try:
                    self._enqueue_failure(self.current_target or "unknown", reason)
                except Exception:
                    _log.error("enqueue_failure callback failed", exc_info=True)
        except Exception:
            self.status = self.STATUS_FAILED
            _log.error("Result callback error", exc_info=True)

    def _on_cancel_done(self, _future: Any) -> None:
        self.status = self.STATUS_IDLE
        self._goal_handle = None
        _log.info("Goal cancel confirmed")

    # -- helpers -------------------------------------------------------

    def _build_pose(
        self,
        frame_id: str,
        x: float, y: float,
        ox: float, oy: float, oz: float, ow: float,
    ) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = frame_id or "map"
        pose.header.stamp = rclpy.time.Time().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.x = float(ox)
        pose.pose.orientation.y = float(oy)
        pose.pose.orientation.z = float(oz)
        pose.pose.orientation.w = float(ow)
        return pose

    @staticmethod
    def _status_reason(status_code: int) -> str:
        return {
            GoalStatus.STATUS_ABORTED: "Navigation aborted (path blocked or planning failed).",
            GoalStatus.STATUS_CANCELED: "Navigation cancelled.",
        }.get(status_code, f"Navigation ended unexpectedly (status {status_code}).")