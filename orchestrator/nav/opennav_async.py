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
from geometry_msgs.msg import PoseStamped
import math

import tf2_ros
from rclpy.duration import Duration

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
        waypoints: dict,
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
        self._last_tag_msg_time: Optional[float] = None
        self.create_subscription(
            PoseStamped, "/detected_dock_pose", self._on_tag_pose, 10,
        )
        self._predock_thread: Optional[threading.Thread] = None
        # TF for pose-refinement at staging
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._waypoints = waypoints
    # -- public API ----------------------------------------------------

    def dock(self, dock_id: str = "sage_charger") -> str:
        """Send a DockRobot goal with pre-search for tag visibility.

        Caller is responsible for navigating to staging first — we never
        use opennav_docking's internal nav-to-staging path because the
        pre-search here needs the robot to already be at staging.
        """
        if self.is_busy():
            return "Already busy with a dock/undock action."

        if not self._dock_client.wait_for_server(timeout_sec=0.5):
            self.status = self.STATUS_FAILED
            self._on_failure("DOCKING_STATION", "Docking action server unavailable.")
            return "Docking server unavailable."

        self.status = self.STATUS_DOCKING
        self._active_action = "dock"

        def _run():
            try:
                # Clear any stale detection timestamp from the Nav2 approach so
                # the "already visible?" check in _step_spin_search only counts
                # detections from the current stationary position.
                self._last_tag_msg_time = None

                if not self._step_spin_search():
                    self.status = self.STATUS_FAILED
                    self._active_action = None
                    _log.warning("Tag not found during step-spin search")
                    self._on_failure(
                        "DOCKING_STATION",
                        "Couldn't find dock marker after Nav2 arrival.",
                    )
                    return

                goal = DockRobot.Goal()
                goal.use_dock_id = True
                goal.dock_id = dock_id
                goal.navigate_to_staging_pose = False

                send_future = self._dock_client.send_goal_async(goal)
                send_future.add_done_callback(self._on_goal_response)

            except Exception as exc:
                self.status = self.STATUS_FAILED
                self._active_action = None
                _log.error("Pre-dock setup error", exc_info=True)
                self._on_failure("DOCKING_STATION", f"Pre-dock error: {exc}")
                
        self._predock_thread = threading.Thread(
            target=_run, daemon=True, name="predock-search",
        )
        self._predock_thread.start()
        return "Dock requested (refining pose first)."

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
        if self._predock_thread and self._predock_thread.is_alive():
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

    def _on_tag_pose(self, _msg) -> None:
        self._last_tag_msg_time = time.monotonic()

    def _tag_seen_recently(self, max_age_s: float = 0.5) -> bool:
        if self._last_tag_msg_time is None:
            return False
        return (time.monotonic() - self._last_tag_msg_time) < max_age_s

    def _lookup_base_link_in_map(self) -> Optional[tuple]:
        """Returns (x, y, yaw) of base_link in the map frame, or None."""
        try:
            import rclpy
            t = self._tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time(),
                timeout=Duration(seconds=0.1),
            )
            x = t.transform.translation.x
            y = t.transform.translation.y
            q = t.transform.rotation
            yaw = 2.0 * math.atan2(q.z, q.w)
            return (x, y, yaw)
        except Exception:
            return None

    def _refine_pose_at_staging(self, timeout_s: float = 6.0,
                                max_forward_m: float = 0.15) -> bool:
        """Drive small residual pose error to zero using AMCL ground truth.

        Caps total forward travel to max_forward_m. Returns True on
        convergence, False on timeout/cap. Best-effort — caller should
        attempt to dock regardless of return value.
        """
        target = self._waypoints.get("DOCKING_STATION")
        if target is None:
            _log.warning("Refine: no DOCKING_STATION waypoint")
            return False

        target_x = target['x']
        target_y = target['y']
        target_yaw = 2.0 * math.atan2(target['oz'], target['ow'])

        start_pose = self._lookup_base_link_in_map()
        if start_pose is None:
            _log.warning("Refine: no TF available, skipping")
            return False
        start_x, start_y, _ = start_pose

        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            pose = self._lookup_base_link_in_map()
            if pose is None:
                time.sleep(0.05)
                continue
            x, y, yaw = pose

            traveled = math.hypot(x - start_x, y - start_y)
            if traveled > max_forward_m:
                _log.info("Refine: forward cap reached (%.2fm)", traveled)
                self._cmd_vel_pub.publish(Twist())
                return False

            dx = target_x - x
            dy = target_y - y
            dyaw = ((target_yaw - yaw + math.pi) % (2 * math.pi)) - math.pi

            # Forward error projected onto current heading
            forward_err = dx * math.cos(yaw) + dy * math.sin(yaw)

            if abs(forward_err) < 0.04 and abs(dyaw) < 0.05:
                _log.info("Refine: converged (forward=%.2fm, dyaw=%.2frad)",
                        forward_err, dyaw)
                self._cmd_vel_pub.publish(Twist())
                return True

            twist = Twist()
            twist.linear.x = max(0.04, min(0.06, 0.5 * forward_err))
            twist.angular.z = max(-0.4, min(0.4, 1.5 * dyaw))
            self._cmd_vel_pub.publish(twist)
            time.sleep(0.05)

        self._cmd_vel_pub.publish(Twist())
        _log.info("Refine: timeout")
        return False
    
    def _ensure_tag_visible(self, timeout_s: float = 12.0) -> bool:
        """In-place spin search for the dock tag. Worst case: full 360°.

        Returns True if the tag became visible and stayed visible briefly,
        False if the timeout elapsed without detection.
        """
        deadline = time.monotonic() + timeout_s

        # If already visible, just confirm it's stable
        time.sleep(0.3)
        if self._tag_seen_recently():
            return True

        spin = Twist()
        spin.angular.z = 0.3   

        while time.monotonic() < deadline:
            if self._tag_seen_recently():
                # Stop and let perception settle before declaring success
                self._cmd_vel_pub.publish(Twist())
                time.sleep(0.5)
                return self._tag_seen_recently()
            self._cmd_vel_pub.publish(spin)
            time.sleep(0.05)

        self._cmd_vel_pub.publish(Twist())
        return False
    

    def _step_spin_search(
        self,
        half_sweep_rad: float = math.radians(45),
        step_rad: float = math.radians(5),
        settle_s: float = 1.0,
        rotation_speed: float = 0.7,
    ) -> bool:
        """Sweep yaw in discrete steps, waiting after each step for detection.

        Sweeps from 0 → +half_sweep → -half_sweep → 0, in `step_rad` increments.
        After each step, stop and wait `settle_s` for AprilTag detection.
        Returns True as soon as the tag is seen, False if the sweep completes
        without detection.

        Total worst-case duration: 4 * half_sweep / step_rad * settle_s.
        With defaults: 4 * 9 * 1.0 = 36 s.
        """
        # Already visible? Done.
        time.sleep(1)
        if self._tag_seen_recently():
            return True

        # Build the step sequence: 0 → +half → -half → 0
        # Each entry is "delta yaw to apply at this step"
        steps_per_quarter = max(1, int(round(half_sweep_rad / step_rad)))
        deltas = (
            [step_rad] * steps_per_quarter +              # 0 → +half
            [-step_rad] * (2 * steps_per_quarter) +       # +half → -half
            [step_rad] * steps_per_quarter                # -half → 0
        )

        for delta in deltas:
            # Rotate by delta. Since we can't rotate in place at slow speeds,
            # we'd have to use the bridge's bumped 0.7 rad/s. Duration of
            # rotation: |delta| / 0.7
            duration = abs(delta) / rotation_speed
            twist = Twist()
            twist.angular.z = math.copysign(rotation_speed, delta)

            end_rotate = time.monotonic() + duration
            while time.monotonic() < end_rotate:
                self._cmd_vel_pub.publish(twist)
                time.sleep(0.05)

            # Stop and settle
            self._cmd_vel_pub.publish(Twist())

            # Wait for settle, checking for detection throughout
            end_settle = time.monotonic() + settle_s
            while time.monotonic() < end_settle:
                if self._tag_seen_recently():
                    # Detection! Brief stabilize, then done.
                    time.sleep(0.3)
                    return self._tag_seen_recently()
                time.sleep(0.05)

        self._cmd_vel_pub.publish(Twist())
        return False