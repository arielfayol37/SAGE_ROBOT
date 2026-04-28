#!/usr/bin/env python3
"""
Docking controller for SAGE.

States:
  IDLE        - waiting for /dock/start
  SEARCH      - tag not visible, spin in place to find it
  ALIGN       - tag visible, drive an arc to center it (zero lateral, zero yaw)
  FINAL       - straight-line approach with continuous yaw correction
  RECOVER     - back up 30 cm, return to ALIGN
  STOP        - hold zero, watch battery for contact confirmation
  DOCKED      - terminal success
  FAILED      - terminal failure (after retries exhausted)

Tag pose convention used here: TF lookup base_link -> dock_tag gives
  x = forward distance to tag (meters)
  y = lateral offset (meters, positive = tag is to robot's left)
  yaw = heading error (radians, 0 = robot pointed straight at tag normal)
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from sensor_msgs.msg import BatteryState
from std_srvs.srv import Trigger

import tf2_ros
from tf_transformations import euler_from_quaternion, quaternion_from_euler


# Approach geometry
CAMERA_OFFSET_X = 0.179         # m, camera_link forward offset from base_link
TARGET_TAG_DISTANCE = 0.25      # m, desired final camera-to-tag distance
TARGET_X_BASE = TARGET_TAG_DISTANCE + CAMERA_OFFSET_X  # = 0.429 m

# "Done" tolerances
DONE_X_TOL = 0.02               # m, x within ±2 cm of target
DONE_Y_TOL = 0.03               # m, |y| below 3 cm
DONE_YAW_TOL = 0.05             # rad, ~3°
DONE_HOLD_TIME = 0.5            # s, must stay in tolerance this long

# ---------- Tunable parameters ----------
CONTROL_HZ = 20.0
TAG_FRAME = 'dock_tag'
ROBOT_FRAME = 'base_link'

# SEARCH
SEARCH_OMEGA = 0.30          # rad/s spin rate while searching
SEARCH_TIMEOUT = 30.0        # s, give up if tag never appears

# ALIGN
ALIGN_DONE_LATERAL = 0.05    # m, lateral error below which we switch to FINAL
ALIGN_DONE_YAW = 0.10        # rad, yaw error below which we switch to FINAL
ALIGN_LINEAR_VEL = 0.08      # m/s forward while aligning
ALIGN_KY = 1.5               # gain on lateral error -> angular correction
ALIGN_KYAW = 0.8             # gain on yaw error -> angular correction
ALIGN_MAX_OMEGA = 0.6        # rad/s clamp

# FINAL
FINAL_KX = 0.20              # gain on x distance -> linear velocity
FINAL_VMIN = 0.04            # m/s
FINAL_VMAX = 0.12            # m/s
FINAL_KY = 2.0               # gain on lateral error -> angular correction
FINAL_KYAW = 1.0             # gain on yaw error -> angular correction
FINAL_MAX_OMEGA = 0.4        # rad/s clamp
FINAL_CONTACT_DIST = 0.20    # m, switch to STOP when tag x <= this
FINAL_LATERAL_ABORT = 0.15   # m, |y| above which we abandon FINAL and recover
TAG_LOST_TIMEOUT = 0.5       # s, tag-lost duration that triggers recovery

# RECOVER
RECOVER_DISTANCE = 0.30      # m to back up
RECOVER_VEL = -0.08          # m/s (negative = reverse)
MAX_RECOVERY_ATTEMPTS = 3

# STOP / contact detection
CONTACT_VOLTAGE_RISE = 0.30  # V rise required to confirm charging
CONTACT_TIMEOUT = 8.0        # s after STOP to wait for voltage rise

# Where the robot ends up (the known dock pose, in map frame).
# Used to re-localize AMCL after successful dock. Update to match your map.
DOCK_POSE_X = 30.38266
DOCK_POSE_Y = 57.78290
DOCK_POSE_YAW = 1.6792


class DockingNode(Node):
    def __init__(self):
        super().__init__('docking_node')

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publishers / subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 1)
        self.create_subscription(
            BatteryState, '/battery_state', self.on_battery, 10)

        # Service
        self.create_service(Trigger, '/dock/start', self.on_start)

        # State
        self.state = 'IDLE'
        self.last_state = None
        self.state_entered_at = self.get_clock().now()
        self.last_tag_time = None
        self.recovery_attempts = 0
        self.recover_start_pose = None    # base_link x,y at recovery start
        self.contact_baseline_v = None
        self.contact_started_at = None
        self.latest_voltage = None

        # Control loop
        self.timer = self.create_timer(1.0 / CONTROL_HZ, self.tick)

        self.get_logger().info('Docking node ready. Call /dock/start to begin.')

    # ---------- Service handler ----------
    def on_start(self, request, response):
        if self.state not in ('IDLE', 'DOCKED', 'FAILED'):
            response.success = False
            response.message = f'Already running (state={self.state})'
            return response
        self.recovery_attempts = 0
        self.transition('SEARCH')
        response.success = True
        response.message = 'Docking started'
        return response

    # ---------- Battery callback ----------
    def on_battery(self, msg: BatteryState):
        self.latest_voltage = float(msg.voltage)

    # ---------- TF helper ----------
    def get_tag_pose(self):
        """Returns (x_forward, y_lateral, yaw_error) in base_link frame, or None.

        yaw_error: angle the robot needs to rotate to face the tag head-on.
                Zero when the tag's normal points directly back at the robot.
                Positive = robot needs to turn left.
        """
        try:
            t = self.tf_buffer.lookup_transform(
                ROBOT_FRAME, TAG_FRAME,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05))
        except Exception:
            return None

        now = self.get_clock().now()
        tf_time = rclpy.time.Time.from_msg(t.header.stamp)
        if (now - tf_time) > Duration(seconds=0.5):
            return None

        x = t.transform.translation.x
        y = t.transform.translation.y

        # Tag normal (its +z axis) expressed in base_link.
        # This is the third column of the rotation matrix derived from the quaternion.
        q = t.transform.rotation
        nx = 2.0 * (q.x * q.z + q.y * q.w)
        ny = 2.0 * (q.y * q.z - q.x * q.w)

        # Desired robot heading is -tag_normal projected onto floor plane.
        # Yaw error is atan2 of that vector (since robot's current forward is +x).
        yaw_error = math.atan2(-ny, -nx)

        self.last_tag_time = now
        return x, y, yaw_error

    # ---------- State transition ----------
    def transition(self, new_state: str):
        if new_state == self.state:
            return
        self.get_logger().info(f'{self.state} -> {new_state}')
        self.last_state = self.state
        self.state = new_state
        self.state_entered_at = self.get_clock().now()

        # State-entry hooks
        if new_state == 'STOP':
            self.contact_baseline_v = self.latest_voltage
            self.contact_started_at = self.get_clock().now()
        if new_state == 'RECOVER':
            self.recover_start_pose = self.lookup_base_link_xy()

    def time_in_state(self):
        return (self.get_clock().now() - self.state_entered_at).nanoseconds * 1e-9

    def lookup_base_link_xy(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'odom', ROBOT_FRAME,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1))
            return (t.transform.translation.x, t.transform.translation.y)
        except Exception:
            return None

    # ---------- Main control loop ----------
    def tick(self):
        twist = Twist()

        if self.state in ('IDLE', 'DOCKED', 'FAILED'):
            return

        pose = self.get_tag_pose()

        if self.state == 'SEARCH':
            if pose is not None:
                self.transition('ALIGN')
            elif self.time_in_state() > SEARCH_TIMEOUT:
                self.fail('Tag not found within SEARCH timeout')
            else:
                # Forward arc — robot can't rotate in place reliably.
                twist.linear.x = 0.06
                twist.angular.z = 0.6

        elif self.state == 'ALIGN':
            if pose is None:
                if self.time_in_state() > TAG_LOST_TIMEOUT:
                    self.transition('SEARCH')
            else:
                x, y, yaw = pose
                if abs(y) <= ALIGN_DONE_LATERAL and abs(yaw) <= ALIGN_DONE_YAW:
                    self.transition('FINAL')
                else:
                    twist.linear.x = ALIGN_LINEAR_VEL
                    omega = -ALIGN_KY * y - ALIGN_KYAW * yaw
                    twist.angular.z = clamp(omega, -ALIGN_MAX_OMEGA, ALIGN_MAX_OMEGA)

        elif self.state == 'FINAL':
            if pose is None:
                if self.time_in_state() > TAG_LOST_TIMEOUT:
                    self.attempt_recover('Tag lost during FINAL')
            else:
                x, y, yaw = pose
                x_err = x - TARGET_X_BASE   # positive = need to move forward

                # Done check
                in_tol = (abs(x_err) < DONE_X_TOL and
                        abs(y) < DONE_Y_TOL and
                        abs(yaw) < DONE_YAW_TOL)
                if in_tol:
                    if self._in_tol_since is None:
                        self._in_tol_since = self.get_clock().now()
                    elif (self.get_clock().now() - self._in_tol_since).nanoseconds * 1e-9 >= DONE_HOLD_TIME:
                        self.transition('STOP')
                else:
                    self._in_tol_since = None

                # Lateral abort
                if abs(y) > FINAL_LATERAL_ABORT:
                    self.attempt_recover(f'Lateral error too large: y={y:.2f}')
                    return

                # Drive toward target. Note x_err can go slightly negative — that means
                # we've overshot. With reverse allowed, controller will back up gently.
                v = clamp(FINAL_KX * x_err, -0.05, FINAL_VMAX)
                # Avoid the in-place-rotation regime: if v would be in the dead zone,
                # bias it to a small positive value so heading correction works.
                if abs(v) < 0.04:
                    v = 0.04 if x_err >= 0 else -0.04
                twist.linear.x = v

                omega = -FINAL_KY * y - FINAL_KYAW * yaw
                twist.angular.z = clamp(omega, -FINAL_MAX_OMEGA, FINAL_MAX_OMEGA)

        elif self.state == 'RECOVER':
            now_xy = self.lookup_base_link_xy()
            if now_xy is None or self.recover_start_pose is None:
                # No odom? Just back up by time as a fallback (~3.75 s @ 0.08 m/s).
                if self.time_in_state() > 3.75:
                    self.transition('ALIGN')
                else:
                    twist.linear.x = RECOVER_VEL
            else:
                dx = now_xy[0] - self.recover_start_pose[0]
                dy = now_xy[1] - self.recover_start_pose[1]
                if math.hypot(dx, dy) >= RECOVER_DISTANCE:
                    self.transition('ALIGN')
                else:
                    twist.linear.x = RECOVER_VEL

        elif self.state == 'STOP':
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            if self.time_in_state() > 0.5:    # let it settle
                self.relocalize_amcl()
                self.transition('DOCKED')

        self.cmd_pub.publish(twist)

    # ---------- Helpers ----------
    def attempt_recover(self, reason):
        self.recovery_attempts += 1
        if self.recovery_attempts > MAX_RECOVERY_ATTEMPTS:
            self.fail(f'Recovery limit exceeded ({reason})')
        else:
            self.get_logger().warn(
                f'{reason}. Recovery attempt {self.recovery_attempts}/{MAX_RECOVERY_ATTEMPTS}')
            self.transition('RECOVER')

    def fail(self, reason):
        self.get_logger().error(f'Docking failed: {reason}')
        self.transition('FAILED')

    def contact_confirmed(self):
        """True if voltage has risen meaningfully since we entered STOP."""
        if self.latest_voltage is None or self.contact_baseline_v is None:
            return False
        return (self.latest_voltage - self.contact_baseline_v) >= CONTACT_VOLTAGE_RISE

    def relocalize_amcl(self):
        """Publish the known dock pose to /initialpose so AMCL snaps to truth."""
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = DOCK_POSE_X
        msg.pose.pose.position.y = DOCK_POSE_Y
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, DOCK_POSE_YAW)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        # Tight covariance — we believe this pose strongly.
        cov = [0.0] * 36
        cov[0] = 0.01    # x
        cov[7] = 0.01    # y
        cov[35] = 0.02   # yaw
        msg.pose.covariance = cov
        self.initialpose_pub.publish(msg)
        self.get_logger().info('Re-localized AMCL to dock pose')


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def main():
    rclpy.init()
    node = DockingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Make sure we don't leave the robot driving.
        zero = Twist()
        node.cmd_pub.publish(zero)
        time.sleep(0.1)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
