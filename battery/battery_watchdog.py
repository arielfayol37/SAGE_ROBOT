#!/usr/bin/env python3
# battery_watchdog.py — watch /battery_state and email when voltage goes low.
#
# Run:
#   ros2 run <your_pkg> battery_watchdog.py
# or:
#   python3 battery_watchdog.py
#
# Deps: rclpy, sensor_msgs, requests
#   pip install requests

import threading
import time

import requests
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState


class BatteryWatchdog(Node):
    def __init__(self):
        super().__init__('battery_watchdog')

        # --- Parameters ---
        self.declare_parameter('battery_topic', '/battery_state')
        self.declare_parameter('voltage_threshold', 12.5)     # trigger below this
        self.declare_parameter('recovery_threshold', 13.0)    # clear alarm above this (hysteresis)
        self.declare_parameter('email_endpoint', 'http://localhost:8004/api/kb/send-emails')
        self.declare_parameter('email_subject', 'SAGE ROBOT LOW BATTERY')
        self.declare_parameter('email_text',
            'My battery level is low, please plug me in.')
        # Debounce: require this many consecutive low readings before firing.
        # At 50 Hz publish rate, 10 samples = 200 ms of sustained low voltage.
        self.declare_parameter('consecutive_low_required', 10)
        # Don't re-send reminders more often than this (seconds).
        self.declare_parameter('cooldown_s', 1200.0)
        self.declare_parameter('http_timeout_s', 5.0)

        self.battery_topic   = self.get_parameter('battery_topic').value
        self.low_thresh      = float(self.get_parameter('voltage_threshold').value)
        self.recovery_thresh = float(self.get_parameter('recovery_threshold').value)
        self.endpoint        = self.get_parameter('email_endpoint').value
        self.subject         = self.get_parameter('email_subject').value
        self.text            = self.get_parameter('email_text').value
        self.consec_required = int(self.get_parameter('consecutive_low_required').value)
        self.cooldown_s      = float(self.get_parameter('cooldown_s').value)
        self.http_timeout    = float(self.get_parameter('http_timeout_s').value)

        if self.recovery_thresh < self.low_thresh:
            self.get_logger().warn(
                f"recovery_threshold ({self.recovery_thresh}) < voltage_threshold "
                f"({self.low_thresh}); forcing recovery = threshold + 0.5V"
            )
            self.recovery_thresh = self.low_thresh + 0.5

        # --- State ---
        self._consec_low    = 0
        self._alarm_active  = False      # latched until voltage recovers
        self._last_email_ts = 0.0        # time.monotonic()

        self.create_subscription(BatteryState, self.battery_topic, self._on_battery, 10)

        self.get_logger().info(
            f"Watching '{self.battery_topic}': low<{self.low_thresh:.2f}V, "
            f"recovery>={self.recovery_thresh:.2f}V, "
            f"debounce={self.consec_required} msgs, "
            f"cooldown={self.cooldown_s:.0f}s, "
            f"endpoint={self.endpoint}"
        )

    # ========== Subscriber callback ==========
    def _on_battery(self, msg: BatteryState):
        v = float(msg.voltage)

        # Ignore NaN / zero readings (sensor not ready, or bridge reported unknown)
        if v != v or v <= 0.0:
            return

        # Clear latch once voltage recovers comfortably above threshold
        if self._alarm_active and v >= self.recovery_thresh:
            self._alarm_active = False
            self._consec_low = 0
            self.get_logger().info(
                f"Battery recovered: {v:.2f}V >= {self.recovery_thresh:.2f}V. Alarm cleared."
            )
            return

        # Debounce counter
        if v < self.low_thresh:
            self._consec_low += 1
        else:
            self._consec_low = 0
            return

        if self._consec_low < self.consec_required:
            return

        # We're in a sustained-low state. Respect cooldown between emails.
        now = time.monotonic()
        if self._alarm_active and (now - self._last_email_ts) < self.cooldown_s:
            return

        # Fire. Update state optimistically so we don't re-queue every message
        # while the HTTP request is in flight.
        self._last_email_ts = now
        self._alarm_active  = True

        self.get_logger().warn(
            f"Low battery detected: {v:.2f}V (< {self.low_thresh:.2f}V). Sending alert email."
        )
        # Offload HTTP to a thread so a slow/dead endpoint doesn't stall the executor.
        threading.Thread(target=self._send_email, args=(v,), daemon=True).start()

    # ========== HTTP worker ==========
    def _send_email(self, voltage: float):
        payload = {
            'subject': self.subject,
            'text':    f"{self.text} (current voltage: {voltage:.2f} V)",
        }
        try:
            resp = requests.post(self.endpoint, json=payload, timeout=self.http_timeout)
            if 200 <= resp.status_code < 300:
                self.get_logger().info(f"Alert email sent (HTTP {resp.status_code}).")
            else:
                body = (resp.text or '')[:200]
                self.get_logger().error(
                    f"Email endpoint returned HTTP {resp.status_code}: {body}"
                )
        except requests.RequestException as e:
            self.get_logger().error(f"Email POST failed: {e}")


def main():
    rclpy.init()
    node = BatteryWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()