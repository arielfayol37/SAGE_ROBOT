#!/usr/bin/env python3
"""
serial_bridge.py — ROS 2 <-> MCU serial bridge

Bidirectional framed protocol:

    [SOF=0x7E][VER=0x01][TYPE][LEN][PAYLOAD x LEN][CRC16_LO][CRC16_HI]

CRC-16/CCITT-FALSE (poly=0x1021, init=0xFFFF, no xorout) over
[VER, TYPE, LEN, PAYLOAD]. Framing recovery scans for the next SOF on
any CRC / length mismatch, so a single corrupted or dropped byte can't
desync the link.

Frame types
-----------
    0x01 CMD     host -> mcu    float32 v, float32 w            (8 B)
    0x02 ODOM    mcu -> host    float32 x, y, theta, v, w       (20 B)
    0x03 IMU     mcu -> host    float32 gx,gy,gz, ax,ay,az      (24 B)
    0x04 BATTERY mcu -> host    float32 voltage                 (4 B)

Publishes
---------
    /odom            nav_msgs/Odometry
    /imu/data_raw    sensor_msgs/Imu
    /battery_state   sensor_msgs/BatteryState
    TF: odom -> base_link (optional, via ~publish_tf)

Subscribes
----------
    /cmd_vel         geometry_msgs/Twist
"""

import glob
import math
import struct
import threading
import time
from typing import Optional

import serial
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, Imu
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler


# ---------------------------------------------------------------------------
# Protocol
# ---------------------------------------------------------------------------
SOF = 0x7E
PROTOCOL_VERSION = 0x01

TYPE_CMD = 0x01
TYPE_ODOM = 0x02
TYPE_IMU = 0x03
TYPE_BATTERY = 0x04

FMT_CMD = '<ff'
FMT_ODOM = '<fffff'
FMT_IMU = '<ffffff'
FMT_BATTERY = '<f'

LEN_CMD = struct.calcsize(FMT_CMD)          # 8
LEN_ODOM = struct.calcsize(FMT_ODOM)        # 20
LEN_IMU = struct.calcsize(FMT_IMU)          # 24
LEN_BATTERY = struct.calcsize(FMT_BATTERY)  # 4

FRAME_OVERHEAD = 6  # SOF + VER + TYPE + LEN + CRC(2)
MIN_FRAME_LEN = FRAME_OVERHEAD
MAX_PAYLOAD_LEN = 64  # sanity limit for this protocol

EXPECTED_LEN = {
    TYPE_ODOM: LEN_ODOM,
    TYPE_IMU: LEN_IMU,
    TYPE_BATTERY: LEN_BATTERY,
}


def crc16_ccitt(data, crc: int = 0xFFFF) -> int:
    """CRC-16/CCITT-FALSE (poly=0x1021, init=0xFFFF)."""
    for b in data:
        crc ^= (b & 0xFF) << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def build_frame(typ: int, payload: bytes) -> bytes:
    """Wrap a payload in the framing protocol."""
    if len(payload) > 0xFF:
        raise ValueError("payload too large")
    header = bytes((PROTOCOL_VERSION, typ, len(payload)))
    body = header + payload
    crc = crc16_ccitt(body)
    return bytes((SOF,)) + body + bytes((crc & 0xFF, (crc >> 8) & 0xFF))


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------
class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')

        # ---- parameters ----
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('imu_topic', '/imu/data_raw')
        self.declare_parameter('battery_topic', '/battery_state')
        self.declare_parameter('frame_odom', 'odom')
        self.declare_parameter('frame_base', 'base_link')
        self.declare_parameter('frame_imu', 'imu_link')
        self.declare_parameter('serial_timeout_s', 0.05)
        self.declare_parameter('rx_chunk', 256)
        self.declare_parameter('reconnect_period_s', 1.0)
        self.declare_parameter('odom_stale_warn_s', 1.0)
        self.declare_parameter('publish_tf', True)
        # battery display helpers
        self.declare_parameter('battery_v_min', 0.0)
        self.declare_parameter('battery_v_max', 0.0)
        self.declare_parameter('battery_cell_count', 0)
        # diagonal covariances (x, y, z, roll, pitch, yaw)
        self.declare_parameter('pose_covariance_diag',
                               [1e-3, 1e-3, 1e6, 1e6, 1e6, 5e-3])
        self.declare_parameter('twist_covariance_diag',
                               [5e-3, 1e6, 1e6, 1e6, 1e6, 5e-3])
        self.declare_parameter('imu_angular_vel_variance', 0.02)
        self.declare_parameter('imu_linear_accel_variance', 0.04)

        self._port_param = self.get_parameter('port').value
        self._baud = int(self.get_parameter('baud').value)
        self._cmd_topic = self.get_parameter('cmd_topic').value
        self._odom_topic = self.get_parameter('odom_topic').value
        self._imu_topic = self.get_parameter('imu_topic').value
        self._battery_topic = self.get_parameter('battery_topic').value
        self._frame_odom = self.get_parameter('frame_odom').value
        self._frame_base = self.get_parameter('frame_base').value
        self._frame_imu = self.get_parameter('frame_imu').value
        self._serial_timeout = float(self.get_parameter('serial_timeout_s').value)
        self._rx_chunk = int(self.get_parameter('rx_chunk').value)
        self._reconnect_period = float(self.get_parameter('reconnect_period_s').value)
        self._odom_stale_s = float(self.get_parameter('odom_stale_warn_s').value)
        self._publish_tf = bool(self.get_parameter('publish_tf').value)
        self._battery_v_min = float(self.get_parameter('battery_v_min').value)
        self._battery_v_max = float(self.get_parameter('battery_v_max').value)
        self._battery_cells = int(self.get_parameter('battery_cell_count').value)
        self._pose_cov = list(self.get_parameter('pose_covariance_diag').value)
        self._twist_cov = list(self.get_parameter('twist_covariance_diag').value)
        self._imu_gyro_var = float(self.get_parameter('imu_angular_vel_variance').value)
        self._imu_accel_var = float(self.get_parameter('imu_linear_accel_variance').value)

        # ---- state ----
        self.ser: Optional[serial.Serial] = None
        self._ser_lock = threading.Lock()
        self._shutdown = threading.Event()
        self._rx_buf = bytearray()
        self._last_odom_time = 0.0
        self._stats = {
            'rx_frames': 0,
            'rx_crc_errors': 0,
            'rx_resyncs': 0,
            'rx_bad_len': 0,
            'tx_errors': 0,
            'reconnects': 0,
        }

        # ---- publishers / subscribers ----
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.odom_pub = self.create_publisher(Odometry, self._odom_topic, sensor_qos)
        self.imu_pub = self.create_publisher(Imu, self._imu_topic, sensor_qos)
        self.battery_pub = self.create_publisher(BatteryState, self._battery_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self._publish_tf else None

        self.create_subscription(Twist, self._cmd_topic, self.on_cmd_vel, 10)

        # periodic checks
        self.create_timer(0.5, self._liveness_check)
        self.create_timer(30.0, self._log_stats)

        # ---- RX thread ----
        self._rx_thread = threading.Thread(
            target=self._rx_loop, name='serial_rx', daemon=True)
        self._rx_thread.start()

        self.get_logger().info(
            f"serial_bridge started | TX:{self._cmd_topic} "
            f"RX:{self._odom_topic},{self._imu_topic},{self._battery_topic}"
        )

    # -----------------------------------------------------------------
    # Serial connection management
    # -----------------------------------------------------------------
    def _pick_port(self) -> str:
        candidates = sorted(glob.glob('/dev/ttyACM*'))
        if candidates:
            return candidates[0]
        return self._port_param

    def _open_serial(self) -> bool:
        port = self._pick_port()
        try:
            ser = serial.Serial(
                port=port,
                baudrate=self._baud,
                timeout=self._serial_timeout,
                write_timeout=0.2,
                exclusive=True,
            )
            # CDC-ACM devices reset on DTR toggle; wait for MCU to come back.
            time.sleep(1.5)
            try:
                ser.reset_input_buffer()
                ser.reset_output_buffer()
            except Exception:
                pass
            with self._ser_lock:
                self.ser = ser
            self._rx_buf.clear()
            self.get_logger().info(f"Opened {port}@{self._baud}")
            return True
        except Exception as e:
            self.get_logger().warn(f"Open {port}@{self._baud} failed: {e}")
            return False

    def _close_serial(self):
        with self._ser_lock:
            s = self.ser
            self.ser = None
        if s is not None:
            try:
                s.close()
            except Exception:
                pass

    # -----------------------------------------------------------------
    # TX path
    # -----------------------------------------------------------------
    def on_cmd_vel(self, msg: Twist):
        v = float(msg.linear.x)
        w = float(msg.angular.z)
        if not (math.isfinite(v) and math.isfinite(w)):
            self.get_logger().warn("Non-finite cmd_vel; ignoring.")
            return
        
        min_inplace_w = 0.7
        stationary_v_thresh = 0.05

        # If robot is basically not translating, enforce a minimum usable turn speed
        if abs(v) < stationary_v_thresh and 0.0 < abs(w) < min_inplace_w:
            w = min_inplace_w if w > 0.0 else -min_inplace_w

        frame = build_frame(TYPE_CMD, struct.pack(FMT_CMD, v, w))

        with self._ser_lock:
            s = self.ser
        if s is None:
            return
        try:
            s.write(frame)
        except Exception as e:
            self._stats['tx_errors'] += 1
            self.get_logger().warn(f"Serial write failed: {e}")
            # close; RX thread will reconnect
            self._close_serial()

    # -----------------------------------------------------------------
    # RX path
    # -----------------------------------------------------------------
    def _rx_loop(self):
        while not self._shutdown.is_set():
            # (re)connect as needed
            with self._ser_lock:
                have_port = self.ser is not None
            if not have_port:
                if not self._open_serial():
                    # Sleep with shutdown-awareness
                    if self._shutdown.wait(self._reconnect_period):
                        return
                    continue
                else:
                    self._stats['reconnects'] += 1

            # read
            try:
                with self._ser_lock:
                    s = self.ser
                if s is None:
                    continue
                chunk = s.read(self._rx_chunk)
            except Exception as e:
                self.get_logger().warn(f"Serial read failed: {e}; reconnecting.")
                self._close_serial()
                if self._shutdown.wait(self._reconnect_period):
                    return
                continue

            if chunk:
                self._rx_buf.extend(chunk)
                # Keep buffer bounded if someone upstream floods garbage
                if len(self._rx_buf) > 4096:
                    del self._rx_buf[:-1024]
                self._parse_frames()

    def _parse_frames(self):
        buf = self._rx_buf
        while True:
            # locate next candidate SOF
            try:
                i = buf.index(SOF)
            except ValueError:
                # nothing useful
                buf.clear()
                return
            if i > 0:
                # discard any garbage preceding SOF
                del buf[:i]
                self._stats['rx_resyncs'] += 1

            # need at least SOF + VER + TYPE + LEN
            if len(buf) < 4:
                return

            ver = buf[1]
            typ = buf[2]
            ln = buf[3]

            if ver != PROTOCOL_VERSION or ln > MAX_PAYLOAD_LEN:
                # bogus header; skip this SOF and scan again
                del buf[:1]
                self._stats['rx_resyncs'] += 1
                continue

            frame_len = 4 + ln + 2  # SOF + header(3) + payload + CRC(2)
            if len(buf) < frame_len:
                return  # wait for more bytes

            body = buf[1:4 + ln]
            crc_rx = buf[4 + ln] | (buf[5 + ln] << 8)
            crc_calc = crc16_ccitt(body)

            if crc_rx != crc_calc:
                del buf[:1]
                self._stats['rx_crc_errors'] += 1
                continue

            expected = EXPECTED_LEN.get(typ)
            if expected is not None and expected != ln:
                del buf[:frame_len]
                self._stats['rx_bad_len'] += 1
                continue

            payload = bytes(buf[4:4 + ln])
            del buf[:frame_len]
            self._stats['rx_frames'] += 1

            if typ == TYPE_ODOM:
                self._on_odom(payload)
            elif typ == TYPE_IMU:
                self._on_imu(payload)
            elif typ == TYPE_BATTERY:
                self._on_battery(payload)
            # else: unknown type — silently ignore

    # -----------------------------------------------------------------
    # Frame handlers
    # -----------------------------------------------------------------
    def _on_odom(self, payload: bytes):
        try:
            x, y, th, v, w = struct.unpack(FMT_ODOM, payload)
        except struct.error:
            return
        if not all(math.isfinite(f) for f in (x, y, th, v, w)):
            self.get_logger().warn("Non-finite odom; dropping.")
            return

        now = self.get_clock().now().to_msg()
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, float(th))

        od = Odometry()
        od.header.stamp = now
        od.header.frame_id = self._frame_odom
        od.child_frame_id = self._frame_base

        od.pose.pose.position.x = float(x)
        od.pose.pose.position.y = float(y)
        od.pose.pose.position.z = 0.0
        od.pose.pose.orientation.x = qx
        od.pose.pose.orientation.y = qy
        od.pose.pose.orientation.z = qz
        od.pose.pose.orientation.w = qw

        od.twist.twist.linear.x = float(v)
        od.twist.twist.angular.z = float(w)

        # Diagonal covariance
        for i in range(6):
            od.pose.covariance[i * 7] = float(self._pose_cov[i])
            od.twist.covariance[i * 7] = float(self._twist_cov[i])

        self.odom_pub.publish(od)

        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header.stamp = now
            tf.header.frame_id = self._frame_odom
            tf.child_frame_id = self._frame_base
            tf.transform.translation.x = float(x)
            tf.transform.translation.y = float(y)
            tf.transform.translation.z = 0.0
            tf.transform.rotation.x = qx
            tf.transform.rotation.y = qy
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf)

        self._last_odom_time = time.monotonic()

    def _on_imu(self, payload: bytes):
        try:
            gx, gy, gz, ax, ay, az = struct.unpack(FMT_IMU, payload)
        except struct.error:
            return
        if not all(math.isfinite(f) for f in (gx, gy, gz, ax, ay, az)):
            return

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_imu
        msg.angular_velocity.x = float(gx)
        msg.angular_velocity.y = float(gy)
        msg.angular_velocity.z = float(gz)
        msg.linear_acceleration.x = float(ax)
        msg.linear_acceleration.y = float(ay)
        msg.linear_acceleration.z = float(az)
        # orientation not provided
        msg.orientation_covariance[0] = -1.0
        for i in range(3):
            msg.angular_velocity_covariance[i * 4] = self._imu_gyro_var
            msg.linear_acceleration_covariance[i * 4] = self._imu_accel_var
        self.imu_pub.publish(msg)

    def _on_battery(self, payload: bytes):
        try:
            (voltage,) = struct.unpack(FMT_BATTERY, payload)
        except struct.error:
            return
        if not math.isfinite(voltage):
            return

        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_base
        msg.voltage = float(voltage)

        nan = float('nan')
        msg.temperature = nan
        msg.current = nan
        msg.charge = nan
        msg.capacity = nan
        msg.design_capacity = nan

        if self._battery_v_max > self._battery_v_min:
            pct = (voltage - self._battery_v_min) / (self._battery_v_max - self._battery_v_min)
            msg.percentage = float(max(0.0, min(1.0, pct)))
        else:
            msg.percentage = nan

        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
        msg.present = True

        if self._battery_cells > 0:
            per_cell = voltage / self._battery_cells
            msg.cell_voltage = [per_cell] * self._battery_cells
            msg.cell_temperature = [nan] * self._battery_cells
        else:
            msg.cell_voltage = []
            msg.cell_temperature = []

        self.battery_pub.publish(msg)

    # -----------------------------------------------------------------
    # Housekeeping
    # -----------------------------------------------------------------
    def _liveness_check(self):
        if self._last_odom_time == 0.0:
            return
        age = time.monotonic() - self._last_odom_time
        if age > self._odom_stale_s:
            self.get_logger().warn(
                f"No odometry for {age:.1f}s; MCU link may be stalled.",
                throttle_duration_sec=5.0,
            )

    def _log_stats(self):
        self.get_logger().info(
            "link stats: "
            f"frames={self._stats['rx_frames']} "
            f"crc_err={self._stats['rx_crc_errors']} "
            f"resyncs={self._stats['rx_resyncs']} "
            f"bad_len={self._stats['rx_bad_len']} "
            f"tx_err={self._stats['tx_errors']} "
            f"reconnects={self._stats['reconnects']}"
        )

    def destroy_node(self):
        self._shutdown.set()
        try:
            if self._rx_thread.is_alive():
                self._rx_thread.join(timeout=1.5)
        except Exception:
            pass
        self._close_serial()
        super().destroy_node()


def main():
    rclpy.init()
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()