#!/usr/bin/env python3
# serial_bridge.py — simple, extensible STM32 <-> ROS 2 bridge
#
# Host -> MCU  : TYPE=0x01  payload: <ff>           # v, w
# MCU  -> Host : TYPE=0x02  payload: <fffff>        # x, y, theta, v, w   (ODOM ~50 Hz)
#                TYPE=0x03  payload: <ffffff>       # gx, gy, gz, ax, ay, az (IMU 100–200 Hz)
#                TYPE=0x10  payload: ...            # (optional) telemetry, define later
#
# Frame for all directions: [SOF=0x7E][TYPE:1][LEN:1][PAYLOAD:LEN]
# Little-endian floats. No checksum (USB CDC is reliable); easy to add later.

import struct
import threading
import time
import math
import serial

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf_transformations import quaternion_from_euler

SOF          = 0x7E
TYPE_CMD     = 0x01
TYPE_ODOM    = 0x02
TYPE_IMU     = 0x03
TYPE_TLM     = 0x10  # reserved example for future telemetry

FMT_CMD      = '<ff'         # v, w
FMT_ODOM     = '<fffff'      # x, y, th, v, w
FMT_IMU      = '<ffffff'     # gx, gy, gz, ax, ay, az
LEN_CMD      = struct.calcsize(FMT_CMD)
LEN_ODOM     = struct.calcsize(FMT_ODOM)
LEN_IMU      = struct.calcsize(FMT_IMU)

G_STANDARD   = 9.80665  # m/s^2

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')

        # --- Params (minimal; keep it simple) ---
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('frame_odom', 'odom')
        self.declare_parameter('frame_base', 'base_link')
        self.declare_parameter('frame_imu',  'imu_link')
        self.declare_parameter('serial_timeout_s', 0.01)  # short blocking read
        self.declare_parameter('rx_chunk', 128)

        # IMU calibration options
        self.declare_parameter('calib_enable', True)
        self.declare_parameter('calib_duration_s', 5.)
        self.declare_parameter('stationary_gyro_thresh', 0.08)   # rad/s (vector magnitude)
        self.declare_parameter('stationary_accel_g_tol', 2.0)    # m/s^2, | |a| - g | <= tol
        self.declare_parameter('imu_z_positive_down', True)      # az ≈ +g at rest (your convention)
        self.declare_parameter('cov_floor_gyro', 1e-6)
        self.declare_parameter('cov_floor_accel', 1e-5)

        port  = self.get_parameter('port').value
        baud  = int(self.get_parameter('baud').value)
        self.cmd_topic  = self.get_parameter('cmd_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.imu_topic  = self.get_parameter('imu_topic').value
        self.frame_odom = self.get_parameter('frame_odom').value
        self.frame_base = self.get_parameter('frame_base').value
        self.frame_imu  = self.get_parameter('frame_imu').value
        self.serial_timeout_s = float(self.get_parameter('serial_timeout_s').value)
        self.rx_chunk   = int(self.get_parameter('rx_chunk').value)

        # Calibration params
        self.calib_enable = bool(self.get_parameter('calib_enable').value)
        self.calib_duration_s = float(self.get_parameter('calib_duration_s').value)
        self.stationary_gyro_thresh = float(self.get_parameter('stationary_gyro_thresh').value)
        self.stationary_accel_g_tol = float(self.get_parameter('stationary_accel_g_tol').value)
        self.imu_z_positive_down = bool(self.get_parameter('imu_z_positive_down').value)
        self.cov_floor_gyro = float(self.get_parameter('cov_floor_gyro').value)
        self.cov_floor_accel = float(self.get_parameter('cov_floor_accel').value)
        self._g_sign = +1.0 if self.imu_z_positive_down else -1.0

        # --- Serial ---
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baud,
                timeout=self.serial_timeout_s,
                write_timeout=0.2
            )
            time.sleep(1.5)  # let CDC enumerate/reset
            self.get_logger().info(f"Opened {port}@{baud}")
        except Exception as e:
            self.get_logger().error(f"Failed to open {port}@{baud}: {e}")
            raise

        # --- ROS I/O ---
        self.create_subscription(Twist, self.cmd_topic, self.on_cmd_vel, 10)
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.imu_pub  = self.create_publisher(Imu, self.imu_topic, 50)
        # NOTE: We intentionally do NOT publish TF here.
        # robot_localization will own odom->base_link.

        # --- RX worker state ---
        self._shutdown = threading.Event()
        self._buf = bytearray()
        self._rx_thread = threading.Thread(target=self._rx_loop, name='serial_rx', daemon=True)
        self._rx_thread.start()

        # --- IMU calibration state ---
        self._calib_done = (not self.calib_enable)
        self._calib_start = time.monotonic()
        self._calib_window_s = self.calib_duration_s
        self._N = 0
        self._sum_g = [0.0, 0.0, 0.0]
        self._sum2_g = [0.0, 0.0, 0.0]
        self._sum_a = [0.0, 0.0, 0.0]
        self._sum2_a = [0.0, 0.0, 0.0]
        self._bias_g = [0.0, 0.0, 0.0]
        self._bias_a = [0.0, 0.0, 0.0]
        self._cov_g  = [1e-3, 1e-3, 1e-3]   # defaults used until calibrated
        self._cov_a  = [1e-2, 1e-2, 2e-2]

        self.get_logger().info(
            f"TX:{self.cmd_topic} | RX-> ODOM:{self.odom_topic}, IMU:{self.imu_topic} "
            f"| Frames: odom='{self.frame_odom}' base='{self.frame_base}' imu='{self.frame_imu}' | "
            f"IMU calib: enable={self.calib_enable}, window={self._calib_window_s:.1f}s, z_pos_down={self.imu_z_positive_down}"
        )

    # ========== TX: /cmd_vel -> serial (send immediately, no forced rate) ==========
    def on_cmd_vel(self, msg: Twist):
        v = float(msg.linear.x)
        w = float(msg.angular.z)
        try:
            frame = struct.pack('<BBB', SOF, TYPE_CMD, LEN_CMD) + struct.pack(FMT_CMD, v, w)
            self.ser.write(frame)
        except Exception as e:
            self.get_logger().warn(f"serial write failed: {e}")

    # ========== RX thread: read, frame, dispatch ==========
    def _rx_loop(self):
        read = self.ser.read
        buf  = self._buf
        while not self._shutdown.is_set():
            try:
                chunk = read(self.rx_chunk)
                if chunk:
                    buf.extend(chunk)

                # parse as many frames as available
                while True:
                    # find SOF
                    try:
                        i = buf.index(SOF)
                    except ValueError:
                        buf.clear()
                        break

                    # need header
                    if len(buf) - i < 3:
                        if i > 0:
                            del buf[:i]
                        break

                    typ = buf[i+1]
                    ln  = buf[i+2]
                    frame_len = 3 + ln

                    # wait for full frame
                    if len(buf) - i < frame_len:
                        if i > 0:
                            del buf[:i]
                        break

                    payload = buf[i+3 : i+3+ln]
                    # consume frame
                    del buf[:i+frame_len]

                    # dispatch by TYPE
                    if   typ == TYPE_ODOM and ln == LEN_ODOM: self._on_odom(payload)
                    elif typ == TYPE_IMU  and ln == LEN_IMU : self._on_imu(payload)
                    elif typ == TYPE_TLM:  self._on_telemetry(payload)  # placeholder
                    else:
                        # unknown type or len mismatch; continue scanning
                        continue

            except Exception as e:
                self.get_logger().warn(f"serial RX error: {e}")
                time.sleep(0.01)

    # ========== Helpers for calibration ==========
    def _maybe_accumulate_calib(self, gx, gy, gz, ax, ay, az):
        """Accumulate samples only when 'stationary' to avoid mis-calibration."""
        if self._calib_done:
            return

        # Simple stationary detector
        gyro_mag = math.sqrt(gx*gx + gy*gy + gz*gz)
        accel_mag = math.sqrt(ax*ax + ay*ay + az*az)
        # accel magnitude check uses absolute magnitude (always ~9.81), independent of sign convention
        if (gyro_mag <= self.stationary_gyro_thresh) and (abs(accel_mag - G_STANDARD) <= self.stationary_accel_g_tol):
            self._N += 1
            self._sum_g[0]  += gx; self._sum_g[1]  += gy; self._sum_g[2]  += gz
            self._sum2_g[0] += gx*gx; self._sum2_g[1] += gy*gy; self._sum2_g[2] += gz*gz
            self._sum_a[0]  += ax; self._sum_a[1]  += ay; self._sum_a[2]  += az
            self._sum2_a[0] += ax*ax; self._sum2_a[1] += ay*ay; self._sum2_a[2] += az*az

        # Finalize after time window
        if (time.monotonic() - self._calib_start) >= self._calib_window_s:
            if self._N == 0:
                # No valid stationary samples → extend calibration window
                self._calib_start = time.monotonic()
                self.get_logger().warn(
                    "IMU calibration: no stationary samples yet; extending window..."
                )
                return

            N = self._N  # we have samples; use the real N
            mean_g = [self._sum_g[i]/N for i in range(3)]
            mean_a = [self._sum_a[i]/N for i in range(3)]
            var_g  = [max(self.cov_floor_gyro,  self._sum2_g[i]/N - mean_g[i]**2) for i in range(3)]
            var_a  = [max(self.cov_floor_accel, self._sum2_a[i]/N - mean_a[i]**2) for i in range(3)]

            # Gyro bias = mean
            self._bias_g = mean_g

            # Accel bias: KEEP gravity in Z (your Z is positive-down)
            g_keep = +G_STANDARD if self._g_sign > 0 else -G_STANDARD
            self._bias_a = [mean_a[0], mean_a[1], mean_a[2] - g_keep]

            self._cov_g = var_g
            self._cov_a = var_a

            self._calib_done = True
            self.get_logger().info(
                f"IMU calibration complete (N={N}). "
                f"bias_g={['%.4f'%b for b in self._bias_g]}, "
                f"bias_a={['%.4f'%b for b in self._bias_a]}, "
                f"cov_g={['%.2e'%c for c in self._cov_g]}, "
                f"cov_a={['%.2e'%c for c in self._cov_a]}"
            )

    # ========== Handlers ==========
    def _on_odom(self, p: bytes):
        try:
            x, y, th, v, w = struct.unpack(FMT_ODOM, p)
        except struct.error:
            return

        now = self.get_clock().now().to_msg()
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, float(th))

        od = Odometry()
        od.header.stamp = now
        od.header.frame_id = self.frame_odom
        od.child_frame_id  = self.frame_base

        od.pose.pose.position.x = float(x)
        od.pose.pose.position.y = float(y)
        od.pose.pose.position.z = 0.0
        od.pose.pose.orientation.x = qx
        od.pose.pose.orientation.y = qy
        od.pose.pose.orientation.z = qz
        od.pose.pose.orientation.w = qw

        od.twist.twist.linear.x  = float(v)
        od.twist.twist.linear.y  = 0.0
        od.twist.twist.linear.z  = 0.0
        od.twist.twist.angular.x = 0.0
        od.twist.twist.angular.y = 0.0
        od.twist.twist.angular.z = float(w)

        # Diagonal covariances (non-zero for used DoFs; large for unused)
        # pose:  x,     y,     z,    roll,  pitch, yaw
        pose_diag  = [1e-3, 1e-3, 1e3,  1e3,  1e3,  5e-3]
        # twist: vx,    vy,    vz,   vroll, vpitch,vyaw
        twist_diag = [5e-3, 1e3,  1e3,  1e3,  1e3,  5e-3]
        for i, val in enumerate(pose_diag):
            od.pose.covariance[i*6 + i] = float(val)
        for i, val in enumerate(twist_diag):
            od.twist.covariance[i*6 + i] = float(val)

        self.odom_pub.publish(od)

    def _on_imu(self, p: bytes):
        try:
            gx, gy, gz, ax, ay, az = struct.unpack(FMT_IMU, p)
        except struct.error:
            return

        # Accumulate for startup calibration (biased/raw values)
        if not self._calib_done:
            self._maybe_accumulate_calib(gx, gy, gz, ax, ay, az)

        # Apply biases if ready
        if self._calib_done:
            gx -= self._bias_g[0]; gy -= self._bias_g[1]; gz -= self._bias_g[2]
            ax -= self._bias_a[0]; ay -= self._bias_a[1]; az -= self._bias_a[2]

        now = self.get_clock().now().to_msg()
        m = Imu()
        m.header.stamp = now
        m.header.frame_id = self.frame_imu

        # No orientation provided
        m.orientation_covariance[0] = -1.0
        m.orientation_covariance[4] = -1.0
        m.orientation_covariance[8] = -1.0

        # Angular velocity (rad/s)
        m.angular_velocity.x = float(gx)
        m.angular_velocity.y = float(gy)
        m.angular_velocity.z = float(gz)
        # Use measured variances (diagonal only)
        m.angular_velocity_covariance[0] = float(self._cov_g[0])
        m.angular_velocity_covariance[4] = float(self._cov_g[1])
        m.angular_velocity_covariance[8] = float(self._cov_g[2])

        # Linear acceleration (m/s^2) — gravity retained in Z
        m.linear_acceleration.x = float(ax)
        m.linear_acceleration.y = float(ay)
        m.linear_acceleration.z = float(az)
        m.linear_acceleration_covariance[0] = float(self._cov_a[0])
        m.linear_acceleration_covariance[4] = float(self._cov_a[1])
        m.linear_acceleration_covariance[8] = float(self._cov_a[2])

        self.imu_pub.publish(m)

    def _on_telemetry(self, p: bytes):
        # Placeholder: define your telemetry format later.
        pass

    # ========== Cleanup ==========
    def destroy_node(self):
        self._shutdown.set()
        try:
            if self._rx_thread.is_alive():
                self._rx_thread.join(timeout=1.0)
        except Exception:
            pass
        try:
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = SerialBridge()
    try:
        rclpy.spin(node)  # simple executor; our RX thread does the I/O work
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
