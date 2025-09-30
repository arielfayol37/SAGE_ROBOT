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
        self.imu_pub  = self.create_publisher(Imu, self.imu_topic, 10)
        # NOTE: We intentionally do NOT publish TF here.
        # robot_localization will own odom->base_link.

        # --- RX worker state ---
        self._shutdown = threading.Event()
        self._buf = bytearray()
        self._rx_thread = threading.Thread(target=self._rx_loop, name='serial_rx', daemon=True)
        self._rx_thread.start()

        self.get_logger().info(f"TX:{self.cmd_topic} | RX-> ODOM:{self.odom_topic}, IMU:{self.imu_topic} "
                               f"| Frames: odom='{self.frame_odom}' base='{self.frame_base}' imu='{self.frame_imu}'")

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
        # Starter covariances (tune/measure later)
        m.angular_velocity_covariance[0] = 1e-3
        m.angular_velocity_covariance[4] = 1e-3
        m.angular_velocity_covariance[8] = 1e-3

        # Linear acceleration (m/s^2)
        m.linear_acceleration.x = float(ax)
        m.linear_acceleration.y = float(ay)
        m.linear_acceleration.z = float(az)
        m.linear_acceleration_covariance[0] = 1e-2
        m.linear_acceleration_covariance[4] = 1e-2
        m.linear_acceleration_covariance[8] = 2e-2

        self.imu_pub.publish(m)

    def _on_telemetry(self, p: bytes):
        # Placeholder: define your telemetry format later.
        # For now, just ignore or log occasionally.
        # Example: self.get_logger().info_once("Telemetry frame received")
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
