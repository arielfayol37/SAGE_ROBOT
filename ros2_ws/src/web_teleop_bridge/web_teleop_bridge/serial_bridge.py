#!/usr/bin/env python3
# serial_cmdvel_and_rx.py — old behavior (odom + TF), new wire protocol
#
# Host -> MCU  : [0x7E][TYPE=0x01][LEN=8][<ff>]                  # v, w (float32 LE)
# MCU  -> Host : [0x7E][TYPE=0x02][LEN=20][<fffff>]              # x, y, theta, v, w (ODOM)
#                [0x7E][TYPE=0x03][LEN=24][<ffffff>] (ignored)   # IMU frames are ignored here
#
# We publish:
#   /odom (nav_msgs/Odometry)
#   odom -> base_link TF (optional, like your old bridge)
#
# This intentionally does NOT publish /imu/data or fuse anything.

import struct
import time
import serial

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler

# --- New protocol constants (match your new bridge/MCU) ---
SOF       = 0x7E
TYPE_CMD  = 0x01
TYPE_ODOM = 0x02
TYPE_IMU  = 0x03  # we ignore these frames

FMT_CMD   = '<ff'       # v, w
FMT_ODOM  = '<fffff'    # x, y, theta, v, w
LEN_CMD   = struct.calcsize(FMT_CMD)    # 8
LEN_ODOM  = struct.calcsize(FMT_ODOM)   # 20
LEN_IMU   = struct.calcsize('<ffffff')  # 24

class SerialCmdVelAndRx(Node):
    def __init__(self):
        super().__init__('serial_cmdvel_and_rx')

        # --- params (same spirit as your old file) ---
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('rx_topic',  'mcu_rx')     # debug array of odom frame
        self.declare_parameter('odom_topic','/odom')
        self.declare_parameter('frame_odom','odom')
        self.declare_parameter('frame_base','base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('poll_ms', 5.0)            # serial poll period
        self.declare_parameter('rx_chunk', 128)           # read chunk size

        port        = self.get_parameter('port').value
        baud        = int(self.get_parameter('baud').value)
        cmd_topic   = self.get_parameter('cmd_topic').value
        rx_topic    = self.get_parameter('rx_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.frame_odom = self.get_parameter('frame_odom').value
        self.frame_base = self.get_parameter('frame_base').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        poll_ms     = float(self.get_parameter('poll_ms').value)
        self.rx_chunk = int(self.get_parameter('rx_chunk').value)

        # --- serial ---
        self.ser = serial.Serial(port, baud, timeout=0.0, write_timeout=0.2)
        time.sleep(1.5)  # CDC reset settle
        self.get_logger().info(f"Opened {port}@{baud} | TX:{cmd_topic} ODOM:{self.odom_topic} | proto=SOF/TYPE/LEN")

        # --- ROS I/O ---
        self.create_subscription(Twist, cmd_topic, self.on_cmd_vel, 10)
        self.rx_pub   = self.create_publisher(Float32MultiArray, rx_topic, 10)
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- state ---
        self.buf = bytearray()

        # --- timer poller (simple, no threads) ---
        self.create_timer(poll_ms / 1000.0, self.poll_serial)

    # ========= TX: /cmd_vel -> serial (new protocol) =========
    def on_cmd_vel(self, msg: Twist):
        v = float(msg.linear.x)
        w = float(msg.angular.z)
        # keep your existing binary TX format: 0x78 'x' + v + w (float32 little-endian)
        pkt = struct.pack('<Bff', 0x78, v, w)
        try:
            n = self.ser.write(pkt)
            if n != 9:
                self.get_logger().warn(f"short write {n}/9")
        except Exception as e:
            self.get_logger().error(f"serial write failed: {e}")

    # ========= RX: serial -> /odom (+ TF) =========
    def poll_serial(self):
        try:
            chunk = self.ser.read(self.rx_chunk)
            if chunk:
                self.buf.extend(chunk)

            # parse as many frames as possible
            while True:
                # find SOF
                try:
                    i = self.buf.index(SOF)
                except ValueError:
                    self.buf.clear()
                    return

                # need header (TYPE + LEN)
                if len(self.buf) - i < 3:
                    if i > 0:
                        del self.buf[:i]
                    return

                typ = self.buf[i+1]
                ln  = self.buf[i+2]
                frame_len = 3 + ln

                # wait for full frame
                if len(self.buf) - i < frame_len:
                    if i > 0:
                        del self.buf[:i]
                    return

                payload = self.buf[i+3 : i+3+ln]
                # consume frame
                del self.buf[:i+frame_len]

                # dispatch
                if   typ == TYPE_ODOM and ln == LEN_ODOM:
                    self._handle_odom(payload)
                elif typ == TYPE_IMU:
                    # IMU frames present on the wire but ignored in this bridge
                    continue
                else:
                    # unknown or length mismatch; skip
                    continue

        except Exception as e:
            self.get_logger().warn(f"serial poll error: {e}")

    # ========= ODOM handler =========
    def _handle_odom(self, p: bytes):
        try:
            x, y, th, v, w = struct.unpack(FMT_ODOM, p)
        except struct.error:
            return

        # debug/plot array like before
        arr = Float32MultiArray()
        arr.data = [x, y, th, v, w]
        self.rx_pub.publish(arr)

        # publish odometry + optional TF (odom -> base_link)
        self.publish_odom_and_tf(x, y, th, v, w)

    def publish_odom_and_tf(self, x, y, theta, v, omega):
        now = self.get_clock().now().to_msg()
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, float(theta))

        # Odometry
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.frame_odom
        odom.child_frame_id  = self.frame_base

        odom.pose.pose.position.x = float(x)
        odom.pose.pose.position.y = float(y)
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x  = float(v)
        odom.twist.twist.angular.z = float(omega)

        # Diagonal covariances (same as your old bridge)
        # pose: [x y z roll pitch yaw] -> indices 0,7,14,21,28,35
        pose_diag = [1e-3, 1e-3, 1e3, 1e3, 1e3, 5e-3]
        for i, val in enumerate(pose_diag):
            odom.pose.covariance[i*7] = float(val)
        # twist: [vx vy vz vroll vpitch vyaw]
        twist_diag = [5e-3, 1e3, 1e3, 1e3, 1e3, 5e-3]
        for i, val in enumerate(twist_diag):
            odom.twist.covariance[i*7] = float(val)

        self.odom_pub.publish(odom)

        # TF: odom -> base_link
        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp = now
            tf.header.frame_id = self.frame_odom
            tf.child_frame_id  = self.frame_base
            tf.transform.translation.x = float(x)
            tf.transform.translation.y = float(y)
            tf.transform.translation.z = 0.0
            tf.transform.rotation.x = qx
            tf.transform.rotation.y = qy
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf)

    def destroy_node(self):
        try:
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()

def main():
    rclpy.init()
    node = SerialCmdVelAndRx()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
