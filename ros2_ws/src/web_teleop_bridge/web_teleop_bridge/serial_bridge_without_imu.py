#!/usr/bin/env python3
# serial_bridge_without_imu.py
#
# Host -> MCU  : 0x78 + <ff>                      # v, w   (same TX format you already use)
# MCU  -> Host : [0x7E][TYPE=0x02][LEN=20][<fffff>]
#                 payload = x, y, theta, v, w
#
# Publishes:
#   /odom                  nav_msgs/Odometry
#   odom -> base_link      TF
#
# Ignores any other incoming frame types.

import struct
import threading
import time
import serial
import glob

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler

SOF       = 0x7E
TYPE_ODOM = 0x02
FMT_ODOM  = '<fffff'      # x, y, th, v, w
LEN_ODOM  = struct.calcsize(FMT_ODOM)
TYPE_BATTERY = 0x04

FMT_BATTERY  = '<f'          # voltage
LEN_BATTERY  = struct.calcsize(FMT_BATTERY)

class SerialBridgeNoImu(Node):
    def __init__(self):
        super().__init__('serial_bridge_no_imu')

        # Parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('frame_odom', 'odom')
        self.declare_parameter('frame_base', 'base_link')
        self.declare_parameter('serial_timeout_s', 0.01)
        self.declare_parameter('rx_chunk', 128)
        self.declare_parameter('battery_topic', '/battery_state')
        # Optional: lets you show a % on the BatteryState message.
        # Leave min >= max to skip percentage estimation.
        self.declare_parameter('battery_v_min', 0.0)
        self.declare_parameter('battery_v_max', 0.0)
        self.declare_parameter('battery_cell_count', 0)   # 0 = unknown

        self.battery_topic = self.get_parameter('battery_topic').value
        self.battery_v_min = float(self.get_parameter('battery_v_min').value)
        self.battery_v_max = float(self.get_parameter('battery_v_max').value)
        self.battery_cells = int(self.get_parameter('battery_cell_count').value)
        port_param = self.get_parameter('port').value
        baud = int(self.get_parameter('baud').value)
        self.cmd_topic = self.get_parameter('cmd_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.frame_odom = self.get_parameter('frame_odom').value
        self.frame_base = self.get_parameter('frame_base').value
        self.serial_timeout_s = float(self.get_parameter('serial_timeout_s').value)
        self.rx_chunk = int(self.get_parameter('rx_chunk').value)

        # Prefer detected ACM port if present
        detected_port = next(iter(glob.glob('/dev/ttyACM*')), None)
        port = detected_port if detected_port is not None else port_param

        # Serial
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baud,
                timeout=self.serial_timeout_s,
                write_timeout=0.2,
            )
            time.sleep(1.5)  # let USB CDC settle/reset
            self.get_logger().info(f"Opened {port}@{baud}")
        except Exception as e:
            self.get_logger().error(f"Failed to open {port}@{baud}: {e}")
            raise

        # ROS I/O
        self.create_subscription(Twist, self.cmd_topic, self.on_cmd_vel, 10)
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.battery_pub = self.create_publisher(BatteryState, self.battery_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # RX worker
        self._shutdown = threading.Event()
        self._buf = bytearray()
        self._rx_thread = threading.Thread(
            target=self._rx_loop,
            name='serial_rx_no_imu',
            daemon=True,
        )
        self._rx_thread.start()

        self.get_logger().info(
            f"TX:{self.cmd_topic} | RX->ODOM:{self.odom_topic} | "
            f"Frames: odom='{self.frame_odom}' base='{self.frame_base}'"
        )

    def on_cmd_vel(self, msg: Twist):
        v = float(msg.linear.x)
        w = float(msg.angular.z)

        min_inplace_w = 0.7
        stationary_v_thresh = 0.05

        # If robot is basically not translating, enforce a minimum usable turn speed
        if abs(v) < stationary_v_thresh and 0.0 < abs(w) < min_inplace_w:
            w = min_inplace_w if w > 0.0 else -min_inplace_w
        # Keep your existing TX format exactly:
        # 0x78 + float32(v) + float32(w)
        pkt = struct.pack('<Bff', 0x78, v, w)

        try:
            n = self.ser.write(pkt)
            if n != 9:
                self.get_logger().warn(f"Short serial write {n}/9")
        except Exception as e:
            self.get_logger().error(f"Serial write failed: {e}")

    def _rx_loop(self):
        read = self.ser.read
        buf = self._buf

        while not self._shutdown.is_set():
            try:
                chunk = read(self.rx_chunk)
                if chunk:
                    buf.extend(chunk)

                while True:
                    try:
                        i = buf.index(SOF)
                    except ValueError:
                        buf.clear()
                        break

                    # Need at least SOF + TYPE + LEN
                    if len(buf) - i < 3:
                        if i > 0:
                            del buf[:i]
                        break

                    typ = buf[i + 1]
                    ln = buf[i + 2]
                    frame_len = 3 + ln

                    # Wait for full frame
                    if len(buf) - i < frame_len:
                        if i > 0:
                            del buf[:i]
                        break

                    payload = buf[i + 3:i + 3 + ln]
                    del buf[:i + frame_len]

                    if typ == TYPE_ODOM and ln == LEN_ODOM:
                        self._on_odom(payload)
                    elif typ == TYPE_BATTERY and ln == LEN_BATTERY: 
                        self._on_battery(payload)
                    else:
                        # Ignore all non-odom frames
                        continue

            except Exception as e:
                self.get_logger().warn(f"Serial RX error: {e}")
                time.sleep(0.01)

    def _on_odom(self, payload: bytes):
        try:
            x, y, th, v, w = struct.unpack(FMT_ODOM, payload)
        except struct.error:
            return

        now = self.get_clock().now().to_msg()
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, float(th))

        # Publish /odom
        od = Odometry()
        od.header.stamp = now
        od.header.frame_id = self.frame_odom
        od.child_frame_id = self.frame_base

        od.pose.pose.position.x = float(x)
        od.pose.pose.position.y = float(y)
        od.pose.pose.position.z = 0.0
        od.pose.pose.orientation.x = qx
        od.pose.pose.orientation.y = qy
        od.pose.pose.orientation.z = qz
        od.pose.pose.orientation.w = qw

        od.twist.twist.linear.x = float(v)
        od.twist.twist.linear.y = 0.0
        od.twist.twist.linear.z = 0.0
        od.twist.twist.angular.x = 0.0
        od.twist.twist.angular.y = 0.0
        od.twist.twist.angular.z = float(w)

        pose_diag = [1e-3, 1e-3, 1e3, 1e3, 1e3, 5e-3]
        twist_diag = [5e-3, 1e3, 1e3, 1e3, 1e3, 5e-3]

        for i, val in enumerate(pose_diag):
            od.pose.covariance[i * 6 + i] = float(val)

        for i, val in enumerate(twist_diag):
            od.twist.covariance[i * 6 + i] = float(val)

        self.odom_pub.publish(od)

        # Publish TF: odom -> base_link
        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = self.frame_odom
        tf.child_frame_id = self.frame_base

        tf.transform.translation.x = float(x)
        tf.transform.translation.y = float(y)
        tf.transform.translation.z = 0.0
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(tf)
        
    def _on_battery(self, p: bytes):
        try:
            (voltage,) = struct.unpack(FMT_BATTERY, p)
        except struct.error:
            return

        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_base  # or a dedicated 'battery_link' if you have one

        msg.voltage = float(voltage)

        # Unknowns per REP-140: report NaN rather than 0.0
        nan = float('nan')
        msg.temperature     = nan
        msg.current         = nan
        msg.charge          = nan
        msg.capacity        = nan
        msg.design_capacity = nan

        # Optional linear percentage estimate if the user gave us min/max.
        # Linear mapping is crude for LiPo/LiIon but fine as a dashboard readout.
        if self.battery_v_max > self.battery_v_min:
            pct = (voltage - self.battery_v_min) / (self.battery_v_max - self.battery_v_min)
            msg.percentage = float(max(0.0, min(1.0, pct)))
        else:
            msg.percentage = nan

        msg.power_supply_status     = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        msg.power_supply_health     = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
        msg.present = True

        # Per-cell info: only publish if we know the cell count.
        if self.battery_cells > 0:
            per_cell = voltage / self.battery_cells
            msg.cell_voltage    = [per_cell] * self.battery_cells
            msg.cell_temperature = [nan]      * self.battery_cells
        else:
            msg.cell_voltage     = []
            msg.cell_temperature = []

        self.battery_pub.publish(msg)

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
    node = SerialBridgeNoImu()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()