#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler
import serial, time, struct

START_TX = 0x78               # host -> MCU: 0x78 + v + w (float32 LE)
START_RX = 0x79               # MCU  -> host: 0x79 + x y th v w (float32 LE)
RX_FMT  = '<fffff'
RX_SIZE = 1 + struct.calcsize(RX_FMT)  # 21 bytes

class SerialCmdVelAndRx(Node):
    def __init__(self):
        super().__init__('serial_cmdvel_and_rx')

        # --- params ---
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('rx_topic',  'mcu_rx')
        self.declare_parameter('odom_topic','/odom')
        self.declare_parameter('frame_odom','odom')
        self.declare_parameter('frame_base','base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('poll_ms', 5.0)  # serial poll period

        port       = self.get_parameter('port').value
        baud       = self.get_parameter('baud').value
        cmd_topic  = self.get_parameter('cmd_topic').value
        rx_topic   = self.get_parameter('rx_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.frame_odom = self.get_parameter('frame_odom').value
        self.frame_base = self.get_parameter('frame_base').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        poll_ms    = float(self.get_parameter('poll_ms').value)

        # --- serial (single owner) ---
        self.ser = serial.Serial(port, baud, timeout=0.0, write_timeout=0.2)
        time.sleep(1.5)  # allow CDC-ACM reset
        self.get_logger().info(f"Opened {port}@{baud} | TX:{cmd_topic} RX:{rx_topic} ODOM:{self.odom_topic}")

        # --- ROS I/O ---
        self.create_subscription(Twist, cmd_topic, self.on_cmd_vel, 10)
        self.rx_pub   = self.create_publisher(Float32MultiArray, rx_topic, 10)
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- state ---
        self.buf = bytearray()

        # --- poll timer (no thread needed) ---
        self.create_timer(poll_ms / 1000.0, self.poll_serial)

    # ========= TX: /cmd_vel -> serial =========
    def on_cmd_vel(self, msg: Twist):
        v = float(msg.linear.x)
        w = float(msg.angular.z)
        pkt = struct.pack('<Bff', START_TX, v, w)  # 9 bytes
        try:
            n = self.ser.write(pkt)
            if n != 9:
                self.get_logger().warn(f"short write {n}/9")
        except Exception as e:
            self.get_logger().error(f"serial write failed: {e}")

    # ========= RX: serial -> /mcu_rx + /odom (+ TF) =========
    def poll_serial(self):
        try:
            chunk = self.ser.read(128)
            if chunk:
                self.buf.extend(chunk)

            while True:
                # find start byte
                try:
                    i = self.buf.index(START_RX)
                except ValueError:
                    self.buf.clear()
                    return

                # wait for full frame
                if len(self.buf) - i < RX_SIZE:
                    if i > 0:
                        del self.buf[:i]
                    return

                frame   = self.buf[i:i+RX_SIZE]
                payload = frame[1:1+struct.calcsize(RX_FMT)]
                try:
                    x, y, th, v, w = struct.unpack(RX_FMT, payload)
                except struct.error:
                    # bad payload; drop start and continue
                    del self.buf[:i+1]
                    continue

                # consume the frame
                del self.buf[:i+RX_SIZE]

                # publish simple array (debug/plotting)
                arr = Float32MultiArray()
                arr.data = [x, y, th, v, w]
                self.rx_pub.publish(arr)

                # publish odometry + TF (odom -> base_link)
                self.publish_odom_and_tf(x, y, th, v, w)

        except Exception as e:
            self.get_logger().warn(f"serial poll error: {e}")

    def publish_odom_and_tf(self, x, y, theta, v, omega):
        now = self.get_clock().now().to_msg()
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, float(theta))

        # Odometry message
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

        # Fill diagonal covariances (others left zero)
        # pose: [x y z roll pitch yaw] -> indices 0,7,14,21,28,35
        pose_diag = [1e-3, 1e-3, 1e3, 1e3, 1e3, 5e-3]
        for i, val in enumerate(pose_diag):
            odom.pose.covariance[i*7] = val
        # twist: [vx vy vz vroll vpitch vyaw]
        twist_diag = [5e-3, 1e3, 1e3, 1e3, 1e3, 5e-3]
        for i, val in enumerate(twist_diag):
            odom.twist.covariance[i*7] = val

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
