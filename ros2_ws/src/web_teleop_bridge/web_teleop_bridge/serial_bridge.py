# serial_bridge.py
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial, time, struct
from threading import Thread, Event
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')

        # ---- Params ----
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')   # default to /cmd_vel
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('frame_odom', 'odom')
        self.declare_parameter('frame_base', 'base_link')

        port  = self.get_parameter('port').get_parameter_value().string_value
        baud  = self.get_parameter('baud').get_parameter_value().integer_value
        self.cmd_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.frame_odom = self.get_parameter('frame_odom').get_parameter_value().string_value
        self.frame_base = self.get_parameter('frame_base').get_parameter_value().string_value

        # ---- Serial ----
        self.ser = serial.Serial(port, baud, timeout=0.01, write_timeout=0.2)
        time.sleep(1.5)  # some CDC ACMs reset on open
        self.get_logger().info(f"Opened {port}@{baud} | tx:{self.cmd_topic} rx:{self.odom_topic}")

        # ---- ROS I/O ----
        self.create_subscription(Twist, self.cmd_topic, self.on_cmd_vel, 10)
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 20)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ---- Reader thread ----
        self._stop_evt = Event()
        self._thread = Thread(target=self.reader_loop, daemon=True)
        self._thread.start()

    # ================== WRITE: /cmd_vel -> serial ==================
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

    # ================== READ: serial -> /odom + TF ==================
    def reader_loop(self):
        """
        Expect binary packets:
          0x79 (b'y') + 5 x float32:
            x, y, theta, v, omega
        Total: 21 bytes
        """
        packet_size = 21
        while not self._stop_evt.is_set():
            try:
                # Sync to start byte
                start = self.ser.read(1)
                if start != b'y':
                    continue
                data = self.ser.read(packet_size - 1)
                if len(data) != packet_size - 1:
                    continue
                x, y, theta, v, omega = struct.unpack('<fffff', data)
                self.publish_odom_and_tf(x, y, theta, v, omega)
            except Exception as e:
                self.get_logger().warn(f"serial read error: {e}")

    def publish_odom_and_tf(self, x, y, theta, v, w):
        now = self.get_clock().now().to_msg()
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, theta)

        # Odometry message (nice to have for tools; TF is what nav really uses)
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.frame_odom
        odom.child_frame_id  = self.frame_base
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x  = v
        odom.twist.twist.angular.z = w
        # simple covariances (tune later)
        odom.pose.covariance[0] = 1e-3
        odom.pose.covariance[7] = 1e-3
        odom.pose.covariance[35] = 5e-3
        odom.twist.covariance[0] = 5e-3
        odom.twist.covariance[35] = 5e-3
        self.odom_pub.publish(odom)

        # TF: odom -> base_link
        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = self.frame_odom
        tf.child_frame_id  = self.frame_base
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf)

    # lifecycle cleanup
    def destroy_node(self):
        self._stop_evt.set()
        try:
            if self._thread.is_alive():
                self._thread.join(timeout=0.5)
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
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
