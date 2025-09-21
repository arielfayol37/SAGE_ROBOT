# serial_bridge.py
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial, time, struct

class CmdVelToSerial(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_serial')
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('topic', '/turtle1/cmd_vel')

        port  = self.get_parameter('port').get_parameter_value().string_value
        baud  = self.get_parameter('baud').get_parameter_value().integer_value
        topic = self.get_parameter('topic').get_parameter_value().string_value

        # add small write_timeout so we catch stalls instead of blocking forever
        self.ser = serial.Serial(port, baud, timeout=1, write_timeout=0.2)
        time.sleep(1.5)  # some STM32 CDCs reset on open

        self.create_subscription(Twist, topic, self.on_cmd_vel, 10)
        self.get_logger().info(f"Opened {port}@{baud}, listening to {topic}")

    def on_cmd_vel(self, msg: Twist):
        v = float(msg.linear.x)
        w = float(msg.angular.z)
        # MCU wants: 0x78 ('x'=120), then two little-endian float32s
        pkt = struct.pack('<Bff', 120, v, w)
        try:
            n = self.ser.write(pkt)  # should be 9
            if n != 9:
                self.get_logger().warn(f"Short write: {n}/9 bytes")
        except Exception as e:
            self.get_logger().error(f"Serial write failed: {e}")

def main():
    rclpy.init()
    node = CmdVelToSerial()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try: node.ser.close()
        except Exception: pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
