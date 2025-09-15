import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import websockets, asyncio, json

class ControlBridge(Node):
    def __init__(self):
        super().__init__("control_bridge")
        self.pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

    async def ws_loop(self):
        uri = "ws://localhost:8765"
        async with websockets.connect(uri) as ws:
            # 👇 send role hello so server will relay browser messages to us
            await ws.send(json.dumps({"role": "robot"}))

            async for msg in ws:
                data = json.loads(msg)
                if data.get("type") == "control":
                    twist = Twist()
                    twist.linear.x = float(data.get("x", 0.0))
                    twist.angular.z = float(data.get("z", 0.0))
                    self.pub.publish(twist)

def main():
    rclpy.init()
    node = ControlBridge()
    try:
        asyncio.run(node.ws_loop())
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
