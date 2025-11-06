#!/usr/bin/env python3
"""
SAGE Hub — minimal ROS2 <-> WebSocket bridge

WS endpoints:
  /ws/teleop   (browser -> hub):  {type:"control", x:<m/s>, z:<rad/s>}
  /ws/status   (hub -> browser):  {type:"battery", ...}, {type:"ui_state", ...}
                                  also replies {type:"pong", t} if it receives {type:"ping", t}

ROS:
  Pub: /cmd_vel                (geometry_msgs/Twist)
  Sub: /battery_state          (sensor_msgs/BatteryState)
  Sub: /sage/ui_state_json     (std_msgs/String with JSON)
"""

import os
import time
import json
import asyncio
import threading
from typing import Set, Dict, Optional

import websockets
from websockets.server import WebSocketServerProtocol
from websockets.exceptions import ConnectionClosedOK, ConnectionClosedError

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String as RosString

# ---- Config ----
HOST = os.environ.get("SAGE_WS_HOST", "0.0.0.0")
PORT = int(os.environ.get("SAGE_WS_PORT", "8765"))
CONTROL_HZ_MAX = float(os.environ.get("SAGE_CONTROL_HZ_MAX", "30"))  # rate-limit /cmd_vel
PING_INTERVAL = 20
PING_TIMEOUT = 20


class SageHubNode(Node):
    def __init__(self):
        super().__init__("sage_hub")

        # client sets
        self.teleop_clients: Set[WebSocketServerProtocol] = set()
        self.status_clients: Set[WebSocketServerProtocol] = set()

        # snapshots to push on new connections
        self.last_battery: Optional[Dict] = None
        self.last_ui_state: Optional[Dict] = None

        # ROS pubs/subs
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # QoS: battery = sensor-ish; ui_state = reliable control/status
        bat_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        ui_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,  # set to TRANSIENT_LOCAL if your publisher latches
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(BatteryState, "/battery_state", self._on_battery, bat_qos)
        self.create_subscription(RosString, "/sage/ui_state_json", self._on_ui_json, ui_qos)

        # control rate limiter
        self._last_cmd_time = 0.0
        self._min_cmd_dt = 1.0 / max(1e-3, CONTROL_HZ_MAX)

        # the asyncio loop the websockets server runs on (assigned in run_server)
        self.loop: Optional[asyncio.AbstractEventLoop] = None

    # ------------------ ROS -> WS ------------------

    def _on_battery(self, msg: BatteryState):
        try:
            pct = float(msg.percentage) if msg.percentage is not None else 0.0
        except Exception:
            pct = 0.0
        payload = {
            "type": "battery",
            "pct": max(0.0, min(1.0, pct)),
            "voltage": float(msg.voltage) if msg.voltage is not None else None,
            "is_charging": (msg.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING),
        }
        # self.get_logger().info(f"Battery update: {payload}")
        self.last_battery = payload
        # Post to the WS server loop (set in run_server) — guard until available
        if self.loop is not None:
            asyncio.run_coroutine_threadsafe(self._broadcast_status(payload), self.loop)

    def _on_ui_json(self, msg: RosString):
        try:
            data = json.loads(msg.data)
            # self.get_logger().info(f"UI state update: {data}")
            if isinstance(data, dict):
                data["type"] = "ui_state"
                self.last_ui_state = data
                if self.loop is not None:
                    asyncio.run_coroutine_threadsafe(self._broadcast_status(data), self.loop)
        except Exception as e:
            self.get_logger().warn(f"ui_state_json parse error: {e}")  

    # ------------------ WS helpers ------------------

    async def _send_json(self, ws: WebSocketServerProtocol, payload: Dict):
        try:
            await ws.send(json.dumps(payload))
        except Exception:
            pass  # client likely gone; cleanup happens on close

    async def _broadcast_status(self, payload: Dict):
        """Broadcast to all status clients."""
        if not self.status_clients:
            return
        data = json.dumps(payload)
        dead = []
        for ws in list(self.status_clients):
            try:
                await ws.send(data)
            except Exception:
                dead.append(ws)
        for ws in dead:
            self.status_clients.discard(ws)

    async def _on_connect(self, ws: WebSocketServerProtocol, room: str):
        if room == "teleop":
            self.teleop_clients.add(ws)
        else:
            self.status_clients.add(ws)
            # push latest snapshots so the UI populates instantly
            if self.last_battery:
                await self._send_json(ws, self.last_battery)
            if self.last_ui_state:
                await self._send_json(ws, self.last_ui_state)

    async def _on_disconnect(self, ws: WebSocketServerProtocol):
        self.teleop_clients.discard(ws)
        self.status_clients.discard(ws)

    # ------------------ Main WS handler ------------------

    async def ws_handler(self, ws: WebSocketServerProtocol):
        # websockets>=12 provides the request path on the protocol object
        path = getattr(ws, "path", "/") or "/"

        # simple router by path suffix; default to status if unknown
        if path.endswith("/ws/teleop") or path == "/ws/teleop":
            room = "teleop"
        elif path.endswith("/ws/status") or path == "/ws/status":
            room = "status"
        else:
            room = "status"  # change to "teleop" to treat legacy root as teleop

        await self._on_connect(ws, room)

        try:
            async for raw in ws:
                try:
                    msg = json.loads(raw)
                except Exception:
                    continue

                mtype = msg.get("type")

                # latency check
                if mtype == "ping":
                    await self._send_json(ws, {"type": "pong", "t": msg.get("t")})
                    continue

                # teleop control
                if mtype == "control":
                    now = time.time()
                    if now - self._last_cmd_time >= self._min_cmd_dt:
                        x = float(msg.get("x", 0.0))
                        z = float(msg.get("z", 0.0))
                        tw = Twist()
                        tw.linear.x = x
                        tw.angular.z = z
                        self.cmd_pub.publish(tw)
                        self._last_cmd_time = now
                    continue

                # on-demand snapshot
                if mtype == "get_status":
                    if self.last_battery:
                        await self._send_json(ws, self.last_battery)
                    if self.last_ui_state:
                        await self._send_json(ws, self.last_ui_state)
                    continue

        except (ConnectionClosedOK, ConnectionClosedError):
            # Normal browser refresh/close; suppress traceback noise
            pass
        finally:
            await self._on_disconnect(ws)


def main():
    # Start ROS spinning in a background thread
    rclpy.init()
    node = SageHubNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    async def run_server():
        # IMPORTANT: bind the running loop so ROS callbacks can post into it
        node.loop = asyncio.get_running_loop()
        async with websockets.serve(
            node.ws_handler, HOST, PORT,
            ping_interval=PING_INTERVAL, ping_timeout=PING_TIMEOUT
        ):
            node.get_logger().info(
                f"SAGE hub WS listening on ws://{HOST}:{PORT} (paths: /ws/teleop, /ws/status)"
            )
            await asyncio.Future()  # run forever

    try:
        asyncio.run(run_server())
    finally:
        executor.shutdown()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            # Can happen if shutdown already called during Ctrl+C
            pass


if __name__ == "__main__":
    main()
