"""
doa_server.py  —  SAGE Direction-of-Arrival WebSocket broadcaster

Reads the auto-selected beam azimuth from the XVF3800 USB mic array
and broadcasts it as JSON over a WebSocket on port 8766.

The face UI (index_face.tsx) connects to ws://<host>:8766/ws/doa
and moves the pupils to follow the speaker.

Message format sent to clients:
    { "type": "doa", "angle_deg": 42.7 }

    0°   = directly behind the robot
    90°  = robot's left side
    180° = directly in front (front-facing speaker → eyes look forward)
    270° = robot's right side

Dependencies:
    pip install pyusb websockets

Usage:
    python doa_server.py
    python doa_server.py --port 8766 --interval 0.08
"""

import asyncio
import json
import math
import struct
import time
import argparse
import logging
from typing import Set

try:
    import usb.core
    import usb.util
    USB_AVAILABLE = True
except ImportError:
    USB_AVAILABLE = False
    logging.warning("pyusb not available — running in simulation mode.")

try:
    import websockets
    from websockets.server import WebSocketServerProtocol
except ImportError:
    raise SystemExit("websockets package required: pip install websockets")

# ── XVF3800 USB constants ─────────────────────────────────────────────
VID = 0x2886
PID = 0x001A
AEC_AZIMUTH_RESID = 33
AEC_AZIMUTH_CMDID = 0x80 | 75
NUM_FLOATS        = 4
STATUS_PLUS_DATA_LEN = 1 + 4 * NUM_FLOATS
TIMEOUT_MS        = 100_000

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [doa_server] %(levelname)s %(message)s",
)
log = logging.getLogger("doa_server")

# ── Shared state ──────────────────────────────────────────────────────
_clients:   Set["WebSocketServerProtocol"] = set()
_last_angle: float = 0.0          # degrees, updated by reader loop


def _read_aec_azimuth(dev) -> tuple:
    """Read 4 azimuth floats from XVF3800; returns (beam1, beam2, free_run, auto_sel) in radians."""
    resp = dev.ctrl_transfer(
        usb.util.CTRL_IN | usb.util.CTRL_TYPE_VENDOR | usb.util.CTRL_RECIPIENT_DEVICE,
        0,
        AEC_AZIMUTH_CMDID,
        AEC_AZIMUTH_RESID,
        STATUS_PLUS_DATA_LEN,
        TIMEOUT_MS,
    )
    data = resp.tobytes()[1:1 + 4 * NUM_FLOATS]
    return struct.unpack("<ffff", data)


async def _doa_reader_loop(interval: float) -> None:
    """Continuously poll the mic array and update _last_angle."""
    global _last_angle

    dev = None
    if USB_AVAILABLE:
        dev = usb.core.find(idVendor=VID, idProduct=PID)
        if dev is None:
            log.warning("XVF3800 not found — running in simulation mode.")
        else:
            log.info("XVF3800 connected.")
            try:
                if dev.is_kernel_driver_active(3):
                    dev.detach_kernel_driver(3)
            except Exception:
                pass

    sim_angle = 180.0  # start facing front

    while True:
        try:
            if dev is not None:
                _, _, _, auto_sel = _read_aec_azimuth(dev)
                _last_angle = (math.degrees(auto_sel) + 360.0) % 360.0
            else:
                # Simulation: slowly sweep back and forth
                # sim_angle = (sim_angle + 1.2) % 360.0
                _last_angle = sim_angle

            msg = json.dumps({"type": "doa", "angle_deg": round(_last_angle, 2)})
            dead: Set["WebSocketServerProtocol"] = set()
            for ws in list(_clients):
                try:
                    await ws.send(msg)
                except Exception:
                    dead.add(ws)
            _clients.difference_update(dead)

        except Exception as exc:
            log.error("DOA read error: %s", exc)

        await asyncio.sleep(interval)


async def _ws_handler(websocket: "WebSocketServerProtocol") -> None:
    """Handle a new face-UI WebSocket connection."""
    log.info("Client connected from %s", websocket.remote_address)
    _clients.add(websocket)
    try:
        # Send the current angle immediately so the UI doesn't wait
        await websocket.send(
            json.dumps({"type": "doa", "angle_deg": round(_last_angle, 2)})
        )
        # Keep the connection alive — the reader loop does the broadcasting
        await websocket.wait_closed()
    finally:
        _clients.discard(websocket)
        log.info("Client disconnected from %s", websocket.remote_address)


async def _main(host: str, port: int, interval: float) -> None:
    log.info("Starting DOA WebSocket server on ws://%s:%d/ws/doa", host, port)
    log.info("Poll interval: %.0f ms", interval * 1000)

    async with websockets.serve(_ws_handler, host, port):
        await _doa_reader_loop(interval)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="SAGE DOA WebSocket server")
    parser.add_argument("--host",     default="0.0.0.0",  help="Bind address (default 0.0.0.0)")
    parser.add_argument("--port",     default=8766, type=int, help="WebSocket port (default 8766)")
    parser.add_argument("--interval", default=0.08, type=float,
                        help="Poll interval in seconds (default 0.08 = 80 ms)")
    args = parser.parse_args()

    try:
        asyncio.run(_main(args.host, args.port, args.interval))
    except KeyboardInterrupt:
        log.info("Stopped.")