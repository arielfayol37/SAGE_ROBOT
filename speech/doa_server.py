"""
Direction-of-Arrival WebSocket broadcaster for SAGE.

Reads the auto-selected beam azimuth from the XVF3800 USB mic array
and broadcasts it as JSON over a WebSocket on port 8766.

The face UI connects to ``ws://<host>:8766/ws/doa`` and moves the
pupils to follow the speaker.

Message format::

    {"type": "doa", "angle_deg": 42.7}

    0°   = directly behind the robot
    90°  = robot's left side
    180° = directly in front
    270° = robot's right side

Usage::

    python doa_server.py
    python doa_server.py --port 8766 --interval 0.08
"""

from __future__ import annotations

import argparse
import asyncio
import json
import logging
import math
import struct
from typing import Optional, Set

try:
    import usb.core
    import usb.util
    _USB_AVAILABLE = True
except ImportError:
    _USB_AVAILABLE = False

try:
    import websockets
    from websockets.server import WebSocketServerProtocol
except ImportError:
    raise SystemExit("websockets package required:  pip install websockets")

# -- XVF3800 USB constants --------------------------------------------
_VID = 0x2886
_PID = 0x001A
_AEC_AZIMUTH_RESID = 33
_AEC_AZIMUTH_CMDID = 0x80 | 75
_NUM_FLOATS = 4
_STATUS_PLUS_DATA_LEN = 1 + 4 * _NUM_FLOATS
_USB_TIMEOUT_MS = 100_000

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [doa_server] %(levelname)s %(message)s",
)
_log = logging.getLogger("doa_server")

# -- Shared state ------------------------------------------------------
_clients: Set[WebSocketServerProtocol] = set()
_last_angle: float = 0.0


def _read_aec_azimuth(dev: "usb.core.Device") -> tuple[float, ...]:
    """Read four azimuth floats from the XVF3800 (radians)."""
    resp = dev.ctrl_transfer(
        usb.util.CTRL_IN | usb.util.CTRL_TYPE_VENDOR | usb.util.CTRL_RECIPIENT_DEVICE,
        0,
        _AEC_AZIMUTH_CMDID,
        _AEC_AZIMUTH_RESID,
        _STATUS_PLUS_DATA_LEN,
        _USB_TIMEOUT_MS,
    )
    data = resp.tobytes()[1 : 1 + 4 * _NUM_FLOATS]
    return struct.unpack("<ffff", data)


async def _doa_reader_loop(interval: float) -> None:
    """Poll the mic array and broadcast angle updates."""
    global _last_angle

    dev: Optional["usb.core.Device"] = None
    if _USB_AVAILABLE:
        dev = usb.core.find(idVendor=_VID, idProduct=_PID)
        if dev is None:
            _log.warning("XVF3800 not found — running in simulation mode.")
        else:
            _log.info("XVF3800 connected.")
            try:
                if dev.is_kernel_driver_active(3):
                    dev.detach_kernel_driver(3)
            except Exception:
                pass
    else:
        _log.warning("pyusb not available — running in simulation mode.")

    sim_angle = 180.0

    while True:
        try:
            if dev is not None:
                _, _, _, auto_sel = _read_aec_azimuth(dev)
                _last_angle = (math.degrees(auto_sel) + 360.0) % 360.0
            else:
                _last_angle = sim_angle

            msg = json.dumps({"type": "doa", "angle_deg": round(_last_angle, 2)})
            dead: Set[WebSocketServerProtocol] = set()
            for ws in list(_clients):
                try:
                    await ws.send(msg)
                except Exception:
                    dead.add(ws)
            _clients -= dead

        except Exception as exc:
            _log.error("DOA read error: %s", exc)

        await asyncio.sleep(interval)


async def _ws_handler(websocket: WebSocketServerProtocol) -> None:
    """Handle a new face-UI WebSocket connection."""
    _log.info("Client connected from %s", websocket.remote_address)
    _clients.add(websocket)
    try:
        await websocket.send(
            json.dumps({"type": "doa", "angle_deg": round(_last_angle, 2)})
        )
        await websocket.wait_closed()
    finally:
        _clients.discard(websocket)
        _log.info("Client disconnected from %s", websocket.remote_address)


async def _main(host: str, port: int, interval: float) -> None:
    _log.info("Starting DOA WebSocket server on ws://%s:%d", host, port)
    _log.info("Poll interval: %.0f ms", interval * 1000)
    async with websockets.serve(_ws_handler, host, port):
        await _doa_reader_loop(interval)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="SAGE DOA WebSocket server")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", default=8766, type=int)
    parser.add_argument("--interval", default=0.08, type=float,
                        help="Poll interval in seconds (default 0.08)")
    args = parser.parse_args()

    try:
        asyncio.run(_main(args.host, args.port, args.interval))
    except KeyboardInterrupt:
        _log.info("Stopped.")
