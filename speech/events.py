"""
Priority-queue event dispatcher for SAGE.

Currently handles a single event kind — ``arrival`` — but the design
supports arbitrary event types via the ``kind`` key.
"""

from __future__ import annotations

import queue
import threading
import time
from typing import Any, Callable, Dict, Optional, TYPE_CHECKING

import logger

if TYPE_CHECKING:
    from piper_tts import PiperTTS
    from tools import ToolRegistry

_log = logger.get("general")

# Priority constants (lower = higher priority)
_ARRIVAL_PRIORITY = 10


class EventDispatcher:
    """Background consumer that drains a :class:`queue.PriorityQueue`.

    Arrival events are held until TTS playback and STT recording have
    both finished, then an LLM pass is triggered so the robot can
    announce the arrival.
    """

    def __init__(
        self,
        *,
        tts: PiperTTS,
        tool_registry: ToolRegistry,
        is_recording: Callable[[], bool],
        on_arrival: Callable[[str], None],
        shutdown_flag: Callable[[], bool],
    ) -> None:
        self._tts = tts
        self._tools = tool_registry
        self._is_recording = is_recording
        self._on_arrival = on_arrival
        self._shutdown = shutdown_flag
        self._queue: queue.PriorityQueue[tuple[int, float, Dict[str, Any]]] = (
            queue.PriorityQueue()
        )
        self._thread: Optional[threading.Thread] = None

    # -- public API ----------------------------------------------------

    def enqueue_arrival(self, target: str) -> None:
        """Called from the Nav2 result callback (any thread)."""
        epoch = self._tools.nav_epoch
        _log.info("Enqueue arrival event: %s (epoch %d)", target, epoch)
        self._queue.put((
            _ARRIVAL_PRIORITY,
            time.time(),
            {"kind": "arrival", "target": target, "epoch": epoch},
        ))

    def start(self) -> None:
        """Spawn the background dispatcher thread."""
        if self._thread is not None and self._thread.is_alive():
            return
        self._thread = threading.Thread(
            target=self._run, daemon=True, name="event-dispatcher",
        )
        self._thread.start()

    # -- internals -----------------------------------------------------

    def _run(self) -> None:
        while not self._shutdown():
            try:
                _, _, event = self._queue.get(timeout=0.25)
            except queue.Empty:
                continue

            if event.get("kind") == "arrival":
                self._handle_arrival(event)

    def _handle_arrival(self, event: Dict[str, Any]) -> None:
        target = event.get("target", "unknown")
        epoch = event.get("epoch")
        _log.info("Processing arrival event: %s (epoch %d)", target, epoch)

        # Wait for TTS / recording to finish before speaking
        while (self._tts.is_playing() or self._is_recording()) and not self._shutdown():
            time.sleep(0.05)

        if self._shutdown():
            return

        # Drop stale events (user sent a new goal in the meantime)
        if epoch != self._tools.nav_epoch or target != self._tools.current_target:
            _log.info("Drop stale arrival event for %s (epoch mismatch)", target)
            return

        self._on_arrival(target)
