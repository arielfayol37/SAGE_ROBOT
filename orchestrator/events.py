"""
Priority-queue event dispatcher for SAGE.

Handles ``arrival`` and ``failure`` events. Both are held until TTS
playback and STT recording have both finished, then an LLM pass is
triggered so the robot can announce them.
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

# Priority constants (lower = higher priority).
# Failures slightly higher priority than arrivals so they cut ahead
# if both somehow queue up at once.
_FAILURE_PRIORITY = 5
_ARRIVAL_PRIORITY = 10


class EventDispatcher:
    """Background consumer that drains a :class:`queue.PriorityQueue`.

    Events are held until TTS playback and STT recording have both
    finished, then an LLM pass is triggered so the robot can speak.
    """

    def __init__(
        self,
        *,
        tts: PiperTTS,
        tool_registry: ToolRegistry,
        is_recording: Callable[[], bool],
        on_arrival: Callable[[str], None],
        on_failure: Callable[[str, str], None],
        shutdown_flag: Callable[[], bool],
    ) -> None:
        self._tts = tts
        self._tools = tool_registry
        self._is_recording = is_recording
        self._on_arrival = on_arrival
        self._on_failure = on_failure
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

    def enqueue_failure(self, target: str, reason: str) -> None:
        """Called from a result callback when navigation/docking fails."""
        epoch = self._tools.nav_epoch
        _log.info("Enqueue failure event: %s (epoch %d) — %s",
                  target, epoch, reason)
        self._queue.put((
            _FAILURE_PRIORITY,
            time.time(),
            {
                "kind": "failure",
                "target": target,
                "reason": reason,
                "epoch": epoch,
            },
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

            kind = event.get("kind")
            if kind == "arrival":
                self._handle_arrival(event)
            elif kind == "failure":
                self._handle_failure(event)

    def _handle_arrival(self, event: Dict[str, Any]) -> None:
        target = event.get("target", "unknown")
        epoch = event.get("epoch")
        _log.info("Processing arrival event: %s (epoch %d)", target, epoch)

        if not self._wait_for_speech_done():
            return

        # Drop stale events (user sent a new goal in the meantime)
        if epoch != self._tools.nav_epoch or target != self._tools.current_target:
            _log.info("Drop stale arrival event for %s (epoch mismatch)", target)
            return

        self._on_arrival(target)

    def _handle_failure(self, event: Dict[str, Any]) -> None:
        target = event.get("target", "unknown")
        reason = event.get("reason", "Unknown reason.")
        epoch = event.get("epoch")
        _log.info("Processing failure event: %s (epoch %d) — %s",
                  target, epoch, reason)

        if not self._wait_for_speech_done():
            return

        # Drop stale failures the same way we drop stale arrivals.
        # If the user already moved on to a different goal, the old
        # failure isn't relevant anymore.
        if epoch != self._tools.nav_epoch:
            _log.info("Drop stale failure event for %s (epoch mismatch)", target)
            return

        self._on_failure(target, reason)

    def _wait_for_speech_done(self) -> bool:
        """Block until TTS and recording finish. Returns False on shutdown."""
        while (self._tts.is_playing() or self._is_recording()) and not self._shutdown():
            time.sleep(0.05)
        return not self._shutdown()