"""
Minimal, fast Piper TTS wrapper with Pulse/PipeWire routing support.

Streaming model
---------------
A single utterance keeps one ``aplay`` process alive for its entire
duration. Callers push text fragments in with :meth:`feed` and Piper
synthesises them sentence-by-sentence while ``aplay`` plays whatever
has already been written. This gives:

- First-sentence latency ≈ Piper synth time for one sentence, regardless
  of the total length of the utterance.
- No audible gaps between fragments (same ``aplay`` process keeps
  draining its buffer).
- Eager streaming: a caller can ``feed`` sentences as they arrive from
  a token stream, and audio starts playing on the first one.

Public API
----------
- :meth:`begin_utterance` — start a new utterance (stops any in-flight one).
- :meth:`feed` — push more text into the current utterance.
- :meth:`end_utterance` — signal end-of-text; ``aplay`` drains naturally.
- :meth:`say` — convenience: begin → feed → end.
- :meth:`stop` — hard barge-in.

Usage::

    tts = PiperTTS("model.onnx", aplay_device="pulse")
    tts.on_start = lambda: print("speaking…")
    tts.on_end   = lambda: print("idle.")

    tts.begin_utterance()
    for sentence in stream_of_sentences:
        tts.feed(sentence)
    tts.end_utterance()
"""

from __future__ import annotations

import logging
import os
import queue
import subprocess
import threading
from typing import Callable, Optional

from piper import PiperVoice

_log = logging.getLogger("sage.tts")
_SENTINEL = object()


class PiperTTS:
    """Streaming TTS engine backed by Piper + aplay."""

    def __init__(
        self,
        model_path: str,
        *,
        aplay_device: Optional[str] = None,
        pulse_sink_name: Optional[str] = None,
        buffer_time_us: Optional[int] = None,
        period_size: Optional[int] = None,
        warmup: bool = True,
        on_start: Optional[Callable[[], None]] = None,
        on_end: Optional[Callable[[], None]] = None,
    ) -> None:
        self.voice = PiperVoice.load(model_path)

        self.aplay_device = aplay_device
        self.pulse_sink_name = pulse_sink_name  # kept for API compatibility
        self.buffer_time_us = buffer_time_us
        self.period_size = period_size

        self.on_start = on_start
        self.on_end = on_end

        self._proc: Optional[subprocess.Popen[bytes]] = None
        self._lock = threading.Lock()
        self._play_thread: Optional[threading.Thread] = None
        self._queue: Optional[queue.Queue] = None
        self._stop_flag = threading.Event()
        self._on_start_fired = False

        if warmup:
            self._warmup()

    # -- context manager -----------------------------------------------

    def __enter__(self) -> "PiperTTS":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    # -- public API ----------------------------------------------------

    def begin_utterance(self) -> None:
        """Start a new utterance.

        Stops any in-progress utterance first. Audio playback begins on
        the first :meth:`feed`; ``aplay`` is opened lazily so an
        utterance with no fed text never opens the audio device.
        """
        self.stop()
        self._stop_flag.clear()
        self._on_start_fired = False
        self._queue = queue.Queue()
        self._play_thread = threading.Thread(
            target=self._utterance_worker, daemon=True,
        )
        self._play_thread.start()

    def feed(self, text: str) -> None:
        """Append a text fragment to the current utterance.

        If no utterance is active, one is started automatically. Empty
        or whitespace-only fragments are ignored.
        """
        if not text or not text.strip():
            return
        if (
            self._queue is None
            or self._play_thread is None
            or not self._play_thread.is_alive()
        ):
            self.begin_utterance()
        assert self._queue is not None
        self._queue.put(text)

    def end_utterance(self, *, block: bool = False) -> None:
        """Signal end-of-utterance; let ``aplay`` drain naturally.

        Parameters
        ----------
        block:
            If ``True``, wait until playback fully completes.
        """
        if self._queue is None:
            return
        self._queue.put(_SENTINEL)
        if block and self._play_thread is not None:
            self._play_thread.join()

    def say(self, text: str, *, block: bool = False, interrupt: bool = True) -> None:
        """One-shot convenience: begin → feed → end."""
        if not text or not text.strip():
            return
        if interrupt:
            self.stop()
        self.begin_utterance()
        self.feed(text)
        self.end_utterance(block=block)

    def is_playing(self) -> bool:
        """Return ``True`` if audio is currently being played."""
        with self._lock:
            return self._proc is not None and self._proc.poll() is None

    def stop(self) -> None:
        """Hard-stop the current utterance (barge-in)."""
        self._stop_flag.set()

        with self._lock:
            self._terminate_locked()

        # Drain the queue and signal the worker to exit.
        q = self._queue
        if q is not None:
            try:
                while True:
                    q.get_nowait()
            except queue.Empty:
                pass
            q.put(_SENTINEL)

        thread = self._play_thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=0.5)

        self._queue = None
        self._play_thread = None

    def close(self) -> None:
        """Stop playback and release resources."""
        self.stop()

    # -- internal ------------------------------------------------------

    def _warmup(self) -> None:
        """Pull a tiny utterance through the model to populate caches."""
        try:
            for _ in self.voice.synthesize("."):
                pass
        except Exception:
            _log.debug("TTS warmup failed", exc_info=True)

    def _terminate_locked(self) -> None:
        """Hard-kill aplay. Caller must hold ``self._lock``."""
        proc = self._proc
        if proc is not None and proc.poll() is None:
            try:
                proc.terminate()
                proc.wait(timeout=0.2)
            except Exception:
                try:
                    proc.kill()
                except Exception:
                    pass
        self._proc = None

    def _build_aplay_cmd(self, sample_rate: int, channels: int) -> list[str]:
        cmd = [
            "aplay", "-q",
            "-t", "raw",
            "-f", "S16_LE",
            "-c", str(channels),
            "-r", str(sample_rate),
        ]
        if self.aplay_device:
            cmd += ["-D", self.aplay_device]
        if self.buffer_time_us is not None:
            cmd += ["--buffer-time", str(self.buffer_time_us)]
        if self.period_size is not None:
            cmd += ["--period-size", str(self.period_size)]
        return cmd

    def _utterance_worker(self) -> None:
        """Consume the text queue and stream audio to aplay until sentinel."""
        q = self._queue
        if q is None:  # defensive
            return

        try:
            while True:
                item = q.get()
                if item is _SENTINEL:
                    break
                if self._stop_flag.is_set():
                    continue
                self._synth_and_stream(item)
        finally:
            # End-of-utterance: close aplay's stdin so it drains, then wait.
            with self._lock:
                proc = self._proc
            if proc is not None and not self._stop_flag.is_set():
                try:
                    if proc.stdin is not None:
                        proc.stdin.close()
                except Exception:
                    pass
                try:
                    proc.wait()
                except Exception:
                    pass
            with self._lock:
                if self._proc is proc:
                    self._proc = None

            # Fire on_end iff we ever fired on_start during this utterance,
            # so callbacks always come in matched pairs.
            if self._on_start_fired:
                self._on_start_fired = False
                self._fire_callback(self.on_end)

    def _synth_and_stream(self, text: str) -> None:
        """Synthesise *text* and write it into the current aplay process."""
        try:
            chunks = self.voice.synthesize(text)
        except Exception:
            _log.debug("Synthesis init failed", exc_info=True)
            return

        try:
            first = next(chunks)
        except StopIteration:
            return
        except Exception:
            _log.debug("Synthesis failed before first chunk", exc_info=True)
            return

        if self._stop_flag.is_set():
            return

        # Open aplay lazily on the first chunk of the utterance.
        fire_start = False
        with self._lock:
            if self._proc is None or self._proc.poll() is not None:
                sample_rate = getattr(first, "sample_rate", 22050)
                channels = getattr(first, "sample_channels", 1)
                cmd = self._build_aplay_cmd(sample_rate, channels)
                env = os.environ.copy()
                self._proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, env=env)
                fire_start = True
            proc = self._proc

        if fire_start:
            self._on_start_fired = True
            self._fire_callback(self.on_start)

        if proc.stdin is None:
            return

        try:
            try:
                proc.stdin.write(first.audio_int16_bytes)
            except BrokenPipeError:
                return
            for chunk in chunks:
                if self._stop_flag.is_set() or proc.poll() is not None:
                    break
                try:
                    proc.stdin.write(chunk.audio_int16_bytes)
                except BrokenPipeError:
                    break
        except Exception:
            _log.debug("Streaming write failed", exc_info=True)

    @staticmethod
    def _fire_callback(cb: Optional[Callable[[], None]]) -> None:
        if callable(cb):
            try:
                cb()
            except Exception:
                _log.debug("TTS callback error", exc_info=True)