"""
Minimal, fast Piper TTS wrapper with Pulse/PipeWire routing support.

- Loads the voice model once at init.
- Synthesises full text to a temp WAV (``/dev/shm`` when available).
- Plays via ``aplay`` (ALSA or Pulse depending on config).
- Supports stop / barge-in and status callbacks.

Usage::

    tts = PiperTTS("model.onnx", aplay_device="pulse")
    tts.on_start = lambda: print("speaking…")
    tts.on_end   = lambda: print("idle.")
    tts.say("Hello!", block=False)
"""

from __future__ import annotations

import logging
import os
import subprocess
import tempfile
import threading
import uuid
import wave
from typing import Callable, Optional

from piper import PiperVoice

_log = logging.getLogger("sage.tts")


class PiperTTS:
    """Synchronous-API TTS engine backed by Piper + aplay."""

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

        # FIX: force ALSA stable path unless explicitly overridden
        self.aplay_device = aplay_device

        self.pulse_sink_name = pulse_sink_name
        self.buffer_time_us = buffer_time_us
        self.period_size = period_size

        self.on_start = on_start
        self.on_end = on_end

        self._proc: Optional[subprocess.Popen[bytes]] = None
        self._lock = threading.Lock()
        self._play_thread: Optional[threading.Thread] = None

        if warmup:
            self._warmup()

    # -- context manager -----------------------------------------------

    def __enter__(self) -> "PiperTTS":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    # -- public API ----------------------------------------------------

    def say(self, text: str, *, block: bool = False, interrupt: bool = True) -> None:
        """Synthesise *text* and play it.

        Parameters
        ----------
        block:
            If ``True``, block until playback finishes.
        interrupt:
            If ``True`` (default), stop any in-progress playback first.
        """
        if not text or not text.strip():
            return
        if interrupt:
            self.stop()

        wav_path = self._tmpwav_path()
        with wave.open(wav_path, "wb") as wav_file:
            wav_file.setnchannels(1)
            wav_file.setsampwidth(2)
            wav_file.setframerate(16000)
            self.voice.synthesize_wav(text, wav_file)

        if block:
            self._play_blocking(wav_path)
        else:
            self._play_background(wav_path)

    def is_playing(self) -> bool:
        """Return ``True`` if audio is currently being played."""
        with self._lock:
            return self._proc is not None and self._proc.poll() is None

    def stop(self) -> None:
        """Stop current playback immediately."""
        with self._lock:
            self._terminate_locked()

    def close(self) -> None:
        """Stop playback and release resources."""
        self.stop()

    # -- internal ------------------------------------------------------

    def _warmup(self) -> None:
        """Synthesise a single dot to load the model into memory."""
        path = self._tmpwav_path()
        try:
            with wave.open(path, "wb") as wav_file:
                self.voice.synthesize_wav(".", wav_file)
        finally:
            self._safe_rm(path)

    def _terminate_locked(self) -> None:
        """Stop the subprocess and clean up its temp file.  Caller holds lock."""
        if self._proc is not None and self._proc.poll() is None:
            try:
                self._proc.terminate()
                self._proc.wait(timeout=0.2)
            except Exception:
                try:
                    self._proc.kill()
                except Exception:
                    pass
        self._proc = None

    def _play_cmd_and_env(self, wav_path: str) -> tuple[list[str], dict[str, str]]:
        env = os.environ.copy()

        cmd = ["aplay", "-q"]
        if self.aplay_device:
            cmd += ["-D", self.aplay_device]

        if self.buffer_time_us is not None:
            cmd += ["--buffer-time", str(self.buffer_time_us)]
        if self.period_size is not None:
            cmd += ["--period-size", str(self.period_size)]

        cmd.append(wav_path)
        return cmd, env

    def _play_blocking(self, wav_path: str) -> None:
        cmd, env = self._play_cmd_and_env(wav_path)

        with self._lock:
            self._proc = subprocess.Popen(cmd, env=env)

        try:
            self._fire_callback(self.on_start)
            self._proc.wait()
        finally:
            with self._lock:
                self._proc = None
                self._safe_rm(wav_path)
            self._fire_callback(self.on_end)

    def _play_background(self, wav_path: str) -> None:
        if self._play_thread is not None and self._play_thread.is_alive():
            self._play_thread.join(timeout=0.05)
        self._play_thread = threading.Thread(
            target=self._play_blocking, args=(wav_path,), daemon=True,
        )
        self._play_thread.start()

    @staticmethod
    def _fire_callback(cb: Optional[Callable[[], None]]) -> None:
        if callable(cb):
            try:
                cb()
            except Exception:
                _log.debug("TTS callback error", exc_info=True)

    @staticmethod
    def _tmpwav_path() -> str:
        base = "/dev/shm" if os.path.isdir("/dev/shm") else tempfile.gettempdir()
        return os.path.join(base, f"piper_{uuid.uuid4().hex}.wav")

    @staticmethod
    def _safe_rm(path: str) -> None:
        try:
            os.remove(path)
        except OSError:
            pass
