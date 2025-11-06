# piper_tts.py
import os
import tempfile
import uuid
import wave
import subprocess
import threading
from typing import Optional, Callable
from piper import PiperVoice


class PiperTTS:
    """
    Minimal, fast Piper wrapper.
    - Loads voice once.
    - Synthesizes full text to a temp WAV (RAM if /dev/shm available).
    - Plays via ALSA (aplay).
    - Supports stop/barge-in and status checks.

    Usage:
        tts = PiperTTS("model.onnx")
        tts.on_start = lambda: print("speaking...")
        tts.on_end   = lambda: print("idle.")
        tts.say("Hello!", block=False)
    """

    def __init__(
        self,
        model_path: str,
        *,
        aplay_device: Optional[str] = None,        # e.g., "hw:1,0" or None for default
        buffer_time_us: Optional[int] = None,      # e.g., 40000 … 120000
        period_size: Optional[int] = None,         # e.g., 256, 512, 1024
        warmup: bool = True,
        on_start: Optional[Callable[[], None]] = None,
        on_end: Optional[Callable[[], None]] = None,
    ):
        self.voice = PiperVoice.load(model_path)
        self.aplay_device = aplay_device
        self.buffer_time_us = buffer_time_us
        self.period_size = period_size

        self.on_start: Optional[Callable[[], None]] = on_start
        self.on_end: Optional[Callable[[], None]] = on_end

        self._proc: Optional[subprocess.Popen] = None
        self._lock = threading.Lock()
        self._play_thread: Optional[threading.Thread] = None
        self._last_wav_path: Optional[str] = None

        if warmup:
            # Tiny warmup to avoid first-utterance lag
            p = self._tmpwav_path()
            try:
                with wave.open(p, "wb") as w:
                    # Synthesize a trivial utterance; content doesn't matter
                    self.voice.synthesize_wav(".", w)
            finally:
                self._safe_rm(p)

    # ----------------- public API -----------------
    def say(self, text: str, *, block: bool = False, interrupt: bool = True):
        """
        Synthesize and play `text`.
        - block=False: returns immediately (play in background).
        - interrupt=True: stop current playback before starting a new one.
        """
        if not text or not text.strip():
            return

        if interrupt:
            self.stop()

        wav_path = self._tmpwav_path()
        with wave.open(wav_path, "wb") as w:
            self.voice.synthesize_wav(text, w)

        if block:
            self._play_blocking(wav_path)
        else:
            self._play_background(wav_path)

    def is_playing(self) -> bool:
        with self._lock:
            if self._proc is None:
                return False
            return self._proc.poll() is None

    def stop(self):
        """Stop current playback immediately and cleanup temp file."""
        with self._lock:
            if self._proc and self._proc.poll() is None:
                try:
                    self._proc.terminate()
                    self._proc.wait(timeout=0.2)
                except Exception:
                    try:
                        self._proc.kill()
                    except Exception:
                        pass
            self._proc = None

            if self._last_wav_path:
                self._safe_rm(self._last_wav_path)
                self._last_wav_path = None

    def close(self):
        """Alias for stop()."""
        self.stop()

    # ----------------- internals -----------------
    def _aplay_cmd(self, wav_path: str):
        cmd = ["aplay", "-q"]
        if self.aplay_device:
            cmd += ["-D", self.aplay_device]
        if self.buffer_time_us is not None:
            cmd += ["--buffer-time", str(self.buffer_time_us)]
        if self.period_size is not None:
            cmd += ["--period-size", str(self.period_size)]
        cmd += [wav_path]
        return cmd

    def _play_blocking(self, wav_path: str):
        # Launch aplay process
        with self._lock:
            self._last_wav_path = wav_path
            self._proc = subprocess.Popen(self._aplay_cmd(wav_path))

        # Fire on_start outside the lock to avoid user callback deadlocks
        try:
            if callable(self.on_start):
                try:
                    self.on_start()
                except Exception:
                    pass

            # Block until playback finishes (or is stopped)
            self._proc.wait()
        finally:
            # Cleanup & on_end
            with self._lock:
                self._proc = None
                self._safe_rm(wav_path)
                self._last_wav_path = None

            if callable(self.on_end):
                try:
                    self.on_end()
                except Exception:
                    pass

    def _play_background(self, wav_path: str):
        # Only one play thread at a time; with interrupt=True this should be rare
        if self._play_thread and self._play_thread.is_alive():
            self._play_thread.join(timeout=0.05)

        self._play_thread = threading.Thread(
            target=self._play_blocking, args=(wav_path,), daemon=True
        )
        self._play_thread.start()

    @staticmethod
    def _tmpwav_path() -> str:
        base = "/dev/shm" if os.path.isdir("/dev/shm") else tempfile.gettempdir()
        return os.path.join(base, f"piper_{uuid.uuid4().hex}.wav")

    @staticmethod
    def _safe_rm(path: str):
        try:
            os.remove(path)
        except Exception:
            pass
