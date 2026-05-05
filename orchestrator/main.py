#!/usr/bin/env python3
"""
SAGE — tour-guide robot main entry point.

Wires together: STT → LLM (streaming + tool calls) → TTS, with
Nav2 navigation and a priority-event dispatcher for arrival
announcements.

All mutable state lives inside :class:`SageApp`; there are no module-
level globals.
"""

from __future__ import annotations

import os
import signal
import subprocess
import sys
import threading
import time
from typing import List, Dict, Any

import openai
import torch

import logger
from config import AppConfig
from events import EventDispatcher
from nav.navigation import NavManager
from speech.piper_tts import PiperTTS
from llm.streaming import run_conversation_turn
from llm.tools import ToolRegistry, build_tool_schemas
from ui_state_client import UIStatePublisher
from utils import read_openai_key, play_wav
from speech.web_ptt import PushToTalkServer
from waypoints import WAYPOINTS

_log = logger.get("general")

# Type alias
Messages = List[Dict[str, Any]]


class SageApp:
    """Top-level application object — owns every subsystem.

    Lifecycle::

        app = SageApp()
        app.run()           # blocks until Ctrl-C
    """

    def __init__(self, config: AppConfig | None = None) -> None:
        self.cfg = config or AppConfig.from_env()

        # Logging first — everything else uses it
        logger.setup(self.cfg.logging)

        self._shutdown = threading.Event()
        self._is_recording = threading.Event()
        self._messages: Messages = []
        self._messages_lock = threading.Lock()
        self._llm_lock = threading.Lock()

        # -- UI state publisher (ROS 2) --------------------------------
        self.ui = UIStatePublisher()

        # -- TTS -------------------------------------------------------
        self.tts = PiperTTS(
            model_path=self.cfg.tts.model_path,
            aplay_device=self.cfg.tts.aplay_device,
            buffer_time_us=self.cfg.tts.buffer_time_us,
            period_size=self.cfg.tts.period_size,
        )
        self.tts.on_start = self.ui.speaking
        self.tts.on_end = self.ui.idle
        _log.info("PiperTTS ready")

        # -- OpenAI client ---------------------------------------------
        api_key = read_openai_key(self.cfg.llm.api_key_path)
        self.llm_client = openai.OpenAI(api_key=api_key)

        # -- Navigation (lazy-start) -----------------------------------
        self.nav = NavManager(
            enqueue_arrival=self._enqueue_arrival_proxy,
            enqueue_failure=self._enqueue_failure_proxy,
            waypoints=WAYPOINTS,
        )

        # -- Tool registry ---------------------------------------------
        self.tool_schemas = build_tool_schemas()
        self.tool_registry = ToolRegistry(
            nav=self.nav,
            ui=self.ui,
            endpoints=self.cfg.endpoints,
            api_key_path=self.cfg.llm.api_key_path,
        )

        # -- Event dispatcher (needs tool_registry) --------------------
        self.events = EventDispatcher(
            tts=self.tts,
            tool_registry=self.tool_registry,
            is_recording=lambda: self._is_recording.is_set(),
            on_arrival=self._handle_arrival,
            on_failure=self._handle_failure,
            shutdown_flag=lambda: self._shutdown.is_set(),
        )

        # -- STT (built in run() after torch device detection) ---------
        self.recorder: Any = None

    # ==================================================================
    # Main loop
    # ==================================================================

    def run(self) -> None:
        """Start all background threads and enter the listen-respond loop."""
        signal.signal(signal.SIGINT, self._on_sigint)

        # Start background subsystems
        self.events.start()
        self.nav.ensure_started()

        device = "cuda" if torch.cuda.is_available() else "cpu"
        _log.info("Whisper device: %s", device)

        # Start web push-to-talk server
        self.ptt_server = PushToTalkServer(self, device=device)
        self.ptt_server.start()

        self._init_recorder(device)

        _log.info("Ready to guide tours with streaming TTS")

        while not self._shutdown.is_set():
            try:
                user_input = self.recorder.text()
            except KeyboardInterrupt:
                break
            except Exception:
                logger.log_exception("recorder.text()")
                continue

            if not user_input:
                if self.ui.last_phase == "listening" and not self.tts.is_playing():
                    self.ui.idle()
                continue

            _log.info("User: %s", user_input)
            self._process_user_input(user_input)
            
        self._teardown()

    # ==================================================================
    # Input processing
    # ==================================================================

    def _process_user_input(self, text: str) -> str:
        """Add user message, trim history, and run an LLM turn.

        Returns the assistant's reply text.
        """
        max_history = self.cfg.llm.max_history_len

        with self._messages_lock:
            self._messages.append({"role": "user", "content": text})
            if len(self._messages) > max_history:
                self._messages[:] = self._messages[-max_history:]
            _log.debug("Message history length: %d", len(self._messages))

        with self._llm_lock:
            reply = run_conversation_turn(
                client=self.llm_client,
                messages=self._messages,
                messages_lock=self._messages_lock,
                tool_schemas=self.tool_schemas,
                tool_registry=self.tool_registry,
                nav=self.nav,
                tts=self.tts,
                ui=self.ui,
                config=self.cfg.llm,
            )
            _log.info("AI: %s", reply)

        return reply

    # ==================================================================
    # Event / arrival handling
    # ==================================================================

    def _enqueue_arrival_proxy(self, target: str) -> None:
        """Called from the Nav2 result callback (any thread)."""
        self.events.enqueue_arrival(target)

    def _enqueue_failure_proxy(self, target: str, reason: str) -> None:
        """Called from a result callback (any thread)."""
        self.events.enqueue_failure(target, reason)

    def _handle_failure(self, target: str, reason: str) -> None:
        """Inject a failure event into history and trigger an LLM pass."""
        failure_msg = f"[EVENT PROMPT] Could not reach {target}. Reason: {reason}"
        with self._messages_lock:
            self._messages.append({
                "role": "system", "name": "event", "content": failure_msg,
            })

        with self._llm_lock:
            reply = run_conversation_turn(
                client=self.llm_client,
                messages=self._messages,
                messages_lock=self._messages_lock,
                tool_schemas=self.tool_schemas,
                tool_registry=self.tool_registry,
                nav=self.nav,
                tts=self.tts,
                ui=self.ui,
                config=self.cfg.llm,
            )
        _log.info("Failure announcement (%s): %d chars", target, len(reply))

    def _handle_arrival(self, target: str) -> None:
        """Inject an arrival event into history and trigger an LLM pass."""
        arrival_msg = f"[EVENT PROMPT] Arrived at {target}."
        with self._messages_lock:
            self._messages.append({
                "role": "system", "name": "event", "content": arrival_msg,
            })

        with self._llm_lock:
            reply = run_conversation_turn(
                client=self.llm_client,
                messages=self._messages,
                messages_lock=self._messages_lock,
                tool_schemas=self.tool_schemas,
                tool_registry=self.tool_registry,
                nav=self.nav,
                tts=self.tts,
                ui=self.ui,
                config=self.cfg.llm,
            )
        _log.info("Arrival announcement (%s): %d chars. Content: %s", target, len(reply), reply)

    # ==================================================================
    # STT initialisation
    # ==================================================================

    def _init_recorder(self, device: str) -> None:
        from RealtimeSTT import AudioToTextRecorder

        cfg = self.cfg.stt
        kwargs: dict[str, Any] = dict(
            language=cfg.language,
            spinner=True,
            model=cfg.whisper_model,
            device=device,
            on_recording_start=self._on_recording_start,
            on_recording_stop=self._on_recording_stop,
            no_log_file=True,
            start_callback_in_new_thread=True,
            silero_sensitivity=cfg.silero_sensitivity,
            silero_use_onnx=cfg.silero_use_onnx,
            silero_deactivity_detection=cfg.silero_deactivity_detection,
            max_recording_duration=cfg.max_recording_duration,
            # wake_word_activation_delay=cfg.wake_word_activation_delay,
        )

        if cfg.use_wakeword:
            kwargs.update(
                wakeword_backend=cfg.wakeword_backend,
                openwakeword_model_paths=cfg.wakeword_model_path,
                openwakeword_inference_framework="onnx",
                wake_words_sensitivity=cfg.wakeword_sensitivity,
                wake_word_buffer_duration=cfg.wakeword_buffer_dur,
                on_wakeword_detected=self._on_wakeword,
                on_wakeword_timeout=self._on_wakeword_timeout,
                wake_words=cfg.wake_word,
            )

        self.recorder = AudioToTextRecorder(**kwargs)
        self.ui.idle()
    # ==================================================================
    # STT / wake-word callbacks
    # ==================================================================

    def _on_recording_start(self) -> None:
        try:
            if self.tts.is_playing():
                _log.debug("TTS stop (barge-in)")
                self.tts.stop()
            self._is_recording.set()
            _log.debug("Recording START")
            self.ui.listening()
        except Exception:
            logger.log_exception("on_recording_start")

    def _on_recording_stop(self) -> None:
        self._is_recording.clear()
        _log.debug("Recording STOP")

    def _on_wakeword(self) -> None:
        if self.tts.is_playing():
            self.tts.stop()
        if self.ui.last_phase != "listening":
            if self.cfg.stt.say_chime:
                chime = self.cfg.stt.ready_chime_path
                threading.Thread(
                    target=play_wav, args=(chime,), daemon=True,
                ).start()
        self.ui.listening()

    def _on_wakeword_timeout(self) -> None:
        _log.debug("Wakeword timeout — no speech detected, resetting UI")
        if not self.tts.is_playing():
            self.ui.idle()
    # ==================================================================
    # Shutdown
    # ==================================================================

    def _on_sigint(self, _sig: int, _frame: Any) -> None:
        _log.info("SIGINT received, shutting down…")
        self._shutdown.set()

    def _teardown(self) -> None:
        """Best-effort cleanup of every subsystem."""
        _log.info("Tearing down…")

        for label, action in (
            ("recorder", lambda: self.recorder and self.recorder.terminate()),
            ("tts",      lambda: self.tts.close()),
            ("nav",      lambda: self.nav.shutdown()),
            ("ui",       lambda: self.ui.close()),
        ):
            try:
                action()
            except Exception:
                _log.debug("Teardown error in %s", label, exc_info=True)

        _log.info("Shutdown complete.")


# ======================================================================
# Entry point
# ======================================================================

def main() -> None:
    app = SageApp()
    app.run()


if __name__ == "__main__":
    main()