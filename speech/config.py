"""
Centralised configuration for the SAGE tour-guide robot.

Every magic number, path, and feature flag lives here so the rest of the
codebase stays free of hardcoded values.  Override at startup via
environment variables where noted.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import Optional


# ---------------------------------------------------------------------------
# Logging verbosity
# ---------------------------------------------------------------------------

@dataclass(frozen=True, slots=True)
class LoggingConfig:
    """Per-subsystem verbosity flags."""
    general:  bool = True
    tokens:   bool = False
    tools:    bool = True
    robot:    bool = False
    tts:      bool = True
    stt:      bool = True


# ---------------------------------------------------------------------------
# TTS / STT / Wake-word
# ---------------------------------------------------------------------------

@dataclass(frozen=True, slots=True)
class TTSConfig:
    model_path:     str = "assets/models/piper/en_US-amy-medium.onnx"
    aplay_device:   str = "pulse"
    buffer_time_us: int = 40_000
    period_size:    int = 256


@dataclass(frozen=True, slots=True)
class STTConfig:
    language:             str  = "en"
    whisper_model:        str  = "tiny.en"
    use_wakeword:         bool = True
    wakeword_backend:     str  = "openwakeword"
    wakeword_model_path:  str  = "assets/models/wakeword/sage_wakeword_2.onnx,assets/models/wakeword/alexa_v0.1.onnx,assets/models/wakeword/hey_jarvis_v0.1.onnx"
    wakeword_sensitivity: float = 0.3
    wakeword_buffer_dur:  float = 0.2
    wake_word:            str  = "sage"
    ready_chime_path:     str  = "assets/audio/ui-wakesound.wav"


# ---------------------------------------------------------------------------
# LLM
# ---------------------------------------------------------------------------

@dataclass(frozen=True, slots=True)
class LLMConfig:
    model:           str = "gpt-4.1-nano"
    api_key_path:    str = "api_keys/api_keys.json"
    max_tool_depth:  int = 3
    max_history_len: int = 12


# ---------------------------------------------------------------------------
# Networking / backend URLs
# ---------------------------------------------------------------------------

@dataclass(frozen=True, slots=True)
class EndpointsConfig:
    robot_backend: str = "http://127.0.0.1:8002"
    kb_search:     str = "http://127.0.0.1:8004/api/kb/search"
    kb_connect_timeout: float = 2.0
    kb_read_timeout:    float = 8.0


# ---------------------------------------------------------------------------
# Web push-to-talk
# ---------------------------------------------------------------------------

@dataclass(frozen=True, slots=True)
class WebConfig:
    enabled:  bool = True
    host:     str  = "0.0.0.0"
    port:     int  = 8005


# ---------------------------------------------------------------------------
# Top-level aggregate
# ---------------------------------------------------------------------------

@dataclass(frozen=True, slots=True)
class AppConfig:
    logging:   LoggingConfig   = field(default_factory=LoggingConfig)
    tts:       TTSConfig       = field(default_factory=TTSConfig)
    stt:       STTConfig       = field(default_factory=STTConfig)
    llm:       LLMConfig       = field(default_factory=LLMConfig)
    endpoints: EndpointsConfig = field(default_factory=EndpointsConfig)
    web:       WebConfig       = field(default_factory=WebConfig)

    @classmethod
    def from_env(cls) -> "AppConfig":
        """Build config with environment-variable overrides where set."""
        # Instantiate defaults first — accessing fields on a slotted
        # frozen dataclass *class* returns descriptors, not values.
        _llm = LLMConfig()
        _tts = TTSConfig()
        _ep  = EndpointsConfig()

        return cls(
            llm=LLMConfig(
                model=os.getenv("SAGE_LLM_MODEL", _llm.model),
                api_key_path=os.getenv("SAGE_API_KEY_PATH", _llm.api_key_path),
            ),
            tts=TTSConfig(
                model_path=os.getenv("SAGE_TTS_MODEL", _tts.model_path),
                aplay_device=os.getenv("SAGE_APLAY_DEVICE", _tts.aplay_device),
            ),
            endpoints=EndpointsConfig(
                kb_search=os.getenv("SAGE_KB_URL", _ep.kb_search),
            ),
        )
