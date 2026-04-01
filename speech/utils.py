"""
Shared utility functions for SAGE.
"""

from __future__ import annotations

import json
import logging
import os
import subprocess
from typing import Optional

_log = logging.getLogger("sage.general")


def read_openai_key(json_path: str) -> str:
    """Read the OpenAI API key from a JSON file or ``OPENAI_API_KEY`` env var.

    The JSON file is expected to contain ``{"openai_key": "sk-…"}``.
    Falls back to the ``OPENAI_API_KEY`` environment variable if the
    file is missing or unreadable.

    Raises
    ------
    RuntimeError
        If no key can be found from either source.
    """
    # 1. Try the JSON file
    if os.path.isfile(json_path):
        try:
            with open(json_path) as fh:
                data = json.load(fh)
            key = data.get("openai_key")
            if key:
                return str(key)
            _log.warning("%s exists but has no 'openai_key' field", json_path)
        except (json.JSONDecodeError, OSError) as exc:
            _log.warning("Could not read %s: %s", json_path, exc)

    # 2. Fallback to environment
    env_key = os.getenv("OPENAI_API_KEY")
    if env_key:
        return env_key

    raise RuntimeError(
        f"No OpenAI API key found.  Provide it in {json_path} "
        "or set the OPENAI_API_KEY environment variable."
    )


def play_wav(path: str, device: Optional[str] = "pulse") -> None:
    """Fire-and-forget WAV playback via ``aplay``.

    Non-blocking so callers (e.g. wake-word handler) are not delayed.
    """
    if not os.path.isfile(path):
        _log.debug("WAV file not found, skipping: %s", path)
        return
    cmd = ["aplay", "-q"]
    if device:
        cmd += ["-D", device]
    cmd.append(path)
    subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
