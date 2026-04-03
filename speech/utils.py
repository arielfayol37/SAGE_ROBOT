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


def read_api_key(
    json_path: str,
    field: str,
    env_var: Optional[str] = None,
) -> Optional[str]:
    """Read an API key from a JSON file, falling back to an env var.

    Parameters
    ----------
    json_path:
        Path to a JSON file mapping service names to keys,
        e.g. ``{"openai_api_key": "sk-…", "tavily_api_key": "tvly-…"}``.
    field:
        The key name to look up inside the JSON object.
    env_var:
        Optional environment variable to check as a fallback.

    Returns ``None`` if neither source provides a value.
    """
    if os.path.isfile(json_path):
        try:
            with open(json_path) as fh:
                data = json.load(fh)
            value = data.get(field)
            if value:
                return str(value)
        except (json.JSONDecodeError, OSError) as exc:
            _log.warning("Could not read %s from %s: %s", field, json_path, exc)

    if env_var:
        env_value = os.getenv(env_var)
        if env_value:
            return env_value

    return None


def read_openai_key(json_path: str) -> str:
    """Read the OpenAI API key from a JSON file or ``OPENAI_API_KEY`` env var.

    The JSON file is expected to contain ``{"openai_api_key": "sk-…"}``.

    Raises
    ------
    RuntimeError
        If no key can be found from either source.
    """
    key = read_api_key(json_path, "openai_api_key", "OPENAI_API_KEY")
    if key:
        return key

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