"""
Lightweight structured logging for SAGE.

Provides a thin wrapper around the stdlib ``logging`` module so every
subsystem gets a child logger whose level is controlled by
:class:`config.LoggingConfig`.
"""

from __future__ import annotations

import logging
import sys
import traceback
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from config import LoggingConfig

_FORMAT = "[%(asctime)s] %(levelname)s: %(name)s — %(message)s"
_DATE_FMT = "%H:%M:%S"

_configured = False


def setup(cfg: LoggingConfig) -> None:
    """Call once at startup to wire up handlers and per-subsystem levels."""
    global _configured
    if _configured:
        return

    root = logging.getLogger("sage")
    root.setLevel(logging.DEBUG)

    handler = logging.StreamHandler(sys.stdout)
    handler.setFormatter(logging.Formatter(_FORMAT, datefmt=_DATE_FMT))
    root.addHandler(handler)

    # Map config booleans → child logger levels
    _level = lambda on: logging.DEBUG if on else logging.WARNING
    logging.getLogger("sage.general").setLevel(_level(cfg.general))
    logging.getLogger("sage.tokens").setLevel(_level(cfg.tokens))
    logging.getLogger("sage.tools").setLevel(_level(cfg.tools))
    logging.getLogger("sage.robot").setLevel(_level(cfg.robot))
    logging.getLogger("sage.tts").setLevel(_level(cfg.tts))
    logging.getLogger("sage.stt").setLevel(_level(cfg.stt))

    _configured = True


def get(name: str = "general") -> logging.Logger:
    """Return a namespaced child logger, e.g. ``logger.get("tools")``."""
    return logging.getLogger(f"sage.{name}")


def log_exception(where: str, logger_name: str = "general") -> None:
    """Log the current exception with full traceback."""
    log = get(logger_name)
    etype, exc, tb = sys.exc_info()
    log.error(
        "%s EXCEPTION: %s\n%s",
        where,
        exc,
        "".join(traceback.format_tb(tb)),
    )
