"""
OpenAI streaming engine for SAGE.

Two layers:
1. :func:`stream_events` — low-level generator yielding ``("text", …)``
   and ``("tools", …)`` tuples from a single streamed completion.
2. :func:`run_conversation_turn` — high-level orchestrator that loops
   through tool calls, appends results, and returns the final assistant
   text.
"""

from __future__ import annotations

import json
import threading
from typing import Any, Dict, Generator, List, Optional, Sequence, Tuple, TYPE_CHECKING

import openai

import logger
from system_prompt import build_system_prompt

if TYPE_CHECKING:
    from config import LLMConfig
    from navigation import NavManager
    from piper_tts import PiperTTS
    from tools import ToolRegistry
    from ui_state_client import UIStatePublisher

_log = logger.get("general")
_log_tok = logger.get("tokens")
_log_tool = logger.get("tools")

# Type alias for the shared message list
Messages = List[Dict[str, Any]]


# ======================================================================
# Low-level streaming generator
# ======================================================================

def stream_events(
    client: openai.OpenAI,
    messages: Sequence[Dict[str, Any]],
    tools: Sequence[Dict[str, Any]],
    model: str,
) -> Generator[Tuple[str, Any], None, None]:
    """Yield ``("text", token_str)`` and ``("tools", calls_list)`` tuples.

    The ``"tools"`` event is emitted at most once, after the stream
    finishes with ``finish_reason == "tool_calls"``.
    """
    pending: Dict[int, Dict[str, Any]] = {}
    finish_reason: Optional[str] = None

    _log.debug("OpenAI stream start (model=%s, msgs=%d)", model, len(messages))

    response = client.chat.completions.create(
        model=model,
        messages=list(messages),
        tools=list(tools) if tools else [],
        tool_choice="auto",
        stream=True,
    )

    for chunk in response:
        if not chunk.choices:
            continue
        choice = chunk.choices[0]
        delta = getattr(choice, "delta", None)
        if delta is None:
            continue

        # --- text tokens ---
        content = getattr(delta, "content", None)
        if content:
            _log_tok.debug("token: %r", content)
            yield ("text", content)

        # --- tool-call deltas ---
        tc_deltas = getattr(delta, "tool_calls", None)
        if tc_deltas:
            for tc in tc_deltas:
                idx = tc.index
                entry = pending.setdefault(idx, {"id": None, "name": None, "args": ""})
                if tc.id:
                    entry["id"] = tc.id
                if tc.function and tc.function.name:
                    entry["name"] = tc.function.name
                if tc.function and tc.function.arguments:
                    entry["args"] += tc.function.arguments
                _log_tool.debug(
                    "tool-delta idx=%d id=%s name=%s args_len=%d",
                    idx, entry["id"], entry["name"], len(entry["args"]),
                )

        if choice.finish_reason:
            finish_reason = choice.finish_reason

    if finish_reason == "tool_calls" and pending:
        ordered = [pending[i] for i in sorted(pending)]
        _log_tool.debug("tool-calls finished with %d call(s)", len(ordered))
        yield ("tools", ordered)


# ======================================================================
# High-level orchestrator
# ======================================================================

def run_conversation_turn(
    *,
    client: openai.OpenAI,
    messages: Messages,
    messages_lock: threading.Lock,
    tool_schemas: List[Dict[str, Any]],
    tool_registry: ToolRegistry,
    nav: NavManager,
    tts: PiperTTS,
    ui: UIStatePublisher,
    config: LLMConfig,
) -> str:
    """Execute a full LLM turn, including multi-pass tool loops.

    Mutates *messages* (under *messages_lock*) and returns the final
    assistant text.
    """
    depth = 0

    while True:
        ui.thinking()

        # Interrupt any leftover TTS from a previous turn
        if tts.is_playing():
            _log.debug("Stopping TTS (new LLM pass)")
            try:
                tts.stop()
            except Exception:
                pass

        # Build request with a fresh system prompt each pass
        system_prompt = build_system_prompt(nav_state=nav.status())
        with messages_lock:
            request_messages = [
                {"role": "system", "content": system_prompt},
                *messages,
            ]

        _log.info("LLM streaming pass (depth=%d)", depth)

        text_parts: list[str] = []
        tool_calls: Optional[List[Dict[str, Any]]] = None

        try:
            for kind, payload in stream_events(
                client, request_messages, tool_schemas, config.model,
            ):
                if kind == "text":
                    text_parts.append(payload)
                elif kind == "tools":
                    tool_calls = payload
                    break
        except Exception:
            logger.log_exception("LLM streaming loop")

        # ----- tool execution path ------------------------------------
        if tool_calls:
            # Record the assistant's tool-call message
            tc_msg: Dict[str, Any] = {
                "role": "assistant",
                "content": "",
                "tool_calls": [
                    {
                        "id": c["id"],
                        "type": "function",
                        "function": {
                            "name": c["name"],
                            "arguments": c.get("args") or "{}",
                        },
                    }
                    for c in tool_calls
                ],
            }
            with messages_lock:
                messages.append(tc_msg)

            # Execute each tool and append results
            for call in tool_calls:
                _log_tool.info("Running tool: %s", call["name"])
                result = tool_registry.execute(call["name"], call.get("args", ""))
                _log_tool.info("Tool result: %s", result)
                with messages_lock:
                    messages.append({
                        "role": "tool",
                        "tool_call_id": call["id"],
                        "content": result,
                    })

            depth += 1
            if depth > config.max_tool_depth:
                _log_tool.warning("Reached max tool depth (%d); stopping.", config.max_tool_depth)
                assistant_text = "".join(text_parts)
                if assistant_text.strip():
                    with messages_lock:
                        messages.append({"role": "assistant", "content": assistant_text})
                return assistant_text

            continue  # another streaming pass

        # ----- text-only path (final) ---------------------------------
        assistant_text = "".join(text_parts)
        _log.info("Assistant text length=%d (depth=%d)", len(assistant_text), depth)

        if assistant_text.strip():
            tts.say(assistant_text, block=False, interrupt=True)
            with messages_lock:
                messages.append({"role": "assistant", "content": assistant_text})

        return assistant_text
