"""
LLM tool definitions and execution for SAGE.

Contains:
- OpenAI function-calling schemas (``TOOL_SCHEMAS``)
- Implementation functions (navigation + KB search)
- A ``ToolRegistry`` that maps tool names → callables for safe dispatch
"""

from __future__ import annotations

import json
import threading
from typing import Any, Callable, Dict, List, Optional, TYPE_CHECKING

import requests

import logger
from waypoints import WAYPOINTS, waypoint_names

if TYPE_CHECKING:
    from navigation import NavManager
    from ui_state_client import UIStatePublisher
    from config import EndpointsConfig

_log = logger.get("tools")


# ======================================================================
# Schemas (sent to the OpenAI API)
# ======================================================================

def build_tool_schemas() -> List[Dict[str, Any]]:
    """Return the list of tool schemas with current waypoint names baked in."""
    return [
        {
            "type": "function",
            "function": {
                "name": "set_goal",
                "description": (
                    "Start or update Nav2 goal to a named waypoint.  "
                    "Returns immediately."
                ),
                "parameters": {
                    "type": "object",
                    "properties": {
                        "location": {
                            "type": "string",
                            "description": (
                                f"One of: {', '.join(waypoint_names())}"
                            ),
                        },
                    },
                    "required": ["location"],
                },
            },
        },
        {
            "type": "function",
            "function": {
                "name": "cancel_goal",
                "description": "Cancel the current Nav2 goal (if any).",
                "parameters": {"type": "object", "properties": {}},
            },
        },
        {
            "type": "function",
            "function": {
                "name": "valpo_search",
                "description": (
                    "Search the local Valparaiso University knowledge base "
                    "for factual information.  Use this whenever the user "
                    "asks about Valpo policies, programs, offices, deadlines, "
                    "facilities, departments, campus resources, or other "
                    "university-specific details.  Returns relevant text "
                    "chunks with citations."
                ),
                "parameters": {
                    "type": "object",
                    "properties": {
                        "query": {
                            "type": "string",
                            "description": (
                                "Natural-language search query describing "
                                "the information needed."
                            ),
                        },
                        "top_k": {
                            "type": "integer",
                            "description": (
                                "Maximum number of relevant chunks to "
                                "retrieve from the knowledge base."
                            ),
                            "default": 3,
                            "minimum": 1,
                            "maximum": 20,
                        },
                    },
                    "required": ["query"],
                },
            },
        },
    ]


# ======================================================================
# Implementation functions
# ======================================================================

class ToolRegistry:
    """Holds tool callables and the shared state they need.

    All navigation epoch/target bookkeeping is protected by a lock so
    the event dispatcher thread and the main LLM thread cannot race.
    """

    def __init__(
        self,
        nav: NavManager,
        ui: UIStatePublisher,
        endpoints: EndpointsConfig,
    ) -> None:
        self._nav = nav
        self._ui = ui
        self._endpoints = endpoints

        self._lock = threading.Lock()
        self._nav_epoch: int = 0
        self._current_target: Optional[str] = None

        self._dispatch: Dict[str, Callable[..., Any]] = {
            "set_goal": self.set_goal,
            "cancel_goal": self.cancel_goal,
            "valpo_search": self.valpo_search,
        }

    # -- properties (thread-safe reads) --------------------------------

    @property
    def nav_epoch(self) -> int:
        with self._lock:
            return self._nav_epoch

    @property
    def current_target(self) -> Optional[str]:
        with self._lock:
            return self._current_target

    # -- dispatch ------------------------------------------------------

    def execute(self, name: str, raw_args: str) -> str:
        """Parse *raw_args* as JSON and call the named tool.

        Returns a string result (for injection into the message history).
        """
        try:
            args = json.loads(raw_args) if raw_args else {}
        except (json.JSONDecodeError, TypeError):
            _log.warning("Tool args JSON parse failed for %s; using empty dict", name)
            args = {}

        func = self._dispatch.get(name)
        if func is None:
            return f"Unknown tool: {name}"

        try:
            result = func(**args)
        except Exception as exc:
            _log.exception("Tool '%s' raised", name)
            return f"Tool '{name}' error: {exc}"

        _log.debug("Tool %s → %s", name, result)
        return str(result) if not isinstance(result, str) else result

    # -- tool implementations ------------------------------------------

    def set_goal(self, location: str) -> str:
        """Send a Nav2 goal for a named waypoint."""
        self._nav.ensure_started()

        if location not in WAYPOINTS:
            valid = ", ".join(waypoint_names())
            return f"Unknown location '{location}'.  Valid: {valid}"

        wp = WAYPOINTS[location]
        with self._lock:
            self._nav_epoch += 1
            self._current_target = location

        msg = self._nav.node.set_goal(
            frame_id=wp.get("frame_id", "map"),
            x=wp["x"], y=wp["y"],
            ox=wp["ox"], oy=wp["oy"], oz=wp["oz"], ow=wp["ow"],
            location_name=location,
        )
        return f"{msg} Target={location}."

    def cancel_goal(self) -> str:
        """Cancel any active Nav2 goal."""
        self._nav.ensure_started()
        return self._nav.node.cancel_goal()

    def valpo_search(self, query: str, top_k: int = 3) -> str:
        """Query the local Valpo knowledge-base search endpoint."""
        top_k = max(1, min(int(top_k), 20))
        self._ui.searching()

        url = self._endpoints.kb_search
        timeout = (
            self._endpoints.kb_connect_timeout,
            self._endpoints.kb_read_timeout,
        )

        try:
            resp = requests.get(url, params={"q": query, "k": top_k}, timeout=timeout)
            resp.raise_for_status()
            data = resp.json()
            results = [
                {
                    "text": r.get("text", ""),
                    "chunk_id": r.get("chunk_id"),
                    "citation": r.get("citation", {}),
                }
                for r in data.get("results", [])
            ]
            return json.dumps({"query": query, "top_k": top_k, "results": results})

        except requests.Timeout:
            return json.dumps({"query": query, "results": [], "error": "KB search timed out."})
        except requests.RequestException as exc:
            return json.dumps({"query": query, "results": [], "error": f"KB request failed: {exc}"})
        except ValueError as exc:
            return json.dumps({"query": query, "results": [], "error": f"Invalid JSON from KB: {exc}"})
