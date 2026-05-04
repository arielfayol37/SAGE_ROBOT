"""
LLM tool definitions and execution for SAGE.

Contains:
- OpenAI function-calling schemas (``TOOL_SCHEMAS``)
- Implementation functions (navigation + KB search)
- A ``ToolRegistry`` that maps tool names → callables for safe dispatch
"""

from __future__ import annotations

import json
import socket
import subprocess
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
                    "Start or update the navigation goal to a named waypoint. "
                    "Returns immediately with an acknowledgment, then continues "
                    "in the background. If the navigation succeeds, you'll be "
                    "told via an arrival event. If it fails (path blocked, "
                    "marker not visible, etc.), you'll be told via a failure "
                    "event with a reason — communicate that reason to the user "
                    "naturally and offer alternatives if appropriate. "
                    "DOCKING_STATION is special: navigating to it will dock the "
                    "robot at the charger. Going elsewhere from DOCKING_STATION "
                    "automatically undocks first."
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
        {
            "type": "function",
            "function": {
                "name": "web_search",
                "description": (
                    "Search the internet for current information.  Use this "
                    "for general knowledge questions, recent events, weather, "
                    "news, or anything NOT specific to Valparaiso University.  "
                    "Returns a list of result snippets with titles and URLs."
                ),
                "parameters": {
                    "type": "object",
                    "properties": {
                        "query": {
                            "type": "string",
                            "description": "The search query.",
                        },
                        "max_results": {
                            "type": "integer",
                            "description": "Number of results to return.",
                            "default": 5,
                            "minimum": 1,
                            "maximum": 10,
                        },
                    },
                    "required": ["query"],
                },
            },
        },
        {
            "type": "function",
            "function": {
                "name": "get_ip_address",
                "description": (
                    "Get the robot's current IP address(es) on the network.  "
                    "Use when someone asks what the robot's IP is, how to "
                    "connect to it, or its network address."
                ),
                "parameters": {"type": "object", "properties": {}},
            },
        },
        {
            "type": "function",
            "function": {
                "name": "dock_status",
                "description": (
                    "Get the current docking state of the robot. Returns one of: "
                    "docked, undocked, docking, undocking, or unknown. Use this "
                    "if the user asks whether you're charging, whether you're at "
                    "the dock, or before suggesting they wait while you dock/undock."
                ),
                "parameters": {"type": "object", "properties": {}},
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
        api_key_path: str = "api_keys/api_keys.json",
    ) -> None:
        self._nav = nav
        self._ui = ui
        self._endpoints = endpoints
        self._api_key_path = api_key_path

        self._lock = threading.Lock()
        self._nav_epoch: int = 0
        self._current_target: Optional[str] = None

        self._dispatch: Dict[str, Callable[..., Any]] = {
            "set_goal": self.set_goal,
            "cancel_goal": self.cancel_goal,
            "valpo_search": self.valpo_search,
            "web_search": self.web_search,
            "get_ip_address": self.get_ip_address,
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
        """Send a navigation goal (handles waypoints, docking, undocking)."""
        self._nav.ensure_started()

        with self._lock:
            self._nav_epoch += 1
            self._current_target = location

        return self._nav.set_goal_by_name(location)

    def cancel_goal(self) -> str:
        """Cancel any active navigation, dock, or undock goal."""
        self._nav.ensure_started()
        return self._nav.cancel_active()

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

    def web_search(self, query: str, max_results: int = 5) -> str:
        """Search the web using Tavily (optimised for LLM consumption).

        Reads the API key from the shared keys file (field:
        ``tavily_api_key``) or the ``TAVILY_API_KEY`` env var.
        Free tier: 1,000 searches/month, no credit card needed.
        Sign up at https://tavily.com
        """
        max_results = max(1, min(int(max_results), 10))
        self._ui.searching()

        try:
            from tavily import TavilyClient
        except ImportError:
            return json.dumps({
                "query": query,
                "results": [],
                "error": "tavily not installed.  Run: pip install tavily-python",
            })

        from utils import read_api_key
        api_key = read_api_key(self._api_key_path, "tavily_api_key", "TAVILY_API_KEY")
        if not api_key:
            return json.dumps({
                "query": query,
                "results": [],
                "error": (
                    "No Tavily API key found.  Add \"tavily_api_key\" to "
                    f"{self._api_key_path} or set TAVILY_API_KEY env var."
                ),
            })

        try:
            client = TavilyClient(api_key=api_key)
            response = client.search(
                query=query,
                max_results=max_results,
                search_depth="basic",
            )

            results = [
                {
                    "title": r.get("title", ""),
                    "content": r.get("content", ""),
                    "url": r.get("url", ""),
                }
                for r in response.get("results", [])
            ]
            _log.info("Web search '%s' → %d results", query, len(results))
            return json.dumps({"query": query, "results": results})

        except Exception as exc:
            _log.exception("Web search failed")
            return json.dumps({"query": query, "results": [], "error": f"Search failed: {exc}"})

    def get_ip_address(self) -> str:
        """Return the robot's current IP addresses."""
        addrs: Dict[str, List[str]] = {}

        # Method 1: parse `hostname -I` (works on Linux, gives all IPs)
        try:
            out = subprocess.check_output(
                ["hostname", "-I"], timeout=2, text=True,
            ).strip()
            if out:
                addrs["all"] = out.split()
        except Exception:
            pass

        # Method 2: connect-to-external trick (gives the "default" IP)
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.connect(("8.8.8.8", 80))
                addrs["primary"] = [s.getsockname()[0]]
        except Exception:
            pass

        if not addrs:
            return json.dumps({"error": "Could not determine IP address."})

        # Flatten to a clean response
        primary = addrs.get("primary", [None])[0]
        all_ips = addrs.get("all", [])
        result = {"primary_ip": primary, "all_ips": all_ips}
        _log.info("IP address query: %s", result)
        return json.dumps(result)
    
    def dock_status(self) -> str:
        self._nav.ensure_started()
        return self._nav.dock_state.value