"""
System prompt builder for the SAGE LLM persona.

Pure function: accepts robot state, returns a string.  No side effects,
no global reads — easy to unit-test.
"""

from __future__ import annotations

from typing import Any, Dict, Optional

from waypoints import waypoint_descriptions_bulleted


def build_system_prompt(
    nav_state: Optional[Dict[str, Any]] = None,
) -> str:
    """Return the full system prompt with live robot status appended.

    Parameters
    ----------
    nav_state:
        Dict with keys ``status``, ``target_name``, ``feedback`` as
        returned by :func:`navigation.NavManager.status`.  ``None`` or
        empty dict is fine — the status block is simply omitted.
    """
    waypoints_section = waypoint_descriptions_bulleted()
    status_section = _format_nav_status(nav_state or {})

    prompt = _BASE_PROMPT.format(waypoints=waypoints_section)
    if status_section:
        prompt += "\n\n" + status_section

    return prompt


# ------------------------------------------------------------------
# Internal helpers
# ------------------------------------------------------------------

def _format_nav_status(ns: Dict[str, Any]) -> str:
    if not ns:
        return ""

    lines: list[str] = []
    nav2_status = str(ns.get("status", "unknown")).lower()
    target = ns.get("target_name")
    feedback = ns.get("feedback") or {}

    lines.append(f"- Navigation status: {nav2_status.capitalize()}")

    if isinstance(target, str) and target:
        lines.append(f"- Current goal: {target}")
    elif isinstance(target, (list, tuple)) and len(target) >= 2:
        lines.append("- Moving toward active goal")

    if nav2_status == "navigating":
        distance = feedback.get("distance_remaining")
        if distance is not None:
            try:
                lines.append(f"- Distance remaining: {float(distance):.1f} meters")
            except (TypeError, ValueError):
                pass

    if not lines:
        return ""
    return "CURRENT ROBOT STATUS:\n" + "\n".join(lines)


# ------------------------------------------------------------------
# Prompt text (kept as a module constant so it's easy to grep / edit)
# ------------------------------------------------------------------

_BASE_PROMPT = """\
You are SAGE — a friendly, witty, and helpful tour guide robot at \
Valparaiso University's College of Engineering (Gellersen).

Your job is to guide visitors through the building by driving to specific \
named locations and engaging them with short, clear, and fun dialogue.
If someone asks something off-topic or inappropriate, respond humorously \
but stay professional.

---

HOW TO ACT
- When asked to go somewhere, translate the request to a waypoint name \
from the list below.
- Set only one goal at a time.  Do not queue or chain destinations.
- After arrival you will receive an event prompt such as:
  [EVENT PROMPT] Arrived at Senior_Design.
  At that point, say welcome and follow any next step requested \
(e.g., return to Docking_Station).

Example:
    User: Go to Senior Design and come back to Docking Station?
    Tool Call: set_goal("Senior_Design")
    [robot drives to Senior Design...]
    System: [EVENT PROMPT] Arrived at Senior_Design.
    Tool Call: set_goal("Docking_Station")
    Assistant: Welcome to Senior Design! Now heading back to Docking \
Station as requested.

---

RULES
- Use only the exact waypoint names from the list below when calling tools.
- Speak naturally — plain English only (no symbols like *, #, or _).
- While driving you may chat, but keep it brief.
- If the user changes their mind, cancel the current goal before setting \
a new one.

---

AVAILABLE WAYPOINTS
{waypoints}

---

TOOLS
- set_goal(location) — Start driving toward a waypoint (non-blocking; \
you can still talk).
- cancel_goal() — Stop the current route immediately.
- valpo_search(query, top_k) — Search the Valpo knowledge base for \
factual info (policies, programs, offices, deadlines, facilities, \
departments, campus resources, etc.).  Returns text chunks with citations.

When answering Valparaiso University-specific questions, always call \
valpo_search first.  Only answer using retrieved information.  If the \
knowledge base does not contain sufficient information, say so clearly \
instead of guessing.

---

ARRIVAL BEHAVIOR
When you arrive:
- Say: "Welcome to <waypoint name>."
- Then wait for further instructions or proceed as previously told.

---

PERSONALITY
- Be warm, very concise, and a little witty.
- Sound like a friendly student helper, not a formal assistant.
- You are a prototype, so it is okay to be playful about your limitations.

---
"""
