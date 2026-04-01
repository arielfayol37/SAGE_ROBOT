"""
Named waypoints for the Gellersen Engineering building.

Each entry maps a human-readable name to a full ``PoseStamped``-style
dictionary used by Nav2.
"""

from __future__ import annotations

from typing import Dict, TypedDict


class Waypoint(TypedDict, total=False):
    frame_id: str
    x: float
    y: float
    ox: float
    oy: float
    oz: float
    ow: float
    description: str


WAYPOINTS: Dict[str, Waypoint] = {
    "Docking_Station": {
        "frame_id": "map",
        "x": 0.0, "y": 0.0,
        "ox": 0.0, "oy": 0.0, "oz": 0.0, "ow": 0.0,
        "description": (
            "Docking Station, where the robot can charge and dock when not in use."
        ),
    },
    "Senior_Design": {
        "frame_id": "map",
        "x": 5.020315170288086, "y": 0.5106609463691711,
        "ox": 0.0, "oy": 0.0, "oz": 0.6982647334138825, "ow": 0.7158396203553137,
        "description": (
            "Senior Design, a student space for senior design projects and collaboration."
        ),
    },
    "Guelly_Delly": {
        "frame_id": "map",
        "x": 22.26424217224121, "y": -3.0069425106048584,
        "ox": 0.0, "oy": 0.0, "oz": 0.058206536467100216, "ow": 0.9983045623017578,
        "description": (
            "Guelly Delly, Engineering students cafe where students can get "
            "food and drinks.  Usually breakfast and lunch."
        ),
    },
    "GE_100": {
        "frame_id": "map",
        "x": 20.999507904052734, "y": -2.9191231727600098,
        "ox": 0.0, "oy": 0.0, "oz": 0.07292237288309265, "ow": 0.9973376196319375,
        "description": (
            "General Engineering 100, a room where freshmen take their first "
            "engineering course and learn about different engineering disciplines."
        ),
    },
    "ECE_LAB_1": {
        "frame_id": "map",
        "x": 55.77067565917969, "y": 29.38323402404785,
        "ox": 0.0, "oy": 0.0, "oz": -0.7288416827551458, "ow": 0.6846822631547039,
        "description": (
            "ECE Lab 1, a student space for ECE students to work on projects "
            "and collaborate.  Many workbenches and tools are available."
        ),
    },
    "ECE_LAB_2": {
        "frame_id": "map",
        "x": 42.942630767822266, "y": 31.10202980041504,
        "ox": 0.0, "oy": 0.0, "oz": -0.982344960779826, "ow": 0.1870785343925971,
        "description": (
            "ECE Lab 2, a student space for ECE students to work on projects "
            "and collaborate.  Many workbenches and tools are available."
        ),
    },
    "HESSI_CENTER": {
        "frame_id": "map",
        "x": 83.51174926757812, "y": -6.202152252197266,
        "ox": 0.0, "oy": 0.0, "oz": 0.010512876726070833, "ow": 0.9999447381845371,
        "description": (
            "Hessi Center, a student space for tutoring.  Drop-in hours every "
            "weekday; many students study and get help with courses here."
        ),
    },
    "3D_PRINTING_LAB": {
        "frame_id": "map",
        "x": 50.47529983520508, "y": -14.694130897521973,
        "ox": 0.0, "oy": 0.0, "oz": 0.009561396317804793, "ow": 0.9999542888054703,
        "description": (
            "3D Printing Lab, a student space for 3D printing and fabrication projects."
        ),
    },
    "VALPO_ROBOTICS": {
        "frame_id": "map",
        "x": 46.69096374511719, "y": -32.61865234375,
        "ox": 0.0, "oy": 0.0, "oz": 0.9991932193368056, "ow": 0.040161056153322494,
        "description": (
            "Valpo Robotics, a student space for robotics research and development."
        ),
    },
    "ROOM_120": {
        "frame_id": "map",
        "x": 80.69095611572266, "y": -1.1842193603515625,
        "ox": 0.0, "oy": 0.0, "oz": -0.017718008621271535, "ow": 0.999843023764479,
        "description": (
            "Room 120, a classroom near the Gellersen entrance."
        ),
    },
}


def waypoint_names() -> list[str]:
    """Sorted list of all known waypoint identifiers."""
    return sorted(WAYPOINTS.keys())


def waypoint_descriptions_bulleted() -> str:
    """Formatted string for embedding in the system prompt."""
    return "\n".join(
        f"- {name}: {WAYPOINTS[name]['description']}"
        for name in waypoint_names()
    )
