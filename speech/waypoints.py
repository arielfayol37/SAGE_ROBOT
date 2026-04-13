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
    "BIO_ENG_LAB": {
        "frame_id": "map",
        "x": 0.9600775837898254, "y": 0.0006705378764308989,
        "ox": 0.0, "oy": 0.0, "oz": 0.00121317649625233, "ow": 0.9999992641011237,
        "description": (
            "Bio Engineering Lab, a student space for bioengineering projects "
            "and research."
        ),
    },
    "SENIOR_DESIGN": {
        "frame_id": "map",
        "x": 25.449546813964844, "y": 2.206489324569702,
        "ox": 0.0, "oy": 0.0, "oz": 0.03274886544333634, "ow": 0.9994636120500707,
        "description": (
            "Senior Design, a student space for senior design projects and collaboration."
        ),
    },
    "GUELLY_DELLY": {
        "frame_id": "map",
        "x": 26.889633178710938, "y": 23.772600173950195,
        "ox": 0.0, "oy": 0.0, "oz": 0.3905704048573187, "ow": 0.9205730600281491,
        "description": (
            "Guelly Delly, Engineering students cafe where students can get "
            "food and drinks.  Usually breakfast and lunch."
        ),
    },
    "MANUFACTURING_LAB": {
        "frame_id": "map",
        "x": 25.077194213867188, "y": 39.90303421020508,
        "ox": 0.0, "oy": 0.0, "oz": 0.6405226761724656, "ow": 0.7679392562624096,
        "description": (
            "Manufacturing Lab, a student space for manufacturing and "
            "fabrication projects."
        ),
    },
    "3D_PRINTING_LAB": {
        "frame_id": "map",
        "x": 30.92706871032715, "y": 48.677703857421875,
        "ox": 0.0, "oy": 0.0, "oz": 0.029439919652491053, "ow": 0.9995665516266813,
        "description": (
            "3D Printing Lab, a student space for 3D printing and fabrication projects."
        ),
    },
    "CLEAN_ROOM": {
        "frame_id": "map",
        "x": -0.44752734899520874, "y": 40.4676628112793,
        "ox": 0.0, "oy": 0.0, "oz": -0.9996298743603909, "ow": 0.027205041559041618,
        "description": (
            "Clean Room, a controlled environment for sensitive fabrication "
            "and research work."
        ),
    },
    "ECE_LAB_1": {
        "frame_id": "map",
        "x": -7.211533069610596, "y": 47.88065719604492,
        "ox": 0.0, "oy": 0.0, "oz": -0.707168915604687, "ow": 0.7070446413080939,
        "description": (
            "ECE Lab 1, a student space for ECE students to work on projects "
            "and collaborate.  Many workbenches and tools are available."
        ),
    },
    "ECE_LAB_2": {
        "frame_id": "map",
        "x": -7.666999340057373, "y": 35.37744140625,
        "ox": 0.0, "oy": 0.0, "oz": 0.731044131015225, "ow": 0.6823301829086813,
        "description": (
            "ECE Lab 2, a student space for ECE students to work on projects "
            "and collaborate.  Many workbenches and tools are available."
        ),
    },
    "MECHATRONICS_LAB": {
        "frame_id": "map",
        "x": 42.799068450927734, "y": 49.41830062866211,
        "ox": 0.0, "oy": 0.0, "oz": 0.052600369698898954, "ow": 0.9986156423306913,
        "description": (
            "Mechatronics Lab, a student space for mechatronics projects "
            "and research."
        ),
    },
    "VALPO_ROBOTICS": {
        "frame_id": "map",
        "x": 55.059078216552734, "y": 48.70744705200195,
        "ox": 0.0, "oy": 0.0, "oz": -0.7216045506340373, "ow": 0.6923054762922572,
        "description": (
            "Valpo Robotics, a student space for robotics research and development."
        ),
    },
    "HESSE_CENTER": {
        "frame_id": "map",
        "x": 22.28543472290039, "y": 80.14366912841797,
        "ox": 0.0, "oy": 0.0, "oz": 0.7410264705354664, "ow": 0.6714758148777582,
        "description": (
            "Hesse Center, a student space for tutoring.  Drop-in hours every "
            "weekday; many students study and get help with courses here."
        ),
    },
    "BATHROOMS": {
        "frame_id": "map",
        "x": 22.28849220275879, "y": 64.49700164794922,
        "ox": 0.0, "oy": 0.0, "oz": -0.6543723827346279, "ow": 0.7561724569925868,
        "description": (
            "Bathrooms, the nearest restroom facilities in the building."
        ),
    },
    "MATERIALS_TESTING_LAB": {
        "frame_id": "map",
        "x": 30.749574661254883, "y": 2.4156084060668945,
        "ox": 0.0, "oy": 0.0, "oz": 0.2852369510501596, "ow": 0.9584570317732605,
        "description": (
            "Materials Testing Lab, a space for materials science testing "
            "and experimentation."
        ),
    },
    "HEAT_POWER_LAB": {
        "frame_id": "map",
        "x": 13.389517784118652, "y": 1.1792845726013184,
        "ox": 0.0, "oy": 0.0, "oz": 0.7350446502996042, "ow": 0.6780187033304705,
        "description": (
            "Heat Power Lab, a space for thermodynamics and heat transfer "
            "projects and experimentation."
        ),
    },
    "TRANSPORTATION_LAB": {
        "frame_id": "map",
        "x": 45.20415496826172, "y": 50.20747756958008,
        "ox": 0.0, "oy": 0.0, "oz": 0.5885957291688836, "ow": 0.8084275277377374,
        "description": (
            "Transportation Lab, a space for transportation engineering "
            "projects and research."
        ),
    },
    "DOCKING_STATION": {
        "frame_id": "map",
        "x": 30.64848518371582, "y": 57.46519088745117,
        "ox": 0.0, "oy": 0.0, "oz": -0.635329496405893, "ow": 0.7722411741137832,
        "description": (
            "Docking Station, where the robot can charge and dock when not in use."
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