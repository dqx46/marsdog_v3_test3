"""Core contracts and runtime primitives.

Note: ``RuntimeConfig``/``FeatureFlags`` are intentionally NOT re-exported here.
``core.config``/``core.flags`` import from ``config.schema``, and ``config.schema``
imports ``core.units`` — eagerly re-exporting them here would make importing
this package import ``config.schema`` before it finishes initializing,
causing a circular import. Import them directly from
``marsdog_control.core.config`` / ``marsdog_control.core.flags`` (or from
``marsdog_control.config``) instead.
"""

from marsdog_control.core.types import (
    BehaviorChannels,
    ControlOutput,
    Controller,
    Direction,
    MotionTarget,
    RobotMode,
    RobotState,
    SafetyReport,
    UserCommand,
)
from marsdog_control.core.units import (
    clamp,
    deg_to_rad,
    mm_to_m,
    m_to_mm,
    ms_to_s,
    rad_to_deg,
    s_to_ms,
)

__all__ = [
    "BehaviorChannels",
    "ControlOutput",
    "Controller",
    "Direction",
    "MotionTarget",
    "RobotMode",
    "RobotState",
    "SafetyReport",
    "UserCommand",
    "clamp",
    "deg_to_rad",
    "mm_to_m",
    "m_to_mm",
    "ms_to_s",
    "rad_to_deg",
    "s_to_ms",
]
