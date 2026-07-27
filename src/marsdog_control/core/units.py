"""Small unit helpers used at configuration boundaries.

Internal runtime code should use SI units:
length in meters, angle in radians, time in seconds.
"""

from __future__ import annotations

import math
from typing import TypeVar

Number = TypeVar("Number", int, float)


def mm_to_m(value_mm: float) -> float:
    return value_mm / 1000.0


def m_to_mm(value_m: float) -> float:
    return value_m * 1000.0


def deg_to_rad(value_deg: float) -> float:
    return math.radians(value_deg)


def rad_to_deg(value_rad: float) -> float:
    return math.degrees(value_rad)


def ms_to_s(value_ms: float) -> float:
    return value_ms / 1000.0


def s_to_ms(value_s: float) -> float:
    return value_s * 1000.0


def clamp(value: Number, min_value: Number, max_value: Number) -> Number:
    return max(min_value, min(max_value, value))


__all__ = [
    "clamp",
    "deg_to_rad",
    "mm_to_m",
    "m_to_mm",
    "ms_to_s",
    "rad_to_deg",
    "s_to_ms",
]
