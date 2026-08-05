"""NaturalTrot spine yaw/roll oscillation (and expected-roll contribution).

Kept out of ``gait_controller.get_targets`` so Spot freeze + phase math stay
testable without the full IK loop.
"""

from __future__ import annotations

import math
from typing import Any, Dict, Tuple


def spine_ramp(t: float, ramp_duration: float) -> float:
    if ramp_duration > 0 and t < ramp_duration:
        s = t / ramp_duration
        return s * s * (3.0 - 2.0 * s)
    return 1.0


def spine_yaw_roll_osc(
    *,
    t: float,
    period: float,
    ramp: float,
    spine_yaw_deg: float,
    spine_roll_deg: float,
    spine_phase_deg: float,
    spine_roll_phase_deg: float,
    spot_turn_active: bool,
) -> Tuple[float, float]:
    """Return (yaw_osc_rad, roll_osc_rad). Spot freezes gait-locked spine."""
    if spot_turn_active:
        return 0.0, 0.0
    bp = (t / max(1e-6, period)) % 1.0
    yaw_osc = (
        math.radians(spine_yaw_deg)
        * math.sin(2.0 * math.pi * bp + math.radians(spine_phase_deg))
        * ramp
    )
    roll_osc = (
        math.radians(spine_roll_deg)
        * math.sin(2.0 * math.pi * bp + math.radians(spine_roll_phase_deg))
        * ramp
    )
    return yaw_osc, roll_osc


def apply_spine_osc_to_targets(
    targets: Dict[int, float],
    *,
    yaw_osc: float,
    roll_osc: float,
    joint_by_name: Dict[str, Any],
    clamp,
) -> Dict[int, float]:
    j_wy = joint_by_name["waist_yaw"]
    targets[j_wy.motor_id] = clamp(
        targets.get(j_wy.motor_id, 0.0) + yaw_osc,
        j_wy.limit_lo, j_wy.limit_hi)
    j_wr = joint_by_name["waist_roll"]
    targets[j_wr.motor_id] = clamp(
        targets.get(j_wr.motor_id, 0.0) + roll_osc,
        j_wr.limit_lo, j_wr.limit_hi)
    return targets


def expected_spine_roll_deg(
    *,
    t: float,
    period: float,
    ramp: float,
    spine_roll_deg: float,
    spine_roll_phase_deg: float,
) -> float:
    if abs(spine_roll_deg) < 0.01:
        return 0.0
    bp = (t / max(1e-6, period)) % 1.0
    return (
        spine_roll_deg
        * math.sin(2.0 * math.pi * bp + math.radians(spine_roll_phase_deg))
        * ramp
    )


__all__ = [
    "apply_spine_osc_to_targets",
    "expected_spine_roll_deg",
    "spine_ramp",
    "spine_yaw_roll_osc",
]
