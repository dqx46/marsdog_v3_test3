"""Spot / cruise ramp + waist-yaw turn bias for StableTrot.get_targets.

Keeps the long IK loop from owning Spot timing / spine-turn scheduling.
"""

from __future__ import annotations

import math
from typing import Any, Dict


def gait_entry_ramp(
    t: float,
    *,
    spot_turn_active: bool,
    ramp_duration: float,
    spot_blend_s: float = 0.25,
) -> float:
    """Smoothstep ramp into gait. Spot uses a short blend (no 1s starve)."""
    if spot_turn_active:
        blend = float(spot_blend_s)
        if blend > 0 and t < blend:
            s = t / blend
            return s * s * (3.0 - 2.0 * s)
        return 1.0
    if ramp_duration > 0 and t < ramp_duration:
        s = t / ramp_duration
        return s * s * (3.0 - 2.0 * s)
    return 1.0


def waist_yaw_turn_cmd(
    *,
    t: float,
    period: float,
    turn_filtered: float,
    ramp: float,
    spot_turn_active: bool,
    waist_yaw_offset: float,
    waist_yaw_turn_sign: float,
    max_turn_waist_yaw: float,
    spot_waist_yaw_rad: float,
    spot_waist_yaw_pulse_rad: float,
) -> float:
    """Absolute waist_yaw command (before joint limit clamp)."""
    if spot_turn_active:
        turn = float(turn_filtered)
        bias = (
            turn
            * float(spot_waist_yaw_rad)
            * ramp
            * waist_yaw_turn_sign
        )
        bp = (t / max(1e-6, period)) % 1.0
        pulse = (
            turn
            * float(spot_waist_yaw_pulse_rad)
            * math.sin(2.0 * math.pi * bp)
            * ramp
            * waist_yaw_turn_sign
        )
        return float(waist_yaw_offset) + bias + pulse
    return (
        float(waist_yaw_offset)
        + float(turn_filtered)
        * float(max_turn_waist_yaw)
        * ramp
        * float(waist_yaw_turn_sign)
    )


def write_waist_joints(
    targets: Dict[int, float],
    *,
    waist_pitch: float,
    waist_yaw: float,
    joint_by_name: Dict[str, Any],
    clamp,
) -> None:
    j_wp = joint_by_name["waist_pitch"]
    targets[j_wp.motor_id] = clamp(
        waist_pitch, j_wp.limit_lo, j_wp.limit_hi)
    j_wy = joint_by_name["waist_yaw"]
    targets[j_wy.motor_id] = clamp(
        waist_yaw, j_wy.limit_lo, j_wy.limit_hi)


__all__ = [
    "gait_entry_ramp",
    "waist_yaw_turn_cmd",
    "write_waist_joints",
]
