"""Teleop policy: gamepad/keyboard stick → body VelocityCommand (SI).

Stick axes stay −1..1 at the input HAL. This module is the only place that
applies engage/cruise and stick→SI scaling. Locomotion schedules consume
``VelocityCommand`` in m/s and rad/s and never see stick percentages.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from marsdog_control.motion.gait_schedule import VelocityCommand

# SoftTrot (T≈0.88s) mid-brisk cruise; near envelope max (~0.13 m/s).
# Half-speed bringup: 0.067. Legacy stick-norm 0.5 ≈ ~0.08 m/s on new recipe.
DEFAULT_CRUISE_VX_MPS = 0.100
DEFAULT_YAW_RATE_MAX = 0.40  # rad/s at |stick_yaw|=1 (matches spot_wz_scale)


@dataclass(frozen=True)
class TeleopPolicy:
    """Operator stick mapping (hardware-facing), not a gait parameter."""

    cruise_vx_mps: float = DEFAULT_CRUISE_VX_MPS
    yaw_rate_max: float = DEFAULT_YAW_RATE_MAX
    engage_threshold: float = 0.15  # |stick_vx| to start walking
    deadzone: float = 0.12  # |stick_yaw| / idle band
    # engage_cruise: stick depth ignored; fixed ±cruise_vx_mps when engaged.
    # proportional: |stick| scales 0→cruise (still capped by cruise_vx_mps).
    mode: str = "engage_cruise"


def stick_to_body_velocity(
    stick_vx: float,
    stick_yaw: float = 0.0,
    *,
    policy: TeleopPolicy | None = None,
    cruise_vx_mps: float | None = None,
    yaw_rate_max: float | None = None,
    engage_threshold: float | None = None,
    deadzone: float | None = None,
    mode: str | None = None,
) -> VelocityCommand:
    """Map stick (−1..1) → body twist command (m/s, rad/s)."""
    p = policy or TeleopPolicy()
    cruise = float(p.cruise_vx_mps if cruise_vx_mps is None else cruise_vx_mps)
    yaw_max = float(p.yaw_rate_max if yaw_rate_max is None else yaw_rate_max)
    thr = float(p.engage_threshold if engage_threshold is None else engage_threshold)
    dz = float(p.deadzone if deadzone is None else deadzone)
    map_mode = str(p.mode if mode is None else mode)

    sx = float(stick_vx)
    sy = float(stick_yaw)

    if abs(sx) <= thr:
        vx = 0.0
    elif map_mode == "proportional":
        span = max(1e-6, 1.0 - thr)
        u = max(0.0, min(1.0, (abs(sx) - thr) / span))
        vx = math.copysign(cruise * u, sx)
    else:
        # engage_cruise (default): on/off → fixed cruise speed
        vx = math.copysign(max(0.0, cruise), sx)

    if abs(sy) <= dz:
        yaw_rate = 0.0
    else:
        yaw_rate = max(-1.0, min(1.0, sy)) * yaw_max

    return VelocityCommand(vx=vx, yaw_rate=yaw_rate, vy=0.0)


def stick_yaw_to_rate(
    stick_yaw: float,
    *,
    yaw_rate_max: float = DEFAULT_YAW_RATE_MAX,
    deadzone: float = 0.12,
) -> float:
    """Convert a yaw stick (or yaw-hold stick-scale) to rad/s."""
    sy = float(stick_yaw)
    if abs(sy) <= deadzone:
        return 0.0
    return max(-1.0, min(1.0, sy)) * float(yaw_rate_max)


__all__ = [
    "DEFAULT_CRUISE_VX_MPS",
    "DEFAULT_YAW_RATE_MAX",
    "TeleopPolicy",
    "stick_to_body_velocity",
    "stick_yaw_to_rate",
]
