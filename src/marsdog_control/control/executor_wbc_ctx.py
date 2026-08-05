"""Shared per-tick context for WBC pipeline helpers."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


_LEGS = ("fl", "fr", "rl", "rr")


@dataclass
class WbcTickCtx:
    """Mutable bag passed through contact → estimate → cmd → acc → QP → apply."""

    state: Any
    targets: dict
    active_gait: Any
    t_rel: float
    use_est: bool
    vel_xyz: list
    current_base_z: float
    q_pin: Any
    v_pin: Any
    foot_z: dict
    foot_vz: dict
    foot_pos: dict
    foot_vel: dict
    leg_is_stance: dict
    contact_snap: Any = None
    spot: bool = False
    jump_now: bool = False
    hold_still: bool = False
    scrub_brake: bool = False
    vx_cmd: float = 0.0
    vy_cmd: float = 0.0
    wz_cmd: float = 0.0
    vz_cmd: float = 0.0
    target_z: float = 0.24
    target_roll: float = 0.0
    target_pitch: float = 0.0
    base_acc_des: Any = None
    swing_acc: Any = None
    foot_pos_des: Any = None
    stance_acc: Any = None
    f_c_des: Any = None
    force_scale: dict = field(default_factory=dict)
    vx_brake: float = 0.0
    vy_brake: float = 0.0
    vx_for_track: float = 0.0
    vy_for_damp: float = 0.0


__all__ = ["WbcTickCtx", "_LEGS"]
