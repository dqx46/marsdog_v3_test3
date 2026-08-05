"""Per-tick prep for StableTrot.get_targets (turn LPF / Spot / lateral / Raibert).

Keeps trajectory scheduling out of the StableTrot class body.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from marsdog_control.motion.spot_orchestration import (
    gait_entry_ramp,
    waist_yaw_turn_cmd,
    write_waist_joints,
)


@dataclass(frozen=True)
class TrotTickPrep:
    """Inputs shared by front/rear IK writers for one control tick."""

    ramp: float
    lat_offset: float
    reactive: float
    dx_raibert: float
    z_front_base: float
    z_rear_base: float


def prepare_stable_trot_tick(
    gait,
    t: float,
    *,
    imu_state: Optional[dict],
    front_hip_offset: float,
    rear_hip_offset: float,
    clamp,
) -> TrotTickPrep:
    """Filter turn, Spot pose, entry ramp, lateral/reactive/Raibert deltas."""
    imu_state = imu_state or {}

    gait._turn_filtered += gait.turn_filter_alpha * (
        gait._turn_cmd - gait._turn_filtered)
    spot_on = bool(getattr(gait, "spot_turn_active", False))
    if spot_on:
        gait._spot_update_pose(t, imu_state)

    ramp = gait_entry_ramp(
        t,
        spot_turn_active=spot_on,
        ramp_duration=float(gait.ramp_duration),
    )

    z_front_base = -(gait.body_height - front_hip_offset)
    z_rear_base = -(gait.body_height - rear_hip_offset)

    if spot_on:
        lat_offset = 0.0
    else:
        lat_offset = gait._lateral_offset(t) * ramp

    if imu_state and not spot_on:
        roll = imu_state.get("roll", 0.0)
        gyro_roll = imu_state.get("gyro_roll", 0.0)
        raw = gait.reactive_kp * roll + gait.reactive_kd * gyro_roll
        raw = clamp(raw, -0.10, 0.10)
        gait._reactive_filtered += 0.15 * (raw - gait._reactive_filtered)
    elif spot_on:
        gait._reactive_filtered *= 0.9
    reactive = gait._reactive_filtered * ramp

    from marsdog_control.motion.sagittal_raibert import update_raibert_from_imu
    dx_raibert = update_raibert_from_imu(gait, imu_state)

    return TrotTickPrep(
        ramp=ramp,
        lat_offset=lat_offset,
        reactive=reactive,
        dx_raibert=dx_raibert,
        z_front_base=z_front_base,
        z_rear_base=z_rear_base,
    )


def apply_trot_waist(gait, targets: dict, *, t: float, ramp: float,
                     joint_by_name, clamp) -> None:
    """Write waist pitch/yaw after leg IK."""
    write_waist_joints(
        targets,
        waist_pitch=float(gait.waist_pitch_offset),
        waist_yaw=waist_yaw_turn_cmd(
            t=t,
            period=float(gait.period),
            turn_filtered=float(gait._turn_filtered),
            ramp=ramp,
            spot_turn_active=bool(getattr(gait, "spot_turn_active", False)),
            waist_yaw_offset=float(gait.waist_yaw_offset),
            waist_yaw_turn_sign=float(gait.waist_yaw_turn_sign),
            max_turn_waist_yaw=float(gait.max_turn_waist_yaw),
            spot_waist_yaw_rad=float(gait.spot_waist_yaw_rad),
            spot_waist_yaw_pulse_rad=float(gait.spot_waist_yaw_pulse_rad),
        ),
        joint_by_name=joint_by_name,
        clamp=clamp,
    )


__all__ = [
    "TrotTickPrep",
    "prepare_stable_trot_tick",
    "apply_trot_waist",
]
