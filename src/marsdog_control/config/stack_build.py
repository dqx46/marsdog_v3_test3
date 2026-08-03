"""Typed snapshots for one-shot control-stack construction.

CLI ``argparse.Namespace`` stops at the app/startup boundary. Motion / FSM /
IMU factories consume these frozen dataclasses instead of digging into ``args``
field-by-field. Hot-path code (``RuntimeStateMachine``) never sees a Namespace.

Gait-only argparse fallbacks come from ``config.gait_tuning.GAIT`` (shared with
``walk_cli``). SoftTrot shape is applied via ``NATURAL_SOFT_TROT_REAL`` before
``from_args``.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Optional

from marsdog_control.config.gait_tuning import GAIT


def _g(args: Any, name: str, default):
    return getattr(args, name, default)


@dataclass(frozen=True)
class FsmDriveConfig:
    """Everything the FSM needs each tick for stick drive / yaw-hold / throttle.

    ``cruise_vx`` is SI m/s (teleop cruise when stick engages). Stick −1..1
    mapping lives in ``input.teleop_policy``; schedules eat SI only.
    """

    gp_trot_threshold: float = 0.15
    gp_deadzone: float = 0.12
    throttle_min_scale: float = 0.5
    # Teleop cruise speed [m/s] after stick engage (sim ``--vx``). Not stick %.
    cruise_vx: float = 0.100
    yaw_rate_max: float = 0.40  # rad/s at |stick_yaw|=1
    yaw_hold: bool = False
    yaw_hold_kp: float = 0.0
    yaw_hold_kd: float = 0.0
    yaw_hold_sign: float = 1.0
    yaw_hold_limit: float = 0.5  # stick-scale clamp for yaw-hold output
    cruise_turn_scale: float = 0.6
    cruise_turn_yamp: float = 1.0
    amp_front: float = 0.026
    amp_rear: float = 0.026
    bwd_amp_scale: float = 0.7
    turn_sign: float = 1.0

    @classmethod
    def from_args(
        cls,
        args: Any,
        *,
        gp_trot_threshold: Optional[float] = None,
        gp_deadzone: Optional[float] = None,
    ) -> "FsmDriveConfig":
        from marsdog_control.input.teleop_policy import (
            DEFAULT_CRUISE_VX_MPS,
            DEFAULT_YAW_RATE_MAX,
        )
        return cls(
            gp_trot_threshold=(
                0.15 if gp_trot_threshold is None
                else float(gp_trot_threshold)),
            gp_deadzone=(
                0.12 if gp_deadzone is None else float(gp_deadzone)),
            throttle_min_scale=float(_g(
                args, "throttle_min_scale", GAIT.throttle_min_scale)),
            cruise_vx=float(_g(args, "cruise_vx", DEFAULT_CRUISE_VX_MPS)),
            yaw_rate_max=float(_g(args, "yaw_rate_max", DEFAULT_YAW_RATE_MAX)),
            yaw_hold=bool(_g(args, "yaw_hold", False)),
            yaw_hold_kp=float(_g(args, "yaw_hold_kp", 0.0)),
            yaw_hold_kd=float(_g(args, "yaw_hold_kd", 0.0)),
            yaw_hold_sign=float(_g(args, "yaw_hold_sign", 1.0)),
            yaw_hold_limit=float(_g(args, "yaw_hold_limit", 0.5)),
            cruise_turn_scale=float(_g(
                args, "cruise_turn_scale", GAIT.cruise_turn_scale)),
            cruise_turn_yamp=float(_g(
                args, "cruise_turn_yamp", GAIT.cruise_turn_yamp)),
            amp_front=float(_g(args, "amp_front", 0.026)),
            amp_rear=float(_g(args, "amp_rear", 0.026)),
            bwd_amp_scale=float(_g(args, "bwd_amp_scale", GAIT.bwd_amp_scale)),
            turn_sign=float(_g(args, "turn_sign", GAIT.turn_sign)),
        )


@dataclass(frozen=True)
class GaitStackConfig:
    """All scalars ``build_controller_set`` needs — snapshotted once from CLI."""

    height: float
    x_shift: float
    hip_abd: float
    waist_pitch: float
    waist_yaw_offset: float
    front_stand_foot_pitch_deg: Optional[float]
    front_stand_tarsus_deg: float
    amp_front: float
    amp_rear: float
    step_h: float
    step_h_front: float
    period: float
    stance: float
    ramp: float
    reactive_kp: float
    reactive_kd: float
    lateral_sway: float
    front_thrust_gain: float
    front_thrust_swing_gain: float
    front_tarsus_push: float
    front_foot_track_deg: float
    front_foot_stance_push_deg: float
    front_foot_swing_track: float
    swing_clearance_per_rad: float
    anti_roll: float
    trot_roll_ff_neg_deg: float
    trot_roll_ff_pos_deg: float
    anti_roll_asym_neg: float
    anti_roll_asym_pos: float
    bwd_amp_scale: float
    bwd_period: float
    bwd_step_h: float
    fwd_use_bwd: bool
    fwd_front_amp_scale: float
    fwd_front_lift: float
    pace_amp: float
    pace_step_h: float
    pace_period: float
    pace_stance: float
    pace_hip_abd: float
    pace_sway: float
    natural_soft_trot: bool
    nat_amp_front: float
    nat_amp_rear: float
    nat_period: float
    nat_step_h: float
    spine_yaw_deg: float
    spine_roll_deg: float
    spine_phase_deg: float
    thigh_swing_front_deg: float
    thigh_swing_rear_deg: float
    retract_front: float
    retract_rear: float
    tarsus_swing_deg: float
    touchdown_compress: float
    anti_roll_soft_scale: float
    toeoff_lift: float
    retract_peak: float
    lift_peak: float
    turn_amp_diff: float
    turn_y_amp: float
    turn_smooth: float
    turn_waist_yaw: float
    waist_yaw_turn_sign: float

    @classmethod
    def from_args(cls, args: Any) -> "GaitStackConfig":
        step_h = float(args.step_h)
        return cls(
            height=float(args.height),
            x_shift=float(_g(args, "x_shift", GAIT.x_shift)),
            hip_abd=float(args.hip_abd),
            waist_pitch=float(_g(args, "waist_pitch", GAIT.waist_pitch)),
            waist_yaw_offset=float(_g(
                args, "waist_yaw_offset", GAIT.waist_yaw_offset)),
            front_stand_foot_pitch_deg=_g(
                args, "front_stand_foot_pitch_deg",
                GAIT.front_stand_foot_pitch_deg),
            front_stand_tarsus_deg=float(_g(
                args, "front_stand_tarsus_deg", GAIT.front_stand_tarsus_deg)),
            amp_front=float(args.amp_front),
            amp_rear=float(args.amp_rear),
            step_h=step_h,
            step_h_front=float(args.step_h_front) if getattr(args, "step_h_front", None) else 0.0,
            period=float(args.period),
            stance=float(args.stance),
            ramp=float(args.ramp),
            reactive_kp=float(_g(args, "reactive_kp", GAIT.reactive_kp)),
            reactive_kd=float(_g(args, "reactive_kd", GAIT.reactive_kd)),
            lateral_sway=float(_g(args, "lateral_sway", GAIT.lateral_sway)),
            front_thrust_gain=float(_g(
                args, "front_thrust_gain", GAIT.front_thrust_gain)),
            front_thrust_swing_gain=float(_g(
                args, "front_thrust_swing_gain", GAIT.front_thrust_swing_gain)),
            front_tarsus_push=float(_g(
                args, "front_tarsus_push", GAIT.front_tarsus_push)),
            front_foot_track_deg=float(_g(
                args, "front_foot_track_deg", GAIT.front_foot_track_deg)),
            front_foot_stance_push_deg=float(_g(
                args, "front_foot_stance_push_deg",
                GAIT.front_foot_stance_push_deg)),
            front_foot_swing_track=float(_g(
                args, "front_foot_swing_track", GAIT.front_foot_swing_track)),
            swing_clearance_per_rad=float(_g(
                args, "swing_clearance_per_rad", GAIT.swing_clearance_per_rad)),
            anti_roll=float(_g(args, "anti_roll", GAIT.anti_roll)),
            trot_roll_ff_neg_deg=float(_g(
                args, "trot_roll_ff_neg_deg", GAIT.trot_roll_ff_neg_deg)),
            trot_roll_ff_pos_deg=float(_g(
                args, "trot_roll_ff_pos_deg", GAIT.trot_roll_ff_pos_deg)),
            anti_roll_asym_neg=float(_g(
                args, "anti_roll_asym_neg", GAIT.anti_roll_asym_neg)),
            anti_roll_asym_pos=float(_g(
                args, "anti_roll_asym_pos", GAIT.anti_roll_asym_pos)),
            bwd_amp_scale=float(_g(args, "bwd_amp_scale", GAIT.bwd_amp_scale)),
            bwd_period=float(_g(args, "bwd_period", GAIT.bwd_period)),
            bwd_step_h=float(_g(args, "bwd_step_h", GAIT.bwd_step_h)),
            fwd_use_bwd=bool(_g(args, "fwd_use_bwd", GAIT.fwd_use_bwd)),
            fwd_front_amp_scale=float(_g(
                args, "fwd_front_amp_scale", GAIT.fwd_front_amp_scale)),
            fwd_front_lift=float(_g(
                args, "fwd_front_lift", GAIT.fwd_front_lift)),
            pace_amp=float(_g(args, "pace_amp", GAIT.pace_amp)),
            pace_step_h=float(_g(args, "pace_step_h", GAIT.pace_step_h)),
            pace_period=float(_g(args, "pace_period", GAIT.pace_period)),
            pace_stance=float(_g(args, "pace_stance", GAIT.pace_stance)),
            pace_hip_abd=float(_g(args, "pace_hip_abd", GAIT.pace_hip_abd)),
            pace_sway=float(_g(args, "pace_sway", GAIT.pace_sway)),
            natural_soft_trot=bool(_g(args, "natural_soft_trot", True)),
            nat_amp_front=float(_g(args, "nat_amp_front", GAIT.nat_amp_front)),
            nat_amp_rear=float(_g(args, "nat_amp_rear", GAIT.nat_amp_rear)),
            nat_period=float(_g(args, "nat_period", GAIT.nat_period)),
            nat_step_h=float(_g(args, "nat_step_h", GAIT.nat_step_h)),
            spine_yaw_deg=float(_g(args, "spine_yaw_deg", GAIT.spine_yaw_deg)),
            spine_roll_deg=float(_g(args, "spine_roll_deg", GAIT.spine_roll_deg)),
            spine_phase_deg=float(_g(args, "spine_phase_deg", GAIT.spine_phase_deg)),
            thigh_swing_front_deg=float(_g(
                args, "thigh_swing_front_deg", GAIT.thigh_swing_front_deg)),
            thigh_swing_rear_deg=float(_g(
                args, "thigh_swing_rear_deg", GAIT.thigh_swing_rear_deg)),
            retract_front=float(_g(args, "retract_front", GAIT.retract_front)),
            retract_rear=float(_g(args, "retract_rear", GAIT.retract_rear)),
            tarsus_swing_deg=float(_g(args, "tarsus_swing_deg", GAIT.tarsus_swing_deg)),
            touchdown_compress=float(_g(
                args, "touchdown_compress", GAIT.touchdown_compress)),
            anti_roll_soft_scale=float(_g(
                args, "anti_roll_soft_scale", GAIT.anti_roll_soft_scale)),
            toeoff_lift=float(_g(args, "toeoff_lift", GAIT.toeoff_lift)),
            retract_peak=float(_g(args, "retract_peak", GAIT.retract_peak)),
            lift_peak=float(_g(args, "lift_peak", GAIT.lift_peak)),
            turn_amp_diff=float(_g(args, "turn_amp_diff", GAIT.turn_amp_diff)),
            turn_y_amp=float(_g(args, "turn_y_amp", GAIT.turn_y_amp)),
            turn_smooth=float(_g(args, "turn_smooth", GAIT.turn_smooth)),
            turn_waist_yaw=float(_g(args, "turn_waist_yaw", GAIT.turn_waist_yaw)),
            waist_yaw_turn_sign=float(_g(
                args, "waist_yaw_turn_sign", GAIT.waist_yaw_turn_sign)),
        )


@dataclass(frozen=True)
class ImuBuildConfig:
    """Scalars for constructing ``ImuAttitudeController`` (one-shot)."""

    imu_test: bool
    natural_trot: bool
    imu_kp: float
    max_corr_mm: float
    imu_ema: float
    damp_hard_mm: float
    damp_gyro_lo: float
    damp_gyro_hi: float
    roll_p_boost: float
    roll_p_lo_deg: float
    roll_p_hi_deg: float
    roll_trim_mm: float
    pitch_trim_mm: float
    auto_trim: bool
    auto_trim_rate: float
    auto_trim_limit_mm: float
    trim_phases: int
    imu_predict_ms: float
    imu_predict_max_ms: float
    imu_gyro_max_age_ms: float
    dynamic_imu_predict: bool

    @classmethod
    def from_args(cls, args: Any) -> "ImuBuildConfig":
        return cls(
            imu_test=bool(args.imu_test),
            natural_trot=bool(args.natural_trot),
            imu_kp=float(args.imu_kp),
            max_corr_mm=float(args.max_corr_mm),
            imu_ema=float(_g(args, "imu_ema", GAIT.imu_ema)),
            damp_hard_mm=float(_g(args, "damp_hard_mm", GAIT.damp_hard_mm)),
            damp_gyro_lo=float(_g(args, "damp_gyro_lo", GAIT.damp_gyro_lo)),
            damp_gyro_hi=float(_g(args, "damp_gyro_hi", GAIT.damp_gyro_hi)),
            roll_p_boost=float(_g(args, "roll_p_boost", GAIT.roll_p_boost)),
            roll_p_lo_deg=float(_g(args, "roll_p_lo_deg", GAIT.roll_p_lo_deg)),
            roll_p_hi_deg=float(_g(args, "roll_p_hi_deg", GAIT.roll_p_hi_deg)),
            roll_trim_mm=float(args.roll_trim_mm),
            pitch_trim_mm=float(args.pitch_trim_mm),
            auto_trim=bool(args.auto_trim),
            auto_trim_rate=float(args.auto_trim_rate),
            auto_trim_limit_mm=float(args.auto_trim_limit_mm),
            trim_phases=int(args.trim_phases),
            imu_predict_ms=float(args.imu_predict_ms),
            imu_predict_max_ms=float(args.imu_predict_max_ms),
            imu_gyro_max_age_ms=float(args.imu_gyro_max_age_ms),
            dynamic_imu_predict=bool(args.dynamic_imu_predict),
        )


__all__ = [
    "FsmDriveConfig",
    "GaitStackConfig",
    "ImuBuildConfig",
]
