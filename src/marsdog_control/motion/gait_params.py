"""Typed gait construction params — shrink StableTrot / SoftTrot call sites.

Defaults derive from ``SOFT_TROT_RECIPE`` (no third hand-copied number set).
``build_controller_set`` builds one ``GaitParams`` / ``SoftTrotBuild`` instead of
spreading 20–45 kwargs. Controllers still accept classic kwargs for tests.
"""

from __future__ import annotations

from dataclasses import dataclass, fields, replace
from typing import Any, Mapping, Optional

from marsdog_control.config.soft_trot_recipe import SOFT_TROT_RECIPE as _R
from marsdog_control.config.schema import GaitConfig as _GaitSchema


_GAIT_SCHEMA = _GaitSchema()


@dataclass(frozen=True)
class GaitParams:
    """Core geometry shared by StableTrot / NaturalTrot / SoftTrot."""

    body_height: float = _R.height
    amp_front: float = _R.amp_front
    amp_rear: float = _R.amp_rear
    step_height: float = _R.step_h
    step_height_front: Optional[float] = _R.step_h_front
    period: float = _R.period
    stance_ratio: float = _R.stance
    x_offset_front: Optional[float] = None
    x_offset_rear: Optional[float] = None
    hip_abduction: float = _GAIT_SCHEMA.hip_abduction_rad
    lateral_sway: float = _R.lateral_sway
    anti_roll: float = _R.anti_roll
    reactive_kp: float = 0.0
    reactive_kd: float = 0.0
    ramp_duration: float = _GAIT_SCHEMA.ramp_s
    front_thrust_gain: float = _R.front_thrust_gain
    front_thrust_swing_gain: float = _R.front_thrust_swing_gain
    front_tarsus_push: float = _R.front_tarsus_push
    front_foot_track_deg: Optional[float] = _R.front_foot_track_deg
    front_foot_stance_push_deg: float = _R.front_foot_stance_push_deg
    front_foot_swing_track: float = _R.front_foot_swing_track
    front_stand_tarsus_deg: float = 0.0
    front_stand_foot_pitch_deg: Optional[float] = _R.front_stand_foot_pitch_deg
    swing_clearance_per_rad: float = _R.swing_clearance_per_rad
    trot_roll_ff_neg_deg: float = _R.trot_roll_ff_neg_deg
    trot_roll_ff_pos_deg: float = _R.trot_roll_ff_pos_deg
    anti_roll_asym_neg: float = _R.anti_roll_asym_neg
    anti_roll_asym_pos: float = _R.anti_roll_asym_pos
    swing_level: float = _R.swing_level
    smooth_gait: bool = False

    def with_overrides(self, **kwargs: Any) -> "GaitParams":
        known = {f.name for f in fields(self)}
        return replace(self, **{k: v for k, v in kwargs.items() if k in known})

    def as_stable_trot_kwargs(self) -> dict[str, Any]:
        return {f.name: getattr(self, f.name) for f in fields(self)}


@dataclass(frozen=True)
class NaturalExtras:
    """NaturalTrot flourish / spine fields layered on ``GaitParams``."""

    spine_yaw_deg: float = _R.spine_yaw_deg
    spine_roll_deg: float = _R.spine_roll_deg
    spine_phase_deg: float = 0.0
    thigh_swing_front_deg: float = _R.thigh_swing_front_deg
    thigh_swing_rear_deg: float = _R.thigh_swing_rear_deg
    retract_front: float = _R.retract_front
    retract_rear: float = _R.retract_rear
    tarsus_swing_deg: float = _R.tarsus_swing_deg

    def as_kwargs(self) -> dict[str, Any]:
        return {f.name: getattr(self, f.name) for f in fields(self)}


@dataclass(frozen=True)
class SoftExtras:
    """NaturalSoftTrot-only foot-shape extras."""

    touchdown_compress: float = _R.touchdown_compress
    anti_roll_soft_scale: float = _R.anti_roll_soft_scale
    toeoff_lift: float = _R.toeoff_lift
    retract_peak: float = _R.retract_peak
    lift_peak: float = _R.lift_peak
    rear_clearance_m: float = _R.rear_clearance_m
    com_shift_m: float = _R.com_shift_m
    com_shift_blend: float = _R.com_shift_blend

    def as_kwargs(self) -> dict[str, Any]:
        return {f.name: getattr(self, f.name) for f in fields(self)}

    @classmethod
    def from_mapping(cls, data: Mapping[str, Any]) -> "SoftExtras":
        known = {f.name for f in fields(cls)}
        return cls(**{k: data[k] for k in known if k in data})


@dataclass(frozen=True)
class SoftTrotBuild:
    """One bundle for constructing NaturalSoftTrot."""

    gait: GaitParams
    natural: NaturalExtras = NaturalExtras()
    soft: SoftExtras = SoftExtras()

    def as_kwargs(self) -> dict[str, Any]:
        out = self.gait.as_stable_trot_kwargs()
        out.update(self.natural.as_kwargs())
        out.update(self.soft.as_kwargs())
        return out

    @classmethod
    def from_gait_stack(
        cls,
        cfg: Any,
        *,
        x_offset_front: float,
        x_offset_rear: float,
        hip_abduction: float,
        spine_yaw_deg: Optional[float] = None,
        spine_roll_deg: Optional[float] = None,
    ) -> "SoftTrotBuild":
        """Build from poured ``GaitStackConfig`` — no natural_params dict."""
        step_h = float(cfg.nat_step_h)
        if cfg.fwd_front_lift > 1e-6:
            step_h_front = float(cfg.fwd_front_lift)
        elif cfg.step_h_front:
            step_h_front = float(cfg.step_h_front)
        else:
            step_h_front = step_h * 0.75
        gait = cfg.shared_gait_params(
            x_offset_front=x_offset_front,
            x_offset_rear=x_offset_rear,
            hip_abduction=hip_abduction,
            amp_front=float(cfg.nat_amp_front),
            amp_rear=float(cfg.nat_amp_rear),
            step_height=step_h,
            step_height_front=step_h_front,
            period=float(cfg.nat_period),
            stance_ratio=float(cfg.stance),
            front_tarsus_push=0.0,
            front_thrust_gain=1.0,
            front_thrust_swing_gain=1.0,
        )
        natural = NaturalExtras(
            spine_yaw_deg=float(
                cfg.spine_yaw_deg if spine_yaw_deg is None else spine_yaw_deg),
            spine_roll_deg=float(
                cfg.spine_roll_deg if spine_roll_deg is None else spine_roll_deg),
            spine_phase_deg=float(cfg.spine_phase_deg),
            thigh_swing_front_deg=float(cfg.thigh_swing_front_deg),
            thigh_swing_rear_deg=float(cfg.thigh_swing_rear_deg),
            retract_front=float(cfg.retract_front),
            retract_rear=float(cfg.retract_rear),
            tarsus_swing_deg=float(cfg.tarsus_swing_deg),
        )
        soft = SoftExtras(
            touchdown_compress=float(cfg.touchdown_compress),
            anti_roll_soft_scale=float(cfg.anti_roll_soft_scale),
            toeoff_lift=float(cfg.toeoff_lift),
            retract_peak=float(cfg.retract_peak),
            lift_peak=float(cfg.lift_peak),
            rear_clearance_m=float(cfg.rear_clearance_m),
            com_shift_m=float(cfg.com_shift_m),
            com_shift_blend=float(cfg.com_shift_blend),
        )
        return cls(gait, natural, soft)


__all__ = [
    "GaitParams",
    "NaturalExtras",
    "SoftExtras",
    "SoftTrotBuild",
]
