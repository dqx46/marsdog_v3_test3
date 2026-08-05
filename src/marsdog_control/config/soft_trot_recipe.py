"""SoftTrot SSOT — geometry / foot / lateral balance / attitude overlay.

Soft may pour geometry/foot/balance/attitude/impedance packages. IMU foot-balance and WBC / Dynamics knobs
live in ``schema.ImuConfig`` / ``DynamicsConfig`` (not Soft).

``SoftBalanceOverlay`` is lateral-only (com_shift / sway).
``SoftAttitudeOverlay`` holds roll position patches (anti_roll / roll_ff / …)
— not LateralOwner; Phase 3 may add AttitudeOverlayOwner.
``leg_kp_scale`` sits on SoftImpedanceAssist (session Soft; Jump/Spot overlay separate).
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, fields, replace
from typing import Any, Mapping, Optional


# CLI / natural_params alias → canonical SoftTrotRecipe field.
_CLI_ALIASES = {
    "nat_period": "period",
    "nat_amp_front": "amp_front",
    "nat_amp_rear": "amp_rear",
    "nat_step_h": "step_h",
}


@dataclass(frozen=True)
class SoftGeometry:
    """Body height, cadence, swing amp, throttle / turn kinematics."""

    height: float = 0.25
    period: float = 1.05
    stance: float = 0.72
    amp_front: float = 0.022
    amp_rear: float = 0.030
    step_h: float = 0.024
    step_h_front: float = 0.020
    fwd_front_lift: float = 0.020
    fwd_front_amp_scale: float = 1.0
    throttle_min_scale: float = 0.45
    turn_y_amp: float = 0.040
    turn_amp_diff: float = 0.012
    turn_waist_yaw: float = 0.40


@dataclass(frozen=True)
class SoftImpedanceAssist:
    """Soft session default for joint-PD softening (not foot geometry).

    Maps to ``ControlConfig.leg_kp_scale`` / ``ImpedanceAssist``; orthogonal
    to ``ForceMode`` τ_ff ownership.
    """

    leg_kp_scale: float = 0.90


@dataclass(frozen=True)
class SoftFootShape:
    """Foot contact / swing shaping (compress, retract, track).

    Spine / swing_level live on ``SoftAttitudeOverlay`` (gated by
    ``AttitudeOverlayGate``); this package is foot-path only.
    """

    touchdown_compress: float = 0.0035
    anti_roll_soft_scale: float = 0.0
    toeoff_lift: float = 0.0008
    retract_peak: float = 0.42
    lift_peak: float = 0.48
    thigh_swing_front_deg: float = 0.0
    thigh_swing_rear_deg: float = 0.0
    retract_front: float = 0.010
    retract_rear: float = 0.008
    tarsus_swing_deg: float = 0.0
    swing_clearance_per_rad: float = 0.35
    rear_clearance_m: float = 0.0
    front_thrust_gain: float = 1.0
    front_thrust_swing_gain: float = 1.0
    front_tarsus_push: float = 0.0
    front_foot_track_deg: float = -78.0
    front_foot_stance_push_deg: float = 0.0
    front_foot_swing_track: float = 1.0
    front_stand_foot_pitch_deg: float = -90.0


@dataclass(frozen=True)
class SoftBalanceOverlay:
    """Lateral CoM only — owned by ``LateralOwner`` (default COM_SHIFT).

    Soft default: ``com_shift_m`` on, ``lateral_sway`` must stay 0.
    """

    com_shift_m: float = 0.004
    com_shift_blend: float = 0.15
    lateral_sway: float = 0.0


@dataclass(frozen=True)
class SoftAttitudeOverlay:
    """Kinematic attitude patches — gated by ``AttitudeOverlayGate``.

    Includes Soft roll patches, IMU swing prelevel weight, and torso spine.
    Defaults off for Soft real; Walk recipes may set spine_*.
    """

    anti_roll: float = 0.0
    anti_roll_asym_neg: float = 1.0
    anti_roll_asym_pos: float = 1.0
    trot_roll_ff_neg_deg: float = 0.0
    trot_roll_ff_pos_deg: float = 0.0
    ff_decouple: bool = False
    swing_level: float = 0.0  # IMU prelevel weight (AttitudeOwner.IMU only)
    spine_yaw_deg: float = 0.0
    spine_roll_deg: float = 0.0


def _flat_from_parts(
    geometry: SoftGeometry,
    foot: SoftFootShape,
    balance: SoftBalanceOverlay,
    attitude: SoftAttitudeOverlay,
    impedance: SoftImpedanceAssist | None = None,
) -> dict[str, Any]:
    out: dict[str, Any] = {}
    out.update(asdict(geometry))
    out.update(asdict(foot))
    out.update(asdict(balance))
    out.update(asdict(attitude))
    out.update(asdict(impedance or SoftImpedanceAssist()))
    return out


@dataclass(frozen=True)
class SoftTrotRecipe:
    """Facade: Geometry + FootShape + Balance + Attitude + ImpedanceAssist.

    2026-08-03 lock D cadence; 2026-08-04 real walk: com_shift=4mm.
    No IMU / WBC pour fields — those live in schema only.
    """

    # ── SoftGeometry ──
    height: float = 0.25
    period: float = 1.05
    stance: float = 0.72
    amp_front: float = 0.022
    amp_rear: float = 0.030
    step_h: float = 0.024
    step_h_front: float = 0.020
    fwd_front_lift: float = 0.020
    fwd_front_amp_scale: float = 1.0
    leg_kp_scale: float = 0.90
    throttle_min_scale: float = 0.45
    turn_y_amp: float = 0.040
    turn_amp_diff: float = 0.012
    turn_waist_yaw: float = 0.40

    # ── SoftFootShape ──
    touchdown_compress: float = 0.0035
    anti_roll_soft_scale: float = 0.0
    toeoff_lift: float = 0.0008
    retract_peak: float = 0.42
    lift_peak: float = 0.48
    thigh_swing_front_deg: float = 0.0
    thigh_swing_rear_deg: float = 0.0
    retract_front: float = 0.010
    retract_rear: float = 0.008
    tarsus_swing_deg: float = 0.0
    swing_clearance_per_rad: float = 0.35
    rear_clearance_m: float = 0.0
    front_thrust_gain: float = 1.0
    front_thrust_swing_gain: float = 1.0
    front_tarsus_push: float = 0.0
    front_foot_track_deg: float = -78.0
    front_foot_stance_push_deg: float = 0.0
    front_foot_swing_track: float = 1.0
    front_stand_foot_pitch_deg: float = -90.0

    # ── SoftBalanceOverlay (lateral) ──
    com_shift_m: float = 0.004
    com_shift_blend: float = 0.15
    lateral_sway: float = 0.0

    # ── SoftAttitudeOverlay ──
    anti_roll: float = 0.0
    anti_roll_asym_neg: float = 1.0
    anti_roll_asym_pos: float = 1.0
    trot_roll_ff_neg_deg: float = 0.0
    trot_roll_ff_pos_deg: float = 0.0
    ff_decouple: bool = False
    swing_level: float = 0.0
    spine_yaw_deg: float = 0.0
    spine_roll_deg: float = 0.0

    @classmethod
    def from_parts(
        cls,
        geometry: Optional[SoftGeometry] = None,
        foot: Optional[SoftFootShape] = None,
        balance: Optional[SoftBalanceOverlay] = None,
        attitude: Optional[SoftAttitudeOverlay] = None,
        impedance: Optional[SoftImpedanceAssist] = None,
    ) -> "SoftTrotRecipe":
        return cls(**_flat_from_parts(
            geometry or SoftGeometry(),
            foot or SoftFootShape(),
            balance or SoftBalanceOverlay(),
            attitude or SoftAttitudeOverlay(),
            impedance or SoftImpedanceAssist(),
        ))

    def as_impedance(self) -> SoftImpedanceAssist:
        return SoftImpedanceAssist(leg_kp_scale=self.leg_kp_scale)

    def as_geometry(self) -> SoftGeometry:
        return SoftGeometry(**{
            f.name: getattr(self, f.name) for f in fields(SoftGeometry)
        })

    def as_foot(self) -> SoftFootShape:
        return SoftFootShape(**{
            f.name: getattr(self, f.name) for f in fields(SoftFootShape)
        })

    def as_balance(self) -> SoftBalanceOverlay:
        return SoftBalanceOverlay(**{
            f.name: getattr(self, f.name) for f in fields(SoftBalanceOverlay)
        })

    def as_attitude(self) -> SoftAttitudeOverlay:
        return SoftAttitudeOverlay(**{
            f.name: getattr(self, f.name) for f in fields(SoftAttitudeOverlay)
        })

    @property
    def nat_period(self) -> float:
        return self.period

    @property
    def nat_amp_front(self) -> float:
        return self.amp_front

    @property
    def nat_amp_rear(self) -> float:
        return self.amp_rear

    @property
    def nat_step_h(self) -> float:
        return self.step_h

    def to_dict(self) -> dict[str, Any]:
        out = asdict(self)
        out["nat_period"] = self.period
        out["nat_amp_front"] = self.amp_front
        out["nat_amp_rear"] = self.amp_rear
        out["nat_step_h"] = self.step_h
        return out

    def with_overrides(self, overrides: Mapping[str, Any]) -> "SoftTrotRecipe":
        known = {f.name for f in fields(self)}
        payload: dict[str, Any] = {}
        for key, value in overrides.items():
            canon = _CLI_ALIASES.get(key, key)
            if canon in known:
                payload[canon] = value
        return replace(self, **payload)

    def shape_dict(self) -> dict[str, Any]:
        return {k: getattr(self, k) for k in SHAPE_FIELDS}

    def control_pour_dict(self) -> dict[str, Any]:
        return {}


SOFT_TROT_RECIPE = SoftTrotRecipe()

GEOMETRY_FIELDS = frozenset(f.name for f in fields(SoftGeometry))
FOOT_SHAPE_FIELDS = frozenset(f.name for f in fields(SoftFootShape))
BALANCE_OVERLAY_FIELDS = frozenset(f.name for f in fields(SoftBalanceOverlay))
ATTITUDE_OVERLAY_FIELDS = frozenset(f.name for f in fields(SoftAttitudeOverlay))
IMPEDANCE_ASSIST_FIELDS = frozenset(f.name for f in fields(SoftImpedanceAssist))

SHAPE_FIELDS = (
    GEOMETRY_FIELDS | FOOT_SHAPE_FIELDS
    | BALANCE_OVERLAY_FIELDS | ATTITUDE_OVERLAY_FIELDS
    | IMPEDANCE_ASSIST_FIELDS
)

CONTROL_POUR_FIELDS: frozenset[str] = frozenset()

SOFT_FORBIDDEN_POUR_FIELDS = frozenset({
    "imu_kp", "imu_kp_pitch", "max_corr_mm", "imu_slew_mm_s",
    "imu_predict_ms", "imu_softstart_s", "dynamic_imu_predict",
    "imu_phase_gate", "imu_phase_td_gain", "imu_phase_swing_gain",
    "td_imu_freeze_i", "tarsus_lead_fl_ms", "tarsus_lead_fr_ms",
    "dm_dq_feedforward",
    "kp_base_roll", "kd_base_roll", "lateral_vel_damp",
    "swing_foot_kp", "com_y_shift_m",
})

NON_GEOMETRY_SOFT_FIELDS = frozenset({"leg_kp_scale"})  # SoftImpedanceAssist
# Migrated into SoftAttitudeOverlay — kept empty for sync-test compat.
ATTITUDE_ADJACENT_FOOT_FIELDS: frozenset[str] = frozenset()

SCHEMA_GEOMETRY_FROM_RECIPE = (
    ("height", "gait", "body_height_m"),
    ("period", "gait", "period_s"),
    ("step_h", "gait", "step_height_m"),
    ("step_h_front", "gait", "front_step_height_m"),
    ("amp_front", "gait", "amp_front_m"),
    ("amp_rear", "gait", "amp_rear_m"),
    ("stance", "gait", "stance_ratio"),
    ("leg_kp_scale", "control", "leg_kp_scale"),
)


def soft_trot_recipe_dict(
    recipe: Optional[SoftTrotRecipe] = None,
) -> dict[str, Any]:
    return (recipe or SOFT_TROT_RECIPE).to_dict()


__all__ = [
    "ATTITUDE_ADJACENT_FOOT_FIELDS",
    "ATTITUDE_OVERLAY_FIELDS",
    "BALANCE_OVERLAY_FIELDS",
    "CONTROL_POUR_FIELDS",
    "FOOT_SHAPE_FIELDS",
    "GEOMETRY_FIELDS",
    "NON_GEOMETRY_SOFT_FIELDS",
    "SCHEMA_GEOMETRY_FROM_RECIPE",
    "SHAPE_FIELDS",
    "SOFT_FORBIDDEN_POUR_FIELDS",
    "SOFT_TROT_RECIPE",
    "IMPEDANCE_ASSIST_FIELDS",
    "SoftAttitudeOverlay",
    "SoftBalanceOverlay",
    "SoftFootShape",
    "SoftGeometry",
    "SoftImpedanceAssist",
    "SoftTrotRecipe",
    "soft_trot_recipe_dict",
]
