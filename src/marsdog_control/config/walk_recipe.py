"""NaturalWalk typed recipe — peer SSOT to SoftTrotRecipe (not Soft shape)."""

from __future__ import annotations

from dataclasses import asdict, dataclass, fields, replace
from typing import Any, Mapping, Optional


@dataclass(frozen=True)
class WalkRecipe:
    """Canonical NaturalWalk geometry (four-beat slow walk)."""

    height: float = 0.24
    period: float = 1.05
    stance: float = 0.75
    amp_front: float = 0.050
    amp_rear: float = 0.058
    step_h: float = 0.034
    step_h_front: float = 0.032
    touchdown_compress: float = 0.006
    anti_roll_soft_scale: float = 0.0
    toeoff_lift: float = 0.007
    retract_peak: float = 0.22
    lift_peak: float = 0.26
    thigh_swing_front_deg: float = 0.0
    thigh_swing_rear_deg: float = 12.0
    retract_front: float = 0.030
    retract_rear: float = 0.034
    tarsus_swing_deg: float = 0.0
    swing_clearance_per_rad: float = 0.35
    front_thrust_gain: float = 1.0
    front_thrust_swing_gain: float = 1.0
    front_tarsus_push: float = 0.0
    front_foot_track_deg: float = -78.0
    front_foot_stance_push_deg: float = 8.0
    front_foot_swing_track: float = 0.0
    front_stand_foot_pitch_deg: float = -90.0
    spine_yaw_deg: float = 5.5
    spine_roll_deg: float = 2.5
    spine_phase_deg: float = 0.0
    lateral_sway: float = 0.010
    com_sway_m: float = 0.024
    anti_roll: float = 0.0
    anti_roll_asym_neg: float = 1.0
    anti_roll_asym_pos: float = 1.0
    trot_roll_ff_neg_deg: float = 0.0
    trot_roll_ff_pos_deg: float = 0.0
    ff_decouple: bool = True
    rear_clearance_m: float = 0.018
    throttle_min_scale: float = 0.55
    # Optional WBC-only dynamics pour (not applied to Soft DynamicsConfig).
    kp_base_roll: Optional[float] = None
    kd_base_roll: Optional[float] = None

    def to_dict(self) -> dict[str, Any]:
        out = asdict(self)
        # Compat aliases consumed by older natural_params readers.
        out["nat_period"] = self.period
        out["nat_amp_front"] = self.amp_front
        out["nat_amp_rear"] = self.amp_rear
        out["nat_step_h"] = self.step_h
        out["fwd_front_lift"] = self.step_h_front
        if out.get("kp_base_roll") is None:
            out.pop("kp_base_roll", None)
        if out.get("kd_base_roll") is None:
            out.pop("kd_base_roll", None)
        return out

    def with_overrides(self, overrides: Mapping[str, Any]) -> "WalkRecipe":
        known = {f.name for f in fields(self)}
        payload = {k: v for k, v in overrides.items() if k in known}
        return replace(self, **payload)


WALK_RECIPE = WalkRecipe()

WALK_RECIPE_WBC = WalkRecipe(
    amp_front=0.054,
    amp_rear=0.062,
    step_h=0.036,
    step_h_front=0.034,
    period=1.00,
    stance=0.75,
    lateral_sway=0.009,
    com_sway_m=0.026,
    spine_yaw_deg=5.0,
    spine_roll_deg=2.4,
    spine_phase_deg=0.0,
    retract_front=0.032,
    retract_rear=0.036,
    retract_peak=0.22,
    lift_peak=0.26,
    toeoff_lift=0.008,
    touchdown_compress=0.006,
    rear_clearance_m=0.020,
    throttle_min_scale=0.50,
    kp_base_roll=74.0,
    kd_base_roll=22.0,
)


def walk_recipe_dict(recipe: Optional[WalkRecipe] = None) -> dict[str, Any]:
    return (recipe or WALK_RECIPE).to_dict()


__all__ = [
    "WALK_RECIPE",
    "WALK_RECIPE_WBC",
    "WalkRecipe",
    "walk_recipe_dict",
]
