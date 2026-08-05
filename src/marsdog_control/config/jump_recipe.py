"""Jump typed recipe — peer SSOT; never poured into Soft DynamicsConfig."""

from __future__ import annotations

from dataclasses import asdict, dataclass, fields, replace
from typing import Any, Mapping, Optional


@dataclass(frozen=True)
class JumpRecipe:
    """Canonical in-place hop timing / clearance / vertical gains."""

    height: float = 0.24
    crouch_depth: float = 0.050
    crouch_s: float = 0.30
    push_s: float = 0.14
    flight_s: float = 0.20
    land_s: float = 0.25
    recover_s: float = 0.28
    flight_clearance: float = 0.030
    land_compress: float = 0.014
    push_vz: float = 0.60
    push_extend: float = 0.022
    front_stand_foot_pitch_deg: float = -90.0
    kp_base_z: float = 80.0
    kd_base_z: float = 10.0

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    def with_overrides(self, overrides: Mapping[str, Any]) -> "JumpRecipe":
        known = {f.name for f in fields(self)}
        payload = {k: v for k, v in overrides.items() if k in known}
        return replace(self, **payload)


JUMP_RECIPE = JumpRecipe()

JUMP_RECIPE_WBC = JumpRecipe(
    crouch_depth=0.070,
    crouch_s=0.24,
    push_s=0.18,
    flight_s=0.30,
    land_s=0.34,
    recover_s=0.40,
    flight_clearance=0.075,
    land_compress=0.022,
    push_vz=2.2,
    push_extend=0.016,
    kp_base_z=140.0,
    kd_base_z=12.0,
)


def jump_recipe_dict(recipe: Optional[JumpRecipe] = None) -> dict[str, Any]:
    return (recipe or JUMP_RECIPE).to_dict()


__all__ = [
    "JUMP_RECIPE",
    "JUMP_RECIPE_WBC",
    "JumpRecipe",
    "jump_recipe_dict",
]
