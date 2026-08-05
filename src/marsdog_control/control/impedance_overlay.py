"""Transient joint-impedance overlays — layered on session ImpedanceAssist.

Session Soft ``leg_kp_scale`` (ImpedanceAssist) is the baseline for the walk.
Jump phase stiffness and Spot abd boost are temporary overlays resolved here;
they must not mutate ``ExecutorConfig.leg_kp_scale`` / policies.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Optional


# Spot: restore effective abd kp ≈ 1.4 when session Soft softens legs (×0.90).
_SPOT_ABD_TARGET_KP = 1.4
_SPOT_ABD_JOINTS = (
    "fl_thigh_roll", "fr_thigh_roll", "rl_hip", "rr_hip",
)


@dataclass(frozen=True)
class ImpedanceLayers:
    """Resolved impedance for one control tick."""

    session_leg_kp: float
    effective_leg_kp: float
    spot_abd_boost_active: bool


def jump_phase_leg_kp(session_scale: float, active_gait: Any, t_rel: float) -> float:
    """Jump phase temporary leg kp — clamps around session Soft baseline."""
    leg_kp_out = float(session_scale)
    if active_gait is None or getattr(active_gait, "family", None) != "jump":
        return leg_kp_out
    phase_name = getattr(getattr(active_gait, "phase", None), "value", "")
    if phase_name == "crouch":
        return max(leg_kp_out, 1.25)
    if phase_name == "push":
        return min(max(leg_kp_out, 1.00), 1.20)
    if phase_name == "flight":
        u = 0.0
        if hasattr(active_gait, "_phase_u"):
            try:
                u = float(active_gait._phase_u(t_rel))
            except Exception:
                u = 0.0
        if u < 0.15:
            return min(max(leg_kp_out, 1.40), 1.55)
        return min(max(leg_kp_out, 0.45), 0.65)
    if phase_name == "land":
        return min(max(leg_kp_out, 0.85), 1.10)
    if phase_name == "recover":
        return min(max(leg_kp_out, 0.55), 0.75)
    return leg_kp_out


def apply_spot_abd_kp_boost(
    kp_phase: Optional[Dict[int, float]],
    *,
    session_leg_kp: float,
    joint_by_name: Dict[str, Any],
) -> Dict[int, float]:
    """Per-joint abd boost so Spot holds hip roll under Soft session scale."""
    out = dict(kp_phase or {})
    leg_s = max(1e-3, float(session_leg_kp))
    boost = _SPOT_ABD_TARGET_KP / leg_s
    for jname in _SPOT_ABD_JOINTS:
        if jname in joint_by_name:
            mid = joint_by_name[jname].motor_id
            out[mid] = max(float(out.get(mid, 1.0)), boost)
    return out


def resolve_impedance_layers(
    session_leg_kp: float,
    active_gait: Any,
    t_rel: float,
) -> ImpedanceLayers:
    """Session ImpedanceAssist × Jump/Spot transient overlays."""
    session = float(session_leg_kp)
    spot = bool(
        active_gait is not None
        and getattr(active_gait, "spot_turn_active", False)
    )
    return ImpedanceLayers(
        session_leg_kp=session,
        effective_leg_kp=jump_phase_leg_kp(session, active_gait, t_rel),
        spot_abd_boost_active=spot,
    )


__all__ = [
    "ImpedanceLayers",
    "apply_spot_abd_kp_boost",
    "jump_phase_leg_kp",
    "resolve_impedance_layers",
]
