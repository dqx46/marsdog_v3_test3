"""Runtime lateral CoM ownership gate.

Startup picks ``LateralOwner`` once (Soft COM_SHIFT / SWAY / …). This module
enforces it on the hot path: non-owner kinematic / force-y / spot-CoM
contributions must be zero.

Walk elevates to ``WALK_COM`` (one intentional owner for abd Y + MPC y pair).
Pace elevates to ``SWAY`` (kinematic only — no force_y).
Spot temporarily elevates to ``SPOT``.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

from marsdog_control.config.control_policies import LateralOwner
from marsdog_control.motion import foot_trajectory as _ft


@dataclass(frozen=True)
class LateralContributions:
    """Raw lateral CoM-Y candidates before ownership gating."""

    kinematic_y: float = 0.0
    force_y: float = 0.0
    spot_xy: Tuple[float, float] = (0.0, 0.0)


@dataclass
class LateralPlanner:
    """One owner for lateral CoM; Spot / Walk / Pace elevate per family."""

    session_owner: LateralOwner
    _spot_active: bool = False
    _family_override: Optional[LateralOwner] = None

    @property
    def effective_owner(self) -> LateralOwner:
        if self._spot_active:
            return LateralOwner.SPOT
        if self._family_override is not None:
            return self._family_override
        return self.session_owner

    def set_spot_active(self, active: bool) -> None:
        self._spot_active = bool(active)

    def sync_from_gait(self, gait) -> LateralOwner:
        """Update Spot / family overrides from the active gait; return owner."""
        if gait is None:
            self._spot_active = False
            self._family_override = None
            return self.effective_owner

        spot = bool(getattr(gait, "spot_turn_active", False))
        self._spot_active = spot
        if spot:
            self._family_override = None
            return self.effective_owner

        family = getattr(gait, "family", None)
        if family == "walk":
            # Single owner for Walk's intentional abd-Y + MPC-y pair.
            self._family_override = LateralOwner.WALK_COM
        elif family == "pace":
            # Pace sway is kinematic-only (no Dynamics/MPC force_y).
            self._family_override = LateralOwner.SWAY
        else:
            self._family_override = None
        return self.effective_owner

    def allows_kinematic(self) -> bool:
        """Any Soft/trot kinematic CoM path (com_shift / sway / walk pair)."""
        return self.effective_owner in (
            LateralOwner.COM_SHIFT,
            LateralOwner.SWAY,
            LateralOwner.WALK_COM,
        )

    def allows_sway_kinematic(self) -> bool:
        """Half-sine / pace / walk abd sway (not Soft event com_shift)."""
        return self.effective_owner in (
            LateralOwner.SWAY, LateralOwner.WALK_COM)

    def allows_force_y(self) -> bool:
        """MPC / Dynamics com_y — FORCE_Y session or Walk's WALK_COM pair."""
        return self.effective_owner in (
            LateralOwner.FORCE_Y, LateralOwner.WALK_COM)

    def allows_spot_com(self) -> bool:
        return self.effective_owner is LateralOwner.SPOT

    def gate(self, raw: LateralContributions) -> LateralContributions:
        o = self.effective_owner
        if o in (LateralOwner.COM_SHIFT, LateralOwner.SWAY, LateralOwner.WALK_COM):
            k = raw.kinematic_y
        else:
            k = 0.0
        f = raw.force_y if self.allows_force_y() else 0.0
        sx, sy = raw.spot_xy if self.allows_spot_com() else (0.0, 0.0)
        return LateralContributions(k, f, (sx, sy))

    def gate_kinematic(self, value: float) -> float:
        """Gate sway-class kinematic (trot sway / pace / walk abd)."""
        return float(value) if self.allows_sway_kinematic() else 0.0

    def gate_force_y(self, value: float) -> float:
        return float(value) if self.allows_force_y() else 0.0

    def gate_spot_com(self, xy: Tuple[float, float]) -> Tuple[float, float]:
        if self.allows_spot_com():
            return (float(xy[0]), float(xy[1]))
        return (0.0, 0.0)

    def soft_kinematic(
        self,
        t: float,
        period: float,
        stance_ratio: float,
        *,
        com_shift_m: float,
        com_shift_blend: float,
        lateral_sway: float,
    ) -> float:
        """Soft kinematic CoM-Y under the effective owner (no dual fallback)."""
        o = self.effective_owner
        if o is LateralOwner.COM_SHIFT:
            if abs(float(com_shift_m)) <= 1e-9:
                return 0.0
            return float(_ft.lateral_offset_soft_trot_com(
                t, period, com_shift_m, com_shift_blend))
        if o is LateralOwner.SWAY:
            return float(_ft.lateral_offset_trot(
                t, period, stance_ratio, lateral_sway))
        return 0.0

    def trot_sway_kinematic(
        self,
        t: float,
        period: float,
        stance_ratio: float,
        lateral_sway: float,
    ) -> float:
        """StableTrot / Pace sway path — only when owner is SWAY."""
        if self.effective_owner is LateralOwner.SWAY:
            return float(_ft.lateral_offset_trot(
                t, period, stance_ratio, lateral_sway))
        return 0.0

    def attach_to(self, *gaits) -> None:
        """Share this planner via ``bind_ownership`` when available."""
        for gait in gaits:
            if gait is None:
                continue
            if hasattr(gait, "bind_ownership"):
                gate = getattr(gait, "_attitude_overlay_gate", None)
                gait.bind_ownership(lateral_planner=self, attitude_gate=gate)
            else:
                gait._lateral_planner = self


__all__ = [
    "LateralContributions",
    "LateralPlanner",
]
