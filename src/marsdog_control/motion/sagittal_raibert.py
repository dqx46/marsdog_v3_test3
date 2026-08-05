"""Sagittal Raibert foot placement — reads SI ``vel_cmd``, not schedule amp.

Schedule owns amp for open-loop foot X. Placement owns dx from speed error
against ``gait.vel_cmd[0]``. When schedule zeros amp but vel_cmd remains
(idle race / test harness), placement may revive recipe amp for X amplitude.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping, Optional, Tuple


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


@dataclass
class SagittalRaibert:
    """Per-gait sagittal placement state (attached on StableTrot / Soft)."""

    enabled: bool = True
    kx: float = 0.05
    dx_max: float = 0.03
    recipe_amp_front: float = 0.022
    recipe_amp_rear: float = 0.030
    dx: float = 0.0
    use_amp: bool = False
    amp_front: float = 0.0
    amp_rear: float = 0.0

    def reset(self) -> None:
        self.dx = 0.0
        self.use_amp = False
        self.amp_front = 0.0
        self.amp_rear = 0.0

    def update(
        self,
        *,
        vel_cmd: Tuple[float, float, float],
        vel_xyz: Optional[Tuple[float, float, float]],
        schedule_amp_front: float,
        schedule_amp_rear: float,
        spot_active: bool = False,
    ) -> float:
        """Update dx / amp; return dx to add on foot X (body +X)."""
        if not self.enabled or spot_active or vel_xyz is None:
            self.reset()
            return 0.0

        vx_cmd = float(vel_cmd[0]) if vel_cmd is not None else 0.0
        v_actual = float(vel_xyz[0])
        sched_f = float(schedule_amp_front)
        sched_r = float(schedule_amp_rear)

        if abs(vx_cmd) <= 0.02:
            self.reset()
            return 0.0

        # Placement active: amp reference prefers schedule, else Soft recipe.
        self.use_amp = True
        if abs(sched_f) + abs(sched_r) < 1e-6:
            self.amp_front = float(self.recipe_amp_front)
            self.amp_rear = float(self.recipe_amp_rear)
        else:
            self.amp_front = abs(sched_f)
            self.amp_rear = abs(sched_r)

        self.dx = _clamp(self.kx * (v_actual - vx_cmd), -self.dx_max, self.dx_max)
        return float(self.dx)


def attach_raibert(
    gait: Any,
    *,
    enabled: bool = True,
    kx: float = 0.05,
    dx_max: float = 0.03,
    recipe_amp_front: Optional[float] = None,
    recipe_amp_rear: Optional[float] = None,
) -> SagittalRaibert:
    """Install / replace ``gait._raibert`` from init kwargs."""
    rf = float(
        recipe_amp_front
        if recipe_amp_front is not None
        else getattr(gait, "amp_front", 0.022)
    )
    rr = float(
        recipe_amp_rear
        if recipe_amp_rear is not None
        else getattr(gait, "amp_rear", 0.030)
    )
    state = SagittalRaibert(
        enabled=bool(enabled),
        kx=float(kx),
        dx_max=float(dx_max),
        recipe_amp_front=abs(rf),
        recipe_amp_rear=abs(rr),
    )
    gait._raibert = state
    return state


def update_raibert_from_imu(gait: Any, imu_state: Optional[Mapping]) -> float:
    """Convenience for ``get_targets`` / Soft tests."""
    rb = getattr(gait, "_raibert", None)
    if rb is None:
        return 0.0
    vel_xyz = None
    if imu_state is not None and "vel_xyz" in imu_state:
        vel_xyz = imu_state["vel_xyz"]
    vel_cmd = getattr(gait, "vel_cmd", (0.0, 0.0, 0.0))
    return rb.update(
        vel_cmd=tuple(vel_cmd) if vel_cmd is not None else (0.0, 0.0, 0.0),
        vel_xyz=tuple(vel_xyz) if vel_xyz is not None else None,
        schedule_amp_front=float(getattr(gait, "amp_front", 0.0)),
        schedule_amp_rear=float(getattr(gait, "amp_rear", 0.0)),
        spot_active=bool(getattr(gait, "spot_turn_active", False)),
    )


__all__ = [
    "SagittalRaibert",
    "attach_raibert",
    "update_raibert_from_imu",
]
