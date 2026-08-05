"""Unitree-style in-place turn: world-hold scrub via ω×r_hip.

Stance (per diagonal): body-frame foothold rotates opposite intended yaw
(``rotate(hip, −drift) − hip`` ≡ linearised ω×r). Front (hx>0) and rear
(hx<0) get opposite body-Y → yaw shear, not same-side abduct.

Swing: cosine return from stance-end scrub toward under-hip.

This scrub path is what actually produces net yaw in our stack; a pure
fixed plant-hold tended to either ±yaw-thrash or tip in roll.

Phases are diagonal trot (FL+RR / FR+RL) — never pace.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable, Dict, Optional, Set, Tuple

LEGS = ("fl", "fr", "rl", "rr")

STATIC_PHASE: Dict[str, float] = {
    "fl": 0.00,
    "rr": 0.00,
    "fr": 0.50,
    "rl": 0.50,
}

CATCH0_LEGS = ("fl", "rr")
CATCH1_LEGS = ("fr", "rl")
PHASE_OFFSET: Dict[str, float] = dict(STATIC_PHASE)

HipFn = Callable[[str], Tuple[float, float]]


@dataclass
class SpotYawStepConfig:
    """``yaw_step_rad`` = yaw accumulated per full cycle at |turn|=1."""

    yaw_step_rad: float = 0.45
    stance_ratio: float = 0.55
    y_hold_max_m: float = 0.055
    x_hold_max_m: float = 0.045
    scrub_x_scale: float = 1.0
    turn_deadzone: float = 0.02
    com_shift_max_m: float = 0.0
    com_shift_gain: float = 0.0
    plant_frac: float = 0.0
    twist_frac: float = 0.0
    catch0_frac: float = 0.0
    catch1_frac: float = 0.0
    catch_prep: float = 0.0
    twist_lead_frac: float = 0.0
    twist_lead_max: float = 0.0
    yaw_lead_max: float = 0.5


@dataclass
class SpotYawStepper:
    cfg: SpotYawStepConfig = field(default_factory=SpotYawStepConfig)
    hip_xy: HipFn = field(default=lambda leg: (0.0, 0.0))

    _xy_cache: Dict[str, Tuple[float, float]] = field(default_factory=dict)
    _swing_u: Dict[str, float] = field(default_factory=dict)
    _in_swing: Dict[str, bool] = field(default_factory=dict)

    yaw: float = 0.0
    yaw_des: float = 0.0
    base_xy: Tuple[float, float] = (0.0, 0.0)
    pose_inited: bool = False
    _prev_t: Optional[float] = None
    _tick_t: Optional[float] = None

    _active: bool = False
    _turn: float = 0.0
    _period: float = 0.85
    _stance: float = 0.55
    phase: str = "idle"

    def __post_init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        for lg in LEGS:
            self._xy_cache[lg] = (0.0, 0.0)
            self._swing_u[lg] = 0.0
            self._in_swing[lg] = False
        self.yaw = 0.0
        self.yaw_des = 0.0
        self.base_xy = (0.0, 0.0)
        self.pose_inited = False
        self._prev_t = None
        self._tick_t = None
        self._active = False
        self._turn = 0.0
        self.phase = "idle"

    def update_pose(
        self,
        t: float,
        *,
        yaw: float,
        base_xy: Optional[Tuple[float, float]] = None,
        vel_xy: Optional[Tuple[float, float]] = None,
        wz: float = 0.0,
    ) -> None:
        was = self.pose_inited
        if base_xy is not None:
            self.base_xy = (float(base_xy[0]), float(base_xy[1]))
            self.pose_inited = True
        elif vel_xy is not None and self._prev_t is not None:
            dt = max(0.0, min(0.05, float(t) - float(self._prev_t)))
            bx, by = self.base_xy
            self.base_xy = (bx + float(vel_xy[0]) * dt, by + float(vel_xy[1]) * dt)
            self.pose_inited = True
        yaw_m = float(yaw)
        if not was:
            self.yaw_des = yaw_m
        self.yaw = yaw_m
        self._prev_t = float(t)
        _ = wz

    def tick(
        self,
        t: float,
        turn: float,
        period: float,
        stance_ratio: Optional[float] = None,
    ) -> None:
        if abs(turn) < float(self.cfg.turn_deadzone):
            if self._active:
                self.reset()
            return

        period = max(1e-3, float(period))
        stance = float(
            self.cfg.stance_ratio if stance_ratio is None else stance_ratio
        )
        stance = max(0.35, min(0.85, stance))
        turn = float(turn)

        if not self._active:
            self.yaw_des = float(self.yaw)
            self._tick_t = float(t)

        self._active = True
        self._turn = turn
        self._period = period
        self._stance = stance
        self.phase = "unitree"

        yaw_per_cycle = float(self.cfg.yaw_step_rad) * turn
        if self._tick_t is not None:
            dt = max(0.0, float(t) - float(self._tick_t))
            if dt > period * 1.5:
                dt = 0.0
            else:
                dt = min(dt, period)
            delta = (dt / period) * yaw_per_cycle
            # Anti-windup: freeze when already leading measured yaw by
            # yaw_lead_max in the integration direction. Do NOT project
            # yaw_des onto [yaw±lead] — that recentres the setpoint whenever
            # measured yaw drifts (including opposite the turn command).
            lead = max(0.0, float(self.cfg.yaw_lead_max))
            err = float(self.yaw_des) - float(self.yaw)
            if not (lead > 0.0 and abs(err) >= lead and err * delta > 0.0):
                self.yaw_des = float(self.yaw_des) + delta
        self._tick_t = float(t)

        for lg in LEGS:
            dx, dy, swinging, u = self._leg_xy(
                lg, float(t), period, stance, yaw_per_cycle
            )
            self._xy_cache[lg] = (dx, dy)
            self._in_swing[lg] = swinging
            self._swing_u[lg] = u if swinging else 0.0

    def _leg_xy(
        self,
        leg: str,
        t: float,
        period: float,
        stance: float,
        yaw_per_cycle: float,
    ) -> Tuple[float, float, bool, float]:
        leg_phase = (t / period + PHASE_OFFSET[leg]) % 1.0
        hx, hy = self.hip_xy(leg)
        sx = float(self.cfg.scrub_x_scale) * hx
        sy = hy
        # Only scrub during the planted fraction of the cycle.
        drift_yaw = yaw_per_cycle * stance
        xlim = float(self.cfg.x_hold_max_m)
        ylim = float(self.cfg.y_hold_max_m)

        if leg_phase < stance:
            stance_t = leg_phase / max(1e-6, stance)
            drift = -drift_yaw * stance_t
            dx, dy = self._rotate_offset(sx, sy, drift)
            return (self._clamp(dx, xlim), self._clamp(dy, ylim), False, 0.0)

        swing_t = (leg_phase - stance) / max(1e-6, 1.0 - stance)
        smooth = 0.5 * (1.0 - math.cos(math.pi * swing_t))
        dx0, dy0 = self._rotate_offset(sx, sy, -drift_yaw)
        dx = dx0 * (1.0 - smooth)
        dy = dy0 * (1.0 - smooth)
        return (self._clamp(dx, xlim), self._clamp(dy, ylim), True, float(swing_t))

    @staticmethod
    def _rotate_offset(hx: float, hy: float, angle: float) -> Tuple[float, float]:
        c, s = math.cos(angle), math.sin(angle)
        fx = c * hx - s * hy
        fy = s * hx + c * hy
        return fx - hx, fy - hy

    def in_swing(self, leg: str) -> bool:
        return bool(self._active and self._in_swing.get(leg, False))

    def swing_progress(self, leg: str) -> float:
        return float(self._swing_u.get(leg, 0.0)) if self.in_swing(leg) else 0.0

    def predict_force_scale(self, leg: str, t: float, period: float) -> float:
        if not self._active and abs(self._turn) < self.cfg.turn_deadzone:
            return 1.0
        period = max(1e-3, float(period))
        stance = float(self._stance if self._active else self.cfg.stance_ratio)
        leg_phase = (float(t) / period + PHASE_OFFSET.get(leg, 0.0)) % 1.0
        return 0.0 if leg_phase >= stance else 1.0

    def swinging_legs(self) -> Set[str]:
        return {lg for lg in LEGS if self.in_swing(lg)}

    def foot_xy(
        self,
        leg: str,
        t: float,
        turn: float,
        period: float,
        stance_ratio: float = 0.0,
        phase_offsets: Optional[Dict[str, float]] = None,
    ) -> Tuple[float, float]:
        _ = (t, period, stance_ratio, phase_offsets)
        if abs(turn) < float(self.cfg.turn_deadzone):
            self._xy_cache[leg] = (0.0, 0.0)
            return (0.0, 0.0)
        return self._xy_cache.get(leg, (0.0, 0.0))

    def cached_xy(self, leg: str) -> Tuple[float, float]:
        return self._xy_cache.get(leg, (0.0, 0.0))

    def com_shift_xy(
        self,
        t: float = 0.0,
        period: float = 1.0,
        stance_ratio: float = 0.75,
        phase_offsets: Optional[Dict[str, float]] = None,
    ) -> Tuple[float, float]:
        _ = (t, period, stance_ratio, phase_offsets)
        return (0.0, 0.0)

    def yaw_error(self) -> float:
        return float(self.yaw_des - self.yaw)

    @staticmethod
    def _clamp(v: float, lim: float) -> float:
        if lim <= 1e-9:
            return 0.0
        return float(max(-lim, min(lim, v)))

    _soft_clamp = _clamp


class StompPhase:
    PLANT = "plant"
    TWIST = "twist"
    CATCH0 = "catch0"
    CATCH1 = "catch1"


__all__ = [
    "LEGS",
    "CATCH0_LEGS",
    "CATCH1_LEGS",
    "STATIC_PHASE",
    "PHASE_OFFSET",
    "StompPhase",
    "SpotYawStepConfig",
    "SpotYawStepper",
]
