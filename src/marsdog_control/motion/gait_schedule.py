"""Velocity command → gait schedule (sim-first locomotion API).

Replaces ad-hoc amp-only throttle with a single map:
  VelocityCommand(vx, yaw_rate) → amps, period, stance, step_h, turn, vel_cmd(SI)

Callers (FSM) apply ``GaitScheduleOutput`` onto the live gait controller.
WBC reads ``gait.vel_cmd`` instead of reverse-engineering amp/period.

Stick map: linear from ``vx_deadzone`` → 1.0 into ``speed_frac``
(``throttle_min_scale`` … 1). ``vx_engage`` is FSM-only (do not reuse here).

Design note — foot arc aspect:
  Amp scales with throttle but a fixed step_h makes crawl look like a piston
  (Δz/Δx → 1). Schedule therefore scales step_height with the same speed_frac.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass(frozen=True)
class VelocityCommand:
    """Normalized drive intent (−1..1), same sense as UserCommand stick axes."""

    vx: float = 0.0  # +forward
    yaw_rate: float = 0.0  # + = turn_cmd sense used by StableTrot (right turn >0)
    vy: float = 0.0  # reserved (crab); unused by soft-trot schedule yet


@dataclass(frozen=True)
class GaitEnvelope:
    """Nominal / bound gait geometry for one family (e.g. NATURAL_SOFT_TROT_WBC)."""

    amp_front_max: float = 0.032
    amp_rear_max: float = 0.036
    step_h_front_max: float = 0.024
    step_h_rear_max: float = 0.034
    # Absolute clearance floor while moving (anti-shuffle / limp)
    step_h_front_floor: float = 0.014
    step_h_rear_floor: float = 0.022
    period_nom: float = 1.05
    period_min: float = 0.95  # full throttle → slightly faster cadence
    period_max: float = 1.20  # crawl → slower cadence
    stance_nom: float = 0.80
    stance_min: float = 0.76  # high speed: shorter double support
    stance_max: float = 0.84  # low speed: longer double support
    throttle_min_scale: float = 0.65
    cruise_turn_scale: float = 0.60
    cruise_turn_yamp: float = 1.0
    vx_deadzone: float = 0.12
    vx_engage: float = 0.30  # matches gp_trot_threshold

    @classmethod
    def from_wbc_soft_trot(
        cls,
        *,
        amp_front: float,
        amp_rear: float,
        period: float,
        stance: float,
        step_h_front: Optional[float] = None,
        step_h_rear: Optional[float] = None,
        step_h_front_floor: Optional[float] = None,
        step_h_rear_floor: Optional[float] = None,
        throttle_min_scale: float = 0.65,
        cruise_turn_scale: float = 0.6,
        cruise_turn_yamp: float = 1.0,
        vx_engage: float = 0.3,
        vx_deadzone: float = 0.12,
    ) -> "GaitEnvelope":
        sh_f = float(step_h_front) if step_h_front is not None else 0.024
        sh_r = float(step_h_rear) if step_h_rear is not None else max(sh_f, 0.034)
        # Floors track envelope so mid-stick (speed_frac<1) still clears ground.
        fl_f = (
            float(step_h_front_floor)
            if step_h_front_floor is not None
            else min(0.018, max(0.012, 0.55 * sh_f))
        )
        fl_r = (
            float(step_h_rear_floor)
            if step_h_rear_floor is not None
            else min(0.030, max(0.020, 0.60 * sh_r))
        )
        return cls(
            amp_front_max=float(amp_front),
            amp_rear_max=float(amp_rear),
            step_h_front_max=sh_f,
            step_h_rear_max=sh_r,
            step_h_front_floor=fl_f,
            step_h_rear_floor=fl_r,
            period_nom=float(period),
            # Dog-like trot cadence; keep min/max close so full-stick doesn't
            # suddenly halve duty and blow roll (seen: st=0.50 @ vx=1 → tip).
            period_min=max(0.48, float(period) * 0.94),
            period_max=min(0.85, float(period) * 1.12),
            stance_nom=float(stance),
            stance_min=max(0.50, float(stance) - 0.03),
            stance_max=min(0.62, float(stance) + 0.03),
            throttle_min_scale=float(throttle_min_scale),
            cruise_turn_scale=float(cruise_turn_scale),
            cruise_turn_yamp=float(cruise_turn_yamp),
            vx_engage=float(vx_engage),
            vx_deadzone=float(vx_deadzone),
        )


@dataclass(frozen=True)
class GaitScheduleOutput:
    amp_front: float
    amp_rear: float
    step_height_front: float
    step_height: float
    period: float
    stance_ratio: float
    turn_cmd: float
    turn_y_gain: float
    # SI body twist for WBC/MPC (world-aligned x forward)
    vel_cmd: Tuple[float, float, float]  # vx, vy, wz
    speed_frac: float  # 0..1 within envelope after engage


class SoftTrotSchedule:
    """Maps stick velocity → soft-trot schedule inside a fixed envelope."""

    def __init__(self, envelope: Optional[GaitEnvelope] = None):
        self.env = envelope or GaitEnvelope()

    def map(self, cmd: VelocityCommand) -> GaitScheduleOutput:
        e = self.env
        vx = float(cmd.vx)
        yaw = float(cmd.yaw_rate)
        mag = min(1.0, abs(vx))
        # Stick authority is linear from deadzone → 1.0.
        # (vx_engage is FSM entry only; using it here crushed mid-stick range:
        #  stick 0.55→1.0 used to span only ~30% of amp with throttle_min=0.7.)
        if mag < e.vx_deadzone:
            u = 0.0
            speed_frac = 0.0
        else:
            span = max(1e-6, 1.0 - e.vx_deadzone)
            u = max(0.0, min(1.0, (mag - e.vx_deadzone) / span))
            speed_frac = e.throttle_min_scale + (1.0 - e.throttle_min_scale) * u

        sign = 1.0 if vx >= 0.0 else -1.0
        amp_f = sign * e.amp_front_max * speed_frac
        amp_r = sign * e.amp_rear_max * speed_frac
        # Lift tracks stride for arc shape, but never below clearance floor
        # (scaled-only lift made crawl look like a limp shuffle).
        if speed_frac > 1e-9:
            step_f = max(e.step_h_front_floor, e.step_h_front_max * speed_frac)
            step_r = max(e.step_h_rear_floor, e.step_h_rear_max * speed_frac)
        else:
            step_f = 0.0
            step_r = 0.0

        # Cadence & duty vs speed: crawl = slow period + high stance
        period = e.period_max + (e.period_min - e.period_max) * u
        stance = e.stance_max + (e.stance_min - e.stance_max) * u
        if speed_frac <= 1e-9:
            period = e.period_nom
            stance = e.stance_nom

        # Body speed for MPC/WBC: kinematic no-slip (+ optional scrub prior).
        # Scrub is currently 0 — dog-trot plant undershoots; positive scrub
        # caused estimator/WBC to brake after the first strides.
        from marsdog_control.control.velocity_model import VX_SCRUB_OFFSET_MPS

        avg_amp = 0.5 * (abs(amp_f) + abs(amp_r))
        if speed_frac > 0:
            vx_kin = sign * (2.0 * avg_amp / max(1e-3, period))
            vx_si = vx_kin + sign * float(VX_SCRUB_OFFSET_MPS)
        else:
            vx_si = 0.0
        # Turn stick → yaw rate proxy (not SI; gait uses turn_cmd directly)
        turn_cmd = yaw * e.cruise_turn_scale
        # Rough wz for MPC yaw tracking (scale stick to ~0.4 rad/s at full)
        wz_si = turn_cmd * 0.4

        return GaitScheduleOutput(
            amp_front=amp_f,
            amp_rear=amp_r,
            step_height_front=float(step_f),
            step_height=float(step_r),
            period=float(period),
            stance_ratio=float(stance),
            turn_cmd=float(turn_cmd),
            turn_y_gain=float(e.cruise_turn_yamp),
            vel_cmd=(float(vx_si), float(cmd.vy), float(wz_si)),
            speed_frac=float(speed_frac),
        )


def apply_schedule_to_gait(gait, sched: GaitScheduleOutput) -> None:
    """Write schedule fields onto a live gait controller instance."""
    gait.amp_front = sched.amp_front
    gait.amp_rear = sched.amp_rear
    if hasattr(gait, "step_height_front"):
        gait.step_height_front = sched.step_height_front
    if hasattr(gait, "step_height"):
        gait.step_height = sched.step_height
    if hasattr(gait, "set_period"):
        gait.set_period(sched.period)
    else:
        gait.period = sched.period
    gait.stance_ratio = sched.stance_ratio
    gait.turn_cmd = sched.turn_cmd
    gait.turn_y_gain = sched.turn_y_gain
    gait.vel_cmd = sched.vel_cmd
    gait.speed_frac = sched.speed_frac


__all__ = [
    "VelocityCommand",
    "GaitEnvelope",
    "GaitScheduleOutput",
    "SoftTrotSchedule",
    "apply_schedule_to_gait",
]
