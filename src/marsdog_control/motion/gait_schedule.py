"""Velocity command → gait schedule (sim-first locomotion API).

Replaces ad-hoc amp-only throttle with a single map:
  VelocityCommand(vx, yaw_rate) → amps, period, stance, step_h, turn, vel_cmd(SI)

Spot turn (vx≈0, yaw≠0): Unitree-style continuous diagonal trot with
body-frame yaw scrub + explicit hip abduction (``SpotYawStepper``),
open-loop accumulating yaw, plus real ``vel_cmd.wz``. Cruise trot unchanged.

Callers (FSM) apply ``GaitScheduleOutput`` onto the live gait controller.
WBC reads ``gait.vel_cmd`` instead of reverse-engineering amp/period.

Stick map: linear from ``vx_deadzone`` → 1.0 into ``speed_frac``
(``throttle_min_scale`` … 1). ``vx_engage`` is FSM-only (do not reuse here).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional, Tuple

# Open-dog trot: exact diagonal pairs (no soft stagger → no 3-leg overlap).
SPOT_DIAGONAL_PHASE: Dict[str, float] = {
    "fl": 0.00,
    "rr": 0.00,
    "fr": 0.50,
    "rl": 0.50,
}

# Spot STATIC turn: one leg swings at a time (stance_ratio≈0.75 → 25% swing each),
# 3 feet always planted. The 3 planted feet are world-held and reprojected at the
# accumulating yaw_des, so they cooperatively rotate the base (no diagonal ±wz
# couple, real ground purchase). Swing leg re-centres under its hip in the new
# heading. Sequence rr → fr → rl → fl keeps the CoM inside the support triangle.
SPOT_STATIC_PHASE: Dict[str, float] = {
    "rr": 0.00,
    "fr": 0.25,
    "rl": 0.50,
    "fl": 0.75,
}


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
    # Cruise turn geometry (forward gait)
    cruise_turn_y_amp_m: float = 0.025
    cruise_turn_amp_diff_m: float = 0.020
    vx_deadzone: float = 0.12
    vx_engage: float = 0.30  # matches gp_trot_threshold
    # Spot: Raibert plant-hold turn (swing → ω×r, stance HOLD).
    spot_period: float = 0.85
    spot_stance: float = 0.55
    spot_step_h_front: float = 0.045
    spot_step_h_rear: float = 0.045
    spot_yaw_step_rad: float = 0.45
    spot_dx_scale: float = 0.0
    spot_turn_amp_diff_m: float = 0.0
    spot_turn_scale: float = 1.0
    spot_turn_y_gain: float = 0.0
    spot_wz_scale: float = 0.40
    spot_y_hold_max_m: float = 0.055
    spot_com_shift_m: float = 0.0

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
        turn_y_amp: Optional[float] = None,
        turn_amp_diff: Optional[float] = None,
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
        y_amp = float(turn_y_amp) if turn_y_amp is not None else 0.025
        amp_diff = float(turn_amp_diff) if turn_amp_diff is not None else 0.020
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
            # Bounds scale with CLI/preset period (no hard 0.85s cap).
            period_min=max(0.40, float(period) * 0.94),
            period_max=float(period) * 1.12,
            stance_nom=float(stance),
            stance_min=max(0.50, float(stance) - 0.03),
            stance_max=min(0.62, float(stance) + 0.03),
            throttle_min_scale=float(throttle_min_scale),
            cruise_turn_scale=float(cruise_turn_scale),
            cruise_turn_yamp=float(cruise_turn_yamp),
            cruise_turn_y_amp_m=y_amp,
            cruise_turn_amp_diff_m=amp_diff,
            vx_engage=float(vx_engage),
            vx_deadzone=float(vx_deadzone),
            # Spot: Raibert plant-hold (swing → ω×r, stance HOLD).
            spot_period=0.85,
            spot_stance=0.55,
            spot_step_h_front=0.045,
            spot_step_h_rear=0.045,
            spot_yaw_step_rad=0.45,
            spot_dx_scale=0.0,
            spot_turn_amp_diff_m=0.0,
            spot_turn_scale=1.0,
            spot_turn_y_gain=0.0,
            spot_wz_scale=0.40,
            spot_y_hold_max_m=0.055,
            spot_com_shift_m=0.0,
        )

    @classmethod
    def from_walk(
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
        throttle_min_scale: float = 0.55,
        vx_engage: float = 0.3,
        vx_deadzone: float = 0.12,
    ) -> "GaitEnvelope":
        """Four-beat Walk envelope — high duty, slow cadence; no spot fields used."""
        sh_f = float(step_h_front) if step_h_front is not None else 0.034
        sh_r = float(step_h_rear) if step_h_rear is not None else max(sh_f, 0.038)
        fl_f = (
            float(step_h_front_floor)
            if step_h_front_floor is not None
            else min(0.022, max(0.014, 0.55 * sh_f))
        )
        fl_r = (
            float(step_h_rear_floor)
            if step_h_rear_floor is not None
            else min(0.028, max(0.018, 0.60 * sh_r))
        )
        return cls(
            amp_front_max=float(amp_front),
            amp_rear_max=float(amp_rear),
            step_h_front_max=sh_f,
            step_h_rear_max=sh_r,
            step_h_front_floor=fl_f,
            step_h_rear_floor=fl_r,
            period_nom=float(period),
            period_min=max(0.88, float(period) * 0.94),
            period_max=min(1.25, float(period) * 1.10),
            stance_nom=float(stance),
            stance_min=max(0.68, float(stance) - 0.03),
            stance_max=min(0.78, float(stance) + 0.03),
            throttle_min_scale=float(throttle_min_scale),
            cruise_turn_scale=0.0,  # v1: forward-only
            cruise_turn_yamp=0.0,
            cruise_turn_y_amp_m=0.0,
            cruise_turn_amp_diff_m=0.0,
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
    speed_frac: float
    # Absolute turn geometry written onto gait.max_turn_*
    turn_y_amp: float = 0.025
    turn_amp_diff: float = 0.020
    spot_turn: bool = False
    spot_y_hold_max: float = 0.055
    spot_yaw_step: float = 0.45
    spot_dx_scale: float = 0.0
    spot_com_shift: float = 0.0


class SoftTrotSchedule:
    def __init__(self, envelope: Optional[GaitEnvelope] = None):
        self.env = envelope or GaitEnvelope()

    def map(self, cmd: VelocityCommand) -> GaitScheduleOutput:
        e = self.env
        vx = float(cmd.vx)
        yaw = float(cmd.yaw_rate)
        mag = min(1.0, abs(vx))
        yaw_mag = min(1.0, abs(yaw))

        # Abduction-led in-place spin when stick has yaw but no forward walk.
        if mag < e.vx_deadzone and yaw_mag >= e.vx_deadzone:
            return self._map_spot_turn(yaw)

        # Stick authority is linear from deadzone → 1.0.
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
        if speed_frac > 1e-9:
            step_f = max(e.step_h_front_floor, e.step_h_front_max * speed_frac)
            step_r = max(e.step_h_rear_floor, e.step_h_rear_max * speed_frac)
        else:
            step_f = 0.0
            step_r = 0.0

        period = e.period_max + (e.period_min - e.period_max) * u
        stance = e.stance_max + (e.stance_min - e.stance_max) * u
        if speed_frac <= 1e-9:
            period = e.period_nom
            stance = e.stance_nom

        from marsdog_control.control.velocity_model import VX_SCRUB_OFFSET_MPS

        avg_amp = 0.5 * (abs(amp_f) + abs(amp_r))
        if speed_frac > 0:
            vx_kin = sign * (2.0 * avg_amp / max(1e-3, period))
            vx_si = vx_kin + sign * float(VX_SCRUB_OFFSET_MPS)
        else:
            vx_si = 0.0
        turn_cmd = yaw * e.cruise_turn_scale
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
            turn_y_amp=float(e.cruise_turn_y_amp_m),
            turn_amp_diff=float(e.cruise_turn_amp_diff_m),
            spot_turn=False,
            spot_y_hold_max=float(e.spot_y_hold_max_m),
            spot_yaw_step=float(e.spot_yaw_step_rad),
            spot_dx_scale=float(e.spot_dx_scale),
            spot_com_shift=float(e.spot_com_shift_m),
        )

    def _map_spot_turn(self, yaw: float) -> GaitScheduleOutput:
        """In-place turn: diagonal trot, Raibert ω×r plant-hold."""
        e = self.env
        yaw_mag = min(1.0, abs(yaw))
        span = max(1e-6, 1.0 - e.vx_deadzone)
        u = max(0.0, min(1.0, (yaw_mag - e.vx_deadzone) / span))
        turn_frac = 0.55 + 0.45 * u
        sign = 1.0 if yaw >= 0.0 else -1.0
        turn_cmd = sign * turn_frac * e.spot_turn_scale
        wz_si = sign * turn_frac * e.spot_wz_scale
        return GaitScheduleOutput(
            amp_front=0.0,
            amp_rear=0.0,
            step_height_front=float(e.spot_step_h_front),
            step_height=float(e.spot_step_h_rear),
            period=float(e.spot_period),
            stance_ratio=float(e.spot_stance),
            turn_cmd=float(turn_cmd),
            turn_y_gain=0.0,
            vel_cmd=(0.0, 0.0, float(wz_si)),
            speed_frac=0.0,
            turn_y_amp=0.0,
            turn_amp_diff=0.0,
            spot_turn=True,
            spot_y_hold_max=float(e.spot_y_hold_max_m),
            spot_yaw_step=float(e.spot_yaw_step_rad),
            spot_dx_scale=0.0,
            spot_com_shift=float(e.spot_com_shift_m),
        )


@dataclass(frozen=True)
class JumpScheduleOutput:
    """Jump stick map — trigger / rejump only; no Soft amp or spot fields."""

    trigger: bool = False
    auto_rejump: bool = False
    vel_cmd: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    speed_frac: float = 0.0


class JumpSchedule:
    """In-place hop intent — stick hold → trigger+rejump; release → idle.

    Never enables spot turn; yaw stick ignored in v1.
    """

    def __init__(self, *, vx_deadzone: float = 0.12):
        self.vx_deadzone = float(vx_deadzone)

    def map(self, cmd: VelocityCommand) -> JumpScheduleOutput:
        intent = abs(float(cmd.vx)) >= self.vx_deadzone
        return JumpScheduleOutput(
            trigger=intent,
            auto_rejump=intent,
            vel_cmd=(0.0, 0.0, 0.0),
            speed_frac=1.0 if intent else 0.0,
        )


def apply_jump_schedule(gait, sched: JumpScheduleOutput) -> None:
    """Write Jump schedule onto JumpController only."""
    if getattr(gait, "family", None) != "jump":
        return
    gait.spot_turn_active = False
    gait.vel_cmd = sched.vel_cmd
    gait.speed_frac = float(sched.speed_frac)
    gait.auto_rejump = bool(sched.auto_rejump)
    if sched.trigger:
        gait.request_jump(True)
    else:
        gait.trigger = False
        gait.auto_rejump = False


class WalkSchedule:
    """Four-beat Walk velocity map — forward only; never enables spot turn.

    Yaw stick is ignored in v1 (turn via SoftTrot / Spot path instead).
    """

    def __init__(self, envelope: Optional[GaitEnvelope] = None):
        self.env = envelope or GaitEnvelope.from_walk(
            amp_front=0.040, amp_rear=0.048, period=1.05, stance=0.74,
        )

    def map(self, cmd: VelocityCommand) -> GaitScheduleOutput:
        e = self.env
        vx = float(cmd.vx)
        mag = min(1.0, abs(vx))

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
        if speed_frac > 1e-9:
            step_f = max(e.step_h_front_floor, e.step_h_front_max * speed_frac)
            step_r = max(e.step_h_rear_floor, e.step_h_rear_max * speed_frac)
        else:
            step_f = 0.0
            step_r = 0.0

        period = e.period_max + (e.period_min - e.period_max) * u
        stance = e.stance_max + (e.stance_min - e.stance_max) * u
        if speed_frac <= 1e-9:
            period = e.period_nom
            stance = e.stance_nom

        from marsdog_control.control.velocity_model import VX_SCRUB_OFFSET_MPS

        avg_amp = 0.5 * (abs(amp_f) + abs(amp_r))
        if speed_frac > 0:
            vx_kin = sign * (2.0 * avg_amp / max(1e-3, period))
            vx_si = vx_kin + sign * float(VX_SCRUB_OFFSET_MPS)
        else:
            vx_si = 0.0

        return GaitScheduleOutput(
            amp_front=amp_f,
            amp_rear=amp_r,
            step_height_front=float(step_f),
            step_height=float(step_r),
            period=float(period),
            stance_ratio=float(stance),
            turn_cmd=0.0,
            turn_y_gain=0.0,
            vel_cmd=(float(vx_si), float(cmd.vy), 0.0),
            speed_frac=float(speed_frac),
            turn_y_amp=0.0,
            turn_amp_diff=0.0,
            spot_turn=False,
            spot_y_hold_max=float(e.spot_y_hold_max_m),
            spot_yaw_step=float(e.spot_yaw_step_rad),
            spot_dx_scale=0.0,
            spot_com_shift=0.0,
        )


def apply_schedule_to_gait(gait, sched: GaitScheduleOutput) -> None:
    """Write schedule fields onto a live gait controller instance."""
    family = getattr(gait, "family", None)
    # Jump uses JumpSchedule / apply_jump_schedule — never Soft amp/period/spot.
    if family == "jump":
        return
    is_walk = family == "walk"
    # Walk stack never accepts spot; force schedule fields safe before write.
    if is_walk and sched.spot_turn:
        sched = GaitScheduleOutput(
            amp_front=sched.amp_front,
            amp_rear=sched.amp_rear,
            step_height_front=sched.step_height_front,
            step_height=sched.step_height,
            period=sched.period,
            stance_ratio=sched.stance_ratio,
            turn_cmd=0.0,
            turn_y_gain=0.0,
            vel_cmd=(sched.vel_cmd[0], sched.vel_cmd[1], 0.0),
            speed_frac=sched.speed_frac,
            turn_y_amp=0.0,
            turn_amp_diff=0.0,
            spot_turn=False,
            spot_y_hold_max=sched.spot_y_hold_max,
            spot_yaw_step=sched.spot_yaw_step,
            spot_dx_scale=0.0,
            spot_com_shift=0.0,
        )

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
    if hasattr(gait, "max_turn_y_amp"):
        gait.max_turn_y_amp = float(sched.turn_y_amp)
    if hasattr(gait, "max_turn_amp_diff"):
        gait.max_turn_amp_diff = float(sched.turn_amp_diff)

    was_spot = bool(getattr(gait, "spot_turn_active", False))
    if hasattr(gait, "spot_turn_active"):
        if is_walk:
            gait.spot_turn_active = False
            if was_spot and hasattr(gait, "_clear_spot_state"):
                gait._clear_spot_state()
        else:
            gait.spot_turn_active = bool(sched.spot_turn)
            if was_spot and not sched.spot_turn and hasattr(gait, "_clear_spot_state"):
                gait._clear_spot_state()

    # Spot: Unitree continuous diagonal trot-turn (never rewrite Walk phases).
    if (
        not is_walk
        and hasattr(gait, "_PHASE_OFFSET")
        and hasattr(gait, "_PHASE_OFFSET_CRUISE")
    ):
        if sched.spot_turn:
            gait._PHASE_OFFSET = dict(SPOT_DIAGONAL_PHASE)
        elif was_spot:
            gait._PHASE_OFFSET = dict(gait._PHASE_OFFSET_CRUISE)

    if hasattr(gait, "turn_filter_alpha"):
        gait.turn_filter_alpha = (
            0.015 if is_walk else (0.12 if sched.spot_turn else 0.015)
        )
    if is_walk:
        return

    if hasattr(gait, "spot_y_hold_max_m"):
        gait.spot_y_hold_max_m = float(sched.spot_y_hold_max)
    if hasattr(gait, "spot_yaw_step_rad"):
        gait.spot_yaw_step_rad = float(sched.spot_yaw_step)
    if hasattr(gait, "spot_dx_scale"):
        gait.spot_dx_scale = float(sched.spot_dx_scale)
    spot = getattr(gait, "_spot", None)
    if spot is not None and hasattr(spot, "cfg"):
        spot.cfg.y_hold_max_m = float(sched.spot_y_hold_max)
        spot.cfg.com_shift_max_m = float(getattr(sched, "spot_com_shift", 0.0))
        spot.cfg.yaw_step_rad = float(sched.spot_yaw_step) or spot.cfg.yaw_step_rad
        spot.cfg.stance_ratio = float(sched.stance_ratio)
        spot.cfg.x_hold_max_m = max(0.045, 0.85 * float(sched.spot_y_hold_max))


__all__ = [
    "VelocityCommand",
    "GaitEnvelope",
    "GaitScheduleOutput",
    "SoftTrotSchedule",
    "WalkSchedule",
    "JumpSchedule",
    "JumpScheduleOutput",
    "apply_schedule_to_gait",
    "apply_jump_schedule",
    "SPOT_DIAGONAL_PHASE",
    "SPOT_STATIC_PHASE",
]
