"""Body velocity (SI) → gait schedule (sim-first locomotion API).

Single map:
  VelocityCommand(vx[m/s], yaw_rate[rad/s]) → amps, period, stance, step_h, turn, vel_cmd

Stick (−1..1) mapping lives in ``input.teleop_policy`` only. This module never
interprets gamepad percentages.

Spot turn (vx≈0, yaw≠0): Unitree-style continuous diagonal trot with
body-frame yaw scrub + explicit hip abduction (``SpotYawStepper``),
open-loop accumulating yaw, plus real ``vel_cmd.wz``. Cruise trot unchanged.

Callers (FSM) apply ``GaitScheduleOutput`` onto the live gait controller.
WBC reads ``gait.vel_cmd`` instead of reverse-engineering amp/period.

``throttle_min_scale`` shapes the amp/period curve vs forward speed authority
(not a stick floor). ``vx_engage`` remains FSM/teleop-only.
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


# SI stop bands for schedule (not stick deadzones).
_VX_STOP_MPS = 0.010
_YAW_STOP_RPS = 0.050
# Geometry ↔ yaw: turn_cmd≈1 maps to roughly this wz (legacy cruise path).
_TURN_CMD_WZ_REF = 0.40


@dataclass(frozen=True)
class VelocityCommand:
    """Body-frame locomotion command in SI units.

    Produced by teleop policy, autonomy, or sim CLI — never raw stick %.
    """

    vx: float = 0.0  # m/s, +forward
    yaw_rate: float = 0.0  # rad/s, + = StableTrot right-turn sense
    vy: float = 0.0  # m/s, reserved (crab)


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

    @staticmethod
    def _stance_band(
        stance: float,
        *,
        floor: float,
        ceil: float,
        classic_lo: float,
        classic_hi: float,
        band: float = 0.03,
    ) -> dict:
        """Build stance_nom/min/max around an explicit duty.

        Classic dog-trot presets (≈0.50–0.62) keep the historical clamp.
        Explicit low/high duties (e.g. ``--stance 0.36``) keep a ±band
        around the requested value so schedule interpolation stays valid.
        """
        st = float(stance)
        lo = st - band
        hi = st + band
        if classic_lo <= st <= classic_hi:
            smin = max(classic_lo, lo)
            smax = min(classic_hi, hi)
        else:
            smin = max(floor, lo)
            smax = min(ceil, hi)
        smin = min(smin, st)
        smax = max(smax, st)
        return {"stance_nom": st, "stance_min": smin, "stance_max": smax}

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
            # Honor CLI/preset duty, including flight-heavy (<0.5). Old floor
            # max(0.50, …) made --stance 0.36 invert min/max and ignore CLI.
            **cls._stance_band(float(stance), floor=0.20, ceil=0.95,
                                classic_lo=0.50, classic_hi=0.62),
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

    def max_forward_vx(self) -> float:
        """Kinematic |vx| at full envelope authority (u=1)."""
        return abs(self._vx_kin_at_u(1.0))

    def vx_at_legacy_norm(self, norm: float) -> float:
        """Map old stick-normalized cruise (0..1) → kinematic vx (m/s)."""
        e = self.env
        mag = max(0.0, min(1.0, abs(float(norm))))
        if mag < e.vx_deadzone:
            return 0.0
        span = max(1e-6, 1.0 - e.vx_deadzone)
        u = max(0.0, min(1.0, (mag - e.vx_deadzone) / span))
        return abs(self._vx_kin_at_u(u))

    def _speed_frac(self, u: float) -> float:
        e = self.env
        u = max(0.0, min(1.0, float(u)))
        return e.throttle_min_scale + (1.0 - e.throttle_min_scale) * u

    def _vx_kin_at_u(self, u: float) -> float:
        from marsdog_control.control.velocity_model import VX_SCRUB_OFFSET_MPS

        e = self.env
        speed_frac = self._speed_frac(u)
        period = e.period_max + (e.period_min - e.period_max) * max(0.0, min(1.0, u))
        avg_amp = 0.5 * (e.amp_front_max + e.amp_rear_max) * speed_frac
        return 2.0 * avg_amp / max(1e-3, period) + float(VX_SCRUB_OFFSET_MPS)

    def _authority_for_vx(self, vx_mps: float) -> float:
        """Invert envelope curve: desired |vx| → authority u∈[0,1]."""
        target = abs(float(vx_mps))
        vmax = self.max_forward_vx()
        if target <= _VX_STOP_MPS or vmax <= _VX_STOP_MPS:
            return 0.0
        if target >= vmax:
            return 1.0
        lo, hi = 0.0, 1.0
        for _ in range(24):
            mid = 0.5 * (lo + hi)
            if self._vx_kin_at_u(mid) < target:
                lo = mid
            else:
                hi = mid
        return 0.5 * (lo + hi)

    def map(self, cmd: VelocityCommand) -> GaitScheduleOutput:
        e = self.env
        vx = float(cmd.vx)
        yaw = float(cmd.yaw_rate)
        vy = float(cmd.vy)

        if abs(vx) < _VX_STOP_MPS and abs(yaw) >= _YAW_STOP_RPS:
            return self._map_spot_turn(yaw)

        if abs(vx) < _VX_STOP_MPS:
            u = 0.0
            speed_frac = 0.0
        else:
            u = self._authority_for_vx(vx)
            speed_frac = self._speed_frac(u)

        sign = 1.0 if vx >= 0.0 else -1.0
        amp_f = sign * e.amp_front_max * speed_frac
        amp_r = sign * e.amp_rear_max * speed_frac
        if speed_frac > 1e-9:
            # Lift ∝ speed_frac; anti-limp floor only above throttle_min_scale.
            scuff_f = min(0.010, 0.45 * e.step_h_front_floor)
            scuff_r = min(0.012, 0.45 * e.step_h_rear_floor)
            tms = max(1e-6, e.throttle_min_scale)
            floor_blend = max(
                0.0, min(1.0, (speed_frac - tms) / max(1e-6, 1.0 - tms))
            )
            floor_f = scuff_f + (e.step_h_front_floor - scuff_f) * floor_blend
            floor_r = scuff_r + (e.step_h_rear_floor - scuff_r) * floor_blend
            step_f = max(floor_f, e.step_h_front_max * speed_frac)
            step_r = max(floor_r, e.step_h_rear_max * speed_frac)
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

        wz_si = float(yaw)
        turn_auth = max(-1.0, min(1.0, yaw / _TURN_CMD_WZ_REF))
        turn_cmd = turn_auth * e.cruise_turn_scale

        return GaitScheduleOutput(
            amp_front=amp_f,
            amp_rear=amp_r,
            step_height_front=float(step_f),
            step_height=float(step_r),
            period=float(period),
            stance_ratio=float(stance),
            turn_cmd=float(turn_cmd),
            turn_y_gain=float(e.cruise_turn_yamp),
            vel_cmd=(float(vx_si), float(vy), float(wz_si)),
            speed_frac=float(speed_frac),
            turn_y_amp=float(e.cruise_turn_y_amp_m),
            turn_amp_diff=float(e.cruise_turn_amp_diff_m),
            spot_turn=False,
            spot_y_hold_max=float(e.spot_y_hold_max_m),
            spot_yaw_step=float(e.spot_yaw_step_rad),
            spot_dx_scale=float(e.spot_dx_scale),
            spot_com_shift=float(e.spot_com_shift_m),
        )

    def _map_spot_turn(self, yaw_rate: float) -> GaitScheduleOutput:
        """In-place turn: diagonal trot, Raibert ω×r plant-hold."""
        e = self.env
        yaw_mag = abs(float(yaw_rate))
        full = max(1e-6, float(e.spot_wz_scale))
        u = max(0.0, min(1.0, yaw_mag / full))
        turn_frac = 0.55 + 0.45 * u
        sign = 1.0 if yaw_rate >= 0.0 else -1.0
        turn_cmd = sign * turn_frac * e.spot_turn_scale
        wz_si = sign * min(yaw_mag, full)
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
    """In-place hop intent — nonzero forward cmd → trigger+rejump; else idle.

    Never enables spot turn; yaw ignored in v1. ``vx`` is SI m/s.
    """

    def __init__(self, *, vx_engage_mps: float = 0.02, vx_deadzone: float | None = None):
        # vx_deadzone kept as deprecated alias (old stick units misused as m/s gate).
        if vx_deadzone is not None and vx_engage_mps == 0.02:
            self.vx_engage_mps = float(vx_deadzone)
        else:
            self.vx_engage_mps = float(vx_engage_mps)

    def map(self, cmd: VelocityCommand) -> JumpScheduleOutput:
        intent = abs(float(cmd.vx)) >= self.vx_engage_mps
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

    def max_forward_vx(self) -> float:
        return SoftTrotSchedule(self.env).max_forward_vx()

    def map(self, cmd: VelocityCommand) -> GaitScheduleOutput:
        """Forward Walk — same SI inversion as SoftTrot; yaw ignored (v1)."""
        soft = SoftTrotSchedule(self.env).map(
            VelocityCommand(vx=float(cmd.vx), yaw_rate=0.0, vy=float(cmd.vy))
        )
        return GaitScheduleOutput(
            amp_front=soft.amp_front,
            amp_rear=soft.amp_rear,
            step_height_front=soft.step_height_front,
            step_height=soft.step_height,
            period=soft.period,
            stance_ratio=soft.stance_ratio,
            turn_cmd=0.0,
            turn_y_gain=0.0,
            vel_cmd=(soft.vel_cmd[0], soft.vel_cmd[1], 0.0),
            speed_frac=soft.speed_frac,
            turn_y_amp=0.0,
            turn_amp_diff=0.0,
            spot_turn=False,
            spot_y_hold_max=soft.spot_y_hold_max,
            spot_yaw_step=soft.spot_yaw_step,
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
