"""Balance-control runtime components."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Optional

from marsdog_control.control.imu_balance import imu_phase_gain
from marsdog_control.control.ramps import softstart_gain


@dataclass
class BalanceOutput:
    imu_dz: Optional[dict] = None
    imu_state: Optional[dict] = None
    t_rel: float = 0.0
    effective_roll: float = 0.0
    effective_pitch: float = 0.0
    gyro_roll: float = 0.0
    gyro_pitch: float = 0.0
    gait_phase: float = 0.0
    in_softstart: bool = False


@dataclass
class RuntimeBalanceConfig:
    imu_softstart_s: float = 1.5
    trim_ramp_s: float = 3.0
    d_ramp_s: float = 3.0
    imu_phase_gate: bool = True
    phase_td_gain: float = 0.35
    phase_swing_gain: float = 0.70
    td_window: float = 0.15
    variable_impedance: bool = False
    td_imu_freeze_i: bool = True
    imu_slew_m_s: float = 0.0
    ff_decouple: bool = False
    auto_trim: bool = False
    ramp_s: float = 3.5


class RuntimeBalanceController:
    """IMU balance policy wrapper used by the walk runtime.

    This owns loop-time IMU policies that used to live inline in the app:
    D-ramp, touchdown freeze, phase gain, soft-start, trim ramp and stable
    auto-trim sampling. The low-level PID remains ``ImuAttitudeController``.
    """

    def __init__(self, imu_ctrl, config: RuntimeBalanceConfig, *, clock=time):
        self.imu_ctrl = imu_ctrl
        self.config = config
        self.clock = clock
        self.prev_t = clock.monotonic()
        self.ff_hist: list[list[float]] = []
        self._ff_sample_t = 0.0

    def reset(self) -> None:
        self.prev_t = self.clock.monotonic()
        self.ff_hist.clear()
        self._ff_sample_t = 0.0

    def update(self, *, imu, active_gait, t_gait: float, mode: str,
               throttle: float) -> BalanceOutput:
        now_wall = self.clock.time()
        t_rel = (now_wall - t_gait) if active_gait else 0.0
        eff_roll = 0.0
        eff_pitch = 0.0
        if imu and imu.connected:
            eff_roll = imu.roll
            eff_pitch = imu.pitch
            if self.config.ff_decouple and active_gait:
                if hasattr(active_gait, "get_expected_roll"):
                    eff_roll -= math.radians(active_gait.get_expected_roll(t_rel))
                if hasattr(active_gait, "get_expected_pitch"):
                    eff_pitch -= math.radians(active_gait.get_expected_pitch(t_rel))

        d_ramp = min(1.0, t_rel / self.config.d_ramp_s) if active_gait else 1.0
        gyro_roll = imu.gyro_roll * d_ramp if (imu and imu.connected) else 0.0
        gyro_pitch = imu.gyro_pitch * d_ramp if (imu and imu.connected) else 0.0

        touchdown_active = False
        if active_gait and self.config.variable_impedance:
            per = active_gait.period
            offsets = active_gait._PHASE_OFFSET
            for leg in ("fl", "fr", "rl", "rr"):
                phase = (t_rel / per + offsets[leg]) % 1.0
                if phase < self.config.td_window or phase > 1.0 - self.config.td_window:
                    touchdown_active = True
                    break

        now_mono = self.clock.monotonic()
        dt_s = max(0.001, now_mono - self.prev_t)
        self.prev_t = now_mono

        gait_phase = 0.0
        if active_gait:
            gait_phase = (t_rel / active_gait.period) % 1.0

        ss_gain = 1.0
        if active_gait and self.config.imu_softstart_s > 1e-6:
            ss_gain = softstart_gain(t_rel, self.config.imu_softstart_s)
        in_softstart = ss_gain < 0.999

        trim_gain = softstart_gain(t_rel, self.config.trim_ramp_s) if active_gait else 1.0

        imu_dz = None
        if self.imu_ctrl.enabled and imu and imu.connected:
            phase_gain = 1.0
            if self.config.imu_phase_gate and active_gait:
                phase_gain = imu_phase_gain(
                    gait_phase, active_gait.stance_ratio, self.config.td_window,
                    td_gain=self.config.phase_td_gain,
                    swing_gain=self.config.phase_swing_gain,
                )
            ages = imu.frame_ages(now_mono)
            imu_dz = self.imu_ctrl.update(
                eff_roll, eff_pitch,
                gyro_roll, gyro_pitch,
                freeze_integrator=((self.config.td_imu_freeze_i and touchdown_active)
                                   or in_softstart),
                slew_limit_m_per_s=self.config.imu_slew_m_s,
                dt_s=dt_s,
                gait_phase=gait_phase,
                trim_gain=trim_gain,
                phase_gain=phase_gain,
                angle_age_s=ages["angle"],
                gyro_age_s=ages["gyro"],
            )
            if imu_dz and ss_gain < 1.0:
                imu_dz = {k: v * ss_gain for k, v in imu_dz.items()}

        self._sample_trim_if_stable(
            active_gait=active_gait,
            mode=mode,
            throttle=throttle,
            t_rel=t_rel,
            in_softstart=in_softstart,
            eff_roll=eff_roll,
            gyro_roll=gyro_roll,
        )

        imu_state = None
        if imu and imu.connected:
            imu_state = {"roll": eff_roll, "gyro_roll": gyro_roll}
        return BalanceOutput(
            imu_dz=imu_dz,
            imu_state=imu_state,
            t_rel=t_rel,
            effective_roll=eff_roll,
            effective_pitch=eff_pitch,
            gyro_roll=gyro_roll,
            gyro_pitch=gyro_pitch,
            gait_phase=gait_phase,
            in_softstart=in_softstart,
        )

    def _sample_trim_if_stable(self, *, active_gait, mode: str, throttle: float,
                               t_rel: float, in_softstart: bool,
                               eff_roll: float, gyro_roll: float) -> None:
        if not (
            self.config.auto_trim
            and active_gait
            and mode in ("trot_fwd", "natural_fwd")
            and not in_softstart
            and t_rel > (self.config.ramp_s + 1.0)
            and throttle > 0.6
            and abs(math.degrees(eff_roll)) < 3.0
            and abs(math.degrees(gyro_roll)) < 20.0
        ):
            return
        now = self.clock.time()
        if now - self._ff_sample_t > 0.2:
            self._ff_sample_t = now
            self.ff_hist.append(self.imu_ctrl.get_roll_ff_mm())


class NullBalanceController:
    """No-op balance controller used by tests and dry pipeline assembly."""

    def update(self, state, fsm) -> BalanceOutput:
        active = getattr(fsm, "active_gait", None)
        t_rel = 0.0
        if active is not None:
            import time

            t_rel = time.time() - getattr(fsm, "t_gait", time.time())
        return BalanceOutput(t_rel=t_rel)


__all__ = [
    "BalanceOutput",
    "RuntimeBalanceConfig",
    "RuntimeBalanceController",
    "NullBalanceController",
]
