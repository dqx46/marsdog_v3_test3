"""Runtime recorder orchestration."""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Callable


@dataclass
class RecorderRuntime:
    """Owns loop logging cadence and row context assembly."""

    writer: object
    write_log: Callable
    interval: int = 5
    clock: object = time
    run_start_mono: float = field(init=False)
    cycle: int = 0

    def __post_init__(self) -> None:
        self.run_start_mono = self.clock.monotonic()
        self.interval = max(1, int(self.interval))

    def maybe_record(self, *, mode: str, lz, evo, dm, incos, targets: dict,
                     loop_dt_s: float, active_gait, throttle: float, imu,
                     imu_dz, imu_ctrl, kp_phase, trq_ff, fsm, cmd,
                     stand, bench_tarsus_side=None, bench_start=None,
                     control_period_ms: float = 0.0) -> None:
        self.cycle += 1
        if not self.writer or (self.cycle % self.interval) != 0:
            return

        if bench_tarsus_side and bench_start is not None:
            t_rel_log = self.clock.monotonic() - bench_start
        elif active_gait:
            t_rel_log = self.clock.time() - fsm.t_gait
        else:
            t_rel_log = 0.0

        ramp_f = 0.0
        if active_gait and hasattr(active_gait, "ramp_duration"):
            if active_gait.ramp_duration > 0:
                rf = t_rel_log / active_gait.ramp_duration
            else:
                rf = 1.0
            ramp_f = min(1.0, max(0.0, 3 * rf * rf - 2 * rf * rf * rf))

        controller_name = (
            active_gait.__class__.__name__
            if active_gait is not None
            else stand.__class__.__name__
        )
        request_mode = cmd.request_mode.value if cmd.request_mode is not None else ""

        self.write_log(
            self.writer, t_rel_log, mode, lz, evo, dm, incos, targets,
            loop_dt_s * 1000.0, active_gait, throttle, imu, imu_dz,
            imu_ctrl=imu_ctrl, ramp_frac=ramp_f,
            kp_phase=kp_phase, trq_ff=trq_ff,
            run_t_s=self.clock.monotonic() - self.run_start_mono,
            fsm_mode=fsm.mode.value,
            gait_active=(active_gait is not None),
            controller_name=controller_name,
            input_vx=cmd.vx,
            input_turn=cmd.turn,
            input_has_stick=cmd.has_stick,
            input_request_mode=request_mode,
            control_period_ms=control_period_ms,
        )


__all__ = ["RecorderRuntime"]
