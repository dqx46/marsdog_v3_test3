"""Ground tarsus sweep-bench helpers and runtime overlay."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence


def tarsus_bench_reference(elapsed_s, frequencies_hz, amplitude_rad,
                           cycles=3.0, settle_s=2.0):
    """Return ground sweep reference ``(delta, frequency, done)``.

    Each frequency stage starts and ends continuously at zero.
    """
    t = max(0.0, elapsed_s)
    for frequency in frequencies_hz:
        duration = max(1.0, cycles) / frequency
        if t < settle_s:
            return 0.0, frequency, False
        t -= settle_s
        if t < duration:
            return amplitude_rad * math.sin(2.0 * math.pi * frequency * t), frequency, False
        t -= duration
    return 0.0, 0.0, True


@dataclass
class TarsusBenchConfig:
    side: str  # "fl" or "fr"
    frequencies_hz: Sequence[float]
    amplitude_rad: float
    cycles: float = 3.0
    settle_s: float = 2.0
    max_error_deg: float = 5.0
    max_tilt_deg: float = 8.0
    max_torque_nm: float = 8.0
    kp_by_id: Optional[dict] = None


@dataclass
class TarsusBenchResult:
    abort: bool = False
    done: bool = False
    reason: str = ""
    frequency_hz: float = 0.0


class TarsusBenchRuntime:
    """Owns the stand-mode single-tarsus sweep overlay used by walk."""

    def __init__(self, config: TarsusBenchConfig, *, clock=None):
        import time as _time
        self.config = config
        self.clock = clock or _time
        self.motor_id = 4 if config.side == "fl" else 8
        self.start_mono: Optional[float] = None
        self.base_target: Optional[float] = None
        self.base_frozen = False
        self.done = False
        self.frequency_hz = 0.0

    def apply(self, *, mode: str, motion, state, dm) -> TarsusBenchResult:
        if mode != "stand":
            return TarsusBenchResult(abort=True, reason="测试期间离开 STAND")

        if self.start_mono is None:
            self.start_mono = self.clock.monotonic()
            print("[BENCH] 先原地回中并稳定，然后自动逐档扫频；q/ESC可随时退出")

        elapsed = self.clock.monotonic() - self.start_mono
        settle = max(0.5, self.config.settle_s)
        if elapsed < settle:
            self.base_target = motion.q.get(self.motor_id)
        elif not self.base_frozen:
            self.base_frozen = True
            print(f"[BENCH] {self.config.side.upper()} 中心已固定: "
                  f"{math.degrees(self.base_target):.2f}°")

        delta, self.frequency_hz, self.done = tarsus_bench_reference(
            elapsed,
            self.config.frequencies_hz,
            self.config.amplitude_rad,
            self.config.cycles,
            settle,
        )
        base = self.base_target
        actual = state.joint_pos.get(self.motor_id)
        if base is None or actual is None:
            return TarsusBenchResult(
                abort=True, reason="指定 tarsus 无有效目标/反馈")

        command = base + delta
        error = abs(command - actual)
        kp_map = self.config.kp_by_id or {}
        kp_now = kp_map.get(self.motor_id, 0.0)
        p_torque = kp_now * error
        measured_torque = abs(dm.get_torque(self.motor_id)) if dm is not None else 0.0
        tilt_deg = max(abs(math.degrees(state.roll)),
                       abs(math.degrees(state.pitch)))
        if (math.degrees(error) > self.config.max_error_deg
                or tilt_deg > self.config.max_tilt_deg
                or max(p_torque, measured_torque) > self.config.max_torque_nm):
            return TarsusBenchResult(
                abort=True,
                reason=(
                    "安全阈值触发: "
                    f"error={math.degrees(error):.2f}°, tilt={tilt_deg:.2f}°, "
                    f"tau(est/meas)={p_torque:.2f}/{measured_torque:.2f}Nm"
                ),
            )

        motion.q[self.motor_id] = command
        return TarsusBenchResult(
            done=self.done,
            frequency_hz=self.frequency_hz,
        )


__all__ = [
    "TarsusBenchConfig",
    "TarsusBenchResult",
    "TarsusBenchRuntime",
    "tarsus_bench_reference",
]
