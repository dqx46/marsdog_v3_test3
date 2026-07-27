"""Loop-time motion target selection for special modes and normal gait."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from marsdog_control.core.types import MotionTarget, RobotMode
from marsdog_control.motion.motion_planner import (
    build_calf_pitch_direction_test_target,
    build_hip_abduction_test_target,
    build_leg_pitch_direction_test_target,
    build_motion_target,
)


@dataclass
class DirectionTestConfig:
    hip_abd: bool = False
    leg_pitch: bool = False
    calf_pitch: bool = False
    leg_pitch_amp_rad: float = 0.0
    calf_pitch_amp_rad: float = 0.0
    fade_s: float = 1.0
    base: Optional[dict] = None
    start_mono: float = 0.0


def select_motion_target(
    *,
    fsm,
    state,
    imu_dz,
    imu_state,
    online,
    cur_pos,
    smooth_tgt,
    stand,
    lie_down_hold: bool = False,
    lie_down_targets: Optional[dict] = None,
    direction_test: Optional[DirectionTestConfig] = None,
    clock=None,
) -> MotionTarget:
    """Choose the motion target for one control cycle.

    Priority: lie-down hold > direction tests > normal planner.
    """
    if lie_down_hold:
        return MotionTarget(
            q=dict(lie_down_targets or {}),
            dq={},
            source_mode=RobotMode.STAND,
        )

    if direction_test is not None:
        import time as _time
        clock = clock or _time
        elapsed = clock.monotonic() - direction_test.start_mono
        base = direction_test.base or {}
        online_set = set(online)
        if direction_test.hip_abd:
            return build_hip_abduction_test_target(
                stand, base, online_set,
                elapsed_s=elapsed,
                duration_s=direction_test.fade_s,
            )
        if direction_test.leg_pitch:
            return build_leg_pitch_direction_test_target(
                base, online_set,
                amplitude_rad=direction_test.leg_pitch_amp_rad,
                elapsed_s=elapsed,
                duration_s=direction_test.fade_s,
            )
        if direction_test.calf_pitch:
            return build_calf_pitch_direction_test_target(
                base, online_set,
                amplitude_rad=direction_test.calf_pitch_amp_rad,
                elapsed_s=elapsed,
                duration_s=direction_test.fade_s,
            )

    return build_motion_target(
        fsm, state, imu_dz, imu_state, online, cur_pos, smooth_tgt)


__all__ = ["DirectionTestConfig", "select_motion_target"]
