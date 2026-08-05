"""Gait-tuning map — where to change what (no multi-source guessing).

Operator cheat-sheet
--------------------
1. **Natural Soft Trot 形状/手感** (period / amp / step / foot track / soft shape)
   → ``config/soft_trot_recipe.py`` 里的 ``SoftTrotRecipe`` / ``SOFT_TROT_RECIPE``
   （``NATURAL_SOFT_TROT`` 是其 dict 兼容出口；WBC/REAL 同对象）。
   启动时会灌进 CLI args（显式 ``--flag`` 优先保留）。

2. **全局增益 / DM / IMU / features** (leg_kp、dm_kp、imu_kp…)
   → ``config/schema.py``（CLI 经 ``defaults.CLI`` 跟随；核心几何从 Recipe 派生）。
   SoftTrot 预设会覆盖 lead/predict 等为 0。

3. **仅本次试验的覆盖**
   → 命令行 ``--flag``（记入 ``_explicit_cli``，不被预设盖掉）。
   SoftTrot 步频优先用 ``--gait-period SEC`` 或 ``--gait-hz HZ``
   （同时覆盖 ``period`` / ``nat_period``）。

``GaitCliDefaults`` Soft 形状字段从 ``SOFT_TROT_RECIPE`` 派生（见
``tests/test_gait_tuning_sync.py``）；勿手写第二份数字。
"""

from __future__ import annotations

from dataclasses import dataclass, fields
from types import SimpleNamespace
from typing import Iterable, Optional

from marsdog_control.config.soft_trot_recipe import SOFT_TROT_RECIPE as _R


@dataclass(frozen=True)
class GaitCliDefaults:
    """Argparse / from_args defaults for gait knobs outside RuntimeConfig."""

    # stand / geometry (not Soft shape; x_shift SSOT lives here)
    # 2026-08-04 真机试走：x_shift=0（不前后偏置）；左右用 com_shift=4mm
    x_shift: float = 0.0
    waist_pitch: float = 0.05
    waist_yaw_offset: float = 0.0
    front_stand_foot_pitch_deg: float = _R.front_stand_foot_pitch_deg
    front_stand_tarsus_deg: float = 0.0

    # forward / thrust / foot — derived from SoftTrotRecipe
    fwd_front_lift: float = _R.fwd_front_lift
    fwd_front_amp_scale: float = _R.fwd_front_amp_scale
    fwd_use_bwd: bool = False
    front_thrust_gain: float = _R.front_thrust_gain
    front_thrust_swing_gain: float = _R.front_thrust_swing_gain
    front_tarsus_push: float = _R.front_tarsus_push
    front_foot_track_deg: float = _R.front_foot_track_deg
    front_foot_stance_push_deg: float = _R.front_foot_stance_push_deg
    front_foot_swing_track: float = _R.front_foot_swing_track
    swing_clearance_per_rad: float = _R.swing_clearance_per_rad
    reactive_kp: float = 0.0
    reactive_kd: float = 0.0
    lateral_sway: float = _R.lateral_sway
    com_shift_m: float = _R.com_shift_m
    com_shift_blend: float = _R.com_shift_blend
    anti_roll: float = _R.anti_roll
    trot_roll_ff_neg_deg: float = _R.trot_roll_ff_neg_deg
    trot_roll_ff_pos_deg: float = _R.trot_roll_ff_pos_deg
    anti_roll_asym_neg: float = _R.anti_roll_asym_neg
    anti_roll_asym_pos: float = _R.anti_roll_asym_pos

    # backward / pace (not Soft SSOT)
    bwd_amp_scale: float = 0.7
    bwd_period: float = 0.85
    bwd_step_h: float = 0.015
    pace_amp: float = 0.008
    pace_step_h: float = 0.015
    pace_period: float = 1.2
    pace_stance: float = 0.75
    pace_hip_abd: float = 0.0
    pace_sway: float = 0.015

    # natural shape — derived from SoftTrotRecipe (nat_* = amp/period/step aliases)
    nat_period: float = _R.period
    nat_amp_front: float = _R.amp_front
    nat_amp_rear: float = _R.amp_rear
    nat_step_h: float = _R.step_h
    spine_yaw_deg: float = _R.spine_yaw_deg
    spine_roll_deg: float = _R.spine_roll_deg
    spine_phase_deg: float = 0.0
    thigh_swing_front_deg: float = _R.thigh_swing_front_deg
    thigh_swing_rear_deg: float = _R.thigh_swing_rear_deg
    retract_front: float = _R.retract_front
    retract_rear: float = _R.retract_rear
    tarsus_swing_deg: float = _R.tarsus_swing_deg
    touchdown_compress: float = _R.touchdown_compress
    anti_roll_soft_scale: float = _R.anti_roll_soft_scale
    toeoff_lift: float = _R.toeoff_lift
    retract_peak: float = _R.retract_peak
    lift_peak: float = _R.lift_peak

    # turn layer
    turn_amp_diff: float = _R.turn_amp_diff
    turn_y_amp: float = _R.turn_y_amp
    turn_smooth: float = 0.015
    turn_waist_yaw: float = _R.turn_waist_yaw
    waist_yaw_turn_sign: float = 1.0
    cruise_turn_scale: float = 0.6
    cruise_turn_yamp: float = 1.0
    turn_sign: float = 1.0
    throttle_min_scale: float = _R.throttle_min_scale

    # misc experimental (still gait-adjacent CLI)
    damp_hard_mm: float = 3.0
    damp_gyro_lo: float = 20.0
    damp_gyro_hi: float = 80.0
    roll_p_boost: float = 1.0
    roll_p_lo_deg: float = 6.0
    roll_p_hi_deg: float = 14.0
    imu_ema: float = 0.0
    swing_level: float = _R.swing_level
    rear_clearance_m: float = _R.rear_clearance_m


GAIT = GaitCliDefaults()


def gait_cli_namespace(defaults: Optional[GaitCliDefaults] = None) -> SimpleNamespace:
    """Flat namespace for ``walk_cli`` ``default=`` wiring."""
    d = defaults or GAIT
    return SimpleNamespace(**{f.name: getattr(d, f.name) for f in fields(d)})


def print_tuning_banner(
    *,
    natural_soft: bool,
    natural_active: bool,
    overridden: Iterable[str] = (),
    height: float,
    period: float,
    amp_front: float,
    amp_rear: float,
    step_h: float,
    stance: float,
) -> None:
    """One-screen map so operators do not hunt multi-source knobs."""
    gait_name = (
        "NaturalSoftTrot" if natural_soft
        else ("NaturalTrot" if natural_active else "StableTrot")
    )
    print(f"\n{'='*62}")
    print(f"  Marsdog Walk — {gait_name}")
    print(f"  体高={height:.3f}m  周期={period:.2f}s  stance={stance:.0%}")
    print(f"  摆幅 前=±{amp_front*100:.1f}cm  后=±{amp_rear*100:.1f}cm  "
          f"步高={step_h*100:.1f}cm")
    print(f"{'-'*62}")
    print("  调参入口（单源，不要两边改）:")
    if natural_soft or natural_active:
        print("    [形状] config/soft_trot_recipe.py → SoftTrotRecipe"
              if natural_soft else
              "    [形状] motion/gait_recipes.py → NATURAL_TROT_REAL")
    print("    [增益/DM/IMU] config/schema.py  (CLI 自动跟随; Soft 只灌几何+足形+overlay)")
    print("    [本次覆盖] 命令行 --flag  (显式优先于预设)")
    ov = list(overridden)
    if ov:
        print("    本次显式 CLI 覆盖: " + ", ".join(ov))
    print(f"{'='*62}\n")


def soft_trot_shape_keys() -> frozenset:
    """Keys that SoftTrot preset owns (must stay in sync with GaitCliDefaults)."""
    return frozenset({
        "nat_period", "nat_amp_front", "nat_amp_rear", "nat_step_h",
        "spine_yaw_deg", "spine_roll_deg",
        "thigh_swing_front_deg", "thigh_swing_rear_deg",
        "retract_front", "retract_rear", "tarsus_swing_deg",
        "touchdown_compress", "anti_roll_soft_scale", "toeoff_lift",
        "retract_peak", "lift_peak",
        "front_foot_track_deg", "front_stand_foot_pitch_deg",
        "front_foot_swing_track", "front_foot_stance_push_deg",
        "swing_level",
        "swing_clearance_per_rad",
        "com_shift_m", "com_shift_blend",
        "rear_clearance_m",
    })


__all__ = [
    "GAIT",
    "GaitCliDefaults",
    "gait_cli_namespace",
    "print_tuning_banner",
    "soft_trot_shape_keys",
]
