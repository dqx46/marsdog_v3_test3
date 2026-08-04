"""Gait-tuning map — where to change what (no multi-source guessing).

Operator cheat-sheet
--------------------
1. **Natural Soft Trot 形状/手感** (period / amp / step / foot track / soft shape)
   → ``motion/gait_recipes.py`` 里的 ``NATURAL_SOFT_TROT``
   （``NATURAL_SOFT_TROT_WBC`` / ``_REAL`` 是同一对象别名）。
   启动时会灌进 CLI args（显式 ``--flag`` 优先保留）。

2. **全局增益 / DM / IMU / features** (leg_kp、dm_kp、imu_kp…)
   → ``config/schema.py``（CLI 经 ``defaults.CLI`` 跟随）。
   SoftTrot 预设会覆盖 lead/predict 等为 0。

3. **仅本次试验的覆盖**
   → 命令行 ``--flag``（记入 ``_explicit_cli``，不被预设盖掉）。
   SoftTrot 步频优先用 ``--gait-period SEC`` 或 ``--gait-hz HZ``
   （同时覆盖 ``period`` / ``nat_period``）。

``GaitCliDefaults`` 是 **未进 RuntimeConfig** 的步态细参 argparse 默认值单点；
形状键与 ``NATURAL_SOFT_TROT`` 对齐（见 ``tests/test_gait_tuning_sync.py``）。
"""

from __future__ import annotations

from dataclasses import dataclass, fields
from types import SimpleNamespace
from typing import Iterable, Optional


@dataclass(frozen=True)
class GaitCliDefaults:
    """Argparse / from_args defaults for gait knobs outside RuntimeConfig."""

    # stand / geometry
    # 2026-08-04 真机 sim_com_balance: final com_x=-0.015 → x_shift=-com_x
    # 叠在原默认 0.015 的站立偏置上 → 固化 0.030（脚前移/重心后移 +30mm）
    x_shift: float = 0.030
    waist_pitch: float = 0.05
    waist_yaw_offset: float = 0.0
    front_stand_foot_pitch_deg: float = -90.0
    front_stand_tarsus_deg: float = 0.0

    # forward / thrust / foot — SoftTrot SSOT aligned
    fwd_front_lift: float = 0.020
    fwd_front_amp_scale: float = 1.0
    fwd_use_bwd: bool = False
    front_thrust_gain: float = 1.0
    front_thrust_swing_gain: float = 1.0
    front_tarsus_push: float = 0.0
    front_foot_track_deg: float = -78.0
    front_foot_stance_push_deg: float = 0.0
    front_foot_swing_track: float = 1.0
    swing_clearance_per_rad: float = 0.35
    reactive_kp: float = 0.0
    reactive_kd: float = 0.0
    lateral_sway: float = 0.0
    # SoftTrot 横向质心规划 (m); >0 时用事件型移重，覆盖 half-sine lateral_sway
    com_shift_m: float = 0.012
    com_shift_blend: float = 0.15
    anti_roll: float = 0.0
    trot_roll_ff_neg_deg: float = 0.0
    trot_roll_ff_pos_deg: float = 0.0
    anti_roll_asym_neg: float = 1.0
    anti_roll_asym_pos: float = 1.0

    # backward / pace
    bwd_amp_scale: float = 0.7
    bwd_period: float = 0.85
    bwd_step_h: float = 0.015
    pace_amp: float = 0.008
    pace_step_h: float = 0.015
    pace_period: float = 1.2
    pace_stance: float = 0.75
    pace_hip_abd: float = 0.0
    pace_sway: float = 0.015

    # natural shape — aligned with NATURAL_SOFT_TROT
    nat_period: float = 1.05
    nat_amp_front: float = 0.022
    nat_amp_rear: float = 0.030
    nat_step_h: float = 0.024
    spine_yaw_deg: float = 0.0
    spine_roll_deg: float = 0.0
    spine_phase_deg: float = 0.0
    thigh_swing_front_deg: float = 0.0
    thigh_swing_rear_deg: float = 0.0
    retract_front: float = 0.010
    retract_rear: float = 0.008
    tarsus_swing_deg: float = 0.0
    touchdown_compress: float = 0.0035
    anti_roll_soft_scale: float = 0.0
    toeoff_lift: float = 0.0008
    retract_peak: float = 0.42
    lift_peak: float = 0.48

    # turn layer
    turn_amp_diff: float = 0.012
    turn_y_amp: float = 0.040
    turn_smooth: float = 0.015
    turn_waist_yaw: float = 0.40
    waist_yaw_turn_sign: float = 1.0
    cruise_turn_scale: float = 0.6
    cruise_turn_yamp: float = 1.0
    turn_sign: float = 1.0
    throttle_min_scale: float = 0.45

    # misc experimental (still gait-adjacent CLI)
    damp_hard_mm: float = 3.0
    damp_gyro_lo: float = 20.0
    damp_gyro_hi: float = 80.0
    roll_p_boost: float = 1.0
    roll_p_lo_deg: float = 6.0
    roll_p_hi_deg: float = 14.0
    imu_ema: float = 0.0
    swing_level: float = 0.0
    rear_clearance_m: float = 0.0


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
        print("    [形状] motion/gait_recipes.py → NATURAL_SOFT_TROT"
              if natural_soft else
              "    [形状] motion/gait_recipes.py → NATURAL_TROT_REAL")
    print("    [增益/DM/IMU] config/schema.py  (CLI 自动跟随)")
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
