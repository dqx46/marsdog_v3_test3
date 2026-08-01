"""Per-joint MIT gains in **brand-native** units + per-brand scales.

Authoritative table for ``WalkRuntimeState`` / ``CommandExecutor``.

Design (2026-08):
  - Numbers in ``JOINT_GAINS`` are what we intend to send (modulo phase /
    jump overlay). Same anatomical L/R joints stay symmetric by default.
  - Softening is **per brand** via ``BRAND_GAIN_SCALE``, never a global
    ``leg_kp_scale`` across LZ/EVO/Incos/DM (their MIT kp/kd ranges differ).
  - Tune Incos front calves with ``tests/Motor_test/bench_motor_track.py sine``.
"""

from __future__ import annotations

from typing import Dict, Mapping, MutableMapping

# Per-brand multiplicative scale applied in ``resolve_gains``.
# Incos starts at 1.0 — absolute kp/kd live in JOINT_GAINS after sweep.
BRAND_GAIN_SCALE: Dict[str, Dict[str, float]] = {
    "lz": {"kp": 1.0, "kd": 1.0},
    "evo": {"kp": 1.0, "kd": 1.0},
    "incos": {"kp": 1.0, "kd": 1.0},
    "dm": {"kp": 1.0, "kd": 1.0},
}

# Fallback when ``use_joint_gains=False`` (e.g. soft_disable ramp).
BRAND_DEFAULT_GAINS: Dict[str, Dict[str, float]] = {
    "lz": {"kp": 45.0, "kd": 4.0},
    "evo": {"kp": 30.0, "kd": 4.0},
    "incos": {"kp": 35.0, "kd": 2.5},
    "dm": {"kp": 30.0, "kd": 0.5},
}

# 2026-08: remove global leg_kp_scale=0.65. Prior effective ≈ table×0.65 for
# LZ/EVO legs is baked in below so SoftTrot feel does not jump stiff overnight.
# Incos calves: symmetric sweep start (was FL70/FR90 asymmetry); kd raised to
# cut ground squeal. Re-confirm with Motor_test sine --ids 3,7.
JOINT_GAINS = {
    "fl_hip_pitch":  {"kp": 100.0, "kd": 5.0, "trq_ff": 0.0},
    "fr_hip_pitch":  {"kp": 100.0, "kd": 5.0, "trq_ff": 0.0},
        # Incos front (IDs 2/3/6/7): 2026-08-01 hang sine ±3° T=2s grid.
    # ENCOS V1.19 力位混控空载参考 KP≈15 KD≈0.5；协议 KP≤500 KD≤5。
    # 选 35/2.5：RMS 已接近 45 档、阻尼更足，L/R 同值（勿再 70 vs 90）。
    # trq_ff: 静态重力偏置(Nm)。外展轴重力臂≈0 → 0；小腿站姿需抗重力 → 非零。
    # (开启 gravity_comp/WBC 时 executor 会用动力学 τ 覆盖，此值作无补偿回退。)
    "fl_thigh_roll": {"kp": 35.0,  "kd": 2.5, "trq_ff": 0.0},
    "fr_thigh_roll": {"kp": 35.0,  "kd": 2.5, "trq_ff": 0.0},
    "fl_calf":       {"kp": 35.0,  "kd": 2.5, "trq_ff": 0.35},
    "fr_calf":       {"kp": 35.0,  "kd": 2.5, "trq_ff": 0.35},
    # 达妙 tarsus：扫频对齐后 FL/FR 同增益；外置 1:2 在 mapping 里 /N²
    "fl_tarsus":     {"kp": 220.0, "kd": 10.0, "trq_ff": 0.0},
    "fr_tarsus":     {"kp": 220.0, "kd": 10.0, "trq_ff": 0.0},
    "rl_hip":        {"kp": 78.0,  "kd": 10.0, "trq_ff": 0.0},  # EVO: KD_MAX=50
    "rr_hip":        {"kp": 78.0,  "kd": 10.0, "trq_ff": 0.0},
    "rl_thigh":      {"kp": 91.0,  "kd": 5.0, "trq_ff": 0.30},
    "rr_thigh":      {"kp": 91.0,  "kd": 5.0, "trq_ff": 0.30},
    "rl_calf":       {"kp": 78.0,  "kd": 5.0, "trq_ff": 0.45},
    "rr_calf":       {"kp": 78.0,  "kd": 5.0, "trq_ff": 0.45},
    "head_pitch":    {"kp": 30.0,  "kd": 3.0, "trq_ff": 0.0},
    "head_yaw":      {"kp": 30.0,  "kd": 3.0, "trq_ff": 0.0},
    "head_roll":     {"kp": 30.0,  "kd": 3.0, "trq_ff": 0.0},
    "neck_pitch":    {"kp": 30.0,  "kd": 5.0, "trq_ff": 0.0},
    "waist_yaw":     {"kp": 50.0,  "kd": 5.0, "trq_ff": 0.0},
    "waist_pitch":   {"kp": 60.0,  "kd": 5.0, "trq_ff": 0.0},
    "waist_roll":    {"kp": 50.0,  "kd": 5.0, "trq_ff": 0.0},
}


def brand_scales(mtype: str,
                 table: Mapping[str, Mapping[str, float]] | None = None,
                 ) -> Dict[str, float]:
    src = table if table is not None else BRAND_GAIN_SCALE
    row = src.get(mtype) or {"kp": 1.0, "kd": 1.0}
    return {"kp": float(row.get("kp", 1.0)), "kd": float(row.get("kd", 1.0))}


def set_brand_kp_scale(mtype: str, kp: float,
                       table: MutableMapping[str, Dict[str, float]] | None = None,
                       ) -> None:
    """Runtime helper for sweep scripts; mutates ``BRAND_GAIN_SCALE`` by default."""
    dst = BRAND_GAIN_SCALE if table is None else table
    cur = dict(dst.get(mtype, {"kp": 1.0, "kd": 1.0}))
    cur["kp"] = float(kp)
    dst[mtype] = cur


__all__ = [
    "BRAND_GAIN_SCALE",
    "BRAND_DEFAULT_GAINS",
    "JOINT_GAINS",
    "brand_scales",
    "set_brand_kp_scale",
]
