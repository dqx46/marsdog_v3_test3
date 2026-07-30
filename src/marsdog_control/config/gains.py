"""Default per-joint impedance gains for live walk actuation.

Authoritative table consumed by ``WalkRuntimeState`` / ``CommandExecutor``.
Tune front calf here — not in ``apps/walk.py``.
"""

from __future__ import annotations

# 2026-07-17: fl/fr_calf softened to kp=50 kd=1 (was 120/5) for Incos compliance.
# 2026-07-21: after shared-CAN Incos fix + EVO keep-alive rate cut, retune weak
# axes from limb sine (JOINT_GAINS + vel FF): tarsus and front calves were the
# remaining high-RMS joints at period≤1s; hips are OK at 120/10 with vel FF.
# 2026-07-21b: FR calf/tarsus tracked worse than FL at identical gains → bump
# FR only (mechanical asymmetry), keep FL.
# 2026-07-30: sim 与真机共用 JOINT_GAINS。真机因克斯 thigh_roll 软化
# (70/1.5, 80/2.0) 会让仿真欠阻尼起步蹦跳；此处保持原版 sim 增益。
# 真机软化应放到 Real-only 覆盖，勿改本表。
JOINT_GAINS = {
    "fl_hip_pitch":  {"kp": 150.0, "kd": 5.0, "trq_ff": 0.0},
    "fr_hip_pitch":  {"kp": 150.0, "kd": 5.0, "trq_ff": 0.0},
    "fl_thigh_roll": {"kp": 80.0,  "kd": 5.0, "trq_ff": 0.0},
    "fr_thigh_roll": {"kp": 90.0,  "kd": 5.0, "trq_ff": 0.0},
    "fl_calf":       {"kp": 70.0,  "kd": 1.5, "trq_ff": 0.35},
    "fr_calf":       {"kp": 90.0,  "kd": 2.0, "trq_ff": 0.40},
    # 达妙 tarsus：2026-07-21 单独扫频 ±15°（vel FF），FL/FR 同增益即可对齐；
    # 220/10 在 T=1s RMS≈1.0°，再抬到 300 收益变小。
    "fl_tarsus":     {"kp": 220.0, "kd": 10.0, "trq_ff": 0.0},
    "fr_tarsus":     {"kp": 220.0, "kd": 10.0, "trq_ff": 0.0},
    "rl_hip":        {"kp": 120.0, "kd": 10.0, "trq_ff": 0.0},  # EVO: KD_MAX=50
    "rr_hip":        {"kp": 120.0, "kd": 10.0, "trq_ff": 0.0},  # EVO: KD_MAX=50
    "rl_thigh":      {"kp": 140.0, "kd": 5.0, "trq_ff": 0.30},
    "rr_thigh":      {"kp": 140.0, "kd": 5.0, "trq_ff": 0.30},
    "rl_calf":       {"kp": 120.0, "kd": 5.0, "trq_ff": 0.45},
    "rr_calf":       {"kp": 120.0, "kd": 5.0, "trq_ff": 0.45},
    "head_pitch":    {"kp": 30.0,  "kd": 3.0, "trq_ff": 0.0},
    "head_yaw":      {"kp": 30.0,  "kd": 3.0, "trq_ff": 0.0},
    "head_roll":     {"kp": 30.0,  "kd": 3.0, "trq_ff": 0.0},
    "neck_pitch":    {"kp": 30.0,  "kd": 5.0, "trq_ff": 0.0},   # EVO: KD_MAX=50
    "waist_yaw":     {"kp": 50.0,  "kd": 5.0, "trq_ff": 0.0},   # EVO: KD_MAX=50
    "waist_pitch":   {"kp": 60.0,  "kd": 5.0, "trq_ff": 0.0},   # EVO: KD_MAX=50
    "waist_roll":    {"kp": 50.0,  "kd": 5.0, "trq_ff": 0.0},
}

__all__ = ["JOINT_GAINS"]
