"""Gain / gravity helpers for CommandExecutor.

Split from ``executor.py`` so brand/phase gain math stays independently testable.
"""

from __future__ import annotations

from marsdog_control.config.joints import JOINT_BY_NAME as JBN, JOINT_MAP
from marsdog_control.control.gravity_comp import leg_gravity_ff


_GC_LEG_JOINTS = {
    "fl": {"hip_pitch": "fl_hip_pitch", "calf": "fl_calf"},
    "fr": {"hip_pitch": "fr_hip_pitch", "calf": "fr_calf"},
    "rl": {"thigh": "rl_thigh", "calf": "rl_calf"},
    "rr": {"thigh": "rr_thigh", "calf": "rr_calf"},
}


def gravity_trq(targets, grav_scale):
    """按当前目标位姿计算腿部 pitch 关节重力补偿前馈 (电机端 Nm)。"""
    out = {}
    for leg, jmap in _GC_LEG_JOINTS.items():
        angles = {}
        ok = True
        for key, jname in jmap.items():
            j = JBN[jname]
            if j.motor_id not in targets:
                ok = False
                break
            angles[key] = targets[j.motor_id]
        if not ok:
            continue
        ff = leg_gravity_ff(leg, angles)
        for key, jname in jmap.items():
            j = JBN[jname]
            out[j.motor_id] = ff[key] * grav_scale
    return out


def _is_leg_joint(name: str) -> bool:
    return name[:3] in ("fl_", "fr_", "rl_", "rr_")


_LEG_MOTOR_IDS = [
    (j.motor_id, j.name[:2]) for j in JOINT_MAP if _is_leg_joint(j.name)
]


def resolve_gains(
    j,
    kp_scale,
    use_joint_gains,
    kp_lz,
    kd_lz,
    kp_evo,
    kd_evo,
    leg_kp_scale,
    joint_gains,
    phase_scale=1.0,
    trq_override=None,
    brand_gain_scale=None,
    kp_incos=None,
    kd_incos=None,
):
    """解析单个关节最终的 (kp, kd, trq) — 各品牌 MIT 原生量纲。

    Softening is per-``mtype`` via ``BRAND_GAIN_SCALE`` (or ``brand_gain_scale``).
    ``leg_kp_scale`` is ``ImpedanceAssist`` (session Soft default / jump-spot
    overlay) — orthogonal to ``ForceMode`` τ_ff ownership. WBC still multiplies
    leg joint kp by this scale so τ_ff can dominate MIT PD.
    """
    from marsdog_control.config.gains import (
        BRAND_DEFAULT_GAINS,
        BRAND_GAIN_SCALE,
        brand_scales,
    )

    b = brand_scales(j.mtype, brand_gain_scale or BRAND_GAIN_SCALE)
    # Jump/spot may pass leg_kp_scale != SoftTrot default on leg joints.
    leg_overlay = float(leg_kp_scale) if _is_leg_joint(j.name) else 1.0
    kp_mult = float(kp_scale) * float(b["kp"]) * float(phase_scale) * leg_overlay
    kd_mult = float(b["kd"])
    if use_joint_gains:
        g = joint_gains.get(j.name, {"kp": 30.0, "kd": 4.0, "trq_ff": 0.0})
        trq = g["trq_ff"] if trq_override is None else trq_override
        return g["kp"] * kp_mult, g["kd"] * kd_mult, trq

    if j.mtype == "lz":
        base_kp, base_kd = float(kp_lz), float(kd_lz)
    elif j.mtype == "evo":
        base_kp, base_kd = float(kp_evo), float(kd_evo)
    elif j.mtype == "incos":
        # Prefer explicit IncOS channel; else share the MIT fade channel (kp_evo).
        # Must use ``is not None`` — soft_disable ramps through 0.0.
        d = BRAND_DEFAULT_GAINS["incos"]
        if kp_incos is not None:
            base_kp = float(kp_incos)
        elif kp_evo is not None:
            base_kp = float(kp_evo)
        else:
            base_kp = float(d["kp"])
        if kd_incos is not None:
            base_kd = float(kd_incos)
        elif kd_evo is not None:
            base_kd = float(kd_evo)
        else:
            base_kd = float(d["kd"])
    else:
        d = BRAND_DEFAULT_GAINS.get(j.mtype, {"kp": 30.0, "kd": 4.0})
        base_kp, base_kd = float(d["kp"]), float(d["kd"])
    return base_kp * kp_mult, base_kd * kd_mult, (
        0.0 if trq_override is None else trq_override)
