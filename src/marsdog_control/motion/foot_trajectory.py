"""足端轨迹纯几何函数 — 从 `gait_controller.py` 的 `StableTrot`/`NaturalTrot`/
`NaturalSoftTrot` 抠出的**纯函数**版本 (Phase O)。

为什么单独拆出来:
  这些函数只依赖显式传入的标量(phase/amp/cx/...), 不读写 ``self``, 也不调用任何
  被子类重写的方法(不涉及虚函数分发) —— 是真正可独立单测、可独立推理的"轨迹形状"
  纯数学, 和"这是哪个步态类/这个实例现在的可调参数是什么"完全无关。

和谁划清界限:
  三个类各自的 ``_leg_xz`` 仍然留在 `gait_controller.py`, 因为它们要调用
  ``self._swing_z(...)``(每个子类重写的虚方法) 决定摆动相抬腿高度 —— 这一层
  "选哪种抬腿曲线"的多态分发是控制器身份的一部分, 不应该被拉平成纯函数, 否则
  改一个子类的 ``_swing_z`` 就不再能自动影响它的摆动相高度。
  这里只抠出摆动相/支撑相里**纯几何**的部分(X 轨迹曲线、支撑相 anti-roll 抬腿量、
  横向摆动、转向、相位门控…)。

零行为改动: 每个函数都是原方法体的逐字搬运(仅把 ``self.xxx`` 换成显式参数)。
真实性由三处保证: (1) 离线 54 单元 + parity 3 逐字节金样; (2) 真机复验;
(3) 本文件不做任何默认值猜测——所有默认值原样抄自 `gait_controller.py` 构造函数。
"""

from __future__ import annotations

import math


# ─────────────────────────────────────────────────────────────────────────────
# 摆动相 Z 曲线 (每个步态类的 ``_swing_z`` 逐字搬运)
# ─────────────────────────────────────────────────────────────────────────────

def three_phase_swing_z(swing_t: float, step_h: float,
                         rise_end: float = 0.4, cruise_end: float = 0.7) -> float:
    """StableTrot: 三段式摆动 Z: 快起 → 巡航 → 柔落, C1 连续。"""
    if swing_t < rise_end:
        s = math.sin(math.pi * swing_t / (2.0 * rise_end))
        return step_h * s * s
    elif swing_t < cruise_end:
        return step_h
    else:
        fall_t = (swing_t - cruise_end) / (1.0 - cruise_end)
        s = math.cos(math.pi * fall_t / 2.0)
        return step_h * s * s


def sin2_swing_z(swing_t: float, step_h: float) -> float:
    """NaturalTrot: sin² 抬腿, 起落脚端速度为 0。"""
    s = math.sin(math.pi * swing_t)
    return step_h * s * s


def minimum_jerk(u: float) -> float:
    """5 次多项式 minimum-jerk: u∈[0,1] → [0,1], 首尾一阶+二阶导数均为 0。"""
    u = max(0.0, min(1.0, u))
    return u * u * u * (10.0 + u * (-15.0 + 6.0 * u))


def minimum_jerk_bump(u: float, peak: float = 0.5) -> float:
    """先用 minimum_jerk 升到 1 再降回 0 的"钟形"包络, 峰值位置可调。"""
    peak = max(0.05, min(0.95, peak))
    if u <= peak:
        return minimum_jerk(u / peak)
    return 1.0 - minimum_jerk((u - peak) / (1.0 - peak))


def minimum_jerk_swing_z(swing_t: float, step_h: float, lift_peak: float) -> float:
    """NaturalSoftTrot: minimum-jerk 钟形抬腿。"""
    return step_h * minimum_jerk_bump(swing_t, lift_peak)


# ─────────────────────────────────────────────────────────────────────────────
# 支撑相 anti-roll / 横向摆动 / 转向 / 相位门控
# ─────────────────────────────────────────────────────────────────────────────

def anti_roll_diag_scale(t: float, period: float, stance_ratio: float,
                          asym_neg: float, asym_pos: float) -> float:
    """按对角支撑相缩放 anti_roll: FL+RR 负 roll 大 → 加大伸腿。"""
    bp = (t / period) % 1.0
    sr = stance_ratio
    if bp < sr:
        return asym_neg
    if bp < 0.5:
        return 1.0
    if bp < 0.5 + sr:
        return asym_pos
    return 1.0


def stance_anti_roll_lift(stance_t: float, anti_roll: float, diag_scale: float) -> float:
    """支撑相 anti-roll Z 补偿(负值=腿伸长=撑起 body), 半正弦包络。"""
    return -anti_roll * diag_scale * math.sin(math.pi * stance_t)


def lateral_offset_trot(t: float, period: float, stance_ratio: float,
                         lateral_sway: float) -> float:
    """StableTrot/NaturalTrot: 横向 CoM 偏移, 与对角支撑严格同步。"""
    phase = (t / period) % 1.0
    sr = stance_ratio
    if phase < sr:
        t_norm = phase / sr
        return lateral_sway * math.sin(math.pi * t_norm)
    else:
        t_norm = (phase - sr) / (1.0 - sr)
        return -lateral_sway * math.sin(math.pi * t_norm)


def lateral_offset_pace(t: float, period: float, stance_ratio: float,
                         lateral_sway: float) -> float:
    """StablePace 专用横向偏移 — 全周期余弦, 抬腿瞬间重心已转移。"""
    phase = (t / period) % 1.0
    return lateral_sway * math.cos(
        2.0 * math.pi * (phase - stance_ratio / 2.0)
    )


def expected_diagonal_roll(t: float, period: float, stance_ratio: float,
                            roll_ff_neg_deg: float, roll_ff_pos_deg: float) -> float:
    """对角 Trot 支撑引起的预期 roll (度), 半正弦包络, 支撑中期达峰。"""
    if roll_ff_neg_deg <= 1e-6:
        return 0.0
    phase = (t / period) % 1.0
    sr = stance_ratio
    if phase < sr:
        t_norm = phase / sr
        return -roll_ff_neg_deg * math.sin(math.pi * t_norm)
    if phase < 0.5:
        return 0.0
    if phase < 0.5 + sr:
        t_norm = (phase - 0.5) / sr
        return roll_ff_pos_deg * math.sin(math.pi * t_norm)
    return 0.0


def leg_y_turn(leg: str, phase: float, turn: float, stance_ratio: float,
               max_turn_y_amp: float, turn_y_gain: float) -> float:
    """转向时的 Y 轴跨步偏移量 (正值=向左跨步)。"""
    if abs(turn) < 0.001:
        return 0.0
    y_amp = (
        -turn * max_turn_y_amp if leg.startswith('f') else turn * max_turn_y_amp
    ) * turn_y_gain
    if phase < stance_ratio:
        stance_t = phase / stance_ratio
        return y_amp * math.cos(math.pi * stance_t)
    else:
        swing_t = (phase - stance_ratio) / (1.0 - stance_ratio)
        return -y_amp * math.cos(math.pi * swing_t)


def stance_weight(phase: float, stance_ratio: float, swing_level: float,
                   taper: float) -> float:
    """支撑相平滑门控权重 [0,1]: 支撑腿(踩地)接受 IMU Z 修正。"""
    if phase >= stance_ratio:
        return swing_level
    up = phase / taper if taper > 1e-6 else 1.0
    down = (stance_ratio - phase) / taper if taper > 1e-6 else 1.0
    stance_w = max(0.0, min(1.0, min(up, down)))
    return max(stance_w, swing_level)


def foot_track_gate(phase: float, stance_ratio: float, floor: float,
                     taper: float) -> float:
    """足朝向跟踪门控 [floor,1]: 支撑相=1(脚尖指地), 摆动相=floor。"""
    if phase >= stance_ratio:
        return floor
    up = phase / taper if taper > 1e-6 else 1.0
    down = (stance_ratio - phase) / taper if taper > 1e-6 else 1.0
    stance_w = max(0.0, min(1.0, min(up, down)))
    return max(stance_w, floor)


# ─────────────────────────────────────────────────────────────────────────────
# 摆动相大腿 flourish / 前腿跗关节收放 (NaturalTrot / NaturalSoftTrot)
# ─────────────────────────────────────────────────────────────────────────────

def swing_flourish_hann(leg: str, phase: float, stance_ratio: float,
                         thigh_swing_front_deg: float,
                         thigh_swing_rear_deg: float) -> float:
    """NaturalTrot: 摆动相大腿 Hann 窗偏置(URDF rad), 支撑相为零。"""
    if phase < stance_ratio:
        return 0.0
    swing_t = (phase - stance_ratio) / (1.0 - stance_ratio)
    gate = 0.5 * (1.0 - math.cos(2.0 * math.pi * swing_t))
    if leg.startswith("f"):
        return math.radians(thigh_swing_front_deg) * gate
    return -math.radians(thigh_swing_rear_deg) * gate


def swing_flourish_mj(leg: str, phase: float, stance_ratio: float,
                       thigh_swing_front_deg: float,
                       thigh_swing_rear_deg: float,
                       peak: float = 0.45) -> float:
    """NaturalSoftTrot: 摆动相大腿 minimum-jerk 钟形偏置(URDF rad)。"""
    if phase < stance_ratio:
        return 0.0
    swing_t = (phase - stance_ratio) / (1.0 - stance_ratio)
    gate = minimum_jerk_bump(swing_t, peak)
    if leg.startswith("f"):
        return math.radians(thigh_swing_front_deg) * gate
    return -math.radians(thigh_swing_rear_deg) * gate


def tarsus_swing_delta_hann(leg: str, phase: float, stance_ratio: float,
                             tarsus_swing_deg: float) -> float:
    """NaturalTrot: 前腿跗关节摆动相收放 (URDF rad 增量), 仅前腿。

    摆动前70%: 跗关节微收(脚尖抬起, "翻爪"); 后30%: 伸展回零。
    """
    if not leg.startswith("f"):
        return 0.0
    if phase < stance_ratio:
        return 0.0
    swing_t = (phase - stance_ratio) / (1.0 - stance_ratio)
    if swing_t < 0.7:
        gate = 0.5 * (1.0 - math.cos(math.pi * swing_t / 0.7))
        return math.radians(tarsus_swing_deg) * gate
    else:
        release_t = (swing_t - 0.7) / 0.3
        s = math.cos(math.pi * release_t / 2.0)
        return math.radians(tarsus_swing_deg) * s * s


def tarsus_swing_delta_mj(leg: str, phase: float, stance_ratio: float,
                           tarsus_swing_deg: float, peak: float = 0.42) -> float:
    """NaturalSoftTrot: 前腿跗关节摆动相收放, minimum-jerk 钟形。"""
    if not leg.startswith("f"):
        return 0.0
    if phase < stance_ratio:
        return 0.0
    swing_t = (phase - stance_ratio) / (1.0 - stance_ratio)
    return math.radians(tarsus_swing_deg) * minimum_jerk_bump(swing_t, peak)


# ─────────────────────────────────────────────────────────────────────────────
# 各步态类的足端 X 轨迹形状 (不含 Z —— Z 由调用方按各自的 ``self._swing_z`` 虚函数决定)
#
# 返回 (x, is_swing, u): u 是该分支内的归一化进度(stance_t 或 swing_t),
# 调用方据此决定 lift: 支撑相走 anti-roll 公式, 摆动相调 self._swing_z(u, sh)。
# ─────────────────────────────────────────────────────────────────────────────

def stable_trot_x(phase: float, amp: float, cx: float, stance_ratio: float,
                   smooth_gait: bool) -> tuple:
    """StableTrot: 支撑相余弦/匀速 + 摆动相余弦/Hermite (由 smooth_gait 二选一)。"""
    if phase < stance_ratio:
        stance_t = phase / stance_ratio
        if smooth_gait:
            x = cx + amp * (1.0 - 2.0 * stance_t)
        else:
            x = cx + amp * math.cos(math.pi * stance_t)
        return x, False, stance_t
    else:
        swing_t = (phase - stance_ratio) / (1.0 - stance_ratio)
        if smooth_gait:
            sr = stance_ratio
            m = -2.0 * amp * (1.0 - sr) / sr
            u = swing_t
            u2 = u * u
            u3 = u2 * u
            h00 = 2.0 * u3 - 3.0 * u2 + 1.0
            h10 = u3 - 2.0 * u2 + u
            h01 = -2.0 * u3 + 3.0 * u2
            h11 = u3 - u2
            x = cx + h00 * (-amp) + h10 * m + h01 * amp + h11 * m
        else:
            x = cx - amp * math.cos(math.pi * swing_t)
        return x, True, swing_t


def natural_trot_x(phase: float, amp: float, cx: float, stance_ratio: float,
                    retract: float) -> tuple:
    """NaturalTrot: 支撑相匀速 + 摆动相 Hermite - 回缩弧线(Hann 窗)。"""
    sr = stance_ratio
    if phase < sr:
        stance_t = phase / sr
        x = cx + amp * (1.0 - 2.0 * stance_t)
        return x, False, stance_t
    else:
        swing_t = (phase - sr) / (1.0 - sr)
        m = -2.0 * amp * (1.0 - sr) / sr
        u = swing_t
        u2 = u * u
        u3 = u2 * u
        h00 = 2.0 * u3 - 3.0 * u2 + 1.0
        h10 = u3 - 2.0 * u2 + u
        h01 = -2.0 * u3 + 3.0 * u2
        h11 = u3 - u2
        x = cx + h00 * (-amp) + h10 * m + h01 * amp + h11 * m
        retract_envelope = 0.5 * (1.0 - math.cos(2.0 * math.pi * swing_t))
        x -= retract * retract_envelope
        return x, True, swing_t


def natural_soft_trot_x(phase: float, amp: float, cx: float, stance_ratio: float,
                         retract: float, retract_peak: float) -> tuple:
    """NaturalSoftTrot: 支撑相/摆动相均 minimum-jerk, 摆动相额外回缩(mj 钟形)。"""
    sr = stance_ratio
    if phase < sr:
        stance_t = phase / sr
        u = minimum_jerk(stance_t)
        x = cx + amp * (1.0 - 2.0 * u)
        return x, False, stance_t
    else:
        swing_t = (phase - sr) / (1.0 - sr)
        u = minimum_jerk(swing_t)
        x = cx - amp + 2.0 * amp * u
        x -= retract * minimum_jerk_bump(swing_t, retract_peak)
        return x, True, swing_t


def natural_soft_trot_stance_lift(stance_t: float, anti_roll: float,
                                   anti_roll_soft_scale: float, diag_scale: float,
                                   touchdown_compress: float,
                                   toeoff_lift: float) -> float:
    """NaturalSoftTrot 支撑相 Z: 触地缓冲 + 离地缓冲 + 软化 anti-roll。"""
    td = minimum_jerk_bump(min(stance_t / 0.24, 1.0), 0.45) if stance_t < 0.24 else 0.0
    toe = (
        minimum_jerk_bump((stance_t - 0.82) / 0.18, 0.55)
        if stance_t > 0.82 else 0.0
    )
    support = math.sin(math.pi * stance_t)
    return (
        touchdown_compress * td
        + toeoff_lift * toe
        - anti_roll * anti_roll_soft_scale * diag_scale * support * support
    )
