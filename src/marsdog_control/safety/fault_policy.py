"""电机故障分级策略 — bring-up 阶段"缺了哪个电机还能不能站/走"的唯一裁决点。

动机 (架构复盘遗留项 P1「故障分级策略」):
  重构前后 `walk_bringup.bringup_motors_and_board` 只有一道二元闸门——
  ``if not online: abort``——只要有 ≥1 个电机在线就直接进 `fade_to_stand`。
  真机验证时头部两个电机(head_pitch/head_yaw)离线, 站立/行走完全不受影响,
  说明"缺电机"这件事本身有轻重之分, 不该是非黑即白的开关：

    - 缺腿部承重电机(hip/thigh/calf/前腿tarsus) → 那条腿会瘫/顶不住体重,
      站立瞬间大概率是硬摔而不是"跛着走", 应该直接拒绝进入 fade_to_stand。
    - 缺头/颈/腰电机 → 影响表情/转向手感, 不影响四足承重, 可降级继续。

本模块只做纯分类判断(哪些 id 缺, 属于哪一档), 不碰硬件/不做 I/O 决策 ——
决策(abort vs 继续)留给调用方(`runtime/walk_bringup.py`), 保持"策略"和
"执行"分离, 可离线单测。
"""

from __future__ import annotations

import enum
from dataclasses import dataclass, field
from typing import Dict, Iterable, List


# 腿部承重关节: 缺任意一个, 对应腿站立时会顶不住体重或方向失控。
# 后腿 tarsus (rl_tarsus/rr_tarsus, id 22/23) 不在此列: bus="none", 从不接线,
# 从未出现在 `board.online_ids()/missing_ids()` 的候选集合里。
LEG_CRITICAL_JOINT_NAMES: frozenset = frozenset({
    "fl_hip_pitch", "fl_thigh_roll", "fl_calf", "fl_tarsus",
    "fr_hip_pitch", "fr_thigh_roll", "fr_calf", "fr_tarsus",
    "rl_hip", "rl_thigh", "rl_calf",
    "rr_hip", "rr_thigh", "rr_calf",
})


class MotorFaultTier(enum.Enum):
    OK = "ok"                # 无缺失, 或缺失的都是非承重关节
    DEGRADED = "degraded"    # 缺非承重关节(头/颈/腰), 可继续但功能降级
    ABORT = "abort"           # 缺 ≥1 个腿部承重关节, 不允许进入站立/步态


@dataclass
class MotorFaultReport:
    tier: MotorFaultTier
    missing_critical: List[int] = field(default_factory=list)
    missing_noncritical: List[int] = field(default_factory=list)

    @property
    def ok_to_stand(self) -> bool:
        return self.tier is not MotorFaultTier.ABORT

    def describe(self, joint_by_id: Dict[int, object]) -> str:
        def _names(ids: Iterable[int]) -> str:
            return ", ".join(
                f"{mid}({joint_by_id[mid].name})" for mid in ids if mid in joint_by_id
            )

        if self.tier is MotorFaultTier.OK:
            return "[fault] 全部关节在线"
        if self.tier is MotorFaultTier.ABORT:
            return (
                f"[fault][ABORT] 腿部承重电机离线: {_names(self.missing_critical)} "
                "— 站立会因缺失关节顶不住体重/方向失控, 拒绝进入 fade_to_stand"
            )
        return (
            f"[fault][DEGRADED] 非承重电机离线(不影响站立/步态): "
            f"{_names(self.missing_noncritical)}"
        )


def classify_motor_fault(missing_ids: Iterable[int],
                          joint_by_id: Dict[int, object]) -> MotorFaultReport:
    """按 `LEG_CRITICAL_JOINT_NAMES` 把缺失电机分成承重/非承重两档并给出分级。"""
    missing_critical: List[int] = []
    missing_noncritical: List[int] = []
    for mid in missing_ids:
        j = joint_by_id.get(mid)
        name = getattr(j, "name", None)
        if name in LEG_CRITICAL_JOINT_NAMES:
            missing_critical.append(mid)
        else:
            missing_noncritical.append(mid)

    if missing_critical:
        tier = MotorFaultTier.ABORT
    elif missing_noncritical:
        tier = MotorFaultTier.DEGRADED
    else:
        tier = MotorFaultTier.OK
    return MotorFaultReport(
        tier=tier,
        missing_critical=sorted(missing_critical),
        missing_noncritical=sorted(missing_noncritical),
    )


__all__ = [
    "LEG_CRITICAL_JOINT_NAMES",
    "MotorFaultTier",
    "MotorFaultReport",
    "classify_motor_fault",
]
