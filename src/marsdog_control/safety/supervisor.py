"""安全监督层 — 永远最后兜底。

不管上层是位置步态、VMC、WBC 还是 MPC, 最终的 MotionTarget 都必须过这里再下发。
本层只做"事实层面的安全钳制", 不做步态平滑(细粒度抗 stick-slip 限速仍在 motion 层)。

检查项:
  1. 关节硬限位钳制 (JOINT_BY_ID 的 limit_lo/hi) — 当前系统没有单一最终限位, 这是新增兜底。
  2. 单周期粗跳变钳制 — |目标 - 实测| 超过 max_delta 的截断, 只抓 IK/读数毛刺, 不当平滑器。
  3. 姿态摔倒守卫 — |roll|/|pitch| 过大 -> 要求上层进入 ESTOP。
  4. IMU 时效降级 — IMU 数据过期 -> 报告降级, 上层据此停用姿态反馈。

纯逻辑, 不碰硬件, 可离线单测。
"""

from __future__ import annotations

# [解耦] 真实实现已从 mocap_to_real 下沉到此 src 模块; 保持逐字一致的扁平 import,
# 由 ensure_legacy_path() 保证 mocap_to_real 在 sys.path 上可解析(其 compat 别名回指
# 本 src 包, 单一模块实体)。
from marsdog_control.compat import ensure_legacy_path as _ensure_legacy_path
_ensure_legacy_path()

import math

from marsdog_control.config.joints import JOINT_BY_ID
from marsdog_control.core.types import MotionTarget, RobotState, SafetyReport
from marsdog_control.motion.kinematics import urdf_limits


class SafetySupervisor:
    def __init__(self, *, fall_guard_deg: float = 45.0,
                 max_delta_rad: float = math.radians(20.0),
                 imu_max_age_s: float = 0.3,
                 require_imu: bool = False):
        # fall_guard_deg: roll/pitch 超过即要求 ESTOP(比 imu_controller 内部的姿态限更外层)
        self.fall_guard_rad = math.radians(fall_guard_deg)
        # max_delta_rad: 单周期允许的最大目标跳变。注意: 相对"上一周期的输出指令"而不是
        #   实测位置 —— 若相对实测, 一次坏的/滞后的传感器读数就能把腿从步态轨迹上硬拽走,
        #   反而制造危险。相对上次输出是纯输出限速(毛刺守卫), 与传感器好坏解耦。
        #   必须远大于步态单周期合法幅度(200Hz 下即使 100deg/s 也只有 0.5deg/周期)。
        self.max_delta_rad = max_delta_rad
        self.imu_max_age_s = imu_max_age_s
        # require_imu: 若 True 且 IMU 过期, 视为通信丢失 -> ESTOP; 否则只降级(停姿态反馈)。
        self.require_imu = require_imu
        self._prev_q: dict[int, float] = {}   # 上一周期实际下发的安全目标

    def reset(self):
        """在进入主循环/大过渡后调用, 清掉上次输出记忆, 避免首周期误限。"""
        self._prev_q.clear()

    def filter(self, state: RobotState,
               target: MotionTarget) -> tuple[MotionTarget, SafetyReport]:
        report = SafetyReport()
        safe_q = dict(target.q)

        for mid, q in target.q.items():
            j = JOINT_BY_ID.get(mid)
            if j is None:
                continue
            clamped = False

            # 1) 单周期粗跳变钳制(相对上次输出指令, 与传感器解耦), 首周期不限
            prev = self._prev_q.get(mid)
            if prev is not None:
                delta = q - prev
                if delta > self.max_delta_rad:
                    q = prev + self.max_delta_rad
                    clamped = True
                elif delta < -self.max_delta_rad:
                    q = prev - self.max_delta_rad
                    clamped = True

            # 2) 关节硬限位钳制(最终兜底，URDF空间)
            u_lo, u_hi = urdf_limits(j)
            if q < u_lo:
                q = u_lo
                clamped = True
            elif q > u_hi:
                q = u_hi
                clamped = True

            safe_q[mid] = q
            self._prev_q[mid] = q
            if clamped:
                report.clamped_ids.append(mid)

        # 3) 姿态摔倒守卫
        if state.imu_connected and _fresh(state, self.imu_max_age_s):
            if (abs(state.roll) > self.fall_guard_rad
                    or abs(state.pitch) > self.fall_guard_rad):
                report.triggered_estop = True
                report.reason = (f"姿态越界 roll={math.degrees(state.roll):.1f}° "
                                 f"pitch={math.degrees(state.pitch):.1f}° "
                                 f"> {math.degrees(self.fall_guard_rad):.0f}°")

        # 4) IMU 时效降级 / 通信丢失
        if state.imu_connected and not _fresh(state, self.imu_max_age_s):
            report.imu_degraded = True
            if self.require_imu:
                report.triggered_estop = True
                report.reason = (f"IMU 数据过期 {state.imu_age_s*1000:.0f}ms "
                                 f"> {self.imu_max_age_s*1000:.0f}ms (通信丢失)")

        report.ok = (not report.clamped_ids
                     and not report.triggered_estop
                     and not report.imu_degraded)

        safe_target = MotionTarget(q=safe_q, dq=dict(target.dq),
                                   source_mode=target.source_mode)
        return safe_target, report


def _fresh(state: RobotState, max_age_s: float) -> bool:
    return state.imu_age_s <= max_age_s
