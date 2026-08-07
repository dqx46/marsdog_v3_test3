"""Sit / lie-down / stand-up session handling outside the normal FSM path."""

from __future__ import annotations

import math
import time as _time
from dataclasses import dataclass, field
from typing import Callable, Optional

from marsdog_control.core.types import RobotMode

# pose 名 → 日志标签
_POSE_LABEL = {
    "lie_down": "趴下",
    "sit": "坐下",
    "zero": "回零",
}


@dataclass
class LieDownSessionResult:
    """Outcome of handling one pose-hold request edge."""

    handled: bool = False
    break_loop: bool = False
    continue_loop: bool = False
    targets: Optional[dict] = None


@dataclass
class LieDownSession:
    """Owns keyboard/LT-triggered sit & lie-down hold state and transitions.

    - 同一姿势键再按一次 → 缓慢起立回 STAND
    - 另一姿势键 → 从当前姿势过渡到新姿势并保持

    接缝防抖：
    - 捕获姿势先经 motor→urdf→motor 规范化，与 hold 下发路径同源（含限位钳制），
      避免过渡直发超限角、hold 突然钳回造成最后一帧目标跳变
    - 过渡末端刚度落到 hold 入口软刚度，避免全增益收尾
    - 实测仍滞后时补 actual→target 收敛段
    - hold 切入后 kp / 重力补偿分别缓入
    """

    build_target: Callable
    read_positions: Callable
    smooth_transition: Callable
    dm_fixed_targets: dict
    build_sit_target: Optional[Callable] = None
    build_zero_target: Optional[Callable] = None
    transition_s: float = 4.0
    # 回零默认稍短（对齐 go_zero 工具 ~3s）；趴下/坐下仍用 transition_s
    zero_transition_s: float = 3.0
    # hold 切入后重力补偿从 0 平滑接入，避免与 smooth_transition（无 τ_g）接缝抽搐
    grav_ramp_s: float = 1.5
    # hold 切入后刚度从 kp_start→1；过渡末端也落到同一 kp_start
    kp_ramp_s: float = 1.2
    kp_start: float = 0.40
    transition_kp_start: float = 0.35
    # 过渡结束后实测误差超过阈值则补收敛段
    settle_err_tol_deg: float = 3.5
    settle_s_min: float = 0.7
    settle_s_max: float = 1.8
    settle_s_per_rad: float = 4.0  # 误差越大收敛越长；~16° → ~1.1s
    hold: bool = False
    targets: dict = field(default_factory=dict)
    active_pose: Optional[str] = None  # "lie_down" | "sit" | "zero" | None
    hold_t0: Optional[float] = None

    def _builder_for(self, pose: str) -> Optional[Callable]:
        if pose == "sit":
            return self.build_sit_target
        if pose == "zero":
            return self.build_zero_target
        return self.build_target

    def _transition_s_for(self, pose: str) -> float:
        if pose == "zero":
            return float(self.zero_transition_s)
        return float(self.transition_s)

    def _clear_hold(self) -> None:
        self.hold = False
        self.targets = {}
        self.active_pose = None
        self.hold_t0 = None

    def _enter_hold(self, pose: str, targets_urdf: dict, *, mono: float) -> None:
        self.targets = targets_urdf
        self.hold = True
        self.active_pose = pose
        self.hold_t0 = float(mono)

    @staticmethod
    def _smoothstep01(u: float) -> float:
        u = 0.0 if u < 0.0 else (1.0 if u > 1.0 else u)
        return u * u * (3.0 - 2.0 * u)

    def hold_grav_blend(self, mono: float) -> float:
        """hold 期间重力补偿缩放 ∈[0,1]：刚切入为 0，grav_ramp_s 内 smoothstep→1。"""
        if not self.hold or self.hold_t0 is None:
            return 1.0
        ramp = max(0.0, float(self.grav_ramp_s))
        if ramp <= 1e-9:
            return 0.0
        return self._smoothstep01((float(mono) - self.hold_t0) / ramp)

    def hold_kp_blend(self, mono: float) -> float:
        """hold 期间刚度缩放 ∈[kp_start,1]：刚切入为 kp_start，kp_ramp_s 内→1。"""
        if not self.hold or self.hold_t0 is None:
            return 1.0
        start = min(1.0, max(0.05, float(self.kp_start)))
        ramp = max(0.0, float(self.kp_ramp_s))
        if ramp <= 1e-9:
            return 1.0
        s = self._smoothstep01((float(mono) - self.hold_t0) / ramp)
        return start + (1.0 - start) * s

    @staticmethod
    def _max_tracking_error_rad(cur: dict, target: dict) -> float:
        mx = 0.0
        for mid, q in target.items():
            if mid not in cur:
                continue
            mx = max(mx, abs(float(cur[mid]) - float(q)))
        return mx

    @staticmethod
    def _canonicalize_motor_pose(motor_pose: dict, *, label: str) -> tuple[dict, dict]:
        """使过渡终点与 hold 下发同源：motor→urdf→motor（含限位钳制）。

        Returns ``(motor_for_transition, urdf_for_hold)``.
        """
        from marsdog_control.backends.real import (
            JOINT_BY_ID,
            motor_pose_to_urdf,
            urdf_pose_to_motor,
        )

        urdf = motor_pose_to_urdf(motor_pose)
        motor = urdf_pose_to_motor(urdf)
        jumps = []
        for mid, q0 in motor_pose.items():
            q1 = motor.get(mid)
            if q1 is None:
                continue
            d_deg = math.degrees(float(q1) - float(q0))
            if abs(d_deg) >= 1.0:
                name = JOINT_BY_ID[mid].name if mid in JOINT_BY_ID else str(mid)
                jumps.append((abs(d_deg), mid, name, math.degrees(float(q0)),
                              math.degrees(float(q1))))
        if jumps:
            jumps.sort(reverse=True)
            parts = [
                f"{name} {a:+.1f}°→{b:+.1f}°"
                for _, _mid, name, a, b in jumps[:6]
            ]
            print(
                f"\n[{label}] 姿势已按 hold 下发路径规范化"
                f"（限位钳制 {len(jumps)} 轴）: " + ", ".join(parts)
            )
        return motor, urdf

    def _pose_transition(
            self, lz, evo, dm, incos, from_pos, to_pos, duration, label: str) -> bool:
        """姿势过渡：末端刚度落到 hold 入口，避免最后一帧全增益。"""
        kwargs = dict(
            label=label,
            kp_start=float(self.transition_kp_start),
            kp_end=float(self.kp_start),
        )
        try:
            return bool(self.smooth_transition(
                lz, evo, dm, incos, from_pos, to_pos, duration, **kwargs))
        except TypeError:
            # 旧签名无 kp_* 时回退
            return bool(self.smooth_transition(
                lz, evo, dm, incos, from_pos, to_pos, duration, label=label))

    @staticmethod
    def _merge_dm_into(cur: dict, *, dm, board, dm_tarsus_active: bool,
                       dm_fixed_targets: dict) -> None:
        """过渡起点的达妙角：主动跟踪时读实测，否则用开机固定角。"""
        if dm is None:
            return
        if dm_tarsus_active and board is not None:
            cur.update(board.get_angles((4, 8), include_dm=True))
        else:
            cur.update(dm_fixed_targets)

    def _close_tracking_gap(
            self, *, lz, evo, dm, incos, board, dm_tarsus_active: bool,
            target_motor: dict, label: str) -> bool:
        """指令插值后若实测仍滞后，从 actual 再平滑补一段到 target。

        Returns False if the transition was aborted (stop requested).
        """
        cur = self.read_positions(lz, evo, incos)
        self._merge_dm_into(
            cur, dm=dm, board=board,
            dm_tarsus_active=dm_tarsus_active,
            dm_fixed_targets=self.dm_fixed_targets)
        from_pos = {
            mid: float(cur[mid]) if mid in cur else float(q)
            for mid, q in target_motor.items()
        }
        max_err = self._max_tracking_error_rad(from_pos, target_motor)
        tol = math.radians(float(self.settle_err_tol_deg))
        if max_err <= tol:
            return True
        per = max(0.0, float(self.settle_s_per_rad))
        dur = max_err * per if per > 1e-9 else float(self.settle_s_min)
        dur = min(float(self.settle_s_max), max(float(self.settle_s_min), dur))
        print(
            f"\n[{label}] 实测未到位 max={math.degrees(max_err):.1f}° "
            f"(阈值{float(self.settle_err_tol_deg):.1f}°), "
            f"补收敛过渡 {dur:.1f}s..."
        )
        return self._pose_transition(
            lz, evo, dm, incos, from_pos, target_motor, dur, f"{label}-settle")

    def handle_request(self, *, fsm, online, targets_now, lz, evo, dm, incos,
                       board=None, dm_tarsus_active: bool = False,
                       smooth_tgt=None, safety=None,
                       pose: str = "lie_down",
                       mono: Optional[float] = None) -> LieDownSessionResult:
        if pose not in _POSE_LABEL:
            pose = "lie_down"
        label_cn = _POSE_LABEL[pose]

        dur = self._transition_s_for(pose)

        # 同一姿势再触发 → 起立
        if self.hold and self.active_pose == pose:
            print(f"\n[{pose}] 再次触发: 从{label_cn}姿势缓慢起立 "
                  f"({dur:.1f}s)...")
            stand_targets = fsm.stand.get_targets(0)          # URDF 空间(供 fsm/主循环)
            stand_targets = {
                mid: q for mid, q in stand_targets.items()
                if mid in online
            }
            # 起立过渡是直连 send_all(电机空间), cur_stand 来自 get_angles/read_positions
            # 也是电机空间; 站姿目标是 URDF, 必须经唯一真源映射转到电机空间, 否则
            # sign=-1 的左侧关节会反向(与 fade/shutdown 同一个坐标系错配 bug)。
            from marsdog_control.backends.real import urdf_pose_to_motor
            stand_targets_motor = urdf_pose_to_motor(stand_targets)
            cur_stand = self.read_positions(lz, evo, incos)
            self._merge_dm_into(
                cur_stand, dm=dm, board=board,
                dm_tarsus_active=dm_tarsus_active,
                dm_fixed_targets=self.dm_fixed_targets)
            if smooth_tgt is not None:
                smooth_tgt.clear()
            if safety is not None:
                safety.reset()
            if not self._pose_transition(
                    lz, evo, dm, incos, cur_stand, stand_targets_motor,
                    dur, f"{pose}-stand"):
                return LieDownSessionResult(handled=True, break_loop=True)
            if not self._close_tracking_gap(
                    lz=lz, evo=evo, dm=dm, incos=incos, board=board,
                    dm_tarsus_active=dm_tarsus_active,
                    target_motor=stand_targets_motor,
                    label=f"{pose}-stand"):
                return LieDownSessionResult(handled=True, break_loop=True)
            self._clear_hold()
            fsm.request_transition(
                RobotMode.STAND, targets_now=stand_targets,
                blend_time=0.0, quiet=True)
            print(f"\n[{pose}] 已起立回 STAND")
            return LieDownSessionResult(
                handled=True, continue_loop=True, targets=dict(stand_targets))

        builder = self._builder_for(pose)
        if builder is None:
            print(f"\n[{pose}] 未配置{label_cn}目标生成器, 忽略")
            return LieDownSessionResult(handled=True)

        pose_targets = builder(set(online))
        if not pose_targets:
            hint = {
                "sit": "sit_pose.json",
                "lie_down": "lie_down_pose.json",
            }.get(pose, "")
            extra = f"（检查 mocap_to_real/{hint}）" if hint else ""
            print(f"\n[{pose}] 没有可用{label_cn}目标{extra}, 忽略")
            return LieDownSessionResult(handled=True)

        # 过渡与 hold 必须同一终点：捕获角先走 hold 下发路径（含限位）
        pose_targets, pose_targets_urdf = self._canonicalize_motor_pose(
            pose_targets, label=pose)

        from_hint = ""
        if self.hold and self.active_pose and self.active_pose != pose:
            from_hint = f"（当前{_POSE_LABEL.get(self.active_pose, self.active_pose)}）"
        print(f"\n[{pose}] 触发: 从当前姿态{from_hint}缓慢过渡到{label_cn}姿势 "
              f"({dur:.1f}s, 末端刚度{float(self.kp_start):.0%})...")
        fsm.request_transition(
            RobotMode.STAND, targets_now=targets_now,
            blend_time=0.0, quiet=True)
        cur_pose = self.read_positions(lz, evo, incos)
        self._merge_dm_into(
            cur_pose, dm=dm, board=board,
            dm_tarsus_active=dm_tarsus_active,
            dm_fixed_targets=self.dm_fixed_targets)
        if smooth_tgt is not None:
            smooth_tgt.clear()
        if safety is not None:
            safety.reset()
        if not self._pose_transition(
                lz, evo, dm, incos, cur_pose, pose_targets,
                dur, pose):
            return LieDownSessionResult(handled=True, break_loop=True)
        if not self._close_tracking_gap(
                lz=lz, evo=evo, dm=dm, incos=incos, board=board,
                dm_tarsus_active=dm_tarsus_active,
                target_motor=pose_targets,
                label=pose):
            return LieDownSessionResult(handled=True, break_loop=True)
        # 回零时同步达妙固定角，避免 hold 期间固定路径把脚尖拉回旧角
        if pose == "zero":
            for mid in (4, 8):
                if mid in pose_targets:
                    self.dm_fixed_targets[mid] = float(pose_targets[mid])
        self._enter_hold(
            pose, pose_targets_urdf,
            mono=float(mono if mono is not None else _time.monotonic()))
        print(f"\n[{pose}] 已{label_cn}, 保持该姿势；再按同键起立，"
              f"另一姿势键可切换；q/ESC 退出并缓速失能")
        print(
            f"\n[{pose}] 接缝缓入: 刚度 {float(self.kp_start):.0%}→100%/"
            f"{float(self.kp_ramp_s):.1f}s, 重力 0→100%/{float(self.grav_ramp_s):.1f}s"
        )
        return LieDownSessionResult(
            handled=True, continue_loop=True, targets=dict(pose_targets_urdf))


__all__ = ["LieDownSession", "LieDownSessionResult"]
