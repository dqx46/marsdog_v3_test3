"""Sit / lie-down / stand-up session handling outside the normal FSM path."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Optional

from marsdog_control.core.types import RobotMode

# pose 名 → 日志标签
_POSE_LABEL = {
    "lie_down": "趴下",
    "sit": "坐下",
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
    """

    build_target: Callable
    read_positions: Callable
    smooth_transition: Callable
    dm_fixed_targets: dict
    build_sit_target: Optional[Callable] = None
    transition_s: float = 4.0
    # hold 切入后重力补偿从 0 平滑接入，避免与 smooth_transition（无 τ_g）接缝抽搐
    grav_ramp_s: float = 1.5
    hold: bool = False
    targets: dict = field(default_factory=dict)
    active_pose: Optional[str] = None  # "lie_down" | "sit" | None
    hold_t0: Optional[float] = None

    def _builder_for(self, pose: str) -> Optional[Callable]:
        if pose == "sit":
            return self.build_sit_target
        return self.build_target

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

    def hold_grav_blend(self, mono: float) -> float:
        """hold 期间重力补偿缩放 ∈[0,1]：刚切入为 0，grav_ramp_s 内 smoothstep→1。"""
        if not self.hold or self.hold_t0 is None:
            return 1.0
        ramp = max(0.0, float(self.grav_ramp_s))
        if ramp <= 1e-9:
            return 0.0
        u = (float(mono) - self.hold_t0) / ramp
        u = 0.0 if u < 0.0 else (1.0 if u > 1.0 else u)
        return u * u * (3.0 - 2.0 * u)

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

    def handle_request(self, *, fsm, online, targets_now, lz, evo, dm, incos,
                       board=None, dm_tarsus_active: bool = False,
                       smooth_tgt=None, safety=None,
                       pose: str = "lie_down",
                       mono: Optional[float] = None) -> LieDownSessionResult:
        if pose not in _POSE_LABEL:
            pose = "lie_down"
        label_cn = _POSE_LABEL[pose]

        # 同一姿势再触发 → 起立
        if self.hold and self.active_pose == pose:
            print(f"\n[{pose}] 再次触发: 从{label_cn}姿势缓慢起立 "
                  f"({self.transition_s:.1f}s)...")
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
            if not self.smooth_transition(
                    lz, evo, dm, incos, cur_stand, stand_targets_motor,
                    self.transition_s, label=f"{pose}-stand"):
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
            print(f"\n[{pose}] 没有可用{label_cn}目标"
                  f"（检查 mocap_to_real/{'sit' if pose == 'sit' else 'lie_down'}_pose.json）, 忽略")
            return LieDownSessionResult(handled=True)

        from_hint = ""
        if self.hold and self.active_pose and self.active_pose != pose:
            from_hint = f"（当前{_POSE_LABEL.get(self.active_pose, self.active_pose)}）"
        print(f"\n[{pose}] 触发: 从当前姿态{from_hint}缓慢过渡到{label_cn}姿势 "
              f"({self.transition_s:.1f}s)...")
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
        if not self.smooth_transition(
                lz, evo, dm, incos, cur_pose, pose_targets,
                self.transition_s, label=pose):
            return LieDownSessionResult(handled=True, break_loop=True)
        # 直连过渡用电机帧(捕获姿势与 cur 同域)。主循环 hold 时 targets 经
        # backend.send(×sign×gear) 下发, 故存 URDF 帧供循环消费。
        from marsdog_control.backends.real import motor_pose_to_urdf
        import time as _time
        pose_targets_urdf = motor_pose_to_urdf(pose_targets)
        self._enter_hold(
            pose, pose_targets_urdf,
            mono=float(mono if mono is not None else _time.monotonic()))
        print(f"\n[{pose}] 已{label_cn}, 保持该姿势；再按同键起立，"
              f"另一姿势键可切换；q/ESC 退出并缓速失能")
        print(f"\n[{pose}] 重力补偿将在 {self.grav_ramp_s:.1f}s 内缓入"
              f"（消除过渡→保持接缝抽搐）")
        return LieDownSessionResult(
            handled=True, continue_loop=True, targets=dict(pose_targets_urdf))


__all__ = ["LieDownSession", "LieDownSessionResult"]
