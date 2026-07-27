"""Lie-down / stand-up session handling outside the normal FSM path."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Optional

from marsdog_control.core.types import RobotMode


@dataclass
class LieDownSessionResult:
    """Outcome of handling one lie-down request edge."""

    handled: bool = False
    break_loop: bool = False
    continue_loop: bool = False
    targets: Optional[dict] = None


@dataclass
class LieDownSession:
    """Owns LT-triggered lie-down hold state and transitions."""

    build_target: Callable
    read_positions: Callable
    smooth_transition: Callable
    dm_fixed_targets: dict
    transition_s: float = 4.0
    hold: bool = False
    targets: dict = field(default_factory=dict)

    def handle_request(self, *, fsm, online, targets_now, lz, evo, dm, incos,
                       board=None, dm_tarsus_active: bool = False,
                       smooth_tgt=None, safety=None) -> LieDownSessionResult:
        if self.hold:
            print("\n[lie-down] LT再次触发: 从趴下姿势缓慢起立 (4.0s)...")
            stand_targets = fsm.stand.get_targets(0)
            stand_targets = {
                mid: q for mid, q in stand_targets.items()
                if mid in online
            }
            cur_stand = self.read_positions(lz, evo, incos)
            if dm is not None and dm_tarsus_active and board is not None:
                cur_stand.update(board.get_angles((4, 8), include_dm=True))
            elif dm is not None:
                cur_stand.update(self.dm_fixed_targets)
            if smooth_tgt is not None:
                smooth_tgt.clear()
            if safety is not None:
                safety.reset()
            if not self.smooth_transition(
                    lz, evo, dm, incos, cur_stand, stand_targets,
                    self.transition_s, label="lie-stand"):
                return LieDownSessionResult(handled=True, break_loop=True)
            self.hold = False
            self.targets = {}
            fsm.request_transition(
                RobotMode.STAND, targets_now=stand_targets,
                blend_time=0.0, quiet=True)
            print("\n[lie-down] 已起立回 STAND")
            return LieDownSessionResult(
                handled=True, continue_loop=True, targets=dict(stand_targets))

        lie_targets = self.build_target(set(online))
        if not lie_targets:
            print("\n[lie-down] 没有可用趴下目标, 忽略")
            return LieDownSessionResult(handled=True)

        print("\n[lie-down] LT触发: 从当前姿态缓慢过渡到趴下姿势 (4.0s)...")
        fsm.request_transition(
            RobotMode.STAND, targets_now=targets_now,
            blend_time=0.0, quiet=True)
        cur_lie = self.read_positions(lz, evo, incos)
        if dm is not None:
            cur_lie.update(self.dm_fixed_targets)
        if smooth_tgt is not None:
            smooth_tgt.clear()
        if safety is not None:
            safety.reset()
        if not self.smooth_transition(
                lz, evo, dm, incos, cur_lie, lie_targets,
                self.transition_s, label="lie-down"):
            return LieDownSessionResult(handled=True, break_loop=True)
        self.targets = dict(lie_targets)
        self.hold = True
        print("\n[lie-down] 已趴下, 保持该姿势；再次按 LT 起立，按 q/ESC 退出并缓速失能")
        return LieDownSessionResult(
            handled=True, continue_loop=True, targets=dict(lie_targets))


__all__ = ["LieDownSession", "LieDownSessionResult"]
