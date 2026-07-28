"""显式运行时状态机 — 唯一的模式转移入口。

重构前: mode 是个字符串, 在手柄/键盘/清理各处被直接赋值, 没有单一入口也没有守卫。
重构后: 所有 mode 变更必须走 RuntimeStateMachine.request_transition(), 非法转移被
拒绝并打印原因(绝不静默改 mode)。手柄/键盘只产出 UserCommand, 由 FSM.update 翻译。

模式收敛: trot_fwd/trot_bwd/pace_fwd/pace_bwd 压成 TROT/PACE + Direction 字段;
natural_trot 归为 NATURAL(具体 NaturalTrot / NaturalSoftTrot 由启动时构造的 nat_fwd
控制器承载)。

本模块只依赖 robot_types 和被注入的控制器对象, 不碰硬件, 可离线单测。
"""

from __future__ import annotations

# [解耦] 真实实现已从 mocap_to_real 下沉到此 src 模块; 保持逐字一致的扁平 import,
# 由 ensure_legacy_path() 保证 mocap_to_real 在 sys.path 上可解析(其 compat 别名回指
# 本 src 包, 单一模块实体)。
from marsdog_control.compat import ensure_legacy_path as _ensure_legacy_path
_ensure_legacy_path()

import math
import time
from typing import Dict, Optional, Set

from marsdog_control.config.stack_build import FsmDriveConfig
from marsdog_control.core.types import Direction, RobotMode, RobotState, UserCommand
from marsdog_control.motion.gait_schedule import (
    SoftTrotSchedule,
    VelocityCommand,
    GaitEnvelope,
    apply_schedule_to_gait,
)


# 合法转移表: from_mode -> 允许直达的 to_mode 集合。ESTOP 从任意态恒可达(单独处理)。
_LEGAL: Dict[RobotMode, Set[RobotMode]] = {
    RobotMode.BOOT:     {RobotMode.STAND},
    RobotMode.STAND:    {RobotMode.TROT, RobotMode.PACE, RobotMode.ZEROING,
                         RobotMode.NATURAL, RobotMode.SHUTDOWN},
    RobotMode.ZEROING:  {RobotMode.NATURAL, RobotMode.STAND},
    RobotMode.TROT:     {RobotMode.STAND, RobotMode.PACE, RobotMode.TROT},   # 自转移=换方向
    RobotMode.PACE:     {RobotMode.STAND, RobotMode.TROT, RobotMode.PACE},
    RobotMode.NATURAL:  {RobotMode.STAND},
    RobotMode.ESTOP:    {RobotMode.SHUTDOWN},
    RobotMode.SHUTDOWN: set(),
}


def _wrap_rad(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


class RuntimeStateMachine:
    """持有控制器组 + 当前模式 + blend 状态, 是运行期唯一改 mode 的地方。"""

    def __init__(self, controllers, drive: FsmDriveConfig, *,
                 height: float, fwd_amp_front: float, fwd_amp_rear: float,
                 natural_configured: bool = False,
                 start_mode: RobotMode = RobotMode.STAND,
                 clock=None):
        # controllers: gait_recipes.ControllerSet (stand/fwd/bwd/pace_fwd/pace_bwd/nat_fwd)
        (self.stand, self.trot_fwd, self.trot_bwd,
         self.pace_fwd, self.pace_bwd, self.nat_fwd) = controllers.as_tuple()
        self.drive = drive
        self.clock = clock or time
        # natural_configured: 启动时用 --natural-soft-trot/--natural-trot 构造了 nat_fwd
        # (达妙零点由上电前手动归零约定保证, 自然步态即主动驱动 tarsus)。
        self.natural_configured = bool(natural_configured)
        self.height = height
        self.fwd_amp_front = fwd_amp_front
        self.fwd_amp_rear = fwd_amp_rear
        # 摇杆/空格 "走": 配了自然步态 -> NaturalSoftTrot(NATURAL), 否则 StableTrot。
        self.walk_is_natural = self.natural_configured
        self.walk_mode = RobotMode.NATURAL if self.walk_is_natural else RobotMode.TROT
        # nat_fwd 的满幅(=预设/仿真值), 摇杆油门按比例线性缩放到这个上限。
        self.nat_amp_front = getattr(self.nat_fwd, "amp_front", fwd_amp_front)
        self.nat_amp_rear = getattr(self.nat_fwd, "amp_rear", fwd_amp_rear)
        self._nat_schedule = SoftTrotSchedule(
            GaitEnvelope.from_wbc_soft_trot(
                amp_front=self.nat_amp_front,
                amp_rear=self.nat_amp_rear,
                period=float(getattr(self.nat_fwd, "period", 1.05)),
                stance=float(getattr(self.nat_fwd, "stance_ratio", 0.80)),
                step_h_front=float(
                    getattr(self.nat_fwd, "step_height_front", 0.018)
                ),
                step_h_rear=float(getattr(self.nat_fwd, "step_height", 0.018)),
                throttle_min_scale=float(drive.throttle_min_scale),
                cruise_turn_scale=float(drive.cruise_turn_scale),
                cruise_turn_yamp=float(drive.cruise_turn_yamp),
                vx_engage=float(drive.gp_trot_threshold),
                vx_deadzone=float(drive.gp_deadzone),
                turn_y_amp=float(getattr(self.nat_fwd, "max_turn_y_amp", 0.025)),
                turn_amp_diff=float(
                    getattr(self.nat_fwd, "max_turn_amp_diff", 0.020)
                ),
            )
        )
        self._trot_schedule = SoftTrotSchedule(
            GaitEnvelope.from_wbc_soft_trot(
                amp_front=fwd_amp_front,
                amp_rear=fwd_amp_rear,
                period=float(getattr(self.trot_fwd, "period", 0.9)),
                stance=float(getattr(self.trot_fwd, "stance_ratio", 0.65)),
                step_h_front=float(
                    getattr(self.trot_fwd, "step_height_front", 0.018)
                ),
                step_h_rear=float(getattr(self.trot_fwd, "step_height", 0.018)),
                throttle_min_scale=float(drive.throttle_min_scale),
                cruise_turn_scale=float(drive.cruise_turn_scale),
                cruise_turn_yamp=float(drive.cruise_turn_yamp),
                vx_engage=float(drive.gp_trot_threshold),
                vx_deadzone=float(drive.gp_deadzone),
                turn_y_amp=float(getattr(self.trot_fwd, "max_turn_y_amp", 0.025)),
                turn_amp_diff=float(
                    getattr(self.trot_fwd, "max_turn_amp_diff", 0.020)
                ),
            )
        )

        self.mode = RobotMode.BOOT
        self.direction = Direction.FWD
        self.active_gait = None          # None=站立/estop; 否则=正在跑的步态控制器
        self.throttle = 0.0

        # blend(步态/站立切换位置混合)状态, 由 motion 层读取消费
        self.t_gait = self.clock.time()
        self.blend_active = False
        self.blend_from: Dict[int, float] = {}
        self.blend_start = 0.0
        self.blend_dur = 0.5
        self.just_switched = False       # 本周期是否刚切换(motion 层据此清平滑缓冲)

        self._yaw_target: Optional[float] = None

        # BOOT 只合法直达 STAND; 先落到 STAND, 再按需转到 start_mode(TROT/NATURAL)。
        self.request_transition(RobotMode.STAND, Direction.FWD,
                                targets_now=None, blend_time=0.0, quiet=True)
        if start_mode not in (RobotMode.BOOT, RobotMode.STAND):
            self.request_transition(start_mode, Direction.FWD, targets_now=None,
                                    blend_time=0.0, quiet=True)

    # ── 查询 ─────────────────────────────────────────────────────────────
    @property
    def active_controller(self):
        return self.active_gait if self.active_gait is not None else self.stand

    def is_gait_mode(self) -> bool:
        return self.active_gait is not None

    def dm_active(self) -> bool:
        """达妙 tarsus 是否应主动驱动。

        只要配了自然步态, tarsus 全程主动驱动 —— STAND 时到新站姿角度, NATURAL
        时跟踪步态。零点由上电前手动归零约定保证。
        """
        return self.walk_is_natural

    def consume_just_switched(self) -> bool:
        v = self.just_switched
        self.just_switched = False
        return v

    # ── 共享步态参数(dev 调参旁路调用, FSM 是控制器组的持有者) ──────────────
    def set_height(self, h: float):
        self.height = h
        for c in (self.stand, self.trot_fwd, self.trot_bwd,
                  self.pace_fwd, self.pace_bwd, self.nat_fwd):
            c.set_height(h)

    def set_period(self, p: float):
        for c in (self.trot_fwd, self.trot_bwd,
                  self.pace_fwd, self.pace_bwd, self.nat_fwd):
            c.set_period(p)

    def adjust_fwd_amp(self, delta: float, lo: float = 0.005, hi: float = 0.06):
        self.trot_fwd.amp_front = max(lo, min(hi, self.trot_fwd.amp_front + delta))
        self.trot_fwd.amp_rear = max(lo, min(hi, self.trot_fwd.amp_rear + delta))

    # ── 唯一转移入口 ──────────────────────────────────────────────────────
    def request_transition(self, target: RobotMode,
                           direction: Optional[Direction] = None,
                           *, targets_now: Optional[Dict[int, float]] = None,
                           blend_time: float = 0.6, quiet: bool = False) -> bool:
        """请求切到 target 模式。合法且过守卫才执行, 否则拒绝并返回 False。"""
        # ESTOP 从任意非终态恒可达
        if target is RobotMode.ESTOP:
            return self._commit(target, direction, targets_now, blend_time)

        allowed = _LEGAL.get(self.mode, set())
        if target not in allowed and target is not self.mode:
            if not quiet:
                print(f"\n[fsm] 拒绝转移 {self.mode.value} -> {target.value} (非法)")
            return False

        return self._commit(target, direction, targets_now, blend_time)

    def _commit(self, target: RobotMode, direction: Optional[Direction],
                targets_now: Optional[Dict[int, float]], blend_time: float) -> bool:
        if direction is not None:
            self.direction = direction

        # 选定 active_gait
        if target is RobotMode.STAND or target is RobotMode.ESTOP:
            self.active_gait = None
        elif target is RobotMode.TROT:
            self.active_gait = self.trot_fwd if self.direction is Direction.FWD else self.trot_bwd
        elif target is RobotMode.PACE:
            self.active_gait = self.pace_fwd if self.direction is Direction.FWD else self.pace_bwd
        elif target is RobotMode.NATURAL:
            self.active_gait = self.nat_fwd
        # ZEROING/SHUTDOWN: 不改 active_gait(ZEROING 本轮只做守卫态)

        # 启动位置混合(复用旧 _start_trot / 站立回退的语义)
        if self.active_gait is not None:
            self.t_gait = self.clock.time()
            self.active_gait.set_height(self.height)
            self.active_gait._reactive_filtered = 0.0
            self.just_switched = True    # motion 层据此清 _smooth_tgt
        else:
            self.stand.set_height(self.height)

        if targets_now and blend_time > 1e-6:
            self.blend_from = dict(targets_now)
            self.blend_start = self.clock.time()
            self.blend_dur = blend_time
            self.blend_active = True

        self.mode = target
        self._yaw_target = None
        return True

    # ── 每周期: 把 UserCommand 翻译成转移 + 施加油门/转向 ────────────────────
    def update(self, state: RobotState, cmd: UserCommand,
               last_targets: Optional[Dict[int, float]]):
        """运行期唯一改 mode 的地方。先处理离散请求, 再跑摇杆连续驱动策略。"""
        # 1) 离散模式请求(键盘 space/'3'、手柄 START)
        if cmd.request_mode is not None:
            self._handle_request_mode(cmd, last_targets)

        # 2) 摇杆连续驱动策略(仅手柄在线时; 键盘-only 不做"松杆归站立"覆盖)
        if cmd.has_stick:
            self._apply_stick_drive(state, cmd, last_targets)

    def _handle_request_mode(self, cmd: UserCommand, last_targets):
        req = cmd.request_mode
        if req is RobotMode.NATURAL:
            if self.mode is not RobotMode.NATURAL:
                self.request_transition(RobotMode.NATURAL, Direction.FWD,
                                        targets_now=last_targets, blend_time=0.6)
        elif req is RobotMode.STAND:
            if self.mode is not RobotMode.STAND:
                self.request_transition(RobotMode.STAND, targets_now=last_targets,
                                        blend_time=0.6)
        elif req is RobotMode.TROT:
            # 站立<->"走" 切换(键盘 space / 手柄 START 的 toggle 语义)。
            # "走" = walk_mode: 配了软 trot 就是 NATURAL(NaturalSoftTrot), 否则 StableTrot。
            if self.mode is RobotMode.STAND:
                self.request_transition(self.walk_mode, Direction.FWD,
                                        targets_now=last_targets, blend_time=0.6)
            else:
                self.request_transition(RobotMode.STAND, targets_now=last_targets,
                                        blend_time=0.6)

    def _apply_stick_drive(self, state: RobotState, cmd: UserCommand, last_targets):
        a = self.drive
        vx = cmd.vx
        rx = cmd.turn
        thr = a.gp_trot_threshold
        deadzone = a.gp_deadzone
        has_walk = abs(vx) > thr

        # 2a) 手柄 dpad = pace
        if cmd.pace and cmd.request_dir is not None:
            self.throttle = 1.0 if cmd.request_dir is Direction.FWD else -1.0
            new_dir = cmd.request_dir
            if self.mode is not RobotMode.PACE or self.direction is not new_dir:
                self.request_transition(RobotMode.PACE, new_dir,
                                        targets_now=last_targets, blend_time=0.4)
            return

        # 2b) 推杆前进/后退 -> 进入 walk_mode(软 trot 时=NaturalSoftTrot)
        if has_walk:
            self.throttle = vx
            if self.walk_mode is RobotMode.NATURAL:
                new_dir = Direction.FWD if vx > 0 else Direction.BWD
                if self.mode is not RobotMode.NATURAL or self.direction is not new_dir:
                    b_time = 0.6 if self.mode is RobotMode.STAND else 0.3
                    self.request_transition(RobotMode.NATURAL, new_dir,
                                            targets_now=last_targets, blend_time=b_time)
                self._apply_natural_throttle(state, vx, rx)
            else:
                new_dir = Direction.FWD if vx > 0 else Direction.BWD
                if self.mode is not RobotMode.TROT or self.direction is not new_dir:
                    b_time = 0.6 if self.mode is RobotMode.STAND else 0.3
                    self.request_transition(RobotMode.TROT, new_dir,
                                            targets_now=last_targets, blend_time=b_time)
                self._apply_trot_throttle(state, vx, rx)
            return

        # 2c) 只有转向摇杆 = 原地转（髋外展迈腿为主，不是前后差速）
        if abs(rx) > deadzone:
            self.throttle = 0.0
            if self.walk_mode is RobotMode.NATURAL:
                if self.mode is not RobotMode.NATURAL or self.direction is not Direction.FWD:
                    b_time = 0.6 if self.mode is RobotMode.STAND else 0.3
                    self.request_transition(RobotMode.NATURAL, Direction.FWD,
                                            targets_now=last_targets, blend_time=b_time)
                sched = self._nat_schedule.map(
                    VelocityCommand(vx=0.0, yaw_rate=rx)
                )
                apply_schedule_to_gait(self.nat_fwd, sched)
            else:
                if self.mode is not RobotMode.TROT or self.direction is not Direction.FWD:
                    b_time = 0.6 if self.mode is RobotMode.STAND else 0.3
                    self.request_transition(RobotMode.TROT, Direction.FWD,
                                            targets_now=last_targets, blend_time=b_time)
                sched = self._trot_schedule.map(
                    VelocityCommand(vx=0.0, yaw_rate=rx)
                )
                apply_schedule_to_gait(self.trot_fwd, sched)
            return

        # 2d) 无输入 = 归站立(松杆即停, 软 trot 也停 -> 回到带前腿 tarsus 的新站姿)
        self.throttle = 0.0
        if self.walk_mode is RobotMode.NATURAL and self.mode is RobotMode.NATURAL:
            # 复位摆幅/转向, 便于下次从满幅重新进入
            self.nat_fwd.turn_cmd = 0.0
            self.nat_fwd.amp_front = self.nat_amp_front
            self.nat_fwd.amp_rear = self.nat_amp_rear
        if self.mode is RobotMode.TROT and self.direction is Direction.FWD:
            self.trot_fwd.turn_cmd = 0.0
            self.trot_fwd.amp_front = self.fwd_amp_front
            self.trot_fwd.amp_rear = self.fwd_amp_rear
        if self.mode is not RobotMode.STAND:
            self.request_transition(RobotMode.STAND, targets_now=last_targets,
                                    blend_time=0.6)

    def _apply_natural_throttle(self, state: RobotState, vx: float, rx: float):
        """摇杆油门/转向 → SoftTrotSchedule → nat_fwd (amp/period/stance/vel_cmd)。"""
        a = self.drive
        deadzone = a.gp_deadzone
        yaw_cmd = rx
        if vx >= 0 and a.yaw_hold and abs(rx) <= deadzone and state.imu_connected:
            if self._yaw_target is None:
                self._yaw_target = state.yaw
            _err = math.degrees(_wrap_rad(state.yaw - self._yaw_target))
            _rate = math.degrees(state.gyro_yaw)
            _auto = a.yaw_hold_sign * (a.yaw_hold_kp * _err + a.yaw_hold_kd * _rate)
            _lim = a.yaw_hold_limit
            yaw_cmd = max(-_lim, min(_lim, _auto))
        elif state.imu_connected:
            self._yaw_target = state.yaw

        sched = self._nat_schedule.map(VelocityCommand(vx=vx, yaw_rate=yaw_cmd))
        apply_schedule_to_gait(self.nat_fwd, sched)

    def _apply_trot_throttle(self, state: RobotState, vx: float, rx: float):
        """线性油门 → SoftTrotSchedule（前进）/ 兼容后退缩放。"""
        a = self.drive
        deadzone = a.gp_deadzone

        if self.direction is Direction.FWD:
            yaw_cmd = rx
            if a.yaw_hold and abs(rx) <= deadzone and state.imu_connected:
                if self._yaw_target is None:
                    self._yaw_target = state.yaw
                _err = math.degrees(_wrap_rad(state.yaw - self._yaw_target))
                _rate = math.degrees(state.gyro_yaw)
                _auto = a.yaw_hold_sign * (a.yaw_hold_kp * _err + a.yaw_hold_kd * _rate)
                _lim = a.yaw_hold_limit
                yaw_cmd = max(-_lim, min(_lim, _auto))
            elif state.imu_connected:
                self._yaw_target = state.yaw
            sched = self._trot_schedule.map(VelocityCommand(vx=vx, yaw_rate=yaw_cmd))
            apply_schedule_to_gait(self.trot_fwd, sched)
        else:
            thr = a.gp_trot_threshold
            _mag = min(1.0, abs(vx))
            _span = 1.0 - thr
            _norm = (_mag - thr) / _span if _span > 1e-6 else 1.0
            _norm = max(0.0, min(1.0, _norm))
            a_scale = a.throttle_min_scale + (1.0 - a.throttle_min_scale) * _norm
            self.trot_bwd.amp_front = -a.amp_rear * a.bwd_amp_scale * a_scale
            self.trot_bwd.amp_rear = -a.amp_front * a.bwd_amp_scale * a_scale
            self.trot_bwd.turn_y_gain = a.cruise_turn_yamp
            self.trot_bwd.turn_cmd = rx * a.cruise_turn_scale
            avg = 0.5 * (abs(self.trot_bwd.amp_front) + abs(self.trot_bwd.amp_rear))
            period = float(getattr(self.trot_bwd, "period", 0.9))
            self.trot_bwd.vel_cmd = (-2.0 * avg / max(1e-3, period), 0.0, 0.0)
