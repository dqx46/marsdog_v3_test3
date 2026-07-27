"""Steady-state walk control loop — the real RuntimePipeline tick path.

``apps/walk.py`` owns startup/shutdown. This module owns every control-cycle
of the live loop so the main application no longer embeds algorithm glue.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional, Sequence

from marsdog_control.config.joints import JointDesc
from marsdog_control.control.executor import CommandExecutor
from marsdog_control.control.balance import RuntimeBalanceController
from marsdog_control.control.imu_balance import ImuAttitudeController
from marsdog_control.core.protocols import ClockLike
from marsdog_control.core.types import Direction, RobotState
from marsdog_control.backends import RobotBackend
from marsdog_control.hardware.behavior.tail import TailController
from marsdog_control.hardware.board import MotorBoard
from marsdog_control.input.hal import WalkInputHAL
from marsdog_control.io.recorder import RecorderRuntime
from marsdog_control.motion.gait_controller import GaitController
from marsdog_control.motion.mode_select import DirectionTestConfig, select_motion_target
from marsdog_control.motion.tarsus_bench import TarsusBenchRuntime
from marsdog_control.runtime.fsm import RuntimeStateMachine
from marsdog_control.runtime.lie_down_session import LieDownSession
from marsdog_control.runtime.status import RuntimeStatusDisplay
from marsdog_control.runtime.walk_state import WalkRuntimeState
from marsdog_control.safety.supervisor import SafetySupervisor


@dataclass
class WalkLoopResult:
    estopped_fall: bool = False
    dm_tarsus_active: bool = False
    lie_down_hold: bool = False


@dataclass
class LoopHardware:
    """Bundle of live driver handles shared across the control tick.

    Collapses the 7 loose ``lz/evo/dm/incos/imu/online/board`` params into one
    component so the assembly/tick injection surface stays small. Per-handle
    access is preserved via read-only properties on ``WalkLoopContext``.

    ``lz``/``evo``/``dm``/``incos``/``imu``/``online`` stay ``object`` on
    purpose: they are the actual hardware HAL boundary, deliberately
    structural so real drivers and ``tests/parity/fake_hardware.py`` fakes
    are interchangeable without a shared base class. ``board`` has a real
    published contract (``MotorBoard``), so it gets the precise type.
    """

    lz: object = None
    evo: object = None
    dm: object = None
    incos: object = None
    imu: object = None
    online: object = None
    board: Optional[MotorBoard] = None


def mode_str(fsm) -> str:
    """按实际正在消费的 controller 记录模式，避免状态枚举与执行路径不一致。"""
    gait = fsm.active_gait
    if gait is None:
        return "stand"
    if gait is fsm.nat_fwd:
        return "natural_fwd" if fsm.direction is Direction.FWD else "natural_bwd"
    if gait in (fsm.pace_fwd, fsm.pace_bwd):
        return "pace_fwd" if fsm.direction is Direction.FWD else "pace_bwd"
    if gait in (fsm.trot_fwd, fsm.trot_bwd):
        return "trot_fwd" if fsm.direction is Direction.FWD else "trot_bwd"
    return f"gait:{gait.__class__.__name__}"


@dataclass
class WalkLoopContext:
    """Everything the steady-state loop needs for one live walk session."""

    # hardware / sensors (single bundle; per-handle access via properties below)
    hw: LoopHardware
    runtime_state: WalkRuntimeState

    # input (Input HAL: keyboard + gamepad → command / dev-tuning)
    fsm: RuntimeStateMachine
    input_hal: WalkInputHAL
    bark_with_mouth: Callable[[], None]

    # motion / control components
    stand: GaitController
    safety: SafetySupervisor
    imu_ctrl: ImuAttitudeController
    balance_runtime: RuntimeBalanceController
    executor: CommandExecutor
    lie_down_session: LieDownSession
    recorder: RecorderRuntime
    status_display: RuntimeStatusDisplay

    # mutable loop state
    targets: dict
    cur_pos: dict
    smooth_tgt: dict
    dm_tarsus_active: bool
    joint_direction_test: bool
    hip_abd_test: bool
    leg_pitch_test: bool
    direction_test_start: float
    direction_test_duration_s: float
    direction_test_cfg: Optional[DirectionTestConfig] = None
    bench_runtime: Optional[TarsusBenchRuntime] = None
    bench_tarsus_side: Optional[str] = None
    tail: Optional[TailController] = None
    joint_map: Optional[Sequence[JointDesc]] = None
    control_hz: float = 200.0
    clock: Optional[ClockLike] = None
    backend: Optional[RobotBackend] = None

    # exit flags
    estopped_fall: bool = False
    running: bool = True

    # Per-handle read-only access so tick code keeps using ctx.lz / ctx.evo / ...
    @property
    def lz(self):
        return self.hw.lz

    @property
    def evo(self):
        return self.hw.evo

    @property
    def dm(self):
        return self.hw.dm

    @property
    def incos(self):
        return self.hw.incos

    @property
    def imu(self):
        return self.hw.imu

    @property
    def online(self):
        return self.hw.online

    @property
    def board(self):
        return self.hw.board

    def board_now(self):
        # runtime_state.board is the authoritative live handle (set at assembly);
        # fall back to the construction-time board only if it was never wired.
        if self.runtime_state.board is not None:
            return self.runtime_state.board
        return self.hw.board


def tick_walk_loop(ctx: WalkLoopContext) -> bool:
    """Run one fixed control-pipeline cycle. Return False to leave the loop."""
    clock = ctx.clock
    t_loop = clock.time()
    rt = ctx.runtime_state

    # ① State estimation
    if ctx.backend is not None:
        state = ctx.backend.read_state(ctx.online)
    else:
        # Fallback for old dry tests that haven't wired backend
        state = RobotState()

    # ② Input
    cmd, dev_key = ctx.input_hal.poll(ctx.fsm)
    if cmd.request_bark:
        ctx.bark_with_mouth()
    if cmd.quit:
        print("\n[quit]")
        return False
    if cmd.estop:
        print("\n[estop] 用户急停 -> ESTOP")
        from marsdog_control.core.types import RobotMode
        ctx.fsm.request_transition(RobotMode.ESTOP, targets_now=ctx.targets)
        return False
    if cmd.request_lie_down:
        lie_res = ctx.lie_down_session.handle_request(
            fsm=ctx.fsm,
            online=ctx.online,
            targets_now=ctx.targets,
            lz=ctx.lz,
            evo=ctx.evo,
            dm=ctx.dm,
            incos=ctx.incos,
            board=ctx.board_now(),
            dm_tarsus_active=ctx.dm_tarsus_active,
            smooth_tgt=ctx.smooth_tgt,
            safety=ctx.safety,
        )
        if lie_res.targets is not None:
            ctx.targets = dict(lie_res.targets)
        if lie_res.break_loop:
            return False
        if lie_res.continue_loop:
            return True

    if (not ctx.joint_direction_test) and (not ctx.lie_down_session.hold):
        ctx.input_hal.apply_dev_tuning(
            dev_key, ctx.fsm, ctx.imu_ctrl, ctx.lz, ctx.evo, ctx.dm, ctx.incos)

    # ③ Behavior FSM
    if (not ctx.joint_direction_test) and (not ctx.lie_down_session.hold):
        ctx.fsm.update(state, cmd, ctx.targets)
    if ctx.fsm.consume_just_switched():
        ctx.smooth_tgt.clear()

    if not ctx.joint_direction_test:
        ctx.dm_tarsus_active = ctx.fsm.dm_active()
    rt.dm.active = ctx.dm_tarsus_active
    active_trot = ctx.fsm.active_gait
    t_gait = ctx.fsm.t_gait
    height = ctx.fsm.height
    throttle = ctx.fsm.throttle
    mode = mode_str(ctx.fsm)
    if ctx.tail is not None:
        ctx.tail.set_mode(
            "lie_down" if ctx.lie_down_session.hold else
            "trot" if active_trot is not None else
            "stand"
        )

    # ── IMU 闭环 ──
    balance = ctx.balance_runtime.update(
        imu=ctx.imu,
        active_gait=active_trot,
        t_gait=t_gait,
        mode=mode,
        throttle=throttle,
    )
    imu_dz = balance.imu_dz
    imu_state = balance.imu_state
    if imu_state is None:
        imu_state = {}
    if hasattr(state, "vel_xyz"):
        imu_state["vel_xyz"] = state.vel_xyz

    # WBC 负责姿态/力：关掉 IMU 足高修正，避免双环抢控制
    if getattr(ctx.executor.config, "wbc_enabled", False) and getattr(
        ctx.executor.config, "disable_imu_foot_balance", True
    ):
        imu_dz = {"fl": 0.0, "fr": 0.0, "rl": 0.0, "rr": 0.0}
    
    # ── Motion planner ──
    t_rel = balance.t_rel
    motion = select_motion_target(
        fsm=ctx.fsm,
        state=state,
        imu_dz=imu_dz,
        imu_state=imu_state,
        online=ctx.online,
        cur_pos=ctx.cur_pos,
        smooth_tgt=ctx.smooth_tgt,
        stand=ctx.stand,
        lie_down_hold=ctx.lie_down_session.hold,
        lie_down_targets=ctx.lie_down_session.targets,
        direction_test=ctx.direction_test_cfg,
        clock=clock,
    )

    if ctx.bench_runtime is not None:
        bench_res = ctx.bench_runtime.apply(
            mode=mode, motion=motion, state=state, dm=ctx.dm)
        if bench_res.abort:
            print(f"\n[BENCH-ABORT] {bench_res.reason}")
            ctx.estopped_fall = True
            return False

    # ── Safety ──
    safe_motion, safety_report = ctx.safety.filter(state, motion)
    ctx.targets = safe_motion.q
    if safety_report.triggered_estop:
        print(f"\n[safety] 触发急停: {safety_report.reason}")
        from marsdog_control.core.types import RobotMode
        ctx.fsm.request_transition(RobotMode.ESTOP, targets_now=ctx.targets)
        ctx.estopped_fall = True
        return False

    # ── EVO 掉线重新使能 ──
    if ctx.joint_map is not None:
        for j in ctx.joint_map:
            if j.mtype == "evo" and j.motor_id in ctx.online:
                idx = j.motor_id - 1
                if ctx.evo.is_connected[idx] and ctx.evo.status[idx] != 0x02:
                    ctx.evo.enter_motor_state(j.motor_id)

    # ── Executor (explicit runtime state, no walk globals) ──
    rt.sync_executor_config(ctx.executor)
    output = ctx.executor.build(
        state, safe_motion, ctx.fsm,
        active_gait=active_trot, t_rel=t_rel, clock=clock)
    velocities = output.target.dq
    kp_phase = output.kp_phase
    trq_ff = output.trq_ff
    ctrl_dt = output.control_period_s

    # ── Send ──
    if ctx.backend is not None:
        ctx.backend.send(output)

    # ── Log ──
    dt = clock.time() - t_loop
    ctx.recorder.maybe_record(
        mode=mode, lz=ctx.lz, evo=ctx.evo, dm=ctx.dm, incos=ctx.incos,
        targets=ctx.targets,
        loop_dt_s=dt, active_gait=active_trot, throttle=throttle,
        imu=ctx.imu, imu_dz=imu_dz, imu_ctrl=ctx.imu_ctrl,
        kp_phase=kp_phase, trq_ff=trq_ff,
        fsm=ctx.fsm, cmd=cmd, stand=ctx.stand,
        bench_tarsus_side=ctx.bench_tarsus_side,
        bench_start=(ctx.bench_runtime.start_mono if ctx.bench_runtime else None),
        control_period_ms=ctrl_dt * 1000.0,
    )

    if ctx.bench_runtime is not None and ctx.bench_runtime.done:
        print("\n[BENCH] 全部频点完成，自动回站立并缓速失能。")
        return False

    # ── Status ──
    ctx.status_display.update(
        mode=mode,
        height=height,
        active_gait=active_trot,
        cmd=cmd,
        imu=ctx.imu,
        imu_dz=imu_dz,
        lie_down_hold=ctx.lie_down_session.hold,
        joint_direction_test=ctx.joint_direction_test,
        hip_abd_test=ctx.hip_abd_test,
        leg_pitch_test=ctx.leg_pitch_test,
        direction_test_start=ctx.direction_test_start,
        direction_test_duration_s=ctx.direction_test_duration_s,
        lz=ctx.lz,
        evo=ctx.evo,
        incos=ctx.incos,
        board=ctx.board_now(),
    )

    # ── Frequency ──
    elapsed = clock.time() - t_loop
    sleep_t = 1.0 / ctx.control_hz - elapsed
    if sleep_t > 0:
        clock.sleep(sleep_t)
    return ctx.running


def run_steady_state_loop(ctx: WalkLoopContext) -> WalkLoopResult:
    """Compat wrapper: prefer ``RuntimePipeline.tick`` via ``run_pipeline_loop``.

    Kept so parity / older call sites that hold a bare ``WalkLoopContext`` still
    work; production walk goes through ``RuntimeApp(pipeline=...).run()``.
    """
    from marsdog_control.runtime.app import RuntimePipeline, run_pipeline_loop
    result = run_pipeline_loop(RuntimePipeline(walk_loop=ctx))
    assert result is not None
    return result


__all__ = [
    "WalkLoopContext",
    "WalkLoopResult",
    "mode_str",
    "tick_walk_loop",
    "run_steady_state_loop",
]
