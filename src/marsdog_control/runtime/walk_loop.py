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


def _set_stand_hip_abduction(stand, hip_abduction: float) -> None:
    if hasattr(stand, "set_hip_abduction"):
        stand.set_hip_abduction(hip_abduction)
    else:
        stand.hip_abduction = float(hip_abduction)
        if hasattr(stand, "_update_cache"):
            stand._update_cache()


def clear_abd_flare(ctx: "WalkLoopContext", *, reason: str = "") -> None:
    """收回站立外展验证角，恢复 flare 前的 hip_abduction。"""
    if not ctx.abd_flare_active and ctx.abd_flare_base is None:
        return
    if ctx.abd_flare_base is not None:
        _set_stand_hip_abduction(ctx.stand, ctx.abd_flare_base)
    ctx.abd_flare_active = False
    ctx.abd_flare_base = None
    if reason:
        print(f"\n[abd-flare] OFF ({reason})")


def toggle_abd_flare(ctx: "WalkLoopContext") -> None:
    """键盘 a：站立时切换四腿外展打开/收回，用于肉眼核对关节方向。"""
    import math

    if ctx.lie_down_session.hold:
        print("\n[abd-flare] 坐下/趴下保持中，忽略 (请先起立回 STAND)")
        return
    if ctx.fsm.active_gait is not None:
        print("\n[abd-flare] 仅站立可用；请先 SPACE 回站立再按 a")
        return

    if ctx.abd_flare_active:
        clear_abd_flare(ctx, reason="再按 a")
        return

    base = float(getattr(ctx.stand, "hip_abduction", 0.08))
    target = max(float(ctx.abd_flare_rad), base + 0.06)
    ctx.abd_flare_base = base
    ctx.abd_flare_active = True
    _set_stand_hip_abduction(ctx.stand, target)
    print(
        f"\n[abd-flare] ON: 站立外展 {base:.3f}→{target:.3f} rad "
        f"({math.degrees(target):.1f}°)  URDF+ = 四腿同时向外\n"
        "  关节: ID2 fl_thigh_roll / ID6 fr_thigh_roll / "
        "ID9 rl_hip / ID12 rr_hip\n"
        "  期望: 四条腿都向外打开；若某腿向内收 → 该关节 sign 或轴约定反了\n"
        "  再按 a=收回；SPACE 走路会自动收回"
    )


def mode_str(fsm) -> str:
    """按实际正在消费的 controller 记录模式，避免状态枚举与执行路径不一致。"""
    gait = fsm.active_gait
    if gait is None:
        return "stand"
    if getattr(fsm, "jump_fwd", None) is not None and gait is fsm.jump_fwd:
        return "jump_fwd"
    if getattr(fsm, "walk_fwd", None) is not None and gait is fsm.walk_fwd:
        return "walk_fwd" if fsm.direction is Direction.FWD else "walk_bwd"
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
    # 站立外展方向验证(键盘 a): URDF+ 四腿同时外开 ID2/6/9/12
    abd_flare_active: bool = False
    abd_flare_base: Optional[float] = None
    abd_flare_rad: float = 0.16  # ~9.2°；应明显外开且仍在软限位内
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
    if cmd.request_abd_flare_toggle:
        toggle_abd_flare(ctx)

    if cmd.request_lie_down or cmd.request_sit or cmd.request_go_zero:
        if ctx.abd_flare_active:
            clear_abd_flare(ctx, reason="切姿势前收回外展")
        # 同周期多键：回零 > 坐下 > 趴下
        if cmd.request_go_zero:
            pose = "zero"
        elif cmd.request_sit:
            pose = "sit"
        else:
            pose = "lie_down"
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
            pose=pose,
            mono=clock.monotonic(),
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
    # 走路时强制收回外展验证，避免与 SoftTrot 髋外展叠加
    if ctx.abd_flare_active and active_trot is not None:
        clear_abd_flare(ctx, reason="进入步态")
    t_gait = ctx.fsm.t_gait
    height = ctx.fsm.height
    throttle = ctx.fsm.throttle
    mode = mode_str(ctx.fsm)
    if ctx.tail is not None:
        if ctx.lie_down_session.hold:
            pose = ctx.lie_down_session.active_pose
            if pose == "sit":
                tail_mode = "sit"
            elif pose == "zero":
                tail_mode = "stand"
            else:
                tail_mode = "lie_down"
        elif active_trot is not None:
            tail_mode = "trot"
        else:
            tail_mode = "stand"
        # TailController 仅识别 stand/trot/lie_down；sit 暂按 lie_down 行为
        ctx.tail.set_mode("lie_down" if tail_mode == "sit" else tail_mode)

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
    # Spot world-hold needs yaw (and optional base XY).
    imu_state["yaw"] = float(getattr(state, "yaw", 0.0))
    if "base_xy" not in imu_state:
        if hasattr(state, "base_xy") and state.base_xy is not None:
            imu_state["base_xy"] = state.base_xy
        elif ctx.backend is not None and hasattr(ctx.backend, "data"):
            # MuJoCo freejoint XY (sim). Real relies on vel integration in gait.
            try:
                imu_state["base_xy"] = (
                    float(ctx.backend.data.qpos[0]),
                    float(ctx.backend.data.qpos[1]),
                )
            except Exception:
                pass
    if "base_z" not in imu_state and ctx.backend is not None and hasattr(ctx.backend, "data"):
        try:
            imu_state["base_z"] = float(ctx.backend.data.qpos[2])
        except Exception:
            pass

    # WBC 负责姿态/力：关掉 IMU 足高修正，避免双环抢控制
    if getattr(ctx.executor.config, "wbc_enabled", False) and getattr(
        ctx.executor.config, "disable_imu_foot_balance", True
    ):
        imu_dz = {"fl": 0.0, "fr": 0.0, "rl": 0.0, "rr": 0.0}
    # 姿势保持：目标已是捕获关节角，禁止 IMU 足高再改（防 hold 首帧抽一下）
    if ctx.lie_down_session.hold:
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
    # 姿势 hold：刚度+重力缓入，避免过渡指令到位但实测滞后时全增益猛拉
    _ex_cfg = ctx.executor.config
    _grav_saved = (_ex_cfg.gravity_comp, float(_ex_cfg.gravity_scale))
    _kp_saved = float(_ex_cfg.kp_scale)
    if ctx.lie_down_session.hold:
        mono_now = clock.monotonic()
        blend = ctx.lie_down_session.hold_grav_blend(mono_now)
        kp_b = ctx.lie_down_session.hold_kp_blend(mono_now)
        if blend <= 1e-6:
            _ex_cfg.gravity_comp = False
        else:
            _ex_cfg.gravity_scale = _grav_saved[1] * blend
        _ex_cfg.kp_scale = _kp_saved * float(kp_b)
    try:
        output = ctx.executor.build(
            state, safe_motion, ctx.fsm,
            active_gait=active_trot, t_rel=t_rel, clock=clock)
    finally:
        _ex_cfg.gravity_comp, _ex_cfg.gravity_scale = _grav_saved
        _ex_cfg.kp_scale = _kp_saved
    velocities = output.target.dq
    kp_phase = output.kp_phase
    trq_ff = output.trq_ff
    ctrl_dt = output.control_period_s

    # Sim: nail freejoint XY while STAND / jump ground-hold (kills soft-sole scrub).
    # While walking: light viscous XY damp bleeds the constant scrub overshoot
    # (truth was ~0.13 vs cmd ~0.04). Do NOT key off vel_cmd — SoftTrot cruise
    # is often ~0.04 and used to falsely freeze mid-gait.
    if ctx.backend is not None and hasattr(ctx.backend, "set_xy_hold_damp"):
        hold_xy = False
        if active_trot is None:
            hold_xy = True
        else:
            fam = getattr(active_trot, "family", None)
            if fam == "jump":
                ph = getattr(getattr(active_trot, "phase", None), "value", "idle")
                hold_xy = ph not in ("crouch", "push", "flight")
        ctx.backend.set_xy_hold_damp(1.0 if hold_xy else 0.0)
        if hasattr(ctx.backend, "set_xy_walk_damp"):
            # ~55/s matches SoftTrot cmd≈truth at cruise 0.10 (scrub alone was ~3×).
            ctx.backend.set_xy_walk_damp(0.0 if hold_xy else 55.0)

    # ── Send ──
    if ctx.backend is not None:
        ctx.backend.send(output)

    # ── Log ──
    # 注意：这里的 dt 是 sleep 之前的执行时间，用于记录计算耗时。
    # 真正的周期时间（包含 sleep）在下一帧的 t_loop 计算。
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
        pose_hold_name=ctx.lie_down_session.active_pose,
        joint_direction_test=ctx.joint_direction_test,
        hip_abd_test=ctx.hip_abd_test,
        leg_pitch_test=ctx.leg_pitch_test,
        direction_test_start=ctx.direction_test_start,
        direction_test_duration_s=ctx.direction_test_duration_s,
        abd_flare_active=ctx.abd_flare_active,
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
