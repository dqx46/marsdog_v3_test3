"""Post-bring-up session orchestration for the live walk app.

``apps/walk.main()`` owns CLI parsing, startup validation and hardware bring-up,
then hands a fully-populated :class:`WalkSessionContext` to :func:`run_walk_session`.
This module runs the sequence that used to live inline in ``main()``:

    direction-test precheck -> fade-to-stand -> IMU calibration -> derive read-only
    view -> operator inputs / Input HAL -> steady-state loop assembly ->
    RuntimeApp(pipeline).run() -> shutdown (finally).

Seams that the parity harness patches on the ``walk`` module (``KeyReader``,
``TailController``, ``bark_with_mouth`` and the ``clock``) are passed in by
``main`` so the fakes still apply — this module must NOT import them directly.
Everything else (pipeline, assembly, bring-up helpers, config constants) is
imported here so ``apps/walk.py`` stays a thin shell.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Optional

from marsdog_control.config.joints import (
    JOINT_MAP, JOINT_BY_ID, JOINT_BY_NAME as JBN,
)
from marsdog_control.motion.motion_planner import (
    _HIP_ABDUCTION_TEST_IDS,
    _LEG_PITCH_TEST_JOINTS,
    _CALF_PITCH_TEST_JOINTS,
)
from marsdog_control.config.schema import RuntimeConfig
from marsdog_control.core.protocols import ClockLike
from marsdog_control.runtime.app import RuntimeApp, RuntimePipeline
from marsdog_control.runtime.shutdown import (
    WalkShutdownContext, run_walk_shutdown,
)
from marsdog_control.runtime.walk_assembly import assemble_walk_loop_context
from marsdog_control.backends.real import RealRobotBackend
from marsdog_control.runtime.walk_bringup import (
    HardwareSession,
    calibrate_imu_after_stand,
    fade_to_stand,
    open_operator_inputs,
)
from marsdog_control.runtime.walk_controllers import WalkControlStack
from marsdog_control.runtime.walk_loop import LoopHardware
from marsdog_control.runtime.walk_services import WalkServices
from marsdog_control.runtime.walk_startup import WalkStartupContext
from marsdog_control.runtime.walk_state import WalkRuntimeState

# tarsus (达妙 4/8) 真实电机不参与 IK/步态; 与主控保持一致用 bus!="none" 过滤。
_REAL_JOINTS = [j for j in JOINT_MAP if j.bus != "none"]


@dataclass
class WalkSessionContext:
    """Everything :func:`run_walk_session` needs after hardware bring-up.

    Runtime bundles (``startup``/``stack``/``hw``) are passed whole rather than
    unpacked so the field list stays bounded; ``svc`` (:class:`WalkServices`) is
    the single I/O owner. Parity-patched seams come in as ``*_cls`` / callables.
    """

    # runtime bundles
    startup: WalkStartupContext
    stack: WalkControlStack
    hw: HardwareSession
    svc: WalkServices
    runtime_state: WalkRuntimeState
    args: Any                          # residual CLI (scope/devtools); migrated fields → runtime_config
    runtime_config: RuntimeConfig
    cur_pos: dict

    # logging / scope
    log_file: Any
    log_writer: Any
    log_path: Optional[str]
    scope_proc: Any

    # parity-patched seams (read from walk module at call time by main;
    # kept as `Any`/bare `Callable` on purpose — the harness swaps in fakes
    # that don't share a base class with the real thing)
    key_reader_cls: Any
    gamepad_cls: Any
    input_state_cls: Any
    tail_cls: Any
    bark_with_mouth: Callable[[], None]
    clock: Optional[ClockLike]
    gamepad_device: str

    # app-bound seams / paths
    input_hal_cls: Any
    build_lie_down_target: Callable
    stop_scope: Callable
    load_trim_cal: Callable
    save_trim_cal: Callable
    trim_cal_path: str
    control_hz: float
    build_sit_target: Optional[Callable] = None


def _direction_test_precheck(ctx: WalkSessionContext) -> bool:
    """Return True if OK to continue; shut down + return False if a test axis is offline."""
    startup = ctx.startup
    if not startup.joint_direction_test:
        return True
    if startup.hip_abd_test:
        test_ids = _HIP_ABDUCTION_TEST_IDS
    elif startup.leg_pitch_test:
        test_ids = tuple(JBN[name].motor_id for name in _LEG_PITCH_TEST_JOINTS)
    else:
        test_ids = tuple(JBN[name].motor_id for name in _CALF_PITCH_TEST_JOINTS)
    missing_joints = [mid for mid in test_ids if mid not in ctx.hw.online]
    if missing_joints:
        names = ", ".join(JOINT_BY_ID[mid].name for mid in missing_joints)
        print(f"[FATAL] 关节方向测试取消：以下测试轴离线: {names}")
        hw = ctx.hw
        ctx.svc.shutdown_motors(hw.lz, hw.evo, hw.dm, hw.incos)
        return False
    return True


def run_walk_session(ctx: WalkSessionContext) -> None:
    """Run the full post-bring-up walk session (fade -> loop -> shutdown)."""
    startup = ctx.startup
    stack = ctx.stack
    hw = ctx.hw
    svc = ctx.svc
    args = ctx.args
    runtime_state = ctx.runtime_state
    lz, evo, dm, incos = hw.lz, hw.evo, hw.dm, hw.incos
    imu, imu_ok, online = hw.imu, hw.imu_ok, hw.online
    stand, fsm, safety, imu_ctrl = stack.stand, stack.fsm, stack.safety, stack.imu_ctrl

    if not _direction_test_precheck(ctx):
        return

    # ── 统一基准：所有方向测试也必须先走正常主控站姿 ───────────────────────
    soft_disable = bool(getattr(args, "soft_disable", False))
    stand_ready = fade_to_stand(
        stand=stand, cur_pos=ctx.cur_pos, online=online,
        lz=lz, evo=evo, dm=dm, incos=incos,
        fade_s=startup.fade_s,
        smooth_transition=svc.smooth_transition,
        recover_lz_stand_faults=svc.recover_lz_stand_faults,
        shutdown_motors=svc.shutdown_motors,
        joint_direction_test=startup.joint_direction_test,
        hip_abd_test=startup.hip_abd_test,
        leg_pitch_test=startup.leg_pitch_test,
        # Hot-start: full stiffness throughout (no kp 0.3 soft dip).
        kp_start=0.3 if soft_disable else 1.0,
        kp_end=1.0,
        stop_pose_hold=svc.stop_pose_hold,
    )
    if not stand_ready.ok:
        return
    direction_test_base = stand_ready.direction_test_base

    # Hold stand pose through IMU/gamepad setup — log showed waist_pitch
    # drifting +15° then snapping when the main loop resumed after ~2s gap.
    stand_hold = dict(stand_ready.stand_motor) if stand_ready.stand_motor else {}
    if stand_hold:
        if dm is not None:
            stand_hold.update(getattr(svc, "dm_fixed_targets", {}) or {})
        svc.start_pose_hold(lz, evo, dm, incos, stand_hold)
        print("[hold] 站立后保位 ON（覆盖 IMU/手柄初始化空窗）")

    # ── IMU 校准（站立后做，确保零位准确）─────────────────────────────────
    calibrate_imu_after_stand(
        imu=imu, imu_ok=imu_ok, imu_ctrl=imu_ctrl,
        joint_direction_test=startup.joint_direction_test,
        no_imu=startup.no_imu,
    )

    # 运行期权威状态全部在 fsm; 下面这些每周期从 fsm 派生, 只给下游只读消费。
    active_trot = fsm.active_gait

    # ── 尾巴 / 键盘 / 手柄 ────────────────────────────────────────────────
    ops = open_operator_inputs(
        gamepad_enabled=startup.gamepad_enabled,
        tail_enabled=startup.tail_enabled,
        key_reader_cls=ctx.key_reader_cls,
        gamepad_cls=ctx.gamepad_cls,
        input_state_cls=ctx.input_state_cls,
        tail_cls=ctx.tail_cls,
        gamepad_device=ctx.gamepad_device,
        clock=ctx.clock,
    )
    kb, gp, inp, tail = ops.kb, ops.gp, ops.inp, ops.tail
    input_hal = ctx.input_hal_cls(
        gamepad=gp, keyboard=kb, state=inp,
        runtime_state=runtime_state, check_motors=svc.check_motors)

    if active_trot:
        print(f"[gait] 已进入 Trot  {active_trot.describe()}")

    targets = (dict(direction_test_base) if startup.joint_direction_test
               else stand.get_targets(0))
    smooth_tgt = {}
    # 航向保持已移入 fsm(_yaw_target); 安全层清一次输出记忆, 首周期不误触发跳变闸。
    safety.reset()
    direction_test_start = ctx.clock.monotonic()
    estopped_fall = False   # 安全层摔倒急停: 清理时跳过"回站立", 直接进缓速失能斜坡
    lie_down_hold = False

    runtime_state.board = svc.board
    hw_loop = LoopHardware(
        lz=lz, evo=evo, dm=dm, incos=incos, imu=imu, online=online,
        board=svc.board)
    real_backend = RealRobotBackend(svc, lz, evo, dm, incos, imu)
    loop_ctx = assemble_walk_loop_context(
        startup=startup,
        runtime_state=runtime_state,
        hw=hw_loop,
        fsm=fsm, input_hal=input_hal, stand=stand, safety=safety,
        imu_ctrl=imu_ctrl,
        targets=targets, cur_pos=ctx.cur_pos, smooth_tgt=smooth_tgt,
        real_joints=_REAL_JOINTS, joint_map=JOINT_MAP,
        direction_test_base=direction_test_base,
        direction_test_start=direction_test_start,
        control_hz=ctx.control_hz, clock=ctx.clock,
        write_log=svc.write_log, log_writer=ctx.log_writer,
        bark_with_mouth=ctx.bark_with_mouth,
        backend=real_backend,
        build_lie_down_target=ctx.build_lie_down_target,
        build_sit_target=ctx.build_sit_target,
        read_positions=svc.read_positions,
        smooth_transition=svc.smooth_transition,
        tail=tail,
    )

    try:
        pipeline = RuntimePipeline(
            walk_loop=loop_ctx,
            config=ctx.runtime_config or RuntimeConfig(),
        )
        # Sole steady-state path: RuntimeApp → pipeline.tick() → tick_walk_loop
        # Keep stand-hold until the first control tick is about to run; stop in
        # finally via shutdown_motors as well.
        svc.stop_pose_hold()
        loop_result = RuntimeApp(pipeline=pipeline).run()
        runtime_state.dm.active = loop_result.dm_tarsus_active
        estopped_fall = loop_result.estopped_fall
        lie_down_hold = loop_result.lie_down_hold
    finally:
        svc.stop_pose_hold()
        run_walk_shutdown(WalkShutdownContext(
            kb=kb,
            gp=gp,
            tail=tail,
            board=svc.board,
            balance_runtime=loop_ctx.balance_runtime,
            args=args,
            imu_ctrl=imu_ctrl,
            stand=stand,
            lz=lz,
            evo=evo,
            dm=dm,
            incos=incos,
            imu=imu,
            log_file=ctx.log_file,
            log_path=ctx.log_path,
            scope_proc=ctx.scope_proc,
            dm_tarsus_active=runtime_state.dm.active,
            joint_direction_test=startup.joint_direction_test,
            lie_down_hold=lie_down_hold,
            estopped_fall=estopped_fall,
            actuation_runtime=svc.actuation_runtime,
            smooth_transition=svc.smooth_transition,
            shutdown_motors=svc.shutdown_motors,
            stop_scope=ctx.stop_scope,
            load_trim_cal=ctx.load_trim_cal,
            save_trim_cal=ctx.save_trim_cal,
            trim_cal_path=ctx.trim_cal_path,
            control_hz=ctx.control_hz,
            clock=ctx.clock,
        ))


__all__ = ["WalkSessionContext", "run_walk_session"]
