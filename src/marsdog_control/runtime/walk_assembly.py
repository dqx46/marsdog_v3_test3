"""Assemble the live walk steady-state loop context from wired components."""

from __future__ import annotations

from typing import Callable, Optional

from marsdog_control.control.balance import RuntimeBalanceConfig, RuntimeBalanceController
from marsdog_control.control.executor import CommandExecutor, ExecutorConfig
from marsdog_control.io.recorder import RecorderRuntime
from marsdog_control.motion.mode_select import DirectionTestConfig
from marsdog_control.motion.tarsus_bench import TarsusBenchRuntime
from marsdog_control.runtime.lie_down_session import LieDownSession
from marsdog_control.runtime.status import RuntimeStatusDisplay
from marsdog_control.runtime.walk_loop import WalkLoopContext
from marsdog_control.runtime.walk_startup import WalkStartupContext
from marsdog_control.runtime.walk_state import WalkRuntimeState
from marsdog_control.backends import RobotBackend


def assemble_walk_loop_context(
    *,
    startup: WalkStartupContext,
    runtime_state: WalkRuntimeState,
    hw,
    fsm, input_hal, stand, safety, imu_ctrl,
    targets, cur_pos, smooth_tgt,
    real_joints, joint_map,
    direction_test_base,
    direction_test_start: float,
    control_hz: float,
    clock,
    write_log: Callable,
    log_writer,
    bark_with_mouth: Callable,
    build_lie_down_target: Callable,
    build_sit_target: Optional[Callable] = None,
    read_positions: Callable,
    smooth_transition: Callable,
    backend: Optional[RobotBackend] = None,
    tail=None,
) -> WalkLoopContext:
    """Build every steady-state component and return a ready ``WalkLoopContext``.

    Every startup-resolved scalar (direction-test amplitudes, bench config,
    balance softstart/phase-gate knobs, ff_decouple/auto_trim/ramp) comes from
    the single ``startup`` bundle — this function never reads ``args``.
    """
    from marsdog_control.config.control_policies import (
        ControlPolicyError, ForceMode,
    )
    from marsdog_control.motion.attitude_overlay import (
        AttitudeOverlayGate, bind_ownership,
    )
    from marsdog_control.motion.lateral_planner import LateralPlanner

    session = getattr(startup, "session", None)
    policies = getattr(session, "policies", None) if session is not None else None
    if policies is None:
        raise ControlPolicyError(
            "assemble_walk_loop_context requires WalkSessionConfig.policies "
            "(no silent default ownership)"
        )

    joint_direction_test = startup.joint_direction_test
    hip_abd_test = startup.hip_abd_test
    leg_pitch_test = startup.leg_pitch_test
    calf_pitch_test = startup.calf_pitch_test

    dm_fixed = runtime_state.dm.fixed_targets
    recorder = RecorderRuntime(log_writer, write_log, interval=5, clock=clock)
    status_display = RuntimeStatusDisplay(real_joints, dm_fixed, clock=clock)
    lie_down_session = LieDownSession(
        build_target=build_lie_down_target,
        build_sit_target=build_sit_target,
        read_positions=read_positions,
        smooth_transition=smooth_transition,
        dm_fixed_targets=dm_fixed,
    )

    direction_test_cfg = None
    if joint_direction_test:
        direction_test_cfg = DirectionTestConfig(
            hip_abd=hip_abd_test,
            leg_pitch=leg_pitch_test,
            calf_pitch=calf_pitch_test,
            leg_pitch_amp_rad=startup.leg_pitch_test_amp_rad,
            calf_pitch_amp_rad=startup.calf_pitch_test_amp_rad,
            fade_s=startup.fade_s,
            base=direction_test_base,
            start_mono=direction_test_start,
        )

    bench_runtime = None
    if startup.bench_cfg is not None:
        bench_runtime = TarsusBenchRuntime(startup.bench_cfg, clock=clock)

    imp = runtime_state.impedance
    rt_cfg = getattr(startup, "runtime_config", None)
    force_mode = policies.force
    # Prefer session handles from prepare_walk_startup / control stack.
    lateral_planner = getattr(startup, "lateral_planner", None)
    attitude_gate = getattr(startup, "attitude_gate", None)
    if lateral_planner is None:
        lateral_planner = LateralPlanner(session_owner=policies.lateral)
    if attitude_gate is None:
        attitude_gate = AttitudeOverlayGate(attitude=policies.attitude)
    # Re-bind stand + FSM gaits to the same session planner/gate (idempotent
    # if assemble_walk_control_stack already bound the same instances).
    gait_handles = [stand]
    for attr in (
        "trot_fwd", "trot_bwd", "pace_fwd", "pace_bwd",
        "nat_fwd", "nat_bwd", "walk_fwd", "walk_bwd", "jump_fwd",
    ):
        gait_handles.append(getattr(fsm, attr, None))
    bind_ownership(
        lateral_planner=lateral_planner,
        attitude_gate=attitude_gate,
        gaits=gait_handles,
    )

    executor = CommandExecutor(
        config=ExecutorConfig(
            variable_impedance=imp.enabled,
            gravity_comp=runtime_state.gravity_comp,
            vmc_enabled=force_mode is ForceMode.VMC,
            wbc_enabled=force_mode is ForceMode.WBC,
            force_mode=force_mode,
            td_kp_scale=imp.td_kp_scale,
            swing_kp_scale=imp.swing_kp_scale,
            td_window=imp.td_window,
            leg_kp_scale=policies.impedance.effective_leg_kp_scale(),
            gravity_scale=runtime_state.gravity_scale,
        ),
        runtime_config=rt_cfg,
        lateral_planner=lateral_planner,
    )
    # ff_decouple only meaningful with IMU attitude owner.
    ff_decouple = bool(startup.ff_decouple) and bool(
        attitude_gate.allows_ff_decouple())
    balance_runtime = RuntimeBalanceController(
        imu_ctrl,
        RuntimeBalanceConfig(
            imu_softstart_s=startup.imu_softstart_s,
            trim_ramp_s=startup.trim_ramp_s,
            imu_phase_gate=startup.imu_phase_gate,
            phase_td_gain=startup.phase_td_gain,
            phase_swing_gain=startup.phase_swing_gain,
            td_window=imp.td_window,
            variable_impedance=imp.enabled,
            td_imu_freeze_i=startup.td_imu_freeze_i,
            imu_slew_m_s=startup.imu_slew_m_s,
            ff_decouple=ff_decouple,
            auto_trim=startup.auto_trim,
            ramp_s=startup.ramp_s,
        ),
        clock=clock,
    )

    runtime_state.board = hw.board
    return WalkLoopContext(
        hw=hw,
        runtime_state=runtime_state,
        fsm=fsm, input_hal=input_hal,
        bark_with_mouth=bark_with_mouth,
        stand=stand, safety=safety, imu_ctrl=imu_ctrl,
        balance_runtime=balance_runtime, executor=executor,
        lie_down_session=lie_down_session,
        recorder=recorder, status_display=status_display,
        backend=backend,
        targets=targets,
        cur_pos=cur_pos,
        smooth_tgt=smooth_tgt,
        dm_tarsus_active=runtime_state.dm.active,
        joint_direction_test=joint_direction_test,
        hip_abd_test=hip_abd_test,
        leg_pitch_test=leg_pitch_test,
        direction_test_start=direction_test_start,
        direction_test_duration_s=startup.fade_s,
        direction_test_cfg=direction_test_cfg,
        bench_runtime=bench_runtime,
        bench_tarsus_side=startup.bench_cfg.side if startup.bench_cfg else None,
        tail=tail,
        joint_map=joint_map,
        control_hz=control_hz,
        clock=clock,
        policies=policies,
        lateral_planner=lateral_planner,
    )


__all__ = ["assemble_walk_loop_context"]
