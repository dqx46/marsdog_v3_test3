"""Factories for gait controllers, FSM, safety, and IMU attitude PID.

Phase A extraction from ``apps.walk.main``: keep construction logic out of the
app shell while preserving the same parameter wiring and print side-effects.

Phase M: CLI ``args`` is snapshotted once into ``GaitStackConfig`` /
``FsmDriveConfig`` / ``ImuBuildConfig`` at the top of
``assemble_walk_control_stack``; factories below never dig into a Namespace.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Callable, Optional

from marsdog_control.compat import ensure_legacy_path

ensure_legacy_path()

from marsdog_control.motion import gait_controller  # noqa: E402
from marsdog_control.motion.gait_controller import GaitController  # noqa: E402
from marsdog_control.motion.gait_recipes import ControllerSet, build_controller_set  # noqa: E402
from marsdog_control.control.imu_balance import ImuAttitudeController  # noqa: E402
from marsdog_control.config.stack_build import (  # noqa: E402
    FsmDriveConfig,
    GaitStackConfig,
    ImuBuildConfig,
)
from marsdog_control.core.types import RobotMode  # noqa: E402
from marsdog_control.runtime.fsm import RuntimeStateMachine  # noqa: E402
from marsdog_control.safety.supervisor import SafetySupervisor  # noqa: E402


@dataclass
class ForwardGaitAmps:
    """Forward-direction amp/period derived from gait stack config."""

    amp_front: float
    amp_rear: float
    step_h: float
    step_h_front: float
    period: float


@dataclass
class WalkControlStack:
    """Gait controllers + FSM + safety + IMU closed-loop controller."""

    controllers: ControllerSet
    stand: GaitController
    trot_fwd: GaitController
    trot_bwd: GaitController
    pace_fwd: GaitController
    pace_bwd: GaitController
    nat_fwd: GaitController
    walk_fwd: GaitController
    jump_fwd: GaitController
    fsm: RuntimeStateMachine
    safety: SafetySupervisor
    imu_ctrl: ImuAttitudeController
    fwd: ForwardGaitAmps
    natural_active: bool
    start_mode: RobotMode


def compute_forward_gait_amps(cfg: GaitStackConfig) -> ForwardGaitAmps:
    """Resolve forward amp/period, including the fwd-use-bwd experiment."""
    if cfg.fwd_use_bwd:
        amp_front = cfg.amp_rear * cfg.bwd_amp_scale
        amp_rear = cfg.amp_front * cfg.bwd_amp_scale
        step_h = cfg.bwd_step_h
        def_front_lift = cfg.bwd_step_h
        period = cfg.bwd_period
    else:
        amp_front = cfg.amp_front
        amp_rear = cfg.amp_rear
        step_h = cfg.step_h
        def_front_lift = (
            cfg.step_h_front if cfg.step_h_front > 1e-6 else cfg.step_h)
        period = cfg.period

    amp_front *= cfg.fwd_front_amp_scale
    step_h_front = (
        cfg.fwd_front_lift if cfg.fwd_front_lift > 1e-6 else def_front_lift)
    return ForwardGaitAmps(
        amp_front=amp_front,
        amp_rear=amp_rear,
        step_h=step_h,
        step_h_front=step_h_front,
        period=period,
    )


def build_gait_controllers(
    cfg: GaitStackConfig,
    *,
    natural_active: bool,
    natural_params: dict,
    walk_params: Optional[dict] = None,
    jump_params: Optional[dict] = None,
    no_spine: bool = False,
):
    """Wrap ``gait_recipes.build_controller_set`` with walk-startup defaults."""
    return build_controller_set(
        cfg,
        front_x0=gait_controller._FRONT_X0,
        rear_x0=gait_controller._REAR_X0,
        # Always pass recipe dict so soft-trot extras (rear_clearance_m, retract, …)
        # apply; cfg alone does not carry every NATURAL_SOFT_TROT_WBC key.
        natural_params=natural_params,
        walk_params=walk_params,
        jump_params=jump_params,
        natural_spine_yaw_deg=(
            0.0 if no_spine else (
                None if natural_active
                else natural_params.get("spine_yaw_deg", 1.5))
        ),
        natural_spine_roll_deg=(
            0.0 if no_spine else (
                None if natural_active
                else natural_params.get("spine_roll_deg", 0.6))
        ),
        pace_use_stand_offsets=False,
    )


def build_safety_supervisor() -> SafetySupervisor:
    return SafetySupervisor(
        fall_guard_deg=45.0,
        max_delta_rad=math.radians(20.0),
        imu_max_age_s=0.3,
        require_imu=False,
    )


def build_runtime_fsm(
    controllers,
    drive: FsmDriveConfig,
    *,
    fwd: ForwardGaitAmps,
    natural_configured: bool,
    natural_walk: bool = False,
    natural_jump: bool = False,
    start_mode: RobotMode,
    height: float,
) -> RuntimeStateMachine:
    return RuntimeStateMachine(
        controllers, drive,
        height=height,
        fwd_amp_front=fwd.amp_front,
        fwd_amp_rear=fwd.amp_rear,
        natural_configured=natural_configured,
        natural_walk=natural_walk,
        natural_jump=natural_jump,
        start_mode=start_mode,
    )


def build_imu_attitude_controller(
    cfg: ImuBuildConfig,
    *,
    load_trim_cal: Optional[Callable[[], Any]] = None,
) -> ImuAttitudeController:
    """Build the stand/walk IMU PID (test mode vs normal/NaturalTrot)."""
    if cfg.imu_test:
        imu_ctrl = ImuAttitudeController(
            kp_roll=0.06,
            kp_pitch=0.05,
            ki_roll=0.20,
            ki_pitch=0.20,
            kd_roll=0.002,
            kd_pitch=0.002,
            decay_rate=0.990,
            max_correction=0.030,
            deadzone_deg=1.0,
            fall_guard_deg=35.0,
            predict_lead_s=cfg.imu_predict_ms / 1000.0,
            prediction_max_s=cfg.imu_predict_max_ms / 1000.0,
            gyro_max_age_s=cfg.imu_gyro_max_age_ms / 1000.0,
            dynamic_prediction=cfg.dynamic_imu_predict,
        )
        print("[IMU-TEST] 标准PID: kp=0.06/0.05, ki=0.001, kd=0.002, max=30mm")
        return imu_ctrl

    if cfg.natural_trot:
        kp = cfg.imu_kp
    else:
        kp = cfg.imu_kp if cfg.imu_kp > 1e-6 else 0.03
    imu_ctrl = ImuAttitudeController(
        kp_roll=kp,
        kp_pitch=kp,
        ki_roll=0.10,
        ki_pitch=0.10,
        kd_roll=0.002,
        kd_pitch=0.002,
        decay_rate=0.995,
        fast_decay=0.90,
        max_correction=cfg.max_corr_mm / 1000.0,
        deadzone_deg=1.5,
        fall_guard_deg=22.0,
        gyro_ema=cfg.imu_ema,
        damp_soft_mm=3.0,
        damp_hard_mm=cfg.damp_hard_mm,
        damp_gyro_lo=cfg.damp_gyro_lo,
        damp_gyro_hi=cfg.damp_gyro_hi,
        p_boost=cfg.roll_p_boost,
        p_sched_lo_deg=cfg.roll_p_lo_deg,
        p_sched_hi_deg=cfg.roll_p_hi_deg,
        roll_trim_mm=cfg.roll_trim_mm,
        pitch_trim_mm=cfg.pitch_trim_mm,
        auto_trim=cfg.auto_trim,
        auto_trim_rate=cfg.auto_trim_rate,
        auto_trim_limit_mm=cfg.auto_trim_limit_mm,
        ff_phases=cfg.trim_phases,
        predict_lead_s=cfg.imu_predict_ms / 1000.0,
        prediction_max_s=cfg.imu_predict_max_ms / 1000.0,
        gyro_max_age_s=cfg.imu_gyro_max_age_ms / 1000.0,
        dynamic_prediction=cfg.dynamic_imu_predict,
    )
    if cfg.imu_predict_ms > 1e-6:
        print(f"[PRED] IMU 动态预测: angle年龄 + 执行提前{cfg.imu_predict_ms:.0f}ms, "
              f"上限{cfg.imu_predict_max_ms:.0f}ms, gyro>{cfg.imu_gyro_max_age_ms:.0f}ms降级")
    if abs(cfg.roll_trim_mm) > 1e-6 or abs(cfg.pitch_trim_mm) > 1e-6:
        print(f"[T] 静态配平: roll={cfg.roll_trim_mm:+.1f}mm "
              f"pitch={cfg.pitch_trim_mm:+.1f}mm  (热键 k/l 调 roll)")
    if cfg.auto_trim:
        cal = load_trim_cal() if load_trim_cal is not None else None
        loaded = False
        if isinstance(cal, dict) and cal.get("roll_ff_mm"):
            loaded = imu_ctrl.set_roll_ff_mm(cal["roll_ff_mm"])
        if loaded:
            lv = imu_ctrl.get_roll_ff_mm()
            desc = (f"值{lv[0]:+.2f}mm" if len(lv) == 1
                    else f"峰峰{imu_ctrl.roll_ff_span_mm:.1f}mm")
            print(f"[AT] 已加载本机配平({cal.get('phases')}相位, {desc}) "
                  f"→ 随振幅斜坡渐入, 起步不过修")
        else:
            print("[AT] 无匹配标定(或相位数变化), 本次现场学习并保存 (仅首次有瞬态)")
        mode = (f"ILC×{cfg.trim_phases}相位"
                if cfg.trim_phases > 1 else "单值直流")
        print(f"[AT] roll 在线自学习({mode}): rate={cfg.auto_trim_rate:.2f} "
              f"±{cfg.auto_trim_limit_mm:.0f}mm (每台狗自适应, 结束自动保存)")
    if cfg.imu_ema > 1e-6:
        print(f"[P2] IMU D项 gyro EMA 滤波 = {cfg.imu_ema:.2f}")
    if cfg.damp_hard_mm > 3.0 + 1e-6:
        print(f"[B] 非线性阻尼开启: {3.0:.0f}→{cfg.damp_hard_mm:.0f}mm "
              f"@ {cfg.damp_gyro_lo:.0f}-{cfg.damp_gyro_hi:.0f} deg/s")
    if cfg.roll_p_boost > 1.0 + 1e-6:
        print(f"[D] P增益调度开启: x1→x{cfg.roll_p_boost:.1f} "
              f"@ {cfg.roll_p_lo_deg:.0f}-{cfg.roll_p_hi_deg:.0f}° "
              f"max={cfg.max_corr_mm:.0f}mm")
    return imu_ctrl


def assemble_walk_control_stack(
    args,
    *,
    natural_active: bool,
    natural_params: dict,
    gp_trot_threshold: float,
    gp_deadzone: float,
    natural_soft: bool = False,
    natural_walk: bool = False,
    walk_params: Optional[dict] = None,
    natural_jump: bool = False,
    jump_params: Optional[dict] = None,
    trot_flag: bool = False,
    no_spine: bool = False,
    load_trim_cal: Optional[Callable[[], Any]] = None,
) -> WalkControlStack:
    """Build controllers + FSM + safety + IMU for walk startup.

    Boundary: snapshot post-preset CLI ``args`` once into typed stack configs.
    Mode flags come from ``WalkStartupContext`` (not re-read from Namespace).
    """
    gait_cfg = GaitStackConfig.from_args(args)
    imu_cfg = ImuBuildConfig.from_args(args)
    drive = FsmDriveConfig.from_args(
        args, gp_trot_threshold=gp_trot_threshold, gp_deadzone=gp_deadzone)

    fwd = compute_forward_gait_amps(gait_cfg)
    if abs(gait_cfg.fwd_front_amp_scale - 1.0) > 1e-6 or gait_cfg.fwd_front_lift > 1e-6:
        print(f"[前腿支撑] 前腿摆幅×{gait_cfg.fwd_front_amp_scale:.2f}"
              f"→±{fwd.amp_front*100:.2f}cm  "
              f"前腿抬腿高={fwd.step_h_front*100:.2f}cm "
              f"(后腿摆幅±{fwd.amp_rear*100:.2f}cm/抬腿{fwd.step_h*100:.2f}cm)")
    if drive.yaw_hold:
        print(f"[航向保持] 开启: kp={drive.yaw_hold_kp:.3f} kd={drive.yaw_hold_kd:.3f} "
              f"sign={drive.yaw_hold_sign:+.0f} 上限±{drive.yaw_hold_limit:.2f} "
              f"→ 不打方向自动走直线(纠反了就把 --yaw-hold-sign 改成 -1)")
    if gait_cfg.fwd_use_bwd:
        print(f"[实验] 前进套用后退配方: 周期={fwd.period:.2f}s "
              f"摆幅前/后=±{fwd.amp_front*100:.1f}/{fwd.amp_rear*100:.1f}cm "
              f"抬腿前/后={fwd.step_h_front*100:.1f}/{fwd.step_h*100:.1f}cm "
              f"髋外展+0.01 (方向朝前)")

    controllers = build_gait_controllers(
        gait_cfg,
        natural_active=natural_active,
        natural_params=natural_params,
        walk_params=walk_params,
        jump_params=jump_params,
        no_spine=no_spine,
    )
    (stand, trot_fwd, trot_bwd, pace_fwd, pace_bwd,
     nat_fwd, walk_fwd, jump_fwd) = controllers.as_tuple()

    print(f"[转向层] 步幅差={gait_cfg.turn_amp_diff*100:.1f}cm "
          f"跨步={gait_cfg.turn_y_amp*100:.1f}cm "
          f"平滑={gait_cfg.turn_smooth:.3f} 腰yaw={gait_cfg.turn_waist_yaw:.2f} "
          f"(独立于前进参数)")
    print(f"[走+转] 边走边转权限×{drive.cruise_turn_scale:.2f} "
          f"蟹步增益={drive.cruise_turn_yamp:.2f} "
          f"| 转向符号={drive.turn_sign:+.0f} "
          f"腰符号={gait_cfg.waist_yaw_turn_sign:+.0f}")

    if trot_flag and not natural_active:
        start_mode = RobotMode.TROT
    else:
        start_mode = RobotMode.STAND

    fsm = build_runtime_fsm(
        controllers, drive,
        fwd=fwd,
        natural_configured=bool(natural_soft or natural_active),
        natural_walk=natural_walk,
        natural_jump=natural_jump,
        start_mode=start_mode,
        height=gait_cfg.height,
    )
    safety = build_safety_supervisor()
    if natural_jump:
        walk_name = "Jump"
    elif natural_walk:
        walk_name = "NaturalWalk"
    elif fsm.walk_is_natural and natural_soft:
        walk_name = "NaturalSoftTrot"
    elif fsm.walk_is_natural:
        walk_name = "NaturalTrot"
    else:
        walk_name = "StableTrot"
    print(f"[fsm] 起始模式={fsm.mode.value}  "
          f"摇杆/空格 走= {walk_name}")

    imu_ctrl = build_imu_attitude_controller(imu_cfg, load_trim_cal=load_trim_cal)

    return WalkControlStack(
        controllers=controllers,
        stand=stand,
        trot_fwd=trot_fwd,
        trot_bwd=trot_bwd,
        pace_fwd=pace_fwd,
        pace_bwd=pace_bwd,
        nat_fwd=nat_fwd,
        walk_fwd=walk_fwd,
        jump_fwd=jump_fwd,
        fsm=fsm,
        safety=safety,
        imu_ctrl=imu_ctrl,
        fwd=fwd,
        natural_active=natural_active,
        start_mode=start_mode,
    )
