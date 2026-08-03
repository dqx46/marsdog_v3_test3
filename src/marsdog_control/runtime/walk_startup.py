"""Walk startup script: CLI validation, presets, RuntimeState feature wiring.

Config boundary (single source):

1. Validate bench / direction-test knobs on the CLI ``Namespace`` (boundary).
2. Apply natural presets onto ``Namespace`` (boundary only).
3. ``bootstrap_runtime_config`` → required ``RuntimeConfig`` (FATAL if missing).
4. Wire ``RuntimeState`` + ``WalkStartupContext`` from typed config / residual CLI.

After this function returns, hot-path code must not re-read migrated ``args`` fields.
"""

from __future__ import annotations

import math
import os
import sys
from dataclasses import dataclass
from typing import Any, Optional

from marsdog_control.apps.walk_cli import apply_preset_preserving_cli
from marsdog_control.compat import ensure_legacy_path, project_root
from marsdog_control.config.gait_tuning import GAIT, print_tuning_banner
from marsdog_control.config.real_patches import print_patch_banner
from marsdog_control.config.schema import RuntimeConfig
from marsdog_control.motion.gait_recipes import (
    NATURAL_SOFT_TROT_REAL,
    NATURAL_SOFT_TROT_WBC,
    NATURAL_TROT_REAL,
    NATURAL_WALK_REAL,
    NATURAL_WALK_WBC,
    JUMP_REAL,
    JUMP_WBC,
)
from marsdog_control.motion.tarsus_bench import TarsusBenchConfig
from marsdog_control.runtime.walk_state import WalkRuntimeState

ensure_legacy_path()

from marsdog_control.motion import gait_controller  # noqa: E402
from marsdog_control.motion import kinematics  # noqa: E402


@dataclass
class WalkStartupContext:
    """Result of successful pre-bring-up startup preparation."""

    args: Any
    runtime_config: RuntimeConfig
    runtime_state: WalkRuntimeState
    natural_active: bool
    natural_soft: bool
    natural_params: dict
    natural_walk: bool
    walk_params: dict
    natural_jump: bool
    jump_params: dict
    trot_flag: bool
    no_spine: bool
    joint_direction_test: bool
    hip_abd_test: bool
    leg_pitch_test: bool
    calf_pitch_test: bool
    dm_active: bool
    imu_softstart_s: float
    trim_ramp_s: float
    imu_phase_gate: bool
    phase_td_gain: float
    phase_swing_gain: float
    td_imu_freeze_i: bool
    imu_slew_m_s: float
    # scalars resolved once at startup so assemble_walk_loop_context() never
    # has to read `args` directly (single seam: CLI -> here -> loop assembly).
    fade_s: float
    no_imu: bool
    ff_decouple: bool
    auto_trim: bool
    ramp_s: float
    leg_pitch_test_amp_rad: float
    calf_pitch_test_amp_rad: float
    front_foot_track_deg: float
    x_shift: float
    capture_lie_pose: bool
    logging_enabled: bool
    gamepad_enabled: bool
    tail_enabled: bool
    bench_tarsus_side: Optional[str]
    bench_cfg: Optional[TarsusBenchConfig]


def emit_typed_config(args) -> Optional[RuntimeConfig]:
    """Build/validate/print typed RuntimeConfig (one-way; no args write-back).

    Returns ``None`` on fatal validation / build failure (caller must abort).
    """
    _root = str(project_root())
    for _p in (_root, os.path.join(_root, "src")):
        if _p not in sys.path:
            sys.path.insert(0, _p)

    from marsdog_control.config import bootstrap_runtime_config

    result = bootstrap_runtime_config(args)
    if result.fatal or result.config is None:
        return None
    return result.config


def prepare_walk_startup(
    args,
    *,
    runtime_state: WalkRuntimeState,
    joint_gains: dict,
) -> Optional[WalkStartupContext]:
    """Validate CLI, apply presets, wire RuntimeState features + banner.

    Returns ``None`` when startup must abort (FATAL already printed).
    """
    try:
        args._bench_freqs = [
            float(v.strip()) for v in args.bench_tarsus_frequencies.split(",")
            if v.strip()
        ]
    except ValueError:
        print("[FATAL] --bench-tarsus-frequencies 必须是逗号分隔数字")
        return None
    if args.bench_tarsus_side:
        if not (args.natural_trot or args.natural_soft_trot):
            print("[FATAL] tarsus 地面扫频必须配合 --natural-soft-trot")
            return None
        if not 0.0 < args.bench_tarsus_amp_deg <= 2.0:
            print("[FATAL] 地面扫频幅度必须在 (0,2.0]°")
            return None
        if not args._bench_freqs or any(f <= 0.0 or f > 3.0 for f in args._bench_freqs):
            print("[FATAL] 地面扫频频率必须在 (0,3]Hz")
            return None
        args.no_gamepad = True
        print(f"[BENCH] 地面小扰动: {args.bench_tarsus_side.upper()} "
              f"±{args.bench_tarsus_amp_deg:.2f}°, 频率={args._bench_freqs}Hz")

    direction_test_count = sum(
        v is not None
        for v in (args.hip_abd_test, args.leg_pitch_test, args.calf_pitch_test)
    )
    if direction_test_count > 1:
        print("[FATAL] --hip-abd-test / --leg-pitch-test / --calf-pitch-test 不能同时使用")
        return None
    hip_abd_test = args.hip_abd_test is not None
    leg_pitch_test = args.leg_pitch_test is not None
    calf_pitch_test = args.calf_pitch_test is not None
    joint_direction_test = hip_abd_test or leg_pitch_test or calf_pitch_test
    if hip_abd_test:
        if not 0.0 < args.hip_abd_test <= 0.20:
            print("[FATAL] --hip-abd-test RAD 必须在 (0, 0.20] rad 内；建议 0.08")
            return None
        args.hip_abd = args.hip_abd_test
        print(f"[hip-abd-test] 启用: 主控站姿 hip_abd={args.hip_abd:.3f}rad "
              f"({math.degrees(args.hip_abd):.1f}°); 仅 ID 2/6/9/12 会运动")
    if leg_pitch_test:
        if not 0.0 < args.leg_pitch_test <= 0.30:
            print("[FATAL] --leg-pitch-test RAD 必须在 (0, 0.30] rad 内；建议 0.20")
            return None
        print(f"[leg-pitch-test] 启用: 幅度={args.leg_pitch_test:.3f}rad "
              f"({math.degrees(args.leg_pitch_test):.1f}°); "
              f"前腿大腿向前(URDF-{args.leg_pitch_test:.2f}) / 后腿大腿向后(URDF+{args.leg_pitch_test:.2f}); "
              f"仅 ID 1/5/10/13 会运动")
    if calf_pitch_test:
        if not 0.0 < args.calf_pitch_test <= 0.30:
            print("[FATAL] --calf-pitch-test RAD 必须在 (0, 0.30] rad 内；建议 0.20")
            return None
        print(f"[calf-pitch-test] 启用: 幅度={args.calf_pitch_test:.3f}rad "
              f"({math.degrees(args.calf_pitch_test):.1f}°); "
              f"四个小腿都向前(URDF-{args.calf_pitch_test:.2f}); "
              f"仅 ID 3/7/11/14 会运动")

    natural_soft = bool(args.natural_soft_trot)
    natural_walk = bool(getattr(args, "natural_walk", False))
    natural_jump = bool(getattr(args, "jump", False))
    # --jump 覆盖 --natural-walk
    if natural_jump and natural_walk:
        print("[jump] --jump 覆盖 --natural-walk；Walk 路径不激活")
        natural_walk = False
    # SoftTrot / NaturalTrot 仍算 natural_active；Walk/Jump 也启 tarsus。
    natural_active = bool(
        args.natural_trot or natural_soft or natural_walk or natural_jump
    )
    if natural_active:
        print("[tarsus] 约定: 上电前已手动掰到达妙硬限位零点(无 CLI 确认开关)")

    # SoftTrot 预设仍灌进 args（默认路径）；Walk/Jump 用独立 params，不覆盖 SoftTrot 配方数字。
    # SoftTrot 统一用 NATURAL_SOFT_TROT_WBC 大步几何（amp/period/stance），
    # 不再因未开 --wbc 退回 REAL 小碎步；--wbc 只切换控制器(WBC+MPC vs VMC)。
    if natural_soft:
        natural_params = dict(NATURAL_SOFT_TROT_WBC)
    else:
        natural_params = dict(NATURAL_TROT_REAL)
    print(
            f"[nat] SoftTrot 大步预设: amp={natural_params['amp_front']*100:.1f}/"
            f"{natural_params['amp_rear']*100:.1f}cm "
            f"front_scale={natural_params.get('fwd_front_amp_scale', 1.0):.2f} "
            f"period={natural_params['period']:.2f}s"
        )
    overridden: list = []
    # SoftTrot 默认开启时始终灌 Soft 预设（即使同时 --jump/--natural-walk，nat_fwd 仍要 Soft）。
    if natural_soft or bool(args.natural_trot):
        overridden = apply_preset_preserving_cli(args, natural_params)
        if overridden:
            print("[nat] 显式 CLI 覆盖预设: " + ", ".join(overridden))
        T = float(getattr(args, "nat_period", natural_params.get("period", 0.87)))
        st = float(getattr(args, "stance", natural_params.get("stance", 0.56)))
        if T > 1e-6:
            print(
                f"[cadence] SoftTrot T={T:.3f}s  f={1.0 / T:.2f} Hz  "
                f"stance={st:.2f} (swing={1.0 - st:.2f})"
            )
        com_m = float(getattr(args, "com_shift_m",
                              natural_params.get("com_shift_m", 0.0)))
        if abs(com_m) > 1e-6:
            blend = float(getattr(args, "com_shift_blend",
                                  natural_params.get("com_shift_blend", 0.12)))
            sign_note = "正=FL+RR→右(反相优)" if com_m > 0 else "负=旧同号方向"
            print(
                f"[COM] SoftTrot 横向移重 com_shift={com_m*1000:+.1f}mm "
                f"blend={blend:.2f} ({sign_note}; --com-shift 0 关闭)"
            )
        else:
            sway = float(getattr(args, "lateral_sway",
                                 natural_params.get("lateral_sway", 0.0)))
            print(
                f"[COM] SoftTrot 横向移重 OFF "
                f"(回退 lateral_sway={sway*1000:.1f}mm 半正弦)"
            )

    walk_params = dict(
        NATURAL_WALK_WBC if bool(getattr(args, "wbc", False)) else NATURAL_WALK_REAL
    )
    if natural_walk:
        print(
            f"[walk] NaturalWalk 预设: amp={walk_params['amp_front']*100:.1f}/"
            f"{walk_params['amp_rear']*100:.1f}cm "
            f"period={walk_params['period']:.2f}s stance={walk_params['stance']:.2f} "
            f"sway={walk_params['lateral_sway']*1000:.1f}mm "
            f"spine={walk_params['spine_yaw_deg']:.1f}/"
            f"{walk_params['spine_roll_deg']:.1f}°"
        )

    jump_params = dict(
        JUMP_WBC if bool(getattr(args, "wbc", False)) else JUMP_REAL
    )
    if natural_jump:
        print(
            f"[jump] Jump 预设: crouch={jump_params['crouch_depth']*1000:.0f}mm "
            f"push={jump_params['push_s']:.2f}s flight={jump_params['flight_s']:.2f}s "
            f"push_vz={jump_params['push_vz']:.2f}m/s "
            f"kp_z={jump_params.get('kp_base_z', 80.0):.0f} "
            f"(力控增益挂 JumpController, 不灌 Soft args)"
        )

    # Preset may mutate args; bootstrap after preset so RuntimeConfig sees final values.
    runtime_config = emit_typed_config(args)
    if runtime_config is None:
        return None

    features = runtime_config.features
    gait = runtime_config.gait
    control = runtime_config.control
    imu = runtime_config.imu
    dm_cfg = runtime_config.dm_tarsus
    safety = runtime_config.safety
    devtools = runtime_config.devtools

    # Bench forced no_gamepad on args before bootstrap — re-sync feature view.
    if args.bench_tarsus_side:
        # features was built with no_gamepad=True already if args mutated before
        # bootstrap; if somehow not, treat gamepad as off for bench.
        gamepad_enabled = False
    else:
        gamepad_enabled = features.gamepad_enabled

    runtime_state.joint_gains = joint_gains

    # Derived once: natural OR joint-direction test drives tarsus.
    dm_active = bool(features.dm_tarsus_active or joint_direction_test)
    if dm_active:
        lead_max_s = min(0.10, max(0.0, float(getattr(args, "nat_period", GAIT.nat_period)) * 0.15))
        runtime_state.apply_dm_tarsus(
            active=True,
            kp_fl=dm_cfg.kp_fl,
            kp_fr=dm_cfg.kp_fr,
            kd_fl=dm_cfg.kd_fl,
            kd_fr=dm_cfg.kd_fr,
            lead_fl_s=dm_cfg.lead_fl_s,
            lead_fr_s=dm_cfg.lead_fr_s,
            lead_max_s=lead_max_s,
            lead_max_rad=dm_cfg.lead_max_rad,
            dq_feedforward=features.dm_dq_feedforward_enabled,
            dq_max_rps=dm_cfg.dq_max_rad_s,
        )
        mode_label = (
            "方向测试" if joint_direction_test
            else ("NaturalSoftTrot" if natural_soft else "NaturalTrot")
        )
        print(f"[tarsus] 主动驱动已启用({mode_label}), "
              f"FL kp/kd={runtime_state.dm.kp_by_id[4]:.0f}/{runtime_state.dm.kd_by_id[4]:.1f}, "
              f"FR={runtime_state.dm.kp_by_id[8]:.0f}/{runtime_state.dm.kd_by_id[8]:.1f}, "
              f"lead={runtime_state.dm.reference_lead_s[4]*1000:.0f}/"
              f"{runtime_state.dm.reference_lead_s[8]*1000:.0f}ms, "
              f"lead限幅=±{math.degrees(runtime_state.dm.reference_lead_max_rad):.1f}°, "
              f"dq_ff={'ON' if runtime_state.dm.dq_feedforward else 'OFF'}"
              f"(±{runtime_state.dm.dq_max_rps:.1f}rad/s)")
    else:
        runtime_state.dm.active = False

    runtime_state.apply_control_features(
        leg_kp_scale=control.leg_kp_scale,
        var_impedance=features.variable_impedance_enabled,
        td_kp_scale=control.td_kp_scale,
        swing_kp_scale=control.swing_kp_scale,
        td_window=control.td_window_s,
        gravity_comp=features.gravity_comp_enabled,
        gravity_scale=control.gravity_scale,
        vmc_enabled=features.vmc_enabled,
        wbc_enabled=features.wbc_enabled,
    )
    from marsdog_control.config.gains import BRAND_GAIN_SCALE, JOINT_GAINS
    # Prefer the table wired into this session (sim → SIM_JOINT_GAINS).
    active_gains = runtime_state.joint_gains or JOINT_GAINS
    print(
        "[gains] brand scales "
        + ", ".join(
            f"{m}=kp×{s['kp']:.2f}/kd×{s['kd']:.2f}"
            for m, s in BRAND_GAIN_SCALE.items()
        )
    )
    fc, rc = active_gains.get("fl_calf", {}), active_gains.get("fr_calf", {})
    fr = active_gains.get("fl_thigh_roll", {})
    print(
        f"[gains] active calf L/R = "
        f"{fc.get('kp', float('nan')):.0f}/{fc.get('kd', float('nan')):.1f} | "
        f"{rc.get('kp', float('nan')):.0f}/{rc.get('kd', float('nan')):.1f} "
        f"thigh_roll={fr.get('kp', float('nan')):.0f}/{fr.get('kd', float('nan')):.1f} "
        f"(leg_kp_scale overlay={runtime_state.leg_kp_scale:.2f})"
    )
    if abs(runtime_state.leg_kp_scale - 1.0) > 1e-6:
        print(f"[P3] 临时 leg_kp_scale 叠加 = {runtime_state.leg_kp_scale:.2f} "
              f"(非品牌专属增益)")
    if runtime_state.impedance.enabled:
        print(f"[柔顺A] 相位可变阻抗开启: 触地kp={runtime_state.impedance.td_kp_scale:.2f} "
              f"摆动kp={runtime_state.impedance.swing_kp_scale:.2f} "
              f"触地窗口={runtime_state.impedance.td_window:.2f} (支撑中期=1.0)")
    
    if runtime_config.features.vmc_enabled:
        print(f"[实验] 解耦 VMC 开启: 使用 Z/Roll 雅可比重算 trq_ff。腿部已软化 (kp_scale={runtime_state.leg_kp_scale:.2f})!")
    elif runtime_state.gravity_comp:
        print(f"[柔顺B] 重力补偿前馈开启: scale={runtime_state.gravity_scale:.2f} "
              f"(腿部 pitch 关节 trq_ff 由 τ_g(q) 计算, 替换静态值)")

    td_imu_freeze_i = bool(getattr(args, "td_imu_freeze_i", False))
    imu_slew_m_s = max(0.0, control.imu_slew_m_s)
    imu_softstart_s = max(0.0, imu.softstart_s)
    trim_ramp_s = max(3.0, gait.ramp_s * 1.5, imu_softstart_s)
    if imu_softstart_s > 1e-6:
        print(f"[SS] IMU修正软启动: 反馈{imu_softstart_s:.1f}s / 配平{trim_ramp_s:.1f}s "
              f"内 0→1(smoothstep), 期间冻结积分+auto-trim → 消除起步'起飞'")
    if td_imu_freeze_i or imu_slew_m_s > 1e-9:
        print("[E] IMU冲击保护开启: "
              f"td_freeze_i={'ON' if td_imu_freeze_i else 'OFF'}  "
              f"slew={imu_slew_m_s*1000:.0f}mm/s")

    imu_phase_gate = imu.phase_gate_enabled
    phase_td_gain = imu.phase_td_gain
    phase_swing_gain = imu.phase_swing_gain
    if imu_phase_gate:
        print(f"[F] IMU相位门控: 触地/离地×{phase_td_gain:.2f}  "
              f"摆动×{phase_swing_gain:.2f}  支撑中期×1.0 "
              f"(换腿窗口少打反馈, 实机/仿真同源)")

    if getattr(args, "abd_legacy", False):
        gait_controller.ABD_LEGACY = True
        kinematics.ABD_LEGACY = True
        print("[P1] 外展方向 = LEGACY(修正前): 翻转 fl_thigh_roll/rl_hip/rr_hip")

    swing_level = float(getattr(args, "swing_level", 0.0))
    gait_controller.SWING_LEVEL = max(0.0, min(1.0, swing_level))
    if gait_controller.SWING_LEVEL > 1e-6:
        print(f"[P2] 摆动腿 IMU 预调平权重 = {gait_controller.SWING_LEVEL:.2f}")

    gait_controller.SMOOTH_GAIT = features.smooth_gait_enabled
    if gait_controller.SMOOTH_GAIT:
        print("[C] 平滑步态开启: 支撑相匀速 + 摆动Hermite速度匹配 (消除一冲一冲)")

    anti_roll = float(getattr(args, "anti_roll", GAIT.anti_roll))
    if abs(anti_roll - GAIT.anti_roll) > 1e-6:
        print(f"[C] anti_roll = {anti_roll*1000:.1f}mm")

    print_tuning_banner(
        natural_soft=natural_soft,
        natural_active=natural_active,
        overridden=overridden,
        height=gait.body_height_m,
        period=gait.period_s,
        amp_front=gait.amp_front_m,
        amp_rear=gait.amp_rear_m,
        step_h=gait.step_height_m,
        stance=gait.stance_ratio,
    )
    print_patch_banner(args)

    bench_cfg = None
    bench_side = devtools.bench_tarsus_side or args.bench_tarsus_side
    if bench_side:
        bench_cfg = TarsusBenchConfig(
            side=bench_side,
            frequencies_hz=args._bench_freqs,
            amplitude_rad=math.radians(args.bench_tarsus_amp_deg),
            cycles=args.bench_tarsus_cycles,
            settle_s=args.bench_tarsus_settle_s,
            max_error_deg=math.degrees(safety.bench_max_error_rad),
            max_tilt_deg=math.degrees(safety.bench_max_tilt_rad),
            max_torque_nm=safety.bench_max_torque_nm,
            kp_by_id=runtime_state.dm.kp_by_id,
        )

    return WalkStartupContext(
        args=args,
        runtime_config=runtime_config,
        runtime_state=runtime_state,
        natural_active=natural_active,
        natural_soft=natural_soft,
        natural_params=natural_params,
        natural_walk=natural_walk,
        walk_params=walk_params,
        natural_jump=natural_jump,
        jump_params=jump_params,
        trot_flag=bool(getattr(args, "trot", False)),
        no_spine=bool(getattr(args, "no_spine", False)),
        joint_direction_test=joint_direction_test,
        hip_abd_test=hip_abd_test,
        leg_pitch_test=leg_pitch_test,
        calf_pitch_test=calf_pitch_test,
        dm_active=dm_active,
        imu_softstart_s=imu_softstart_s,
        trim_ramp_s=trim_ramp_s,
        imu_phase_gate=imu_phase_gate,
        phase_td_gain=phase_td_gain,
        phase_swing_gain=phase_swing_gain,
        td_imu_freeze_i=td_imu_freeze_i,
        imu_slew_m_s=imu_slew_m_s,
        fade_s=gait.fade_s,
        no_imu=not features.imu_enabled,
        ff_decouple=features.ff_decouple_enabled,
        auto_trim=imu.auto_trim_enabled,
        ramp_s=gait.ramp_s,
        leg_pitch_test_amp_rad=float(args.leg_pitch_test or 0.0),
        calf_pitch_test_amp_rad=float(args.calf_pitch_test or 0.0),
        front_foot_track_deg=float(getattr(
            args, "front_foot_track_deg", GAIT.front_foot_track_deg)),
        x_shift=float(getattr(args, "x_shift", GAIT.x_shift)),
        capture_lie_pose=bool(devtools.capture_lie_pose),
        logging_enabled=features.logging_enabled,
        gamepad_enabled=gamepad_enabled,
        tail_enabled=features.tail_enabled,
        bench_tarsus_side=bench_side,
        bench_cfg=bench_cfg,
    )


__all__ = [
    "WalkStartupContext",
    "emit_typed_config",
    "prepare_walk_startup",
]
