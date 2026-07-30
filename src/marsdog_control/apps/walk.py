#!/usr/bin/env python3
"""Marsdog walk.py — 最简步态控制脚本 (Phase 1 重写)

纯开环运动学步态, 无 IMU 闭环, 无速度前馈。
用于验证 IK 和基础 trot 轨迹是否正确。

用法:
    python3 walk.py                  # 站立模式启动
    python3 walk.py --trot           # 直接进入 trot 模式

手柄控制 (Xbox / PS2):
    左摇杆 Y    — 前进/后退
    右摇杆 X    — 转向 (Phase 4 实现)
    START       — 切换站立 / Trot
    SELECT / B  — 紧急停止并退出
    LB / RB     — 减慢 / 加快步频

键盘控制:
    SPACE / s   — 切换站立 / Trot
    + / =       — 加快步频
    - / _       — 减慢步频
    u / d       — 体高 +/- 1cm
    f / v       — 摆幅 +/- 5mm
    p           — 打印电机状态
    q / ESC     — 安全退出
"""

import math
import os
import time

from marsdog_control.compat import ensure_legacy_path, legacy_dir

ensure_legacy_path()
_APP_RESOURCE_DIR = str(legacy_dir())

from marsdog_control.hardware.motors.lingzu import MotorLz
from marsdog_control.hardware.motors.evo import MotorEvo
from marsdog_control.hardware.motors.damiao import MotorDamiao
from marsdog_control.hardware.motors.incos import MotorIncos
from marsdog_control.control import gravity_comp as gcomp
from marsdog_control.apps.walk_cli import (  # noqa: E402
    apply_preset_preserving_cli,  # noqa: F401 — re-export for parity / tests
    parse_args,
)
from marsdog_control.runtime.walk_controllers import (  # noqa: E402
    assemble_walk_control_stack,
)
from marsdog_control.runtime.walk_startup import (  # noqa: E402
    prepare_walk_startup as _prepare_walk_startup,
)
# Re-exported for mocap_to_real/test_imu_dm_pipeline.py (walk.front_foot_pitch_from_motor).
from marsdog_control.motion.kinematics import front_foot_pitch_from_motor  # noqa: F401
from marsdog_control.config.joints import (
    JOINT_MAP, JOINT_BY_ID, JOINT_BY_NAME as JBN,
    ALL_IDS, LZ_CAN_IDS, LZ_SERIAL_IDS, EVO_CAN_IDS, DM_CAN_IDS, INCOS_CAN_IDS,
    DEFAULT_LZ_KP, DEFAULT_LZ_KD,
    DEFAULT_EVO_KP, DEFAULT_EVO_KD,
    DEFAULT_DM_KP, DEFAULT_DM_KD,
    DM_MASTER_ID_BY_SLAVE,
)
from marsdog_control.config.bus_config import (
    LZ_CAN1_DEVICE, EVO_CAN0_DEVICE, LZ_SERIAL_DEVICE,
    DM_CAN_DEVICE, INCOS_CAN_DEVICE, BAUD, IMU_DEVICE, IMU_BAUD, GAMEPAD_DEVICE)
from marsdog_control.hardware.sensors.imu_wt901 import ImuWT901
from marsdog_control.control.balance import (
    RuntimeBalanceConfig as _RuntimeBalanceConfig,
    RuntimeBalanceController as _RuntimeBalanceController,
)
from marsdog_control.control.executor import (
    CommandExecutor as _CommandExecutor,
    ExecutorConfig as _ExecutorConfig,
)
from marsdog_control.motion.tarsus_bench import (
    TarsusBenchConfig as _TarsusBenchConfig,
    TarsusBenchRuntime as _TarsusBenchRuntime,
    tarsus_bench_reference as _tarsus_bench_reference_impl,
)
from marsdog_control.runtime.lie_down_session import LieDownSession as _LieDownSession
from marsdog_control.runtime.status import RuntimeStatusDisplay as _RuntimeStatusDisplay
# mode_str re-exported for mocap_to_real/test_imu_dm_pipeline.py (walk._mode_str).
from marsdog_control.runtime.walk_loop import mode_str as _mode_str  # noqa: F401
from marsdog_control.runtime.walk_state import WalkRuntimeState as _WalkRuntimeState
from marsdog_control.runtime.walk_bringup import (
    bringup_imu as _bringup_imu,
    bringup_motors_and_board as _bringup_motors_and_board,
)
# bring-up 后编排(fade/IMU 校准/输入/loop/shutdown)整体在 walk_session。
from marsdog_control.runtime.walk_session import (
    WalkSessionContext as _WalkSessionContext,
    run_walk_session,
)
from marsdog_control.hardware.input.gamepad import Gamepad
from marsdog_control.hardware.behavior.audio import bark_with_mouth
from marsdog_control.hardware.behavior.tail import TailController
from marsdog_control.motion.pose_contract import (
    assert_foot_tracking_requires_tarsus,
    assert_stand_matches_gait_start,
)
# ─────────────────────────────────────────────────────────────────────────────
# 常量
# ─────────────────────────────────────────────────────────────────────────────
GP_DEADZONE       = 0.12
GP_TROT_THRESHOLD = 0.15
GP_PERIOD_STEP    = 0.05
GP_LIE_DOWN_LT_THRESHOLD = 0.60
GP_BARK_RT_THRESHOLD = 0.60

CONTROL_HZ   = 200.0

# 权威增益表已迁入 config/gains.py；此处保留同名绑定供旧引用/热改兼容。
from marsdog_control.config.gains import JOINT_GAINS  # noqa: E402

# CLI: parse_args / apply_preset_preserving_cli → apps/walk_cli.py (re-exported above)

# ─────────────────────────────────────────────────────────────────────────────
# 键盘 / 输入
# ─────────────────────────────────────────────────────────────────────────────

from marsdog_control.input.user_input import (  # noqa: E402
    InputState as _InputState,
    KeyReader,
)
from marsdog_control.input.hal import WalkInputHAL as _WalkInputHAL  # noqa: E402


# ─────────────────────────────────────────────────────────────────────────────
# 电机
# ─────────────────────────────────────────────────────────────────────────────

_CAN1_JOINTS   = [j for j in JOINT_MAP if j.mtype == "lz" and j.bus == "lz_can_a"]
_SERIAL_JOINTS = [j for j in JOINT_MAP if j.mtype == "lz" and j.bus == "lz_can_b"]
_EVO_JOINTS    = [j for j in JOINT_MAP if j.mtype == "evo"]
_DM_JOINTS     = [j for j in JOINT_MAP if j.mtype == "dm"]
_INCOS_JOINTS  = [j for j in JOINT_MAP if j.mtype == "incos"]

# rl_tarsus/rr_tarsus (id 22/23) 在 joint_config.py 里 bus="none" — 这两个是被动
# (mimic)关节, 物理上根本没有独立电机, 不接在任何总线上。凡是要查询/操作"真实电机
# 硬件状态"(enable/disable/position/fault)的地方都必须用这份列表, 否则会去查一个
# 从未连接过的 id 的 is_enabled/is_connected 数组项(恒为 False), 报出虚假的
# "disabled"/"离线" 警告。构造步态目标角度(IK/stand pose)不受影响, 那些值本来就
# 不会被 send_all() 发出去(所有总线路由列表都按 bus 名精确匹配, "none" 匹配不到)。
_REAL_JOINTS = [j for j in JOINT_MAP if j.bus != "none"]
# [解耦] 趴下姿态 I/O 已迁入 src/marsdog_control/motion/lie_down.py;
# walk 保留同名薄封装与默认路径, 调用点不变。
from marsdog_control.motion.lie_down import (  # noqa: E402
    NON_BODY_LIE_MOTOR_IDS as _NON_BODY_LIE_MOTOR_IDS,
    LIE_DOWN_TARGETS_RAD,
    default_lie_down_pose_path as _default_lie_down_pose_path,
    default_sit_pose_path as _default_sit_pose_path,
    load_lie_down_pose_from_log as _load_lie_down_pose_from_log,
    save_lie_down_pose as _save_lie_down_pose,
    load_lie_down_pose as _load_lie_down_pose,
    build_lie_down_target as _build_lie_down_target,
    build_sit_target as _build_sit_target,
)

_LIE_DOWN_POSE_PATH = _default_lie_down_pose_path(
    _APP_RESOURCE_DIR)
_SIT_POSE_PATH = _default_sit_pose_path(
    _APP_RESOURCE_DIR)


def load_lie_down_pose_from_log(path: str) -> dict:
    return _load_lie_down_pose_from_log(path)


def save_lie_down_pose(path: str, pose: dict) -> None:
    return _save_lie_down_pose(path, pose)


def load_lie_down_pose(path: str = _LIE_DOWN_POSE_PATH) -> dict:
    return _load_lie_down_pose(path)

# 前腿 tarsus (达妙, 4=fl_tarsus / 8=fr_tarsus): 默认先固定在开机读到的角度,
# 不参与步态/IK 计算, 只用低刚度 MIT 保持不动 (今天验证过的安全默认行为)。
# 在 main() 里读到初始位置后填充; 权威副本随 WalkRuntimeState.dm.fixed_targets,
# 此处保留同名 dict 供 read_state / robot_hw 等旧调用点按引用共享。
DM_FIXED_TARGETS = {}

# [Phase C] tarsus 主动控制 / 各类 DM kp/kd/lead/dq 旋钮已全部迁入
# WalkRuntimeState.dm(由 walk_startup.apply_dm_tarsus 从 CLI/RuntimeConfig 装配),
# 不再有模块级镜像全局。唯一权威来源: parse_args() default → schema.py → WalkRuntimeState。


def tarsus_bench_reference(elapsed_s, frequencies_hz, amplitude_rad,
                           cycles=3.0, settle_s=2.0):
    return _tarsus_bench_reference_impl(
        elapsed_s, frequencies_hz, amplitude_rad, cycles, settle_s)


# [Phase C] leg_kp_scale / 相位可变阻抗 / 重力补偿等控制旋钮已全部迁入
# WalkRuntimeState(由 walk_startup.apply_control_features 从 CLI/RuntimeConfig 装配),
# 不再有模块级镜像全局。executor 层拥有自己的 _LEG_MOTOR_IDS / gravity_trq 真源。


# [解耦] 硬件 I/O / 诊断 / 日志实现体已整体下沉到
# src/marsdog_control/runtime/walk_services.py (WalkServices)。main() 直接用 svc.xxx;
# walk 仅保留 send_all / read_state / check_motors / find_lz_recoverable_faults 少数同名壳,
# 维持 parity/legacy(mocap_to_real 工具) patch/import 面。
from marsdog_control.hardware.board import RkMotorBoard as _RkMotorBoard  # noqa: E402
from marsdog_control.runtime.walk_services import WalkServices as _WalkServices  # noqa: E402

_RUNTIME = None  # WalkRuntimeState — authoritative live knobs
_SVC = None      # WalkServices — owns board / diagnostics / logging I/O


def _ensure_runtime() -> _WalkRuntimeState:
    global _RUNTIME
    if _RUNTIME is None:
        _RUNTIME = _WalkRuntimeState(joint_gains=JOINT_GAINS)
    return _RUNTIME


def _services() -> _WalkServices:
    global _SVC
    if _SVC is None:
        _SVC = _WalkServices(
            runtime_state=_ensure_runtime(),
            real_joints=_REAL_JOINTS,
            resource_dir=_APP_RESOURCE_DIR,
            control_hz=CONTROL_HZ,
            clock=time,
        )
    return _SVC


def build_lie_down_target(online, pose_path: str = _LIE_DOWN_POSE_PATH) -> dict:
    return _build_lie_down_target(online, pose_path=pose_path)


def build_sit_target(online, pose_path: str = _SIT_POSE_PATH) -> dict:
    return _build_sit_target(online, pose_path=pose_path)


# ─────────────────────────────────────────────────────────────────────────────
# 软件示波器 / trim 标定 I/O
# ─────────────────────────────────────────────────────────────────────────────

from marsdog_control.io.scope import (  # noqa: E402
    start_scope as _start_scope_impl,
    stop_scope as _stop_scope,
)
from marsdog_control.io.trim_cal import (  # noqa: E402
    load_trim_cal as _load_trim_cal_impl,
    save_trim_cal as _save_trim_cal_impl,
)


def _start_scope(log_path, args):
    return _start_scope_impl(log_path, args, resource_dir=_APP_RESOURCE_DIR)


# ─────────────────────────────────────────────────────────────────────────────
# 固定控制管线的分层函数 (State / Input / Motion / Actuation)
# ─────────────────────────────────────────────────────────────────────────────

# [解耦] motion 层常量随实现迁入 src/marsdog_control/motion/motion_planner.py;
# 这里复用同一份, 主控日志/方向测试分支引用保持不变(单一真源在 src)。
# _RATELIMIT_IDS re-exported for mocap_to_real/test_imu_dm_pipeline.py.
from marsdog_control.motion.motion_planner import _RATELIMIT_IDS  # noqa: E402,F401


# [解耦] motion 层实现(目标生成/站立IMU调平/三类方向测试)已迁入
# src/marsdog_control/motion/motion_planner.py; walk 复用同一份代码, 主循环调用不变。
from marsdog_control.motion.motion_planner import (  # noqa: E402
    build_motion_target,
    build_hip_abduction_test_target,
    build_leg_pitch_direction_test_target,
    build_calf_pitch_direction_test_target,
    _apply_stand_imu_dz,
)


# ─────────────────────────────────────────────────────────────────────────────
# 主函数
# ─────────────────────────────────────────────────────────────────────────────

_TRIM_CAL_PATH = os.path.join(_APP_RESOURCE_DIR, "trim_cal.json")


def _load_trim_cal():
    return _load_trim_cal_impl(_TRIM_CAL_PATH)


def _save_trim_cal(roll_ff_mm, roll_trim_mm=0.0):
    return _save_trim_cal_impl(_TRIM_CAL_PATH, roll_ff_mm, roll_trim_mm)


def main(args=None):
    if args is None:
        args = parse_args()

    # ── 校验 / 预设 / RuntimeState 特性接线（runtime/walk_startup）────────
    startup = _prepare_walk_startup(
        args,
        runtime_state=_ensure_runtime(),
        joint_gains=JOINT_GAINS,
    )
    if startup is None:
        return
    args = startup.args
    runtime_state = startup.runtime_state
    runtime_config = startup.runtime_config
    imu_cfg = runtime_config.imu
    joint_direction_test = startup.joint_direction_test
    hip_abd_test = startup.hip_abd_test
    leg_pitch_test = startup.leg_pitch_test
    calf_pitch_test = startup.calf_pitch_test
    imu_softstart_s = startup.imu_softstart_s
    _trim_ramp_s = startup.trim_ramp_s
    imu_phase_gate = startup.imu_phase_gate
    phase_td_gain = startup.phase_td_gain
    phase_swing_gain = startup.phase_swing_gain
    td_imu_freeze_i = startup.td_imu_freeze_i
    imu_slew_m_s = startup.imu_slew_m_s

    # 硬件 I/O 服务(下发/只读/诊断/平滑/日志/关机)的唯一所有者; board 于 bring-up 后写入。
    svc = _services()

    # ── IMU + 电机 bring-up（已迁出 apps/walk）────────────────────────────
    imu, imu_ok = _bringup_imu(
        imu_cls=ImuWT901,
        imu_device=IMU_DEVICE,
        imu_baud=IMU_BAUD,
        angle_tau_s=imu_cfg.angle_tau_s,
        gyro_tau_s=imu_cfg.gyro_tau_s,
        require_imu=bool(startup.bench_tarsus_side),
    )
    if startup.bench_tarsus_side and not imu_ok:
        return

    hw = _bringup_motors_and_board(
        motor_lz_cls=MotorLz,
        motor_evo_cls=MotorEvo,
        motor_damiao_cls=MotorDamiao,
        motor_incos_cls=MotorIncos,
        board_cls=_RkMotorBoard,
        lz_serial_device=LZ_SERIAL_DEVICE,
        lz_can1_device=LZ_CAN1_DEVICE,
        evo_can0_device=EVO_CAN0_DEVICE,
        dm_can_device=DM_CAN_DEVICE,
        incos_can_device=INCOS_CAN_DEVICE,
        baud=BAUD,
        joint_map=JOINT_MAP,
        dm_joints=_DM_JOINTS,
        dm_master_id_by_slave=DM_MASTER_ID_BY_SLAVE,
        incos_can_ids=INCOS_CAN_IDS,
        joint_by_id=JOINT_BY_ID,
        all_ids=ALL_IDS,
        shutdown_motors=svc.shutdown_motors,
        clock=time,
    )
    if hw is None:
        return
    lz, evo, dm, incos = hw.lz, hw.evo, hw.dm, hw.incos
    svc.board = hw.board
    online = hw.online
    DM_FIXED_TARGETS.clear()
    DM_FIXED_TARGETS.update(hw.dm_fixed_targets)
    runtime_state.dm.fixed_targets = DM_FIXED_TARGETS
    runtime_state.board = hw.board
    hw.imu = imu
    hw.imu_ok = imu_ok

    # ── 读取当前位置 ───────────────────────────────────────────────────────
    print("[pos] 读取当前位置...")
    cur_pos = svc.read_positions(lz, evo, incos)
    # 补上达妙 tarsus 的真实开机位置(read_positions 不含 dm), 保证 DM_TARSUS_ACTIVE=True 时
    # 首次 fade-to-stand 的插值起点是电机实际角度, 而不是误当成 0.0 去斜坡。
    if dm is not None:
        cur_pos.update(DM_FIXED_TARGETS)
    if startup.capture_lie_pose:
        save_lie_down_pose(_LIE_DOWN_POSE_PATH, cur_pos)
        kept = sorted(mid for mid in cur_pos if mid not in _NON_BODY_LIE_MOTOR_IDS)
        print(f"[lie-down] 已保存当前电机位置为趴下姿势: {_LIE_DOWN_POSE_PATH}")
        print("[lie-down] 已排除头部/脖子 ID 15/16/17/18; 保存电机: "
              + ", ".join(str(mid) for mid in kept))
        svc.shutdown_motors(lz, evo, dm, incos)
        return
    direction_test_start = None
    direction_test_base = None

    # ── 日志 ──────────────────────────────────────────────────────────────
    log_file, log_writer, log_path = svc.setup_log(startup.logging_enabled, args)
    scope_proc = _start_scope(log_path, args)

    # 重心后移: 正 x_shift = 四脚整体向前 → 身体相对落脚点后移
    # （实际偏移在 gait_recipes.StandingPoseConfig.from_args 内叠加）
    if abs(startup.x_shift) > 1e-6:
        print(f"[COM] 落脚点整体X偏移 = {startup.x_shift*1000:+.0f}mm (正=脚前移/重心后移)")

    # ── 步态 / FSM / Safety / IMU（runtime/walk_controllers 工厂）──────────
    stack = assemble_walk_control_stack(
        args,
        natural_active=startup.natural_active,
        natural_params=startup.natural_params,
        gp_trot_threshold=GP_TROT_THRESHOLD,
        gp_deadzone=GP_DEADZONE,
        natural_soft=startup.natural_soft,
        natural_walk=getattr(startup, "natural_walk", False),
        walk_params=getattr(startup, "walk_params", None),
        natural_jump=getattr(startup, "natural_jump", False),
        jump_params=getattr(startup, "jump_params", None),
        trot_flag=startup.trot_flag,
        no_spine=startup.no_spine,
        load_trim_cal=_load_trim_cal,
    )
    stand = stack.stand

    if getattr(startup, "natural_jump", False):
        startup_gait = stack.jump_fwd
    elif getattr(startup, "natural_walk", False):
        startup_gait = stack.walk_fwd
    elif startup.natural_active:
        startup_gait = stack.nat_fwd
    elif startup.trot_flag:
        startup_gait = stack.trot_fwd
    else:
        startup_gait = None
    if startup_gait is not None:
        assert_foot_tracking_requires_tarsus(
            front_foot_track_deg=startup.front_foot_track_deg,
            stand_controller=stand,
            context="walk startup",
        )
        assert_stand_matches_gait_start(
            stand,
            startup_gait,
            context=f"walk startup {startup_gait.__class__.__name__}",
        )

    # ── bring-up 后编排(方向预检 → fade → IMU 校准 → 输入 → loop → shutdown)
    #    已整体迁入 runtime/walk_session.py; main 只负责装配 session ctx 并交棒。
    #    parity harness patch 在 walk 模块上的 seam(KeyReader/TailController/
    #    bark_with_mouth/time)在此按当前值读入, 假件仍生效。
    run_walk_session(_WalkSessionContext(
        startup=startup, stack=stack, hw=hw, svc=svc,
        runtime_state=runtime_state, args=args, runtime_config=runtime_config,
        cur_pos=cur_pos,
        log_file=log_file, log_writer=log_writer, log_path=log_path,
        scope_proc=scope_proc,
        key_reader_cls=KeyReader, gamepad_cls=Gamepad,
        input_state_cls=_InputState, tail_cls=TailController,
        bark_with_mouth=bark_with_mouth, clock=time,
        gamepad_device=GAMEPAD_DEVICE,
        input_hal_cls=_WalkInputHAL,
        build_lie_down_target=build_lie_down_target,
        build_sit_target=build_sit_target,
        stop_scope=_stop_scope,
        load_trim_cal=_load_trim_cal, save_trim_cal=_save_trim_cal,
        trim_cal_path=_TRIM_CAL_PATH,
        control_hz=CONTROL_HZ,
    ))


if __name__ == "__main__":
    main()
