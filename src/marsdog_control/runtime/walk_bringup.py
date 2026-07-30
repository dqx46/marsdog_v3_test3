"""Hardware bring-up for the live walk application.

Keeps motor/IMU/board open-and-enable sequencing out of ``apps/walk.py`` so the
app shell can stay CLI + assembly.
"""

from __future__ import annotations

import os
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Optional, Sequence

from marsdog_control.hardware.board import MotorBoard
from marsdog_control.safety.fault_policy import classify_motor_fault


@dataclass
class HardwareSession:
    # lz/evo/dm/incos/imu stay `Any`: real hardware HAL boundary, deliberately
    # structural (see `core/protocols.py` module docstring for the rationale).
    lz: Any
    evo: Any
    dm: Any
    incos: Any
    imu: Any
    imu_ok: bool
    board: Optional[MotorBoard]
    online: list
    dm_fixed_targets: dict = field(default_factory=dict)


def bringup_imu(
    *,
    imu_cls,
    imu_device: str,
    imu_baud: int,
    angle_tau_s: float,
    gyro_tau_s: float,
    require_imu: bool = False,
    ensure_hz: int = 200,
) -> tuple[Any, bool]:
    """Open the first available IMU port. Returns ``(imu_or_None, imu_ok)``.

    Before opening the reader thread, persist ``ensure_hz`` (default 200) to
    WT901 flash if RRATE is wrong — sample firmware defaults to 10 Hz.
    """
    for imu_port in [imu_device]:
        if not os.path.exists(imu_port):
            continue
        if ensure_hz:
            try:
                from marsdog_control.apps.tools.calibration.imu_set_rate import (
                    ensure_rate,
                )
                ok_rate, msg, _ = ensure_rate(imu_port, imu_baud, ensure_hz)
                print(f"[IMU] 回传速率: {msg}")
                if not ok_rate:
                    print(f"[IMU] WARNING: 未能固定到 {ensure_hz}Hz")
            except Exception as e:
                print(f"[IMU] WARNING: 速率自检失败: {e}")
        candidate = imu_cls(
            imu_port, imu_baud,
            angle_tau_s=max(0.0, angle_tau_s),
            gyro_tau_s=max(0.0, gyro_tau_s),
        )
        if candidate.begin():
            print(f"[IMU] 已连接 {imu_port}, 帧数={candidate.update_count}")
            return candidate, True
    print("[IMU] 未连接, 日志中 IMU 列将为 NaN")
    if require_imu:
        print("[FATAL] 地面 tarsus 小扰动必须有 IMU 倾角守卫；未启动任何电机")
    return None, False


def bringup_motors_and_board(
    *,
    motor_lz_cls,
    motor_evo_cls,
    motor_damiao_cls,
    motor_incos_cls,
    board_cls,
    lz_serial_device: str,
    lz_can1_device: str,
    evo_can0_device: str,
    dm_can_device: str,
    incos_can_device: str,
    baud: int,
    joint_map: Sequence,
    dm_joints: Sequence,
    dm_master_id_by_slave: dict,
    incos_can_ids: Sequence[int],
    joint_by_id: dict,
    all_ids: Sequence[int],
    shutdown_motors: Callable,
    clock=time,
) -> Optional[HardwareSession]:
    """Open buses, enable motors, wrap them in ``RkMotorBoard``.

    Returns ``None`` when no motors are online (after attempting shutdown).
    """
    lz = motor_lz_cls()
    evo = motor_evo_cls()
    incos = None
    dm_fixed_targets: dict = {}

    print(f"[init] 1/5 灵足 Serial ({lz_serial_device})...")
    lz.init_serial(lz_serial_device, baud)
    print(f"[init] 2/5 灵足 CAN1   ({lz_can1_device})...")
    lz.init_can1_serial(lz_can1_device, baud)
    print(f"[init] 3/5 泉智博 CAN0 ({evo_can0_device})...")
    evo.init_serial(evo_can0_device, baud)

    print(f"[init] 4/5 因克斯 CAN   ({incos_can_device})...")
    incos = motor_incos_cls()
    if not incos.begin(incos_can_device, incos_can_ids, baud):
        if getattr(incos, "_running", False):
            print(f"  [WARNING] {incos_can_device} 已打开, "
                  f"但因克斯 ID{list(incos_can_ids)} 无应答(未上电?)")
        else:
            print(f"  [WARNING] {incos_can_device} 打开失败, "
                  f"因克斯小腿本次不可用")
            incos = None

    print(f"[init] 5/5 达妙 u2can  ({dm_can_device})...")
    dm = motor_damiao_cls()
    if dm.begin(dm_can_device, baud):
        clock.sleep(1.5)
        for j in dm_joints:
            dm.add_motor(j.motor_id, master_id=dm_master_id_by_slave.get(j.motor_id))
        for j in dm_joints:
            mid = j.motor_id
            dm_online, dm_pos, _dm_err, _dm_link_ok = dm.probe(mid)
            if dm_online:
                dm_fixed_targets[mid] = dm_pos
                dm.enable(mid)
                clock.sleep(0.02)
        dm.start_worker()
        print("  [DM] 持久化严格串行收发 worker 已启动（主循环不等待 ACK）")
    else:
        print(f"  [WARNING] {dm_can_device} 打开失败, 达妙 tarsus 本次不可用")
        dm = None

    for j in joint_map:
        if j.mtype == "lz" and lz.is_connected[j.motor_id - 1]:
            lz.enable(j.motor_id)
            clock.sleep(0.002)
    clock.sleep(0.05)

    for _attempt in range(5):
        not_enabled = []
        for j in joint_map:
            if j.mtype == "evo" and evo.is_connected[j.motor_id - 1]:
                idx = j.motor_id - 1
                if evo.status[idx] != 0x02:
                    not_enabled.append(j)
                    evo.enter_motor_state(j.motor_id)
                    clock.sleep(0.005)
        if not not_enabled:
            break
        clock.sleep(0.05)

    clock.sleep(0.4)
    board = board_cls.from_existing(
        lz, evo, dm, incos, dm_fixed_targets=dm_fixed_targets)
    online = sorted(board.online_ids())
    missing = board.missing_ids()
    for mid in missing:
        print(f"  [WARNING] Motor {mid} ({joint_by_id[mid].name}) 离线")

    if not online:
        print("[ERROR] 无在线电机")
        shutdown_motors(lz, evo, dm, incos)
        return None

    # [故障分级] 缺腿部承重电机(hip/thigh/calf/前腿tarsus)时拒绝进站立——那条
    # 腿顶不住体重, 站立会摔而不是"跛着走"; 缺头/颈/腰(非承重)只降级不拒绝。
    fault = classify_motor_fault(missing, joint_by_id)
    print(fault.describe(joint_by_id))
    if not fault.ok_to_stand:
        print("[ABORT] 拒绝进入站立/步态 —— 承重电机缺失, 站立会直接摔倒")
        shutdown_motors(lz, evo, dm, incos)
        return None

    print(f"\n[online] {len(online)}/{len(all_ids)} 电机在线\n")
    return HardwareSession(
        lz=lz, evo=evo, dm=dm, incos=incos,
        imu=None, imu_ok=False,
        board=board, online=online,
        dm_fixed_targets=dm_fixed_targets,
    )


@dataclass
class StandReadyResult:
    ok: bool
    stand_pos: dict = field(default_factory=dict)
    direction_test_base: Optional[dict] = None


def fade_to_stand(
    *,
    stand,
    cur_pos: dict,
    online,
    lz, evo, dm, incos,
    fade_s: float,
    smooth_transition: Callable,
    recover_lz_stand_faults: Callable,
    shutdown_motors: Callable,
    joint_direction_test: bool = False,
    hip_abd_test: bool = False,
    leg_pitch_test: bool = False,
) -> StandReadyResult:
    """Fade into the stand pose and optionally build a direction-test base."""
    # stand.get_targets() 是纯 URDF 空间; cur_pos 来自 board.get_angles() 是电机空间。
    # smooth_transition / recover 直连 send_all(电机空间), 因此这里必须先把站姿目标
    # 经唯一真源映射 urdf_pose_to_motor 转到电机空间(等价于主控 backend.send 的 j.sign),
    # 否则 sign=-1 的左侧关节会被当电机值直发而反向(淡入错→主循环snap正→Ctrl+C又错)。
    from marsdog_control.backends.real import urdf_pose_to_motor
    stand_pos = stand.get_targets(0)                # URDF 空间(供 direction_test_base / 返回给主循环)
    stand_motor = urdf_pose_to_motor(stand_pos)     # 电机空间(供 fade/recover 实际下发)
    print(f"[fade] 过渡到正常站姿 ({fade_s:.1f}s)...")
    ok = smooth_transition(
        lz, evo, dm, incos, cur_pos, stand_motor, fade_s, label="stand")
    if not ok:
        shutdown_motors(lz, evo, dm, incos)
        return StandReadyResult(ok=False)
    if not recover_lz_stand_faults(lz, evo, dm, incos, online, stand_motor):
        shutdown_motors(lz, evo, dm, incos)
        return StandReadyResult(ok=False)
    print("[ok] 已站立\n")

    direction_test_base = None
    if joint_direction_test:
        direction_test_base = {
            mid: stand_pos.get(mid, cur_pos.get(mid, 0.0)) for mid in online
        }
        if hip_abd_test:
            test_desc = "四髋外展"
        elif leg_pitch_test:
            test_desc = "前腿大腿向前 / 后腿大腿向后"
        else:
            test_desc = "四个小腿都向前"
        print(f"[direction-test] 已在正常站姿；{test_desc}将在 {fade_s:.1f}s 内由主控制"
              "管线缓慢运动。观察确认后按 q 回正常站姿并失能。\n")
    return StandReadyResult(
        ok=True, stand_pos=stand_pos, direction_test_base=direction_test_base)


def calibrate_imu_after_stand(
    *,
    imu,
    imu_ok: bool,
    imu_ctrl,
    joint_direction_test: bool,
    no_imu: bool,
    calibrate_s: float = 1.5,
) -> None:
    """Post-stand IMU zeroing / closed-loop enable (skipped during direction tests)."""
    if joint_direction_test:
        return
    if imu_ok and not no_imu:
        print("[IMU] 站立后校准...")
        imu.calibrate(calibrate_s)
        imu_ctrl.enable()
        print(f"[IMU] 闭环已启用 (Roll+Pitch 速度形): {imu_ctrl.describe()}")
    elif imu_ok and no_imu:
        print("[IMU] 站立后校准(仅记录, 闭环关闭 --no-imu)...")
        imu.calibrate(calibrate_s)
        print("[IMU] 闭环未启用 (--no-imu): IMU 仅记录不修正, 用于 ON/OFF 对照")
    else:
        print("[IMU] 闭环未启用 (IMU 未连接)")


@dataclass
class OperatorInputs:
    kb: Any
    gp: Any
    inp: Any
    tail: Any


def open_operator_inputs(
    *,
    key_reader_cls,
    gamepad_cls,
    input_state_cls,
    tail_cls,
    gamepad_device: str,
    clock=time,
    gamepad_enabled: bool = True,
    tail_enabled: bool = True,
    # Deprecated: prefer gamepad_enabled / tail_enabled from WalkStartupContext.
    args=None,
) -> OperatorInputs:
    """Open keyboard / optional gamepad / optional tail for the live loop."""
    if args is not None:
        gamepad_enabled = not bool(getattr(args, "no_gamepad", False))
        tail_enabled = not bool(getattr(args, "no_tail", False))

    tail = None
    if tail_enabled:
        tail = tail_cls()
        if not tail.begin():
            tail = None

    kb = key_reader_cls()
    kb.start()
    kb.flush()

    gp = None
    gp_ly_offset = 0.0
    if gamepad_enabled:
        gp = gamepad_cls(device=gamepad_device)
        if not gp.connected:
            print("[gamepad] 未找到手柄, 仅键盘控制")
            gp = None
        else:
            print("[gamepad] 已连接, 校准摇杆零位(请松开摇杆)...")
            clock.sleep(0.5)
            samples = []
            for _ in range(20):
                samples.append(gp.get_state().ly)
                clock.sleep(0.025)
            gp_ly_offset = sum(samples) / len(samples)
            print(f"[gamepad] 零位偏移: ly={gp_ly_offset:+.3f} (已自动补偿)")

    inp = input_state_cls()
    inp.gp_ly_offset = gp_ly_offset

    print("─" * 52)
    if gp:
        print("  手柄: 左摇杆Y=前进/后退  START=切模式  LB/RB=步频")
        print("        LT=趴下  RT=狗叫/狗头动作")
        print("        SELECT/B=紧急退出")
    print("  键盘: SPACE/s=切换  +/-=步频  u/d=体高  f/v=摆幅")
    print("        [ ]=P增益调度  ; '=kp  , .=触地kp(柔顺A)  n m=重力补偿(柔顺B)  "
          "k l=roll配平  p=状态  q/ESC=退出")
    print("─" * 52 + "\n")
    return OperatorInputs(kb=kb, gp=gp, inp=inp, tail=tail)


__all__ = [
    "HardwareSession",
    "OperatorInputs",
    "StandReadyResult",
    "bringup_imu",
    "bringup_motors_and_board",
    "calibrate_imu_after_stand",
    "fade_to_stand",
    "open_operator_inputs",
]
