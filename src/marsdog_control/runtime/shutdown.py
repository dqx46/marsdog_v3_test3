"""Shutdown helpers for the new runtime architecture."""

from __future__ import annotations

import os
import statistics
import time
from dataclasses import dataclass
from typing import Callable, Optional

from marsdog_control.hardware import RobotHardware


def shutdown_hardware(hardware: RobotHardware, reason: str = "") -> None:
    hardware.shutdown(reason)


@dataclass
class WalkShutdownContext:
    """Resources needed to safely unwind the walk runtime."""

    kb: object
    gp: Optional[object]
    tail: Optional[object]
    board: object
    balance_runtime: object
    args: object
    imu_ctrl: object
    stand: object
    lz: object
    evo: object
    dm: Optional[object]
    incos: Optional[object]
    imu: object
    log_file: Optional[object]
    log_path: Optional[str]
    scope_proc: Optional[object]
    dm_tarsus_active: bool
    joint_direction_test: bool
    lie_down_hold: bool
    estopped_fall: bool
    actuation_runtime: Callable
    smooth_transition: Callable
    shutdown_motors: Callable
    stop_scope: Callable
    load_trim_cal: Callable
    save_trim_cal: Callable
    trim_cal_path: str
    control_hz: float
    clock: object = time


def run_walk_shutdown(ctx: WalkShutdownContext) -> None:
    """Run the real-robot safe shutdown sequence for the walk app."""

    ctx.kb.stop()
    if ctx.gp:
        ctx.gp.close()
    if ctx.tail is not None:
        ctx.tail.close()

    _save_auto_trim_calibration(ctx)

    # cur2 是电机空间(board.get_angles); stand.get_targets() 是纯 URDF 空间。
    # smooth_transition / soft_disable 均直连电机空间, 必须经唯一真源映射把站姿
    # 目标转到电机空间(等价主控 backend.send 的 j.sign), 否则回站立/失能时 sign=-1
    # 的左侧关节会反向(与淡入同一个坐标系错配 bug)。
    from marsdog_control.backends.real import urdf_pose_to_motor
    cur2 = ctx.board.get_angles(include_dm=ctx.dm_tarsus_active)
    stand_final = urdf_pose_to_motor(ctx.stand.get_targets(0))
    if ctx.joint_direction_test:
        print("\n\n[cleanup] 关节方向测试 -> 回正常站姿 (1.5s)...")
        ctx.smooth_transition(
            ctx.lz, ctx.evo, ctx.dm, ctx.incos, cur2, stand_final, 1.5,
            label="direction-return")
    elif ctx.lie_down_hold:
        print("\n\n[cleanup] 已在趴下姿势 -> 不回站立, 直接缓速失能...")
        stand_final = dict(cur2)
    elif ctx.estopped_fall:
        print("\n\n[cleanup] 摔倒急停 -> 跳过回站立, 直接缓速失能...")
        stand_final = dict(cur2)
    else:
        print("\n\n[cleanup] 回站立 (1.5s)...")
        ctx.smooth_transition(
            ctx.lz, ctx.evo, ctx.dm, ctx.incos, cur2, stand_final, 1.5,
            label="return")

    disable_secs = 8.0
    print(f"[cleanup] {disable_secs:.0f}s 缓速失能...")
    ctx.board.soft_disable(
        stand_final, ctx.actuation_runtime(),
        duration_s=disable_secs,
        control_hz=ctx.control_hz,
        stop_check=lambda: False,
        clock=ctx.clock,
    )

    print("[cleanup] 失能电机...")
    ctx.board.disable()

    ctx.shutdown_motors(ctx.lz, ctx.evo, ctx.dm, ctx.incos)
    if ctx.imu is not None and getattr(ctx.imu, "connected", False):
        ctx.imu.close()
    if ctx.log_file:
        ctx.log_file.close()
        if ctx.log_path:
            print(f"[log] 已保存: {os.path.basename(ctx.log_path)}")
            print(f"[log] 路径: {ctx.log_path}")
    ctx.stop_scope(ctx.scope_proc)
    print("[cleanup] 完成。")


def _save_auto_trim_calibration(ctx: WalkShutdownContext) -> None:
    if (ctx.joint_direction_test
            or not getattr(ctx.args, "auto_trim", False)
            or ctx.imu_ctrl is None):
        return

    count = len(ctx.imu_ctrl.get_roll_ff_mm())
    previous = ctx.load_trim_cal()
    values = None
    source = ""
    history = ctx.balance_runtime.ff_hist

    if len(history) >= 20:
        tail = history[int(len(history) * 0.6):]
        candidate = [statistics.median(sample[i] for sample in tail)
                     for i in range(count)]
        source = f"{len(tail)}样本中位数"

        if (isinstance(previous, dict)
                and previous.get("roll_ff_mm")
                and len(previous["roll_ff_mm"]) == count):
            alpha = 0.25
            step_limit = 1.0
            values = []
            for i in range(count):
                old = float(previous["roll_ff_mm"][i])
                ema = alpha * candidate[i] + (1 - alpha) * old
                lo = old - step_limit
                hi = old + step_limit
                values.append(round(max(lo, min(hi, ema)), 3))
            source += "+跨run EMA限幅"
        else:
            values = [round(v, 3) for v in candidate]
    elif (isinstance(previous, dict)
          and previous.get("roll_ff_mm")
          and len(previous["roll_ff_mm"]) == count):
        values = [round(float(v), 3) for v in previous["roll_ff_mm"]]
        source = f"样本不足({len(history)}), 沿用上次"
    else:
        values = ctx.imu_ctrl.get_roll_ff_mm()
        source = f"样本不足({len(history)}), 兜底末值"

    if ctx.save_trim_cal(values, ctx.imu_ctrl.roll_trim * 1000.0):
        print(f"[AT] 本机配平已保存({ctx.imu_ctrl.ff_phases}相位, {source}, "
              f"值{values[0]:+.2f}mm) → {os.path.basename(ctx.trim_cal_path)}")


__all__ = [
    "WalkShutdownContext",
    "run_walk_shutdown",
    "shutdown_hardware",
]
