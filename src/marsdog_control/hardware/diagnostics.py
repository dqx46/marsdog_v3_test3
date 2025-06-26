"""Motor diagnostics and LZ stand-fault recovery helpers."""

from __future__ import annotations

import math
import sys
import time
from typing import Callable, Optional


def check_motors_board(board, real_joints, label=""):
    """Board-based motor status printer."""
    feedback = board.get_feedback(j.motor_id for j in real_joints)
    disabled = []
    lines = []
    for j in real_joints:
        sample = feedback.samples.get(j.motor_id)
        enabled = bool(sample.enabled) if sample is not None else False
        pos = math.degrees(sample.position) if sample is not None else 0.0
        fault = sample.fault if sample is not None else -1
        flag = "OK" if enabled else "!!"
        lines.append(f"  [{flag}] Motor {j.motor_id:2d} ({j.name:18s}) {pos:+8.2f}°"
                     + ("" if enabled else f"  fault={fault}"))
        if not enabled:
            disabled.append(j)
    print(f"\n── 电机状态 {label} ──")
    print("\n".join(lines))
    if disabled:
        print(f"  *** {len(disabled)} 个电机失能: "
              + ", ".join(f"Motor {j.motor_id}({j.name})" for j in disabled))
    return disabled


def check_motors(lz, evo, dm, incos, real_joints, dm_fixed_targets, label=""):
    """打印电机使能/位置/fault 快照，返回失能关节列表。"""
    disabled = []
    lines = []
    for j in real_joints:
        mid = j.motor_id
        idx = mid - 1
        if j.mtype == "lz":
            en = lz.is_enabled[idx]
            pos = math.degrees(lz.get_position(mid))
            fault = lz.fault[idx]
        elif j.mtype == "incos":
            en = incos.is_enabled[idx] if incos is not None else False
            pos = math.degrees(incos.get_position(mid)) if incos is not None else 0.0
            fault = incos.fault[idx] if incos is not None else -1
        elif j.mtype == "dm":
            en = mid in dm_fixed_targets
            pos = math.degrees(dm.get_position(mid)) if dm is not None else 0.0
            fault = dm.get_error(mid) if dm is not None else -1
        else:
            en = (evo.status[idx] == 0x02)
            pos = math.degrees(evo.get_position(mid))
            fault = evo.fault[idx]
        flag = "OK" if en else "!!"
        lines.append(f"  [{flag}] Motor {mid:2d} ({j.name:18s}) {pos:+8.2f}°"
                     + ("" if en else f"  fault={fault}"))
        if not en:
            disabled.append(j)
    print(f"\n── 电机状态 {label} ──")
    print("\n".join(lines))
    if disabled:
        print(f"  *** {len(disabled)} 个电机失能: "
              + ", ".join(f"Motor {j.motor_id}({j.name})" for j in disabled))
    return disabled


def find_lz_recoverable_faults(lz, joints, targets, *,
                               max_error_rad=math.radians(15.0),
                               low_torque_nm=0.10):
    """找出在线但需要清错/重使能的 LZ 电机。

    LZ 偶发会出现"有反馈、mode 看似正常, 但几乎不出力且目标误差很大"的状态。
    这和通信离线不同；最新日志里的 rl_calf(ID11) 就是这种形态。判据保守地要求:
      - mode != 2（未进 MIT）; 或
      - 显式 disabled 或 fault != 0; 或
      - 目标误差很大且扭矩反馈接近 0。
    """
    faults = []
    for j in joints:
        if j.mtype != "lz":
            continue
        mid = j.motor_id
        if mid not in targets:
            continue
        idx = mid - 1
        enabled = bool(lz.is_enabled[idx])
        fault = int(lz.fault[idx])
        mode = int(lz.mode[idx])
        actual = lz.get_position(mid)
        err = abs(targets[mid] - actual)
        torque = abs(lz.torque[idx])
        if (
            mode != 2
            or (not enabled)
            or fault != 0
            or (err > max_error_rad and torque < low_torque_nm)
        ):
            faults.append(j)
    return faults


def recover_lz_stand_faults(
    lz, evo, dm, incos, online, stand_pos, *,
    real_joints,
    dm_fixed_targets,
    read_positions_fn: Callable,
    smooth_transition_fn: Callable,
    attempts=2,
    max_error_rad=math.radians(15.0),
    low_torque_nm=0.10,
):
    """起立后恢复 LZ 失能/疑似无出力电机，然后重新平滑拉回站姿。"""
    online_joints = [
        j for j in real_joints
        if j.motor_id in online and j.mtype == "lz"
    ]
    for attempt in range(1, attempts + 1):
        faults = find_lz_recoverable_faults(
            lz, online_joints, stand_pos,
            max_error_rad=max_error_rad,
            low_torque_nm=low_torque_nm,
        )
        if not faults:
            return True

        print("\n[recover] 检测到 LZ 电机失能/疑似无出力: "
              + ", ".join(
                  f"M{j.motor_id}({j.name}) "
                  f"err={math.degrees(stand_pos[j.motor_id] - lz.get_position(j.motor_id)):+.1f}° "
                  f"mode={lz.mode[j.motor_id - 1]} fault={lz.fault[j.motor_id - 1]} "
                  f"tq={lz.torque[j.motor_id - 1]:+.2f}Nm"
                  for j in faults
              ))
        for j in faults:
            print(f"[recover] clear fault + enable M{j.motor_id}({j.name}) "
                  f"attempt {attempt}/{attempts}")
            lz.re_enable(j.motor_id)
            # re_enable 末尾是零增益 MIT；立刻用当前位置锁一帧，避免空窗无力。
            if lz.ensure_mit(j.motor_id, tag="recover"):
                q = float(lz.get_position(j.motor_id))
                lz.mit_control(j.motor_id, q, 0.0, 80.0, 4.0, 0.0)
            time.sleep(0.03)

        cur = read_positions_fn(lz, evo, incos)
        if dm is not None:
            cur.update(dm_fixed_targets)
        smooth_transition_fn(
            lz, evo, dm, incos, cur, stand_pos, 1.0, label="recover-stand")
        time.sleep(0.1)

    remaining = find_lz_recoverable_faults(
        lz, online_joints, stand_pos,
        max_error_rad=max_error_rad,
        low_torque_nm=low_torque_nm,
    )
    if remaining:
        print("\n[FATAL] LZ 电机恢复失败, 为避免瘸腿继续进入步态, 本次退出: "
              + ", ".join(f"M{j.motor_id}({j.name})" for j in remaining))
        return False
    return True


def smooth_transition(
    lz, evo, dm, incos, from_pos, to_pos, duration, label="fade", *,
    send_fn: Callable,
    control_hz: float = 200.0,
    stop_check: Optional[Callable[[], bool]] = None,
    clock=None,
    kp_start: float = 0.3,
    kp_end: float = 1.0,
    on_first_send: Optional[Callable[[], None]] = None,
):
    """按 smoothstep 在 duration 内从 from_pos 插值到 to_pos，每步调用 send_fn。

    ``send_fn(cur, kp_scale)`` 负责实际下发；``stop_check`` 为真时提前中断并返回 False。

    ``kp_start``/``kp_end``: 刚度随进度线性变化。坐下/趴下应让 ``kp_end``
    与 hold 入口软刚度对齐，避免过渡最后一帧全增益猛冲。

    ``on_first_send``: 首帧 send 之后回调一次（用于 pose-hold → fade 无缝交接）。
    """
    clock = clock or time
    steps = max(1, int(duration * control_hz))
    kp0 = float(kp_start)
    kp1 = float(kp_end)
    t0 = clock.monotonic()
    first = True
    for step in range(steps + 1):
        if stop_check is not None and stop_check():
            return False
        alpha = step / steps
        alpha = 3 * alpha * alpha - 2 * alpha * alpha * alpha
        kp_s = kp0 + (kp1 - kp0) * alpha
        cur = {}
        for mid in set(from_pos) | set(to_pos):
            a = from_pos.get(mid, 0.0)
            b = to_pos.get(mid, 0.0)
            cur[mid] = a + (b - a) * alpha
        # 注意: from_pos/to_pos 都不含达妙 tarsus id 时, cur 也不会有,
        # send_all 内部会自动回退用 DM_FIXED_TARGETS 保持固定角度不动。
        send_fn(lz, evo, dm, incos, cur, kp_s)
        if first:
            first = False
            if on_first_send is not None:
                try:
                    on_first_send()
                except Exception:
                    pass
        pct = int(alpha * 100)
        sys.stdout.write(f"\r  [{label}] {pct:3d}%  step {step}/{steps}   ")
        sys.stdout.flush()
        next_t = t0 + (step + 1) / control_hz
        sleep_t = next_t - clock.monotonic()
        if sleep_t > 0:
            clock.sleep(sleep_t)
    sys.stdout.write("\n")
    return True


__all__ = [
    "check_motors_board",
    "check_motors",
    "find_lz_recoverable_faults",
    "recover_lz_stand_faults",
    "smooth_transition",
]
