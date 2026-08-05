#!/usr/bin/env python3
"""交互式姿势捕获：拖动关节到位后按键保存。

流程:
  1. 打开全部电机总线并短暂使能，随后全部失能（可自由拖动）
  2. 手动把机器人摆成目标姿势
  3. 按键保存当前电机位置（电机帧 rad）:
       z  — 保存为 sit_pose.json（坐下）
       p  — 保存为 lie_down_pose.json（趴下）
       r  — 只打印当前位置，不保存
       q / ESC — 退出

用法:
  ./capture_pose.sh
  python3 -m marsdog_control.apps.tools.calibration.capture_pose
"""

from __future__ import annotations

import math
import os
import select
import signal
import sys
import termios
import time
import tty

from marsdog_control.compat import legacy_dir
from marsdog_control.config.joints import JOINT_BY_ID
from marsdog_control.hardware.board import RkMotorBoard
from marsdog_control.hardware.mapping import REAL_JOINTS
from marsdog_control.motion.lie_down import (
    NON_BODY_LIE_MOTOR_IDS,
    default_lie_down_pose_path,
    default_sit_pose_path,
    save_lie_down_pose,
)

_RESOURCE_DIR = str(legacy_dir())

_stop = False


def _sig(_signum, _frame):
    global _stop
    _stop = True


signal.signal(signal.SIGINT, _sig)
signal.signal(signal.SIGTERM, _sig)


# ── 键盘 ─────────────────────────────────────────────────────────────
def _kb_init():
    try:
        old = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        return old
    except (termios.error, ValueError):
        return None


def _kb_restore(old):
    if old is not None:
        try:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        except (termios.error, ValueError):
            pass


def _getkey():
    try:
        r, _, _ = select.select([sys.stdin], [], [], 0)
        if not r:
            return None
        ch = sys.stdin.read(1)
        if ch == "\x1b":
            # 吞掉可能的 CSI 序列，当作 ESC
            for _ in range(8):
                r2, _, _ = select.select([sys.stdin], [], [], 0.01)
                if not r2:
                    break
                sys.stdin.read(1)
            return "\x1b"
        return ch
    except (ValueError, OSError):
        return None


# ── 读位置（失能后脉冲回读，保证反馈是新的）──────────────────────────
def _pulse_read_lz(lz, mid: int) -> float:
    lz.enable(mid)
    time.sleep(0.01)
    p = lz.get_position(mid)
    for _ in range(12):
        lz.mit_control(mid, p, 0.0, 0.0, 0.0, 0.0)
        time.sleep(0.003)
        p = lz.get_position(mid)
    lz.disable(mid)
    return float(p)


def _pulse_read_evo(evo, mid: int) -> float:
    evo.enter_motor_state(mid)
    time.sleep(0.02)
    p = evo.get_position(mid)
    for _ in range(8):
        evo.ptm_control(mid, p, 0.0, 0.0, 0.0, 0.0)
        time.sleep(0.005)
        p = evo.get_position(mid)
    evo.enter_rest_state(mid)
    return float(p)


def _pulse_read_incos(incos, mid: int) -> float:
    # kp=0 触发反馈且不产生刚度
    p = incos.get_position(mid)
    for _ in range(8):
        incos.mit_control(mid, p, 0.0, 0.0, 0.0, 0.0)
        time.sleep(0.004)
        p = incos.get_position(mid)
    incos.disable(mid)
    return float(p)


def _pulse_read_dm(dm, mid: int) -> float:
    online, pos, _err, _link = dm.probe(mid)
    if online:
        return float(pos)
    # probe 失败时退回缓存
    return float(dm.get_position(mid))


def snapshot_positions(board: RkMotorBoard,
                       online: list[int] | None = None) -> dict[int, float]:
    """对电机做一次脉冲回读，返回电机帧 rad。

    必须使用启动时冻结的 online 列表：失能后 EVO 后台轮询会把偶发无应答的
    电机标成 is_connected=False（实测漏过 waist_yaw=19），若再调 online_ids()
    会少读关节。
    """
    if online is None:
        online = sorted(board.online_ids())
    else:
        online = sorted(online)
    out: dict[int, float] = {}
    print("[read] 正在读取当前位置...")
    for mid in online:
        joint = JOINT_BY_ID.get(mid)
        if joint is None or joint.bus == "none":
            continue
        try:
            if joint.mtype == "lz" and board.lz is not None:
                out[mid] = _pulse_read_lz(board.lz, mid)
            elif joint.mtype == "evo" and board.evo is not None:
                out[mid] = _pulse_read_evo(board.evo, mid)
            elif joint.mtype == "incos" and board.incos is not None:
                out[mid] = _pulse_read_incos(board.incos, mid)
            elif joint.mtype == "dm" and board.dm is not None:
                out[mid] = _pulse_read_dm(board.dm, mid)
            else:
                cached = board.get_angles([mid], include_dm=True)
                if mid in cached:
                    out[mid] = float(cached[mid])
        except Exception as exc:
            print(f"  [WARN] Motor {mid} 脉冲读取失败: {exc}")
            try:
                cached = board.get_angles([mid], include_dm=True)
                if mid in cached:
                    out[mid] = float(cached[mid])
                    print(f"  [WARN] Motor {mid} 改用缓存角 "
                          f"{math.degrees(out[mid]):+.2f}°")
            except Exception:
                pass
    missing = [mid for mid in online if mid not in out]
    if missing:
        print(f"  [WARN] 以下电机未读到位置: {missing}")
    return out


def print_pose(pose: dict[int, float], label: str = "当前位置") -> None:
    print(f"\n[{label}] {len(pose)} 个电机 (排除头/颈后写入 JSON 的集合标 *)")
    for mid in sorted(pose):
        j = JOINT_BY_ID.get(mid)
        name = j.name if j is not None else "?"
        mark = " " if mid in NON_BODY_LIE_MOTOR_IDS else "*"
        print(f"  {mark} Motor {mid:2d} ({name:18s})  "
              f"{math.degrees(pose[mid]):+8.2f}°  ({pose[mid]:+.6f} rad)")
    print()


def save_pose(path: str, pose: dict[int, float], pose_name: str) -> None:
    save_lie_down_pose(path, pose, pose_name=pose_name)
    kept = sorted(mid for mid in pose if mid not in NON_BODY_LIE_MOTOR_IDS)
    print(f"[save] 已保存 {pose_name} → {path}")
    print(f"[save] 写入电机 ID: {kept}")
    print(f"[save] 已排除头/颈 ID {sorted(NON_BODY_LIE_MOTOR_IDS)}\n")


def main() -> int:
    global _stop

    sit_path = default_sit_pose_path(_RESOURCE_DIR)
    lie_path = default_lie_down_pose_path(_RESOURCE_DIR)

    print("=" * 64)
    print("  Marsdog 姿势捕获 (拖动示教)")
    print("  电机上电后会失能，可自由摆姿势")
    print("-" * 64)
    print("  z      读取并保存为坐下  sit_pose.json")
    print("  p      读取并保存为趴下  lie_down_pose.json")
    print("  r      只读取并打印，不保存")
    print("  q/ESC  退出")
    print("=" * 64)
    print(f"  保存目录: {_RESOURCE_DIR}")
    print(f"  sit → {sit_path}")
    print(f"  lie → {lie_path}\n")

    board = RkMotorBoard()
    kb_old = None
    try:
        print("[init] 打开电机总线...")
        board.start()
        online = sorted(board.online_ids())
        expected = sorted(j.motor_id for j in REAL_JOINTS)
        if not online:
            print("[ERROR] 无在线电机，退出")
            return 1
        missing = [mid for mid in expected if mid not in online]
        names = []
        for mid in online:
            j = JOINT_BY_ID.get(mid)
            names.append(f"{mid}:{j.name if j else '?'}")
        print(f"[online] {len(online)}/{len(expected)}  "
              f"{', '.join(names)}")
        if missing:
            miss_names = []
            for mid in missing:
                j = JOINT_BY_ID.get(mid)
                miss_names.append(f"{mid}:{j.name if j else '?'}")
            print(f"[WARN] 启动时未进入 online 列表: {', '.join(miss_names)}")
            print("[WARN] 仍会尝试读取这些电机；若保存结果缺关节请重跑")
            # 捕获示教：宁可多试，不要因为 EVO 误标离线就永远漏存
            online = expected
        print()

        print("[init] 失能全部电机 — 现在可以拖动摆姿势")
        board.disable()
        print("[ready] 等待按键...\n")

        kb_old = _kb_init()
        while not _stop:
            key = _getkey()
            if key is None:
                time.sleep(0.05)
                continue
            if key in ("q", "Q", "\x1b", "\x03"):
                print("\n[quit] 退出")
                break
            if key in ("z", "Z", "p", "P", "r", "R"):
                # 用启动时的 online，避免失能后 EVO 丢应答导致少关节
                pose = snapshot_positions(board, online=online)
                # 读完再确保失能，方便继续拖
                board.disable()
                if not pose:
                    print("[ERROR] 未读到任何电机位置\n")
                    continue
                if key in ("z", "Z"):
                    print_pose(pose, "坐下 sit")
                    save_pose(sit_path, pose, "sit")
                elif key in ("p", "P"):
                    print_pose(pose, "趴下 lie_down")
                    save_pose(lie_path, pose, "lie_down")
                else:
                    print_pose(pose, "预览")
                print("[ready] 可继续拖动，或再按 z/p 覆盖保存；q 退出\n")
            # 忽略其它键
    except Exception as exc:
        print(f"[ERROR] {exc}")
        return 1
    finally:
        _kb_restore(kb_old)
        print("[cleanup] 关闭总线...")
        try:
            board.disable()
        except Exception:
            pass
        try:
            board.close()
        except Exception:
            pass
        print("[cleanup] 完成。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
