#!/usr/bin/env python3
"""平滑回零 — 全部电机从当前位置渐变到零位并保持。

与 ``run_walk`` 同一管线:
  ``get_device_config`` → ``open_walk_hardware`` / ``bringup_motors_and_board``
  → ``WalkServices.send_all`` → ``resolve_gains(JOINT_GAINS × leg_kp_scale)``
  + ``static_trq_ff_by_id``

覆盖品牌: LZ (serial+CAN1) / EVO / Incos / DM 前 tarsus。

用法:
  ./run_with_env.sh python -m marsdog_control.apps.tools.calibration.go_zero
  ./run_with_env.sh python -m marsdog_control.apps.tools.calibration.go_zero --fade 5
  ./run_with_env.sh python -m marsdog_control.apps.tools.calibration.go_zero --ids 2,3,6,7
  ./run_with_env.sh python -m marsdog_control.apps.tools.calibration.go_zero --include-head
  ./run_with_env.sh python -m marsdog_control.apps.tools.calibration.go_zero --soft-disable
"""

from __future__ import annotations

import argparse
import math
import select
import signal
import sys
import termios
import time
import tty

from marsdog_control.config.gains import (
    JOINT_GAINS,
    WALK_LEG_KP_SCALE,
    static_trq_ff_by_id,
)
from marsdog_control.config.joints import JOINT_BY_ID, JOINT_MAP
from marsdog_control.runtime.walk_hw import open_walk_hardware
from marsdog_control.runtime.walk_state import WalkRuntimeState

# head_pitch / head_yaw / head_roll / neck_pitch — 与 JOINT_MAP 一致
HEAD_IDS = {15, 16, 17, 18}
DM_IDS = {4, 8}

CONTROL_HZ = 200.0

_stop = False


def _sig(s, f):
    global _stop
    _stop = True


signal.signal(signal.SIGINT, _sig)


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


def _kbhit():
    try:
        r, _, _ = select.select([sys.stdin], [], [], 0)
        return bool(r)
    except (ValueError, OSError):
        return False


def _getkey():
    return sys.stdin.read(1) if _kbhit() else None


def _precise_wait(timer: list, dt: float) -> None:
    timer[0] += dt
    now = time.perf_counter()
    remain = timer[0] - now
    if remain > 0.0002:
        time.sleep(remain - 0.0001)
    while time.perf_counter() < timer[0]:
        pass
    if timer[0] < time.perf_counter() - dt:
        timer[0] = time.perf_counter()


def _ensure_mit_preflight(lz, evo, joints) -> list[int]:
    """Force LZ into MIT (mode=2) and EVO into MOTOR_STATE before fade.

    Hot-start bring-up can leave responders online-but-unlocked; go_zero must
    not stop the hold thread until these are claimed.
    """
    fixed: list[int] = []
    if lz is not None:
        lz_ids = [j.motor_id for j in joints if j.mtype == "lz"]
        claimed, _failed = lz.claim_mit_all(lz_ids, tag="go_zero")
        fixed.extend(claimed)
    for j in joints:
        mid = j.motor_id
        if j.mtype == "evo" and evo is not None:
            idx = mid - 1
            if not evo.is_connected[idx]:
                continue
            if evo.status[idx] != 0x02:
                evo.enter_motor_state(mid)
                time.sleep(0.005)
                fixed.append(mid)
    return fixed


def _print_drive_status(lz, evo, incos, dm, joints) -> None:
    """Diagnose MIT / motor-state before fade so soft axes are visible."""
    if lz is not None:
        lz.print_mit_status(
            [j.motor_id for j in joints if j.mtype == "lz"],
            tag="mit",
        )
    print("[mit] 其它驱动:")
    for j in joints:
        mid = j.motor_id
        if j.mtype == "evo" and evo is not None:
            idx = mid - 1
            st = evo.status[idx] if 0 <= idx < len(evo.status) else -1
            flag = "OK" if st == 0x02 else "!!"
            print(
                f"  [{flag}] EVO ID{mid:2d} ({j.name:18s}) "
                f"status=0x{st:02X} connected={bool(evo.is_connected[idx])}"
            )
        elif j.mtype == "incos" and incos is not None:
            idx = mid - 1
            en = bool(incos.is_enabled[idx]) if 0 <= idx < len(incos.is_enabled) else False
            conn = bool(incos.is_connected[idx]) if 0 <= idx < len(incos.is_connected) else False
            flag = "OK" if conn else "!!"
            print(
                f"  [{flag}] INC ID{mid:2d} ({j.name:18s}) "
                f"connected={conn} enabled={en}"
            )
        elif j.mtype == "dm" and dm is not None:
            print(f"  [OK] DM  ID{mid:2d} ({j.name:18s}) worker={dm.worker_running}")


def main(argv: list[str] | None = None) -> int:
    global _stop
    _stop = False

    ap = argparse.ArgumentParser(
        description="全部电机平滑回零（run_walk SoftTrot gains 管线）",
    )
    ap.add_argument("--fade", type=float, default=3.0, help="渐变时间 秒 (默认 3)")
    ap.add_argument("--id", type=int, default=None, help="只回零单个电机 ID")
    ap.add_argument(
        "--ids", type=str, default=None,
        help="只回零多个电机 ID，逗号分隔，如 2,3,6,7",
    )
    ap.add_argument(
        "--leg-kp-scale", type=float, default=None,
        help=f"腿部 kp 叠加 (默认=run_walk SoftTrot {WALK_LEG_KP_SCALE})",
    )
    ap.add_argument(
        "--include-head", action="store_true",
        help="同时回零头部/脖子电机 (默认跳过 ID 15–18)",
    )
    ap.add_argument(
        "--dm", action=argparse.BooleanOptionalAction, default=True,
        help="回零达妙前 tarsus ID4/8（默认开；--no-dm 跳过）",
    )
    ap.add_argument(
        "--soft-disable", action="store_true",
        help="冷启: bring-up clear_fault 失能再使能进 MIT（菜单默认；"
             "热启保持使能请去掉此开关）",
    )
    args = ap.parse_args(argv)

    leg_scale = (
        float(args.leg_kp_scale)
        if args.leg_kp_scale is not None
        else float(WALK_LEG_KP_SCALE)
    )
    clear_fault = bool(args.soft_disable)

    print("=" * 60)
    print("  Marsdog 全关节回零  [LZ + EVO + Incos"
          + (" + DM" if args.dm else "") + "]")
    print(f"  渐变时间={args.fade}s")
    print(
        f"  gains=run_walk JOINT_GAINS  leg_kp_scale={leg_scale:.2f}  "
        f"(含静态 trq_ff)"
    )
    print(
        f"  bring-up={'clear_fault 冷启' if clear_fault else 'keep-enabled 热启'}"
    )
    if not args.include_head:
        print(f"  头部/脖子跳过 (ID {sorted(HEAD_IDS)})，加 --include-head 可一起回零")
    if not args.dm:
        print("  达妙 tarsus 已跳过 (--no-dm)")
    print("=" * 60)

    runtime_state = WalkRuntimeState(
        joint_gains=JOINT_GAINS,
        leg_kp_scale=leg_scale,
    )
    # SoftTrot go-zero moves DM to 0 via active path (not fixed-only).
    if args.dm:
        runtime_state.dm.active = True

    bundle = open_walk_hardware(
        runtime_state,
        clear_fault=clear_fault,
        control_hz=CONTROL_HZ,
        with_imu=False,
        clock=time,
    )
    if bundle is None:
        print("[ERROR] 电机 bring-up 失败 / 无在线电机")
        return 1

    lz, evo, dm, incos = bundle.lz, bundle.evo, bundle.dm, bundle.incos
    svc = bundle.svc
    online_ids = {int(mid) for mid in (bundle.online or [])}
    hot_hold = getattr(bundle.session, "hot_hold", None)

    # ── 筛选目标关节 ─────────────────────────────────────────────
    if args.ids is not None:
        target_ids = [int(x.strip()) for x in args.ids.split(",") if x.strip()]
        candidate = []
        for mid in target_ids:
            j = JOINT_BY_ID.get(mid)
            if j is None or j.bus == "none":
                print(f"[ERROR] 未知/未接线 motor ID: {mid}")
                if hot_hold is not None:
                    hot_hold.stop()
                svc.shutdown_motors(lz, evo, dm, incos)
                return 2
            candidate.append(j)
    elif args.id is not None:
        j = JOINT_BY_ID.get(args.id)
        if j is None or j.bus == "none":
            print(f"[ERROR] 未知/未接线 motor ID: {args.id}")
            if hot_hold is not None:
                hot_hold.stop()
            svc.shutdown_motors(lz, evo, dm, incos)
            return 2
        candidate = [j]
    else:
        candidate = [
            j for j in JOINT_MAP
            if j.bus != "none"
            and (args.include_head or j.motor_id not in HEAD_IDS)
            and (args.dm or j.motor_id not in DM_IDS)
        ]

    joints = []
    for j in candidate:
        mid = j.motor_id
        if mid in online_ids or mid in runtime_state.dm.fixed_targets:
            joints.append(j)
        else:
            print(f"[skip] Motor {mid:2d} ({j.name}) 离线")

    if not joints:
        print("[ERROR] 没有在线电机，退出。")
        if hot_hold is not None:
            hot_hold.stop()
        svc.shutdown_motors(lz, evo, dm, incos)
        return 1

    print(
        f"\n[target] {len(joints)}/{len(candidate)} 电机: "
        f"{[j.motor_id for j in joints]}"
    )

    trq_ff = static_trq_ff_by_id(JOINT_GAINS)

    # ── MIT 预检（hot_hold 仍在跑，避免空窗掉因克斯超时）────────────
    fixed = _ensure_mit_preflight(lz, evo, joints)
    if fixed:
        print(f"[mit] 已补使能/进 MIT: {fixed}")
    _print_drive_status(lz, evo, incos, dm, joints)

    print("\n[pos] 读取当前位置...")
    cur_pos = svc.read_positions(lz, evo, incos)
    if dm is not None:
        cur_pos.update(runtime_state.dm.fixed_targets)
        for j in joints:
            if j.mtype == "dm" and j.motor_id not in cur_pos:
                try:
                    cur_pos[j.motor_id] = float(dm.get_position(j.motor_id))
                except Exception:
                    cur_pos[j.motor_id] = 0.0
    # Include all target joints even if read_positions omitted a brand.
    for j in joints:
        mid = j.motor_id
        if mid in cur_pos:
            continue
        try:
            if j.mtype == "lz" and lz is not None:
                cur_pos[mid] = float(lz.get_position(mid))
            elif j.mtype == "evo" and evo is not None:
                cur_pos[mid] = float(evo.get_position(mid))
            elif j.mtype == "incos" and incos is not None:
                cur_pos[mid] = float(incos.get_position(mid))
            elif j.mtype == "dm" and dm is not None:
                cur_pos[mid] = float(dm.get_position(mid))
        except Exception:
            cur_pos[mid] = 0.0

    print("[pos] 当前位置:")
    for j in joints:
        mid = j.motor_id
        p = float(cur_pos.get(mid, 0.0))
        print(f"      Motor {mid:2d} ({j.name:18s}) [{j.model}]  {math.degrees(p):8.2f}°")

    # 与 walk.py 同源: 先 pose_hold，再停 hot_hold，fade 无 MIT 空窗。
    hold_targets = dict(cur_pos)
    svc.start_pose_hold(lz, evo, dm, incos, hold_targets)
    if hot_hold is not None:
        hot_hold.stop()
        bundle.session.hot_hold = None
        hot_hold = None

    joint_ids = [j.motor_id for j in joints]
    print(f"\n[fade] 开始 {args.fade:.1f}s 平滑回零 — 按 q/ESC 急停\n")
    kb_old = _kb_init()
    steps = max(1, int(args.fade * CONTROL_HZ))
    dt = 1.0 / CONTROL_HZ
    timer = [time.perf_counter()]
    hz_t0 = time.perf_counter()
    hz_cnt = 0
    pose_hold_stopped = False

    def _send(targets: dict) -> None:
        nonlocal pose_hold_stopped
        # Keep non-target axes at last measured so board path stays consistent
        full = dict(cur_pos)
        full.update(targets)
        if dm is not None:
            for mid, p in runtime_state.dm.fixed_targets.items():
                full.setdefault(mid, p)
        svc.send_all(
            lz, evo, dm, incos, full,
            use_joint_gains=True,
            kp_scale=1.0,
            trq_ff=trq_ff,
        )
        # Overlap: first fade frame sent, then stop pose-hold (no MIT gap).
        if not pose_hold_stopped:
            svc.stop_pose_hold()
            pose_hold_stopped = True

    try:
        for step in range(steps + 1):
            if _stop:
                break
            key = _getkey()
            if key and key in ("q", "Q", "\x1b"):
                print("\n[ESTOP] 用户急停！")
                _stop = True
                break

            alpha = step / steps
            alpha = 3 * alpha * alpha - 2 * alpha * alpha * alpha
            targets = {
                mid: float(cur_pos.get(mid, 0.0)) * (1.0 - alpha)
                for mid in joint_ids
            }
            # Track DM fixed targets toward zero too
            if dm is not None:
                for mid in joint_ids:
                    if mid in DM_IDS:
                        runtime_state.dm.fixed_targets[mid] = targets[mid]
            _send(targets)

            hz_cnt += 1
            if hz_cnt % 200 == 0:
                elapsed = time.perf_counter() - hz_t0
                actual_hz = hz_cnt / max(elapsed, 1e-9)
                print(
                    f"\r  进度: {alpha * 100:5.1f}%  实际频率: {actual_hz:.1f}Hz   ",
                    end="", flush=True,
                )
            _precise_wait(timer, dt)

        print()

        if not _stop:
            print("[hold] 已到零位，保持中 — 按 q/ESC 退出\n")
            zero = {mid: 0.0 for mid in joint_ids}
            if dm is not None:
                for mid in joint_ids:
                    if mid in DM_IDS:
                        runtime_state.dm.fixed_targets[mid] = 0.0
            timer[0] = time.perf_counter()
            while not _stop:
                key = _getkey()
                if key and key in ("q", "Q", "\x1b"):
                    break
                _send(zero)
                _precise_wait(timer, dt)

    finally:
        _kb_restore(kb_old)
        svc.stop_pose_hold()
        if hot_hold is not None:
            hot_hold.stop()
        print("\n[cleanup] 关闭主机侧 IO（run_walk WalkServices.shutdown_motors）...")
        # disable=False: close host IO only; leave last MIT hold (same as walk hot-exit).
        svc.shutdown_motors(lz, evo, dm, incos, disable=False)
        print("[cleanup] 完成。")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
