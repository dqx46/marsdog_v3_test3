#!/usr/bin/env python3
"""平滑回零脚本 — 全部电机从当前位置渐变到零位并保持。

流程:
  1. 初始化三条总线 (serial → can1 → can0)
  2. 使能所有在线电机
  3. 读取当前位置
  4. 3 秒内平滑插值到 0 rad
  5. 保持零位，等待按 q/ESC 退出
  6. 退出时 disable 所有电机

用法:
  python go_zero.py              # 默认 3 秒渐变
  python go_zero.py --fade 5    # 5 秒渐变（更保守）
  python go_zero.py --id 10     # 只回零单个电机
  python go_zero.py --ids 7,10  # 只回零指定电机
  python go_zero.py --kp 8      # 更低刚度（更柔顺）
"""

import sys, os, time, math, struct, select, tty, termios, signal, argparse
from marsdog_control.config.joints import JOINT_MAP, JOINT_BY_ID
from marsdog_control.hardware.motors.lingzu import MotorLz
from marsdog_control.hardware.motors.evo import MotorEvo

# 头部/脖子电机默认跳过（机械结构特殊，单独标定）
HEAD_IDS = {13, 14, 15, 16}  # head_pitch, head_yaw, head_roll, neck_pitch

CONTROL_HZ  = 200     # 目标控制频率 200Hz
# SEND_INTV 已删除：三路 USB-CAN 各自独立，无总线冲突，直接 send_bulk

_stop = False
def _sig(s, f): global _stop; _stop = True
signal.signal(signal.SIGINT, _sig)


# ── 键盘 ─────────────────────────────────────────────────────────
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


# ── 发指令（批量并行，200Hz 优化）────────────────────────────────
def _send_all(lz, evo, joints, targets, kp_lz, kd_lz, kp_evo, kd_evo):
    """三路总线并行批量发送：
    - LZ CAN1 (ttyUSB0) 和 LZ Serial (ttyUSB2) 同时开启两个线程
    - EVO CAN0 (ttyUSB1) 在主线程发（已有独立接收线程）
    共用 send_bulk，一次 write() 完成一条总线的所有电机。
    """
    import threading as _th
    from marsdog_control.hardware.motors.lingzu import RS05_CAN_IDS, RS05_SERIAL_IDS

    # 按总线分组
    can1_ids, can1_pos, can1_kps, can1_kds = [], [], [], []
    ser_ids,  ser_pos,  ser_kps,  ser_kds  = [], [], [], []
    evo_ids,  evo_pos,  evo_kps,  evo_kds  = [], [], [], []

    for j in joints:
        mid = j.motor_id
        tgt = targets[mid]
        if j.mtype == "lz" and mid in RS05_CAN_IDS:
            can1_ids.append(mid); can1_pos.append(tgt)
            can1_kps.append(kp_lz); can1_kds.append(kd_lz)
        elif j.mtype == "lz" and mid in RS05_SERIAL_IDS:
            ser_ids.append(mid);  ser_pos.append(tgt)
            ser_kps.append(kp_lz); ser_kds.append(kd_lz)
        else:
            evo_ids.append(mid);  evo_pos.append(tgt)
            evo_kps.append(kp_evo); evo_kds.append(kd_evo)

    def _send_can1():
        if can1_ids:
            lz.mit_controls_can1(can1_ids, can1_pos,
                                  kps=can1_kps, kds=can1_kds)

    def _send_serial():
        if ser_ids:
            lz.mit_controls_serial(ser_ids, ser_pos,
                                   kps=ser_kps, kds=ser_kds)

    # 并行发 CAN1 + Serial
    t1 = _th.Thread(target=_send_can1, daemon=True)
    t2 = _th.Thread(target=_send_serial, daemon=True)
    t1.start(); t2.start()

    # EVO 在主线程发（CAN0 独立，不阻塞 LZ）
    for i, mid in enumerate(evo_ids):
        evo.ptm_control(mid, evo_pos[i], 0.0, evo_kps[i], evo_kds[i], 0.0)

    t1.join(); t2.join()


# ── 主程序 ───────────────────────────────────────────────────────
def main():
    global _stop

    ap = argparse.ArgumentParser(description="全部电机平滑回零（含多圈软件补偿）")
    ap.add_argument("--fade",         type=float, default=3.0,  help="渐变时间 秒 (默认 3)")
    ap.add_argument("--id",           type=int, default=None,  help="只回零单个电机 ID")
    ap.add_argument("--ids",          type=str, default=None, help="只回零多个电机 ID，逗号分隔，如 7,10")
    ap.add_argument("--kp-lz",        type=float, default=10.0, help="灵足 Kp (默认 10)")
    ap.add_argument("--kd-lz",        type=float, default=1.0,  help="灵足 Kd (默认 1)")
    ap.add_argument("--kp-evo",       type=float, default=15.0, help="泉智博 Kp (默认 15)")
    ap.add_argument("--kd-evo",       type=float, default=2.0,  help="泉智博 Kd (默认 2)")
    ap.add_argument("--include-head", action="store_true",      help="同时回零头部/脖子电机 (默认跳过)")
    args = ap.parse_args()

    print("=" * 60)
    print("  Marsdog 全关节回零  [multi-turn 软件补偿已启用]")
    print(f"  渐变时间={args.fade}s  kp_lz={args.kp_lz}  kd_lz={args.kd_lz}")
    print(f"  kp_evo={args.kp_evo}  kd_evo={args.kd_evo}")
    if not args.include_head:
        print(f"  头部/脖子跳过 (ID {sorted(HEAD_IDS)})，加 --include-head 可一起回零")
    print("=" * 60)

    # ── 初始化总线（全部 USB-CAN 串口模式）─────────────────────────
    from marsdog_control.config.bus_config import LZ_CAN1_DEVICE, EVO_CAN0_DEVICE, LZ_SERIAL_DEVICE, BAUD
    lz  = MotorLz()
    evo = MotorEvo()

    print(f"\n[init] 1/3 灵足 Serial ({LZ_SERIAL_DEVICE} — 后腿/头/腰)...")
    lz.init_serial(LZ_SERIAL_DEVICE, BAUD)

    print(f"[init] 2/3 灵足 CAN1   ({LZ_CAN1_DEVICE} — 前腿+head_roll)...")
    lz.init_can1_serial(LZ_CAN1_DEVICE, BAUD)

    print(f"[init] 3/3 泉智博 CAN0 ({EVO_CAN0_DEVICE} — 后腿hip+颈腰)...")
    evo.init_serial(EVO_CAN0_DEVICE, BAUD)

    # ── 筛选目标关节 ─────────────────────────────────────────────
    if args.ids is not None:
        target_ids = [int(x.strip()) for x in args.ids.split(",")]
        candidate = []
        for mid in target_ids:
            j = JOINT_BY_ID.get(mid)
            if j is None:
                print(f"[ERROR] 未知 motor ID: {mid}")
                lz.end(); evo.end(); return
            candidate.append(j)
    elif args.id is not None:
        j = JOINT_BY_ID.get(args.id)
        if j is None:
            print(f"[ERROR] 未知 motor ID: {args.id}")
            lz.end(); evo.end(); return
        candidate = [j]
    else:
        candidate = [j for j in JOINT_MAP
                     if args.include_head or j.motor_id not in HEAD_IDS]

    joints = []
    for j in candidate:
        mid = j.motor_id
        online = lz.is_connected[mid-1] if j.mtype == "lz" else evo.is_connected[mid-1]
        if online:
            joints.append(j)
        else:
            print(f"[skip] Motor {mid:2d} ({j.name}) 离线")

    if not joints:
        print("[ERROR] 没有在线电机，退出。")
        lz.end(); evo.end(); return

    total_expected = len(candidate)
    print(f"\n[online] {len(joints)}/{total_expected} 电机在线: "
          f"{[j.motor_id for j in joints]}")

    # ── 使能泉智博 ───────────────────────────────────────────────
    for j in joints:
        if j.mtype == "evo":
            evo.enter_motor_state(j.motor_id)
            time.sleep(0.002)

    # ── 读当前位置 ───────────────────────────────────────────────
    print("\n[pos] 等待反馈稳定...")
    time.sleep(0.4)
    cur = {}
    for j in joints:
        mid = j.motor_id
        cur[mid] = lz.get_position(mid) if j.mtype == "lz" else evo.get_position(mid)

    print("[pos] 当前位置（已含多圈补偿）:")
    for j in joints:
        mid = j.motor_id
        compensated = cur[mid]
        if j.mtype == "lz":
            raw = lz.position[mid - 1]           # 原始多圈值
            offset = lz._pos_offset[mid - 1]     # 补偿偏移
            offset_str = (f"  raw={math.degrees(raw):+.1f}°"
                          f"  offset={math.degrees(offset):+.0f}°" if offset != 0.0 else "")
        else:
            offset_str = ""
        print(f"      Motor {mid:2d} ({j.name:18s}) [{j.model}]  "
              f"{math.degrees(compensated):8.2f}°{offset_str}")

    # ── 平滑渐变到零位 ───────────────────────────────────────────
    print(f"\n[fade] 开始 {args.fade:.1f}s 平滑回零 — 按 q/ESC 急停\n")
    kb_old = _kb_init()
    steps  = int(args.fade * CONTROL_HZ)
    t0     = time.monotonic()

    _dt = 1.0 / CONTROL_HZ   # 5ms @ 200Hz
    _timer = [time.perf_counter()]  # 用 list 让嵌套函数可以修改

    def _precise_wait():
        """精准等到下一个控制周期（sleep + busy-spin 混合，<100μs 抖动）。"""
        _timer[0] += _dt
        now = time.perf_counter()
        remain = _timer[0] - now
        if remain > 0.0002:
            time.sleep(remain - 0.0001)
        while time.perf_counter() < _timer[0]:
            pass
        if _timer[0] < time.perf_counter() - _dt:   # 严重过期，重置时基
            _timer[0] = time.perf_counter()

    # 统计实际频率
    _hz_t0 = time.perf_counter()
    _hz_cnt = 0

    try:
        for step in range(steps + 1):
            if _stop:
                break
            key = _getkey()
            if key and key in ('q', 'Q', '\x1b'):
                print("\n[ESTOP] 用户急停！")
                _stop = True
                break

            alpha = step / steps
            alpha = 3*alpha*alpha - 2*alpha*alpha*alpha   # cubic smoothstep

            targets = {j.motor_id: cur[j.motor_id] * (1 - alpha) for j in joints}
            _send_all(lz, evo, joints, targets,
                      args.kp_lz, args.kd_lz, args.kp_evo, args.kd_evo)

            _hz_cnt += 1
            if _hz_cnt % 200 == 0:
                elapsed = time.perf_counter() - _hz_t0
                actual_hz = _hz_cnt / elapsed
                pct = alpha * 100
                print(f"\r  进度: {pct:5.1f}%  实际频率: {actual_hz:.1f}Hz   ",
                      end="", flush=True)

            _precise_wait()

        print()

        # ── 到位后保持 ───────────────────────────────────────────
        if not _stop:
            print("[hold] 已到零位，保持中 — 按 q/ESC 退出并 disable\n")
            zero = {j.motor_id: 0.0 for j in joints}
            _timer[0] = time.perf_counter()
            while not _stop:
                key = _getkey()
                if key and key in ('q', 'Q', '\x1b'):
                    break
                _send_all(lz, evo, joints, zero,
                          args.kp_lz, args.kd_lz, args.kp_evo, args.kd_evo)
                _precise_wait()

    finally:
        _kb_restore(kb_old)
        print("\n[cleanup] 停止所有电机...")
        lz.end()
        evo.end()
        print("[cleanup] 完成。")


if __name__ == "__main__":
    main()
