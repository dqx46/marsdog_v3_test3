#!/usr/bin/env python3
"""通讯基准测试：测量各 USB-CAN 总线的发送吞吐量、往返时延和接收帧率。

用法:
  python3 comm_bench.py           # 测试全部三条总线
  python3 comm_bench.py --bus 0   # 仅测试 ttyUSB0
  python3 comm_bench.py --duration 5  # 持续 5 秒
"""
import sys, os, time, argparse, threading, statistics
from marsdog_control.config.bus_config import LZ_CAN1_DEVICE, EVO_CAN0_DEVICE, LZ_SERIAL_DEVICE, BAUD
from marsdog_control.hardware.motors.lingzu import (MotorLz, RS05_CAN_IDS, RS05_SERIAL_IDS,
                          float_to_uint, P_MIN, P_MAX, KP_MIN, KP_MAX, KD_MIN, KD_MAX)
from marsdog_control.hardware.motors.evo import MotorEvo, MEVO_KNOWN_IDS
from marsdog_control.hardware.motors.can_serial import CanSerial


def bench_tx_throughput(cs: CanSerial, n_motors: int, duration: float = 2.0,
                         ext_frame: bool = True):
    """测量发送吞吐量（frames/s）和每帧 TX 耗时。"""
    # 构造 n_motors 个 MIT dummy 帧
    frames = []
    for mid in range(1, n_motors + 1):
        can_id = ((0x01 & 0x1F) << 24) | ((0xFD & 0xFFFF) << 8) | mid
        data = b'\x00' * 8
        frames.append((data, 8, can_id if ext_frame else mid))

    # 热身
    for _ in range(5):
        cs.send_bulk(frames)

    # 计时
    tx_count = 0
    t0 = time.perf_counter()
    deadline = t0 + duration
    while time.perf_counter() < deadline:
        cs.send_bulk(frames)
        tx_count += 1

    elapsed = time.perf_counter() - t0
    tx_hz = tx_count / elapsed
    frame_hz = tx_count * n_motors / elapsed
    per_frame_us = elapsed / (tx_count * n_motors) * 1e6
    return tx_hz, frame_hz, per_frame_us


def bench_rtt(cs: CanSerial, motor_ids: list, ext_frame: bool, duration: float = 2.0):
    """测量往返时延（RTT）：发一帧立刻等回包，记录时延分布。"""
    rtts = []
    missed = 0
    deadline = time.monotonic() + duration

    for mid in motor_ids[:1]:  # 只测第一个电机，避免串扰
        if not (time.monotonic() < deadline):
            break
        if ext_frame:
            # LZ disable（安全，有回包）
            can_id = ((0x04 & 0x1F) << 24) | ((0xFD & 0xFFFF) << 8) | mid
            data = b'\x00' * 8
        else:
            # EVO rest state（有回包）
            can_id = mid
            data = bytes([0xFF] * 7 + [0xFD])

        for _ in range(min(100, int((deadline - time.monotonic()) / 0.01))):
            cs.flush()
            t_send = time.perf_counter()
            cs.send_msg(data, 8, can_id)
            resp = cs.read_msg_blocking(0.020)
            t_recv = time.perf_counter()
            if resp is not None:
                rtts.append((t_recv - t_send) * 1000)
            else:
                missed += 1

    return rtts, missed


def bench_recv_rate(cs: CanSerial, n_motors: int, frames_per_cycle: list,
                    duration: float = 2.0):
    """测量接收帧率（接收线程与发送线程并行）。"""
    recv_count = [0]
    running = [True]

    def _recv_thread():
        while running[0]:
            r = cs.read_msg()
            if r is not None:
                recv_count[0] += 1
            else:
                time.sleep(0.0001)

    t = threading.Thread(target=_recv_thread, daemon=True)
    t.start()

    tx_count = 0
    t0 = time.perf_counter()
    deadline = t0 + duration
    while time.perf_counter() < deadline:
        cs.send_bulk(frames_per_cycle)
        tx_count += 1
        time.sleep(0.005)  # 5ms 发一次，模拟 200Hz

    elapsed = time.perf_counter() - t0
    running[0] = False
    time.sleep(0.05)

    tx_frames = tx_count * n_motors
    rx_frames = recv_count[0]
    return tx_frames, rx_frames, rx_frames / elapsed


def print_section(title):
    print(f"\n{'='*60}")
    print(f"  {title}")
    print(f"{'='*60}")


def bench_bus(dev, n_motors, ext_frame, label, duration):
    print_section(f"{label}  ({dev})")
    cs = CanSerial()
    if not cs.begin(dev, BAUD):
        print(f"  [FAIL] 无法打开 {dev}")
        return

    # ── 1. TX 吞吐量 ───────────────────────────────────────────────
    print(f"\n[TX 吞吐] {n_motors} 个电机，send_bulk 测试 {duration:.0f}s ...")
    tx_hz, frame_hz, per_frame_us = bench_tx_throughput(cs, n_motors, duration, ext_frame)
    print(f"  bulk 循环频率: {tx_hz:.1f} Hz")
    print(f"  帧发送速率:    {frame_hz:.1f} frames/s")
    print(f"  单帧平均耗时:  {per_frame_us:.1f} μs")
    print(f"  → 理论 200Hz × {n_motors} 电机 = {200*n_motors} frames/s  "
          f"{'✓ 足够' if frame_hz > 200*n_motors else '✗ 不足'}")

    # ── 2. RTT ─────────────────────────────────────────────────────
    if ext_frame:
        motor_ids = list(range(1, n_motors + 1))
    else:
        motor_ids = MEVO_KNOWN_IDS[:n_motors]

    print(f"\n[RTT] 单电机往返时延（电机需通电）...")
    rtts, missed = bench_rtt(cs, motor_ids, ext_frame, min(duration, 1.0))
    if rtts:
        print(f"  样本数: {len(rtts)}  丢包: {missed}")
        print(f"  平均:  {statistics.mean(rtts):.2f} ms")
        print(f"  中位数: {statistics.median(rtts):.2f} ms")
        print(f"  最小:  {min(rtts):.2f} ms")
        print(f"  最大:  {max(rtts):.2f} ms")
        if len(rtts) > 1:
            print(f"  标准差: {statistics.stdev(rtts):.2f} ms")
    else:
        print(f"  无回包（电机可能未通电，仅参考 TX 数据）")

    cs.end()


def bench_full_loop(duration: float = 3.0, target_hz_list=None):
    """模拟完整控制循环：三路并行发送，统计实际控制频率。"""
    if target_hz_list is None:
        target_hz_list = [200, 300, 400]
    print_section("完整控制循环仿真 (多频率)")
    lz = MotorLz()
    evo = MotorEvo()

    from marsdog_control.config.bus_config import LZ_CAN1_DEVICE, EVO_CAN0_DEVICE, LZ_SERIAL_DEVICE, BAUD
    print("  初始化电机（约 3s）...")
    lz.init_serial(LZ_SERIAL_DEVICE, BAUD)
    lz.init_can1_serial(LZ_CAN1_DEVICE, BAUD)
    evo.init_serial(EVO_CAN0_DEVICE, BAUD)
    time.sleep(0.2)

    from marsdog_control.config.joints import JOINT_MAP, JOINT_BY_ID
    online_lz   = [j for j in JOINT_MAP if j.mtype == "lz"  and lz.is_connected[j.motor_id-1]]
    online_evo  = [j for j in JOINT_MAP if j.mtype == "evo" and evo.is_connected[j.motor_id-1]]
    can1_joints = [j for j in online_lz if j.motor_id in RS05_CAN_IDS]
    ser_joints  = [j for j in online_lz if j.motor_id in RS05_SERIAL_IDS]
    evo_joints  = online_evo

    print(f"  在线: LZ-CAN1={len(can1_joints)} LZ-Serial={len(ser_joints)} EVO={len(evo_joints)}")

    def _send_can1():
        ids = [j.motor_id for j in can1_joints]
        pos = [lz.get_position(mid) for mid in ids]
        if ids:
            lz.mit_controls_can1(ids, pos)

    def _send_serial():
        ids = [j.motor_id for j in ser_joints]
        pos = [lz.get_position(mid) for mid in ids]
        if ids:
            lz.mit_controls_serial(ids, pos)

    for target_hz in target_hz_list:
        _dt = 1.0 / target_hz
        loop_times = []
        send_times = []
        _timer = [time.perf_counter()]

        def _precise_wait():
            _timer[0] += _dt
            now = time.perf_counter()
            r = _timer[0] - now
            if r > 0.0002:
                time.sleep(r - 0.0001)
            while time.perf_counter() < _timer[0]:
                pass

        print(f"\n  ── 目标 {target_hz}Hz ({_dt*1000:.2f}ms/cycle) 运行 {duration:.0f}s ──")
        _timer[0] = time.perf_counter()
        deadline = _timer[0] + duration
        prev = time.perf_counter()

        while time.perf_counter() < deadline:
            t_loop_start = time.perf_counter()

            t1 = threading.Thread(target=_send_can1, daemon=True)
            t2 = threading.Thread(target=_send_serial, daemon=True)
            t1.start(); t2.start()
            if evo_joints:
                evo_ids = [j.motor_id for j in evo_joints]
                evo_pos = [evo.get_position(mid) for mid in evo_ids]
                evo.ptm_controls(evo_ids, evo_pos)
            t1.join(); t2.join()

            now = time.perf_counter()
            send_times.append((now - t_loop_start) * 1000)

            _precise_wait()

            now2 = time.perf_counter()
            loop_times.append((now2 - prev) * 1000)
            prev = now2

        if loop_times:
            loop_times = loop_times[10:]
            send_times = send_times[10:]
            actual_hz = 1000.0 / statistics.mean(loop_times)
            pct95_send = sorted(send_times)[int(len(send_times)*0.95)]
            overrun = sum(1 for t in loop_times if t > _dt*1000*1.1) / len(loop_times) * 100
            print(f"  实际控制频率: {actual_hz:.1f} Hz")
            print(f"  循环周期: 均值={statistics.mean(loop_times):.2f}ms  "
                  f"中位={statistics.median(loop_times):.2f}ms  "
                  f"最大={max(loop_times):.2f}ms")
            print(f"  发送耗时: 均值={statistics.mean(send_times):.2f}ms  "
                  f"p95={pct95_send:.2f}ms  最大={max(send_times):.2f}ms")
            print(f"  超时率: {overrun:.1f}%")
            ok = actual_hz >= target_hz * 0.95
            print(f"  {'✓' if ok else '✗'} 目标 {target_hz}Hz → 实际 {actual_hz:.1f}Hz")

    lz.end()
    evo.end()


def main():
    ap = argparse.ArgumentParser(description="USB-CAN 通讯基准测试")
    ap.add_argument("--bus",      type=int, default=-1, help="指定总线 0/1/2，-1=全部")
    ap.add_argument("--duration", type=float, default=2.0, help="每项测试时长（秒）")
    ap.add_argument("--full",     action="store_true", help="运行完整控制循环仿真")
    args = ap.parse_args()

    buses = [
        (LZ_CAN1_DEVICE,   7, True,  "LZ CAN1  (ttyUSB0 — 前腿+head_roll, 灵足 MIT 扩展帧)"),
        (EVO_CAN0_DEVICE,  5, False, "EVO CAN0 (ttyUSB1 — 后腿hip+颈腰, 泉智博 PTM 标准帧)"),
        (LZ_SERIAL_DEVICE, 7, True,  "LZ Serial(ttyUSB2 — 后腿thigh/calf+头+腰, 灵足 MIT 扩展帧)"),
    ]

    if args.bus >= 0:
        buses = [buses[args.bus]]

    for dev, n_motors, ext_frame, label in buses:
        bench_bus(dev, n_motors, ext_frame, label, args.duration)

    if args.full:
        bench_full_loop(args.duration * 1.5)

    print("\n测试完成。")


if __name__ == "__main__":
    main()
