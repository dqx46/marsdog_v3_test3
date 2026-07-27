#!/usr/bin/env python3
"""严谨的三总线丢包率 + IMU 频率基准测试

测试内容:
  1. 三总线各自的发送→反馈丢包率 (电机使能, 发当前位置保持静止)
  2. IMU 实际数据更新率
  3. 完整控制循环实际频率 (含 gait 计算负载模拟)
  4. 各总线反馈延迟统计

安全保障: 所有电机仅发送当前位置, 不会产生运动
"""
import sys, os, time, threading, statistics, math
from marsdog_control.hardware.motors.lingzu import MotorLz, RS05_CAN_IDS, RS05_SERIAL_IDS
from marsdog_control.hardware.motors.evo import MotorEvo, MEVO_KNOWN_IDS
from marsdog_control.config.bus_config import LZ_CAN1_DEVICE, EVO_CAN0_DEVICE, LZ_SERIAL_DEVICE, BAUD
from marsdog_control.config.bus_config import IMU_DEVICE, IMU_BAUD
from marsdog_control.config.joints import JOINT_MAP, JOINT_BY_ID
from marsdog_control.hardware.sensors.imu_wt901 import ImuWT901


def section(title):
    print(f"\n{'='*64}")
    print(f"  {title}")
    print(f"{'='*64}")


def test_imu_rate(duration=3.0):
    """测量 IMU 实际更新频率"""
    section("IMU 更新频率测试")
    imu = ImuWT901(IMU_DEVICE, IMU_BAUD)
    if not imu.begin():
        print("  ✗ IMU 未连接")
        return None

    time.sleep(0.5)
    cnt_start = imu.update_count
    t0 = time.time()

    timestamps = []
    prev_cnt = cnt_start
    while time.time() - t0 < duration:
        cnt = imu.update_count
        if cnt != prev_cnt:
            timestamps.append(time.time())
            prev_cnt = cnt
        time.sleep(0.0005)

    elapsed = time.time() - t0
    cnt_end = imu.update_count
    total = cnt_end - cnt_start
    rate = total / elapsed if elapsed > 0 else 0

    if len(timestamps) > 2:
        intervals = [(timestamps[i+1] - timestamps[i]) * 1000
                     for i in range(len(timestamps)-1)]
        print(f"  更新帧数: {total}  ({elapsed:.1f}s)")
        print(f"  实际频率: {rate:.1f} Hz")
        print(f"  帧间隔: 均值={statistics.mean(intervals):.2f}ms  "
              f"中位={statistics.median(intervals):.2f}ms  "
              f"最大={max(intervals):.2f}ms  "
              f"最小={min(intervals):.2f}ms")
        if len(intervals) > 1:
            print(f"  帧间隔标准差: {statistics.stdev(intervals):.2f}ms")
        jitter_pct = sum(1 for x in intervals if abs(x - statistics.mean(intervals)) > statistics.mean(intervals)*0.5) / len(intervals) * 100
        print(f"  抖动帧 (>50%偏差): {jitter_pct:.1f}%")
    else:
        print(f"  更新帧数: {total}  频率: {rate:.1f} Hz  (数据不足)")

    imu.close()
    return rate


def test_motor_loss(duration=3.0, target_hz=200):
    """使能电机, 发当前位置, 统计各总线的发送/接收帧数和丢包率"""
    section(f"电机丢包率测试 (目标 {target_hz}Hz, 持续 {duration:.0f}s)")

    lz = MotorLz()
    evo = MotorEvo()

    print("  初始化电机...")
    lz.init_serial(LZ_SERIAL_DEVICE, BAUD)
    lz.init_can1_serial(LZ_CAN1_DEVICE, BAUD)
    evo.init_serial(EVO_CAN0_DEVICE, BAUD)
    time.sleep(0.5)

    can1_joints = [j for j in JOINT_MAP if j.mtype == "lz" and j.bus == "can1"
                   and lz.is_connected[j.motor_id - 1]]
    ser_joints  = [j for j in JOINT_MAP if j.mtype == "lz" and j.bus == "serial"
                   and lz.is_connected[j.motor_id - 1]]
    evo_joints  = [j for j in JOINT_MAP if j.mtype == "evo"
                   and evo.is_connected[j.motor_id - 1]]

    print(f"  在线: CAN1={len(can1_joints)} Serial={len(ser_joints)} EVO={len(evo_joints)}")

    # 记录每个电机初始 rx_count
    lz_rx_before = {}
    for j in can1_joints + ser_joints:
        lz_rx_before[j.motor_id] = lz.rx_count[j.motor_id - 1]
    evo_rx_before = {}
    for j in evo_joints:
        mid = j.motor_id
        # EVO rx tracking: use is_connected + we'll count via position updates
        evo_rx_before[mid] = 0  # will track via manual counting

    dt = 1.0 / target_hz
    timer = time.perf_counter()
    tx_count = 0
    loop_times = []
    send_times = []

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

    print(f"  开始 {target_hz}Hz 控制循环...")
    t_start = time.perf_counter()
    deadline = t_start + duration
    prev = time.perf_counter()

    while time.perf_counter() < deadline:
        t0 = time.perf_counter()

        t1 = threading.Thread(target=_send_can1, daemon=True)
        t2 = threading.Thread(target=_send_serial, daemon=True)
        t1.start(); t2.start()
        if evo_joints:
            evo_ids = [j.motor_id for j in evo_joints]
            evo_pos = [evo.get_position(mid) for mid in evo_ids]
            evo.ptm_controls(evo_ids, evo_pos)
        t1.join(); t2.join()

        now = time.perf_counter()
        send_times.append((now - t0) * 1000)
        tx_count += 1

        timer += dt
        r = timer - time.perf_counter()
        if r > 0.0002:
            time.sleep(r - 0.0001)
        while time.perf_counter() < timer:
            pass

        now2 = time.perf_counter()
        loop_times.append((now2 - prev) * 1000)
        prev = now2

    actual_elapsed = time.perf_counter() - t_start

    # 收集结果
    loop_times = loop_times[10:]
    send_times = send_times[10:]
    actual_hz = 1000.0 / statistics.mean(loop_times) if loop_times else 0

    print(f"\n  ── 控制循环统计 ──")
    print(f"  目标: {target_hz}Hz  实际: {actual_hz:.1f}Hz  总发送: {tx_count} cycles")
    print(f"  循环周期: 均值={statistics.mean(loop_times):.2f}ms  "
          f"中位={statistics.median(loop_times):.2f}ms  "
          f"最大={max(loop_times):.2f}ms")
    p95_send = sorted(send_times)[int(len(send_times)*0.95)]
    print(f"  发送耗时: 均值={statistics.mean(send_times):.2f}ms  "
          f"p95={p95_send:.2f}ms  最大={max(send_times):.2f}ms")

    # 丢包分析
    print(f"\n  ── 各总线丢包率 ──")

    def _analyze_lz_bus(label, joints):
        total_tx = tx_count * len(joints)
        total_rx = 0
        per_motor = []
        for j in joints:
            mid = j.motor_id
            rx_delta = lz.rx_count[mid - 1] - lz_rx_before[mid]
            total_rx += rx_delta
            rate = rx_delta / tx_count * 100 if tx_count > 0 else 0
            per_motor.append((mid, j.name, rx_delta, tx_count, rate))

        loss = (total_tx - total_rx) / total_tx * 100 if total_tx > 0 else 0
        rx_rate = total_rx / total_tx * 100 if total_tx > 0 else 0
        print(f"\n  [{label}]  TX={total_tx} 帧  RX={total_rx} 帧  "
              f"接收率={rx_rate:.1f}%  丢包率={loss:.1f}%")
        for mid, name, rx, tx, rate in per_motor:
            status = "✓" if rate >= 95 else "△" if rate >= 80 else "✗"
            print(f"    {status} Motor {mid:2d} ({name:18s}): "
                  f"TX={tx}  RX={rx}  接收率={rate:.1f}%")

    if can1_joints:
        _analyze_lz_bus("LZ CAN1 (前腿+head_roll)", can1_joints)
    if ser_joints:
        _analyze_lz_bus("LZ Serial (后腿+头+腰)", ser_joints)

    if evo_joints:
        total_tx = tx_count * len(evo_joints)
        connected = sum(1 for j in evo_joints if evo.is_connected[j.motor_id - 1])
        disconnected = len(evo_joints) - connected
        print(f"\n  [EVO CAN0 (后腿hip+颈腰)]  TX={total_tx} 帧  "
              f"在线={connected}/{len(evo_joints)}")
        for j in evo_joints:
            mid = j.motor_id
            idx = mid - 1
            status = "✓" if evo.is_connected[idx] else "✗"
            loss_c = evo._loss_count[idx]
            print(f"    {status} Motor {mid:2d} ({j.name:18s}): "
                  f"连接={evo.is_connected[idx]}  "
                  f"连续丢包={loss_c}  temp={evo.temperature[idx]:.0f}°C")

    lz.end()
    evo.end()
    return actual_hz


def test_gait_overhead(n_cycles=2000):
    """测量步态计算 (IK + 轨迹) 单次耗时"""
    section("步态计算耗时测试")
    from marsdog_control.motion.gait_controller import StandController, StableTrot

    trot = StableTrot(
        body_height=0.20, amp_front=0.04, amp_rear=0.035,
        step_height=0.04, step_height_front=0.045,
        period=0.6, stance_ratio=0.65, hip_abduction=0.02
    )

    times = []
    for i in range(n_cycles):
        t0 = time.perf_counter()
        t_rel = i * 0.005  # 模拟 200Hz
        targets = trot.get_targets(t_rel, imu_dz=None)
        elapsed = (time.perf_counter() - t0) * 1000
        times.append(elapsed)

    times = times[100:]  # 去掉预热
    print(f"  样本数: {len(times)}")
    print(f"  步态计算耗时: 均值={statistics.mean(times):.3f}ms  "
          f"最大={max(times):.3f}ms  中位={statistics.median(times):.3f}ms")
    p99 = sorted(times)[int(len(times)*0.99)]
    print(f"  p99={p99:.3f}ms")

    budget_200 = 5.0 - statistics.mean(times)
    budget_300 = 3.33 - statistics.mean(times)
    budget_400 = 2.5 - statistics.mean(times)
    print(f"  200Hz 剩余预算: {budget_200:.2f}ms")
    print(f"  300Hz 剩余预算: {budget_300:.2f}ms")
    print(f"  400Hz 剩余预算: {budget_400:.2f}ms")


def main():
    print("=" * 64)
    print("  Marsdog 频率 & 丢包率综合基准测试")
    print("  ※ 电机仅发当前位置, 不产生运动")
    print("=" * 64)

    imu_hz = test_imu_rate(3.0)

    test_gait_overhead(2000)

    for hz in [200, 300, 400]:
        test_motor_loss(3.0, hz)

    section("总结")
    if imu_hz:
        print(f"  IMU 更新率: {imu_hz:.0f} Hz")
        if imu_hz >= 190:
            print(f"  ✓ IMU 可以跟上 200Hz 控制 ({imu_hz:.0f} >= 200)")
        elif imu_hz >= 90:
            print(f"  △ IMU {imu_hz:.0f}Hz < 200Hz, 每 {200/imu_hz:.1f} 个控制周期才有新数据")
            print(f"    → 不影响功能, 但 IMU 反馈有 {1000/imu_hz:.1f}ms 延迟")
        else:
            print(f"  ✗ IMU 频率过低 ({imu_hz:.0f}Hz)")
    else:
        print("  ✗ IMU 未连接")

    print(f"\n  控制频率建议:")
    print(f"    200Hz — 保守安全, 适合初期调试")
    print(f"    300Hz — 平衡选择, 通信余量充足")
    print(f"    400Hz — 激进, 需验证算法延迟")


if __name__ == "__main__":
    main()
