#!/usr/bin/env python3
"""达妙 u2can 链路延迟基准测试 — 只读位置+原地保持(不主动位移), 用于确认
control_mit() 往返延迟能否跟上步态帧率 (数十 Hz)。

安全性: 先用 probe() 读到电机当前真实位置, 后续 control_mit 的 q 目标始终等于
这个读到的位置 (保持不动), kp 从很低开始逐步验证, 不会让电机产生位移。
"""
import statistics
import sys
import time

from marsdog_control.config.bus_config import DM_CAN_DEVICE
from marsdog_control.config.joints import DM_MASTER_ID_BY_SLAVE
from marsdog_control.hardware.motors.damiao import MotorDamiao, DM_KNOWN_IDS

N_WARMUP = 20
N_SAMPLES = 300
TEST_KP = 5.0   # 很低的 kp, 只是为了测链路延迟, 不做实际保持力
TEST_KD = 0.5


def main():
    dev = DM_CAN_DEVICE
    print(f"[bench] dm_can 设备: {dev}")
    dm = MotorDamiao()
    if not dm.begin(dev):
        print("[FATAL] 打开 dm_can 失败"); sys.exit(1)

    for sid in DM_KNOWN_IDS:
        dm.add_motor(sid, master_id=DM_MASTER_ID_BY_SLAVE.get(sid))

    print("[bench] 读取当前位置(用作保持目标, 确认不会移动)...")
    hold_q = {}
    for sid in DM_KNOWN_IDS:
        online, pos, err, link_ok = dm.probe(sid)
        print(f"  id={sid} online={online} pos={pos:+.4f}rad err={err} link_ok={link_ok}")
        if not online:
            print(f"[FATAL] 电机 {sid} 不在线, 无法安全做 control_mit 基准测试(不知道当前位置)")
            dm.end(); sys.exit(1)
        hold_q[sid] = pos

    print(f"\n[bench] 预热 {N_WARMUP} 帧 (每帧对 {len(DM_KNOWN_IDS)} 个电机各发一次 control_mit, 严格串行)...")
    for _ in range(N_WARMUP):
        for sid in DM_KNOWN_IDS:
            dm.control_mit(sid, TEST_KP, TEST_KD, hold_q[sid], 0.0, 0.0)

    print(f"[bench] 正式采样 {N_SAMPLES} 帧...")
    frame_times = []
    per_motor_times = {sid: [] for sid in DM_KNOWN_IDS}
    t_start = time.monotonic()
    for _ in range(N_SAMPLES):
        f0 = time.monotonic()
        for sid in DM_KNOWN_IDS:
            m0 = time.monotonic()
            dm.control_mit(sid, TEST_KP, TEST_KD, hold_q[sid], 0.0, 0.0)
            per_motor_times[sid].append(time.monotonic() - m0)
        frame_times.append(time.monotonic() - f0)
    t_total = time.monotonic() - t_start

    print("\n[bench] 结果 (严格串行, 2 个电机一帧):")
    print(f"  总耗时={t_total:.3f}s  帧数={N_SAMPLES}  可跑帧率={N_SAMPLES/t_total:.1f} Hz")
    fm = statistics.mean(frame_times) * 1000
    fp95 = sorted(frame_times)[int(len(frame_times)*0.95)] * 1000
    fmax = max(frame_times) * 1000
    print(f"  每帧(2电机)耗时: 均值={fm:.2f}ms  p95={fp95:.2f}ms  max={fmax:.2f}ms")
    for sid in DM_KNOWN_IDS:
        ts = per_motor_times[sid]
        tm = statistics.mean(ts) * 1000
        tp95 = sorted(ts)[int(len(ts)*0.95)] * 1000
        tmax = max(ts) * 1000
        print(f"  电机{sid} 单次control_mit耗时: 均值={tm:.2f}ms  p95={tp95:.2f}ms  max={tmax:.2f}ms")

    print("\n[bench] 验证不动: 复测一次当前位置, 与保持目标对比")
    for sid in DM_KNOWN_IDS:
        online, pos, err, link_ok = dm.probe(sid)
        drift = pos - hold_q[sid]
        print(f"  id={sid} 目标={hold_q[sid]:+.4f} 实测={pos:+.4f} 漂移={drift*1000:+.2f}mrad")

    hz_achievable = N_SAMPLES / t_total
    print(f"\n[结论] 达妙2电机串行一帧耗时 {fm:.2f}ms(均值)/{fp95:.2f}ms(p95) "
          f"→ 理论可跑 {hz_achievable:.1f}Hz (仅达妙自身, 未算lz/evo/IK耗时)")
    if hz_achievable < 50:
        print("      低于常见控制频率(50-100Hz), 建议 _do_dm 线程单独降频运行, "
              "不要用主循环频率硬跑达妙, 否则会拖慢整帧。")
    else:
        print("      看起来能跟上典型 50-100Hz 主循环, 但仍建议实测主循环整体耗时"
              "(lz+evo+dm并行)以确认真实瓶颈。")

    dm.end()


if __name__ == "__main__":
    main()
