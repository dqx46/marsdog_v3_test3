#!/usr/bin/env python3
"""IMU 轴向校正工具

让用户做特定姿态动作，记录 IMU 原始数据，确定坐标变换。
目标世界坐标系: 右手系, +X朝前, +Y朝左, +Z朝上
"""

import marsdog_control.apps.tools.misc.serial_fallback as serial
import struct
import time
import math
from marsdog_control.config.bus_config import IMU_DEVICE, IMU_BAUD

PORT = IMU_DEVICE
BAUD = IMU_BAUD

def read_imu_raw(ser):
    """读取一组完整的 acc/gyro/angle 数据"""
    acc = [0.0, 0.0, 0.0]
    gyro = [0.0, 0.0, 0.0]
    angle = [0.0, 0.0, 0.0]
    
    buf = bytearray()
    got = set()
    t0 = time.time()
    
    while len(got) < 3 and time.time() - t0 < 0.5:
        data = ser.read(64)
        if not data:
            continue
        buf.extend(data)
        while len(buf) >= 11:
            idx = buf.find(0x55)
            if idx < 0:
                buf.clear()
                break
            if idx > 0:
                del buf[:idx]
            if len(buf) < 11:
                break
            frame = bytes(buf[:11])
            chk = sum(frame[:10]) & 0xFF
            if chk != frame[10]:
                del buf[:1]
                continue
            ptype = frame[1]
            d = struct.unpack_from('<hhh', frame, 2)
            if ptype == 0x51:
                acc[0] = d[0] / 32768.0 * 16.0
                acc[1] = d[1] / 32768.0 * 16.0
                acc[2] = d[2] / 32768.0 * 16.0
                got.add('acc')
            elif ptype == 0x52:
                gyro[0] = d[0] / 32768.0 * 2000.0
                gyro[1] = d[1] / 32768.0 * 2000.0
                gyro[2] = d[2] / 32768.0 * 2000.0
                got.add('gyro')
            elif ptype == 0x53:
                angle[0] = d[0] / 32768.0 * 180.0
                angle[1] = d[1] / 32768.0 * 180.0
                angle[2] = d[2] / 32768.0 * 180.0
                got.add('angle')
            del buf[:11]
    return acc, gyro, angle

def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    ser.reset_input_buffer()
    time.sleep(0.3)
    
    print("=" * 60)
    print("  IMU 轴向校正工具")
    print("  目标: +X朝前, +Y朝左, +Z朝上 (右手坐标系)")
    print("=" * 60)
    print()
    print("操作说明: 按 Enter 开始采集当前姿态数据 (采集2秒取平均)")
    print("          每个姿态保持不动后按 Enter")
    print()
    
    poses = [
        ("静止水平 (狗正常站立)", "重力应该全在Z轴(负方向)"),
        ("抬头 (绕Y轴, 狗头朝上约30-45°)", "pitch变正, acc_x变负"),
        ("向左倾斜 (绕X轴, 左侧抬高约30°)", "roll变正, acc_y变负"),
        ("左转 (绕Z轴, 逆时针转约90°)", "yaw变正"),
    ]
    
    results = []
    for i, (name, expect) in enumerate(poses):
        print(f"\n{'─'*60}")
        print(f"  姿态 {i+1}/{len(poses)}: {name}")
        print(f"  预期: {expect}")
        input("  >>> 摆好姿态后按 Enter 开始采集...")
        
        # 采集 2 秒
        samples_acc = []
        samples_gyro = []
        samples_angle = []
        ser.reset_input_buffer()
        t0 = time.time()
        while time.time() - t0 < 2.0:
            acc, gyro, angle = read_imu_raw(ser)
            samples_acc.append(acc)
            samples_gyro.append(gyro)
            samples_angle.append(angle)
            time.sleep(0.01)
        
        n = len(samples_acc)
        avg_acc = [sum(s[j] for s in samples_acc)/n for j in range(3)]
        avg_gyro = [sum(s[j] for s in samples_gyro)/n for j in range(3)]
        avg_angle = [sum(s[j] for s in samples_angle)/n for j in range(3)]
        
        print(f"  采集 {n} 帧")
        print(f"  ACC (g):   X={avg_acc[0]:+7.3f}  Y={avg_acc[1]:+7.3f}  Z={avg_acc[2]:+7.3f}")
        print(f"  GYRO(°/s): X={avg_gyro[0]:+7.1f}  Y={avg_gyro[1]:+7.1f}  Z={avg_gyro[2]:+7.1f}")
        print(f"  ANGLE(°):  X={avg_angle[0]:+7.2f}  Y={avg_angle[1]:+7.2f}  Z={avg_angle[2]:+7.2f}")
        results.append((name, avg_acc, avg_gyro, avg_angle))
    
    print(f"\n{'═'*60}")
    print("  采集完成! 汇总:")
    print(f"{'═'*60}")
    print(f"  {'姿态':<30} {'ACC_X':>7} {'ACC_Y':>7} {'ACC_Z':>7} | {'ANG_X':>7} {'ANG_Y':>7} {'ANG_Z':>7}")
    print(f"  {'─'*30} {'─'*7} {'─'*7} {'─'*7}   {'─'*7} {'─'*7} {'─'*7}")
    for name, acc, gyro, angle in results:
        print(f"  {name:<30} {acc[0]:+7.3f} {acc[1]:+7.3f} {acc[2]:+7.3f} | {angle[0]:+7.2f} {angle[1]:+7.2f} {angle[2]:+7.2f}")
    
    ser.close()
    print("\n完成。根据以上数据确定 IMU 原始轴 → 世界坐标系的映射关系。")

if __name__ == "__main__":
    main()
