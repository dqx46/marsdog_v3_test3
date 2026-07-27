#!/usr/bin/env python3
"""排查 rr_hip (Motor 10, PA43) — 单独测试定位能力"""

import time
import math
import sys
sys.path.insert(0, '/home/jetson/marsdog_ws/mocap_to_real')

from marsdog_control.hardware.motors.evo import MotorEvo
from marsdog_control.config.bus_config import EVO_CAN0_DEVICE

CAN0_PATH = EVO_CAN0_DEVICE
MOTOR_ID = 10  # rr_hip, CAN ID=35 on can0

KP = 35.0
KD = 3.0

def main():
    print(f"=== rr_hip (Motor {MOTOR_ID}) 单独排查 ===")
    print(f"总线: {CAN0_PATH}")
    print()

    evo = MotorEvo()
    if not evo.init_serial(CAN0_PATH):
        print("[ERROR] 无法打开总线!")
        return
    time.sleep(0.3)

    # 检查是否在线
    pos = evo.get_position(MOTOR_ID)
    if pos is None:
        print("[ERROR] Motor 10 不在线!")
        return
    print(f"[OK] Motor 10 在线, 当前位置: {math.degrees(pos):+.2f}°")

    # 使能
    evo.enter_motor_state(MOTOR_ID)
    time.sleep(0.2)

    pos = evo.get_position(MOTOR_ID)
    print(f"使能后位置: {math.degrees(pos):+.2f}°")
    print()

    # 测试1: 保持当前位置，看漂移
    print("── 测试1: 保持当前位置 3秒，观察漂移 ──")
    target = pos
    print(f"目标: {math.degrees(target):+.2f}°")
    
    for i in range(30):
        evo.ptm_control(MOTOR_ID, target, 0.0, KP, KD, 0.0)
        time.sleep(0.1)
    
    pos_after = evo.get_position(MOTOR_ID)
    drift = math.degrees(pos_after - target)
    print(f"3秒后实际: {math.degrees(pos_after):+.2f}°, 漂移: {drift:+.2f}°")
    if abs(drift) > 3.0:
        print(f"  ⚠️  漂移 {abs(drift):.1f}° > 3° — 电机可能有问题!")
    else:
        print(f"  ✓ 漂移正常")
    print()

    # 测试2: 小幅度来回摆动 (±5°)
    print("── 测试2: ±5° 来回摆动，测跟踪能力 ──")
    center = target
    amplitude = math.radians(5.0)
    errors = []
    
    for cycle in range(3):
        # 正向
        tgt = center + amplitude
        print(f"  → 目标: {math.degrees(tgt):+.2f}°", end="")
        for i in range(15):
            evo.ptm_control(MOTOR_ID, tgt, 0.0, KP, KD, 0.0)
            time.sleep(0.1)
        actual = evo.get_position(MOTOR_ID)
        err = math.degrees(actual - tgt)
        errors.append(abs(err))
        print(f"  实际: {math.degrees(actual):+.2f}°  误差: {err:+.2f}°")
        
        # 反向
        tgt = center - amplitude
        print(f"  ← 目标: {math.degrees(tgt):+.2f}°", end="")
        for i in range(15):
            evo.ptm_control(MOTOR_ID, tgt, 0.0, KP, KD, 0.0)
            time.sleep(0.1)
        actual = evo.get_position(MOTOR_ID)
        err = math.degrees(actual - tgt)
        errors.append(abs(err))
        print(f"  实际: {math.degrees(actual):+.2f}°  误差: {err:+.2f}°")
    
    avg_err = sum(errors) / len(errors)
    max_err = max(errors)
    print(f"\n  平均误差: {avg_err:.2f}°, 最大误差: {max_err:.2f}°")
    if max_err > 5.0:
        print(f"  ⚠️  跟踪很差! 可能是: 机械松动/电机故障/编码器问题")
    elif max_err > 2.0:
        print(f"  ⚠️  跟踪一般，有延迟或摩擦")
    else:
        print(f"  ✓ 跟踪良好")
    print()

    # 测试3: 对比 rl_hip (Motor 7) 作为参照
    print("── 测试3: 对比 rl_hip (Motor 7) 作为参照 ──")
    MOTOR_RL = 7
    pos_rl = evo.get_position(MOTOR_RL)
    if pos_rl is None:
        print("  [SKIP] Motor 7 不在线")
    else:
        evo.enter_motor_state(MOTOR_RL)
        time.sleep(0.2)
        center_rl = pos_rl
        
        # 同样的±5°测试
        errors_rl = []
        for cycle in range(2):
            tgt = center_rl + amplitude
            for i in range(15):
                evo.ptm_control(MOTOR_RL, tgt, 0.0, KP, KD, 0.0)
                time.sleep(0.1)
            actual = evo.get_position(MOTOR_RL)
            errors_rl.append(abs(math.degrees(actual - tgt)))
            
            tgt = center_rl - amplitude
            for i in range(15):
                evo.ptm_control(MOTOR_RL, tgt, 0.0, KP, KD, 0.0)
                time.sleep(0.1)
            actual = evo.get_position(MOTOR_RL)
            errors_rl.append(abs(math.degrees(actual - tgt)))
        
        avg_rl = sum(errors_rl) / len(errors_rl)
        max_rl = max(errors_rl)
        print(f"  rl_hip (Motor 7): 平均误差={avg_rl:.2f}°, 最大={max_rl:.2f}°")
        print(f"  rr_hip (Motor 10): 平均误差={avg_err:.2f}°, 最大={max_err:.2f}°")
        
        if max_err > max_rl * 3:
            print(f"  ⚠️  rr_hip 比 rl_hip 差 {max_err/max_rl:.1f} 倍! 硬件问题可能性大")
        
        # 回位
        for i in range(10):
            evo.ptm_control(MOTOR_RL, center_rl, 0.0, KP, KD, 0.0)
            time.sleep(0.1)

    # 回位 rr_hip
    print("\n回到原位...")
    for i in range(15):
        evo.ptm_control(MOTOR_ID, target, 0.0, KP, KD, 0.0)
        time.sleep(0.1)
    
    final_pos = evo.get_position(MOTOR_ID)
    print(f"最终位置: {math.degrees(final_pos):+.2f}° (目标: {math.degrees(target):+.2f}°)")
    print("\n=== 测试完成 ===")

if __name__ == "__main__":
    main()
