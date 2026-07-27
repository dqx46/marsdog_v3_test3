#!/usr/bin/env python3
"""逐关节方向验证：每个关节给一个小动作，用户确认方向正确性。
用法: python3 test_directions.py [关节编号]
不带参数则从头开始逐个测试。
"""
import sys, time, math
sys.path.insert(0, '.')

from marsdog_control.config.joints import JOINT_MAP, JOINT_BY_NAME, ALL_IDS
from marsdog_control.hardware.motors.lingzu import MotorLz
from marsdog_control.hardware.motors.evo import MotorEvo
from marsdog_control.config.bus_config import LZ_SERIAL_DEVICE, LZ_CAN1_DEVICE, EVO_CAN0_DEVICE
from marsdog_control.motion.kinematics import urdf_to_motor

BAUD = 921600

# 测试列表: (关节名, URDF角度增量, 预期物理运动描述)
TESTS = [
    # -- 前左腿 --
    ("fl_hip_pitch",   -0.15, "左前大腿 向前抬（膝盖前移）"),
    ("fl_hip_pitch",   +0.15, "左前大腿 向后摆（膝盖后移）"),
    ("fl_thigh_roll",  -0.08, "左前腿 向外展开（远离身体）"),
    ("fl_thigh_roll",  +0.08, "左前腿 向内收（靠近身体）"),
    ("fl_calf",        +0.15, "左前小腿 伸直（膝盖打开）"),
    ("fl_calf",        -0.15, "左前小腿 弯曲（膝盖收缩）"),
    # -- 前右腿 --
    ("fr_hip_pitch",   -0.15, "右前大腿 向前抬（膝盖前移）"),
    ("fr_hip_pitch",   +0.15, "右前大腿 向后摆（膝盖后移）"),
    ("fr_thigh_roll",  -0.08, "右前腿 向外展开（远离身体）"),
    ("fr_thigh_roll",  +0.08, "右前腿 向内收（靠近身体）"),
    ("fr_calf",        +0.15, "右前小腿 伸直（膝盖打开）"),
    ("fr_calf",        -0.15, "右前小腿 弯曲（膝盖收缩）"),
    # -- 后左腿 --
    ("rl_hip",         -0.08, "左后腿 向外展开（远离身体）"),
    ("rl_hip",         +0.08, "左后腿 向内收（靠近身体）"),
    ("rl_thigh",       +0.15, "左后大腿 向前摆（膝盖前移）"),
    ("rl_thigh",       -0.15, "左后大腿 向后摆（膝盖后移）"),
    ("rl_calf",        +0.15, "左后小腿 伸直（膝盖打开）"),
    ("rl_calf",        -0.15, "左后小腿 弯曲（膝盖收缩）"),
    # -- 后右腿 --
    ("rr_hip",         +0.08, "右后腿 向外展开（远离身体）"),
    ("rr_hip",         -0.08, "右后腿 向内收（靠近身体）"),
    ("rr_thigh",       +0.15, "右后大腿 向前摆（膝盖前移）"),
    ("rr_thigh",       -0.15, "右后大腿 向后摆（膝盖后移）"),
    ("rr_calf",        +0.15, "右后小腿 伸直（膝盖打开）"),
    ("rr_calf",        -0.15, "右后小腿 弯曲（膝盖收缩）"),
]

def send_one(lz, evo, j, motor_rad, kp=20.0, kd=3.0):
    if j.mtype == "lz":
        lz.mit_control(j.motor_id, motor_rad, 0.0, kp, kd, 0.0)
    else:
        evo.ptm_control(j.motor_id, motor_rad, 0.0, kp, kd, 0.0)

def main():
    start_idx = int(sys.argv[1]) if len(sys.argv) > 1 else 0

    lz = MotorLz()
    evo = MotorEvo()

    print("[init] 初始化电机...")
    lz.init_serial(LZ_SERIAL_DEVICE, BAUD)
    lz.init_can1_serial(LZ_CAN1_DEVICE, BAUD)
    evo.init_serial(EVO_CAN0_DEVICE, BAUD)

    for j in JOINT_MAP:
        if j.mtype == "evo" and evo.is_connected[j.motor_id - 1]:
            evo.enter_motor_state(j.motor_id)
            time.sleep(0.002)
    time.sleep(0.5)

    print(f"[ready] 共 {len(TESTS)} 项测试，从第 {start_idx} 项开始")
    print("  操作: Enter=执行动作  n=跳过  q=退出\n")

    for idx in range(start_idx, len(TESTS)):
        joint_name, urdf_delta, expected = TESTS[idx]
        j = JOINT_BY_NAME[joint_name]
        mid = j.motor_id
        conn = lz.is_connected[mid-1] if j.mtype == "lz" else evo.is_connected[mid-1]

        if not conn:
            print(f"[{idx:2d}] SKIP {joint_name} (Motor {mid}) 离线")
            continue

        # 读当前位置作为基准
        if j.mtype == "lz":
            cur_motor = lz.get_position(mid)
        else:
            cur_motor = evo.get_position(mid)

        motor_delta = urdf_to_motor(j, urdf_delta)
        target_motor = cur_motor + motor_delta

        print(f"{'='*56}")
        print(f"  [{idx:2d}] {joint_name}  (Motor {mid}, sign={j.sign:+d})")
        print(f"  URDF增量: {math.degrees(urdf_delta):+.1f}°  电机增量: {math.degrees(motor_delta):+.1f}°")
        print(f"  预期动作: {expected}")
        print(f"{'='*56}")

        cmd = input("  Enter=执行 / n=跳过 / q=退出: ").strip().lower()
        if cmd == 'q':
            break
        if cmd == 'n':
            continue

        # 缓慢移动到目标 (1s)
        steps = 50
        for i in range(steps + 1):
            alpha = i / steps
            m = cur_motor + motor_delta * alpha
            send_one(lz, evo, j, m)
            time.sleep(0.02)

        input("  >>> 观察完毕，Enter 回正...")

        # 回正
        for i in range(steps + 1):
            alpha = i / steps
            m = target_motor + (cur_motor - target_motor) * alpha
            send_one(lz, evo, j, m)
            time.sleep(0.02)

        print("  >>> 已回正\n")

    print("\n[done] 测试结束")
    lz.end()
    evo.end()

if __name__ == "__main__":
    main()
