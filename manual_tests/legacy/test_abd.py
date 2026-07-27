#!/usr/bin/env python3
"""髋外展方向验证：先站起来，然后逐条腿向外展开再回正，用户确认方向是否正确。
也支持同时打开所有外展: python3 test_abd.py all
"""
import sys, time, math
sys.path.insert(0, '.')

from marsdog_control.config.joints import JOINT_MAP, JOINT_BY_NAME, ALL_IDS
from marsdog_control.hardware.motors.lingzu import MotorLz
from marsdog_control.hardware.motors.evo import MotorEvo
from marsdog_control.config.bus_config import LZ_SERIAL_DEVICE, LZ_CAN1_DEVICE, EVO_CAN0_DEVICE
from marsdog_control.motion.kinematics import urdf_to_motor
from marsdog_control.motion.gait_controller import StandController

BAUD = 921600
ABD_ANGLE = 0.30  # ~17° 外展测试角度，非常明显

JOINTS_TO_TEST = [
    ("fl_thigh_roll", -1),  # 左前: 负URDF = 向外
    ("fr_thigh_roll", -1),  # 右前: 负URDF = 向外
    ("rl_hip",        +1),  # 左后: 正URDF = 向外 (已修正)
    ("rr_hip",        -1),  # 右后: 负URDF = 向外 (已修正)
]

def send_all_targets(lz, evo, targets, kp=30.0, kd=4.0):
    for j in JOINT_MAP:
        mid = j.motor_id
        if mid not in targets:
            continue
        conn = lz.is_connected[mid-1] if j.mtype == "lz" else evo.is_connected[mid-1]
        if not conn:
            continue
        motor_rad = targets[mid]
        if j.mtype == "lz":
            lz.mit_control(mid, motor_rad, 0.0, kp, kd, 0.0)
        else:
            evo.ptm_control(mid, motor_rad, 0.0, kp, kd, 0.0)

def smooth_transition(lz, evo, start, end, duration=2.0, kp=30.0, kd=4.0):
    """从 start 平滑过渡到 end (dict of motor_id -> rad)"""
    hz = 100
    steps = int(duration * hz)
    for i in range(steps + 1):
        alpha = i / steps
        alpha_smooth = alpha * alpha * (3 - 2 * alpha)  # smoothstep
        targets = {}
        for mid in end:
            s = start.get(mid, end[mid])
            targets[mid] = s + (end[mid] - s) * alpha_smooth
        send_all_targets(lz, evo, targets, kp, kd)
        time.sleep(1.0 / hz)

def read_positions(lz, evo):
    pos = {}
    for j in JOINT_MAP:
        mid = j.motor_id
        if j.mtype == "lz":
            p = lz.get_position(mid)
        else:
            p = evo.get_position(mid)
        pos[mid] = p if p is not None else 0.0
    return pos

def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else None
    valid_modes = ["fl", "fr", "rl", "rr", "all"]
    if mode and mode not in valid_modes:
        print(f"用法: python3 test_abd.py [fl|fr|rl|rr|all]")
        return

    lz = MotorLz()
    evo = MotorEvo()

    print("[init] 初始化电机...")
    lz.init_serial(LZ_SERIAL_DEVICE, BAUD)
    lz.init_can1_serial(LZ_CAN1_DEVICE, BAUD)
    evo.init_serial(EVO_CAN0_DEVICE, BAUD)

    for j in JOINT_MAP:
        if j.mtype == "lz" and lz.is_connected[j.motor_id - 1]:
            lz.enable(j.motor_id)
            time.sleep(0.002)
    time.sleep(0.05)
    for j in JOINT_MAP:
        if j.mtype == "evo" and evo.is_connected[j.motor_id - 1]:
            evo.enter_motor_state(j.motor_id)
            time.sleep(0.005)
    time.sleep(0.5)

    # 读取当前位置
    cur_pos = read_positions(lz, evo)

    # 站立目标
    stand = StandController(body_height=0.24)
    stand_targets = stand.get_targets(0)

    print("[stand] 先站起来 (3秒)...")
    smooth_transition(lz, evo, cur_pos, stand_targets, duration=3.0)
    print("[stand] 已站立！\n")

    # 持续发送站立指令保持姿态
    def hold_standing():
        send_all_targets(lz, evo, stand_targets)

    if mode == "all":
        # 同时打开所有外展
        abd_targets = dict(stand_targets)
        for joint_name, sign in JOINTS_TO_TEST:
            j = JOINT_BY_NAME[joint_name]
            urdf_abd = ABD_ANGLE * sign
            abd_targets[j.motor_id] = urdf_to_motor(j, urdf_abd)

        print("="*50)
        print(f"  同时外展四条腿，每条向外 {math.degrees(ABD_ANGLE):.1f}°")
        print("="*50)
        input("  按 Enter 开始外展...")

        smooth_transition(lz, evo, stand_targets, abd_targets, duration=1.5)

        print("  >>> 已外展。观察四条腿是否都向外打开！")
        print("      如有向内收拢的，说明方向反了。")
        input("  按 Enter 回正...")

        smooth_transition(lz, evo, abd_targets, stand_targets, duration=1.5)
        print("  >>> 已回正")
    else:
        # 逐条腿测试
        tests = JOINTS_TO_TEST
        if mode:
            tests = [(n, s) for n, s in tests if n.startswith(mode)]

        for joint_name, sign in tests:
            j = JOINT_BY_NAME[joint_name]
            mid = j.motor_id
            conn = lz.is_connected[mid-1] if j.mtype == "lz" else evo.is_connected[mid-1]
            if not conn:
                print(f"[SKIP] {joint_name} (Motor {mid}) 离线")
                continue

            urdf_abd = ABD_ANGLE * sign
            abd_targets = dict(stand_targets)
            abd_targets[mid] = urdf_to_motor(j, urdf_abd)

            print(f"\n{'='*50}")
            print(f"  测试: {joint_name}  (Motor {mid})")
            print(f"  预期: 该腿向外展开 {math.degrees(ABD_ANGLE):.1f}°")
            print(f"{'='*50}")
            input("  按 Enter 开始外展...")

            smooth_transition(lz, evo, stand_targets, abd_targets, duration=1.0)

            print(f"  >>> 已外展，观察该腿是否向外打开")
            input("  按 Enter 回正...")

            smooth_transition(lz, evo, abd_targets, stand_targets, duration=1.0)
            print(f"  >>> 已回正")

    # 趴下
    print("\n[down] 缓慢趴下 (2秒)...")
    smooth_transition(lz, evo, stand_targets, cur_pos, duration=2.0)

    print("\n[done] 测试完成，关闭电机")
    for j in JOINT_MAP:
        if j.mtype == "lz":
            lz.disable(j.motor_id)
        else:
            evo.enter_rest_state(j.motor_id)
        time.sleep(0.002)
    lz.end()
    evo.end()

if __name__ == "__main__":
    main()
