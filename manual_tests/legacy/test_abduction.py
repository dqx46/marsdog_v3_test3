import time
import math
import sys
import os

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from marsdog_control.hardware.motors.lingzu import MotorLz
from marsdog_control.hardware.motors.evo import MotorEvo
from marsdog_control.config.joints import JOINT_MAP, LZ_CAN_IDS, LZ_SERIAL_IDS, EVO_CAN_IDS
from marsdog_control.config.bus_config import LZ_CAN1_DEVICE, EVO_CAN0_DEVICE, LZ_SERIAL_DEVICE, BAUD
from marsdog_control.motion.gait_controller import StandController

def main():
    print("初始化总线...")
    lz = MotorLz()
    evo = MotorEvo()

    print(f"  Serial ({LZ_SERIAL_DEVICE})...")
    lz.init_serial(LZ_SERIAL_DEVICE, BAUD)
    print(f"  CAN1   ({LZ_CAN1_DEVICE})...")
    lz.init_can1_serial(LZ_CAN1_DEVICE, BAUD)
    print(f"  EVO    ({EVO_CAN0_DEVICE})...")
    evo.init_serial(EVO_CAN0_DEVICE, BAUD)

    # 启动电机
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

    # 获取站立姿态的基础目标
    stand = StandController(body_height=0.24)
    targets = stand.get_targets(0)

    # 外展角度 (向外打开)
    abd_angle = 0.2  # 约 11.5 度

    fl_roll_id = next(j.motor_id for j in JOINT_MAP if j.name == "fl_thigh_roll")
    fr_roll_id = next(j.motor_id for j in JOINT_MAP if j.name == "fr_thigh_roll")
    rl_roll_id = next(j.motor_id for j in JOINT_MAP if j.name == "rl_hip")
    rr_roll_id = next(j.motor_id for j in JOINT_MAP if j.name == "rr_hip")

    targets[fl_roll_id] = abd_angle   # 左前向外 (+)
    targets[fr_roll_id] = -abd_angle  # 右前向外 (-)
    targets[rl_roll_id] = abd_angle   # 左后向外 (+)
    targets[rr_roll_id] = -abd_angle  # 右后向外 (-)

    print(f"\n外展测试：所有腿应向外打开约 11.5°")
    print(f"  FL thigh_roll (ID {fl_roll_id}): +{math.degrees(abd_angle):.1f}°")
    print(f"  FR thigh_roll (ID {fr_roll_id}): -{math.degrees(abd_angle):.1f}°")
    print(f"  RL hip        (ID {rl_roll_id}): +{math.degrees(abd_angle):.1f}°")
    print(f"  RR hip        (ID {rr_roll_id}): -{math.degrees(abd_angle):.1f}°")
    print(f"\n按 Ctrl+C 退出...")

    try:
        while True:
            for j in JOINT_MAP:
                mid = j.motor_id
                if mid in targets:
                    if j.mtype == "lz":
                        lz.send_rad(mid, targets[mid], kp=30.0, kd=3.0)
                    else:
                        evo.send_rad(mid, targets[mid], kp=30.0, kd=3.0)
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\n关闭电机...")
    finally:
        for j in JOINT_MAP:
            if j.mtype == "lz":
                lz.disable(j.motor_id)
            else:
                evo.enter_rest_state(j.motor_id)
            time.sleep(0.002)
        lz.end()
        evo.end()
        print("完成。")

if __name__ == "__main__":
    main()
