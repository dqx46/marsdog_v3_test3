#!/usr/bin/env python3
"""验证 IMU 坐标变换是否正确 (世界坐标系: +X前 +Y左 +Z上)

预期:
  抬头 → pitch 增大 (正)
  左倾(左高右低) → roll 增大 (正)
  左转 → yaw 增大 (正)
"""
import time
import math
from marsdog_control.hardware.sensors.imu_wt901 import ImuWT901
from marsdog_control.config.bus_config import IMU_DEVICE, IMU_BAUD

imu = ImuWT901(IMU_DEVICE, IMU_BAUD)
if not imu.begin():
    print("IMU 未连接!")
    exit(1)

time.sleep(0.5)
print(f"IMU 在线, 帧数={imu.update_count}")
imu.calibrate(1.0)

print("\n世界坐标系: +X朝前, +Y朝左, +Z朝上")
print("验证: 抬头→pitch+, 左倾→roll+, 左转→yaw+")
print("─" * 60)
print(f"  {'Roll':>8}  {'Pitch':>8}  {'Yaw':>8}  |  {'gRoll':>7}  {'gPitch':>7}")
print("─" * 60)

try:
    while True:
        r = math.degrees(imu.roll)
        p = math.degrees(imu.pitch)
        y = math.degrees(imu.yaw)
        gr = math.degrees(imu.gyro_roll)
        gp = math.degrees(imu.gyro_pitch)
        print(f"  {r:+8.2f}  {p:+8.2f}  {y:+8.2f}  |  {gr:+7.1f}  {gp:+7.1f}")
        time.sleep(0.5)
except KeyboardInterrupt:
    pass
imu.close()
print("\n完成。")
