#!/usr/bin/env python3
"""读取 WT901 IMU 原始数据，每 0.7 秒打印一行。Ctrl+C 退出。"""

import marsdog_control.apps.tools.misc.serial_fallback as serial
import struct
import time
import sys

from marsdog_control.config.bus_config import IMU_DEVICE, IMU_BAUD

PORT = sys.argv[1] if len(sys.argv) > 1 else IMU_DEVICE
BAUD = IMU_BAUD

def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.1,
                        exclusive=True, rtscts=False, dsrdtr=False)
    ser.reset_input_buffer()
    time.sleep(0.5)

    acc = [0.0, 0.0, 0.0]
    gyro = [0.0, 0.0, 0.0]
    angle = [0.0, 0.0, 0.0]
    buf = bytearray()

    print("IMU原始数据 | 端口: %s @ %d" % (PORT, BAUD))
    print("ACC = 加速度(g)  GYRO = 角速度(°/s)  ANGLE = 姿态角(°)")
    print("─" * 90)
    print(f"  {'ACC_X':>7} {'ACC_Y':>7} {'ACC_Z':>7}  |"
          f"  {'GYR_X':>7} {'GYR_Y':>7} {'GYR_Z':>7}  |"
          f"  {'ANG_X':>7} {'ANG_Y':>7} {'ANG_Z':>7}")
    print("─" * 90)

    try:
        while True:
            raw = ser.read(ser.in_waiting or 1)
            if raw:
                buf.extend(raw)
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
                        acc = [d[i] / 32768.0 * 16.0 for i in range(3)]
                    elif ptype == 0x52:
                        gyro = [d[i] / 32768.0 * 2000.0 for i in range(3)]
                    elif ptype == 0x53:
                        angle = [d[i] / 32768.0 * 180.0 for i in range(3)]
                    del buf[:11]

            print(f"  {acc[0]:+7.3f} {acc[1]:+7.3f} {acc[2]:+7.3f}  |"
                  f"  {gyro[0]:+7.1f} {gyro[1]:+7.1f} {gyro[2]:+7.1f}  |"
                  f"  {angle[0]:+7.2f} {angle[1]:+7.2f} {angle[2]:+7.2f}")
            time.sleep(0.7)
    except KeyboardInterrupt:
        print("\n退出。")
    finally:
        ser.close()

if __name__ == "__main__":
    main()
