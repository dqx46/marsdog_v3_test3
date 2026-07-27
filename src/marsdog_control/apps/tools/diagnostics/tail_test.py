#!/usr/bin/env python3
"""Tail motor test script.

Every run:
1. Enable both tail motors.
2. Command both motors back to the saved 0 deg position.
3. Raise pitch and wag yaw.
4. On exit, return both motors to 0 deg and disable them.

Edit the parameters below for quick testing.
"""

import math
import struct
import time

import marsdog_control.apps.tools.misc.serial_fallback as serial

from marsdog_control.config.bus_config import TAIL_485_DEVICE

# ==========================
# Easy-to-edit parameters
# ==========================
PITCH_ID = 24
YAW_ID = 25

PITCH_UP_DEG = 50.0
WAG_DEG = 400.0
WAG_HZ = 1.8
RUN_SECONDS = 16.0
MAX_SPEED_DPS = 2200.0

BAUD = 115200

# Startup sequence timing
RETURN_ZERO_S = 0.8
PITCH_UP_S = 0.8
COMMAND_DT_S = 0.025
DISABLE_DELAY_S = 0.2


def build_packet(cmd, motor_id, data=b""):
    packet = bytearray([0x3E, cmd, motor_id, len(data)])
    packet.append(sum(packet) & 0xFF)
    packet.extend(data)
    if data:
        packet.append(sum(data) & 0xFF)
    return bytes(packet)


def send_packet(ser, cmd, motor_id, data=b"", reply_len=None, timeout_s=0.03):
    old_timeout = ser.timeout
    ser.timeout = timeout_s
    ser.reset_input_buffer()
    packet = build_packet(cmd, motor_id, data)
    ser.write(packet)
    ser.flush()
    if reply_len:
        reply = ser.read(reply_len)
    else:
        # Most control commands reply with status2 (13 bytes), while simple
        # commands reply with 5 bytes. Read whatever is immediately available.
        time.sleep(timeout_s)
        reply = ser.read(ser.in_waiting or 0)
    ser.timeout = old_timeout
    return reply


def enable_motor(ser, motor_id):
    reply = send_packet(ser, 0x88, motor_id, reply_len=5, timeout_s=0.08)
    print(f"enable ID{motor_id}: {reply.hex(' ')}")


def disable_motor(ser, motor_id):
    reply = send_packet(ser, 0x80, motor_id, reply_len=5, timeout_s=0.08)
    print(f"disable ID{motor_id}: {reply.hex(' ')}")


def position_abs(ser, motor_id, deg, speed_dps=MAX_SPEED_DPS):
    angle = int(round(deg * 100.0))       # 0.01 deg
    speed = int(round(speed_dps * 100.0)) # 0.01 deg/s
    data = struct.pack("<qI", angle, speed)
    send_packet(ser, 0xA4, motor_id, data, timeout_s=0.015)


def main():
    print(f"port: {TAIL_485_DEVICE}")
    print(f"pitch={PITCH_UP_DEG:+.1f} deg, yaw=±{WAG_DEG:.1f} deg, "
          f"speed={MAX_SPEED_DPS:.1f} deg/s")

    with serial.Serial(TAIL_485_DEVICE, BAUD, timeout=0.1, write_timeout=0.1) as ser:
        try:
            for motor_id in (PITCH_ID, YAW_ID):
                enable_motor(ser, motor_id)
                time.sleep(0.05)

            print("return to saved zero")
            position_abs(ser, PITCH_ID, 0.0)
            position_abs(ser, YAW_ID, 0.0)
            time.sleep(RETURN_ZERO_S)

            print(f"pitch up -> {PITCH_UP_DEG:+.1f} deg")
            position_abs(ser, PITCH_ID, PITCH_UP_DEG)
            position_abs(ser, YAW_ID, 0.0)
            time.sleep(PITCH_UP_S)

            print("start wag")
            t0 = time.monotonic()
            while time.monotonic() - t0 < RUN_SECONDS:
                t = time.monotonic() - t0
                yaw = WAG_DEG * math.sin(2.0 * math.pi * WAG_HZ * t)
                position_abs(ser, PITCH_ID, PITCH_UP_DEG)
                position_abs(ser, YAW_ID, yaw)
                time.sleep(COMMAND_DT_S)
        finally:
            print("return to zero and disable")
            position_abs(ser, YAW_ID, 0.0)
            position_abs(ser, PITCH_ID, 0.0)
            time.sleep(RETURN_ZERO_S)
            for motor_id in (PITCH_ID, YAW_ID):
                disable_motor(ser, motor_id)
                time.sleep(DISABLE_DELAY_S)
            print("done")


if __name__ == "__main__":
    main()
