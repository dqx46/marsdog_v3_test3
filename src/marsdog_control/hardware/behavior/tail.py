#!/usr/bin/env python3
"""Background tail behavior controller for Marsdog.

This module keeps tail serial traffic out of the 200 Hz leg control loop.
It assumes the motor ROM zero has already been set mechanically.
"""

import math
import struct
import threading
import time

# [解耦] 真实实现已下沉到此 src 模块; 保持逐字一致的扁平 import(serial 本地垫片 +
# bus_config), 由 ensure_legacy_path() 把 mocap_to_real 置于 sys.path 前部保证解析。
from marsdog_control.compat import ensure_legacy_path as _ensure_legacy_path
_ensure_legacy_path()

import serial

from marsdog_control.config.bus_config import TAIL_485_DEVICE

PITCH_ID = 24
YAW_ID = 25

PITCH_UP_DEG = 50.0
WAG_DEG = 400.0

TROT_WAG_HZ = 1.8
TROT_SPEED_DPS = 2200.0

LIE_DOWN_WAG_HZ = 0.6
LIE_DOWN_SPEED_DPS = 800.0

BAUD = 115200
COMMAND_DT_S = 0.025
STARTUP_ZERO_S = 0.8
DISABLE_DELAY_S = 0.15


def _build_packet(cmd, motor_id, data=b""):
    packet = bytearray([0x3E, cmd, motor_id, len(data)])
    packet.append(sum(packet) & 0xFF)
    packet.extend(data)
    if data:
        packet.append(sum(data) & 0xFF)
    return bytes(packet)


class TailController:
    """Small async state machine for tail pitch/yaw motors."""

    def __init__(self, device=TAIL_485_DEVICE):
        self.device = device
        self._ser = None
        self._thread = None
        self._running = False
        self._lock = threading.Lock()
        self._mode = "stand"
        self._phase_t0 = time.monotonic()
        self._connected = False

    @property
    def connected(self):
        return self._connected

    def begin(self):
        try:
            self._ser = serial.Serial(
                self.device, BAUD, timeout=0.05, write_timeout=0.05
            )
            for motor_id in (PITCH_ID, YAW_ID):
                self._enable_motor(motor_id)
                time.sleep(0.03)
            print(f"[tail] 已连接 {self.device}, 启动先回零")
            self._position_abs(PITCH_ID, 0.0, TROT_SPEED_DPS)
            self._position_abs(YAW_ID, 0.0, TROT_SPEED_DPS)
            time.sleep(STARTUP_ZERO_S)
            self._running = True
            self._thread = threading.Thread(
                target=self._loop, name="tail-control", daemon=True
            )
            self._thread.start()
            self._connected = True
            return True
        except Exception as e:
            print(f"[tail] 启动失败: {e}")
            self.close(disable=False)
            return False

    def set_mode(self, mode):
        if mode not in ("stand", "trot", "lie_down"):
            mode = "stand"
        with self._lock:
            if mode != self._mode:
                self._mode = mode
                self._phase_t0 = time.monotonic()

    def close(self, disable=True):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None
        if self._ser is not None:
            try:
                if disable:
                    print("[tail] 回零并失能")
                    self._position_abs(YAW_ID, 0.0, TROT_SPEED_DPS)
                    self._position_abs(PITCH_ID, 0.0, TROT_SPEED_DPS)
                    time.sleep(STARTUP_ZERO_S)
                    for motor_id in (PITCH_ID, YAW_ID):
                        self._disable_motor(motor_id)
                        time.sleep(DISABLE_DELAY_S)
                self._ser.close()
            except Exception as e:
                print(f"[tail] 关闭失败: {e}")
            finally:
                self._ser = None
                self._connected = False

    def _loop(self):
        while self._running:
            with self._lock:
                mode = self._mode
                t = time.monotonic() - self._phase_t0

            try:
                if mode == "trot":
                    yaw = WAG_DEG * math.sin(2.0 * math.pi * TROT_WAG_HZ * t)
                    self._position_abs(PITCH_ID, PITCH_UP_DEG, TROT_SPEED_DPS)
                    self._position_abs(YAW_ID, yaw, TROT_SPEED_DPS)
                elif mode == "lie_down":
                    yaw = WAG_DEG * math.sin(2.0 * math.pi * LIE_DOWN_WAG_HZ * t)
                    self._position_abs(PITCH_ID, PITCH_UP_DEG, LIE_DOWN_SPEED_DPS)
                    self._position_abs(YAW_ID, yaw, LIE_DOWN_SPEED_DPS)
                else:
                    self._position_abs(YAW_ID, 0.0, TROT_SPEED_DPS)
                    self._position_abs(PITCH_ID, 0.0, TROT_SPEED_DPS)
            except Exception as e:
                print(f"[tail] 控制线程停止: {e}")
                self._running = False
                break

            time.sleep(COMMAND_DT_S)

    def _send_packet(self, cmd, motor_id, data=b"", reply_len=None, timeout_s=0.015):
        if self._ser is None:
            return b""
        old_timeout = self._ser.timeout
        self._ser.timeout = timeout_s
        self._ser.reset_input_buffer()
        self._ser.write(_build_packet(cmd, motor_id, data))
        self._ser.flush()
        reply = self._ser.read(reply_len) if reply_len else b""
        self._ser.timeout = old_timeout
        return reply

    def _enable_motor(self, motor_id):
        return self._send_packet(0x88, motor_id, reply_len=5, timeout_s=0.08)

    def _disable_motor(self, motor_id):
        return self._send_packet(0x80, motor_id, reply_len=5, timeout_s=0.08)

    def _position_abs(self, motor_id, deg, speed_dps):
        angle = int(round(deg * 100.0))
        speed = int(round(speed_dps * 100.0))
        data = struct.pack("<qI", angle, speed)
        return self._send_packet(0xA4, motor_id, data, timeout_s=0.005)
