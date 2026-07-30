"""ENCOS/INCOS EC-A2806 USB-CAN motor driver.

The protocol is ported from body_code_incos_usb(1). It uses the same AT
USB-CAN serial framing as the other serial CAN adapters in this project.
All public position/speed units are radians and radians/second.
"""

import math
import struct
import threading
import time

from marsdog_control.hardware.motors.can_serial import CAN_EFF_FLAG, CanSerial

BAUD = 921600
MAX_ID = 32

KP_MIN, KP_MAX = 0.0, 500.0
KD_MIN, KD_MAX = 0.0, 5.0
POS_MIN, POS_MAX = -12.5, 12.5
SPEED_MIN, SPEED_MAX = -18.0, 18.0
TORQUE_MIN, TORQUE_MAX = -12.0, 12.0
CURRENT_MAX_A = 10.0
TORQUE_COEFF = 1.35

QUERY_POSITION = 1
QUERY_SPEED = 2
QUERY_CURRENT = 3


def _clamp(x, lo, hi):
    return max(lo, min(hi, x))


def _float_to_uint(x, x_min, x_max, bits):
    x = _clamp(x, x_min, x_max)
    return int((x - x_min) * ((1 << bits) - 1) / (x_max - x_min))


def _uint_to_float(v, x_min, x_max, bits):
    return float(v) * (x_max - x_min) / ((1 << bits) - 1) + x_min


def _get_float32_be(data):
    return struct.unpack(">f", bytes(data))[0]


class MotorIncos:
    # 保活重发频率：主环活跃时静默，出现下发空档时以此频率重发上次 MIT 设定值，
    # 防止 MIT 看门狗超时掉力（否则站立/校准等无下发窗口里前腿小腿会软塌）。
    KEEPALIVE_HZ = 200

    def __init__(self):
        self._serial = CanSerial()
        self._lock = threading.Lock()
        self._owns_serial = True
        self._running = False
        self._thread = None
        self._active_ids = []

        # 保活：记录每个电机最后一次 MIT 设定值 (pos, vel, kp, kd, trq)。
        self._cmd_lock = threading.Lock()
        self._last_cmd = {}
        self._last_tx_monotonic = 0.0
        self._keepalive_enabled = False
        self._keepalive_thread = None

        self.position = [0.0] * MAX_ID
        self.velocity = [0.0] * MAX_ID
        self.torque = [0.0] * MAX_ID
        self.current = [0.0] * MAX_ID
        self.motor_temperature = [0.0] * MAX_ID
        self.mos_temperature = [0.0] * MAX_ID
        self.fault = [0] * MAX_ID
        self.is_connected = [False] * MAX_ID
        self.is_enabled = [False] * MAX_ID
        self.rx_count = [0] * MAX_ID
        self.tx_count = [0] * MAX_ID

    def begin(self, device, motor_ids=(2, 3, 6, 7), baud=BAUD):
        self._active_ids = list(motor_ids)
        if not self._serial.begin(device, baud):
            return False

        # 与 static_test.probe_incos 同策略：逐台 flush→查询→短窗收包，失败再试，
        # 避免一次广播多 ID 时个别应答被冲掉（曾出现 ID3 偶发漏检、static_test 却全绿）。
        time.sleep(0.05)
        for mid in self._active_ids:
            ok = False
            for _attempt in range(4):
                with self._lock:
                    self._serial.flush()
                self.query_parameter(mid, QUERY_POSITION)
                deadline = time.monotonic() + 0.08
                while time.monotonic() < deadline:
                    with self._lock:
                        msg = self._serial.read_msg()
                    if msg:
                        self._handle_msg(msg)
                        if self.is_connected[mid - 1]:
                            ok = True
                            break
                    time.sleep(0.001)
                if ok:
                    break
                time.sleep(0.01)
            if ok:
                print(f"[Incos] motor {mid} online")
            else:
                print(f"[Incos] motor {mid} init failed (no query reply)")

        self._running = True
        self._thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._thread.start()
        self.start_keepalive()
        return any(self.is_connected[mid - 1] for mid in self._active_ids)

    def begin_shared(self, serial_obj, lock, motor_ids=(2, 3, 6, 7),
                     register_handler=None):
        """Attach to an already-open USB-CAN bus owned by another driver."""
        self._active_ids = list(motor_ids)
        self._serial = serial_obj
        self._lock = lock
        self._owns_serial = False
        if register_handler is not None:
            register_handler(lambda can_id, dlc, data: self._handle_msg((can_id, dlc, data)))

        time.sleep(0.05)
        for mid in self._active_ids:
            ok = False
            for _attempt in range(4):
                self.query_parameter(mid, QUERY_POSITION)
                deadline = time.monotonic() + 0.08
                while time.monotonic() < deadline:
                    if self.is_connected[mid - 1]:
                        ok = True
                        break
                    time.sleep(0.001)
                if ok:
                    break
                time.sleep(0.01)
            if ok:
                print(f"[Incos] motor {mid} online (shared CAN-A)")
            else:
                print(f"[Incos] motor {mid} init failed on shared CAN-A")

        return any(self.is_connected[mid - 1] for mid in self._active_ids)

    # ── 保活：填补无下发窗口，防 MIT 看门狗掉力 ──────────────────────
    def start_keepalive(self):
        if self._keepalive_thread is not None and self._keepalive_thread.is_alive():
            return
        self._keepalive_enabled = True
        self._keepalive_thread = threading.Thread(
            target=self._keepalive_loop, name="incos-keepalive", daemon=True)
        self._keepalive_thread.start()

    def stop_keepalive(self, timeout_s=1.0):
        self._keepalive_enabled = False
        if self._keepalive_thread is not None:
            self._keepalive_thread.join(timeout_s)
            self._keepalive_thread = None

    def _keepalive_loop(self):
        check_dt = 1.0 / max(1, self.KEEPALIVE_HZ)
        idle_gap = 2.0 * check_dt  # 主环发送快于此则保活自动静默
        while self._running and self._keepalive_enabled:
            if time.monotonic() - self._last_tx_monotonic >= idle_gap:
                with self._cmd_lock:
                    cmds = list(self._last_cmd.items())
                if cmds:
                    frames = [(self._encode_mit(kp, kd, q, dq, tau), 8, mid)
                              for mid, (q, dq, kp, kd, tau) in cmds]
                    with self._lock:
                        self._serial.send_bulk(frames)
                    self._last_tx_monotonic = time.monotonic()
                    for mid, _ in cmds:
                        if 1 <= mid <= MAX_ID:
                            self.tx_count[mid - 1] += 1
            time.sleep(check_dt)

    def _record_cmd(self, motor_id, pos_rad, vel_rad, kp, kd, torque_ff):
        with self._cmd_lock:
            self._last_cmd[motor_id] = (pos_rad, vel_rad, kp, kd, torque_ff)

    def end(self):
        self._running = False
        self.stop_keepalive()
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None
        # Hold current position with zero gains before closing the bus.
        for mid in self._active_ids:
            if self.is_connected[mid - 1]:
                self.mit_control(mid, self.get_position(mid), 0.0, 0.0, 0.0, 0.0)
                self.is_enabled[mid - 1] = False
                time.sleep(0.002)
        if self._owns_serial:
            self._serial.end()

    def query_parameter(self, motor_id, query_code):
        if not 1 <= motor_id <= MAX_ID or not 1 <= query_code <= 39:
            return False
        return self._send(bytes([0xE1, query_code]), 2, motor_id)

    def mit_control(self, motor_id, pos_rad, vel_rad=0.0,
                    kp=10.0, kd=0.5, torque_ff=0.0):
        data = self._encode_mit(kp, kd, pos_rad, vel_rad, torque_ff)
        ok = self._send(data, 8, motor_id)
        if ok and 1 <= motor_id <= MAX_ID:
            self.is_enabled[motor_id - 1] = kp > 0.0 or kd > 0.0
            self._record_cmd(motor_id, pos_rad, vel_rad, kp, kd, torque_ff)
        return ok

    def mit_controls(self, motor_ids, positions, velocities, kps, kds, torques):
        frames = []
        for mid, q, dq, kp, kd, tau in zip(motor_ids, positions, velocities,
                                           kps, kds, torques):
            frames.append((self._encode_mit(kp, kd, q, dq, tau), 8, mid))
        with self._lock:
            ok = self._serial.send_bulk(frames)
        if ok:
            self._last_tx_monotonic = time.monotonic()
            for mid, q, dq, kp, kd, tau in zip(motor_ids, positions, velocities,
                                               kps, kds, torques):
                if 1 <= mid <= MAX_ID:
                    self.is_enabled[mid - 1] = kp > 0.0 or kd > 0.0
                    self.tx_count[mid - 1] += 1
                    self._record_cmd(mid, q, dq, kp, kd, tau)
        return ok

    def get_position(self, motor_id):
        return self.position[motor_id - 1]

    def get_velocity(self, motor_id):
        return self.velocity[motor_id - 1]

    def get_torque(self, motor_id):
        return self.torque[motor_id - 1]

    def get_temperature(self, motor_id):
        return self.motor_temperature[motor_id - 1]

    def disable(self, motor_id):
        """Release the motor by holding current position with zero gains."""
        if not 1 <= motor_id <= MAX_ID:
            return False
        pos = self.get_position(motor_id)
        ok = self.mit_control(motor_id, pos, 0.0, 0.0, 0.0, 0.0)
        if ok:
            self.is_enabled[motor_id - 1] = False
            # mit_control 已把零增益保位记入 _last_cmd，保活会持续保持"已失能"状态。
        return ok

    def _send(self, data, dlc, can_id):
        with self._lock:
            ok = self._serial.send_msg(data, dlc, can_id)
        if ok:
            self._last_tx_monotonic = time.monotonic()
            if 1 <= can_id <= MAX_ID:
                self.tx_count[can_id - 1] += 1
        return ok

    def _encode_mit(self, kp, kd, position_rad, speed_rad_s, torque_nm):
        kp_raw = _float_to_uint(kp, KP_MIN, KP_MAX, 12)
        kd_raw = _float_to_uint(kd, KD_MIN, KD_MAX, 9)
        pos_raw = _float_to_uint(position_rad, POS_MIN, POS_MAX, 16)
        spd_raw = _float_to_uint(speed_rad_s, SPEED_MIN, SPEED_MAX, 12)
        trq_raw = _float_to_uint(torque_nm, TORQUE_MIN, TORQUE_MAX, 12)
        return bytes([
            (kp_raw >> 7) & 0xFF,
            ((kp_raw << 1) | (kd_raw >> 8)) & 0xFF,
            kd_raw & 0xFF,
            (pos_raw >> 8) & 0xFF,
            pos_raw & 0xFF,
            (spd_raw >> 4) & 0xFF,
            ((spd_raw << 4) | (trq_raw >> 8)) & 0xFF,
            trq_raw & 0xFF,
        ])

    def _recv_loop(self):
        while self._running:
            with self._lock:
                msg = self._serial.read_msg()
            if msg:
                self._handle_msg(msg)
            else:
                time.sleep(0.001)

    def _handle_msg(self, msg):
        can_id, dlc, data = msg
        if can_id & CAN_EFF_FLAG:
            return
        mid = can_id & 0x7FF
        if not 1 <= mid <= MAX_ID or dlc < 1:
            return
        self._parse_feedback(mid, data[:dlc])

    def _parse_feedback(self, motor_id, data):
        idx = motor_id - 1
        self.is_connected[idx] = True
        self.rx_count[idx] += 1
        self.fault[idx] = data[0] & 0x1F
        frame_type = data[0] >> 5

        if frame_type == 1 and len(data) == 8:
            pos_raw = (data[1] << 8) | data[2]
            spd_raw = (data[3] << 4) | (data[4] >> 4)
            cur_raw = ((data[4] & 0x0F) << 8) | data[5]
            self.position[idx] = _uint_to_float(pos_raw, POS_MIN, POS_MAX, 16)
            self.velocity[idx] = _uint_to_float(spd_raw, SPEED_MIN, SPEED_MAX, 12)
            self.current[idx] = _uint_to_float(cur_raw, -CURRENT_MAX_A,
                                               CURRENT_MAX_A, 12)
            self.torque[idx] = self.current[idx] * TORQUE_COEFF
            self.motor_temperature[idx] = (float(data[6]) - 50.0) / 2.0
            self.mos_temperature[idx] = (float(data[7]) - 50.0) / 2.0
        elif frame_type == 5 and len(data) >= 6 and data[1] == QUERY_POSITION:
            # Query reply: type/query/float32 degrees.
            self.position[idx] = math.radians(_get_float32_be(data[2:6]))
        elif frame_type == 5 and len(data) >= 6 and data[1] == QUERY_SPEED:
            rpm = _get_float32_be(data[2:6])
            self.velocity[idx] = rpm * (2.0 * math.pi / 60.0)
        elif frame_type == 5 and len(data) >= 6 and data[1] == QUERY_CURRENT:
            self.current[idx] = _get_float32_be(data[2:6])
            self.torque[idx] = self.current[idx] * TORQUE_COEFF
