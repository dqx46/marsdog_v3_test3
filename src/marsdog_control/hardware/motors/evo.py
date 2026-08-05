"""泉智博 MotorEvo driver — PTM control mode (Python port).

All angle inputs/outputs in **radians** (no deg conversion).
"""

import time
import threading

from marsdog_control.hardware.motors.can_bus import CanBus, CAN_EFF_FLAG
from marsdog_control.hardware.motors.can_serial import CanSerial

# State commands (Byte7)
CMD_MOTOR_STATE = 0xFC   # enter motor state (enable)
CMD_REST_STATE = 0xFD    # enter rest state (disable / clear fault)
CMD_SET_ZERO = 0xFE      # set current position as zero

# PTM ranges
THETA_MIN, THETA_MAX = -12.5, 12.5
VEL_MIN, VEL_MAX = -10.0, 10.0
KP_MIN, KP_MAX = 0.0, 250.0
KD_MIN, KD_MAX = 0.0, 50.0
TRQ_MIN, TRQ_MAX = -50.0, 50.0

# Feedback status bytes
STATUS_REST = 0x00
STATUS_PTM = 0x02

# Known MotorEvo IDs
MEVO_KNOWN_IDS = [9, 12, 18, 19, 20]  # 后腿 hip + 颈腰

MAX_ID = 24
LOSS_MAX = 10
POLL_MS = 5    # 200Hz 控制下轮询间隔 5ms


def _clamp(x, lo, hi):
    return max(lo, min(hi, x))


def _float_to_uint(x, x_min, x_max, bits):
    x = _clamp(x, x_min, x_max)
    maxv = (1 << bits) - 1
    return int((x - x_min) / (x_max - x_min) * maxv + 0.5)


def _uint_to_float(v, x_min, x_max, bits):
    maxv = (1 << bits) - 1
    return v / maxv * (x_max - x_min) + x_min


class MotorEvo:

    def __init__(self):
        self._can = CanBus()
        self._serial = None        # CanSerial 实例（USB-CAN 模式时使用）
        self._use_serial = False
        self._lock = threading.Lock()
        self._running = False
        self._thread = None
        self._want_motor = [False] * MAX_ID
        # state arrays, indexed by (motor_id - 1)
        self.position = [0.0] * MAX_ID      # rad
        self.velocity = [0.0] * MAX_ID      # rad/s
        self.torque = [0.0] * MAX_ID        # Nm
        self.temperature = [0.0] * MAX_ID   # °C
        self.status = [0] * MAX_ID
        self.fault = [0] * MAX_ID
        self.is_connected = [False] * MAX_ID
        self._loss_count = [0] * MAX_ID
        self._active_ids = []

    # ── init ─────────────────────────────────────────────────────

    def init(self, interface="can0"):
        if not self._can.begin(interface):
            return False

        probe = bytes([0xFF] * 7 + [CMD_REST_STATE])
        self._can.flush()
        for mid in MEVO_KNOWN_IDS:
            self._can.send_msg(probe, 8, mid)
            time.sleep(0.0005)
        time.sleep(0.005)

        # read responses (30ms window)
        t0 = time.monotonic()
        while time.monotonic() - t0 < 0.030:
            result = self._can.read_msg()
            if result is None:
                break
            can_id, dlc, data = result
            if can_id & CAN_EFF_FLAG:
                continue  # skip RS05 extended frames
            if len(data) >= 8 and data[0] > STATUS_PTM + 3:
                continue  # skip ECHO
            mid = can_id & 0x7FF
            if 1 <= mid <= MAX_ID and mid not in self._active_ids:
                self._active_ids.append(mid)
                self._parse_feedback(mid, data)
                print(f"[MotorEvo] motor {mid} online")

        if not self._active_ids:
            print("[MotorEvo] no motor responded")
            return False

        self._running = True
        self._thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._thread.start()
        return True

    def _send_raw(self, data, dlc, can_id):
        """Send a standard CAN frame via CanSerial or CanBus."""
        if self._use_serial:
            return self._serial.send_msg(data, dlc, can_id)
        return self._can.send_msg(data, dlc, can_id)

    def _recv_raw(self):
        """Receive one frame (blocking with timeout) from CanSerial or CanBus."""
        if self._use_serial:
            return self._serial.read_msg()
        return self._can.read_msg()

    def _flush_raw(self):
        if self._use_serial:
            self._serial.flush()
        else:
            self._can.flush()

    def init_serial(self, device="/dev/ttyUSB1", baud=921600):
        """Open USB-CAN serial as CAN0 and enable EVO motors via CanSerial."""
        self._serial = CanSerial()
        if not self._serial.begin(device, baud):
            self._serial = None
            return False
        self._use_serial = True

        probe = bytes([0xFF] * 7 + [CMD_REST_STATE])
        with self._lock:
            self._flush_raw()
            for mid in MEVO_KNOWN_IDS:
                self._send_raw(probe, 8, mid)
                time.sleep(0.0005)
        time.sleep(0.005)

        t0 = time.monotonic()
        while time.monotonic() - t0 < 0.100:
            with self._lock:
                result = self._recv_raw()
            if result is None:
                time.sleep(0.002)
                continue
            can_id, dlc, data = result
            if can_id & 0x80000000:
                continue  # skip LZ extended frames
            if len(data) >= 8 and data[0] > STATUS_PTM + 3:
                continue  # echo
            mid = can_id & 0x7FF
            if 1 <= mid <= MAX_ID and mid not in self._active_ids:
                self._active_ids.append(mid)
                self._parse_feedback(mid, data)
                print(f"[MotorEvo] motor {mid} online (serial)")
            if len(self._active_ids) == len(MEVO_KNOWN_IDS):
                break  # 全部找到提前退出

        if not self._active_ids:
            print("[MotorEvo] no motor responded on serial")
            return False

        self._running = True
        self._thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._thread.start()
        return True

    def stop_all(self):
        for mid in MEVO_KNOWN_IDS:
            self.enter_rest_state(mid)

    def end(self):
        self._running = False
        self.stop_all()
        if self._thread:
            self._thread.join(timeout=2)
        if self._use_serial and self._serial:
            self._serial.end()
        else:
            self._can.end()

    # ── feedback parsing ─────────────────────────────────────────

    def _parse_feedback(self, motor_id, data):
        """Parse 8-byte feedback: status(1) + pos(2) + vel_hi(1) +
        vel_lo_kp_hi(1) + ... → see MotorEvo.cpp"""
        if len(data) < 8:
            return
        idx = motor_id - 1
        self.status[idx] = data[0]
        pos_raw = (data[1] << 8) | data[2]
        vel_raw = (data[3] << 4) | (data[4] >> 4)
        trq_raw = ((data[4] & 0x0F) << 8) | data[5]
        self.position[idx] = _uint_to_float(pos_raw, THETA_MIN, THETA_MAX, 16)
        self.velocity[idx] = _uint_to_float(vel_raw, VEL_MIN, VEL_MAX, 12)
        self.torque[idx] = _uint_to_float(trq_raw, TRQ_MIN, TRQ_MAX, 12)
        self.fault[idx] = data[6]
        self.temperature[idx] = float(data[7])
        self.is_connected[idx] = True
        self._loss_count[idx] = 0

    # ── state commands ───────────────────────────────────────────

    def enter_motor_state(self, motor_id):
        data = bytes([0xFF] * 7 + [CMD_MOTOR_STATE])
        with self._lock:
            self._want_motor[motor_id - 1] = True
            return self._send_raw(data, 8, motor_id)

    def enter_rest_state(self, motor_id):
        data = bytes([0xFF] * 7 + [CMD_REST_STATE])
        with self._lock:
            self._want_motor[motor_id - 1] = False
            return self._send_raw(data, 8, motor_id)

    def set_zero_position(self, motor_id):
        data = bytes([0xFF] * 7 + [CMD_SET_ZERO])
        with self._lock:
            return self._send_raw(data, 8, motor_id)

    # ── PTM control (rad input, no deg conversion) ───────────────

    def ptm_control(self, motor_id, theta_rad, v_ref_rad=0.0,
                    kp=20.0, kd=3.0, t_ref=0.0):
        """Send PTM 5-parameter control frame. All angles in radians."""
        p = _float_to_uint(theta_rad, THETA_MIN, THETA_MAX, 16)
        v = _float_to_uint(v_ref_rad, VEL_MIN, VEL_MAX, 12)
        kp_i = _float_to_uint(kp, KP_MIN, KP_MAX, 12)
        kd_i = _float_to_uint(kd, KD_MIN, KD_MAX, 12)
        t = _float_to_uint(t_ref, TRQ_MIN, TRQ_MAX, 12)

        data = bytes([
            (p >> 8) & 0xFF,
            p & 0xFF,
            (v >> 4) & 0xFF,
            ((v & 0xF) << 4) | ((kp_i >> 8) & 0xF),
            kp_i & 0xFF,
            (kd_i >> 4) & 0xFF,
            ((kd_i & 0xF) << 4) | ((t >> 8) & 0xF),
            t & 0xFF,
        ])

        with self._lock:
            return self._send_raw(data, 8, motor_id)

    def ptm_controls(self, motor_ids, thetas, v_refs=None,
                     kps=None, kds=None, t_refs=None):
        """Batch PTM control for multiple motors (send_bulk for speed)."""
        n = len(motor_ids)
        if v_refs is None:
            v_refs = [0.0] * n
        if kps is None:
            kps = [20.0] * n
        if kds is None:
            kds = [3.0] * n
        if t_refs is None:
            t_refs = [0.0] * n

        frames = []
        for i in range(n):
            mid = motor_ids[i]
            p = _float_to_uint(thetas[i], THETA_MIN, THETA_MAX, 16)
            v = _float_to_uint(v_refs[i], VEL_MIN, VEL_MAX, 12)
            kp_i = _float_to_uint(kps[i], KP_MIN, KP_MAX, 12)
            kd_i = _float_to_uint(kds[i], KD_MIN, KD_MAX, 12)
            t = _float_to_uint(t_refs[i], TRQ_MIN, TRQ_MAX, 12)
            data = bytes([
                (p >> 8) & 0xFF, p & 0xFF,
                (v >> 4) & 0xFF,
                ((v & 0xF) << 4) | ((kp_i >> 8) & 0xF),
                kp_i & 0xFF,
                (kd_i >> 4) & 0xFF,
                ((kd_i & 0xF) << 4) | ((t >> 8) & 0xF),
                t & 0xFF,
            ])
            frames.append((data, 8, mid))
        with self._lock:
            if self._use_serial and self._serial:
                self._serial.send_bulk(frames)
            elif self._can:
                self._can.send_bulk(frames)

    # ── background receive loop ──────────────────────────────────

    def _recv_loop(self):
        """200Hz 优化版：批量发送所有电机状态查询，统一收包，减少串口往返次数。

        Keep-alive (CMD_MOTOR_STATE / REST) is *not* sent every poll: flooding
        5 state frames at 200Hz on the same USB-CAN as PTM commands was
        observed to make ID9/12 tracking intermittently collapse (slope→0,
        RMS≈7°) even with correct kp/kd + velocity FF — both hips fail in
        lockstep, pointing at the bus/driver rather than mechanics. Send
        keep-alive ~10Hz; other polls only drain RX.
        """
        keepalive_every = max(1, int(round(50.0 / POLL_MS)))  # ~10 Hz
        poll_i = 0
        while self._running:
            t0 = time.monotonic()
            poll_i += 1
            do_keepalive = (poll_i % keepalive_every) == 1

            with self._lock:
                # 1. Occasional motor-state keep-alive (not every tick).
                if do_keepalive:
                    if self._use_serial and self._serial:
                        frames = []
                        for mid in MEVO_KNOWN_IDS:
                            idx = mid - 1
                            cmd = CMD_MOTOR_STATE if self._want_motor[idx] else CMD_REST_STATE
                            frames.append((bytes([0xFF] * 7 + [cmd]), 8, mid))
                        self._serial.send_bulk(frames)
                    else:
                        for mid in MEVO_KNOWN_IDS:
                            idx = mid - 1
                            cmd = CMD_MOTOR_STATE if self._want_motor[idx] else CMD_REST_STATE
                            data = bytes([0xFF] * 7 + [cmd])
                            self._send_raw(data, 8, mid)

            # 2. 等待并收取应答（有 keep-alive 时稍等；否则只排空已有缓冲）
            if do_keepalive:
                time.sleep(0.0015)

            found = set()
            deadline = time.monotonic() + (0.002 if do_keepalive else 0.001)
            while time.monotonic() < deadline:
                with self._lock:
                    result = self._recv_raw()
                if result is None:
                    break
                can_id, dlc, rdata = result
                if can_id & CAN_EFF_FLAG:
                    continue
                if len(rdata) >= 8 and rdata[0] > 0x05:
                    continue
                mid = can_id & 0x7FF
                if mid in MEVO_KNOWN_IDS and mid not in found:
                    self._parse_feedback(mid, rdata)
                    found.add(mid)
                if len(found) == len(MEVO_KNOWN_IDS):
                    break

            # 更新 loss_count
            for mid in MEVO_KNOWN_IDS:
                idx = mid - 1
                if mid in found:
                    self._loss_count[idx] = 0
                    self.is_connected[idx] = True
                else:
                    self._loss_count[idx] += 1
                    if self._loss_count[idx] >= LOSS_MAX:
                        # 保活收窗只有 1–2ms，5 个电机挤同一 USB-CAN 时常收不齐。
                        # init 已确认在线的电机不要因此摘掉（static_test 逐个探针仍全绿，
                        # 但 board.online_ids 会变成 20/21，漏掉 waist_pitch=20 等）。
                        if mid not in self._active_ids:
                            self.is_connected[idx] = False

            # 3. 精准等待剩余时间以维持 POLL_MS 周期
            elapsed = time.monotonic() - t0
            remain = POLL_MS / 1000.0 - elapsed
            if remain > 0.0001:
                time.sleep(remain)

    def get_position(self, motor_id):
        """Return current position in radians."""
        return self.position[motor_id - 1]

    def get_velocity(self, motor_id):
        return self.velocity[motor_id - 1]

    def get_torque(self, motor_id):
        return self.torque[motor_id - 1]

    def get_temperature(self, motor_id):
        return self.temperature[motor_id - 1]
