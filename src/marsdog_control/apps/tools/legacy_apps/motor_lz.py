"""灵足 RS05 motor driver — MIT control mode (Python port).

All angle inputs/outputs in **radians** (no deg conversion).

Architecture mirrors the proven marsdog_firmware motor_driver.py:
- TX via raw socket (fire-and-forget, large SO_SNDBUF)
- RX via separate socket (blocking recv in background thread)
- No locks needed for CAN (TX/RX are independent sockets)
- Serial uses a lightweight lock (send vs recv only)
"""

import math
import struct
import time
import threading

from marsdog_control.hardware.motors.can_bus import CanBus, CAN_EFF_FLAG
from marsdog_control.hardware.motors.can_serial import CanSerial

# RS05 communication types (29-bit extended frame protocol)
CMD_MIT_CTRL = 0x01
CMD_FEEDBACK = 0x02
CMD_ENABLE = 0x03
CMD_DISABLE = 0x04
CMD_SET_ORIGIN = 0x06
CMD_READ_PARAM = 0x11
CMD_WRITE_PARAM = 0x12
CMD_SAVE_DATA = 0x16
CMD_AUTO_REPORT = 0x18

IDX_RUN_MODE = 0x7005
IDX_MECH_POS = 0x7019
MODE_MIT = 0
MASTER_ID = 0xFD

P_MIN, P_MAX = -12.57, 12.57
MIT_KP_MAX = 500.0
MIT_KD_MAX = 5.0

MOTOR_RANGES = {
    'EL05': (-50.0, 50.0, -6.0, 6.0),
    'RS02': (-44.0, 44.0, -17.0, 17.0),
    'RS00': (-33.0, 33.0, -14.0, 14.0),
}
DEFAULT_RANGE = (-50.0, 50.0, -6.0, 6.0)

RS05_CAN_IDS = [1, 2, 3, 4, 5, 6, 15]
RS05_SERIAL_IDS = [8, 9, 11, 12, 13, 14, 19]
RS05_ALL_IDS = RS05_CAN_IDS + RS05_SERIAL_IDS

MAX_ID = 20


def _clamp(x, lo, hi):
    return max(lo, min(hi, x))


def _float_to_uint16(x, x_min, x_max):
    x = _clamp(x, x_min, x_max)
    return int((x - x_min) / (x_max - x_min) * 65535.0 + 0.5)


def _uint16_to_float(v, x_min, x_max):
    return v / 65535.0 * (x_max - x_min) + x_min


class MotorLz:

    def __init__(self):
        self._can = CanBus()
        self._serial = CanSerial()
        self._serial_set = set()
        self._serial_lock = threading.Lock()
        self._running = False
        self._thread = None
        self.position = [0.0] * MAX_ID
        self.velocity = [0.0] * MAX_ID
        self.torque = [0.0] * MAX_ID
        self.temperature = [0.0] * MAX_ID
        self.fault = [0] * MAX_ID
        self.mode = [0] * MAX_ID
        self.is_enabled = [False] * MAX_ID
        self.is_connected = [False] * MAX_ID
        self._pos_offset = [0.0] * MAX_ID
        self._v_range = [DEFAULT_RANGE[:2]] * MAX_ID
        self._t_range = [DEFAULT_RANGE[2:]] * MAX_ID

    def set_motor_model(self, motor_id, model):
        """Set per-motor velocity/torque range based on motor model string."""
        r = MOTOR_RANGES.get(model, DEFAULT_RANGE)
        self._v_range[motor_id - 1] = (r[0], r[1])
        self._t_range[motor_id - 1] = (r[2], r[3])

    # ── bus routing ──────────────────────────────────────────────

    def _build_ext_id(self, comm_type, data2, target_id):
        return ((comm_type & 0x1F) << 24) | ((data2 & 0xFFFF) << 8) | (target_id & 0xFF)

    def _send(self, motor_id, data, dlc, frame_id):
        if motor_id in self._serial_set:
            return self._serial.send_msg(data, dlc, frame_id)
        return self._can.send_msg(data, dlc, frame_id)

    def _send_locked(self, motor_id, data, dlc, frame_id):
        """Send with serial lock if needed (for serial motors during runtime)."""
        if motor_id in self._serial_set:
            with self._serial_lock:
                return self._serial.send_msg(data, dlc, frame_id)
        return self._can.send_msg(data, dlc, frame_id)

    def _read(self, motor_id):
        if motor_id in self._serial_set:
            return self._serial.read_msg()
        return self._can.read_msg()

    def _flush(self, motor_id):
        if motor_id in self._serial_set:
            self._serial.flush()
        else:
            self._can.flush()

    # ── multi-turn compensation ─────────────────────────────────

    def _calc_pos_offset(self, motor_id):
        idx = motor_id - 1
        raw = self.position[idx]
        TWO_PI = 2.0 * math.pi
        n = round(raw / TWO_PI)
        self._pos_offset[idx] = n * TWO_PI
        if n != 0:
            corrected = raw - self._pos_offset[idx]
            print(f"[MotorLz] Motor {motor_id} multi-turn fix: "
                  f"{math.degrees(raw):+.1f} -> {math.degrees(corrected):+.1f} "
                  f"(offset {n}x360)")

    # ── init ─────────────────────────────────────────────────────

    def _start_recv_thread(self):
        if not self._running:
            self._running = True
            self._thread = threading.Thread(target=self._recv_loop, daemon=True)
            self._thread.start()

    def _load_models(self):
        """Auto-load motor model ranges from joint_config if available."""
        try:
            from marsdog_control.config.joints import JOINT_BY_ID
            for mid, jd in JOINT_BY_ID.items():
                if jd.model:
                    self.set_motor_model(mid, jd.model)
        except ImportError:
            pass

    def _init_motor(self, mid, label=""):
        """Init single motor: disable (clear fault) → set MIT → enable."""
        idx = mid - 1
        self.is_connected[idx] = True
        self._disable(mid)
        time.sleep(0.02)
        for attempt in range(5):
            if self._set_run_mode(mid, MODE_MIT):
                time.sleep(0.005)
                if self._enable(mid):
                    self._calc_pos_offset(mid)
                    self._set_auto_report(mid, True)
                    return True
            time.sleep(0.02)
        print(f"[MotorLz] {label}motor {mid} init failed (5 attempts)")
        self.is_connected[idx] = False
        return False

    def init(self, interface="can1"):
        self._load_models()
        if not self._can.begin(interface):
            return False
        for mid in RS05_CAN_IDS:
            self._init_motor(mid, "CAN ")
        self._start_recv_thread()
        return True

    def init_serial(self, device="/dev/ttyUSB0", baud=921600):
        self._serial_set = set(RS05_SERIAL_IDS)
        if not self._serial.begin(device, baud):
            self._serial_set.clear()
            return False
        for mid in RS05_SERIAL_IDS:
            self._init_motor(mid, "serial ")
        return True

    def start_recv(self):
        self._start_recv_thread()

    def stop_all(self):
        for mid in RS05_ALL_IDS:
            self._disable(mid)

    def end(self):
        self._running = False
        self.stop_all()
        if self._thread:
            self._thread.join(timeout=2)
        self._can.end()
        self._serial.end()

    # ── init-time send-and-recv (blocking, used only during init) ─

    def _send_and_recv(self, comm_type, motor_id, data8):
        frame_id = self._build_ext_id(comm_type, MASTER_ID, motor_id)
        is_ser = motor_id in self._serial_set
        self._flush(motor_id)
        if is_ser:
            ok = self._send(motor_id, data8, 8, frame_id)
        else:
            ok = self._can.send_msg_rx(data8, 8, frame_id)
        if not ok:
            return None
        if is_ser:
            deadline = time.monotonic() + 0.020
            while time.monotonic() < deadline:
                result = self._read(motor_id)
                if result is None:
                    continue
                can_id, dlc, rdata = result
                if ((can_id >> 8) & 0xFF) == motor_id:
                    return (can_id, dlc, rdata)
        else:
            for _ in range(10):
                result = self._read(motor_id)
                if result is None:
                    continue
                can_id, dlc, rdata = result
                if ((can_id >> 8) & 0xFF) == motor_id:
                    return (can_id, dlc, rdata)
        return None

    # ── feedback parsing ─────────────────────────────────────────

    def _parse_feedback(self, motor_id, can_id, data):
        if len(data) < 8:
            return
        idx = motor_id - 1
        pos_raw = (data[0] << 8) | data[1]
        vel_raw = (data[2] << 8) | data[3]
        trq_raw = (data[4] << 8) | data[5]
        tmp_raw = (data[6] << 8) | data[7]
        self.position[idx] = _uint16_to_float(pos_raw, P_MIN, P_MAX)
        v_lo, v_hi = self._v_range[idx]
        t_lo, t_hi = self._t_range[idx]
        self.velocity[idx] = _uint16_to_float(vel_raw, v_lo, v_hi)
        self.torque[idx] = _uint16_to_float(trq_raw, t_lo, t_hi)
        self.temperature[idx] = tmp_raw / 10.0
        self.fault[idx] = (can_id >> 16) & 0x3F
        self.mode[idx] = (can_id >> 22) & 0x03
        self.is_enabled[idx] = (self.mode[idx] == 2)
        self.is_connected[idx] = True

    # ── basic commands ───────────────────────────────────────────

    def set_origin(self, motor_id):
        buf = bytes([0x01] + [0x00] * 7)
        return self._send_and_recv(CMD_SET_ORIGIN, motor_id, buf) is not None

    def save_params(self, motor_id):
        buf = bytes([0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08])
        return self._send_and_recv(CMD_SAVE_DATA, motor_id, buf) is not None

    def enable(self, motor_id):
        return self._enable(motor_id)

    def disable(self, motor_id):
        return self._disable(motor_id)

    def _enable(self, motor_id):
        result = self._send_and_recv(CMD_ENABLE, motor_id, b'\x00' * 8)
        if result is None:
            return False
        can_id, dlc, data = result
        self._parse_feedback(motor_id, can_id, data)
        return self.is_enabled[motor_id - 1]

    def _disable(self, motor_id):
        ext_id = self._build_ext_id(CMD_DISABLE, MASTER_ID, motor_id)
        return self._send(motor_id, b'\x00' * 8, 8, ext_id)

    def re_enable(self, motor_id):
        """Full re-enable sequence: disable (clear fault) → set MIT → enable."""
        self._disable(motor_id)
        time.sleep(0.05)
        for _ in range(5):
            if self._set_run_mode(motor_id, MODE_MIT):
                time.sleep(0.005)
                if self._enable(motor_id):
                    self._calc_pos_offset(motor_id)
                    return True
            time.sleep(0.02)
        return False

    def _set_run_mode(self, motor_id, mode):
        return self._write_param_u8(motor_id, IDX_RUN_MODE, mode)

    def _write_param(self, motor_id, index, value_float):
        buf = bytearray(8)
        struct.pack_into("<H", buf, 0, index)
        struct.pack_into("<f", buf, 4, value_float)
        return self._send_and_recv(CMD_WRITE_PARAM, motor_id, bytes(buf)) is not None

    def _write_param_u8(self, motor_id, index, value):
        buf = bytearray(8)
        struct.pack_into("<H", buf, 0, index)
        buf[4] = value
        return self._send_and_recv(CMD_WRITE_PARAM, motor_id, bytes(buf)) is not None

    def _read_param(self, motor_id, index):
        buf = bytearray(8)
        struct.pack_into("<H", buf, 0, index)
        result = self._send_and_recv(CMD_READ_PARAM, motor_id, bytes(buf))
        if result is None:
            return 0.0
        _, _, rdata = result
        if len(rdata) >= 8:
            return struct.unpack_from("<f", rdata, 4)[0]
        return 0.0

    def _set_auto_report(self, motor_id, enable_flag=True):
        """Comm type 0x18: enable/disable periodic feedback broadcast."""
        ext_id = self._build_ext_id(CMD_AUTO_REPORT, MASTER_ID, motor_id)
        data = bytes([0x01, 0x02, 0x03, 0x04, 0x05, 0x06,
                      0x01 if enable_flag else 0x00, 0x00])
        return self._send(motor_id, data, 8, ext_id)

    # ── MIT control ──────────────────────────────────────────────

    def mit_control(self, motor_id, pos_rad, vel_rad=0.0,
                    kp=15.0, kd=1.5, torque_ff=0.0):
        idx = motor_id - 1
        pos_raw = pos_rad + self._pos_offset[idx]
        v_lo, v_hi = self._v_range[idx]
        t_lo, t_hi = self._t_range[idx]
        pos_u = _float_to_uint16(pos_raw, P_MIN, P_MAX)
        vel_u = _float_to_uint16(vel_rad, v_lo, v_hi)
        kp_u = _float_to_uint16(kp, 0.0, MIT_KP_MAX)
        kd_u = _float_to_uint16(kd, 0.0, MIT_KD_MAX)
        trq_u = _float_to_uint16(torque_ff, t_lo, t_hi)

        ext_id = self._build_ext_id(CMD_MIT_CTRL, trq_u, motor_id)
        data = struct.pack(">HHHH", pos_u, vel_u, kp_u, kd_u)

        return self._send_locked(motor_id, data, 8, ext_id)

    def mit_controls(self, motor_ids, positions, velocities=None,
                     kps=None, kds=None, torques=None):
        n = len(motor_ids)
        if velocities is None:
            velocities = [0.0] * n
        if kps is None:
            kps = [15.0] * n
        if kds is None:
            kds = [1.5] * n
        if torques is None:
            torques = [0.0] * n

        for i in range(n):
            mid = motor_ids[i]
            idx = mid - 1
            pos_raw = positions[i] + self._pos_offset[idx]
            v_lo, v_hi = self._v_range[idx]
            t_lo, t_hi = self._t_range[idx]
            pos_u = _float_to_uint16(pos_raw, P_MIN, P_MAX)
            vel_u = _float_to_uint16(velocities[i], v_lo, v_hi)
            kp_u = _float_to_uint16(kps[i], 0.0, MIT_KP_MAX)
            kd_u = _float_to_uint16(kds[i], 0.0, MIT_KD_MAX)
            trq_u = _float_to_uint16(torques[i], t_lo, t_hi)
            ext_id = self._build_ext_id(CMD_MIT_CTRL, trq_u, mid)
            data = struct.pack(">HHHH", pos_u, vel_u, kp_u, kd_u)
            self._send_locked(mid, data, 8, ext_id)

    # ── background receive loop ──────────────────────────────────

    def _recv_loop(self):
        """Background thread: read type-2 feedback from CAN and serial.

        Mirrors the reference motor_driver.py pattern:
        block once on recv(5ms), then drain all pending non-blocking.
        """
        while self._running:
            # ── CAN: block up to 5ms for first frame, then drain ──
            msg = self._can.read_msg()
            if msg is not None:
                self._dispatch_can(msg)
                while True:
                    msg = self._can.read_msg_nonblock()
                    if msg is None:
                        break
                    self._dispatch_can(msg)

            # ── Serial: drain available frames ──
            for _ in range(len(RS05_SERIAL_IDS)):
                with self._serial_lock:
                    result = self._serial.read_msg()
                if result is None:
                    break
                can_id, dlc, data = result
                if not (can_id & CAN_EFF_FLAG):
                    continue
                mid = (can_id >> 8) & 0xFF
                typ = (can_id >> 24) & 0x1F
                if (typ == CMD_FEEDBACK or typ == CMD_AUTO_REPORT) \
                        and 1 <= mid <= MAX_ID \
                        and mid in self._serial_set:
                    self._parse_feedback(mid, can_id, data)

    def _dispatch_can(self, msg):
        """Process a single CAN frame: both type-2 (MIT response) and type-0x18 (auto_report)."""
        can_id, dlc, data = msg
        if not (can_id & CAN_EFF_FLAG):
            return
        mid = (can_id >> 8) & 0xFF
        typ = (can_id >> 24) & 0x1F
        if (typ == CMD_FEEDBACK or typ == CMD_AUTO_REPORT) \
                and 1 <= mid <= MAX_ID \
                and mid not in self._serial_set:
            self._parse_feedback(mid, can_id, data)

    # ── position helpers ─────────────────────────────────────────

    def get_position(self, motor_id):
        idx = motor_id - 1
        return self.position[idx] - self._pos_offset[idx]

    def get_velocity(self, motor_id):
        return self.velocity[motor_id - 1]

    def get_torque(self, motor_id):
        return self.torque[motor_id - 1]

    def get_temperature(self, motor_id):
        return self.temperature[motor_id - 1]
