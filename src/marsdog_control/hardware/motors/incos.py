"""ENCOS/INCOS EC-A2806 USB-CAN motor driver.

The protocol is ported from body_code_incos_usb(1). It uses the same AT
USB-CAN serial framing as the other serial CAN adapters in this project.
All public position/speed units are radians and radians/second.
"""

from __future__ import annotations

import math
import struct
import threading
import time
from typing import Optional

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
# ENCOS §9.3 query code 31: CAN timeout (ms)
QUERY_CAN_TIMEOUT = 31
# ENCOS §9.2.10 config code 0x0b: CAN timeout (ms); 0 = keep last command
CFG_CAN_TIMEOUT = 0x0B
DEFAULT_CAN_TIMEOUT_MS = 500
# Modest MIT hold if last cmd missing when parking enabled
_HOLD_KP = 40.0
_HOLD_KD = 2.0


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

        # Seed MIT hold so keepalive bridges the bring-up gap (esp. DM probe).
        for mid in self._active_ids:
            if self.is_connected[mid - 1]:
                q = self.get_position(mid)
                self.mit_control(mid, q, 0.0, _HOLD_KP, _HOLD_KD, 0.0)
                time.sleep(0.002)

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
                    # Same descending-ID bulk order as mit_controls (see there).
                    cmds = sorted(cmds, key=lambda item: int(item[0]), reverse=True)
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

    @staticmethod
    def encode_can_timeout_frame(timeout_ms: int) -> bytes:
        """ENCOS V1.19 §9.2.10: config CAN timeout (ms); 0 disables timeout.

        Example (manual): ``C00B01F4`` → 500ms.
        """
        ms = max(0, min(0xFFFF, int(timeout_ms)))
        # mode=0x06 in top 3 bits, reserved=0 → 0xC0; code=0x0b; uint16 BE
        return bytes([0xC0, CFG_CAN_TIMEOUT, (ms >> 8) & 0xFF, ms & 0xFF])

    def _drain_rx(self, window_s: float = 0.02) -> None:
        deadline = time.monotonic() + max(0.0, float(window_s))
        while time.monotonic() < deadline:
            with self._lock:
                msg = self._serial.read_msg()
            if not msg:
                time.sleep(0.001)
                continue
            self._handle_msg(msg)

    def _wait_config_or_query(
        self,
        motor_id: int,
        *,
        want_timeout_ms: Optional[int] = None,
        timeout_s: float = 0.15,
    ) -> tuple[bool, Optional[int]]:
        """Wait for config ACK (§10 type4 / 0xFF 0xFE) or query-31 reply.

        Returns ``(ok, reported_timeout_ms)``.
        """
        deadline = time.monotonic() + max(0.05, float(timeout_s))
        reported = None
        got_cfg_ok = False
        while time.monotonic() < deadline:
            with self._lock:
                msg = self._serial.read_msg()
            if not msg:
                time.sleep(0.001)
                continue
            can_id, dlc, data = msg
            if can_id & CAN_EFF_FLAG:
                continue
            mid = can_id & 0x7FF
            if mid != motor_id or dlc < 2:
                # Keep other motors' MIT feedback parsed.
                self._handle_msg(msg)
                continue
            frame_type = data[0] >> 5
            # §9.2.x config reply: FF FE 0B <uint16>
            if dlc >= 5 and data[0] == 0xFF and data[1] == 0xFE and data[2] == CFG_CAN_TIMEOUT:
                reported = (data[3] << 8) | data[4]
                got_cfg_ok = True
                if want_timeout_ms is None or reported == int(want_timeout_ms):
                    return True, reported
            # §10 type4 config: Byte1=code, Byte2=0 fail / 1 ok (some FW)
            if frame_type == 4 and dlc >= 3 and data[1] == CFG_CAN_TIMEOUT:
                if data[2] == 1:
                    got_cfg_ok = True
                    if want_timeout_ms is None:
                        return True, reported
                elif data[2] == 0:
                    return False, reported
            # §10 type5 query: Byte1=31, Byte2-3=uint16 ms
            if frame_type == 5 and dlc >= 4 and data[1] == QUERY_CAN_TIMEOUT:
                reported = (data[2] << 8) | data[3]
                if want_timeout_ms is None or reported == int(want_timeout_ms):
                    return True, reported
            self._handle_msg(msg)
        return got_cfg_ok, reported

    def set_can_timeout_ms(
        self, motor_id, timeout_ms: int, *, verify: bool = False
    ) -> bool:
        """Set per-motor CAN command timeout (ENCOS §9.2.10 / Timeout).

        ``timeout_ms=0``: keep executing the last MIT command after bus disconnect.
        Default firmware value is 500ms (controller disables on silence).
        """
        if not 1 <= motor_id <= MAX_ID:
            return False
        if not verify:
            return self._send(self.encode_can_timeout_frame(timeout_ms), 4, motor_id)
        was_running = self._pause_recv()
        try:
            return self._set_can_timeout_ms_verified(motor_id, int(timeout_ms))
        finally:
            self._resume_recv(was_running)

    def _set_can_timeout_ms_verified(self, motor_id: int, timeout_ms: int) -> bool:
        """Configure timeout while RX/keepalive are already paused."""
        payload = self.encode_can_timeout_frame(timeout_ms)
        target = int(timeout_ms)
        for attempt in range(4):
            # Keep MIT alive across the cfg window (timeout still 500ms until set).
            if self.is_connected[motor_id - 1]:
                self._refresh_hold_cmd(motor_id)
            with self._lock:
                self._serial.flush()
            if not self._send(payload, 4, motor_id):
                time.sleep(0.01)
                continue
            ok, reported = self._wait_config_or_query(
                motor_id, want_timeout_ms=target, timeout_s=0.12)
            if ok and reported == target:
                return True
            self.query_parameter(motor_id, QUERY_CAN_TIMEOUT)
            ok_q, reported = self._wait_config_or_query(
                motor_id, want_timeout_ms=target, timeout_s=0.12)
            if ok_q and reported == target:
                return True
            if reported is not None:
                print(
                    f"[Incos] motor {motor_id}: timeout cfg "
                    f"want={target} got={reported} (try {attempt + 1}/4)"
                )
            else:
                print(
                    f"[Incos] motor {motor_id}: timeout cfg no ACK/query "
                    f"(try {attempt + 1}/4)"
                )
            time.sleep(0.02)
        return False

    def _refresh_hold_cmd(self, motor_id) -> None:
        """Re-send hold MIT at current pose with non-zero gains."""
        with self._cmd_lock:
            cmd = self._last_cmd.get(motor_id)
        q = self.get_position(motor_id)
        if cmd is not None:
            _q0, _dq, kp, kd, tau = cmd
            kp = max(float(kp), _HOLD_KP)
            kd = max(float(kd), _HOLD_KD)
            self.mit_control(motor_id, q, 0.0, kp, kd, float(tau))
            return
        self.mit_control(motor_id, q, 0.0, _HOLD_KP, _HOLD_KD, 0.0)

    def _bulk_hold(self) -> None:
        with self._cmd_lock:
            cmds = [
                (mid, self._last_cmd[mid])
                for mid in self._active_ids
                if mid in self._last_cmd and self.is_connected[mid - 1]
            ]
        if not cmds:
            return
        frames = []
        for mid, (_q, _dq, kp, kd, tau) in sorted(cmds, key=lambda x: -int(x[0])):
            q = self.get_position(mid)
            kp = max(float(kp), _HOLD_KP)
            kd = max(float(kd), _HOLD_KD)
            frames.append((self._encode_mit(kp, kd, q, 0.0, float(tau)), 8, mid))
            self._record_cmd(mid, q, 0.0, kp, kd, float(tau))
        with self._lock:
            self._serial.send_bulk(frames)
        self._last_tx_monotonic = time.monotonic()

    def end(self, *, disable=True):
        if disable:
            for mid in self._active_ids:
                if self.is_connected[mid - 1]:
                    self.set_can_timeout_ms(mid, DEFAULT_CAN_TIMEOUT_MS, verify=False)
                    time.sleep(0.002)
            self._running = False
            self.stop_keepalive()
            if self._thread:
                self._thread.join(timeout=1.0)
                self._thread = None
            for mid in self._active_ids:
                if self.is_connected[mid - 1]:
                    self.mit_control(mid, self.get_position(mid), 0.0, 0.0, 0.0, 0.0)
                    self.is_enabled[mid - 1] = False
                    time.sleep(0.002)
        else:
            # Keep-enabled park: ENCOS Timeout=0. Pause keepalive first — otherwise
            # some IDs miss the cfg frame and still drop after the default 500ms.
            print("[Incos] park hold: set CAN timeout=0 + freeze last MIT")
            self._pause_recv()
            try:
                self._drain_rx(0.02)
                for mid in self._active_ids:
                    if self.is_connected[mid - 1]:
                        self._refresh_hold_cmd(mid)
                        time.sleep(0.003)
                self._bulk_hold()
                time.sleep(0.02)
                for mid in self._active_ids:
                    if not self.is_connected[mid - 1]:
                        continue
                    ok = self._set_can_timeout_ms_verified(mid, 0)
                    self._refresh_hold_cmd(mid)
                    print(
                        f"[Incos] motor {mid}: CAN timeout→0 "
                        f"{'VERIFIED' if ok else 'FAILED (may drop after 500ms)'}"
                    )
                    time.sleep(0.005)
                for _ in range(3):
                    self._bulk_hold()
                    time.sleep(0.01)
            finally:
                self._running = False
                self.stop_keepalive()
                if self._thread:
                    self._thread.join(timeout=1.0)
                    self._thread = None
        if self._owns_serial:
            self._serial.end()

    def query_parameter(self, motor_id, query_code):
        if not 1 <= motor_id <= MAX_ID or not 1 <= query_code <= 39:
            return False
        return self._send(bytes([0xE1, query_code]), 2, motor_id)

    @staticmethod
    def encode_set_zero_frame(motor_id: int) -> bytes:
        """ENCOS V1.19 §7.2: set-current-as-zero (no angle offset), 4 bytes."""
        mid = int(motor_id)
        return bytes([(mid >> 8) & 0xFF, mid & 0xFF, 0x00, 0x03])

    def _pause_recv(self):
        """Stop background RX so 0x7FF ACK is not stolen/dropped."""
        self.stop_keepalive()
        was_running = self._running
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        return was_running

    def _resume_recv(self, was_running: bool):
        if was_running and self._thread is None:
            self._running = True
            self._thread = threading.Thread(target=self._recv_loop, daemon=True)
            self._thread.start()
            self.start_keepalive()

    def set_zero_position(self, motor_id, timeout_s=0.8):
        """Set current encoder position as mechanical zero (ENCOS §7.2).

        Broadcast on CAN ID ``0x7FF``: ``[id_hi, id_lo, 0x00, 0x03]``.
        Success ACK: ``[id_hi, id_lo, 0x01, 0x03]`` on ``0x7FF``.
        Manual: hold still ≥500ms before the command.
        """
        if not 1 <= motor_id <= MAX_ID:
            return False

        # Hold still with zero gains, then pause RX for the ACK window.
        cur = self.get_position(motor_id)
        self.mit_control(motor_id, cur, 0.0, 0.0, 0.0, 0.0)
        time.sleep(0.55)

        was_running = self._pause_recv()
        try:
            payload = self.encode_set_zero_frame(motor_id)
            with self._lock:
                self._serial.flush()
                if not self._serial.send_msg(payload, 4, 0x7FF):
                    return False
            deadline = time.monotonic() + max(0.2, float(timeout_s))
            while time.monotonic() < deadline:
                with self._lock:
                    msg = self._serial.read_msg()
                if not msg:
                    time.sleep(0.002)
                    continue
                can_id, dlc, data = msg
                cid = can_id & 0x7FF
                if cid == 0x7FF and dlc >= 4:
                    mid = (data[0] << 8) | data[1]
                    if mid != motor_id:
                        continue
                    if data[2] == 0x01 and data[3] == 0x03:
                        return True
                    if data[2] == 0x01 and data[3] == 0x00:
                        return False
                elif not (can_id & CAN_EFF_FLAG) and 1 <= cid <= MAX_ID and dlc >= 1:
                    self._parse_feedback(cid, data[:dlc])
            # No ACK: still try a position query — some adapters drop 0x7FF RX.
            self.query_parameter(motor_id, QUERY_POSITION)
            time.sleep(0.05)
            with self._lock:
                while True:
                    msg = self._serial.read_msg()
                    if not msg:
                        break
                    can_id, dlc, data = msg
                    cid = can_id & 0x7FF
                    if not (can_id & CAN_EFF_FLAG) and 1 <= cid <= MAX_ID:
                        self._parse_feedback(cid, data[:dlc])
            return abs(self.get_position(motor_id)) < math.radians(8.0)
        finally:
            self._resume_recv(was_running)

    def mit_control(self, motor_id, pos_rad, vel_rad=0.0,
                    kp=10.0, kd=0.5, torque_ff=0.0):
        data = self._encode_mit(kp, kd, pos_rad, vel_rad, torque_ff)
        ok = self._send(data, 8, motor_id)
        if ok and 1 <= motor_id <= MAX_ID:
            self.is_enabled[motor_id - 1] = kp > 0.0 or kd > 0.0
            self._record_cmd(motor_id, pos_rad, vel_rad, kp, kd, torque_ff)
        return ok

    def mit_controls(self, motor_ids, positions, velocities, kps, kds, torques):
        # USB-CAN + 4 轴同拍 bulk：按 JOINT_MAP 升序 (2,3,6,7) 连发时，线上
        # raw 回包会系统性丢掉中间 ID（实测 ID6 rx/tx≈0.17；单独控 ID6=1.0）。
        # 降序 (7,6,3,2) 四轴均≈1.0。根因在总线/适配器 RX，不是电机坏。
        packed = list(zip(motor_ids, positions, velocities, kps, kds, torques))
        packed.sort(key=lambda row: int(row[0]), reverse=True)
        frames = [(self._encode_mit(kp, kd, q, dq, tau), 8, mid)
                  for mid, q, dq, kp, kd, tau in packed]
        with self._lock:
            ok = self._serial.send_bulk(frames)
        if ok:
            self._last_tx_monotonic = time.monotonic()
            for mid, q, dq, kp, kd, tau in packed:
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
