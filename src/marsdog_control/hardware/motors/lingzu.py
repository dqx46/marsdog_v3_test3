"""灵足电机驱动 v2 — 基于验证过的 marsdog_firmware motor_driver.py

基于已在实际全步态控制中验证过的老版本驱动, 做最小修改以适配新硬件.

保留 (来自老驱动, 核心逻辑不改):
- python-can Bus.recv() 接收 CAN 帧 (自动处理 loopback)
- raw socket 发送 CAN 帧 (低延迟)
- _recv_loop 只处理 type 0x02 (MIT 反馈帧, 不处理 auto_report)
- 使能仅需 enable 命令 (不调 set_run_mode, 不调 auto_report)

新增:
- init()/init_serial()/end() 延迟初始化
- Serial (USB-CAN) 电机支持 (via CanSerial)
- MAX_ID=20 (原 12)
- 多圈位置补偿
- 型号特定速度/力矩编码范围
- is_enabled/is_connected/get_position()/re_enable() 接口
"""
import math
import os
import socket
import struct
import time
import threading

try:
    import can as _pycan
except ImportError:
    _pycan = None  # SocketCAN 模式不可用，但 USB-CAN 串口模式仍正常工作

# [解耦] 真实实现已下沉到此 src 模块; 保持逐字一致的扁平 import, 由
# ensure_legacy_path() 保证 can_serial/joint_config 可解析(其 compat 别名回指 src)。
from marsdog_control.compat import ensure_legacy_path as _ensure_legacy_path
from marsdog_control.compat import legacy_dir as _legacy_dir
_ensure_legacy_path()

from marsdog_control.hardware.motors.can_serial import CanSerial

MAX_ID = 24
MASTER_ID = 0xFD

CAN_FRAME_FMT = "=IB3x8s"
CAN_EFF_FLAG = 0x80000000

P_MIN, P_MAX = -12.57, 12.57
KP_MIN, KP_MAX = 0.0, 500.0
KD_MIN, KD_MAX = 0.0, 5.0

MOTOR_RANGES = {
    'EL05': (-50.0, 50.0, -6.0, 6.0),
    'RS02': (-44.0, 44.0, -17.0, 17.0),
    'RS00': (-33.0, 33.0, -14.0, 14.0),
}
DEFAULT_RANGE = (-50.0, 50.0, -6.0, 6.0)

# lz_can_a: 仅前腿髋 + head_roll。前腿大腿/小腿 ID 2/3/6/7 已迁到因克斯独立总线。
RS05_CAN_IDS = [1, 5, 17]
RS05_SERIAL_IDS = [10, 11, 13, 14, 15, 16, 21]  # lz_can_b: 后腿从关节 + 头/腰


def float_to_uint(x, x_min, x_max, bits):
    span = x_max - x_min
    if x < x_min: x = x_min
    if x > x_max: x = x_max
    return int(((x - x_min) * ((1 << bits) - 1)) / span)


def uint_to_float(u, x_min, x_max, bits):
    span = x_max - x_min
    return float(u) * span / ((1 << bits) - 1) + x_min


class MotorLz:
    """Drop-in replacement for motor_lz.MotorLz, based on old proven driver."""

    def __init__(self):
        self.bus = None       # python-can Bus (CAN RX, proven)
        self.tx_sock = None   # raw socket   (CAN TX, fast)
        self._serial = CanSerial()          # ttyUSB2 — 后腿/头/腰 串口LZ
        self._can1_serial = CanSerial()     # ttyUSB0 — 前腿 CAN1 LZ (USB-CAN 模式)
        self._serial_set = set()            # motor IDs routed to _serial
        self._can1_set = set()              # motor IDs routed to _can1_serial
        self._serial_lock = threading.Lock()
        self._can1_lock = threading.Lock()
        self._can1_std_handlers = []

        self.position    = [0.0] * MAX_ID
        self.velocity    = [0.0] * MAX_ID
        self.torque      = [0.0] * MAX_ID
        self.temperature = [0.0] * MAX_ID
        self.fault       = [0]   * MAX_ID
        self.mode        = [0]   * MAX_ID
        self.is_enabled  = [False] * MAX_ID
        self.is_connected = [False] * MAX_ID

        self._pos_offset = [0.0] * MAX_ID
        self._v_range = [DEFAULT_RANGE[:2]] * MAX_ID
        self._t_range = [DEFAULT_RANGE[2:]] * MAX_ID

        self.rx_count = [0] * MAX_ID
        self.tx_count = [0] * MAX_ID
        self.tx_err   = 0

        self.is_running = False
        self._thread = None
        self._init_hold_stop = None
        self._init_hold_thread = None

        # 软件标定偏置（补偿 Flash 写入失败的老固件电机，如 Motor 12）
        self._calib_offset = self._load_calib()

    @staticmethod
    def _load_calib():
        """从 motor_calib.json 加载软件标定偏置，返回 {motor_id: offset_rad}。"""
        import json
        calib = {}
        calib_path = os.path.join(str(_legacy_dir()), "motor_calib.json")
        if os.path.exists(calib_path):
            try:
                with open(calib_path) as f:
                    data = json.load(f)
                for k, v in data.get("offsets", {}).items():
                    calib[int(k)] = float(v)
                if calib:
                    print("[MotorLz] 软件标定偏置已加载: "
                          + ", ".join(f"Motor {k}={v:+.4f}rad" for k, v in calib.items()))
            except Exception as e:
                print(f"[MotorLz] 加载 motor_calib.json 失败: {e}")
        return calib

    # ── model config ──────────────────────────────────────────────

    def set_motor_model(self, motor_id, model):
        r = MOTOR_RANGES.get(model, DEFAULT_RANGE)
        idx = motor_id - 1
        self._v_range[idx] = (r[0], r[1])
        self._t_range[idx] = (r[2], r[3])

    def _load_models(self):
        try:
            from marsdog_control.config.joints import JOINT_BY_ID
            for mid, jd in JOINT_BY_ID.items():
                if jd.model:
                    self.set_motor_model(mid, jd.model)
        except ImportError:
            pass

    # ── low-level (from old driver, kept intact) ──────────────────

    def _build_ext_id(self, comm_type, data2, target_id):
        return ((comm_type & 0x1F) << 24) | ((data2 & 0xFFFF) << 8) | (target_id & 0xFF)

    def _send(self, motor_id, ext_id, data):
        """Route frame to _can1_serial / _serial / raw CAN socket."""
        if isinstance(data, list):
            data_bytes = bytes(data) + b'\x00' * (8 - len(data))
        else:
            data_bytes = (bytes(data) + b'\x00' * 8)[:8]

        if motor_id in self._can1_set:
            with self._can1_lock:
                self._can1_serial.send_msg(data_bytes, 8, ext_id)
        elif motor_id in self._serial_set:
            with self._serial_lock:
                self._serial.send_msg(data_bytes, 8, ext_id)
        else:
            if self.tx_sock is None:
                return
            frame = struct.pack(CAN_FRAME_FMT,
                                ext_id | CAN_EFF_FLAG, 8, data_bytes)
            try:
                self.tx_sock.send(frame)
            except OSError:
                self.tx_err += 1

        mid = ext_id & 0xFF
        if 1 <= mid <= MAX_ID:
            self.tx_count[mid - 1] += 1

    # ── public API (from old driver) ──────────────────────────────

    def enable(self, motor_id):
        ext_id = self._build_ext_id(0x3, MASTER_ID, motor_id)
        self._send(motor_id, ext_id, [0] * 8)

    def disable(self, motor_id, clear_fault=False):
        ext_id = self._build_ext_id(0x4, MASTER_ID, motor_id)
        data = [0] * 8
        if clear_fault:
            data[0] = 1
        self._send(motor_id, ext_id, data)

    def set_origin(self, motor_id):
        """SetOrigin (comm_type=0x06): 将当前位置设为零点（RAM，需 save_params 写 Flash）。
        data[0]=1 是必须的，全零 payload 电机会忽略。"""
        ext_id = self._build_ext_id(0x06, MASTER_ID, motor_id)
        data = [0] * 8
        data[0] = 1
        self._send(motor_id, ext_id, data)
        return True

    def write_param(self, motor_id, index, value_bytes):
        """WriteParam (comm_type=0x12): 写单个参数（掉电丢失，需 save_params 持久化）。"""
        ext_id = self._build_ext_id(0x12, MASTER_ID, motor_id)
        data = [0] * 8
        data[0] = index & 0xFF
        data[1] = (index >> 8) & 0xFF
        for i, b in enumerate(value_bytes[:4]):
            data[4 + i] = b
        self._send(motor_id, ext_id, data)

    def save_params(self, motor_id):
        """SaveParams (comm_type=0x16): 将参数永久写入 Flash。
        payload 必须是 [0x01..0x08]，全零无效。"""
        ext_id = self._build_ext_id(0x16, MASTER_ID, motor_id)
        self._send(motor_id, ext_id, [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08])
        return True

    def set_zero_via_offset(self, motor_id, current_pos_rad):
        """老固件专用: 写 add_offset 参数(0x702B) 移动零点，不触发运动，重启后生效。
        适用于 set_origin 无效的老固件 RS00 电机（如 Motor 9/12）。"""
        b = list(struct.pack('<f', current_pos_rad))
        self.write_param(motor_id, 0x702B, b)
        time.sleep(0.05)
        self.save_params(motor_id)

    def mit_control(self, motor_id, pos_rad, vel_rad=0.0,
                    kp=15.0, kd=1.5, torque_ff=0.0):
        """Type 1: MIT control (from old driver, + multi-turn & model ranges)."""
        idx = motor_id - 1
        pos_raw = pos_rad + self._pos_offset[idx]
        v_lo, v_hi = self._v_range[idx]
        t_lo, t_hi = self._t_range[idx]

        p_u  = float_to_uint(pos_raw, P_MIN, P_MAX, 16)
        v_u  = float_to_uint(vel_rad, v_lo, v_hi, 16)
        kp_u = float_to_uint(kp, KP_MIN, KP_MAX, 16)
        kd_u = float_to_uint(kd, KD_MIN, KD_MAX, 16)
        t_u  = float_to_uint(torque_ff, t_lo, t_hi, 16)

        ext_id = self._build_ext_id(0x1, t_u, motor_id)
        data = [
            (p_u >> 8) & 0xFF, p_u & 0xFF,
            (v_u >> 8) & 0xFF, v_u & 0xFF,
            (kp_u >> 8) & 0xFF, kp_u & 0xFF,
            (kd_u >> 8) & 0xFF, kd_u & 0xFF,
        ]
        self._send(motor_id, ext_id, data)

    def re_enable(self, motor_id):
        """Clear fault then re-enable (same as old driver reset_motors.py)."""
        self.disable(motor_id, clear_fault=True)
        time.sleep(0.05)
        self.enable(motor_id)
        time.sleep(0.05)
        self.mit_control(motor_id, self.get_position(motor_id), 0.0, 0.0, 0.0, 0.0)
        time.sleep(0.05)
        return self.is_enabled[motor_id - 1]

    # ── multi-turn compensation (新增) ────────────────────────────

    def _calc_pos_offset(self, motor_id):
        idx = motor_id - 1
        raw = self.position[idx]
        TWO_PI = 2.0 * math.pi

        # 多圈补偿（对齐到最近整圈）
        n = round(raw / TWO_PI)
        mt_offset = n * TWO_PI

        # 软件标定偏置（老固件 Flash 写入失败时使用）
        calib = self._calib_offset.get(motor_id, 0.0)

        self._pos_offset[idx] = mt_offset + calib

        corrected = raw - self._pos_offset[idx]
        if n != 0:
            print(f"[MotorLz] Motor {motor_id} multi-turn fix: "
                  f"{math.degrees(raw):+.1f} -> {math.degrees(corrected):+.1f} "
                  f"(offset {n}x360)")
        if calib != 0.0:
            print(f"[MotorLz] Motor {motor_id} calib offset applied: "
                  f"{math.degrees(raw):+.1f} -> {math.degrees(corrected):+.1f} "
                  f"(calib={math.degrees(calib):+.1f}°)")

    # ── feedback parsing (from old driver, + model ranges) ────────

    def _parse_feedback(self, motor_id, ext_id, data):
        """Parse type-2 feedback frame. ext_id = 29-bit ID (no EFF flag)."""
        if len(data) < 8:
            return
        idx = motor_id - 1
        d = data
        p_u = (d[0] << 8) | d[1]
        v_u = (d[2] << 8) | d[3]
        t_u = (d[4] << 8) | d[5]
        tmp_raw = (d[6] << 8) | d[7]

        v_lo, v_hi = self._v_range[idx]
        t_lo, t_hi = self._t_range[idx]

        self.position[idx]    = uint_to_float(p_u, P_MIN, P_MAX, 16)
        self.velocity[idx]    = uint_to_float(v_u, v_lo, v_hi, 16)
        self.torque[idx]      = uint_to_float(t_u, t_lo, t_hi, 16)
        self.temperature[idx] = tmp_raw / 10.0
        self.fault[idx]       = (ext_id >> 16) & 0x3F
        self.mode[idx]        = (ext_id >> 22) & 0x03
        self.is_enabled[idx]  = (self.mode[idx] == 2)
        self.rx_count[idx]   += 1

    # ── receive thread (from old driver, + serial) ────────────────

    # ── 独立接收线程（每条串口各一个，消除串行阻塞）────────────────

    def _recv_serial_loop(self, cs: 'CanSerial', lock: threading.Lock, tag: str):
        """通用串口接收线程：非阻塞轮询，无数据时 sleep 0.5ms。"""
        while self.is_running:
            got = False
            for _ in range(32):   # 每次唤醒最多处理 32 帧
                with lock:
                    result = cs.read_msg()
                if result is None:
                    break
                got = True
                can_id, dlc, data = result
                if not (can_id & CAN_EFF_FLAG):
                    if tag == "can1":
                        for handler in tuple(self._can1_std_handlers):
                            handler(can_id, dlc, data)
                    continue
                eid = can_id & 0x1FFFFFFF
                typ = (eid >> 24) & 0x1F
                mid = (eid >> 8) & 0xFF
                if typ == 0x02 and 1 <= mid <= MAX_ID:
                    self._parse_feedback(mid, eid, data)
            if not got:
                time.sleep(0.0005)   # 0.5ms yield，不空转 CPU

    def _recv_loop(self):
        """兼容旧接口：仅在 SocketCAN 模式下使用。串口模式已迁移至独立线程。"""
        while self.is_running:
            if self.bus:
                msg = self.bus.recv(0.005)
                if msg is not None:
                    self._dispatch_pycan(msg)
                    while True:
                        msg = self.bus.recv(0.0)
                        if msg is None:
                            break
                        self._dispatch_pycan(msg)
            else:
                time.sleep(0.005)

    def _dispatch_pycan(self, msg):
        """Process python-can Message: only type 0x02 (from old driver)."""
        if not msg.is_extended_id:
            return
        eid = msg.arbitration_id
        typ = (eid >> 24) & 0x1F
        mid = (eid >> 8) & 0xFF
        if typ == 0x02 and 1 <= mid <= MAX_ID:
            self._parse_feedback(mid, eid, msg.data)

    # ── 批量 MIT 控制（同一串口的所有电机一次 send_bulk）────────────

    def mit_controls_can1(self, ids, positions, velocities=None,
                          kps=None, kds=None, torques=None):
        """CAN1 串口批量 MIT 控制，一次 write() 发出所有帧。"""
        n = len(ids)
        if velocities is None: velocities = [0.0] * n
        if kps is None:        kps = [15.0] * n
        if kds is None:        kds = [1.5] * n
        if torques is None:    torques = [0.0] * n

        frames = []
        for i, mid in enumerate(ids):
            idx = mid - 1
            pos_raw = positions[i] + self._pos_offset[idx]
            v_lo, v_hi = self._v_range[idx]
            t_lo, t_hi = self._t_range[idx]
            p_u  = float_to_uint(pos_raw, P_MIN, P_MAX, 16)
            v_u  = float_to_uint(velocities[i], v_lo, v_hi, 16)
            kp_u = float_to_uint(kps[i], KP_MIN, KP_MAX, 16)
            kd_u = float_to_uint(kds[i], KD_MIN, KD_MAX, 16)
            t_u  = float_to_uint(torques[i], t_lo, t_hi, 16)
            ext_id = self._build_ext_id(0x1, t_u, mid)
            data = bytes([
                (p_u >> 8) & 0xFF, p_u & 0xFF,
                (v_u >> 8) & 0xFF, v_u & 0xFF,
                (kp_u >> 8) & 0xFF, kp_u & 0xFF,
                (kd_u >> 8) & 0xFF, kd_u & 0xFF,
            ])
            frames.append((data, 8, ext_id))
        with self._can1_lock:
            self._can1_serial.send_bulk(frames)

    def mit_controls_serial(self, ids, positions, velocities=None,
                            kps=None, kds=None, torques=None):
        """Serial 串口批量 MIT 控制，一次 write() 发出所有帧。"""
        n = len(ids)
        if velocities is None: velocities = [0.0] * n
        if kps is None:        kps = [15.0] * n
        if kds is None:        kds = [1.5] * n
        if torques is None:    torques = [0.0] * n

        frames = []
        for i, mid in enumerate(ids):
            idx = mid - 1
            pos_raw = positions[i] + self._pos_offset[idx]
            v_lo, v_hi = self._v_range[idx]
            t_lo, t_hi = self._t_range[idx]
            p_u  = float_to_uint(pos_raw, P_MIN, P_MAX, 16)
            v_u  = float_to_uint(velocities[i], v_lo, v_hi, 16)
            kp_u = float_to_uint(kps[i], KP_MIN, KP_MAX, 16)
            kd_u = float_to_uint(kds[i], KD_MIN, KD_MAX, 16)
            t_u  = float_to_uint(torques[i], t_lo, t_hi, 16)
            ext_id = self._build_ext_id(0x1, t_u, mid)
            data = bytes([
                (p_u >> 8) & 0xFF, p_u & 0xFF,
                (v_u >> 8) & 0xFF, v_u & 0xFF,
                (kp_u >> 8) & 0xFF, kp_u & 0xFF,
                (kd_u >> 8) & 0xFF, kd_u & 0xFF,
            ])
            frames.append((data, 8, ext_id))
        with self._serial_lock:
            self._serial.send_bulk(frames)

    # ── init (新增: 延迟初始化, 保持老驱动的简单 enable 方式) ─────

    def _start_recv_thread(self):
        """启动独立接收线程：每条总线各自检查，避免重复启动。"""
        self.is_running = True
        if self.bus and not getattr(self, '_socketcan_thread_started', False):
            self._socketcan_thread_started = True
            self._thread = threading.Thread(target=self._recv_loop, daemon=True)
            self._thread.start()
        if self._can1_set and not getattr(self, '_can1_thread_started', False):
            self._can1_thread_started = True
            t = threading.Thread(
                target=self._recv_serial_loop,
                args=(self._can1_serial, self._can1_lock, "can1"),
                daemon=True)
            t.start()
        if self._serial_set and not getattr(self, '_serial_thread_started', False):
            self._serial_thread_started = True
            t = threading.Thread(
                target=self._recv_serial_loop,
                args=(self._serial, self._serial_lock, "serial"),
                daemon=True)
            t.start()

    def add_can1_standard_handler(self, handler):
        """Register a parser for standard CAN frames on the CAN-A serial bus."""
        self._can1_std_handlers.append(handler)

    def hold_connected(self, gains_by_id=None, *, default_kp=80.0, default_kd=4.0):
        """MIT-hold every connected motor at its current pose (hot-start bridge)."""
        gains_by_id = gains_by_id or {}
        for mid in list(self._serial_set | self._can1_set):
            idx = mid - 1
            if not (0 <= idx < len(self.is_connected) and self.is_connected[idx]):
                continue
            q = self.get_position(mid)
            kp, kd, tau = gains_by_id.get(mid, (default_kp, default_kd, 0.0))
            try:
                self.mit_control(mid, float(q), 0.0, float(kp), float(kd), float(tau))
            except Exception:
                pass

    def ensure_mit(self, motor_id, *, tag="MotorLz", wait_s=0.15) -> bool:
        """Hot-start: if motor has feedback but mode!=2, enable once into MIT.

        Safe to call after the recv thread is running (async mode update).
        Returns True when motor is MIT and marked connected.
        """
        idx = motor_id - 1
        if not (0 <= idx < len(self.mode)):
            return False
        if self.rx_count[idx] <= 0:
            return False
        if self.mode[idx] != 2:
            print(
                f"[{tag}] motor {motor_id} mode={self.mode[idx]} "
                f"fault={self.fault[idx]} → enable (hot-start)"
            )
            self.enable(motor_id)
            t0 = time.monotonic()
            while time.monotonic() - t0 < wait_s:
                if self.mode[idx] == 2:
                    break
                time.sleep(0.005)
        if self.mode[idx] != 2:
            return False
        self.is_connected[idx] = True
        self._calc_pos_offset(motor_id)
        return True

    def claim_mit_all(
        self,
        motor_ids=None,
        *,
        tag="MotorLz",
        wait_s=0.15,
        hold_kp=80.0,
        hold_kd=4.0,
    ):
        """Force every LZ with feedback into MIT and latch a stiff hold frame.

        Hot-start probe can leave responders with rx>0 but mode!=2, or mode=2
        without a recent MIT command (looks unlocked). Call after bring-up /
        before pose_hold / fade.

        Returns ``(fixed_ids, failed_ids)``.
        """
        if motor_ids is None:
            ids = sorted(self._serial_set | self._can1_set)
            if not ids:
                ids = sorted(set(RS05_SERIAL_IDS) | set(RS05_CAN_IDS))
        else:
            ids = list(motor_ids)

        fixed: list[int] = []
        failed: list[int] = []
        for mid in ids:
            idx = mid - 1
            if not (0 <= idx < len(self.mode)):
                continue
            if self.rx_count[idx] <= 0:
                continue
            was_mode = int(self.mode[idx])
            was_conn = bool(self.is_connected[idx])
            ok = self.ensure_mit(mid, tag=tag, wait_s=wait_s)
            if not ok:
                # Second enable pulse — some RS00 need a longer settle.
                self.enable(mid)
                t0 = time.monotonic()
                while time.monotonic() - t0 < wait_s:
                    if self.mode[idx] == 2:
                        ok = True
                        break
                    time.sleep(0.005)
                if ok:
                    self.is_connected[idx] = True
                    self._calc_pos_offset(mid)
            if ok and self.mode[idx] == 2:
                q = float(self.get_position(mid))
                try:
                    self.mit_control(
                        mid, q, 0.0, float(hold_kp), float(hold_kd), 0.0)
                except Exception:
                    pass
                if was_mode != 2 or not was_conn:
                    fixed.append(mid)
            else:
                failed.append(mid)
                print(
                    f"[{tag}] WARNING ID{mid} 未进 MIT "
                    f"mode={self.mode[idx]} fault={self.fault[idx]} "
                    f"rx={self.rx_count[idx]}"
                )
        return fixed, failed

    def print_mit_status(self, motor_ids=None, *, tag="mit") -> None:
        """Print LZ mode/connected for diagnostics (mode==2 = MIT)."""
        if motor_ids is None:
            ids = sorted(self._serial_set | self._can1_set)
            if not ids:
                ids = sorted(set(RS05_SERIAL_IDS) | set(RS05_CAN_IDS))
        else:
            ids = list(motor_ids)
        print(f"[{tag}] LZ 驱动状态:")
        for mid in ids:
            idx = mid - 1
            if not (0 <= idx < len(self.mode)):
                continue
            mode = int(self.mode[idx])
            rx = int(self.rx_count[idx])
            if rx <= 0:
                continue
            flag = "OK" if mode == 2 and self.is_connected[idx] else "!!"
            print(
                f"  [{flag}] ID{mid:2d} mode={mode} "
                f"connected={bool(self.is_connected[idx])} "
                f"fault={int(self.fault[idx])} rx={rx}"
            )

    def _hot_start_claim(self, mid, *, bus_tag, read_once, wait_s=0.08) -> bool:
        """After a probe reply: enable if mode!=2, then MIT-hold if in MIT."""
        idx = mid - 1
        if self.rx_count[idx] <= 0:
            return False
        if self.mode[idx] != 2:
            print(
                f"[MotorLz] {bus_tag} motor {mid} mode={self.mode[idx]} "
                f"fault={self.fault[idx]} → enable (hot-start)"
            )
            self.enable(mid)
            t0 = time.monotonic()
            while time.monotonic() - t0 < wait_s:
                result = read_once()
                if result is None:
                    continue
                can_id, _dlc, data = result
                if not (can_id & CAN_EFF_FLAG):
                    continue
                eid = can_id & 0x1FFFFFFF
                rmid = (eid >> 8) & 0xFF
                if rmid == mid:
                    self._parse_feedback(mid, eid, data)
                    if self.mode[idx] == 2:
                        break
        if self.mode[idx] != 2:
            return False
        self.is_connected[idx] = True
        self._calc_pos_offset(mid)
        q = self.get_position(mid)
        self.mit_control(mid, q, 0.0, 80.0, 4.0, 0.0)
        return True

    def _start_init_hold(self) -> None:
        """200Hz hold while sequential enable probes later motors on this bus."""
        if self._init_hold_thread is not None and self._init_hold_thread.is_alive():
            return
        stop = threading.Event()
        self._init_hold_stop = stop

        def _loop():
            while not stop.wait(0.005):
                try:
                    self.hold_connected()
                except Exception:
                    pass

        self._init_hold_thread = threading.Thread(
            target=_loop, name="lz-init-hold", daemon=True)
        self._init_hold_thread.start()

    def _stop_init_hold(self) -> None:
        stop = self._init_hold_stop
        th = self._init_hold_thread
        self._init_hold_stop = None
        self._init_hold_thread = None
        if stop is not None:
            stop.set()
        if th is not None and th.is_alive():
            th.join(timeout=0.5)

    def init_serial(self, device="/dev/ttyUSB0", baud=921600, *, clear_fault=True):
        """Open serial USB-CAN and enable serial motors."""
        self._load_models()
        self._serial_set = set(RS05_SERIAL_IDS)

        if not self._serial.begin(device, baud):
            self._serial_set.clear()
            return False

        if not clear_fault:
            self._start_init_hold()

        if clear_fault:
            for mid in RS05_SERIAL_IDS:
                self.disable(mid, clear_fault=True)
                time.sleep(0.005)
            time.sleep(0.05)

        for mid in RS05_SERIAL_IDS:
            with self._serial_lock:
                self._serial.flush()
            ext_id = self._build_ext_id(0x3, MASTER_ID, mid)
            with self._serial_lock:
                self._serial.send_msg(b'\x00' * 8, 8, ext_id)
            # Read enable response synchronously
            t0 = time.monotonic()
            while time.monotonic() - t0 < 0.05:
                with self._serial_lock:
                    result = self._serial.read_msg()
                if result is None:
                    continue
                can_id, dlc, data = result
                if not (can_id & CAN_EFF_FLAG):
                    continue
                eid = can_id & 0x1FFFFFFF
                rmid = (eid >> 8) & 0xFF
                if rmid == mid:
                    self._parse_feedback(mid, eid, data)
                    break
            # Hot-start: if mode!=2, enable once; then freeze pose immediately.
            if not clear_fault:
                def _read_serial():
                    with self._serial_lock:
                        return self._serial.read_msg()

                self._hot_start_claim(
                    mid, bus_tag="serial", read_once=_read_serial)
            time.sleep(0.002)

        self._start_recv_thread()
        if not clear_fault:
            # Second chance after recv thread: still-not-MIT responders.
            for mid in RS05_SERIAL_IDS:
                if self.rx_count[mid - 1] > 0 and self.mode[mid - 1] != 2:
                    if self.ensure_mit(mid, tag="MotorLz/serial"):
                        q = self.get_position(mid)
                        self.mit_control(mid, q, 0.0, 80.0, 4.0, 0.0)

        # Verify and compute multi-turn offsets
        for mid in RS05_SERIAL_IDS:
            idx = mid - 1
            if self.rx_count[idx] > 0 and self.mode[idx] == 2:
                self.is_connected[idx] = True
                self._calc_pos_offset(mid)
            else:
                print(f"[MotorLz] serial motor {mid} init failed "
                      f"(mode={self.mode[idx]} fault={self.fault[idx]} "
                      f"rx={self.rx_count[idx]})")

        if not clear_fault:
            self.hold_connected()
            self._stop_init_hold()
        return True

    def init(self, interface="can1"):
        """Open CAN bus and enable CAN motors (from old driver pattern)."""
        self._load_models()

        os.system(f"ip link set down {interface} 2>/dev/null")
        ret = os.system(f"ip link set up {interface} type can bitrate 1000000")
        if ret != 0:
            print(f"[MotorLz] Failed to set up {interface}")
            return False

        # TX: raw socket (from old driver — 绕过 python-can 开销)
        self.tx_sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.tx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536)
        self.tx_sock.bind((interface,))

        # RX: python-can (from old driver — 自动处理 loopback)
        if _pycan is None:
            raise RuntimeError("python-can 未安装，SocketCAN 模式不可用；请使用 init_can1_serial()")
        self.bus = _pycan.interface.Bus(channel=interface, bustype='socketcan',
                                        bitrate=1000000)

        self._start_recv_thread()

        # Clear faults first
        for mid in RS05_CAN_IDS:
            self.disable(mid, clear_fault=True)
            time.sleep(0.005)
        time.sleep(0.05)

        # Simple enable (from old driver: just enable, no set_run_mode)
        for mid in RS05_CAN_IDS:
            self.enable(mid)
            time.sleep(0.008)
        time.sleep(0.3)

        # Zero-torque MIT to trigger feedback frames
        for mid in RS05_CAN_IDS:
            self.mit_control(mid, 0.0, 0.0, 0.0, 0.0, 0.0)
            time.sleep(0.003)
        time.sleep(0.1)

        # Verify
        for mid in RS05_CAN_IDS:
            idx = mid - 1
            if self.rx_count[idx] > 0 and self.mode[idx] == 2:
                self.is_connected[idx] = True
                self._calc_pos_offset(mid)
            else:
                print(f"[MotorLz] CAN motor {mid} init failed "
                      f"(mode={self.mode[idx]} fault={self.fault[idx]} "
                      f"rx={self.rx_count[idx]})")

        return True

    def init_can1_serial(self, device="/dev/ttyUSB0", baud=921600, *, clear_fault=True):
        """Open USB-CAN serial as CAN1 bus and enable CAN1 LZ motors (IDs 1-6,15)."""
        self._load_models()
        self._can1_set = set(RS05_CAN_IDS)

        if not self._can1_serial.begin(device, baud):
            self._can1_set.clear()
            return False

        if not clear_fault:
            self._start_init_hold()

        if clear_fault:
            for mid in RS05_CAN_IDS:
                self.disable(mid, clear_fault=True)
                time.sleep(0.005)
            time.sleep(0.05)

        # Enable and read feedback
        for mid in RS05_CAN_IDS:
            with self._can1_lock:
                self._can1_serial.flush()
            ext_id = self._build_ext_id(0x3, MASTER_ID, mid)
            with self._can1_lock:
                self._can1_serial.send_msg(b'\x00' * 8, 8, ext_id)
            t0 = time.monotonic()
            while time.monotonic() - t0 < 0.05:
                with self._can1_lock:
                    result = self._can1_serial.read_msg()
                if result is None:
                    continue
                can_id, dlc, data = result
                if not (can_id & CAN_EFF_FLAG):
                    continue
                eid = can_id & 0x1FFFFFFF
                rmid = (eid >> 8) & 0xFF
                if rmid == mid:
                    self._parse_feedback(mid, eid, data)
                    break
            if not clear_fault:
                def _read_can1():
                    with self._can1_lock:
                        return self._can1_serial.read_msg()

                self._hot_start_claim(
                    mid, bus_tag="CAN1-serial", read_once=_read_can1)
            time.sleep(0.002)

        self._start_recv_thread()
        if not clear_fault:
            for mid in RS05_CAN_IDS:
                if self.rx_count[mid - 1] > 0 and self.mode[mid - 1] != 2:
                    if self.ensure_mit(mid, tag="MotorLz/CAN1"):
                        q = self.get_position(mid)
                        self.mit_control(mid, q, 0.0, 80.0, 4.0, 0.0)

        for mid in RS05_CAN_IDS:
            idx = mid - 1
            if self.rx_count[idx] > 0 and self.mode[idx] == 2:
                self.is_connected[idx] = True
                self._calc_pos_offset(mid)
            else:
                print(f"[MotorLz] CAN1-serial motor {mid} init failed "
                      f"(mode={self.mode[idx]} fault={self.fault[idx]} "
                      f"rx={self.rx_count[idx]})")

        if not clear_fault:
            self.hold_connected()
            self._stop_init_hold()
        return True

    def end(self, *, disable=True):
        """Shutdown host-side IO. ``disable=True`` also sends motor disable."""
        self._stop_init_hold()
        self.is_running = False
        if self._thread:
            self._thread.join(timeout=2)
        if disable:
            for mid in RS05_CAN_IDS + RS05_SERIAL_IDS:
                if self.is_connected[mid - 1]:
                    self.disable(mid)
                    time.sleep(0.002)
        if self.tx_sock:
            self.tx_sock.close()
            self.tx_sock = None
        if self.bus:
            self.bus.shutdown()
            self.bus = None
        self._can1_serial.end()
        self._serial.end()

    # ── position helpers (新增) ───────────────────────────────────

    def get_position(self, motor_id):
        return self.position[motor_id - 1] - self._pos_offset[motor_id - 1]

    def get_velocity(self, motor_id):
        return self.velocity[motor_id - 1]

    def get_torque(self, motor_id):
        return self.torque[motor_id - 1]

    def get_temperature(self, motor_id):
        return self.temperature[motor_id - 1]
