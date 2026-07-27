"""达妙 S2325 电机驱动 — 基于 u2can USB-CAN 适配器 (Python port).

u2can 适配器使用 ttyACM* 设备, 921600 波特率, 自有帧协议:
  发送帧: 30 字节 (header 0x55 0xAA)
  接收帧: 16 字节 (header 0xAA, footer 0x55)

达妙 MIT 控制帧与灵足 MIT 协议类似但编码略有不同。

硬件接线:
  u2can USB-CAN → CAN 总线 → 达妙 S2325 (ID 4: fl_tarsus, ID 8: fr_tarsus)
"""

import struct
import time
import os
import termios
import select
import threading

# S2325 参数 (若官方手册有更准确值请更新)
DM_S2325_PMAX = 12.5      # 位置范围 ±12.5 rad
DM_S2325_DQMAX = 30.0     # 速度范围 ±30 rad/s
DM_S2325_TAUMAX = 10.0    # 力矩范围 ±10 Nm
DM_S2325_KP_MAX = 500.0
DM_S2325_KD_MAX = 5.0

DM_KNOWN_IDS = [4, 8]

# 电机控制模式寄存器值 (RID=10 CTRL_MODE), 对照达妙手册/官方例程 damiao.h
MIT_MODE = 1
POS_VEL_MODE = 2
VEL_MODE = 3
POS_FORCE_MODE = 4
POS_VEL_CSP_MODE = 5
VEL_CSP_MODE = 6
TORQUE_CSP_MODE = 7

_CTRL_MODE_NAMES = {
    MIT_MODE: "MIT_MODE",
    POS_VEL_MODE: "POS_VEL_MODE",
    VEL_MODE: "VEL_MODE",
    POS_FORCE_MODE: "POS_FORCE_MODE",
    POS_VEL_CSP_MODE: "POS_VEL_CSP_MODE",
    VEL_CSP_MODE: "VEL_CSP_MODE",
    TORQUE_CSP_MODE: "TORQUE_CSP_MODE",
}

# 寄存器地址 (部分, 具体见达妙手册)
REG_MST_ID = 7
REG_ESC_ID = 8
REG_TIMEOUT = 9
REG_CTRL_MODE = 10

# 这些寄存器是 uint32 类型 (其余大多是 float), 与 damiao.h::is_in_ranges 保持一致
_UINT32_REGS = set(range(7, 11)) | set(range(13, 17)) | set(range(35, 37))

_PARAM_TARGET_ID = 0x7FF  # 参数读/写/保存帧的目标CAN ID (广播, 电机ID编码在data里)
_PARAM_MAX_RETRIES = 20
_PARAM_RETRY_INTERVAL_S = 0.05

# u2can 帧常量
_SEND_FRAME_LEN = 30
_RECV_FRAME_LEN = 16
_RECV_HEADER = 0xAA
_RECV_FOOTER = 0x55


def _float_to_uint(x, x_min, x_max, bits):
    x = max(x_min, min(x_max, x))
    span = x_max - x_min
    return int((x - x_min) / span * ((1 << bits) - 1) + 0.5)


def _uint_to_float(v, x_min, x_max, bits):
    return v / ((1 << bits) - 1) * (x_max - x_min) + x_min


def _build_send_frame(can_id, data_8, cmd=0x01):
    """构造 30 字节 u2can 发送帧.

    cmd: 0x01=转发CAN数据帧并反馈发送状态(用于探测/诊断)
         0x03=非反馈CAN转发(高频控制时用, 减少总线负载)
    """
    f = bytearray(30)
    f[0], f[1] = 0x55, 0xAA
    f[2] = 0x1E              # frame len
    f[3] = cmd
    struct.pack_into('<I', f, 4, 1)    # sendTimes = 1
    struct.pack_into('<I', f, 8, 10)   # timeInterval = 10
    f[12] = 0x00             # IDType: 标准帧
    struct.pack_into('<I', f, 13, can_id)
    f[17] = 0x00             # frameType: 数据帧
    f[18] = 0x08             # CAN data len = 8
    f[19] = 0x00             # idAcc
    f[20] = 0x00             # dataAcc
    f[21:29] = bytes(data_8[:8])
    f[29] = 0x00             # crc (u2can 不校验)
    return bytes(f)


class MotorDamiao:
    """达妙电机控制器 — 单条 u2can 总线上可挂多个电机."""

    def __init__(self):
        self._fd = -1
        self._lock = threading.Lock()
        self._motors = {}     # slave_id -> {master_id, pos, vel, tau, err}
        self._recv_buf = bytearray()
        self._command_lock = threading.Lock()
        self._command_event = threading.Event()
        self._commands = {}
        self._worker = None
        self._worker_stop = threading.Event()

    # ── 初始化/关闭 ──────────────────────────────────────────────

    def begin(self, device, baud=921600):
        """打开 u2can USB-CAN 串口. 返回 True/False."""
        try:
            fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        except OSError:
            return False

        attrs = termios.tcgetattr(fd)
        attrs[0] = 0               # iflag
        attrs[1] = 0               # oflag
        attrs[2] &= ~termios.CSIZE
        attrs[2] |= termios.CS8
        attrs[2] &= ~termios.PARENB
        attrs[2] &= ~termios.CSTOPB
        attrs[2] |= termios.CLOCAL | termios.CREAD
        attrs[3] = 0               # lflag
        attrs[4] = termios.B921600
        attrs[5] = termios.B921600
        attrs[6][termios.VTIME] = 0
        attrs[6][termios.VMIN] = 0
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        termios.tcflush(fd, termios.TCIFLUSH)

        self._fd = fd
        self._recv_buf = bytearray()
        return True

    def end(self):
        self.stop_worker()
        if self._fd >= 0:
            os.close(self._fd)
            self._fd = -1

    def is_open(self):
        return self._fd >= 0

    def add_motor(self, slave_id, master_id=None):
        """注册电机. master_id 默认 = slave_id + 0x10."""
        if master_id is None:
            master_id = slave_id + 0x10
        self._motors[slave_id] = {
            'master_id': master_id,
            'pos': 0.0, 'vel': 0.0, 'tau': 0.0, 'err': 0,
            'feedback_ts': 0.0, 'feedback_seq': 0,
            'command_ts': 0.0, 'command_seq': 0,
            'command_q': 0.0, 'command_dq': 0.0,
            'command_kp': 0.0, 'command_kd': 0.0, 'command_tau': 0.0,
            'rtt_s': float('nan'), 'dropped_commands': 0,
        }

    # ── 底层收发 ─────────────────────────────────────────────────

    def _send_raw(self, frame_bytes):
        if self._fd < 0:
            return False
        try:
            os.write(self._fd, frame_bytes)
            return True
        except OSError:
            return False

    def _read_available(self, timeout_s=0.005):
        """读取串口可用数据追加到 _recv_buf."""
        if self._fd < 0:
            return
        r, _, _ = select.select([self._fd], [], [], timeout_s)
        if r:
            try:
                chunk = os.read(self._fd, 1024)
                self._recv_buf.extend(chunk)
            except OSError:
                pass

    def _parse_one_frame(self):
        """从 _recv_buf 中解析一帧 u2can 接收数据. 返回 (cmd, can_id, can_data) 或 None."""
        while len(self._recv_buf) >= _RECV_FRAME_LEN:
            idx = self._recv_buf.find(bytes([_RECV_HEADER]))
            if idx < 0:
                self._recv_buf.clear()
                return None
            if idx > 0:
                del self._recv_buf[:idx]
            if len(self._recv_buf) < _RECV_FRAME_LEN:
                return None
            footer = self._recv_buf[_RECV_FRAME_LEN - 1]
            if footer != _RECV_FOOTER:
                del self._recv_buf[:1]
                continue
            frame = bytes(self._recv_buf[:_RECV_FRAME_LEN])
            del self._recv_buf[:_RECV_FRAME_LEN]

            cmd = frame[1]
            can_id = struct.unpack_from('<I', frame, 3)[0]
            can_data = frame[7:15]
            return (cmd, can_id, can_data)
        return None

    def _process_feedback(self, can_id, can_data, expected_slave_id=None):
        """解析 MIT 反馈帧, 更新电机状态.

        expected_slave_id: 若提供, 说明这是"刚给这个 slave_id 发了直接寻址的控制帧,
        现在等它的回复"这种严格串行场景 (enable/disable/set_zero/refresh_status/
        control_mit 都是这种模式) —— 此时直接把这帧数据记到该电机上, 不再按
        can_id/master_id 做归属判断。这是必要的, 因为多个达妙电机可以配置相同的
        MasterID (咱们这两个电机目前都是0x63), 那样它们的反馈帧 canId 完全一样,
        单靠 canId 猜不出是哪个电机回的; 但只要访问是严格串行的(一次只对一个
        slave_id 发指令并等它的回复, 不并发), 回复就必然属于刚才寻址的那个电机。
        """
        err = (can_data[0] >> 4) & 0x0F
        q_uint = (can_data[1] << 8) | can_data[2]
        dq_uint = (can_data[3] << 4) | (can_data[4] >> 4)
        tau_uint = ((can_data[4] & 0x0F) << 8) | can_data[5]

        pos = _uint_to_float(q_uint, -DM_S2325_PMAX, DM_S2325_PMAX, 16)
        vel = _uint_to_float(dq_uint, -DM_S2325_DQMAX, DM_S2325_DQMAX, 12)
        tau = _uint_to_float(tau_uint, -DM_S2325_TAUMAX, DM_S2325_TAUMAX, 12)

        if expected_slave_id is not None:
            m = self._motors.get(expected_slave_id)
            if m is not None:
                m['pos'], m['vel'], m['tau'], m['err'] = pos, vel, tau, err
                m['feedback_ts'] = time.monotonic()
                m['feedback_seq'] += 1
                if m['command_ts'] > 0.0:
                    m['rtt_s'] = max(0.0, m['feedback_ts'] - m['command_ts'])
                return expected_slave_id
            return None

        if can_id != 0:
            for sid, m in self._motors.items():
                if m['master_id'] == can_id:
                    m['pos'], m['vel'], m['tau'], m['err'] = pos, vel, tau, err
                    m['feedback_ts'] = time.monotonic()
                    m['feedback_seq'] += 1
                    return sid
        else:
            sid = can_data[0] & 0x0F
            if sid in self._motors:
                m = self._motors[sid]
                m['pos'], m['vel'], m['tau'], m['err'] = pos, vel, tau, err
                m['feedback_ts'] = time.monotonic()
                m['feedback_seq'] += 1
                return sid
        return None

    def _recv_and_parse(self, timeout_s=0.010, expected_slave_id=None):
        """读串口并解析所有可用帧.

        expected_slave_id: 见 _process_feedback 说明. 严格串行访问单个电机时务必传入,
        避免多个电机共用 MasterID 时的归属歧义。

        返回 (motor_ids_updated, link_ack_seen):
          motor_ids_updated: 收到真实电机遥测(CMD=0x11)的电机ID列表
          link_ack_seen: 是否收到过发送ACK(CMD=0x12), 用于判断适配器链路是否存活
        """
        self._read_available(timeout_s)
        parsed = []
        link_ack = False
        while True:
            result = self._parse_one_frame()
            if result is None:
                break
            cmd, can_id, can_data = result
            if cmd == 0x11:
                sid = self._process_feedback(can_id, can_data, expected_slave_id)
                if sid is not None:
                    parsed.append(sid)
            elif cmd == 0x12:
                link_ack = True
        return parsed, link_ack

    # ── 控制命令 ─────────────────────────────────────────────────

    def _control_cmd(self, slave_id, cmd_byte):
        data = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, cmd_byte])
        frame = _build_send_frame(slave_id, data)
        with self._lock:
            self._send_raw(frame)
            time.sleep(0.002)
            self._recv_and_parse(0.010, expected_slave_id=slave_id)
        return True

    def enable(self, slave_id):
        self._control_cmd(slave_id, 0xFC)
        time.sleep(0.050)

    def disable(self, slave_id):
        self._control_cmd(slave_id, 0xFD)

    def set_zero(self, slave_id):
        self._control_cmd(slave_id, 0xFE)

    # ── 寄存器读/写/保存 (切换控制模式等) ────────────────────────
    # 帧格式参考达妙官方例程 damiao.h: write_motor_param/read_motor_param/save_motor_param
    # 均以 target CAN ID = 0x7FF 发送, 电机 slave_id 编码在 data[0..1] (小端), 而不是用
    # slave_id 本身作为 CAN 帧ID (那是控制帧才用的寻址方式)。

    def _recv_param_frames(self, timeout_s=0.20):
        """读串口并只解析参数读/写应答帧 (canData[2] in (0x33,0x55)).

        返回 [(slave_id, RID, raw4bytes), ...]. 普通遥测帧(非参数应答)会被丢弃,
        与官方例程 receive_param() 行为一致 — 不要在这个函数运行期间同时期望
        收到正常 MIT 遥测, 二者不要交叉调用。
        """
        deadline = time.monotonic() + timeout_s
        out = []
        while time.monotonic() < deadline:
            self._read_available(0.01)
            while True:
                result = self._parse_one_frame()
                if result is None:
                    break
                cmd, _can_id, data = result
                if cmd == 0x11 and data[2] in (0x33, 0x55):
                    slave_id = data[0] | (data[1] << 8)
                    rid = data[3]
                    out.append((slave_id, rid, bytes(data[4:8])))
            if out:
                break
        return out

    def write_motor_param(self, slave_id, rid, raw4bytes):
        """写电机寄存器 (原始4字节, 不做类型转换)."""
        id_low = slave_id & 0xFF
        id_high = (slave_id >> 8) & 0xFF
        data = bytes([id_low, id_high, 0x55, rid]) + bytes(raw4bytes[:4])
        frame = _build_send_frame(_PARAM_TARGET_ID, data, cmd=0x01)
        with self._lock:
            self._recv_buf.clear()
            self._send_raw(frame)

    def read_motor_param(self, slave_id, rid, timeout_s=None):
        """读电机寄存器. 返回 float 或 int (取决于寄存器类型), 超时返回 None."""
        id_low = slave_id & 0xFF
        id_high = (slave_id >> 8) & 0xFF
        data = bytes([id_low, id_high, 0x33, rid, 0, 0, 0, 0])
        frame = _build_send_frame(_PARAM_TARGET_ID, data, cmd=0x01)
        for _ in range(_PARAM_MAX_RETRIES):
            with self._lock:
                self._recv_buf.clear()
                self._send_raw(frame)
                frames = self._recv_param_frames(timeout_s or _PARAM_RETRY_INTERVAL_S)
            for sid, got_rid, raw in frames:
                if sid == slave_id and got_rid == rid:
                    if rid in _UINT32_REGS:
                        return struct.unpack('<I', raw)[0]
                    return struct.unpack('<f', raw)[0]
        return None

    def change_motor_param(self, slave_id, rid, value, verify=True):
        """修改电机寄存器参数 (自动按寄存器类型编码 float/uint32), 返回是否成功(若verify)."""
        if rid in _UINT32_REGS:
            raw = struct.pack('<I', int(value))
        else:
            raw = struct.pack('<f', float(value))
        self.write_motor_param(slave_id, rid, raw)
        if not verify:
            return True
        readback = self.read_motor_param(slave_id, rid)
        if readback is None:
            return False
        if rid in _UINT32_REGS:
            return int(readback) == int(value)
        return abs(readback - float(value)) < 0.1

    def get_control_mode(self, slave_id):
        """读取当前控制模式 (RID=10 CTRL_MODE). 返回 int 或 None(超时)."""
        v = self.read_motor_param(slave_id, REG_CTRL_MODE)
        return int(v) if v is not None else None

    def switch_control_mode(self, slave_id, mode):
        """切换控制模式并校验. mode: MIT_MODE / POS_VEL_MODE / VEL_MODE / ...
        返回 True/False。注意: 大多数达妙电机切换控制模式后需要 save_motor_param()
        才能在断电重启后保持, 且切换/保存前建议先 disable() 电机。
        """
        return self.change_motor_param(slave_id, REG_CTRL_MODE, mode)

    def save_motor_param(self, slave_id):
        """保存电机当前所有寄存器参数到 flash (断电不丢失). 会先 disable 电机。"""
        self.disable(slave_id)
        id_low = slave_id & 0xFF
        id_high = (slave_id >> 8) & 0xFF
        data = bytes([id_low, id_high, 0xAA, 0x01, 0, 0, 0, 0])
        frame = _build_send_frame(_PARAM_TARGET_ID, data, cmd=0x01)
        with self._lock:
            self._recv_buf.clear()
            self._send_raw(frame)
            self._recv_param_frames(0.05)  # 消耗掉可能的应答, 不强制要求
        time.sleep(0.15)  # 等待 flash 写入完成

    def refresh_status(self, slave_id):
        """获取电机当前位置/速度/力矩 (不会使能/产生运动).

        实测这颗 S2325 固件对官方例程里的 0xCC "广播刷新状态" 命令 (target=0x7FF)
        不回应 (可能固件版本/型号差异), 但对直接寻址到 slave_id 的 disable(0xFD)
        指令始终会回传一帧完整遥测 (位置/速度/力矩/err), 且这条指令本身是安全的
        (电机保持失能状态, 不会转动)。因此这里改用 disable 帧本身作探测手段。

        返回 (motor_responded, link_ack): motor_responded=电机是否回传遥测,
        link_ack=适配器是否确认发送成功(用于判断链路本身是否存活).
        """
        if slave_id not in self._motors:
            return False, False
        data = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD])
        frame = _build_send_frame(slave_id, data)
        with self._lock:
            self._send_raw(frame)
            time.sleep(0.002)
            parsed, link_ack = self._recv_and_parse(0.015, expected_slave_id=slave_id)
        return (slave_id in parsed), link_ack

    def control_mit(self, slave_id, kp, kd, q, dq, tau):
        """MIT 模式控制: kp/kd/位置/速度/力矩."""
        if slave_id not in self._motors:
            return
        kp_u = _float_to_uint(kp, 0, DM_S2325_KP_MAX, 12)
        kd_u = _float_to_uint(kd, 0, DM_S2325_KD_MAX, 12)
        q_u = _float_to_uint(q, -DM_S2325_PMAX, DM_S2325_PMAX, 16)
        dq_u = _float_to_uint(dq, -DM_S2325_DQMAX, DM_S2325_DQMAX, 12)
        tau_u = _float_to_uint(tau, -DM_S2325_TAUMAX, DM_S2325_TAUMAX, 12)

        data = bytearray(8)
        data[0] = (q_u >> 8) & 0xFF
        data[1] = q_u & 0xFF
        data[2] = dq_u >> 4
        data[3] = ((dq_u & 0x0F) << 4) | ((kp_u >> 8) & 0x0F)
        data[4] = kp_u & 0xFF
        data[5] = kd_u >> 4
        data[6] = ((kd_u & 0x0F) << 4) | ((tau_u >> 8) & 0x0F)
        data[7] = tau_u & 0xFF

        frame = _build_send_frame(slave_id, bytes(data), cmd=0x03)  # 高频控制, 不要求发送ACK
        with self._lock:
            m = self._motors[slave_id]
            m['command_ts'] = time.monotonic()
            m['command_seq'] += 1
            m['command_q'], m['command_dq'] = q, dq
            m['command_kp'], m['command_kd'], m['command_tau'] = kp, kd, tau
            self._send_raw(frame)
            self._recv_and_parse(0.005, expected_slave_id=slave_id)

    # ── 持久化异步收发 worker ────────────────────────────────────

    @property
    def worker_running(self):
        return self._worker is not None and self._worker.is_alive()

    def start_worker(self):
        """启动单 worker；ID 按注册顺序严格串行发送并等待对应反馈。"""
        if self.worker_running:
            return
        self._worker_stop.clear()
        self._worker = threading.Thread(
            target=self._worker_loop, name="damiao-io", daemon=True)
        self._worker.start()

    def stop_worker(self, timeout_s=1.0):
        if not self._worker:
            return
        self._worker_stop.set()
        self._command_event.set()
        self._worker.join(timeout_s)
        self._worker = None

    def set_commands(self, commands):
        """原子覆盖最新命令邮箱。

        commands: {slave_id: (kp, kd, q, dq, tau)}。主控制循环永不等待反馈。
        """
        now = time.monotonic()
        with self._command_lock:
            for sid, values in commands.items():
                if sid not in self._motors:
                    continue
                previous = self._commands.get(sid)
                seq = (previous[0] + 1) if previous else 1
                self._commands[sid] = (seq, now, tuple(values))
        self._command_event.set()

    def _worker_loop(self):
        last_sent = {}
        while not self._worker_stop.is_set():
            self._command_event.wait(0.005)
            self._command_event.clear()
            with self._command_lock:
                snapshot = dict(self._commands)
            for sid in DM_KNOWN_IDS:
                if self._worker_stop.is_set():
                    break
                item = snapshot.get(sid)
                if item is None:
                    continue
                seq, _submitted_ts, values = item
                prev_seq = last_sent.get(sid, 0)
                if seq <= prev_seq:
                    continue
                skipped = max(0, seq - prev_seq - 1)
                self._motors[sid]['dropped_commands'] += skipped
                self.control_mit(sid, *values)
                last_sent[sid] = seq

    def get_timing(self, slave_id, now=None):
        """返回该电机命令/反馈时序快照，所有时间均为 monotonic。"""
        m = self._motors.get(slave_id)
        if not m:
            return {}
        now = time.monotonic() if now is None else now
        return {
            'command_ts': m['command_ts'],
            'command_seq': m['command_seq'],
            'feedback_ts': m['feedback_ts'],
            'feedback_seq': m['feedback_seq'],
            'feedback_age_s': (max(0.0, now - m['feedback_ts'])
                               if m['feedback_ts'] > 0.0 else float('inf')),
            'rtt_s': m['rtt_s'],
            'dropped_commands': m['dropped_commands'],
            'command_q': m['command_q'],
            'command_dq': m['command_dq'],
            'command_kp': m['command_kp'],
            'command_kd': m['command_kd'],
            'command_tau': m['command_tau'],
        }

    # ── 状态查询 ─────────────────────────────────────────────────

    def get_position(self, slave_id):
        m = self._motors.get(slave_id)
        return m['pos'] if m else 0.0

    def get_velocity(self, slave_id):
        m = self._motors.get(slave_id)
        return m['vel'] if m else 0.0

    def get_torque(self, slave_id):
        m = self._motors.get(slave_id)
        return m['tau'] if m else 0.0

    def get_error(self, slave_id):
        m = self._motors.get(slave_id)
        return m['err'] if m else -1

    # ── 探测 (static_test 用) ────────────────────────────────────

    def probe(self, slave_id):
        """探测电机是否在线.

        返回 (online, pos_rad, err, link_ok):
          online  = 电机是否真正回传遥测帧 (CMD=0x11)
          link_ok = u2can 适配器本身链路是否存活 (收到过 CMD=0x12 送达ACK)
                    若 link_ok=False, 说明适配器本身没反应(usb/供电/驱动问题);
                    若 link_ok=True 但 online=False, 说明适配器正常但电机未上电/未接线/ID不对。
        """
        if slave_id not in self._motors:
            self.add_motor(slave_id)

        self._recv_buf.clear()
        m = self._motors[slave_id]

        online = False
        link_ok = False
        for _ in range(4):
            got, ack = self.refresh_status(slave_id)
            online = online or got
            link_ok = link_ok or ack
            if online:
                break
            time.sleep(0.015)

        return online, m['pos'], m['err'], link_ok
