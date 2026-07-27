"""USB 串口设备探测 — 通过总线上特有电机/IMU 识别各 USB-CAN 角色.

五总线拓扑 (2026-07):
  lz_can_a  灵足 CAN-A: ID 1,2,5,6,17           — 前腿主关节 + head_roll
  incos_can 因克斯独立: ID 3,7                  — 前腿小腿
  lz_can_b  灵足 CAN-B: ID 10,11,13,14,15,16,21 — 后腿从关节 + 头部 + waist_roll
  evo_can   泉智博 EVO:  ID 9,12,18,19,20        — 后腿 hip + 颈腰
  dm_can    达妙 u2can:  ID 4,8                  — 前腿 tarsus (S2325)
  imu       WT901G4K IMU (串口 115200)
  tail_485  FTDI USB-RS485                      — 尾巴电机通讯
  mouth_esp32 ESP32/M5Stack                     — 嘴巴控制器

识别策略:
  - dm_can: u2can 适配器 USB VID:PID = 2e88:4603 (HDSC CDC Device), 与其余 CH340
    (1a86:7523) 完全不同, 可直接按 USB 描述符识别, 与物理插口/插拔顺序无关。
  - lz_can_a / lz_can_b / evo_can / incos_can: CH340 转 USB-CAN 无唯一序列号,
    通过总线上探测各角色专属电机 ID 区分；结果与物理 USB Hub 插口 (ID_PATH)
    绑定。因克斯还可按已知插口 platform-fc880000...1.1.4 识别 (电机未上电时)。
  - imu: 非 USB-CAN 协议适配器 (AT 指令握手会失败), 通过被动监听 0x55 帧头
    识别。
"""

import glob
import os
import struct
import subprocess
import time

from marsdog_control.hardware.motors.can_serial import CAN_EFF_FLAG, CanSerial

BAUD_CAN = 921600
BAUD_IMU = 115200

CMD_READ_PARAM = 0x11
IDX_MECH_POS = 0x7019
CMD_REST_STATE = 0xFD
MEVO_THETA_MIN, MEVO_THETA_MAX = -12.5, 12.5

# 各总线用于识别的特征电机 (多 ID 投票，降低误报)
PROBE_LZ_CAN_A_IDS = [1, 2, 5, 6, 17]
PROBE_INCOS_IDS = [3, 7]
PROBE_LZ_CAN_B_IDS = [10, 11, 13, 14, 15, 16, 21]
PROBE_EVO_IDS = [9, 12, 18, 19, 20]

# 因克斯独立 USB-CAN 的固定 Hub 物理口 (电机未上电时靠此识别)
INCOS_CAN_ID_PATH = "platform-fc880000.usb-usb-0:1.1.4:1.0"

INCOS_QUERY_POSITION = bytes([0xE1, 0x01])

# 达妙 u2can 适配器的 USB 身份 (与 CH340 完全不同, 免受插口/顺序影响)
DM_CAN_USB_VID = "2e88"
DM_CAN_USB_PID = "4603"

# 尾巴 USB-RS485 (FTDI FT232R). 尾巴电机可以不上电, 只识别通讯适配器。
TAIL_485_USB_VID = "0403"
TAIL_485_USB_PID = "6001"
TAIL_485_USB_SERIAL_PREFIX = "FTDI_FT232R_USB_UART"

# 嘴巴 ESP32/M5Stack 也走 FTDI VID:PID, 需要靠 USB 字符串/物理口区分。
MOUTH_USB_SERIAL_PREFIX = "Hades2001_M5stack"

ROLE_NAMES = {
    "lz_can_a": "灵足 CAN-A (前腿主关节 + head_roll)",
    "incos_can": "因克斯独立 CAN (前腿小腿 ID 3,7)",
    "lz_can_b": "灵足 CAN-B (后腿从关节 + 头部 + waist_roll)",
    "evo_can":  "泉智博 EVO CAN (后腿hip + 颈腰)",
    "dm_can":   "达妙 u2can (前腿 tarsus S2325)",
    "imu":      "WT901G4K IMU",
    "tail_485": "FTDI USB-RS485 (尾巴电机通讯)",
    "mouth_esp32": "ESP32/M5Stack (嘴巴控制器)",
}

SYMLINK_BY_ROLE = {
    "lz_can_a": "marsdog_lz_can_a",
    "incos_can": "marsdog_incos_can",
    "lz_can_b": "marsdog_lz_can_b",
    "evo_can":  "marsdog_evo_can",
    "dm_can":   "marsdog_dm_can",
    "imu":      "marsdog_imu",
    "tail_485": "marsdog_tail_485",
    "mouth_esp32": "marsdog_mouth",
}


def list_tty_devices():
    """返回当前所有 ttyUSB / ttyACM 设备路径。"""
    devs = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
    return [d for d in devs if os.path.exists(d)]


def get_udev_property(dev, prop):
    try:
        out = subprocess.check_output(
            ["udevadm", "info", "-q", "property", "-n", dev],
            text=True,
            stderr=subprocess.DEVNULL,
        )
    except (subprocess.CalledProcessError, FileNotFoundError):
        return ""
    prefix = prop + "="
    for line in out.splitlines():
        if line.startswith(prefix):
            return line[line.index("=") + 1 :]
    return ""


def get_device_info(dev):
    return {
        "dev": dev,
        "id_path": get_udev_property(dev, "ID_PATH"),
        "id_vendor": get_udev_property(dev, "ID_VENDOR_ID"),
        "id_product": get_udev_property(dev, "ID_MODEL_ID"),
        "id_serial": get_udev_property(dev, "ID_SERIAL"),
        "driver": get_udev_property(dev, "ID_USB_DRIVER"),
    }


def _uint_to_float(v, x_min, x_max, bits):
    maxv = (1 << bits) - 1
    return v / maxv * (x_max - x_min) + x_min


def probe_lz(serial, motor_id, timeout=0.08):
    frame_id = (CMD_READ_PARAM << 24) | (0x00 << 16) | (0xFD << 8) | motor_id
    buf = bytearray(8)
    struct.pack_into("<H", buf, 0, IDX_MECH_POS)

    serial.flush()
    if not serial.send_msg(bytes(buf), 8, frame_id):
        return False

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        result = serial.read_msg()
        if result is None:
            time.sleep(0.001)
            continue
        can_id, _dlc, data = result
        if not (can_id & CAN_EFF_FLAG):
            continue
        resp_id = (can_id >> 8) & 0xFF
        if resp_id == motor_id and len(data) >= 8:
            return True
    return False


def probe_evo(serial, motor_id, timeout=0.08):
    data = bytes([0xFF] * 7 + [CMD_REST_STATE])
    serial.flush()
    if not serial.send_msg(data, 8, motor_id):
        return False

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        result = serial.read_msg()
        if result is None:
            time.sleep(0.001)
            continue
        can_id, _dlc, rdata = result
        if can_id & CAN_EFF_FLAG:
            continue
        if len(rdata) < 8 or rdata[0] > 0x05:
            continue
        if (can_id & 0x7FF) == motor_id:
            return True
    return False


def probe_incos(serial, motor_id, timeout=0.08):
    """因克斯 E1 01 位置查询；有标准帧或 MIT 反馈即视为命中。"""
    serial.flush()
    if not serial.send_msg(INCOS_QUERY_POSITION, len(INCOS_QUERY_POSITION), motor_id):
        return False

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        result = serial.read_msg()
        if result is None:
            time.sleep(0.001)
            continue
        can_id, _dlc, data = result
        if can_id & CAN_EFF_FLAG:
            continue
        if (can_id & 0x7FF) != motor_id or len(data) < 2:
            continue
        frame_type = data[0] >> 5
        if frame_type == 5 and len(data) >= 6 and data[1] == 1:
            return True
        if frame_type == 1 and len(data) == 8:
            return True
    return False


def probe_imu(dev, timeout=1.2, baud=BAUD_IMU):
    """WT901 帧头 0x55, type 0x50~0x5A 任一有效帧即可 (校验和匹配)。

    IMU 数据是被动输出、和 CAN 总线握手无关，某些型号/线材在刚上电或总线
    干扰时会有短暂静默 (读到连续 0x00)，因此这里给足够长的时间窗口 (而非
    严格要求某几种帧类型)，只要出现任意一帧校验和通过就判定在线。
    """
    # 优先 pyserial；没有则用 termios 原生读
    try:
        import marsdog_control.apps.tools.misc.serial as serial
    except ImportError:
        return _probe_imu_termios(dev, baud=baud, timeout=timeout)

    try:
        ser = serial.Serial(dev, baud, timeout=0.1)
    except (OSError, serial.SerialException):
        return False

    try:
        ser.reset_input_buffer()
        deadline = time.monotonic() + timeout
        buf = bytearray()
        while time.monotonic() < deadline:
            chunk = ser.read(64)
            if not chunk:
                continue
            buf.extend(chunk)
            while len(buf) >= 11:
                if buf[0] != 0x55:
                    del buf[0]
                    continue
                frame_type = buf[1]
                if 0x50 <= frame_type <= 0x5A:
                    checksum = sum(buf[:10]) & 0xFF
                    if checksum == buf[10]:
                        return True
                del buf[0]
    finally:
        ser.close()
    return False


def _probe_imu_termios(dev, baud=BAUD_IMU, timeout=1.2):
    """无 pyserial 时用 termios 探测 WT901。"""
    import select
    import termios

    try:
        fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    except OSError:
        return False

    try:
        attrs = termios.tcgetattr(fd)
        attrs[0] = 0
        attrs[1] = 0
        attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        attrs[3] = 0
        speed = getattr(termios, f"B{baud}", termios.B115200)
        attrs[4] = speed
        attrs[5] = speed
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        termios.tcflush(fd, termios.TCIOFLUSH)

        deadline = time.monotonic() + timeout
        buf = bytearray()
        while time.monotonic() < deadline:
            r, _, _ = select.select([fd], [], [], 0.1)
            if not r:
                continue
            try:
                chunk = os.read(fd, 64)
            except BlockingIOError:
                continue
            if not chunk:
                continue
            buf.extend(chunk)
            while len(buf) >= 11:
                if buf[0] != 0x55:
                    del buf[0]
                    continue
                if 0x50 <= buf[1] <= 0x5A and (sum(buf[:10]) & 0xFF) == buf[10]:
                    return True
                del buf[0]
        return False
    finally:
        os.close(fd)


def _score_bus(can, role):
    if role == "evo_can":
        ids = PROBE_EVO_IDS
        probe_fn = probe_evo
    elif role == "lz_can_a":
        ids = PROBE_LZ_CAN_A_IDS
        probe_fn = lambda c, mid: probe_lz(c, mid)
    elif role == "lz_can_b":
        ids = PROBE_LZ_CAN_B_IDS
        probe_fn = lambda c, mid: probe_lz(c, mid)
    elif role == "incos_can":
        ids = PROBE_INCOS_IDS
        probe_fn = probe_incos
    else:
        return 0

    hits = 0
    for mid in ids:
        if probe_fn(can, mid):
            hits += 1
    return hits


def identify_device(dev):
    """识别单个串口设备角色，失败返回 None。"""
    info = get_device_info(dev)

    # 1) 达妙 u2can: 直接按 USB VID:PID 识别 (与插口/顺序无关)
    if (info.get("id_vendor", "").lower() == DM_CAN_USB_VID and
            info.get("id_product", "").lower() == DM_CAN_USB_PID):
        return "dm_can", info

    # 1b) 嘴巴 ESP32/M5Stack 和尾巴 USB-RS485 都可能是 FTDI 0403:6001,
    #     优先按 USB 字符串区分；都不要求外设上电。
    if (info.get("id_vendor", "").lower() == TAIL_485_USB_VID and
            info.get("id_product", "").lower() == TAIL_485_USB_PID):
        serial = info.get("id_serial", "")
        if serial.startswith(MOUTH_USB_SERIAL_PREFIX):
            return "mouth_esp32", info
        if serial.startswith(TAIL_485_USB_SERIAL_PREFIX):
            return "tail_485", info
        if info.get("id_path") == "platform-fc880000.usb-usb-0:1.2.4:1.0":
            return "mouth_esp32", info
        return "tail_485", info

    # 1c) 因克斯独立口：已知 Hub 物理口，电机未上电也能认适配器
    if info.get("id_path") == INCOS_CAN_ID_PATH:
        return "incos_can", info

    # 2) 灵足/泉智博/因克斯 CH340: 需要在总线上探测特征电机
    can = CanSerial()
    if can.begin(dev, BAUD_CAN):
        time.sleep(0.05)
        scores = {
            "evo_can": _score_bus(can, "evo_can"),
            "lz_can_a": _score_bus(can, "lz_can_a"),
            "lz_can_b": _score_bus(can, "lz_can_b"),
            "incos_can": _score_bus(can, "incos_can"),
        }
        can.end()
        info["scores"] = scores

        best_role, best_score = max(scores.items(), key=lambda x: x[1])
        if best_score >= 1:
            return best_role, info

    # 3) IMU: 非 USB-CAN 协议, AT 握手会失败, 靠被动监听识别
    if probe_imu(dev):
        return "imu", info

    # 4) IMU 掉回出厂波特率 (例如 flash 保存指令时序问题/意外恢复出厂设置):
    #    尝试用常见波特率探测，如果确实是 IMU 就自动修复为 BAUD_IMU 并重新验证。
    other_baud = _detect_and_fix_imu_baud(dev)
    if other_baud:
        info["imu_baud_fixed_from"] = other_baud
        return "imu", info

    return None, info


def _detect_and_fix_imu_baud(dev, target_baud=BAUD_IMU):
    """检测 dev 是否为工作在非目标波特率的 IMU，如是则自动改回并持久化。

    返回修复前检测到的波特率 (int)，若不是 IMU 或未能修复则返回 None。
    """
    try:
        from marsdog_control.apps.tools.calibration.imu_set_baud import (
            detect_current_baud, set_baud_persist)
    except ImportError:
        return None

    other_bauds = [b for b in (9600, 230400, 57600, 38400, 19200, 4800) if b != target_baud]
    current = None
    for b in other_bauds:
        if probe_imu(dev, timeout=0.3) if b == target_baud else _probe_imu_at(dev, b):
            current = b
            break
    if current is None:
        return None

    ok, msg = set_baud_persist(dev, target_baud, current_baud=current, verbose=False)
    return current if ok else None


def _probe_imu_at(dev, baud, timeout=0.3):
    """临时以指定波特率探测是否能读到合法 WT901 帧 (用于发现跑错波特率的 IMU)。"""
    try:
        import marsdog_control.apps.tools.misc.serial as serial
    except ImportError:
        return False
    try:
        ser = serial.Serial(dev, baud, timeout=0.1)
    except (OSError, serial.SerialException):
        return False
    try:
        ser.reset_input_buffer()
        deadline = time.monotonic() + timeout
        buf = bytearray()
        while time.monotonic() < deadline:
            chunk = ser.read(64)
            if not chunk:
                continue
            buf.extend(chunk)
            while len(buf) >= 11:
                if buf[0] != 0x55:
                    del buf[0]
                    continue
                ftype = buf[1]
                if 0x50 <= ftype <= 0x5A:
                    if sum(buf[:10]) & 0xFF == buf[10]:
                        return True
                del buf[0]
    finally:
        ser.close()
    return False


def scan_all_devices():
    """扫描所有串口，返回 {role: info} 和未识别列表。"""
    found = {}
    unknown = []

    for dev in list_tty_devices():
        role, info = identify_device(dev)
        info["role"] = role
        if role is None:
            unknown.append(info)
            continue
        if role in found:
            info["duplicate_of"] = found[role]["dev"]
            unknown.append(info)
            continue
        found[role] = info

    return found, unknown


def render_udev_rules(found):
    """根据识别结果生成 udev 规则文本。"""
    lines = [
        "# Marsdog USB 设备固定别名",
        "# 由 setup_usb_devices.py 自动生成",
        "#",
        "# 所有固定别名优先绑定 USB Hub 物理口 (ID_PATH), 只要不换插口即可稳定生效",
        "# 换插口后需重新运行: sudo python3 setup_usb_devices.py --install",
        "",
    ]

    for role in ("lz_can_a", "lz_can_b", "evo_can", "dm_can", "imu",
                 "tail_485", "mouth_esp32"):
        info = found.get(role)
        if not info or not info.get("id_path"):
            continue
        symlink = SYMLINK_BY_ROLE[role]
        lines.append(
            f'SUBSYSTEM=="tty", ENV{{ID_PATH}}=="{info["id_path"]}", '
            f'SYMLINK+="{symlink}", GROUP="dialout", MODE="0666"'
        )

    lines.append("")
    return "\n".join(lines)


def resolve_symlink(role):
    """返回某角色的设备路径，优先 /dev/marsdog_* 符号链接。"""
    link = f"/dev/{SYMLINK_BY_ROLE[role]}"
    if os.path.exists(link):
        return os.path.realpath(link)
    return ""
