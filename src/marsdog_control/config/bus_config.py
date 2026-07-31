"""USB-CAN / IMU / 手柄 / 喇叭设备路径配置。

总线拓扑 (2026-07-29 前腿大腿改因克斯):
  /dev/marsdog_lz_can_a   → 灵足 CAN-A (前腿髋 + head_roll):           ID 1,5,17
  /dev/marsdog_incos_can  → 因克斯独立 USB-CAN (前腿大腿+小腿):         ID 2,3,6,7
  /dev/marsdog_lz_can_b   → 灵足 CAN-B (后腿从 + 头/腰):               ID 10,11,13,14,15,16,21
  /dev/marsdog_evo_can    → 泉智博 EVO (后腿hip + 颈腰):                ID 9,12,18,19,20
  /dev/marsdog_dm_can     → 达妙 u2can (前腿 tarsus S2325, 外置1:2):   ID 4,8
  /dev/marsdog_imu        → WT901G4K IMU
  /dev/marsdog_tail_485   → FTDI USB-RS485 尾巴电机通讯
  /dev/marsdog_mouth      → ESP32/M5Stack 嘴巴控制器
  /dev/input/by-id/usb-S_TGZ_Controller_*-joystick → 假 PS2 / Xbox360 兼容接收器
  plughw:CARD=Device,DEV=0 → USB 喇叭/声卡

首次配置或更换 Hub 后运行:
  python3 -m marsdog_control.apps.tools.diagnostics.setup_usb_devices
  sudo python3 -m marsdog_control.apps.tools.diagnostics.setup_usb_devices --install
"""

import glob
import json
import os

# [解耦] 真实实现已从 mocap_to_real 下沉到此 src 模块。设备映射缓存文件
# usb_device_map.json 仍随部署留在 mocap_to_real, 故把数据目录锚定到 legacy 目录,
# 保持换线/缓存解析行为与原来逐字一致(不依赖本文件的物理位置)。
from marsdog_control.compat import legacy_dir as _legacy_dir

_DIR = str(_legacy_dir())
_DEVICE_MAP_FILE = os.path.join(_DIR, "usb_device_map.json")

_DEVICE_ALIASES = {
    "lz_can_a":  "/dev/marsdog_lz_can_a",
    "lz_can_b":  "/dev/marsdog_lz_can_b",
    "incos_can": "/dev/marsdog_incos_can",
    "evo_can":   "/dev/marsdog_evo_can",
    "dm_can":    "/dev/marsdog_dm_can",
    "imu":       "/dev/marsdog_imu",
    "tail_485":  "/dev/marsdog_tail_485",
    "mouth_esp32": "/dev/marsdog_mouth",
}

_DEVICE_FALLBACKS = {
    # 2026-07-31 串口扫描实测 (xhci-hcd.0.auto)。换物理插口后请重新运行 setup_usb_devices.py。
    # 口1.2.1=灵足CAN-A, 1.2.2=EVO, 1.2.3=灵足CAN-B
    # 口1.3.1=因克斯, 1.3.2=未识别备用, 1.3.3=达妙u2can(ACM)
    # 口1.4=WT901 IMU
    "lz_can_a": [
        "/dev/serial/by-path/platform-xhci-hcd.0.auto-usb-0:1.2.1:1.0-port0",
    ],
    "evo_can": [
        "/dev/serial/by-path/platform-xhci-hcd.0.auto-usb-0:1.2.2:1.0-port0",
    ],
    "lz_can_b": [
        "/dev/serial/by-path/platform-xhci-hcd.0.auto-usb-0:1.2.3:1.0-port0",
    ],
    # CH340 无唯一序列号：禁止用 by-id/ttyUSBn 回退，否则会误绑到灵足/EVO 的口。
    "incos_can": [
        "/dev/serial/by-path/platform-xhci-hcd.0.auto-usb-0:1.3.1:1.0-port0",
    ],
    "imu": [
        "/dev/serial/by-path/platform-xhci-hcd.0.auto-usb-0:1.4:1.0-port0",
    ],
    "dm_can": [
        "/dev/serial/by-path/platform-xhci-hcd.0.auto-usb-0:1.3.3:1.0",
        "/dev/serial/by-id/usb-HDSC_CDC_Device_00000000050C-if00",
        "/dev/ttyACM0",
    ],
    "tail_485": [
        "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG027R1O-if00-port0",
    ],
    "mouth_esp32": [
        "/dev/serial/by-id/usb-Hades2001_M5stack_E55260257C-if00-port0",
    ],
}

_GAMEPAD_CANDIDATES = [
    "/dev/input/by-id/usb-S_TGZ_Controller_3E529620-joystick",
    "/dev/input/by-id/usb-S_TGZ_Controller_3E529650-joystick",
    "/dev/input/by-id/usb-S_TGZ_Controller_3E529630-joystick",
    "/dev/input/js0",
]

SPEAKER_ALSA_DEVICE = "plughw:CARD=Device,DEV=0"


def _load_cached_map():
    if not os.path.exists(_DEVICE_MAP_FILE):
        return {}
    try:
        with open(_DEVICE_MAP_FILE, encoding="utf-8") as f:
            data = json.load(f)
        return {k: v for k, v in data.items() if k in _DEVICE_ALIASES and os.path.exists(v)}
    except (OSError, json.JSONDecodeError, TypeError):
        return {}


def _first_existing(paths, default):
    for path in paths:
        if os.path.exists(path):
            return path
    return default


def resolve_device(role):
    """优先缓存映射 / by-path 回退；仅当 udev 别名与缓存一致时才用别名。

    避免旧的 /dev/marsdog_* 符号链接在换线后仍劫持配置。
    """
    alias = _DEVICE_ALIASES[role]
    cached = _load_cached_map().get(role)
    fallback = _first_existing(_DEVICE_FALLBACKS.get(role, ()), alias)

    preferred = cached or fallback
    if os.path.exists(alias):
        if not preferred or not os.path.exists(preferred):
            return alias
        # 别名与偏好路径指向同一真实设备时，用别名（固定名更清晰）
        if os.path.realpath(alias) == os.path.realpath(preferred):
            return alias
    return preferred if os.path.exists(preferred) else alias


def resolve_gamepad():
    """优先匹配假 PS2 接收器 by-id（序列号可能变化），再回退 by-path / js0。"""
    discovered = [
        p for p in sorted(glob.glob("/dev/input/by-id/usb-S_TGZ_Controller_*-joystick"))
        if not p.endswith("-event-joystick")
    ]
    return _first_existing(discovered + _GAMEPAD_CANDIDATES, "/dev/input/js0")


LZ_CAN_A_DEVICE  = resolve_device("lz_can_a")
LZ_CAN_B_DEVICE  = resolve_device("lz_can_b")
INCOS_CAN_DEVICE = resolve_device("incos_can")
EVO_CAN_DEVICE   = resolve_device("evo_can")
DM_CAN_DEVICE    = resolve_device("dm_can")
IMU_DEVICE       = resolve_device("imu")
TAIL_485_DEVICE  = resolve_device("tail_485")
MOUTH_DEVICE     = resolve_device("mouth_esp32")
GAMEPAD_DEVICE   = resolve_gamepad()

# 向后兼容 (旧代码中引用的名字)
LZ_CAN1_DEVICE   = LZ_CAN_A_DEVICE
EVO_CAN0_DEVICE  = EVO_CAN_DEVICE
LZ_SERIAL_DEVICE = LZ_CAN_B_DEVICE

BAUD = 921600
IMU_BAUD = 115200
