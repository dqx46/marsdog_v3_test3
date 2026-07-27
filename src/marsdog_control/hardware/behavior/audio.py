"""喇叭和狗头动作的轻量行为通道。

RT 触发时:
  1) 异步播放 sounds/bark.wav，不阻塞 200Hz 主控制循环；
  2) 通过 /dev/marsdog_mouth 发送协议 FE FE 01 触发张嘴/闭嘴动作。
"""

import os
import subprocess

# [解耦] 真实实现已下沉到此 src 模块; bark.wav 与 sounds/ 仍随部署留在 mocap_to_real,
# 故把 _DIR 锚定到 legacy 目录, 音频路径与原来逐字一致(不依赖本文件物理位置)。
from marsdog_control.compat import ensure_legacy_path as _ensure_legacy_path
from marsdog_control.compat import legacy_dir as _legacy_dir
_ensure_legacy_path()

from marsdog_control.config.bus_config import MOUTH_DEVICE, SPEAKER_ALSA_DEVICE

_DIR = str(_legacy_dir())
BARK_WAV = os.path.join(_DIR, "sounds", "bark.wav")
MOUTH_PACKET = bytes([0xFE, 0xFE, 0x01])
MOUTH_BAUD = 115200

_bark_proc = None


def play_bark():
    """异步播放狗叫音频；如果上一声还没播完，则跳过避免重叠。"""
    global _bark_proc
    if _bark_proc is not None and _bark_proc.poll() is None:
        return False
    if not os.path.exists(BARK_WAV):
        print(f"[bark] 音频文件不存在: {BARK_WAV}", flush=True)
        return False
    try:
        _bark_proc = subprocess.Popen(
            ["aplay", "-q", "-D", SPEAKER_ALSA_DEVICE, BARK_WAV],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        print("[bark] 汪!", flush=True)
        return True
    except Exception as e:
        print(f"[bark] 播放失败: {e}", flush=True)
        return False


def trigger_mouth():
    """给狗头控制板发送 FE FE 01。当前没有狗头时安全跳过。"""
    if not os.path.exists(MOUTH_DEVICE):
        print(f"[mouth] 未找到狗头控制板串口: {MOUTH_DEVICE}", flush=True)
        return False
    try:
        import serial
        with serial.Serial(MOUTH_DEVICE, MOUTH_BAUD, timeout=0.05) as ser:
            ser.write(MOUTH_PACKET)
            ser.flush()
        print(f"[mouth] FE FE 01 -> {MOUTH_DEVICE}", flush=True)
        return True
    except Exception as e:
        print(f"[mouth] 发送失败({MOUTH_DEVICE}): {e}", flush=True)
        return False


def bark_with_mouth():
    """播放狗叫，并尝试同步触发未来狗头动作。"""
    played = play_bark()
    trigger_mouth()
    return played
