#!/usr/bin/env python3
"""USB 喇叭/声卡播放测试。

默认使用 bus_config.SPEAKER_ALSA_DEVICE，对当前机器是 USB2.0 Device:
  plughw:CARD=Device,DEV=0
"""

import argparse
import math
import os
import struct
import subprocess
import tempfile
import wave

from marsdog_control.config.bus_config import SPEAKER_ALSA_DEVICE
from marsdog_control.hardware.behavior.audio import BARK_WAV


def write_tone(path, freq_hz=880.0, duration_s=1.0, rate=44100, volume=0.35):
    frames = int(duration_s * rate)
    amp = int(32767 * volume)
    with wave.open(path, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(rate)
        for i in range(frames):
            sample = int(amp * math.sin(2.0 * math.pi * freq_hz * i / rate))
            w.writeframesraw(struct.pack("<h", sample))


def main():
    parser = argparse.ArgumentParser(description="播放狗叫/测试音确认喇叭通讯")
    parser.add_argument("--device", default=SPEAKER_ALSA_DEVICE,
                        help=f"ALSA 设备名，默认 {SPEAKER_ALSA_DEVICE}")
    parser.add_argument("--tone", action="store_true", help="播放正弦测试音而不是狗叫")
    parser.add_argument("--freq", type=float, default=880.0, help="测试音频率 Hz")
    parser.add_argument("--duration", type=float, default=1.0, help="播放时长秒")
    args = parser.parse_args()

    if not args.tone:
        if not os.path.exists(BARK_WAV):
            print(f"[speaker] 狗叫音频不存在: {BARK_WAV}")
            return 1
        print(f"[speaker] 播放狗叫 -> {args.device}")
        subprocess.run(["aplay", "-q", "-D", args.device, BARK_WAV], check=True)
        print("[speaker] 播放完成")
        return 0

    fd, path = tempfile.mkstemp(prefix="marsdog_speaker_", suffix=".wav")
    os.close(fd)
    try:
        write_tone(path, freq_hz=args.freq, duration_s=args.duration)
        print(f"[speaker] 播放 {args.freq:.0f}Hz / {args.duration:.1f}s -> {args.device}")
        subprocess.run(["aplay", "-q", "-D", args.device, path], check=True)
        print("[speaker] 播放完成")
    finally:
        try:
            os.remove(path)
        except OSError:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
