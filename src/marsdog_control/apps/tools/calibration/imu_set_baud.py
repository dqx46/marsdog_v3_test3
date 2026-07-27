#!/usr/bin/env python3
"""将 WT901/WT9011 系列 IMU 的串口波特率永久改写到 flash。

关键 bug 修复 (对照维特智能官方 WT9011G4K-Python SDK 协议确认):
  设备收到"写波特率寄存器 (0x04)"指令后会**立即**切换自己的 UART 速率，
  但如果紧接着还在*旧*波特率下发送 "save" (0x00) 指令，设备已经切到新
  速率监听，根本收不到那条 save，于是从未真正写入 flash —— 表现为：
  当前会话内看起来生效了 (立即用新波特率能读到数据)，但只要设备重新上电
  (拔插 / hub 掉线重连) 就会打回原来保存的波特率。

正确顺序:
  1) 在"旧"波特率下: 解锁 (FF AA 69 88 B5) → 写 BAUD 寄存器 (FF AA 04 xx 00)
  2) 关闭串口，用"新"波特率重新打开
  3) 在"新"波特率下发送 save (FF AA 00 00 00)  ← 这一步必须用新波特率
  4) 再次关闭重开，读数据验证

用法:
  python3 imu_set_baud.py /dev/marsdog_imu 115200
  python3 imu_set_baud.py /dev/ttyUSB3 115200 --retries 5
"""

import argparse
import sys
import time

import marsdog_control.apps.tools.misc.serial_fallback as serial

# WT901 BAUD 寄存器 (0x04) 取值表
_BAUD_CODE = {
    4800: 0x00, 9600: 0x01, 19200: 0x02, 38400: 0x03,
    57600: 0x04, 115200: 0x05, 230400: 0x06, 460800: 0x07, 921600: 0x08,
}
# 注: 不同批次固件寄存器编码略有差异, 经实测本机 IMU 用 0x06 = 115200
# (即某些固件把 0x05/0x06 对调了), 因此保留可覆盖选项
_BAUD_CODE_OVERRIDE = {115200: 0x06}

CMD_UNLOCK = bytes([0xFF, 0xAA, 0x69, 0x88, 0xB5])
CMD_SAVE = bytes([0xFF, 0xAA, 0x00, 0x00, 0x00])

_COMMON_BAUDS = [115200, 9600, 230400, 57600, 38400, 19200, 4800]


def _read_valid_frame(dev, baud, duration=0.6):
    """在给定波特率下尝试读到一个校验和正确的 0x55 帧, 返回 True/False。"""
    try:
        ser = serial.Serial(dev, baud, timeout=0.1)
    except (OSError, serial.SerialException):
        return False
    try:
        ser.reset_input_buffer()
        deadline = time.monotonic() + duration
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


def detect_current_baud(dev, candidates=_COMMON_BAUDS):
    """探测 IMU 当前实际使用的波特率。找不到返回 None。"""
    for baud in candidates:
        if _read_valid_frame(dev, baud, duration=0.4):
            return baud
    return None


def set_baud_persist(dev, target_baud, current_baud=None, verbose=True):
    """将 IMU 波特率永久改为 target_baud 并保存到 flash。

    返回 (success: bool, message: str)
    """
    if current_baud is None:
        current_baud = detect_current_baud(dev)
        if current_baud is None:
            return False, "无法探测到 IMU 当前波特率 (读不到有效 0x55 帧)"

    if verbose:
        print(f"[imu_baud] 当前波特率: {current_baud}")

    if current_baud == target_baud:
        if verbose:
            print(f"[imu_baud] 已经是目标波特率 {target_baud}, 仍重发一次 save 以确保持久化")

    code = _BAUD_CODE_OVERRIDE.get(target_baud, _BAUD_CODE.get(target_baud))
    if code is None:
        return False, f"不支持的目标波特率 {target_baud}"

    cmd_set_baud = bytes([0xFF, 0xAA, 0x04, code, 0x00])

    # 1) 旧波特率下: 解锁 + 写 BAUD 寄存器
    try:
        s = serial.Serial(dev, current_baud, timeout=0.3)
    except (OSError, serial.SerialException) as e:
        return False, f"打开串口失败 (@{current_baud}): {e}"
    try:
        s.write(CMD_UNLOCK)
        time.sleep(0.1)
        s.write(cmd_set_baud)
        time.sleep(0.1)
    finally:
        s.close()
    time.sleep(0.2)

    # 2) 用新波特率重新打开，发送 save —— 这是修复的关键一步
    try:
        s2 = serial.Serial(dev, target_baud, timeout=0.3)
    except (OSError, serial.SerialException) as e:
        return False, f"切换到新波特率 {target_baud} 后打开串口失败: {e}"
    try:
        time.sleep(0.05)
        s2.write(CMD_SAVE)
        time.sleep(0.15)
    finally:
        s2.close()
    time.sleep(0.2)

    # 3) 验证
    ok = _read_valid_frame(dev, target_baud, duration=0.6)
    if not ok:
        return False, f"切换后在 {target_baud} 读不到有效帧, 可能失败"

    still_old = _read_valid_frame(dev, current_baud, duration=0.3) if current_baud != target_baud else False
    if still_old:
        return False, f"{target_baud} 能收到数据, 但 {current_baud} 仍有输出, 可能没切干净"

    return True, f"已切换并保存为 {target_baud}，验证通过"


def set_baud_persist_with_retry(dev, target_baud, retries=3, verbose=True):
    """带重试的持久化设置 (应对 USB 总线瞬时掉线等偶发情况)。"""
    last_msg = ""
    for attempt in range(1, retries + 1):
        if verbose:
            print(f"[imu_baud] 第 {attempt}/{retries} 次尝试...")
        ok, msg = set_baud_persist(dev, target_baud, verbose=verbose)
        if verbose:
            print(f"[imu_baud]   -> {'成功' if ok else '失败'}: {msg}")
        if ok:
            return True, msg
        last_msg = msg
        time.sleep(0.5)
    return False, last_msg


def main():
    parser = argparse.ArgumentParser(description="永久修改 WT901/WT9011 IMU 波特率")
    parser.add_argument("device", help="IMU 串口设备路径, 如 /dev/marsdog_imu")
    parser.add_argument("baud", type=int, nargs="?", default=115200, help="目标波特率, 默认 115200")
    parser.add_argument("--retries", type=int, default=3, help="失败重试次数")
    args = parser.parse_args()

    ok, msg = set_baud_persist_with_retry(args.device, args.baud, retries=args.retries)
    print(("[成功] " if ok else "[失败] ") + msg)
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
