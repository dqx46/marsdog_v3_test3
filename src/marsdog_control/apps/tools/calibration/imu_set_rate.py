#!/usr/bin/env python3
"""WT901 输出率独立配置/验收工具；本程序不会导入或启动任何电机。"""

import argparse
import math
import statistics
import sys
import time

import marsdog_control.apps.tools.misc.serial_fallback as serial

from marsdog_control.config.bus_config import IMU_BAUD, IMU_DEVICE
from marsdog_control.hardware.sensors.imu_wt901 import ImuWT901


# WT9011G4K RRATE(0x03) 回传速率码。2000Hz 是芯片内部采样上限, 串口回传由此寄存器
# 决定; 115200 波特率下最高稳跑 200Hz(0x0B), 且已覆盖 200Hz 控制环, 再高无收益。
RATE_CODES = {10: 0x06, 20: 0x07, 50: 0x08, 100: 0x09, 125: 0x0A, 200: 0x0B}
REG_SAVE = 0x00
REG_RSW = 0x02
REG_RRATE = 0x03
REG_KEY = 0x69
KEY_UNLOCK = 0xB588

# RSW(0x02) 输出内容位掩码: bit1=acc bit2=gyro bit3=angle bit4=mag ...
# 驱动只解析 acc/gyro/angle; 高回传率时裁成这三项以留足串口带宽。
RSW_ACC_GYRO_ANGLE = 0x0E
# 高于此频率时自动裁剪输出内容(除非 --keep-content)。
_TRIM_CONTENT_ABOVE_HZ = 100


def register_write(reg, value):
    return bytes((0xFF, 0xAA, reg & 0xFF, value & 0xFF, (value >> 8) & 0xFF))


def register_read(reg):
    return bytes((0xFF, 0xAA, 0x27, reg & 0xFF, (reg >> 8) & 0xFF))


def read_reg(port, baud, reg, timeout=0.6):
    """读取单个寄存器当前值(维特协议 0x55 0x5F 回帧, 4 个连续寄存器)。失败返回 None。"""
    with serial.Serial(port, baud, timeout=0.05, write_timeout=0.2) as ser:
        ser.reset_input_buffer()
        ser.write(register_read(reg))
        ser.flush()
        buf = bytearray()
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            chunk = ser.read(64)
            if chunk:
                buf.extend(chunk)
            while len(buf) >= 11:
                if buf[0] != 0x55 or buf[1] != 0x5F:
                    del buf[0]
                    continue
                frame = bytes(buf[:11])
                if (sum(frame[:10]) & 0xFF) != frame[10]:
                    del buf[0]
                    continue
                # frame[2:10] = 连续 4 个寄存器 (小端), reg 对应第 0 个
                return frame[2] | (frame[3] << 8)
            time.sleep(0.002)
    return None


def percentile(values, q):
    if not values:
        return float("nan")
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, math.ceil(q * len(ordered)) - 1))
    return ordered[index]


def measure(port, baud, duration):
    imu = ImuWT901(port, baud, angle_tau_s=0.0, gyro_tau_s=0.0)
    if not imu.begin():
        raise RuntimeError(f"无法从 {port} 收到有效 WT901 帧")
    start_counts = (
        imu.acc_update_count, imu.gyro_update_count, imu.angle_update_count)
    ages = []
    t0 = time.monotonic()
    try:
        while time.monotonic() - t0 < duration:
            age = imu.frame_ages()["angle"]
            if math.isfinite(age):
                ages.append(age * 1000.0)
            time.sleep(0.002)
        elapsed = time.monotonic() - t0
        end_counts = (
            imu.acc_update_count, imu.gyro_update_count, imu.angle_update_count)
    finally:
        imu.close()
    rates = tuple((end - start) / elapsed for start, end in zip(start_counts, end_counts))
    return {
        "acc_hz": rates[0],
        "gyro_hz": rates[1],
        "angle_hz": rates[2],
        "angle_age_median_ms": statistics.median(ages) if ages else float("nan"),
        "angle_age_p95_ms": percentile(ages, 0.95),
        "samples": len(ages),
    }


def write_rate(port, baud, rate_hz, trim_content=True):
    code = RATE_CODES[rate_hz]
    with serial.Serial(port, baud, timeout=0.05, write_timeout=0.2) as ser:
        ser.reset_input_buffer()
        ser.write(register_write(REG_KEY, KEY_UNLOCK))
        ser.flush()
        time.sleep(0.03)
        if trim_content:
            # 裁成 acc+gyro+angle, 给高回传率留足串口带宽。
            ser.write(register_write(REG_RSW, RSW_ACC_GYRO_ANGLE))
            ser.flush()
            time.sleep(0.03)
        ser.write(register_write(REG_RRATE, code))
        ser.flush()
        time.sleep(0.08)
        ser.write(register_write(REG_KEY, KEY_UNLOCK))
        ser.flush()
        time.sleep(0.03)
        ser.write(register_write(REG_SAVE, 0x0000))
        ser.flush()
        time.sleep(0.30)


def print_result(label, result):
    print(
        f"[{label}] acc/gyro/angle="
        f"{result['acc_hz']:.1f}/{result['gyro_hz']:.1f}/{result['angle_hz']:.1f}Hz, "
        f"angle age 中位/P95="
        f"{result['angle_age_median_ms']:.1f}/{result['angle_age_p95_ms']:.1f}ms"
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=IMU_DEVICE)
    parser.add_argument("--baud", type=int, default=IMU_BAUD)
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--set-rate", type=int, choices=sorted(RATE_CODES),
                        help="持久化设置输出率；不指定时仅只读检测")
    parser.add_argument("--keep-content", action="store_true",
                        help="设速率时不裁剪输出内容(默认高于100Hz会裁成acc+gyro+angle)")
    parser.add_argument("--expect", type=int, default=200, choices=sorted(RATE_CODES),
                        help="--verify-persistent 期望的持久化速率(默认200)")
    parser.add_argument("--yes", action="store_true",
                        help="跳过写入前人工确认（脚本/自动化使用）")
    parser.add_argument("--verify-persistent", action="store_true",
                        help="断电重启后仅复验；等价于只读检测并给出持久化结论")
    args = parser.parse_args()

    duration = max(1.0, args.duration)

    # 只读汇报当前寄存器, 直接看清"为什么是这个速率"。
    cur_rrate = read_reg(args.port, args.baud, REG_RRATE)
    cur_rsw = read_reg(args.port, args.baud, REG_RSW)
    if cur_rrate is not None:
        code2hz = {v: k for k, v in RATE_CODES.items()}
        hz = code2hz.get(cur_rrate & 0xFF, "?")
        print(f"[寄存器] RRATE(0x03)=0x{cur_rrate & 0xFF:02X} (~{hz}Hz)"
              + (f"  RSW(0x02)=0x{cur_rsw:04X}" if cur_rsw is not None else ""))

    before = measure(args.port, args.baud, duration)
    print_result("检测", before)

    if args.verify_persistent:
        tol = max(2.0, args.expect * 0.05)
        age_gate = max(15.0, 3000.0 / args.expect)
        ok = (abs(before["angle_hz"] - args.expect) <= tol
              and before["angle_age_p95_ms"] < age_gate)
        print(f"[持久化复验] 期望{args.expect}Hz → "
              + ("通过" if ok else "失败：禁止进入步态测试"))
        return 0 if ok else 2
    if args.set_rate is None:
        return 0

    trim = (not args.keep_content) and args.set_rate > _TRIM_CONTENT_ABOVE_HZ
    if not args.yes:
        if not sys.stdin.isatty():
            print("[拒绝] 非交互环境写入必须显式加 --yes")
            return 2
        answer = input(
            f"将 {args.port} 的 RRATE 持久化设为 {args.set_rate}Hz"
            + ("(并裁剪输出为acc+gyro+angle)" if trim else "")
            + "；确认电机程序未运行？[y/N] ")
        if answer.strip().lower() not in ("y", "yes"):
            print("已取消，未写入。")
            return 1

    print(f"[写入] KEY解锁 → "
          + (f"RSW=0x{RSW_ACC_GYRO_ANGLE:02X} → " if trim else "")
          + f"RRATE=0x{RATE_CODES[args.set_rate]:02X} → SAVE")
    write_rate(args.port, args.baud, args.set_rate, trim_content=trim)
    after = measure(args.port, args.baud, duration)
    print_result("写后复验", after)
    tolerance = max(2.0, args.set_rate * 0.05)
    ok = abs(after["angle_hz"] - args.set_rate) <= tolerance
    if not ok:
        print("[失败] 输出率未达到目标。可用 "
              f"`--set-rate 10` 回退原10Hz；禁止进入步态测试。")
        return 2
    age_gate = max(15.0, 3000.0 / args.set_rate)
    if after["angle_age_p95_ms"] >= age_gate:
        print(f"[失败] 虽为{args.set_rate}Hz但 angle age P95>={age_gate:.0f}ms；"
              "检查USB/CPU负载/波特率后再试。")
        return 2
    print("[通过] 请给 IMU 断电重启，再运行 "
          f"`... imu_set_rate --verify-persistent --expect {args.set_rate}`。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
