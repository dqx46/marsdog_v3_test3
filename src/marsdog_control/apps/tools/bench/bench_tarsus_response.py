#!/usr/bin/env python3
"""地面单侧 tarsus ≤1° 扫频启动器与日志分析器。"""

import argparse
import csv
import glob
import json
import math
import os
import statistics
import subprocess
import sys


HERE = os.path.dirname(os.path.abspath(__file__))


def percentile(values, q):
    values = sorted(values)
    if not values:
        return float("nan")
    return values[min(len(values) - 1, math.ceil(q * len(values)) - 1)]


def harmonic(samples, frequency):
    """返回信号的一次谐波幅值与相位；samples=(t,value)。"""
    mean = statistics.fmean(v for _, v in samples)
    sin_sum = cos_sum = 0.0
    for t, value in samples:
        centered = value - mean
        sin_sum += centered * math.sin(2.0 * math.pi * frequency * t)
        cos_sum += centered * math.cos(2.0 * math.pi * frequency * t)
    scale = 2.0 / len(samples)
    return scale * math.hypot(sin_sum, cos_sum), math.atan2(cos_sum, sin_sum)


def load_meta(log_path):
    meta_path = os.path.splitext(log_path)[0] + ".meta.json"
    with open(meta_path) as fh:
        return json.load(fh)


def analyze(log_path):
    meta = load_meta(log_path)
    args = meta["final_args"]
    side = args["bench_tarsus_side"]
    motor_id = 4 if side == "fl" else 8
    frequencies = [float(v) for v in args["bench_tarsus_frequencies"].split(",")]
    settle = max(0.5, float(args["bench_tarsus_settle_s"]))
    cycles = max(1.0, float(args["bench_tarsus_cycles"]))

    rows = []
    with open(log_path, newline="") as fh:
        for row in csv.DictReader(fh):
            if int(row["motor_id"]) == motor_id:
                rows.append(row)

    cursor = 0.0
    results = []
    for frequency in frequencies:
        start = cursor + settle
        end = start + cycles / frequency
        cursor = end
        segment = [r for r in rows if start <= float(r["t_s"]) < end]
        coverage = (
            float(segment[-1]["t_s"]) - float(segment[0]["t_s"])
            if len(segment) >= 2 else 0.0
        )
        if len(segment) < 10 or coverage < 0.90 * (cycles / frequency):
            results.append({
                "frequency_hz": frequency,
                "incomplete": True,
                "coverage_s": coverage,
                "expected_s": cycles / frequency,
            })
            continue
        command = [(float(r["t_s"]), float(r["command_deg"])) for r in segment]
        actual = [(float(r["t_s"]), float(r["actual_deg"])) for r in segment]
        cmd_amp, cmd_phase = harmonic(command, frequency)
        act_amp, act_phase = harmonic(actual, frequency)
        errors = [float(r["actual_deg"]) - float(r["command_deg"]) for r in segment]
        feedback_ages = [
            float(r["dm_feedback_age_ms"]) for r in segment
            if math.isfinite(float(r["dm_feedback_age_ms"]))
        ]
        torques = [abs(float(r["torque_nm"])) for r in segment]
        phase_deg = math.degrees(act_phase - cmd_phase)
        while phase_deg > 180.0:
            phase_deg -= 360.0
        while phase_deg < -180.0:
            phase_deg += 360.0
        results.append({
            "frequency_hz": frequency,
            "command_amp_deg": cmd_amp,
            "actual_amp_deg": act_amp,
            "gain": act_amp / cmd_amp if cmd_amp > 1e-6 else float("nan"),
            "phase_deg": phase_deg,
            "lag_ms": -phase_deg / 360.0 / frequency * 1000.0,
            "error_rms_deg": math.sqrt(statistics.fmean(e * e for e in errors)),
            "error_p95_deg": percentile([abs(e) for e in errors], 0.95),
            "feedback_age_p95_ms": percentile(feedback_ages, 0.95),
            "torque_peak_nm": max(torques),
            "torque_sat_pct": 100.0 * sum(t >= 4.8 for t in torques) / len(torques),
        })

    print(f"[分析] {os.path.basename(log_path)} side={side.upper()} ID={motor_id}")
    for result in results:
        if result.get("incomplete"):
            print(
                f"  {result['frequency_hz']:.2f}Hz: INCOMPLETE "
                f"({result['coverage_s']:.2f}/{result['expected_s']:.2f}s)，不参与调参"
            )
            continue
        print(
            f"  {result['frequency_hz']:.2f}Hz: "
            f"amp cmd/actual={result['command_amp_deg']:.2f}/"
            f"{result['actual_amp_deg']:.2f}°, gain={result['gain']:.3f}, "
            f"phase={result['phase_deg']:+.1f}°, lag={result['lag_ms']:+.1f}ms, "
            f"err RMS/P95={result['error_rms_deg']:.2f}/{result['error_p95_deg']:.2f}°, "
            f"feedback age P95={result['feedback_age_p95_ms']:.1f}ms, "
            f"tau peak/sat={result['torque_peak_nm']:.2f}Nm/{result['torque_sat_pct']:.1f}%"
        )
    return results


def latest_log():
    paths = glob.glob(os.path.join(HERE, "log", "walk_log_*.csv"))
    return max(paths, key=os.path.getmtime) if paths else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--analyze", metavar="CSV", help="只分析已有 walk CSV")
    parser.add_argument("--side", choices=("fl", "fr"))
    parser.add_argument("--amp-deg", type=float, default=2.0,
                        help="扫频半幅，默认±2°（仍属于地面小扰动）")
    parser.add_argument("--frequencies", default="0.25,0.5,1.0,2.0")
    parser.add_argument("--cycles", type=float, default=3.0)
    parser.add_argument("--kp", type=float, default=60.0,
                        help="被测侧KP，默认使用当前工作值60")
    parser.add_argument("--kd", type=float, default=3.0,
                        help="被测侧KD，默认使用当前工作值3")
    parser.add_argument("--yes", action="store_true")
    args = parser.parse_args()

    if args.analyze:
        analyze(os.path.abspath(args.analyze))
        return 0
    if not args.side:
        parser.error("--side 或 --analyze 必须指定一个")
    if not 0.0 < args.amp_deg <= 2.0:
        parser.error("--amp-deg 必须在 (0,2.0]°")
    if not args.yes:
        if not sys.stdin.isatty():
            print("[拒绝] 非交互启动必须加 --yes")
            return 2
        try:
            answer = input(
                "确认机器人四脚着地并自主承重站立、测试中不接触机身，"
                "旁边仅有人负责失稳时急停保护，且 tarsus 已归零？[y/N] "
            )
        except (KeyboardInterrupt, EOFError):
            print("\n已取消，未启动任何电机。")
            return 130
        if answer.strip().lower() not in ("y", "yes"):
            print("已取消，未启动任何电机。")
            return 1

    before = set(glob.glob(os.path.join(HERE, "log", "walk_log_*.csv")))
    command = [
        sys.executable, os.path.join(HERE, "walk.py"),
        "--natural-soft-trot", "--no-gamepad",
        "--bench-tarsus-side", args.side,
        "--bench-tarsus-amp-deg", str(args.amp_deg),
        "--bench-tarsus-frequencies", args.frequencies,
        "--bench-tarsus-cycles", str(args.cycles),
        f"--dm-kp-{args.side}", str(args.kp),
        f"--dm-kd-{args.side}", str(args.kd),
        "--no-imu", "--no-auto-trim",
        "--no-var-impedance", "--leg-kp-scale", "0.65",
        "--tarsus-lead-fl-ms", "0", "--tarsus-lead-fr-ms", "0",
    ]
    completed = subprocess.run(command, cwd=HERE, check=False)
    after = set(glob.glob(os.path.join(HERE, "log", "walk_log_*.csv")))
    new_logs = after - before
    log_path = max(new_logs, key=os.path.getmtime) if new_logs else latest_log()
    if log_path:
        analyze(log_path)
    return completed.returncode


if __name__ == "__main__":
    raise SystemExit(main())
