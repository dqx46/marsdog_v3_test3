#!/usr/bin/env python3
"""IMU/Tarsus 分阶段验收：安全的实时IMU检查与地面日志门禁。"""

import argparse
import csv
import glob
import math
import os

from marsdog_control.config.bus_config import IMU_BAUD, IMU_DEVICE
from marsdog_control.apps.tools.calibration.imu_set_rate import measure, percentile


HERE = os.path.dirname(os.path.abspath(__file__))


def finite(row, key):
    try:
        value = float(row[key])
        return value if math.isfinite(value) else None
    except (KeyError, TypeError, ValueError):
        return None


def validate_imu(duration):
    result = measure(IMU_DEVICE, IMU_BAUD, duration)
    passed = (
        95.0 <= result["angle_hz"] <= 105.0
        and result["angle_age_p95_ms"] < 15.0
    )
    print(
        f"[IMU] angle={result['angle_hz']:.1f}Hz, age median/P95="
        f"{result['angle_age_median_ms']:.1f}/{result['angle_age_p95_ms']:.1f}ms "
        f"→ {'PASS' if passed else 'FAIL'}"
    )
    return passed


def validate_log(path):
    with open(path, newline="") as fh:
        rows = list(csv.DictReader(fh))
    required = {
        "target_deg", "command_deg", "dm_feedback_age_ms", "dm_command_seq",
        "dm_feedback_seq", "torque_nm",
    }
    if not rows or not required.issubset(rows[0]):
        print("[LOG] FAIL：旧日志缺少新DM时序列，请用更新后的 walk.py 重新采集")
        return False

    def is_gait(row):
        if "gait_active" in row:
            return row["gait_active"].strip().lower() in ("1", "true", "yes")
        return row.get("mode", "") not in ("", "stand", "boot", "shutdown", "estop")

    gait_rows = [row for row in rows if is_gait(row)]
    modes = sorted({row.get("mode", "") for row in rows})
    print(f"[MODE] 全部模式={','.join(modes)}，步态行={len(gait_rows)}/{len(rows)}")
    if not gait_rows:
        print("[LOG] FAIL：日志中没有实际步态控制器样本，不能用站立误差评价行走跟踪")
        return False

    all_passed = True

    # 每个日志周期有多行电机记录，只取一种 motor_id 得到独立的IMU采样点。
    cycle_rows = [row for row in gait_rows if int(row["motor_id"]) == 4]
    angle_ages = [finite(row, "imu_angle_age_ms") for row in cycle_rows]
    gyro_ages = [finite(row, "imu_gyro_age_ms") for row in cycle_rows]
    angle_ages = [value for value in angle_ages if value is not None]
    gyro_ages = [value for value in gyro_ages if value is not None]
    angle_p95 = percentile(angle_ages, 0.95)
    gyro_p95 = percentile(gyro_ages, 0.95)
    imu_passed = angle_p95 < 15.0 and gyro_p95 < 15.0
    all_passed &= imu_passed
    print(
        f"[IMU日志] angle/gyro age P95={angle_p95:.1f}/{gyro_p95:.1f}ms "
        f"→ {'PASS' if imu_passed else 'FAIL'}"
    )

    for motor_id, label in ((4, "FL"), (8, "FR")):
        motor_rows = [row for row in gait_rows if int(row["motor_id"]) == motor_id]
        errors = []
        command_errors = []
        ages = []
        torques = []
        command_seq = []
        feedback_seq = []
        for row in motor_rows:
            actual = finite(row, "actual_deg")
            target = finite(row, "target_deg")
            command = finite(row, "command_deg")
            age = finite(row, "dm_feedback_age_ms")
            torque = finite(row, "torque_nm")
            if actual is not None and target is not None:
                errors.append(abs(actual - target))
            if actual is not None and command is not None:
                command_errors.append(abs(actual - command))
            if age is not None:
                ages.append(age)
            if torque is not None:
                torques.append(abs(torque))
            command_seq.append(int(row["dm_command_seq"]))
            feedback_seq.append(int(row["dm_feedback_seq"]))
        p95_error = percentile(errors, 0.95)
        p95_command_error = percentile(command_errors, 0.95)
        p95_age = percentile(ages, 0.95)
        saturation = (
            100.0 * sum(value >= 4.8 for value in torques) / len(torques)
            if torques else float("nan")
        )
        monotonic_seq = (
            all(b >= a for a, b in zip(command_seq, command_seq[1:]))
            and all(b >= a for a, b in zip(feedback_seq, feedback_seq[1:]))
        )
        passed = p95_error < 4.0 and p95_age < 30.0 and monotonic_seq
        all_passed &= passed
        print(
            f"[{label} tarsus] error P95={p95_error:.2f}°, "
            f"command error P95={p95_command_error:.2f}°, "
            f"feedback age P95={p95_age:.1f}ms, saturation={saturation:.1f}%, "
            f"seq={'OK' if monotonic_seq else 'BAD'} → {'PASS' if passed else 'FAIL'}"
        )

    anchor_rows = [row for row in gait_rows if int(row["motor_id"]) == 4]
    for leg in ("fl", "fr"):
        target_key = f"foot_pitch_target_{leg}_deg"
        actual_key = f"foot_pitch_actual_{leg}_deg"
        if target_key not in rows[0] or actual_key not in rows[0]:
            continue
        target_pitch = [finite(row, target_key) for row in anchor_rows]
        actual_pitch = [finite(row, actual_key) for row in anchor_rows]
        pairs = [(target, actual) for target, actual in zip(target_pitch, actual_pitch)
                 if target is not None and actual is not None]
        orientation_error = [abs(actual - target) for target, actual in pairs]
        rear_rate = (
            100.0 * sum(actual < -100.0 for _, actual in pairs) / len(pairs)
            if pairs else float("nan")
        )
        target_min = min((target for target, _ in pairs), default=float("nan"))
        pitch_p95 = percentile(orientation_error, 0.95)
        pitch_passed = target_min >= -90.5 and pitch_p95 < 5.0
        all_passed &= pitch_passed
        print(
            f"[{leg.upper()} foot pitch] target min={target_min:.1f}° "
            f"(-90°=朝下), error P95={pitch_p95:.2f}°, "
            f"actual<-100°={rear_rate:.1f}% "
            f"→ {'PASS' if pitch_passed else 'FAIL'}"
        )
    return all_passed


def print_sequence():
    root = HERE
    print("1) 电机不上电：")
    print(f'   cd "{root}" && python3 imu_set_rate.py --verify-persistent')
    print("2) 四脚着地自主承重、测试时不接触机身；旁边仅留急停保护人员：")
    print(f'   cd "{root}" && python3 bench_tarsus_response.py --side fl')
    print(f'   cd "{root}" && python3 bench_tarsus_response.py --side fr')
    print("3) 低速 NaturalSoftTrot 单变量对比（先0ms，再25/40/50ms）：")
    base = (
        f'cd "{root}" && python3 walk.py --natural-soft-trot '
        "--no-var-impedance --leg-kp-scale 0.65"
    )
    print("   " + base + " --tarsus-lead-fl-ms 0 --tarsus-lead-fr-ms 0")
    for lead in (25, 40, 50):
        print("   " + base + f" --tarsus-lead-fl-ms {lead} --tarsus-lead-fr-ms {lead}")
    print("4) 每次结束后验收最新日志：")
    print(f'   cd "{root}" && python3 validate_imu_tarsus.py --latest-log')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--imu", action="store_true", help="仅实时检查IMU（不启动电机）")
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--log")
    parser.add_argument("--latest-log", action="store_true")
    parser.add_argument("--print-sequence", action="store_true")
    args = parser.parse_args()

    if args.print_sequence:
        print_sequence()
    checks = []
    if args.imu:
        checks.append(validate_imu(max(1.0, args.duration)))
    log_path = args.log
    if args.latest_log:
        paths = glob.glob(os.path.join(HERE, "log", "walk_log_*.csv"))
        log_path = max(paths, key=os.path.getmtime) if paths else None
        if log_path is None:
            print("[LOG] FAIL：没有日志")
            checks.append(False)
    if log_path:
        print(f"[LOG] {os.path.abspath(log_path)}")
        checks.append(validate_log(os.path.abspath(log_path)))
    if not checks and not args.print_sequence:
        parser.error("请指定 --imu、--log、--latest-log 或 --print-sequence")
    return 0 if all(checks) else 2


if __name__ == "__main__":
    raise SystemExit(main())
