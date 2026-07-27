#!/usr/bin/env python3
"""头部/脖子四关节 Fourier 周期轨迹控制。

控制对象:
  ID 15 head_pitch (灵足 CAN-B)
  ID 16 head_yaw   (灵足 CAN-B)
  ID 17 head_roll  (灵足 CAN-A)
  ID 18 neck_pitch (泉智博 EVO)

运行逻辑:
  1. 读取同目录 fourier_fit.json（兼容当前 tab 缩进导出格式和标准 JSON）。
  2. 只初始化 ID 15/16/17/18，不使能同总线上的腿部或腰部电机。
  3. 从实机当前角度平滑过渡到 Fourier 轨迹起点，再连续周期运行。
  4. Ctrl+C/到达 --duration 后平滑回到启动角度，然后失能这四个电机。

使用方法:
  python3 head_neck_fourier_control.py                 # 仅预览轨迹范围
  python3 head_neck_fourier_control.py --execute       # 实机连续运行，Ctrl+C 停止
  python3 head_neck_fourier_control.py --execute --duration 15
  python3 head_neck_fourier_control.py --execute --amplitude-scale 0.5

注意:
  运行前确保 walk.py 等其他控制进程已经停止，并托住头部确认机械零位正确。
"""

import argparse
import json
import math
import os
import re
import sys
import threading
import time


THIS_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(THIS_DIR)
DRIVER_DIR = os.path.join(PROJECT_DIR, "mocap_to_real")
sys.path.insert(0, DRIVER_DIR)

from bus_config import BAUD, EVO_CAN_DEVICE, LZ_CAN_A_DEVICE, LZ_CAN_B_DEVICE
from can_serial import CanSerial
from joint_config import JOINT_BY_ID
from motor_evo import CMD_MOTOR_STATE, CMD_REST_STATE, MotorEvo, STATUS_PTM
from motor_lz_v2 import CAN_EFF_FLAG, MASTER_ID, MotorLz


MOTOR_TO_COLUMN = {
    15: "head_pitch_joint",
    16: "head_yaw_joint",
    17: "head_roll_joint",
    18: "neck_pitch_joint",
}
LZ_CAN_B_IDS = (15, 16)
LZ_CAN_A_IDS = (17,)
EVO_IDS = (18,)
ALL_IDS = (15, 16, 17, 18)

DEFAULT_GAINS = {
    15: (30.0, 3.0),
    16: (30.0, 3.0),
    17: (30.0, 3.0),
    18: (30.0, 5.0),
}


def parse_args():
    parser = argparse.ArgumentParser(
        description="用 Fourier 拟合轨迹周期控制头部 ID 15-17 和脖子 ID 18",
        allow_abbrev=False,
    )
    parser.add_argument(
        "--coefficients",
        default=os.path.join(THIS_DIR, "fourier_fit.json"),
        help="Fourier 系数文件（默认同目录 fourier_fit.json）",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="实际使能并控制电机；不加此参数时只做离线轨迹预览",
    )
    parser.add_argument("--duration", type=float, default=0.0,
                        help="周期运动时长(s)，0 表示运行到 Ctrl+C")
    parser.add_argument("--control-hz", type=float, default=100.0,
                        help="控制频率(Hz)，默认 100")
    parser.add_argument("--period", type=float, default=None,
                        help="覆盖拟合文件中的周期(s)")
    parser.add_argument("--amplitude-scale", type=float, default=1.0,
                        help="仅缩放 Fourier 交流分量，默认 1.0")
    parser.add_argument("--ramp-time", type=float, default=2.0,
                        help="启动和退出平滑过渡时间(s)，默认 2.0")
    parser.add_argument("--velocity-ff-scale", type=float, default=0.0,
                        help="解析速度前馈比例；默认 0 更保守，建议范围 0~1")
    parser.add_argument("--max-velocity", type=float, default=2.0,
                        help="速度前馈绝对限幅(rad/s)，默认 2.0")
    parser.add_argument("--max-error", type=float, default=0.60,
                        help="运行期最大允许位置跟踪误差(rad)，默认 0.60")
    parser.add_argument("--max-temperature", type=float, default=75.0,
                        help="最高允许电机温度(°C)，默认 75")
    return parser.parse_args()


def _number(text):
    """解析普通数字，也兼容文件中形如 '30.0JS:30' 的导出标记。"""
    match = re.match(r"\s*([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)", text)
    if not match:
        raise ValueError(f"不是有效数字: {text!r}")
    return float(match.group(1))


def _normalize_standard_json(data):
    columns = data.get("columns", {})
    normalized = {}
    for name in MOTOR_TO_COLUMN.values():
        item = columns.get(name)
        if not isinstance(item, dict):
            raise ValueError(f"系数文件缺少 columns.{name}")
        coeffs = item.get("coefficients", item)
        normalized[name] = {
            key: float(value)
            for key, value in coeffs.items()
            if re.fullmatch(r"(?:a0|[ab]\d+)", key)
        }
    return {
        "period": float(data.get("duration_s", 0.0)),
        "order": int(data.get("order", 0)),
        "columns": normalized,
    }


def _parse_tab_export(text):
    """解析当前 fourier_fit.json 的无括号、tab 缩进导出格式。"""
    names = set(MOTOR_TO_COLUMN.values())
    columns = {}
    current = None
    metadata = {}

    for raw_line in text.splitlines():
        line = raw_line.strip()
        if not line:
            continue
        fields = line.split(None, 1)
        key = fields[0]
        value = fields[1] if len(fields) == 2 else ""

        if key in names:
            current = key
            columns[current] = {}
        elif current is not None and re.fullmatch(r"(?:a0|[ab]\d+)", key):
            columns[current][key] = _number(value)
        elif key in {"duration_s", "order"}:
            metadata[key] = _number(value)

    missing = names.difference(columns)
    if missing:
        raise ValueError(f"tab 格式系数文件缺少列: {sorted(missing)}")
    return {
        "period": float(metadata.get("duration_s", 0.0)),
        "order": int(metadata.get("order", 0)),
        "columns": columns,
    }


def load_fourier(path):
    with open(path, "r", encoding="utf-8") as stream:
        text = stream.read()
    try:
        return _normalize_standard_json(json.loads(text))
    except json.JSONDecodeError:
        return _parse_tab_export(text)


def evaluate_fourier(coeffs, t, period, order, amplitude_scale):
    """返回 q(t) 与解析导数 dq(t)，单位分别为 rad 和 rad/s。"""
    omega = 2.0 * math.pi / period
    q = coeffs["a0"]
    dq = 0.0
    for k in range(1, order + 1):
        ak = coeffs.get(f"a{k}", 0.0)
        bk = coeffs.get(f"b{k}", 0.0)
        phase = k * omega * t
        q += amplitude_scale * (ak * math.cos(phase) + bk * math.sin(phase))
        dq += amplitude_scale * k * omega * (
            -ak * math.sin(phase) + bk * math.cos(phase)
        )
    return q, dq


def smoothstep5(u):
    """五次时间标度 s(u)，端点速度、加速度均为零。"""
    u = max(0.0, min(1.0, u))
    s = 10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5
    ds_du = 30.0 * u**2 - 60.0 * u**3 + 30.0 * u**4
    return s, ds_du


def clamp_target(motor_id, q):
    joint = JOINT_BY_ID[motor_id]
    return max(joint.limit_lo, min(joint.limit_hi, q))


class TargetedHardware:
    """仅打开并使能目标四电机，避免驱动默认初始化触碰同总线其他电机。"""

    def __init__(self):
        self.lz = MotorLz()
        self.evo = MotorEvo()
        self._opened_lz = False
        self._opened_evo = False

    def _enable_lz_one(self, motor_id, serial_obj, lock):
        self.lz.disable(motor_id, clear_fault=True)
        time.sleep(0.03)
        with lock:
            serial_obj.flush()
        ext_id = self.lz._build_ext_id(0x03, MASTER_ID, motor_id)
        with lock:
            serial_obj.send_msg(b"\x00" * 8, 8, ext_id)

        deadline = time.monotonic() + 0.15
        while time.monotonic() < deadline:
            with lock:
                result = serial_obj.read_msg()
            if result is None:
                time.sleep(0.001)
                continue
            can_id, _dlc, data = result
            if not (can_id & CAN_EFF_FLAG):
                continue
            eid = can_id & 0x1FFFFFFF
            response_id = (eid >> 8) & 0xFF
            if response_id == motor_id:
                self.lz._parse_feedback(motor_id, eid, data)
                break

        idx = motor_id - 1
        if self.lz.rx_count[idx] <= 0 or self.lz.mode[idx] != 2:
            raise RuntimeError(
                f"LZ Motor {motor_id} 使能失败: mode={self.lz.mode[idx]}, "
                f"fault={self.lz.fault[idx]}, rx={self.lz.rx_count[idx]}"
            )
        self.lz.is_connected[idx] = True
        self.lz._calc_pos_offset(motor_id)

    def _init_lz(self):
        self.lz._load_models()
        self.lz._serial_set = set(LZ_CAN_B_IDS)
        self.lz._can1_set = set(LZ_CAN_A_IDS)
        if not self.lz._serial.begin(LZ_CAN_B_DEVICE, BAUD):
            raise RuntimeError(f"无法打开灵足 CAN-B: {LZ_CAN_B_DEVICE}")
        self._opened_lz = True
        if not self.lz._can1_serial.begin(LZ_CAN_A_DEVICE, BAUD):
            raise RuntimeError(f"无法打开灵足 CAN-A: {LZ_CAN_A_DEVICE}")

        for motor_id in LZ_CAN_B_IDS:
            self._enable_lz_one(
                motor_id, self.lz._serial, self.lz._serial_lock
            )
        for motor_id in LZ_CAN_A_IDS:
            self._enable_lz_one(
                motor_id, self.lz._can1_serial, self.lz._can1_lock
            )
        self.lz._start_recv_thread()

    def _init_evo(self):
        self.evo._serial = CanSerial()
        if not self.evo._serial.begin(EVO_CAN_DEVICE, BAUD):
            self.evo._serial = None
            raise RuntimeError(f"无法打开 EVO CAN: {EVO_CAN_DEVICE}")
        self._opened_evo = True
        self.evo._use_serial = True
        self.evo._active_ids = list(EVO_IDS)

        # 只对 ID 18 发 Rest/Enable；不改变同总线 ID 9/12/19/20 的状态。
        self.evo.enter_rest_state(18)
        time.sleep(0.05)
        self.evo._running = True
        self.evo._thread = threading.Thread(
            target=self._evo_recv_loop, daemon=True
        )
        self.evo._thread.start()
        self.evo.enter_motor_state(18)

        deadline = time.monotonic() + 0.50
        while time.monotonic() < deadline:
            if (
                self.evo.is_connected[17]
                and self.evo.status[17] == STATUS_PTM
            ):
                return
            time.sleep(0.01)
        raise RuntimeError(
            f"EVO Motor 18 使能失败: status={self.evo.status[17]}, "
            f"fault={self.evo.fault[17]}"
        )

    def _evo_recv_loop(self):
        """仅轮询 ID 18；原驱动线程会轮询并改变同总线全部 EVO 电机状态。"""
        motor_id = 18
        idx = motor_id - 1
        poll_period = 0.005
        while self.evo._running:
            loop_start = time.monotonic()
            command = (
                CMD_MOTOR_STATE if self.evo._want_motor[idx]
                else CMD_REST_STATE
            )
            with self.evo._lock:
                self.evo._send_raw(
                    bytes([0xFF] * 7 + [command]), 8, motor_id
                )

            time.sleep(0.0015)
            found = False
            deadline = time.monotonic() + 0.002
            while time.monotonic() < deadline:
                with self.evo._lock:
                    result = self.evo._recv_raw()
                if result is None:
                    break
                can_id, _dlc, data = result
                if can_id & CAN_EFF_FLAG:
                    continue
                if (can_id & 0x7FF) == motor_id and len(data) >= 8:
                    self.evo._parse_feedback(motor_id, data)
                    found = True
                    break

            if found:
                self.evo._loss_count[idx] = 0
            else:
                self.evo._loss_count[idx] += 1
                if self.evo._loss_count[idx] >= 10:
                    self.evo.is_connected[idx] = False

            remaining = poll_period - (time.monotonic() - loop_start)
            if remaining > 0.0001:
                time.sleep(remaining)

    def initialize(self):
        self._init_lz()
        self._init_evo()
        time.sleep(0.10)
        return self.positions()

    def positions(self):
        return {
            15: self.lz.get_position(15),
            16: self.lz.get_position(16),
            17: self.lz.get_position(17),
            18: self.evo.get_position(18),
        }

    def command(self, targets, velocities):
        self.lz.mit_controls_serial(
            list(LZ_CAN_B_IDS),
            [targets[mid] for mid in LZ_CAN_B_IDS],
            [velocities[mid] for mid in LZ_CAN_B_IDS],
            [DEFAULT_GAINS[mid][0] for mid in LZ_CAN_B_IDS],
            [DEFAULT_GAINS[mid][1] for mid in LZ_CAN_B_IDS],
        )
        self.lz.mit_controls_can1(
            list(LZ_CAN_A_IDS),
            [targets[mid] for mid in LZ_CAN_A_IDS],
            [velocities[mid] for mid in LZ_CAN_A_IDS],
            [DEFAULT_GAINS[mid][0] for mid in LZ_CAN_A_IDS],
            [DEFAULT_GAINS[mid][1] for mid in LZ_CAN_A_IDS],
        )
        self.evo.ptm_controls(
            list(EVO_IDS),
            [targets[mid] for mid in EVO_IDS],
            [velocities[mid] for mid in EVO_IDS],
            [DEFAULT_GAINS[mid][0] for mid in EVO_IDS],
            [DEFAULT_GAINS[mid][1] for mid in EVO_IDS],
        )

    def check_safety(self, targets, max_error, max_temperature):
        actual = self.positions()
        for mid in ALL_IDS:
            if mid in LZ_CAN_A_IDS + LZ_CAN_B_IDS:
                idx = mid - 1
                fault = self.lz.fault[idx]
                temperature = self.lz.temperature[idx]
                enabled = self.lz.is_enabled[idx]
            else:
                idx = mid - 1
                fault = self.evo.fault[idx]
                temperature = self.evo.temperature[idx]
                enabled = self.evo.status[idx] == STATUS_PTM
            error = abs(targets[mid] - actual[mid])
            if not enabled:
                raise RuntimeError(f"Motor {mid} 运行中失能")
            if fault:
                raise RuntimeError(f"Motor {mid} fault={fault}")
            if temperature > max_temperature:
                raise RuntimeError(
                    f"Motor {mid} 温度 {temperature:.1f}°C 超过限制"
                )
            if error > max_error:
                raise RuntimeError(
                    f"Motor {mid} 跟踪误差 {error:.3f}rad 超过限制"
                )

    def close(self):
        # 只失能目标电机，不能调用驱动的 stop_all()。
        if self._opened_evo and self.evo._serial is not None:
            try:
                self.evo.enter_rest_state(18)
            except Exception:
                pass
            self.evo._running = False
            if self.evo._thread:
                self.evo._thread.join(timeout=1.0)
            self.evo._serial.end()
            self._opened_evo = False

        if self._opened_lz:
            self.lz.is_running = False
            for mid in LZ_CAN_B_IDS + LZ_CAN_A_IDS:
                try:
                    self.lz.disable(mid)
                    time.sleep(0.01)
                except Exception:
                    pass
            self.lz._serial.end()
            self.lz._can1_serial.end()
            self._opened_lz = False


def trajectory_sample(model, t, period, amplitude_scale):
    targets = {}
    velocities = {}
    for motor_id, column in MOTOR_TO_COLUMN.items():
        q, dq = evaluate_fourier(
            model["columns"][column],
            t,
            period,
            model["order"],
            amplitude_scale,
        )
        targets[motor_id] = clamp_target(motor_id, q)
        velocities[motor_id] = dq
    return targets, velocities


def preview(model, period, amplitude_scale):
    print(f"Fourier order={model['order']}, period={period:.3f}s, "
          f"amplitude_scale={amplitude_scale:.3f}")
    samples = 2000
    for mid in ALL_IDS:
        values = []
        peak_velocity = 0.0
        for i in range(samples):
            t = period * i / samples
            q, dq = evaluate_fourier(
                model["columns"][MOTOR_TO_COLUMN[mid]],
                t,
                period,
                model["order"],
                amplitude_scale,
            )
            values.append(q)
            peak_velocity = max(peak_velocity, abs(dq))
        joint = JOINT_BY_ID[mid]
        in_limit = min(values) >= joint.limit_lo and max(values) <= joint.limit_hi
        print(
            f"  ID {mid:2d} {joint.name:11s}: "
            f"[{min(values):+.3f}, {max(values):+.3f}] rad, "
            f"|dq|max={peak_velocity:.3f} rad/s, "
            f"limit=[{joint.limit_lo:+.3f}, {joint.limit_hi:+.3f}] "
            f"{'OK' if in_limit else '超限（实机将钳位）'}"
        )


def run_ramp(hardware, start, goal, duration, control_hz, velocity_scale,
             max_velocity):
    if duration <= 0.0:
        hardware.command(goal, {mid: 0.0 for mid in ALL_IDS})
        return
    dt = 1.0 / control_hz
    begin = time.monotonic()
    next_tick = begin
    while True:
        elapsed = time.monotonic() - begin
        u = min(1.0, elapsed / duration)
        s, ds_du = smoothstep5(u)
        targets = {
            mid: clamp_target(mid, start[mid] + s * (goal[mid] - start[mid]))
            for mid in ALL_IDS
        }
        velocities = {
            mid: max(
                -max_velocity,
                min(
                    max_velocity,
                    velocity_scale * ds_du / duration * (goal[mid] - start[mid]),
                ),
            )
            for mid in ALL_IDS
        }
        hardware.command(targets, velocities)
        if u >= 1.0:
            break
        next_tick += dt
        time.sleep(max(0.0, next_tick - time.monotonic()))


def main():
    args = parse_args()
    if args.control_hz <= 0.0:
        raise ValueError("--control-hz 必须大于 0")
    if args.ramp_time < 0.0 or args.amplitude_scale < 0.0:
        raise ValueError("--ramp-time 和 --amplitude-scale 不能为负数")
    if not 0.0 <= args.velocity_ff_scale <= 1.0:
        raise ValueError("--velocity-ff-scale 必须在 [0, 1] 内")

    model = load_fourier(args.coefficients)
    period = args.period if args.period is not None else model["period"]
    if period <= 0.0:
        raise ValueError("Fourier 周期必须大于 0")
    preview(model, period, args.amplitude_scale)

    if not args.execute:
        print("\n当前为预览模式，确认机械空间安全后加 --execute 才会使能电机。")
        return

    hardware = TargetedHardware()
    initial = None
    last_targets = None
    cycle_started = False
    try:
        print("\n[init] 仅初始化 Motor 15/16/17/18 ...")
        initial = hardware.initialize()
        for mid in ALL_IDS:
            print(f"  ID {mid:2d} {JOINT_BY_ID[mid].name:11s}: "
                  f"q0={initial[mid]:+.4f} rad")

        cycle_zero, _ = trajectory_sample(
            model, 0.0, period, args.amplitude_scale
        )
        print(f"[ramp] {args.ramp_time:.2f}s 平滑进入轨迹起点")
        run_ramp(
            hardware, initial, cycle_zero, args.ramp_time, args.control_hz,
            args.velocity_ff_scale, args.max_velocity,
        )

        print("[run] 周期运动开始；按 Ctrl+C 安全返回初始姿态")
        dt = 1.0 / args.control_hz
        begin = time.monotonic()
        next_tick = begin
        safety_counter = 0
        cycle_started = True
        while True:
            elapsed = time.monotonic() - begin
            if args.duration > 0.0 and elapsed >= args.duration:
                break
            targets, analytic_velocities = trajectory_sample(
                model, elapsed, period, args.amplitude_scale
            )
            velocities = {
                mid: max(
                    -args.max_velocity,
                    min(
                        args.max_velocity,
                        args.velocity_ff_scale * analytic_velocities[mid],
                    ),
                )
                for mid in ALL_IDS
            }
            hardware.command(targets, velocities)
            last_targets = targets

            safety_counter += 1
            if safety_counter >= max(1, int(args.control_hz / 10.0)):
                # 10 Hz 安全检查；留出启动后的一个短暂跟踪建立窗口。
                if elapsed > 0.5:
                    hardware.check_safety(
                        targets, args.max_error, args.max_temperature
                    )
                safety_counter = 0

            next_tick += dt
            time.sleep(max(0.0, next_tick - time.monotonic()))
    except KeyboardInterrupt:
        print("\n[stop] 收到 Ctrl+C")
    finally:
        if initial is not None:
            try:
                current_command = (
                    last_targets
                    if cycle_started and last_targets is not None
                    else hardware.positions()
                )
                print(f"[ramp] {args.ramp_time:.2f}s 平滑返回启动姿态")
                run_ramp(
                    hardware, current_command, initial, args.ramp_time,
                    args.control_hz, args.velocity_ff_scale,
                    args.max_velocity,
                )
            except Exception as error:
                print(f"[warning] 返回初始姿态失败，立即失能: {error}")
        hardware.close()
        if args.execute:
            print("[done] Motor 15/16/17/18 已失能")


if __name__ == "__main__":
    main()
