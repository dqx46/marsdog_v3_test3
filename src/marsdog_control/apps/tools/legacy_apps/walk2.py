#!/usr/bin/env python3
"""Marsdog walk.py — 最简步态控制脚本 (Phase 1 重写)

纯开环运动学步态, 无 IMU 闭环, 无速度前馈。
用于验证 IK 和基础 trot 轨迹是否正确。

用法:
    python3 walk.py                  # 站立模式启动
    python3 walk.py --trot           # 直接进入 trot 模式

手柄控制 (Xbox / PS2):
    左摇杆 Y    — 前进/后退
    右摇杆 X    — 转向 (Phase 4 实现)
    START       — 切换站立 / Trot
    SELECT / B  — 紧急停止并退出
    LB / RB     — 减慢 / 加快步频

键盘控制:
    SPACE / s   — 切换站立 / Trot
    + / =       — 加快步频
    - / _       — 减慢步频
    u / d       — 体高 +/- 1cm
    f / v       — 摆幅 +/- 5mm
    p           — 打印电机状态
    q / ESC     — 安全退出
"""

import argparse
import csv
import datetime
import math
import os
import select
import sys
import termios
import threading
import time
import tty
from marsdog_control.hardware.motors.lingzu import MotorLz
from marsdog_control.hardware.motors.evo import MotorEvo
from marsdog_control.motion.gait_controller import StandController, StableTrot, StablePace, _FRONT_HIP_OFFSET, _REAR_HIP_OFFSET
from marsdog_control.motion.kinematics import urdf_to_motor, ik_front_leg_2d, ik_rear_leg_2d
from marsdog_control.config.joints import (
    JOINT_MAP, JOINT_BY_ID, JOINT_BY_NAME as JBN,
    ALL_IDS, LZ_CAN_IDS, LZ_SERIAL_IDS, EVO_CAN_IDS,
    DEFAULT_LZ_KP, DEFAULT_LZ_KD,
    DEFAULT_EVO_KP, DEFAULT_EVO_KD,
)
from marsdog_control.config.bus_config import (LZ_CAN1_DEVICE, EVO_CAN0_DEVICE, LZ_SERIAL_DEVICE,
                        BAUD, IMU_DEVICE, IMU_BAUD, GAMEPAD_DEVICE)
from marsdog_control.hardware.sensors.imu_wt901 import ImuWT901
from marsdog_control.control.imu_balance import ImuAttitudeController
from marsdog_control.hardware.input.gamepad import Gamepad

# ─────────────────────────────────────────────────────────────────────────────
# 常量
# ─────────────────────────────────────────────────────────────────────────────
GP_DEADZONE       = 0.12
GP_TROT_THRESHOLD = 0.15
GP_PERIOD_STEP    = 0.05

CONTROL_HZ   = 200.0

JOINT_GAINS = {
    "fl_hip_pitch":  {"kp": 150.0, "kd": 5.0, "trq_ff": 0.0},
    "fr_hip_pitch":  {"kp": 150.0, "kd": 5.0, "trq_ff": 0.0},
    "fl_thigh_roll": {"kp": 80.0,  "kd": 5.0, "trq_ff": 0.0},
    "fr_thigh_roll": {"kp": 80.0,  "kd": 5.0, "trq_ff": 0.0},
    "fl_calf":       {"kp": 120.0, "kd": 5.0, "trq_ff": 0.35},
    "fr_calf":       {"kp": 120.0, "kd": 5.0, "trq_ff": 0.35},
    "rl_hip":        {"kp": 120.0, "kd": 10.0, "trq_ff": 0.0},  # EVO: KD_MAX=50
    "rr_hip":        {"kp": 120.0, "kd": 10.0, "trq_ff": 0.0},  # EVO: KD_MAX=50
    "rl_thigh":      {"kp": 140.0, "kd": 5.0, "trq_ff": 0.30},
    "rr_thigh":      {"kp": 140.0, "kd": 5.0, "trq_ff": 0.30},
    "rl_calf":       {"kp": 120.0, "kd": 5.0, "trq_ff": 0.45},
    "rr_calf":       {"kp": 120.0, "kd": 5.0, "trq_ff": 0.45},
    "head_pitch":    {"kp": 30.0,  "kd": 3.0, "trq_ff": 0.0},
    "head_yaw":      {"kp": 30.0,  "kd": 3.0, "trq_ff": 0.0},
    "head_roll":     {"kp": 30.0,  "kd": 3.0, "trq_ff": 0.0},
    "neck_pitch":    {"kp": 30.0,  "kd": 5.0, "trq_ff": 0.0},   # EVO: KD_MAX=50
    "waist_yaw":     {"kp": 50.0,  "kd": 5.0, "trq_ff": 0.0},   # EVO: KD_MAX=50
    "waist_pitch":   {"kp": 60.0,  "kd": 5.0, "trq_ff": 0.0},   # EVO: KD_MAX=50
    "waist_roll":    {"kp": 50.0,  "kd": 5.0, "trq_ff": 0.0},
}

# ─────────────────────────────────────────────────────────────────────────────
# 参数
# ─────────────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(description="Marsdog 稳定步态控制 (StableTrot)")
    p.add_argument("--height",      type=float, default=0.24)
    p.add_argument("--period",      type=float, default=0.75,
                   help="Trot 周期 (s), 默认 0.75")
    p.add_argument("--step-h",      type=float, default=0.020,
                   help="后腿抬腿高度 (m), 默认 2cm")
    p.add_argument("--step-h-front", type=float, default=None,
                   help="前腿抬腿高度 (m), 默认 = step_h * 0.75")
    p.add_argument("--amp-front",   type=float, default=0.018,
                   help="前腿半步长 (m), 默认 1.8cm")
    p.add_argument("--amp-rear",    type=float, default=0.022,
                   help="后腿半步长 (m), 默认 2.2cm")
    p.add_argument("--stance",      type=float, default=0.60,
                   help="支撑相比例, 默认 0.60")
    p.add_argument("--hip-abd",     type=float, default=0.08,
                   help="静态髋外展角 (rad), 默认 0.08 (~4.6°)")
    p.add_argument("--waist-pitch", type=float, default=0.05,
                   help="腰部弓背角度 (rad)")
    p.add_argument("--waist-yaw-offset", type=float, default=0.0)
    p.add_argument("--bwd-amp-scale", type=float, default=0.7,
                   help="后退步幅缩放系数, 默认 0.7")
    p.add_argument("--bwd-step-h",  type=float, default=0.015,
                   help="后退抬腿高度 (m), 默认 1.5cm")
    p.add_argument("--front-thrust-gain", type=float, default=1.0,
                   help="前腿推力增益: 放大大腿(hip_pitch)摆动, 小腿近似刚性只调高度 "
                        "(1.0=原全IK协调, >1=大腿出力更多; 默认 1.0 回退原版协调)")
    p.add_argument("--reactive-kp", type=float, default=0.0,
                   help="Raibert 反应式落脚点比例增益 (默认 0.0, 开启需确保仅作用于摆动腿)")
    p.add_argument("--reactive-kd", type=float, default=0.0,
                   help="Raibert 反应式落脚点微分增益 (默认 0.0, 避免gyro噪声导致抽搐)")
    p.add_argument("--bwd-period",  type=float, default=0.85,
                   help="后退步态周期 (s), 默认 0.85")
    p.add_argument("--lateral-sway", type=float, default=0.015,
                   help="横向重心摆动幅度 (m), 默认 15mm")
    p.add_argument("--pace-period", type=float, default=1.2,
                   help="Pace 步态周期 (s), 默认 1.2 (慢速保稳)")
    p.add_argument("--pace-stance", type=float, default=0.75,
                   help="Pace 步态站立比, 默认 0.75 (50%%双支撑)")
    p.add_argument("--pace-sway",   type=float, default=0.015,
                   help="Pace 步态横向重心摆动幅度 (m), 默认 15mm")
    p.add_argument("--pace-amp",    type=float, default=0.008,
                   help="Pace 步态前后步幅 (m), 默认 8mm (小步)")
    p.add_argument("--pace-step-h", type=float, default=0.015,
                   help="Pace 步态抬腿高度 (m), 默认 15mm")
    p.add_argument("--pace-hip-abd", type=float, default=0.00,
                   help="Pace 步态髋关节外展角 (rad), 默认 0.00")
    p.add_argument("--fade",        type=float, default=3.0)
    p.add_argument("--ramp",        type=float, default=2.0,
                   help="步态启动振幅斜坡时间 (s), 默认 2.0")
    p.add_argument("--trot",        action="store_true")
    p.add_argument("--no-gamepad",  action="store_true")
    p.add_argument("--no-log",      action="store_true")
    p.add_argument("--imu-test",    action="store_true",
                   help="IMU 补偿验证模式: 放大增益, stand 下倾斜可见腿部反应")
    return p.parse_args()


# ─────────────────────────────────────────────────────────────────────────────
# 键盘
# ─────────────────────────────────────────────────────────────────────────────

class KeyReader:
    def __init__(self):
        self._old = None
        self._enabled = False

    def start(self):
        try:
            self._old = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
            self._enabled = True
        except termios.error:
            self._enabled = False

    def stop(self):
        if self._old is not None:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old)
            except termios.error:
                pass
            self._old = None
            self._enabled = False

    def flush(self):
        if not self._enabled:
            return
        try:
            import termios as _t
            _t.tcflush(sys.stdin, _t.TCIFLUSH)
        except Exception:
            pass

    def get(self):
        if not self._enabled:
            return None
        try:
            r, _, _ = select.select([sys.stdin], [], [], 0)
            if not r:
                return None
            ch = sys.stdin.read(1)
            if ch == '\x1b':
                for _ in range(10):
                    r2, _, _ = select.select([sys.stdin], [], [], 0.02)
                    if not r2:
                        break
                    b = sys.stdin.read(1)
                    if b.isalpha() or b == '~':
                        break
                return None
            return ch
        except Exception:
            return None


# ─────────────────────────────────────────────────────────────────────────────
# 电机
# ─────────────────────────────────────────────────────────────────────────────

_CAN1_JOINTS   = [j for j in JOINT_MAP if j.mtype == "lz" and j.bus == "can1"]
_SERIAL_JOINTS = [j for j in JOINT_MAP if j.mtype == "lz" and j.bus == "serial"]
_EVO_JOINTS    = [j for j in JOINT_MAP if j.mtype == "evo"]


def _resolve_gains(j, kp_scale, use_joint_gains,
                   kp_lz, kd_lz, kp_evo, kd_evo):
    if use_joint_gains:
        g = JOINT_GAINS.get(j.name, {"kp": 30.0, "kd": 4.0, "trq_ff": 0.0})
        return g["kp"] * kp_scale, g["kd"], g["trq_ff"]
    kp = (kp_lz if j.mtype == "lz" else kp_evo) * kp_scale
    kd = kd_lz if j.mtype == "lz" else kd_evo
    return kp, kd, 0.0


def send_all(lz, evo, targets, kp_scale=1.0, use_joint_gains=True,
             kp_lz=DEFAULT_LZ_KP, kd_lz=DEFAULT_LZ_KD,
             kp_evo=DEFAULT_EVO_KP, kd_evo=DEFAULT_EVO_KD,
             velocities=None):
    can1_ids, can1_pos, can1_vel, can1_kp, can1_kd, can1_trq = [], [], [], [], [], []
    ser_ids,  ser_pos,  ser_vel,  ser_kp,  ser_kd,  ser_trq  = [], [], [], [], [], []
    evo_ids,  evo_pos,  evo_vel,  evo_kp,  evo_kd,  evo_trq  = [], [], [], [], [], []

    for j in _CAN1_JOINTS:
        mid = j.motor_id
        if mid not in targets:
            continue
        kp, kd, trq = _resolve_gains(j, kp_scale, use_joint_gains,
                                      kp_lz, kd_lz, kp_evo, kd_evo)
        can1_ids.append(mid);  can1_pos.append(targets[mid])
        can1_vel.append(velocities.get(mid, 0.0) if velocities else 0.0)
        can1_kp.append(kp);    can1_kd.append(kd);    can1_trq.append(trq)

    for j in _SERIAL_JOINTS:
        mid = j.motor_id
        if mid not in targets:
            continue
        kp, kd, trq = _resolve_gains(j, kp_scale, use_joint_gains,
                                      kp_lz, kd_lz, kp_evo, kd_evo)
        ser_ids.append(mid);   ser_pos.append(targets[mid])
        ser_vel.append(velocities.get(mid, 0.0) if velocities else 0.0)
        ser_kp.append(kp);     ser_kd.append(kd);     ser_trq.append(trq)

    for j in _EVO_JOINTS:
        mid = j.motor_id
        if mid not in targets:
            continue
        kp, kd, trq = _resolve_gains(j, kp_scale, use_joint_gains,
                                      kp_lz, kd_lz, kp_evo, kd_evo)
        evo_ids.append(mid);   evo_pos.append(targets[mid])
        evo_vel.append(velocities.get(mid, 0.0) if velocities else 0.0)
        evo_kp.append(kp);     evo_kd.append(kd);     evo_trq.append(trq)

    def _do_can1():
        if can1_ids:
            lz.mit_controls_can1(can1_ids, can1_pos, can1_vel,
                                 can1_kp, can1_kd, can1_trq)
    def _do_serial():
        if ser_ids:
            lz.mit_controls_serial(ser_ids, ser_pos, ser_vel,
                                   ser_kp, ser_kd, ser_trq)

    t1 = threading.Thread(target=_do_can1, daemon=True)
    t2 = threading.Thread(target=_do_serial, daemon=True)
    t1.start(); t2.start()
    if evo_ids:
        evo.ptm_controls(evo_ids, evo_pos, evo_vel,
                         evo_kp, evo_kd, evo_trq)
    t1.join(); t2.join()


def read_positions(lz, evo):
    time.sleep(0.3)
    pos = {}
    for j in JOINT_MAP:
        mid = j.motor_id
        if j.mtype == "lz":
            p = lz.get_position(mid)
        else:
            p = evo.get_position(mid)
        pos[mid] = p if p is not None else 0.0
    return pos


def check_motors(lz, evo, label=""):
    disabled = []
    lines = []
    for j in JOINT_MAP:
        mid = j.motor_id
        idx = mid - 1
        if j.mtype == "lz":
            en  = lz.is_enabled[idx]
            pos = math.degrees(lz.get_position(mid))
            fault = lz.fault[idx]
        else:
            en  = (evo.status[idx] == 0x02)
            pos = math.degrees(evo.get_position(mid))
            fault = evo.fault[idx]
        flag = "OK" if en else "!!"
        lines.append(f"  [{flag}] Motor {mid:2d} ({j.name:18s}) {pos:+8.2f}°"
                     + ("" if en else f"  fault={fault}"))
        if not en:
            disabled.append(j)
    print(f"\n── 电机状态 {label} ──")
    print("\n".join(lines))
    if disabled:
        print(f"  *** {len(disabled)} 个电机失能: "
              + ", ".join(f"Motor {j.motor_id}({j.name})" for j in disabled))
    return disabled


_stop = False

def smooth_transition(lz, evo, from_pos, to_pos, duration, label="fade"):
    global _stop
    steps = max(1, int(duration * CONTROL_HZ))
    t0 = time.monotonic()
    for step in range(steps + 1):
        if _stop:
            return False
        alpha = step / steps
        alpha = 3 * alpha * alpha - 2 * alpha * alpha * alpha
        kp_s = 0.3 + 0.7 * alpha
        cur = {}
        for mid in set(from_pos) | set(to_pos):
            a = from_pos.get(mid, 0.0)
            b = to_pos.get(mid, 0.0)
            cur[mid] = a + (b - a) * alpha
        send_all(lz, evo, cur, use_joint_gains=True, kp_scale=kp_s)
        pct = int(alpha * 100)
        sys.stdout.write(f"\r  [{label}] {pct:3d}%  step {step}/{steps}   ")
        sys.stdout.flush()
        next_t = t0 + (step + 1) / CONTROL_HZ
        sleep_t = next_t - time.monotonic()
        if sleep_t > 0:
            time.sleep(sleep_t)
    sys.stdout.write("\n")
    return True


# ─────────────────────────────────────────────────────────────────────────────
# 日志
# ─────────────────────────────────────────────────────────────────────────────

def setup_log(enabled):
    if not enabled:
        return None, None, None
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        f"walk_log_{ts}.csv")
    f = open(path, "w", newline="")
    w = csv.writer(f)
    w.writerow([
        "t_s", "mode", "dt_ms", "throttle",
        "height_m", "period_s", "amp_front_cm", "amp_rear_cm",
        "phase_fl", "phase_fr", "phase_rl", "phase_rr",
        "imu_roll_deg", "imu_pitch_deg", "imu_yaw_deg",
        "imu_gyro_roll", "imu_gyro_pitch",
        "imu_dz_fl_mm", "imu_dz_fr_mm", "imu_dz_rl_mm", "imu_dz_rr_mm",
        "imu_roll_p_mm", "imu_roll_d_mm", "imu_ramp_frac",
        "reactive_deg", "lateral_sway_mm", "expected_roll",
        "motor_id", "name", "target_deg", "actual_deg", "error_deg",
    ])
    print(f"[log] CSV: {os.path.basename(path)}")
    print(f"[log] 路径: {path}")
    return f, w, path


def write_log(writer, t_s, mode, lz, evo, targets, dt_ms,
              trot, throttle, imu=None, imu_dz=None,
              imu_ctrl=None, ramp_frac=0.0):
    if writer is None:
        return

    if trot:
        height = trot.body_height
        period = trot.period
        amp_f = trot.amp_front * 100
        amp_r = trot.amp_rear * 100
    else:
        height = period = amp_f = amp_r = 0.0

    legs = ['fl', 'fr', 'rl', 'rr']
    phases = {}
    if trot:
        for leg in legs:
            phases[leg] = (t_s / trot.period + trot._PHASE_OFFSET[leg]) % 1.0
    else:
        for leg in legs:
            phases[leg] = 0.0

    if imu and imu.connected:
        imu_roll  = math.degrees(imu.roll)
        imu_pitch = math.degrees(imu.pitch)
        imu_yaw   = math.degrees(imu.yaw)
        imu_gr    = math.degrees(imu.gyro_roll)
        imu_gp    = math.degrees(imu.gyro_pitch)
    else:
        imu_roll = imu_pitch = imu_yaw = imu_gr = imu_gp = float('nan')

    dz = imu_dz if imu_dz else {'fl': 0, 'fr': 0, 'rl': 0, 'rr': 0}

    reactive_deg = 0.0
    lateral_mm = 0.0
    expected_roll = 0.0
    if trot and hasattr(trot, '_reactive_filtered'):
        reactive_deg = math.degrees(trot._reactive_filtered)
    if trot and hasattr(trot, 'get_expected_roll'):
        expected_roll = trot.get_expected_roll(t_s)
    if trot and hasattr(trot, '_lateral_offset'):
        lateral_mm = trot._lateral_offset(t_s) * 1000

    for j in JOINT_MAP:
        mid = j.motor_id
        tgt = targets.get(mid, float('nan'))
        if j.mtype == "lz":
            act = lz.get_position(mid)
        else:
            act = evo.get_position(mid)
        act = act if act is not None else float('nan')
        err_val = math.degrees(act - tgt) if (not math.isnan(tgt) and not math.isnan(act)) else float('nan')
        writer.writerow([
            f"{t_s:.4f}", mode, f"{dt_ms:.2f}", f"{throttle:.3f}",
            f"{height:.4f}", f"{period:.3f}", f"{amp_f:.2f}", f"{amp_r:.2f}",
            f"{phases['fl']:.4f}", f"{phases['fr']:.4f}",
            f"{phases['rl']:.4f}", f"{phases['rr']:.4f}",
            f"{imu_roll:.2f}", f"{imu_pitch:.2f}", f"{imu_yaw:.1f}",
            f"{imu_gr:.1f}", f"{imu_gp:.1f}",
            f"{dz['fl']*1000:.2f}", f"{dz['fr']*1000:.2f}",
            f"{dz['rl']*1000:.2f}", f"{dz['rr']*1000:.2f}",
            f"{imu_ctrl.roll_out*1000:.3f}" if imu_ctrl else "0",
            f"{imu_ctrl.pitch_out*1000:.3f}" if imu_ctrl else "0",
            f"{ramp_frac:.3f}",
            f"{reactive_deg:.2f}", f"{lateral_mm:.2f}", f"{expected_roll:.2f}",
            mid, j.name,
            f"{math.degrees(tgt):.3f}" if not math.isnan(tgt) else "nan",
            f"{math.degrees(act):.3f}" if not math.isnan(act) else "nan",
            f"{err_val:.3f}" if not math.isnan(err_val) else "nan",
        ])


# ─────────────────────────────────────────────────────────────────────────────
# 主函数
# ─────────────────────────────────────────────────────────────────────────────

def main():
    global _stop
    args = parse_args()

    sh_f = args.step_h_front if args.step_h_front else args.step_h * 0.75
    print(f"\n{'='*62}")
    print(f"  Marsdog Walk — StableTrot (Bezier Z + 分段半余弦 X)")
    print(f"  体高={args.height:.3f}m  周期={args.period:.2f}s  "
          f"stance={args.stance:.0%}")
    print(f"  步高 前={sh_f*100:.1f}cm  后={args.step_h*100:.1f}cm")
    print(f"  摆幅 前=±{args.amp_front*100:.1f}cm  "
          f"后=±{args.amp_rear*100:.1f}cm")
    print(f"{'='*62}\n")

    # ── IMU 初始化（连接但不校准，校准在站立后做）─────────────────────────
    imu_ok = False
    for imu_port in [IMU_DEVICE]:
        imu = ImuWT901(imu_port, IMU_BAUD)
        if imu.begin():
            print(f"[IMU] 已连接 {imu_port}, 帧数={imu.update_count}")
            imu_ok = True
            break
    if not imu_ok:
        print("[IMU] 未连接, 日志中 IMU 列将为 NaN")

    # ── 电机初始化 ─────────────────────────────────────────────────────────
    lz  = MotorLz()
    evo = MotorEvo()

    print(f"[init] 1/3 灵足 Serial ({LZ_SERIAL_DEVICE})...")
    lz.init_serial(LZ_SERIAL_DEVICE, BAUD)
    print(f"[init] 2/3 灵足 CAN1   ({LZ_CAN1_DEVICE})...")
    lz.init_can1_serial(LZ_CAN1_DEVICE, BAUD)
    print(f"[init] 3/3 泉智博 CAN0 ({EVO_CAN0_DEVICE})...")
    evo.init_serial(EVO_CAN0_DEVICE, BAUD)

    for j in JOINT_MAP:
        if j.mtype == "lz" and lz.is_connected[j.motor_id - 1]:
            lz.enable(j.motor_id)
            time.sleep(0.002)
    time.sleep(0.05)

    for attempt in range(5):
        not_enabled = []
        for j in JOINT_MAP:
            if j.mtype == "evo" and evo.is_connected[j.motor_id - 1]:
                idx = j.motor_id - 1
                if evo.status[idx] != 0x02:
                    not_enabled.append(j)
                    evo.enter_motor_state(j.motor_id)
                    time.sleep(0.005)
        if not not_enabled:
            break
        time.sleep(0.05)

    time.sleep(0.4)
    online = []
    for j in JOINT_MAP:
        mid = j.motor_id
        if j.mtype == "lz":
            conn = lz.is_connected[mid - 1]
        else:
            conn = evo.is_connected[mid - 1]
        if conn:
            online.append(mid)
        else:
            print(f"  [WARNING] Motor {mid} ({j.name}) 离线")

    if not online:
        print("[ERROR] 无在线电机"); lz.end(); evo.end(); return

    print(f"\n[online] {len(online)}/{len(ALL_IDS)} 电机在线\n")

    # ── 读取当前位置 ───────────────────────────────────────────────────────
    print("[pos] 读取当前位置...")
    cur_pos = read_positions(lz, evo)

    # ── 日志 ──────────────────────────────────────────────────────────────
    log_file, log_writer, log_path = setup_log(not args.no_log)

    # ── 步态控制器 ────────────────────────────────────────────────────────
    stand = StandController(body_height=args.height, hip_abduction=args.hip_abd)
    stand.waist_pitch_offset = args.waist_pitch
    stand.waist_yaw_offset = args.waist_yaw_offset

    trot_fwd = StableTrot(
        body_height       = args.height,
        amp_front         = args.amp_front,
        amp_rear          = args.amp_rear,
        step_height       = args.step_h,
        step_height_front = args.step_h_front,
        period            = args.period,
        stance_ratio      = args.stance,
        hip_abduction     = args.hip_abd,
        ramp_duration     = args.ramp,
        reactive_kp       = args.reactive_kp,
        reactive_kd       = args.reactive_kd,
        lateral_sway      = args.lateral_sway,
        front_thrust_gain = args.front_thrust_gain,
    )
    trot_fwd.waist_pitch_offset = args.waist_pitch
    trot_fwd.waist_yaw_offset = args.waist_yaw_offset

    bwd_scale = args.bwd_amp_scale
    trot_bwd = StableTrot(
        body_height       = args.height,
        amp_front         = -args.amp_rear  * bwd_scale,
        amp_rear          = -args.amp_front * bwd_scale,
        step_height       = args.bwd_step_h,
        step_height_front = args.bwd_step_h * 0.75,
        period            = args.bwd_period,
        stance_ratio      = args.stance,
        hip_abduction     = args.hip_abd + 0.01,
        ramp_duration     = args.ramp,
        reactive_kp       = args.reactive_kp,
        reactive_kd       = args.reactive_kd,
        lateral_sway      = args.lateral_sway,
        front_thrust_gain = args.front_thrust_gain,
    )
    trot_bwd.waist_pitch_offset = args.waist_pitch
    trot_bwd.waist_yaw_offset = args.waist_yaw_offset

    pace_fwd = StablePace(
        body_height       = args.height,
        amp_front         = args.pace_amp,
        amp_rear          = args.pace_amp,
        step_height       = args.pace_step_h,
        step_height_front = args.pace_step_h,
        period            = args.pace_period,
        stance_ratio      = args.pace_stance,
        hip_abduction     = args.pace_hip_abd,
        ramp_duration     = args.ramp,
        reactive_kp       = args.reactive_kp,
        reactive_kd       = args.reactive_kd,
        lateral_sway      = args.pace_sway,
    )
    pace_fwd.waist_pitch_offset = args.waist_pitch
    pace_fwd.waist_yaw_offset = args.waist_yaw_offset

    pace_bwd = StablePace(
        body_height       = args.height,
        amp_front         = -args.pace_amp,
        amp_rear          = -args.pace_amp,
        step_height       = args.pace_step_h,
        step_height_front = args.pace_step_h,
        period            = args.pace_period,
        stance_ratio      = args.pace_stance,
        hip_abduction     = args.pace_hip_abd,
        ramp_duration     = args.ramp,
        reactive_kp       = args.reactive_kp,
        reactive_kd       = args.reactive_kd,
        lateral_sway      = args.pace_sway,
    )
    pace_bwd.waist_pitch_offset = args.waist_pitch
    pace_bwd.waist_yaw_offset = args.waist_yaw_offset

    # ── IMU 闭环控制器 — 标准 PID + 泄漏积分 ──────────────────────────────
    #   P: 即时恢复力; I: 泄漏积分消稳态误差; D: gyro阻尼(硬限幅防噪声抽搐)
    #   死区/阻尼均平滑过渡(C0连续), 杜绝死区边界阶跃引起的高频震荡。
    if args.imu_test:
        # IMU验证模式 — 更强更快, 便于抬狗时肉眼看清补偿方向
        imu_ctrl = ImuAttitudeController(
            kp_roll  = 0.06,
            kp_pitch = 0.05,
            ki_roll  = 0.001,
            ki_pitch = 0.001,
            kd_roll  = 0.002,
            kd_pitch = 0.002,
            decay_rate     = 0.990,
            max_correction = 0.030,
            deadzone_deg   = 1.0,
            fall_guard_deg = 35.0,
        )
        print("[IMU-TEST] 标准PID: kp=0.06/0.05, ki=0.001, kd=0.002, max=30mm")
    else:
        # 正常模式: 标准PID, 平稳有阻尼
        imu_ctrl = ImuAttitudeController(
            kp_roll  = 0.03,
            kp_pitch = 0.03,
            ki_roll  = 0.0005,
            ki_pitch = 0.0005,
            kd_roll  = 0.002,
            kd_pitch = 0.002,
            decay_rate     = 0.995,
            max_correction = 0.020,
            deadzone_deg   = 1.5,
            fall_guard_deg = 22.0,
        )

    # ── 过渡到站立 ────────────────────────────────────────────────────────
    stand_pos = stand.get_targets(0)
    print(f"[fade] 过渡到站立 ({args.fade:.1f}s)...")
    ok = smooth_transition(lz, evo, cur_pos, stand_pos, args.fade, label="stand")
    if not ok:
        lz.end(); evo.end(); return
    print("[ok] 已站立\n")

    # ── IMU 校准（站立后做，确保零位准确）─────────────────────────────────
    if imu_ok:
        print("[IMU] 站立后校准...")
        imu.calibrate(1.5)
        imu_ctrl.enable()
        print(f"[IMU] 闭环已启用 (Roll+Pitch 速度形): {imu_ctrl.describe()}")
    else:
        print("[IMU] 闭环未启用 (IMU 未连接)")

    mode = "trot_fwd" if args.trot else "stand"
    t_gait = time.time()
    height = args.height
    throttle = 0.0
    active_trot = trot_fwd if mode == "trot_fwd" else None

    # ── 步态切换位置混合 (参考 mydog_ref gait_blend) ──────────────────────
    _BLEND_DURATION = 0.5  # 增加到 0.5s，让步态和站立切换更平滑
    _blend_active = False
    _blend_start = 0.0
    _blend_from = {}

    # ── 键盘 + 手柄 ──────────────────────────────────────────────────────
    kb = KeyReader()
    kb.start()
    kb.flush()

    gp = None
    gp_ly_offset = 0.0
    if not args.no_gamepad:
        gp = Gamepad(device=GAMEPAD_DEVICE)
        if not gp.connected:
            print("[gamepad] 未找到手柄, 仅键盘控制")
            gp = None
        else:
            print("[gamepad] 已连接, 校准摇杆零位(请松开摇杆)...")
            time.sleep(0.5)
            samples = []
            for _ in range(20):
                samples.append(gp.get_state().ly)
                time.sleep(0.025)
            gp_ly_offset = sum(samples) / len(samples)
            print(f"[gamepad] 零位偏移: ly={gp_ly_offset:+.3f} (已自动补偿)")

    _gp_start_prev = False
    _gp_lb_prev = False
    _gp_rb_prev = False

    print("─" * 52)
    if gp:
        print("  手柄: 左摇杆Y=前进/后退  START=切模式  LB/RB=步频")
        print("        SELECT/B=紧急退出")
    print("  键盘: SPACE/s=切换  +/-=步频  u/d=体高  f/v=摆幅")
    print("        p=状态  q/ESC=退出")
    print("─" * 52 + "\n")

    def _start_trot(trot_ctrl, targets_now, blend_time=0.5):
        """统一的步态启动: 启动位置混合 (IMU状态继承, 不reset)。"""
        nonlocal t_gait, active_trot, _blend_active, _blend_start, _blend_from, _BLEND_DURATION
        t_gait = time.time()
        active_trot = trot_ctrl
        active_trot.set_height(height)
        trot_ctrl._reactive_filtered = 0.0
        _smooth_tgt.clear()
        if targets_now:
            _blend_from = dict(targets_now)
            _blend_start = time.time()
            _BLEND_DURATION = blend_time
            _blend_active = True

    if active_trot:
        print(f"[gait] 已进入 Trot  {active_trot.describe()}")

    t_next_print = time.time()
    t_next_status = time.time() + 10.0
    log_cycle = 0
    LOG_INTERVAL = 5
    targets = stand.get_targets(0)
    prev_targets = {}
    prev_t = time.time()
    _RATELIMIT_IDS = {2, 5, 7, 10}  # fl/fr_thigh_roll, rl/rr_hip
    _smooth_tgt = {}

    try:
        while True:
            t_loop = time.time()

            # ── 手柄 ──────────────────────────────────────────────────
            if gp and gp.connected:
                st = gp.get_state()

                if st.select or st.b:
                    print("\n[gamepad] 紧急停止")
                    break

                if st.start and not _gp_start_prev:
                    if mode == "stand":
                        mode = "trot_fwd"
                        _start_trot(trot_fwd, targets, blend_time=0.6)
                        print(f"\r[gamepad] -> TROT_FWD (ramp={args.ramp:.1f}s)  ")
                    else:
                        mode = "stand"
                        active_trot = None
                        _blend_from = dict(targets)
                        _blend_start = time.time()
                        _BLEND_DURATION = 0.6
                        _blend_active = True
                        stand.set_height(height)
                        print(f"\r[gamepad] -> STAND  ")
                _gp_start_prev = st.start

                if st.lb and not _gp_lb_prev:
                    p = (active_trot.period if active_trot else trot_fwd.period)
                    p = min(2.0, p + GP_PERIOD_STEP)
                    trot_fwd.set_period(p)
                    trot_bwd.set_period(p)
                    pace_fwd.set_period(p)
                    pace_bwd.set_period(p)
                    print(f"\r[gamepad] LB: period={p:.2f}s  ")
                _gp_lb_prev = st.lb

                if st.rb and not _gp_rb_prev:
                    p = (active_trot.period if active_trot else trot_fwd.period)
                    p = max(0.25, p - GP_PERIOD_STEP)
                    trot_fwd.set_period(p)
                    trot_bwd.set_period(p)
                    pace_fwd.set_period(p)
                    pace_bwd.set_period(p)
                    print(f"\r[gamepad] RB: period={p:.2f}s  ")
                _gp_rb_prev = st.rb

                vx = -(st.ly - gp_ly_offset)
                rx = -st.rx if abs(st.rx) > GP_DEADZONE else 0.0
                has_walk = abs(vx) > GP_TROT_THRESHOLD
                has_pace = st.dpad_up or st.dpad_down

                if has_pace:
                    throttle = 1.0 if st.dpad_up else -1.0
                    new_mode = "pace_fwd" if st.dpad_up else "pace_bwd"
                    if mode != new_mode:
                        new_trot = pace_fwd if new_mode == "pace_fwd" else pace_bwd
                        _start_trot(new_trot, targets, blend_time=0.4)
                        mode = new_mode
                elif has_walk:
                    throttle = vx
                    new_mode = "trot_fwd" if vx > 0 else "trot_bwd"
                    if mode != new_mode:
                        new_trot = trot_fwd if new_mode == "trot_fwd" else trot_bwd
                        # 从站立切换到运动给 0.6s，运动间切换给 0.3s
                        b_time = 0.6 if mode == "stand" else 0.3
                        _start_trot(new_trot, targets, blend_time=b_time)
                        mode = new_mode

                    if mode == "trot_fwd":
                        trot_fwd.amp_front = args.amp_front
                        trot_fwd.amp_rear = args.amp_rear
                        trot_fwd.turn_cmd = rx
                    else:
                        trot_bwd.amp_front = -args.amp_rear * args.bwd_amp_scale
                        trot_bwd.amp_rear = -args.amp_front * args.bwd_amp_scale
                        trot_bwd.turn_cmd = rx
                else:
                    throttle = 0.0
                    # 如果只有rx摇杆(原地转向)
                    if abs(rx) > GP_DEADZONE:
                        if mode != "trot_fwd":
                            b_time = 0.6 if mode == "stand" else 0.3
                            _start_trot(trot_fwd, targets, blend_time=b_time)
                            mode = "trot_fwd"
                            trot_fwd.amp_front = 0.0
                            trot_fwd.amp_rear = 0.0
                        trot_fwd.turn_cmd = rx
                    else:
                        if mode == "trot_fwd":
                            trot_fwd.turn_cmd = 0.0
                            trot_fwd.amp_front = args.amp_front
                            trot_fwd.amp_rear = args.amp_rear
                        if mode != "stand":
                            mode = "stand"
                            active_trot = None
                            # 恢复到站立时给 0.6s 的过渡
                            _blend_from = dict(targets)
                            _blend_start = time.time()
                            _BLEND_DURATION = 0.6
                            _blend_active = True
                            stand.set_height(height)

            # ── 键盘 ──────────────────────────────────────────────────
            key = kb.get()
            if key in ('q', 'Q', '\x03'):
                print(f"\n[quit] key='{key}'")
                break

            elif key in ('p', 'P'):
                check_motors(lz, evo, label=f"t={time.time()-t_gait:.1f}s")

            elif key in (' ', 's', 'S'):
                if mode == "stand":
                    mode = "trot_fwd"
                    _start_trot(trot_fwd, targets, blend_time=0.6)
                    print(f"\r[gait] -> TROT_FWD  ramp={args.ramp:.1f}s  {active_trot.describe()}")
                else:
                    mode = "stand"
                    active_trot = None
                    _blend_from = dict(targets)
                    _blend_start = time.time()
                    _BLEND_DURATION = 0.6
                    _blend_active = True
                    stand.set_height(height)
                    print(f"\r[gait] -> STAND  height={height:.3f}m")

            elif key in ('+', '='):
                p = max(0.25, trot_fwd.period - 0.05)
                trot_fwd.set_period(p)
                trot_bwd.set_period(p)
                pace_fwd.set_period(p)
                pace_bwd.set_period(p)
                print(f"\r[gait] period={p:.2f}s")

            elif key in ('-', '_'):
                p = min(2.0, trot_fwd.period + 0.05)
                trot_fwd.set_period(p)
                trot_bwd.set_period(p)
                pace_fwd.set_period(p)
                pace_bwd.set_period(p)
                print(f"\r[gait] period={p:.2f}s")

            elif key == 'u':
                height = min(0.30, height + 0.01)
                stand.set_height(height)
                trot_fwd.set_height(height)
                trot_bwd.set_height(height)
                pace_fwd.set_height(height)
                pace_bwd.set_height(height)
                print(f"\r[gait] height={height:.3f}m")

            elif key == 'd':
                height = max(0.15, height - 0.01)
                stand.set_height(height)
                trot_fwd.set_height(height)
                trot_bwd.set_height(height)
                pace_fwd.set_height(height)
                pace_bwd.set_height(height)
                print(f"\r[gait] height={height:.3f}m")

            elif key == 'f':
                trot_fwd.amp_front = min(0.06, trot_fwd.amp_front + 0.005)
                trot_fwd.amp_rear  = min(0.06, trot_fwd.amp_rear + 0.005)
                print(f"\r[gait] fwd amp F=±{trot_fwd.amp_front*100:.1f}cm  R=±{trot_fwd.amp_rear*100:.1f}cm")

            elif key == 'v':
                trot_fwd.amp_front = max(0.005, trot_fwd.amp_front - 0.005)
                trot_fwd.amp_rear  = max(0.005, trot_fwd.amp_rear - 0.005)
                print(f"\r[gait] fwd amp F=±{trot_fwd.amp_front*100:.1f}cm  R=±{trot_fwd.amp_rear*100:.1f}cm")

            # ── IMU 闭环 ─────────────────────────────────────────────
            imu_dz = None
            eff_roll_rad = 0.0
            eff_pitch_rad = 0.0
            if imu and imu.connected:
                eff_roll_rad = imu.roll
                eff_pitch_rad = imu.pitch
                # 注意: 不再扣除 get_expected_roll 前馈。
                # 日志实测该前馈对前进步态相位反了(corr=-0.54), 相减反而把误差放大1.6x,
                # 自激出极限环震荡; 对后退也仅微弱有益。模型相位修对前暂时关闭(仍记录到日志列)。

            # D项软启动: 步态前3秒逐渐引入gyro阻尼，避免冲击噪声激励
            _d_ramp_dur = 3.0
            if active_trot:
                _t_gait_rel = time.time() - t_gait
                d_ramp = min(1.0, _t_gait_rel / _d_ramp_dur)
            else:
                d_ramp = 1.0
            gyro_r_damped = imu.gyro_roll * d_ramp if (imu and imu.connected) else 0.0
            gyro_p_damped = imu.gyro_pitch * d_ramp if (imu and imu.connected) else 0.0

            if imu_ctrl.enabled and imu.connected:
                imu_dz = imu_ctrl.update(
                    eff_roll_rad, eff_pitch_rad,
                    gyro_r_damped, gyro_p_damped)

            # ── 计算目标 ──────────────────────────────────────────────
            imu_state = None
            if imu and imu.connected:
                imu_state = {'roll': eff_roll_rad, 'gyro_roll': gyro_r_damped}

            if active_trot:
                t_rel = time.time() - t_gait
                targets = active_trot.get_targets(t_rel, imu_dz=imu_dz,
                                                  imu_state=imu_state)
            else:
                targets = stand.get_targets(0)
                # Stand 模式也应用 IMU Z补偿 (用于验证IMU方向正确性)
                if imu_dz:
                    _h = stand.body_height
                    _xf = stand.x_offset_front
                    _xr = stand.x_offset_rear
                    _zf = -(_h - _FRONT_HIP_OFFSET)
                    _zr = -(_h - _REAR_HIP_OFFSET)
                    for leg, zb, xc, ik_fn, hp_name, ca_name in [
                        ('fl', _zf, _xf, ik_front_leg_2d, 'fl_hip_pitch', 'fl_calf'),
                        ('fr', _zf, _xf, ik_front_leg_2d, 'fr_hip_pitch', 'fr_calf'),
                        ('rl', _zr, _xr, ik_rear_leg_2d, 'rl_thigh', 'rl_calf'),
                        ('rr', _zr, _xr, ik_rear_leg_2d, 'rr_thigh', 'rr_calf'),
                    ]:
                        dz = imu_dz.get(leg, 0.0)
                        if abs(dz) > 0.0001:
                            z_new = zb + dz
                            hp_u, ca_u = ik_fn(xc, z_new)
                            j_hp = JBN[hp_name]
                            j_ca = JBN[ca_name]
                            targets[j_hp.motor_id] = max(j_hp.limit_lo, min(j_hp.limit_hi,
                                urdf_to_motor(j_hp, hp_u)))
                            targets[j_ca.motor_id] = max(j_ca.limit_lo, min(j_ca.limit_hi,
                                urdf_to_motor(j_ca, ca_u)))

            for mid in online:
                if mid not in targets:
                    targets[mid] = cur_pos.get(mid, 0.0)

            # ── 步态切换位置混合 (参考 mydog_ref gait_blend smoothstep) ──
            if _blend_active:
                elapsed = time.time() - _blend_start
                if elapsed >= _BLEND_DURATION:
                    _blend_active = False
                else:
                    s = elapsed / _BLEND_DURATION
                    s = s * s * (3.0 - 2.0 * s)
                    for mid in _blend_from:
                        if mid in targets:
                            targets[mid] = _blend_from[mid] * (1.0 - s) + targets[mid] * s

            # ── roll/hip 目标速率限制 (抗 stick-slip) ─────────────────
            # Pace 步态需要更快的响应，提高限制到 200°/s
            if active_trot and isinstance(active_trot, StablePace):
                _RL_MAX = 0.0175      # 1.0°/step ≈ 200°/s @200Hz
            else:
                _RL_MAX = 0.0087      # 0.5°/step ≈ 100°/s @200Hz
            for mid in _RATELIMIT_IDS:
                if mid in targets:
                    if mid in _smooth_tgt:
                        delta = targets[mid] - _smooth_tgt[mid]
                        if delta >  _RL_MAX: delta =  _RL_MAX
                        if delta < -_RL_MAX: delta = -_RL_MAX
                        targets[mid] = _smooth_tgt[mid] + delta
                    _smooth_tgt[mid] = targets[mid]

            # ── EVO 掉线重新使能 ──────────────────────────────────────
            for j in JOINT_MAP:
                if j.mtype == "evo" and j.motor_id in online:
                    idx = j.motor_id - 1
                    if evo.is_connected[idx] and evo.status[idx] != 0x02:
                        evo.enter_motor_state(j.motor_id)

            # ── 速度前馈 ────────────────────────────────────────────────
            now_t = time.time()
            ctrl_dt = now_t - prev_t
            if ctrl_dt < 0.001:
                ctrl_dt = 0.001
            velocities = {}
            if prev_targets:
                for mid in targets:
                    if mid in prev_targets:
                        velocities[mid] = (targets[mid] - prev_targets[mid]) / ctrl_dt
            prev_targets = dict(targets)
            prev_t = now_t

            # ── 发送 ──────────────────────────────────────────────────
            send_all(lz, evo, targets, use_joint_gains=True,
                     velocities=velocities)

            # ── 日志 ──────────────────────────────────────────────────
            log_cycle += 1
            dt = time.time() - t_loop
            if log_writer and (log_cycle % LOG_INTERVAL == 0):
                t_rel_log = time.time() - t_gait if active_trot else 0.0
                ramp_f = 0.0
                if active_trot and hasattr(active_trot, 'ramp_duration'):
                    rf = t_rel_log / active_trot.ramp_duration if active_trot.ramp_duration > 0 else 1.0
                    ramp_f = min(1.0, max(0.0, 3*rf*rf - 2*rf*rf*rf))
                write_log(log_writer, t_rel_log, mode, lz, evo, targets,
                          dt * 1000.0, active_trot,
                          throttle, imu, imu_dz,
                          imu_ctrl=imu_ctrl, ramp_frac=ramp_f)

            # ── 状态显示 ──────────────────────────────────────────────
            now = time.time()
            if now >= t_next_print:
                if active_trot:
                    tag = "FWD" if mode == "trot_fwd" else "BWD"
                    sys.stdout.write(
                        f"\r  [TROT_{tag}]  h={height:.3f}m  T={active_trot.period:.2f}s  "
                        f"amp=±{abs(active_trot.amp_front)*100:.1f}/{abs(active_trot.amp_rear)*100:.1f}cm      "
                    )
                else:
                    _roll_deg = math.degrees(imu.roll) if (imu and imu.connected) else 0.0
                    _dz_str = ""
                    if imu_dz:
                        _dz_str = f" dZ: FL{imu_dz.get('fl',0)*1000:+.1f} FR{imu_dz.get('fr',0)*1000:+.1f} RL{imu_dz.get('rl',0)*1000:+.1f} RR{imu_dz.get('rr',0)*1000:+.1f}mm"
                    sys.stdout.write(
                        f"\r  [STAND]  h={height:.3f}m  roll={_roll_deg:+.1f}°{_dz_str}        "
                    )
                sys.stdout.flush()
                t_next_print = now + 0.4

            # ── 定时状态检查 ──────────────────────────────────────────
            if now >= t_next_status:
                disabled_list = []
                for j in JOINT_MAP:
                    mid = j.motor_id
                    idx = mid - 1
                    if j.mtype == "lz":
                        en = lz.is_enabled[idx]
                    else:
                        en = (evo.status[idx] == 0x02)
                    if not en:
                        disabled_list.append(j)
                if disabled_list:
                    print(f"\n  [!] disabled: "
                          + ", ".join(f"M{j.motor_id}({j.name})" for j in disabled_list))
                t_next_status = now + 10.0

            # ── 频率 ──────────────────────────────────────────────────
            elapsed = time.time() - t_loop
            sleep_t = 1.0 / CONTROL_HZ - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    except KeyboardInterrupt:
        pass
    finally:
        kb.stop()
        if gp:
            gp.close()
        _stop = False

        cur2 = {}
        for j in JOINT_MAP:
            mid = j.motor_id
            if j.mtype == "lz":
                p = lz.get_position(mid)
            else:
                p = evo.get_position(mid)
            cur2[mid] = p if p is not None else stand.get_targets(0).get(mid, 0.0)

        print("\n\n[cleanup] 回站立 (1.5s)...")
        stand_final = stand.get_targets(0)
        smooth_transition(lz, evo, cur2, stand_final, 1.5, label="return")

        DISABLE_SECS  = 8.0
        DISABLE_STEPS = int(DISABLE_SECS * CONTROL_HZ)
        print(f"[cleanup] {DISABLE_SECS:.0f}s 缓速失能...")
        t0_disable = time.monotonic()
        for step in range(DISABLE_STEPS + 1):
            alpha = step / DISABLE_STEPS
            # 从全局 10 降到 0
            global_kp = 10.0 * (1.0 - alpha)
            global_kd = 0.5 * (1.0 - alpha)  # KD 也同比例降低
            send_all(lz, evo, stand_final,
                     use_joint_gains=False,
                     kp_lz=global_kp, kd_lz=global_kd,
                     kp_evo=global_kp, kd_evo=global_kd)
            sys.stdout.write(f"\r  [disable] {int(alpha*100):3d}%  "
                             f"kp={global_kp:.2f}   ")
            sys.stdout.flush()
            next_t = t0_disable + (step + 1) / CONTROL_HZ
            sleep_t = next_t - time.monotonic()
            if sleep_t > 0:
                time.sleep(sleep_t)
        print()

        print("[cleanup] 失能电机...")
        for j in JOINT_MAP:
            if j.mtype == "lz":
                lz.disable(j.motor_id)
            else:
                evo.enter_rest_state(j.motor_id)
            time.sleep(0.002)
        for _ in range(3):
            time.sleep(0.05)
            for j in JOINT_MAP:
                if j.mtype == "evo":
                    evo.enter_rest_state(j.motor_id)
                    time.sleep(0.002)

        lz.end()
        evo.end()
        if imu.connected:
            imu.close()
        if log_file:
            log_file.close()
            if log_path:
                print(f"[log] 已保存: {os.path.basename(log_path)}")
                print(f"[log] 路径: {log_path}")
        print("[cleanup] 完成。")


if __name__ == "__main__":
    main()
