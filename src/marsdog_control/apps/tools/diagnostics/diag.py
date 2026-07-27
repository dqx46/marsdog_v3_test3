#!/usr/bin/env python3
"""Marsdog 诊断工具 — 逐项验证 IK、坐标系、关节符号。

在真机上用键盘交互式测试各个子系统，确保每一步都正确后再进入步态控制。

用法:
    python3 diag.py

按键:
    0       — 回到标准站立 (0.24m)
    1 / 2   — 重心左移 / 右移 2cm
    3 / 4   — 重心前移 / 后移 2cm
    5 / 6 / 7 / 8 — 抬 FL / FR / RL / RR 腿 3cm
    9       — 当前选中腿画圆 (按一次开始, 再按停止)
    w / s   — 弓背 / 塌腰 (waist_pitch +/- 5度)
    u / d   — 体高 +/- 1cm
    p       — 打印电机状态
    q / ESC — 安全退出
"""

import math
import os
import select
import sys
import termios
import time
import tty
from marsdog_control.hardware.motors.lingzu import MotorLz
from marsdog_control.hardware.motors.evo import MotorEvo
from marsdog_control.config.joints import (
    JOINT_MAP, JOINT_BY_ID, JOINT_BY_NAME,
    ALL_IDS, DEFAULT_LZ_KP, DEFAULT_LZ_KD,
    DEFAULT_EVO_KP, DEFAULT_EVO_KD,
)
from marsdog_control.config.bus_config import LZ_CAN1_DEVICE, EVO_CAN0_DEVICE, LZ_SERIAL_DEVICE, BAUD
from marsdog_control.motion.kinematics import (
    ik_front_leg_2d, ik_rear_leg_2d,
    fk_front_2d, fk_rear_2d,
    urdf_to_motor, WAIST_Z, FL_HIP_Z, RL_HIP_Z,
)

CONTROL_HZ = 100.0
SEND_INTERVAL = 0.0004

FRONT_HIP_OFFSET = abs(WAIST_Z + FL_HIP_Z)  # 0.031 m
REAR_HIP_OFFSET = abs(RL_HIP_Z)              # 0.015 m

JOINT_GAINS = {
    "fl_hip_pitch":  {"kp": 60.0, "kd": 5.0, "trq_ff": 0.0},
    "fr_hip_pitch":  {"kp": 60.0, "kd": 5.0, "trq_ff": 0.0},
    "fl_thigh_roll": {"kp": 40.0, "kd": 5.0, "trq_ff": 0.0},
    "fr_thigh_roll": {"kp": 40.0, "kd": 5.0, "trq_ff": 0.0},
    "fl_calf":       {"kp": 45.0, "kd": 4.0, "trq_ff": 0.35},
    "fr_calf":       {"kp": 45.0, "kd": 4.0, "trq_ff": 0.35},
    "rl_hip":        {"kp": 40.0, "kd": 5.0, "trq_ff": 0.0},
    "rr_hip":        {"kp": 40.0, "kd": 5.0, "trq_ff": 0.0},
    "rl_thigh":      {"kp": 60.0, "kd": 4.0, "trq_ff": 0.30},
    "rr_thigh":      {"kp": 60.0, "kd": 4.0, "trq_ff": 0.30},
    "rl_calf":       {"kp": 50.0, "kd": 4.0, "trq_ff": 0.45},
    "rr_calf":       {"kp": 50.0, "kd": 4.0, "trq_ff": 0.45},
    "head_pitch":    {"kp": 30.0, "kd": 4.0, "trq_ff": 0.0},
    "head_yaw":      {"kp": 30.0, "kd": 4.0, "trq_ff": 0.0},
    "head_roll":     {"kp": 30.0, "kd": 4.0, "trq_ff": 0.0},
    "neck_pitch":    {"kp": 30.0, "kd": 4.0, "trq_ff": 0.0},
    "waist_yaw":     {"kp": 30.0, "kd": 4.0, "trq_ff": 0.0},
    "waist_pitch":   {"kp": 35.0, "kd": 4.0, "trq_ff": 0.0},
    "waist_roll":    {"kp": 30.0, "kd": 4.0, "trq_ff": 0.0},
}

# ─────────────────────────────────────────────────────────────────────────────
# 工具函数
# ─────────────────────────────────────────────────────────────────────────────

def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _cmd(jname, urdf_angle):
    """URDF 角度 → (motor_id, clamped_motor_angle)"""
    j = JOINT_BY_NAME[jname]
    m = urdf_to_motor(j, urdf_angle)
    return j.motor_id, _clamp(m, j.limit_lo, j.limit_hi)


def compute_stand_targets(body_height, x_front=0.0, x_rear=0.0,
                          waist_pitch=0.0, hip_abd=0.04):
    """计算站立姿态的全部 19 个关节目标 (motor 空间 rad)。"""
    z_front = -(body_height - FRONT_HIP_OFFSET)
    z_rear = -(body_height - REAR_HIP_OFFSET)

    fl_hip, fl_calf = ik_front_leg_2d(x_front, z_front)
    rl_thigh, rl_calf = ik_rear_leg_2d(x_rear, z_rear)

    targets = {}

    # 前左腿
    mid, cmd = _cmd('fl_hip_pitch', fl_hip)
    targets[mid] = cmd
    mid, cmd = _cmd('fl_thigh_roll', hip_abd)
    targets[mid] = cmd
    mid, cmd = _cmd('fl_calf', fl_calf)
    targets[mid] = cmd

    # 前右腿
    mid, cmd = _cmd('fr_hip_pitch', fl_hip)
    targets[mid] = cmd
    mid, cmd = _cmd('fr_thigh_roll', -hip_abd)
    targets[mid] = cmd
    mid, cmd = _cmd('fr_calf', fl_calf)
    targets[mid] = cmd

    # 后左腿
    mid, cmd = _cmd('rl_hip', hip_abd)
    targets[mid] = cmd
    mid, cmd = _cmd('rl_thigh', rl_thigh)
    targets[mid] = cmd
    mid, cmd = _cmd('rl_calf', rl_calf)
    targets[mid] = cmd

    # 后右腿
    mid, cmd = _cmd('rr_hip', -hip_abd)
    targets[mid] = cmd
    mid, cmd = _cmd('rr_thigh', rl_thigh)
    targets[mid] = cmd
    mid, cmd = _cmd('rr_calf', rl_calf)
    targets[mid] = cmd

    # 头/颈/腰
    for name in ('head_pitch', 'head_yaw', 'head_roll', 'neck_pitch',
                 'waist_yaw', 'waist_roll'):
        mid, cmd = _cmd(name, 0.0)
        targets[mid] = cmd
    mid, cmd = _cmd('waist_pitch', waist_pitch)
    targets[mid] = cmd

    return targets


def compute_leg_lifted_targets(base_targets, body_height, leg,
                               lift_height=0.03):
    """在 base_targets 基础上, 将指定腿抬高 lift_height 米。"""
    targets = dict(base_targets)

    z_front = -(body_height - FRONT_HIP_OFFSET)
    z_rear = -(body_height - REAR_HIP_OFFSET)

    if leg in ('fl', 'fr'):
        z_lifted = z_front + lift_height
        hip_u, calf_u = ik_front_leg_2d(0.0, z_lifted)
        mid_hp, cmd_hp = _cmd(f'{leg}_hip_pitch', hip_u)
        mid_ca, cmd_ca = _cmd(f'{leg}_calf', calf_u)
        targets[mid_hp] = cmd_hp
        targets[mid_ca] = cmd_ca
    else:
        z_lifted = z_rear + lift_height
        th_u, ca_u = ik_rear_leg_2d(0.0, z_lifted)
        mid_th, cmd_th = _cmd(f'{leg}_thigh', th_u)
        mid_ca, cmd_ca = _cmd(f'{leg}_calf', ca_u)
        targets[mid_th] = cmd_th
        targets[mid_ca] = cmd_ca

    return targets


def compute_shifted_targets(body_height, dx=0.0, dy=0.0,
                            waist_pitch=0.0, hip_abd=0.04):
    """重心偏移: dx=前后(+前), dy=左右(+左)。

    实现方式: 足端位置不动, 身体平移 → 等效于足端反向偏移。
    横向(dy): 通过调整 thigh_roll / hip_roll 角度实现。
    纵向(dx): 通过调整足端 X 坐标实现。
    """
    z_front = -(body_height - FRONT_HIP_OFFSET)
    z_rear = -(body_height - REAR_HIP_OFFSET)

    fl_hip, fl_calf = ik_front_leg_2d(-dx, z_front)
    fr_hip, fr_calf = ik_front_leg_2d(-dx, z_front)
    rl_thigh, rl_calf = ik_rear_leg_2d(-dx, z_rear)
    rr_thigh, rr_calf = ik_rear_leg_2d(-dx, z_rear)

    # 横向偏移通过 roll 角近似: angle ≈ dy / body_height
    roll_offset = dy / body_height

    targets = {}

    mid, cmd = _cmd('fl_hip_pitch', fl_hip)
    targets[mid] = cmd
    mid, cmd = _cmd('fl_thigh_roll', hip_abd - roll_offset)
    targets[mid] = cmd
    mid, cmd = _cmd('fl_calf', fl_calf)
    targets[mid] = cmd

    mid, cmd = _cmd('fr_hip_pitch', fr_hip)
    targets[mid] = cmd
    mid, cmd = _cmd('fr_thigh_roll', -hip_abd - roll_offset)
    targets[mid] = cmd
    mid, cmd = _cmd('fr_calf', fr_calf)
    targets[mid] = cmd

    mid, cmd = _cmd('rl_hip', hip_abd - roll_offset)
    targets[mid] = cmd
    mid, cmd = _cmd('rl_thigh', rl_thigh)
    targets[mid] = cmd
    mid, cmd = _cmd('rl_calf', rl_calf)
    targets[mid] = cmd

    mid, cmd = _cmd('rr_hip', -hip_abd - roll_offset)
    targets[mid] = cmd
    mid, cmd = _cmd('rr_thigh', rr_thigh)
    targets[mid] = cmd
    mid, cmd = _cmd('rr_calf', rr_calf)
    targets[mid] = cmd

    for name in ('head_pitch', 'head_yaw', 'head_roll', 'neck_pitch',
                 'waist_yaw', 'waist_roll'):
        mid, cmd = _cmd(name, 0.0)
        targets[mid] = cmd
    mid, cmd = _cmd('waist_pitch', waist_pitch)
    targets[mid] = cmd

    return targets


# ─────────────────────────────────────────────────────────────────────────────
# 电机控制
# ─────────────────────────────────────────────────────────────────────────────

def send_all(lz, evo, targets):
    """发送所有关节目标位置（使用分关节增益）。"""
    serial_joints = [j for j in JOINT_MAP if j.bus == "serial"]
    other_joints = [j for j in JOINT_MAP if j.bus != "serial"]

    for j in serial_joints + other_joints:
        mid = j.motor_id
        if mid not in targets:
            continue
        pos = targets[mid]
        g = JOINT_GAINS.get(j.name, {"kp": 30.0, "kd": 4.0, "trq_ff": 0.0})

        if j.mtype == "lz":
            lz.mit_control(mid, pos, 0.0, g["kp"], g["kd"], g["trq_ff"])
        else:
            evo.ptm_control(mid, pos, 0.0, g["kp"], g["kd"], g["trq_ff"])
        time.sleep(SEND_INTERVAL)


def send_all_soft(lz, evo, targets, kp_scale=1.0):
    """发送所有关节目标位置（统一增益, 可调刚度比例）。"""
    serial_joints = [j for j in JOINT_MAP if j.bus == "serial"]
    other_joints = [j for j in JOINT_MAP if j.bus != "serial"]

    for j in serial_joints + other_joints:
        mid = j.motor_id
        if mid not in targets:
            continue
        pos = targets[mid]
        kp_lz = DEFAULT_LZ_KP * kp_scale
        kp_evo = DEFAULT_EVO_KP * kp_scale

        if j.mtype == "lz":
            lz.mit_control(mid, pos, 0.0, kp_lz, DEFAULT_LZ_KD, 0.0)
        else:
            evo.ptm_control(mid, pos, 0.0, kp_evo, DEFAULT_EVO_KD, 0.0)
        time.sleep(SEND_INTERVAL)


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


def smooth_transition(lz, evo, from_pos, to_pos, duration=2.0):
    """S 形插值过渡。"""
    steps = max(1, int(duration * CONTROL_HZ))
    t0 = time.monotonic()
    for step in range(steps + 1):
        alpha = step / steps
        alpha = 3 * alpha * alpha - 2 * alpha * alpha * alpha
        kp_s = 0.3 + 0.7 * alpha
        cur = {}
        for mid in set(from_pos) | set(to_pos):
            a = from_pos.get(mid, 0.0)
            b = to_pos.get(mid, 0.0)
            cur[mid] = a + (b - a) * alpha
        send_all_soft(lz, evo, cur, kp_scale=kp_s)
        pct = int(alpha * 100)
        sys.stdout.write(f"\r  [fade] {pct:3d}%  step {step}/{steps}   ")
        sys.stdout.flush()
        next_t = t0 + (step + 1) / CONTROL_HZ
        sleep_t = next_t - time.monotonic()
        if sleep_t > 0:
            time.sleep(sleep_t)
    sys.stdout.write("\n")


# ─────────────────────────────────────────────────────────────────────────────
# 键盘读取
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
        """清空 stdin 缓冲区中所有积攒的字符。"""
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
                # 丢弃所有 ANSI 转义序列 (Cursor IDE 终端会主动发送)
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
# 主函数
# ─────────────────────────────────────────────────────────────────────────────

def main():
    print(f"\n{'='*62}")
    print(f"  Marsdog 诊断工具 — 逐项验证 IK/坐标系/关节符号")
    print(f"{'='*62}\n")

    # ── 初始化电机 ──────────────────────────────────────────────────────
    lz = MotorLz()
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

    print(f"\n[online] {len(online)}/{len(ALL_IDS)} 电机在线\n")

    if not online:
        print("[ERROR] 无在线电机"); lz.end(); evo.end(); return

    # ── 读取当前位置 & 站立 ──────────────────────────────────────────────
    print("[pos] 读取当前位置...")
    cur_pos = read_positions(lz, evo)

    body_height = 0.24
    waist_pitch = 0.0
    hip_abd = 0.04
    circle_active = False
    circle_leg = 'fl'
    circle_t0 = 0.0

    stand = compute_stand_targets(body_height, waist_pitch=waist_pitch,
                                  hip_abd=hip_abd)

    print("[fade] 过渡到站立姿态 (3s)...")
    smooth_transition(lz, evo, cur_pos, stand, 3.0)
    print("[ok] 已站立\n")

    # ── 当前目标 ─────────────────────────────────────────────────────────
    current_targets = dict(stand)
    state_desc = "站立 (0.24m)"

    # ── 键盘控制 ─────────────────────────────────────────────────────────
    kb = KeyReader()
    kb.start()
    kb.flush()

    print("─" * 52)
    print("  0       — 回到标准站立")
    print("  1 / 2   — 重心左移 / 右移 2cm")
    print("  3 / 4   — 重心前移 / 后移 2cm")
    print("  5/6/7/8 — 抬 FL / FR / RL / RR 腿 3cm")
    print("  a / z   — 选中腿足端向前 / 向后伸 3cm (抬2cm)")
    print("  9       — 当前选中腿画圆 (开/停)")
    print("  w / s   — 弓背 / 塌腰 (waist_pitch ±5°)")
    print("  u / d   — 体高 +/- 1cm")
    print("  p       — 打印电机状态")
    print("  q       — 安全退出")
    print("─" * 52 + "\n")

    try:
        while True:
            t_loop = time.time()

            key = kb.get()

            if key in ('q', 'Q', '\x03'):
                print(f"\n[quit] 按下了 '{key}'")
                break

            new_targets = None

            if key == '0':
                circle_active = False
                new_targets = compute_stand_targets(
                    body_height, waist_pitch=waist_pitch, hip_abd=hip_abd)
                state_desc = f"站立 ({body_height:.2f}m)"

            elif key == '1':
                circle_active = False
                new_targets = compute_shifted_targets(
                    body_height, dy=0.02, waist_pitch=waist_pitch,
                    hip_abd=hip_abd)
                state_desc = "重心左移 +2cm"

            elif key == '2':
                circle_active = False
                new_targets = compute_shifted_targets(
                    body_height, dy=-0.02, waist_pitch=waist_pitch,
                    hip_abd=hip_abd)
                state_desc = "重心右移 -2cm"

            elif key == '3':
                circle_active = False
                new_targets = compute_shifted_targets(
                    body_height, dx=0.02, waist_pitch=waist_pitch,
                    hip_abd=hip_abd)
                state_desc = "重心前移 +2cm"

            elif key == '4':
                circle_active = False
                new_targets = compute_shifted_targets(
                    body_height, dx=-0.02, waist_pitch=waist_pitch,
                    hip_abd=hip_abd)
                state_desc = "重心后移 -2cm"

            elif key == '5':
                circle_active = False
                circle_leg = 'fl'
                base = compute_stand_targets(body_height,
                                             waist_pitch=waist_pitch,
                                             hip_abd=hip_abd)
                new_targets = compute_leg_lifted_targets(
                    base, body_height, 'fl', 0.03)
                state_desc = "抬 FL 腿 3cm"

            elif key == '6':
                circle_active = False
                circle_leg = 'fr'
                base = compute_stand_targets(body_height,
                                             waist_pitch=waist_pitch,
                                             hip_abd=hip_abd)
                new_targets = compute_leg_lifted_targets(
                    base, body_height, 'fr', 0.03)
                state_desc = "抬 FR 腿 3cm"

            elif key == '7':
                circle_active = False
                circle_leg = 'rl'
                base = compute_stand_targets(body_height,
                                             waist_pitch=waist_pitch,
                                             hip_abd=hip_abd)
                new_targets = compute_leg_lifted_targets(
                    base, body_height, 'rl', 0.03)
                state_desc = "抬 RL 腿 3cm"

            elif key == '8':
                circle_active = False
                circle_leg = 'rr'
                base = compute_stand_targets(body_height,
                                             waist_pitch=waist_pitch,
                                             hip_abd=hip_abd)
                new_targets = compute_leg_lifted_targets(
                    base, body_height, 'rr', 0.03)
                state_desc = "抬 RR 腿 3cm"

            elif key == '9':
                if circle_active:
                    circle_active = False
                    new_targets = compute_stand_targets(
                        body_height, waist_pitch=waist_pitch, hip_abd=hip_abd)
                    state_desc = f"站立 ({body_height:.2f}m)"
                else:
                    circle_active = True
                    circle_t0 = time.time()
                    state_desc = f"画圆中 ({circle_leg})"

            elif key == 'a':
                circle_active = False
                leg = circle_leg
                base = compute_stand_targets(body_height,
                                             waist_pitch=waist_pitch,
                                             hip_abd=hip_abd)
                z_front = -(body_height - FRONT_HIP_OFFSET)
                z_rear = -(body_height - REAR_HIP_OFFSET)
                if leg in ('fl', 'fr'):
                    hip_u, calf_u = ik_front_leg_2d(0.03, z_front + 0.02)
                    mid_hp, cmd_hp = _cmd(f'{leg}_hip_pitch', hip_u)
                    mid_ca, cmd_ca = _cmd(f'{leg}_calf', calf_u)
                    base[mid_hp] = cmd_hp
                    base[mid_ca] = cmd_ca
                else:
                    th_u, ca_u = ik_rear_leg_2d(0.03, z_rear + 0.02)
                    mid_th, cmd_th = _cmd(f'{leg}_thigh', th_u)
                    mid_ca, cmd_ca = _cmd(f'{leg}_calf', ca_u)
                    base[mid_th] = cmd_th
                    base[mid_ca] = cmd_ca
                new_targets = base
                state_desc = f"{leg} 足端前伸 +3cm"

            elif key == 'z':
                circle_active = False
                leg = circle_leg
                base = compute_stand_targets(body_height,
                                             waist_pitch=waist_pitch,
                                             hip_abd=hip_abd)
                z_front = -(body_height - FRONT_HIP_OFFSET)
                z_rear = -(body_height - REAR_HIP_OFFSET)
                if leg in ('fl', 'fr'):
                    hip_u, calf_u = ik_front_leg_2d(-0.03, z_front + 0.02)
                    mid_hp, cmd_hp = _cmd(f'{leg}_hip_pitch', hip_u)
                    mid_ca, cmd_ca = _cmd(f'{leg}_calf', calf_u)
                    base[mid_hp] = cmd_hp
                    base[mid_ca] = cmd_ca
                else:
                    th_u, ca_u = ik_rear_leg_2d(-0.03, z_rear + 0.02)
                    mid_th, cmd_th = _cmd(f'{leg}_thigh', th_u)
                    mid_ca, cmd_ca = _cmd(f'{leg}_calf', ca_u)
                    base[mid_th] = cmd_th
                    base[mid_ca] = cmd_ca
                new_targets = base
                state_desc = f"{leg} 足端后伸 -3cm"

            elif key == 'w':
                waist_pitch = min(0.4, waist_pitch + math.radians(5))
                new_targets = compute_stand_targets(
                    body_height, waist_pitch=waist_pitch, hip_abd=hip_abd)
                state_desc = f"弓背 waist_pitch={math.degrees(waist_pitch):+.1f}°"

            elif key == 's':
                waist_pitch = max(0.0, waist_pitch - math.radians(5))
                new_targets = compute_stand_targets(
                    body_height, waist_pitch=waist_pitch, hip_abd=hip_abd)
                state_desc = f"塌腰 waist_pitch={math.degrees(waist_pitch):+.1f}°"

            elif key == 'u':
                body_height = min(0.30, body_height + 0.01)
                new_targets = compute_stand_targets(
                    body_height, waist_pitch=waist_pitch, hip_abd=hip_abd)
                state_desc = f"升高 height={body_height:.2f}m"

            elif key == 'd':
                body_height = max(0.15, body_height - 0.01)
                new_targets = compute_stand_targets(
                    body_height, waist_pitch=waist_pitch, hip_abd=hip_abd)
                state_desc = f"降低 height={body_height:.2f}m"

            elif key in ('p', 'P'):
                print()
                for j in JOINT_MAP:
                    mid = j.motor_id
                    if j.mtype == "lz":
                        act = math.degrees(lz.get_position(mid))
                    else:
                        act = math.degrees(evo.get_position(mid))
                    tgt = math.degrees(current_targets.get(mid, 0.0))
                    err = tgt - act
                    print(f"  Motor {mid:2d} ({j.name:18s})  "
                          f"tgt={tgt:+8.2f}°  act={act:+8.2f}°  "
                          f"err={err:+6.2f}°")
                print()

            # 画圆模式: 实时更新
            if circle_active:
                t_now = time.time() - circle_t0
                radius = 0.01  # 1cm
                freq = 0.5     # 0.5 Hz (2s 一圈)
                phase = 2.0 * math.pi * freq * t_now

                z_front = -(body_height - FRONT_HIP_OFFSET)
                z_rear = -(body_height - REAR_HIP_OFFSET)

                base = compute_stand_targets(body_height,
                                             waist_pitch=waist_pitch,
                                             hip_abd=hip_abd)
                leg = circle_leg
                dx = radius * math.cos(phase)
                dz = radius * math.sin(phase)

                if leg in ('fl', 'fr'):
                    x = dx
                    z = z_front + 0.02 + dz  # 先抬 2cm 再画圆
                    hip_u, calf_u = ik_front_leg_2d(x, z)
                    mid_hp, cmd_hp = _cmd(f'{leg}_hip_pitch', hip_u)
                    mid_ca, cmd_ca = _cmd(f'{leg}_calf', calf_u)
                    base[mid_hp] = cmd_hp
                    base[mid_ca] = cmd_ca
                else:
                    x = dx
                    z = z_rear + 0.02 + dz
                    th_u, ca_u = ik_rear_leg_2d(x, z)
                    mid_th, cmd_th = _cmd(f'{leg}_thigh', th_u)
                    mid_ca, cmd_ca = _cmd(f'{leg}_calf', ca_u)
                    base[mid_th] = cmd_th
                    base[mid_ca] = cmd_ca

                current_targets = base
                send_all(lz, evo, current_targets)

            elif new_targets is not None:
                print(f"  → {state_desc}")
                smooth_transition(lz, evo, current_targets, new_targets, 1.0)
                current_targets = new_targets
            else:
                send_all(lz, evo, current_targets)

            # 状态行
            sys.stdout.write(f"\r  [{state_desc:30s}]  h={body_height:.2f}m  ")
            sys.stdout.flush()

            # 控制频率
            dt = time.time() - t_loop
            sleep_t = 1.0 / CONTROL_HZ - dt
            if sleep_t > 0:
                time.sleep(sleep_t)

    except KeyboardInterrupt:
        print("\n[quit] Ctrl+C")

    finally:
        kb.stop()

        # 安全退出: 回站立 → 缓慢失能
        print("\n[cleanup] 回站立...")
        stand_final = compute_stand_targets(body_height)
        smooth_transition(lz, evo, current_targets, stand_final, 1.5)

        print("[cleanup] 5s 缓速失能...")
        DISABLE_STEPS = int(5.0 * CONTROL_HZ)
        t0 = time.monotonic()
        for step in range(DISABLE_STEPS + 1):
            alpha = step / DISABLE_STEPS
            kp_scale = 1.0 - alpha
            send_all_soft(lz, evo, stand_final, kp_scale=max(kp_scale, 0.0))
            sys.stdout.write(f"\r  [disable] {int(alpha*100):3d}%  "
                             f"kp_scale={kp_scale:.2f}   ")
            sys.stdout.flush()
            next_t = t0 + (step + 1) / CONTROL_HZ
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
        print("[cleanup] 完成。")


if __name__ == "__main__":
    main()
