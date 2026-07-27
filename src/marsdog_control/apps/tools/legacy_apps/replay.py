#!/usr/bin/env python3
"""Marsdog motion-capture CSV replay engine.

Usage:
    # First test: 20% amplitude, 0.3x speed, hanging in the air
    python replay.py --scale 0.2 --speed 0.3

    # Gradually increase
    python replay.py --scale 0.5 --speed 0.5
    python replay.py --scale 1.0 --speed 1.0

    # Only rear-left leg
    python replay.py --scale 0.3 --speed 0.5 --joints 7,8,9

    # Loop playback
    python replay.py --scale 0.5 --speed 0.5 --loop

Controls during playback:
    ESC / q     emergency stop → fade to current → disable
    +  / =      increase scale by 0.1
    -           decrease scale by 0.1
    [           decrease speed by 0.1
    ]           increase speed by 0.1
    SPACE       pause / resume
"""

import argparse
import csv
import math
import os
import sys
import time
import signal
import tty
import termios
import select
import threading
import datetime
from marsdog_control.config.joints import (JOINT_MAP, JOINT_BY_ID, ALL_IDS,
                          DEFAULT_LZ_KP, DEFAULT_LZ_KD,
                          DEFAULT_EVO_KP, DEFAULT_EVO_KD,
                          csv_to_motor_rad)
from marsdog_control.motion.kinematics import compute_standing_pose, print_standing_info
from marsdog_control.hardware.motors.lingzu import MotorLz
from marsdog_control.hardware.motors.evo import MotorEvo

CSV_FPS = 30.0
CONTROL_HZ = 50
FADE_DURATION = 2.0  # seconds for fade-in/out
SEND_INTERVAL = 0.0005  # 0.5ms between CAN frames

# ---------------------------------------------------------------------------
# Globals
# ---------------------------------------------------------------------------
_emergency = False
_paused = False
_scale = 0.3
_speed = 0.5


def signal_handler(sig, frame):
    global _emergency
    _emergency = True


# ---------------------------------------------------------------------------
# CSV loading
# ---------------------------------------------------------------------------

def load_csv(path):
    """Load CSV as list of float lists."""
    rows = []
    with open(path, "r") as f:
        reader = csv.reader(f)
        for line in reader:
            rows.append([float(x) for x in line])
    return rows


def interpolate_row(rows, fractional_index):
    """Linear interpolation between two CSV rows."""
    n = len(rows)
    idx = fractional_index % n
    i0 = int(idx)
    i1 = (i0 + 1) % n
    alpha = idx - i0
    r0 = rows[i0]
    r1 = rows[i1]
    return [r0[c] + (r1[c] - r0[c]) * alpha for c in range(len(r0))]


# ---------------------------------------------------------------------------
# Keyboard input
# ---------------------------------------------------------------------------

class KeyReader:
    """Non-blocking keyboard reader using raw terminal mode."""

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
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old)
            self._old = None
            self._enabled = False

    def get_key(self):
        if not self._enabled:
            return None
        r, _, _ = select.select([sys.stdin], [], [], 0)
        if r:
            return sys.stdin.read(1)
        return None


# ---------------------------------------------------------------------------
# Motor command dispatch
# ---------------------------------------------------------------------------

def send_joint_command(lz, evo, joint, target_rad, kp_lz, kd_lz, kp_evo, kd_evo):
    """Send a position command to one motor."""
    mid = joint.motor_id
    if joint.mtype == "lz":
        lz.mit_control(mid, target_rad, 0.0, kp_lz, kd_lz, 0.0)
    else:
        evo.ptm_control(mid, target_rad, 0.0, kp_evo, kd_evo, 0.0)


def send_all_joints(lz, evo, targets, active_joints, kp_lz, kd_lz, kp_evo, kd_evo):
    """Send commands to all active joints with staggered timing."""
    serial_joints = [j for j in active_joints if j.bus == "serial"]
    can_joints = [j for j in active_joints if j.bus != "serial"]

    for j in serial_joints:
        send_joint_command(lz, evo, j, targets[j.motor_id], kp_lz, kd_lz, kp_evo, kd_evo)
        time.sleep(SEND_INTERVAL)

    for j in can_joints:
        send_joint_command(lz, evo, j, targets[j.motor_id], kp_lz, kd_lz, kp_evo, kd_evo)
        time.sleep(SEND_INTERVAL)


def read_current_positions(lz, evo, active_joints):
    """Read current motor positions (rad) as dict {motor_id: rad}."""
    positions = {}
    time.sleep(0.3)
    for j in active_joints:
        mid = j.motor_id
        if j.mtype == "lz":
            positions[mid] = lz.get_position(mid)
        else:
            positions[mid] = evo.get_position(mid)
    return positions


def check_motors_enabled(lz, evo, active_joints, label=""):
    """Check which motors are enabled, print disabled ones. Returns list of disabled joints."""
    disabled = []
    for j in active_joints:
        mid = j.motor_id
        idx = mid - 1
        if j.mtype == "lz":
            en = lz.is_enabled[idx]
            mode = lz.mode[idx]
            fault = lz.fault[idx]
            pos = math.degrees(lz.get_position(mid))
        else:
            en = (evo.status[idx] == 0x02)
            mode = evo.status[idx]
            fault = evo.fault[idx]
            pos = math.degrees(evo.get_position(mid))
        if not en:
            disabled.append((j, mode, fault, pos))

    if disabled:
        print(f"\n  *** {label} DISABLED ({len(disabled)} motors): ***")
        for j, mode, fault, pos in disabled:
            print(f"    Motor {j.motor_id:2d} ({j.name:>15s}) "
                  f"{j.model:>5s} {j.bus:>6s} mode={mode} fault={fault} pos={pos:+.1f}")
    return disabled


def try_re_enable(lz, evo, disabled_list):
    """Attempt to re-enable disabled motors. Returns count of successfully re-enabled."""
    recovered = 0
    for j, mode, fault, pos in disabled_list:
        mid = j.motor_id
        if j.mtype == "lz":
            if lz.re_enable(mid):
                print(f"    Motor {mid} ({j.name}) re-enabled OK")
                recovered += 1
            else:
                print(f"    Motor {mid} ({j.name}) re-enable FAILED")
        else:
            evo.enter_motor_state(mid)
            time.sleep(0.05)
            if evo.status[mid - 1] == 0x02:
                print(f"    Motor {mid} ({j.name}) re-enabled OK")
                recovered += 1
            else:
                print(f"    Motor {mid} ({j.name}) re-enable FAILED")
    return recovered


# ---------------------------------------------------------------------------
# Smooth transition
# ---------------------------------------------------------------------------

def smooth_transition(lz, evo, start_pos, end_pos, active_joints,
                      duration, kp_lz, kd_lz, kp_evo, kd_evo):
    """Smoothly interpolate from start to end positions over duration seconds."""
    global _emergency
    steps = int(duration * CONTROL_HZ)
    if steps < 1:
        steps = 1

    was_enabled = {}
    for j in active_joints:
        mid = j.motor_id
        idx = mid - 1
        if j.mtype == "lz":
            was_enabled[mid] = lz.is_enabled[idx]
        else:
            was_enabled[mid] = True

    t0 = time.monotonic()
    for step in range(steps + 1):
        if _emergency:
            return
        alpha = step / steps
        alpha = 3 * alpha * alpha - 2 * alpha * alpha * alpha
        targets = {}
        for j in active_joints:
            mid = j.motor_id
            s = start_pos.get(mid, 0.0)
            e = end_pos.get(mid, 0.0)
            targets[mid] = s + (e - s) * alpha
        send_all_joints(lz, evo, targets, active_joints,
                        kp_lz, kd_lz, kp_evo, kd_evo)

        # Monitor for disabling events
        if step % 5 == 0:
            for j in active_joints:
                mid = j.motor_id
                idx = mid - 1
                if j.mtype == "lz":
                    en_now = lz.is_enabled[idx]
                    if was_enabled.get(mid) and not en_now:
                        tgt_d = math.degrees(targets[mid])
                        act_d = math.degrees(lz.get_position(mid))
                        raw_d = math.degrees(lz.position[idx])
                        print(f"\n  [!] Motor {mid} ({j.name}) DISABLED at step {step}/{steps} "
                              f"alpha={alpha:.2f} tgt={tgt_d:+.1f}° act={act_d:+.1f}° "
                              f"raw={raw_d:+.1f}° fault={lz.fault[idx]} mode={lz.mode[idx]}")
                        was_enabled[mid] = False

        next_t = t0 + (step + 1) / CONTROL_HZ
        sleep_t = next_t - time.monotonic()
        if sleep_t > 0:
            time.sleep(sleep_t)


# ---------------------------------------------------------------------------
# Main replay
# ---------------------------------------------------------------------------

def main():
    global _emergency, _paused, _scale, _speed

    parser = argparse.ArgumentParser(description="Marsdog CSV replay engine")
    parser.add_argument("--csv", default=None,
                        help="Path to CSV (default: auto-detect)")
    parser.add_argument("--speed", type=float, default=0.5,
                        help="Playback speed multiplier (default: 0.5)")
    parser.add_argument("--scale", type=float, default=0.3,
                        help="Amplitude scale 0.0~1.0 (default: 0.3)")
    parser.add_argument("--kp-lz", type=float, default=DEFAULT_LZ_KP,
                        help=f"灵足 Kp (default: {DEFAULT_LZ_KP})")
    parser.add_argument("--kd-lz", type=float, default=DEFAULT_LZ_KD,
                        help=f"灵足 Kd (default: {DEFAULT_LZ_KD})")
    parser.add_argument("--kp-evo", type=float, default=DEFAULT_EVO_KP,
                        help=f"泉智博 Kp (default: {DEFAULT_EVO_KP})")
    parser.add_argument("--kd-evo", type=float, default=DEFAULT_EVO_KD,
                        help=f"泉智博 Kd (default: {DEFAULT_EVO_KD})")
    parser.add_argument("--loop", action="store_true", help="Loop playback")
    parser.add_argument("--joints", type=str, default=None,
                        help="Comma-separated motor IDs (e.g., 7,8,9)")
    parser.add_argument("--fade", type=float, default=FADE_DURATION,
                        help=f"Fade-in/out duration (default: {FADE_DURATION}s)")
    parser.add_argument("--log", action="store_true",
                        help="Enable detailed CSV logging to replay_log.csv")
    parser.add_argument("--height", type=float, default=0.25,
                        help="Standing body height in meters (default: 0.25)")
    parser.add_argument("--no-stand", action="store_true",
                        help="Skip standing pose, fade directly to CSV frame 0")
    args = parser.parse_args()

    _scale = args.scale
    _speed = args.speed

    signal.signal(signal.SIGINT, signal_handler)

    # Resolve CSV path
    csv_path = args.csv
    if csv_path is None:
        base = os.path.dirname(os.path.abspath(__file__))
        candidate = os.path.join(base, "..", "Marsdog_RuleBase",
                                 "Marsdog_RuleBase", "marsdog_motion.csv")
        if os.path.exists(candidate):
            csv_path = candidate
        else:
            print("[ERROR] Cannot find marsdog_motion.csv. Use --csv to specify.")
            sys.exit(1)

    # Filter active joints
    if args.joints:
        active_ids = set(int(x) for x in args.joints.split(","))
        active_joints = [j for j in JOINT_MAP if j.motor_id in active_ids]
    else:
        active_joints = list(JOINT_MAP)

    if not active_joints:
        print("[ERROR] No valid joints selected.")
        sys.exit(1)

    # Load CSV
    print(f"[load] CSV: {csv_path}")
    rows = load_csv(csv_path)
    n_frames = len(rows)
    n_cols = len(rows[0]) if rows else 0
    duration_s = n_frames / CSV_FPS
    print(f"[load] {n_frames} frames x {n_cols} cols, {CSV_FPS:.0f}fps = {duration_s:.1f}s")
    print(f"[config] speed={_speed:.1f}x  scale={_scale:.1f}  "
          f"fade={args.fade:.1f}s  loop={args.loop}")
    print(f"[config] Active joints: {[j.motor_id for j in active_joints]}")

    # Initialize motors
    print("\n[init] Initializing motor buses...")
    lz = MotorLz()
    evo = MotorEvo()

    need_can1 = any(j.bus == "can1" for j in active_joints)
    need_serial = any(j.bus == "serial" for j in active_joints)
    need_can0 = any(j.bus == "can0" for j in active_joints)

    # C++ 驱动要求: InitSerial 必须在 Init("can1") 之前
    if need_serial:
        print("[init] Serial (灵足 USB-CAN /dev/ttyUSB0)...")
        if not lz.init_serial("/dev/ttyUSB0", 921600):
            print("[WARNING] Serial init failed, serial motors may not work")

    if need_can1:
        print("[init] CAN1 (灵足 SocketCAN)...")
        if not lz.init("can1"):
            print("[WARNING] CAN1 init failed, CAN1 motors may not work")

    if need_can0:
        print("[init] CAN0 (泉智博 SocketCAN)...")
        if not evo.init("can0"):
            print("[WARNING] CAN0 init failed, CAN0 motors may not work")

    # 过滤掉初始化失败(离线)的电机
    online_joints = []
    for j in active_joints:
        mid = j.motor_id
        if j.mtype == "lz":
            if lz.is_connected[mid - 1]:
                online_joints.append(j)
            else:
                print(f"[init] ✗ Motor {mid} ({j.name}) 离线，已跳过")
        else:
            if evo.is_connected[mid - 1]:
                online_joints.append(j)
            else:
                print(f"[init] ✗ Motor {mid} ({j.name}) 离线，已跳过")
    active_joints = online_joints
    print(f"[init] 在线关节: {[j.motor_id for j in active_joints]}")

    if not active_joints:
        print("[ERROR] 没有在线电机，退出。")
        lz.end()
        evo.end()
        sys.exit(1)

    # 使能 MotorEvo
    for j in active_joints:
        if j.mtype == "evo":
            evo.enter_motor_state(j.motor_id)
            time.sleep(0.01)

    # === 预飞检查: 确认所有电机使能 ===
    time.sleep(0.3)
    disabled = check_motors_enabled(lz, evo, active_joints, "PRE-FLIGHT")
    if disabled:
        print("\n[pre-flight] Attempting to re-enable disabled motors...")
        try_re_enable(lz, evo, disabled)
        time.sleep(0.2)
        disabled2 = check_motors_enabled(lz, evo, active_joints, "PRE-FLIGHT retry")
        if disabled2:
            still_disabled_ids = set(j.motor_id for j, _, _, _ in disabled2)
            active_joints = [j for j in active_joints if j.motor_id not in still_disabled_ids]
            print(f"[pre-flight] 移除失能电机，剩余: {[j.motor_id for j in active_joints]}")
    else:
        print("[pre-flight] All motors enabled OK")

    # --- Logging setup ---
    log_csv_file = None
    log_writer = None
    log_cycle_counter = 0
    LOG_INTERVAL = 10  # log every 10th control cycle (≈10Hz)

    if args.log:
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        log_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                f"replay_log_{ts}.csv")
        log_csv_file = open(log_path, "w", newline="")
        log_writer = csv.writer(log_csv_file)
        log_writer.writerow([
            "time_s", "frame_idx", "scale", "speed",
            "motor_id", "name", "model", "bus",
            "csv_col", "csv_raw_rad", "csv_raw_deg",
            "sign", "frame0_base_rad", "frame0_base_deg",
            "delta_rad", "delta_scaled_rad",
            "motor_target_rad", "motor_target_deg",
            "motor_actual_rad", "motor_actual_deg",
            "error_deg",
            "motor_velocity", "limit_lo_deg", "limit_hi_deg",
            "was_clamped", "is_enabled", "mode", "fault",
        ])
        print(f"[log] Logging enabled → {log_path}")

    def log_snapshot(writer, t_sec, frame_f, cur_scale, cur_speed,
                     interp_row, frame0, targets, active_jts, lz_, evo_):
        """Write one log row per motor at this instant."""
        if writer is None:
            return
        fi = int(frame_f) % n_frames
        for j in active_jts:
            mid = j.motor_id
            raw = interp_row[j.csv_col]
            base = frame0[j.csv_col]
            delta = raw - base
            delta_s = delta * cur_scale
            unclamped = (base + delta_s) * j.sign
            target = targets[mid]
            clamped = (abs(unclamped - target) > 1e-6)

            if j.mtype == "lz":
                actual = lz_.get_position(mid)
                vel = lz_.get_velocity(mid)
                en = lz_.is_enabled[mid - 1]
                m = lz_.mode[mid - 1]
                ft = lz_.fault[mid - 1]
            else:
                actual = evo_.get_position(mid)
                vel = evo_.get_velocity(mid)
                en = (evo_.status[mid - 1] == 0x02)
                m = evo_.status[mid - 1]
                ft = evo_.fault[mid - 1]

            writer.writerow([
                f"{t_sec:.4f}", fi, f"{cur_scale:.2f}", f"{cur_speed:.2f}",
                mid, j.name, j.model, j.bus,
                j.csv_col, f"{raw:.6f}", f"{math.degrees(raw):.3f}",
                j.sign, f"{base:.6f}", f"{math.degrees(base):.3f}",
                f"{delta:.6f}", f"{delta_s:.6f}",
                f"{target:.6f}", f"{math.degrees(target):.3f}",
                f"{actual:.6f}", f"{math.degrees(actual):.3f}",
                f"{math.degrees(actual - target):.3f}",
                f"{vel:.4f}",
                f"{math.degrees(j.limit_lo):.1f}",
                f"{math.degrees(j.limit_hi):.1f}",
                "Y" if clamped else "",
                "Y" if en else "N", m, ft,
            ])

    keys = KeyReader()

    # === Keepalive thread: hold current position until fade-in starts ===
    _keepalive_active = True
    _keepalive_pos = {}

    def _keepalive_loop():
        keep_kp = min(args.kp_lz, 5.0)
        keep_kd = min(args.kd_lz, 1.0)
        while _keepalive_active:
            for j in active_joints:
                mid = j.motor_id
                if mid in _keepalive_pos:
                    pos = _keepalive_pos[mid]
                    if j.mtype == "lz":
                        lz.mit_control(mid, pos, 0.0, keep_kp, keep_kd, 0.0)
                    else:
                        evo.ptm_control(mid, pos, 0.0, args.kp_evo, args.kd_evo, 0.0)
                    time.sleep(SEND_INTERVAL)
            time.sleep(0.015)

    try:
        # Read current positions
        print("[init] Reading current motor positions...")
        current_pos = read_current_positions(lz, evo, active_joints)
        for j in active_joints:
            deg = current_pos[j.motor_id] * 180 / math.pi
            print(f"  Motor {j.motor_id:2d} ({j.name:18s}): {deg:7.2f}°")

        # Start keepalive with current positions
        _keepalive_pos = dict(current_pos)
        keepalive_thread = threading.Thread(target=_keepalive_loop, daemon=True)
        keepalive_thread.start()
        print("[keepalive] Holding positions until fade-in...")

        # Compute frame 0 target positions
        frame0 = rows[0]
        frame0_targets = {}
        for j in active_joints:
            frame0_targets[j.motor_id] = csv_to_motor_rad(
                frame0, j, scale=_scale, frame0_row=frame0)

        # ── 站立姿态计算 ──
        stand_targets = None
        if not args.no_stand:
            all_stand = compute_standing_pose(args.height)
            stand_targets = {j.motor_id: all_stand[j.motor_id] for j in active_joints}
            print(f"\n[stand] 站立姿态 (height={args.height:.3f}m):")
            for j in active_joints:
                sd = math.degrees(stand_targets[j.motor_id])
                f0d = math.degrees(frame0_targets[j.motor_id])
                cd = math.degrees(current_pos.get(j.motor_id, 0.0))
                print(f"  Motor {j.motor_id:2d} ({j.name:18s}): "
                      f"cur={cd:+7.1f}°  stand={sd:+7.1f}°  csv0={f0d:+7.1f}°")

        # Stop keepalive before transitions take over
        _keepalive_active = False
        time.sleep(0.03)

        # Verify motors before transitions
        disabled = check_motors_enabled(lz, evo, active_joints, "PRE-FADE")
        if disabled:
            print("[pre-fade] Re-enabling...")
            try_re_enable(lz, evo, disabled)

        fade_kp_lz = min(args.kp_lz, 5.0)
        fade_kd_lz = min(args.kd_lz, 1.0)

        if stand_targets is not None:
            # Phase A1: current → standing pose
            print(f"\n[phase A1] 过渡到站立姿态 ({args.fade:.1f}s, kp={fade_kp_lz})...")
            smooth_transition(lz, evo, current_pos, stand_targets, active_joints,
                              args.fade, fade_kp_lz, fade_kd_lz,
                              args.kp_evo, args.kd_evo)
            if _emergency:
                print("[ESTOP] Emergency stop during standing transition!")
                raise KeyboardInterrupt

            time.sleep(0.5)
            disabled = check_motors_enabled(lz, evo, active_joints, "POST-STAND")
            if disabled:
                try_re_enable(lz, evo, disabled)

            # Phase A2: standing → CSV frame 0
            print(f"[phase A2] 站立姿态 → CSV 帧0 ({args.fade:.1f}s, kp={fade_kp_lz})...")
            smooth_transition(lz, evo, stand_targets, frame0_targets, active_joints,
                              args.fade, fade_kp_lz, fade_kd_lz,
                              args.kp_evo, args.kd_evo)
        else:
            # Phase A: direct current → CSV frame 0 (original behavior)
            print(f"\n[phase A] 过渡到 CSV 帧0 ({args.fade:.1f}s, kp={fade_kp_lz})...")
            smooth_transition(lz, evo, current_pos, frame0_targets, active_joints,
                              args.fade, fade_kp_lz, fade_kd_lz,
                              args.kp_evo, args.kd_evo)

        if _emergency:
            print("[ESTOP] Emergency stop during fade-in!")
            raise KeyboardInterrupt

        # Log frame 0 snapshot after fade-in
        if args.log:
            time.sleep(0.1)
            log_snapshot(log_writer, 0.0, 0.0, _scale, _speed,
                         frame0, frame0, frame0_targets,
                         active_joints, lz, evo)
            log_csv_file.flush()

        # Phase B: Playback
        print("\n[phase B] Playback started!")
        print("  ESC/q=stop  +/-=scale  [/]=speed  SPACE=pause")
        keys.start()

        t_start = time.monotonic()
        t_paused_total = 0.0
        frame_idx = 0
        last_print = 0.0

        while not _emergency:
            # Handle keyboard
            key = keys.get_key()
            if key:
                if key in ('\x1b', 'q', 'Q'):
                    print("\n[ESTOP] User requested stop.")
                    break
                elif key in ('+', '='):
                    _scale = min(1.0, _scale + 0.1)
                    print(f"\n  [scale → {_scale:.1f}]")
                elif key == '-':
                    _scale = max(0.0, _scale - 0.1)
                    print(f"\n  [scale → {_scale:.1f}]")
                elif key == ']':
                    _speed = min(2.0, _speed + 0.1)
                    print(f"\n  [speed → {_speed:.1f}x]")
                elif key == '[':
                    _speed = max(0.1, _speed - 0.1)
                    print(f"\n  [speed → {_speed:.1f}x]")
                elif key == ' ':
                    _paused = not _paused
                    if _paused:
                        t_pause_start = time.monotonic()
                        print("\n  [PAUSED — press SPACE to resume]")
                    else:
                        t_paused_total += time.monotonic() - t_pause_start
                        print("\n  [RESUMED]")

            if _paused:
                time.sleep(0.01)
                continue

            # Compute current time in CSV space
            t_now = time.monotonic()
            t_elapsed = (t_now - t_start - t_paused_total) * _speed
            csv_time = t_elapsed
            frac_frame = csv_time * CSV_FPS

            # Check end of playback
            if not args.loop and frac_frame >= n_frames - 1:
                print("\n[phase B] Playback complete.")
                break

            # Interpolate CSV row
            interp_row = interpolate_row(rows, frac_frame)

            # Compute motor targets with scaling
            targets = {}
            for j in active_joints:
                targets[j.motor_id] = csv_to_motor_rad(
                    interp_row, j, scale=_scale, frame0_row=frame0)

            # Send commands
            send_all_joints(lz, evo, targets, active_joints,
                            args.kp_lz, args.kd_lz, args.kp_evo, args.kd_evo)

            # CSV logging (every LOG_INTERVAL cycles ≈10Hz)
            log_cycle_counter += 1
            if args.log and (log_cycle_counter % LOG_INTERVAL == 0):
                log_snapshot(log_writer, csv_time, frac_frame,
                             _scale, _speed, interp_row, frame0,
                             targets, active_joints, lz, evo)
                if log_cycle_counter % (LOG_INTERVAL * 10) == 0:
                    log_csv_file.flush()

            # Status display + motor health check (2Hz)
            if t_now - last_print > 0.5:
                fi = int(frac_frame) % n_frames
                pct = (fi / n_frames) * 100

                # Count enabled/disabled
                n_en = 0
                n_dis = 0
                dis_ids = []
                for j in active_joints:
                    mid = j.motor_id
                    idx = mid - 1
                    if j.mtype == "lz":
                        en = lz.is_enabled[idx]
                    else:
                        en = (evo.status[idx] == 0x02)
                    if en:
                        n_en += 1
                    else:
                        n_dis += 1
                        dis_ids.append(mid)

                status = (f"\r  frame {fi:4d}/{n_frames}  {pct:5.1f}%  "
                          f"scale={_scale:.1f}  speed={_speed:.1f}x  "
                          f"t={csv_time:.1f}s  "
                          f"en={n_en}/{n_en+n_dis}")
                if dis_ids:
                    status += f" DIS={dis_ids}"
                print(status + "   ", end="", flush=True)
                last_print = t_now

                # Auto re-enable check (attempt every 2s)
                if dis_ids and (fi % 60 == 0):
                    print(f"\n  [!] {len(dis_ids)} motors disabled: {dis_ids}, re-enabling...")
                    for j in active_joints:
                        if j.motor_id in dis_ids:
                            if j.mtype == "lz":
                                if lz.re_enable(j.motor_id):
                                    print(f"    Motor {j.motor_id} re-enabled OK")
                            else:
                                evo.enter_motor_state(j.motor_id)

            # Timing for target control rate
            next_cycle = t_now + 1.0 / CONTROL_HZ
            sleep_t = next_cycle - time.monotonic()
            if sleep_t > 0:
                time.sleep(sleep_t)

        keys.stop()

        # Phase C: Fade-out → 回到站立姿态 (安全落地)
        if not _emergency:
            fade_start = read_current_positions(lz, evo, active_joints)
            if stand_targets is not None:
                print(f"\n[phase C] 回到站立姿态 ({args.fade:.1f}s)...")
                smooth_transition(lz, evo, fade_start, stand_targets, active_joints,
                                  args.fade, fade_kp_lz, fade_kd_lz,
                                  args.kp_evo, args.kd_evo)
            else:
                print(f"\n[phase C] Fading out ({args.fade:.1f}s)...")
                fade_end = {}
                for j in active_joints:
                    fade_end[j.motor_id] = frame0[j.csv_col] * j.sign
                smooth_transition(lz, evo, fade_start, fade_end, active_joints,
                                  args.fade, fade_kp_lz, fade_kd_lz,
                                  args.kp_evo, args.kd_evo)

    except KeyboardInterrupt:
        pass
    finally:
        keys.stop()
        if log_csv_file is not None:
            log_csv_file.flush()
            log_csv_file.close()
            print(f"\n[log] CSV log saved → {log_path}")
        print("\n[cleanup] Stopping all motors...")
        lz.end()
        evo.end()
        print("[cleanup] Done. Robot safe.")


if __name__ == "__main__":
    main()
