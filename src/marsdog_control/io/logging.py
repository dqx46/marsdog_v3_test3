"""Runtime CSV logging helpers."""

from __future__ import annotations

import csv
import datetime
import json
import math
import os
import sys
import time
from dataclasses import dataclass
from typing import Optional, Sequence

from marsdog_control.config.joints import (
    DEFAULT_EVO_KD,
    DEFAULT_EVO_KP,
    DEFAULT_LZ_KD,
    DEFAULT_LZ_KP,
    JOINT_BY_NAME as JBN,
)
from marsdog_control.control.executor import resolve_gains
from marsdog_control.motion.kinematics import front_foot_pitch_from_motor


@dataclass
class LogRuntime:
    """Runtime knobs mirrored into the log metadata file."""
    active_dm_kp_by_id: dict
    active_dm_kd_by_id: dict
    dm_reference_lead_s: dict
    dm_reference_lead_max_rad: float
    dm_dq_feedforward: bool
    dm_dq_max_rps: float
    leg_kp_scale: float
    var_impedance: bool


@dataclass
class WriteLogRuntime:
    """Per-row logging dependencies that track live walk.py knobs."""
    real_joints: Sequence
    dm_fixed_targets: dict
    joint_gains: dict
    leg_kp_scale: float
    default_lz_kp: float = DEFAULT_LZ_KP
    default_lz_kd: float = DEFAULT_LZ_KD
    default_evo_kp: float = DEFAULT_EVO_KP
    default_evo_kd: float = DEFAULT_EVO_KD


LOG_HEADER = [
    "t_s", "run_t_s", "mode", "fsm_mode", "gait_active", "controller",
    "input_has_stick", "input_vx", "input_turn", "input_request_mode",
    "dt_ms", "control_period_ms", "throttle",
    "height_m", "period_s", "amp_front_cm", "amp_rear_cm",
    "phase_fl", "phase_fr", "phase_rl", "phase_rr",
    "imu_roll_deg", "imu_pitch_deg", "imu_yaw_deg",
    "imu_gyro_roll", "imu_gyro_pitch", "imu_age_ms",
    "imu_raw_roll_deg", "imu_raw_pitch_deg",
    "imu_raw_gyro_roll", "imu_raw_gyro_pitch",
    "imu_angle_age_ms", "imu_gyro_age_ms", "imu_acc_age_ms",
    "imu_angle_seq", "imu_gyro_seq", "imu_acc_seq",
    "imu_predict_lead_ms",
    "imu_p_roll_mm", "imu_i_roll_mm", "imu_d_roll_mm", "imu_trim_roll_mm",
    "imu_p_pitch_mm", "imu_i_pitch_mm", "imu_d_pitch_mm", "imu_trim_pitch_mm",
    "imu_phase_gain",
    "imu_dz_fl_mm", "imu_dz_fr_mm", "imu_dz_rl_mm", "imu_dz_rr_mm",
    "imu_roll_out_mm", "imu_pitch_out_mm", "imu_ramp_frac",
    "reactive_deg", "lateral_sway_mm", "expected_roll",
    "foot_pitch_target_fl_deg", "foot_pitch_command_fl_deg",
    "foot_pitch_actual_fl_deg",
    "foot_pitch_target_fr_deg", "foot_pitch_command_fr_deg",
    "foot_pitch_actual_fr_deg",
    "motor_id", "name", "target_deg", "command_deg", "command_dq_rps",
    "actual_deg", "error_deg", "command_error_deg", "torque_nm",
    "actual_kp", "actual_kd", "kp_phase_scale", "trq_ff_nm",
    "dm_command_seq", "dm_feedback_seq", "dm_feedback_age_ms",
    "dm_rtt_ms", "dm_dropped_commands",
]


def setup_log(enabled: bool, args=None, *, base_dir: str,
              runtime: Optional[LogRuntime] = None):
    if not enabled:
        return None, None, None
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    log_dir = os.path.join(base_dir, "log")
    os.makedirs(log_dir, exist_ok=True)
    path = os.path.join(log_dir, f"walk_log_{ts}.csv")
    if args is not None:
        meta_path = os.path.join(log_dir, f"walk_log_{ts}.meta.json")
        dm = {}
        if runtime is not None:
            dm = {
                "kp_by_id": runtime.active_dm_kp_by_id,
                "kd_by_id": runtime.active_dm_kd_by_id,
                "reference_lead_s": runtime.dm_reference_lead_s,
                "reference_lead_max_deg": math.degrees(runtime.dm_reference_lead_max_rad),
                "dq_feedforward": runtime.dm_dq_feedforward,
                "dq_max_rps": runtime.dm_dq_max_rps,
            }
        meta = {
            "created": datetime.datetime.now().isoformat(timespec="seconds"),
            "argv": list(sys.argv),
            "explicit_cli": sorted(getattr(args, "_explicit_cli", set())),
            "final_args": {
                key: (sorted(value) if isinstance(value, set) else value)
                for key, value in vars(args).items()
                if not key.startswith("_")
            },
            "dm": dm,
            "leg_kp_scale": runtime.leg_kp_scale if runtime is not None else 1.0,
            "var_impedance": runtime.var_impedance if runtime is not None else False,
        }
        with open(meta_path, "w") as meta_file:
            json.dump(meta, meta_file, ensure_ascii=False, indent=2)
    f = open(path, "w", newline="")
    w = csv.writer(f)
    w.writerow(LOG_HEADER)
    print(f"[log] CSV: log/{os.path.basename(path)}")
    print(f"[log] 路径: {path}")
    return f, w, path


def write_log(writer, t_s, mode, lz, evo, dm, incos, targets, dt_ms,
              trot, throttle, runtime: WriteLogRuntime,
              imu=None, imu_dz=None,
              imu_ctrl=None, ramp_frac=0.0, kp_phase=None, trq_ff=None,
              run_t_s=0.0, fsm_mode="", gait_active=False,
              controller_name="", input_vx=0.0, input_turn=0.0,
              input_has_stick=False, input_request_mode="",
              control_period_ms=0.0, feedback=None):
    if writer is None:
        return
    samples = feedback.samples if feedback is not None else {}

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
        imu_roll = math.degrees(imu.roll)
        imu_pitch = math.degrees(imu.pitch)
        imu_yaw = math.degrees(imu.yaw)
        imu_gr = math.degrees(imu.gyro_roll)
        imu_gp = math.degrees(imu.gyro_pitch)
        _now_mono = time.monotonic()
        _ages = imu.frame_ages(_now_mono)
        imu_age_ms = _ages["angle"] * 1000.0
        imu_raw_roll = math.degrees(imu.raw_roll)
        imu_raw_pitch = math.degrees(imu.raw_pitch)
        imu_raw_gr = math.degrees(imu.raw_gyro_roll)
        imu_raw_gp = math.degrees(imu.raw_gyro_pitch)
        imu_angle_age = _ages["angle"] * 1000.0
        imu_gyro_age = _ages["gyro"] * 1000.0
        imu_acc_age = _ages["acc"] * 1000.0
        imu_angle_seq = imu.angle_update_count
        imu_gyro_seq = imu.gyro_update_count
        imu_acc_seq = imu.acc_update_count
    else:
        imu_roll = imu_pitch = imu_yaw = imu_gr = imu_gp = imu_age_ms = float('nan')
        imu_raw_roll = imu_raw_pitch = imu_raw_gr = imu_raw_gp = float('nan')
        imu_angle_age = imu_gyro_age = imu_acc_age = float('nan')
        imu_angle_seq = imu_gyro_seq = imu_acc_seq = 0

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

    foot_pitch_log = {}
    for leg in ("fl", "fr"):
        hip_id = JBN[f"{leg}_hip_pitch"].motor_id
        calf_id = JBN[f"{leg}_calf"].motor_id
        tarsus_id = JBN[f"{leg}_tarsus"].motor_id
        target_values = {
            hip_id: targets.get(hip_id),
            calf_id: targets.get(calf_id),
            tarsus_id: targets.get(tarsus_id),
        }
        command_values = dict(target_values)
        if dm is not None:
            command_values[tarsus_id] = dm.get_timing(tarsus_id).get(
                "command_q", target_values[tarsus_id])
        actual_values = {
            hip_id: (samples[hip_id].position if hip_id in samples
                     else (lz.get_position(hip_id) if lz is not None else None)),
            calf_id: (samples[calf_id].position if calf_id in samples
                      else (incos.get_position(calf_id) if incos is not None else None)),
            tarsus_id: (samples[tarsus_id].position if tarsus_id in samples
                        else (dm.get_position(tarsus_id) if dm is not None else None)),
        }
        foot_pitch_log[leg] = (
            front_foot_pitch_from_motor(leg, target_values),
            front_foot_pitch_from_motor(leg, command_values),
            front_foot_pitch_from_motor(leg, actual_values),
        )

    for j in runtime.real_joints:
        mid = j.motor_id
        sample = samples.get(mid)
        if j.mtype == "dm":
            tgt = targets.get(mid, runtime.dm_fixed_targets.get(mid, float('nan')))
            if sample is not None:
                act = sample.position
                tq = sample.torque
                timing = sample.timing
            else:
                act = dm.get_position(mid) if dm is not None else None
                try:
                    tq = dm.get_torque(mid) if dm is not None else float('nan')
                except Exception:
                    tq = float('nan')
                timing = dm.get_timing(mid) if dm is not None else {}
            cmd = timing.get("command_q", tgt)
            cmd_dq = timing.get("command_dq", 0.0)
            actual_kp = timing.get("command_kp", float("nan"))
            actual_kd = timing.get("command_kd", float("nan"))
            phase_scale = 1.0
            ff_nm = timing.get("command_tau", 0.0)
        else:
            tgt = targets.get(mid, float('nan'))
            if sample is not None:
                act = sample.position
                tq = sample.torque
            elif j.mtype == "lz":
                act = lz.get_position(mid)
                try:
                    tq = lz.get_torque(mid)
                except Exception:
                    tq = float('nan')
            elif j.mtype == "incos":
                act = incos.get_position(mid) if incos is not None else None
                try:
                    tq = incos.get_torque(mid) if incos is not None else float('nan')
                except Exception:
                    tq = float('nan')
            else:
                act = evo.get_position(mid)
                try:
                    tq = evo.get_torque(mid)
                except Exception:
                    tq = float('nan')
            timing = {}
            cmd = tgt
            cmd_dq = float("nan")
            phase_scale = kp_phase.get(mid, 1.0) if kp_phase else 1.0
            ff_nm = trq_ff.get(mid) if trq_ff and mid in trq_ff else None
            actual_kp, actual_kd, _ = resolve_gains(
                j, 1.0, True,
                runtime.default_lz_kp, runtime.default_lz_kd,
                runtime.default_evo_kp, runtime.default_evo_kd,
                runtime.leg_kp_scale, runtime.joint_gains,
                phase_scale, ff_nm)
            if ff_nm is None:
                ff_nm = runtime.joint_gains.get(j.name, {}).get("trq_ff", 0.0)
        act = act if act is not None else float('nan')
        tq = tq if tq is not None else float('nan')
        err_val = (math.degrees(act - tgt)
                   if (tgt is not None and math.isfinite(tgt)
                       and not math.isnan(act)) else float('nan'))
        cmd_err_val = (math.degrees(act - cmd)
                       if (cmd is not None and math.isfinite(cmd)
                           and not math.isnan(act)) else float('nan'))
        comp = imu_ctrl.components if imu_ctrl else {}
        writer.writerow([
            f"{t_s:.4f}", f"{run_t_s:.4f}", mode, fsm_mode,
            int(bool(gait_active)), controller_name,
            int(bool(input_has_stick)), f"{input_vx:.4f}", f"{input_turn:.4f}",
            input_request_mode,
            f"{dt_ms:.2f}", f"{control_period_ms:.2f}", f"{throttle:.3f}",
            f"{height:.4f}", f"{period:.3f}", f"{amp_f:.2f}", f"{amp_r:.2f}",
            f"{phases['fl']:.4f}", f"{phases['fr']:.4f}",
            f"{phases['rl']:.4f}", f"{phases['rr']:.4f}",
            f"{imu_roll:.2f}", f"{imu_pitch:.2f}", f"{imu_yaw:.1f}",
            f"{imu_gr:.1f}", f"{imu_gp:.1f}", f"{imu_age_ms:.1f}",
            f"{imu_raw_roll:.2f}", f"{imu_raw_pitch:.2f}",
            f"{imu_raw_gr:.1f}", f"{imu_raw_gp:.1f}",
            f"{imu_angle_age:.2f}", f"{imu_gyro_age:.2f}", f"{imu_acc_age:.2f}",
            imu_angle_seq, imu_gyro_seq, imu_acc_seq,
            f"{imu_ctrl.prediction_lead_s*1000:.2f}" if imu_ctrl else "0",
            f"{comp.get('p_roll', 0.0)*1000:.3f}",
            f"{comp.get('i_roll', 0.0)*1000:.3f}",
            f"{comp.get('d_roll', 0.0)*1000:.3f}",
            f"{comp.get('trim_roll', 0.0)*1000:.3f}",
            f"{comp.get('p_pitch', 0.0)*1000:.3f}",
            f"{comp.get('i_pitch', 0.0)*1000:.3f}",
            f"{comp.get('d_pitch', 0.0)*1000:.3f}",
            f"{comp.get('trim_pitch', 0.0)*1000:.3f}",
            f"{getattr(imu_ctrl, '_last_phase_gain', 1.0):.3f}" if imu_ctrl else "0",
            f"{dz['fl']*1000:.2f}", f"{dz['fr']*1000:.2f}",
            f"{dz['rl']*1000:.2f}", f"{dz['rr']*1000:.2f}",
            f"{imu_ctrl.roll_out*1000:.3f}" if imu_ctrl else "0",
            f"{imu_ctrl.pitch_out*1000:.3f}" if imu_ctrl else "0",
            f"{ramp_frac:.3f}",
            f"{reactive_deg:.2f}", f"{lateral_mm:.2f}", f"{expected_roll:.2f}",
            f"{foot_pitch_log['fl'][0]:.2f}", f"{foot_pitch_log['fl'][1]:.2f}",
            f"{foot_pitch_log['fl'][2]:.2f}",
            f"{foot_pitch_log['fr'][0]:.2f}", f"{foot_pitch_log['fr'][1]:.2f}",
            f"{foot_pitch_log['fr'][2]:.2f}",
            mid, j.name,
            f"{math.degrees(tgt):.3f}" if not math.isnan(tgt) else "nan",
            f"{math.degrees(cmd):.3f}" if cmd is not None and math.isfinite(cmd) else "nan",
            f"{cmd_dq:.4f}" if math.isfinite(cmd_dq) else "nan",
            f"{math.degrees(act):.3f}" if not math.isnan(act) else "nan",
            f"{err_val:.3f}" if not math.isnan(err_val) else "nan",
            f"{cmd_err_val:.3f}" if not math.isnan(cmd_err_val) else "nan",
            f"{tq:.3f}" if not math.isnan(tq) else "nan",
            f"{actual_kp:.3f}" if math.isfinite(actual_kp) else "nan",
            f"{actual_kd:.3f}" if math.isfinite(actual_kd) else "nan",
            f"{phase_scale:.3f}", f"{ff_nm:.3f}",
            timing.get("command_seq", 0), timing.get("feedback_seq", 0),
            f"{timing.get('feedback_age_s', float('nan'))*1000:.3f}",
            f"{timing.get('rtt_s', float('nan'))*1000:.3f}",
            timing.get("dropped_commands", 0),
        ])


__all__ = ["LOG_HEADER", "LogRuntime", "WriteLogRuntime", "setup_log", "write_log"]
