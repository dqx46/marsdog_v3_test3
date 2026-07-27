"""Deprecated offline bridge: typed config → legacy ``argparse.Namespace``.

**Not used by the walk main path.** Config is one-way: CLI → ``RuntimeConfig``.
This module remains only for offline tools / migration helpers that still need
a Namespace filled from typed defaults.

Prefer reading ``RuntimeConfig`` directly in new code.
"""

from __future__ import annotations

from argparse import Namespace
from typing import Optional

from marsdog_control.config.schema import RuntimeConfig
from marsdog_control.core.units import m_to_mm, rad_to_deg, s_to_ms


def _set_optional(args: Namespace, name: str, value: Optional[float]) -> None:
    setattr(args, name, None if value is None else float(value))


def apply_runtime_config_to_legacy_args(
    args: Namespace,
    config: RuntimeConfig,
) -> Namespace:
    """Mirror typed config values back to legacy CLI field names (offline only).

    Walk bring-up must not call this. Cover only fields that exist in
    ``RuntimeConfig``; unmigrated CLI knobs stay untouched.
    """

    features = config.features
    setattr(args, "no_imu", not features.imu_enabled)
    setattr(args, "gravity_comp", features.gravity_comp_enabled)
    setattr(args, "var_impedance", features.variable_impedance_enabled)
    setattr(args, "yaw_hold", features.yaw_hold_enabled)
    setattr(args, "no_tail", not features.tail_enabled)
    setattr(args, "no_gamepad", not features.gamepad_enabled)
    setattr(args, "no_log", not features.logging_enabled)
    setattr(args, "dm_dq_feedforward", features.dm_dq_feedforward_enabled)
    setattr(args, "ff_decouple", features.ff_decouple_enabled)
    setattr(args, "smooth_gait", features.smooth_gait_enabled)

    gait = config.gait
    setattr(args, "height", gait.body_height_m)
    setattr(args, "period", gait.period_s)
    setattr(args, "step_h", gait.step_height_m)
    _set_optional(args, "step_h_front", gait.front_step_height_m)
    setattr(args, "amp_front", gait.amp_front_m)
    setattr(args, "amp_rear", gait.amp_rear_m)
    setattr(args, "stance", gait.stance_ratio)
    setattr(args, "hip_abd", gait.hip_abduction_rad)
    setattr(args, "ramp", gait.ramp_s)
    setattr(args, "fade", gait.fade_s)
    setattr(args, "natural_trot", gait.natural_trot_enabled)
    setattr(args, "natural_soft_trot", gait.natural_soft_trot_enabled)

    control = config.control
    setattr(args, "leg_kp_scale", control.leg_kp_scale)
    setattr(args, "td_kp_scale", control.td_kp_scale)
    setattr(args, "swing_kp_scale", control.swing_kp_scale)
    setattr(args, "td_window", control.td_window_s)
    setattr(args, "grav_scale", control.gravity_scale)
    setattr(args, "max_corr_mm", m_to_mm(control.max_correction_m))
    setattr(args, "imu_slew_mm_s", m_to_mm(control.imu_slew_m_s))
    setattr(args, "yaw_hold_kp", control.yaw_hold_kp)
    setattr(args, "yaw_hold_kd", control.yaw_hold_kd)
    setattr(args, "yaw_hold_limit", control.yaw_hold_limit)

    imu = config.imu
    setattr(args, "imu_predict_ms", s_to_ms(imu.predict_s))
    setattr(args, "imu_predict_max_ms", s_to_ms(imu.predict_max_s))
    setattr(args, "imu_gyro_max_age_ms", s_to_ms(imu.gyro_max_age_s))
    setattr(args, "dynamic_imu_predict", imu.dynamic_predict_enabled)
    setattr(args, "imu_angle_tau_ms", s_to_ms(imu.angle_tau_s))
    setattr(args, "imu_gyro_tau_ms", s_to_ms(imu.gyro_tau_s))
    setattr(args, "imu_kp", imu.kp)
    setattr(args, "imu_softstart_s", imu.softstart_s)
    setattr(args, "roll_trim_mm", m_to_mm(imu.roll_trim_m))
    setattr(args, "pitch_trim_mm", m_to_mm(imu.pitch_trim_m))
    setattr(args, "auto_trim", imu.auto_trim_enabled)
    setattr(args, "auto_trim_rate", imu.auto_trim_rate_m_rad_s)
    setattr(args, "auto_trim_limit_mm", m_to_mm(imu.auto_trim_limit_m))
    setattr(args, "trim_phases", imu.trim_phases)
    setattr(args, "imu_phase_gate", imu.phase_gate_enabled)
    setattr(args, "imu_phase_td_gain", imu.phase_td_gain)
    setattr(args, "imu_phase_swing_gain", imu.phase_swing_gain)

    safety = config.safety
    setattr(args, "bench_max_error_deg", rad_to_deg(safety.bench_max_error_rad))
    setattr(args, "bench_max_tilt_deg", rad_to_deg(safety.bench_max_tilt_rad))
    setattr(args, "bench_max_torque_nm", safety.bench_max_torque_nm)

    dm = config.dm_tarsus
    setattr(args, "dm_kp_fl", dm.kp_fl)
    setattr(args, "dm_kp_fr", dm.kp_fr)
    setattr(args, "dm_kd_fl", dm.kd_fl)
    setattr(args, "dm_kd_fr", dm.kd_fr)
    setattr(args, "tarsus_lead_fl_ms", s_to_ms(dm.lead_fl_s))
    setattr(args, "tarsus_lead_fr_ms", s_to_ms(dm.lead_fr_s))
    setattr(args, "tarsus_lead_max_deg", rad_to_deg(dm.lead_max_rad))
    setattr(args, "dm_dq_max_rps", dm.dq_max_rad_s)

    devtools = config.devtools
    _set_optional(args, "hip_abd_test", devtools.hip_abduction_test_rad)
    _set_optional(args, "leg_pitch_test", devtools.leg_pitch_test_rad)
    _set_optional(args, "calf_pitch_test", devtools.calf_pitch_test_rad)
    setattr(args, "capture_lie_pose", devtools.capture_lie_pose)
    setattr(args, "bench_tarsus_side", devtools.bench_tarsus_side)

    return args


__all__ = ["apply_runtime_config_to_legacy_args"]
