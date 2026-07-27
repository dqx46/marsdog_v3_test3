"""Default runtime configuration and CLI defaults derived from schema.

``RuntimeConfig`` field defaults are the single source of truth for migrated
knobs. ``walk_cli`` and ``loader`` must read from here — do not hand-copy
magic numbers into argparse ``default=`` or ``_get(..., default)``.
"""

from __future__ import annotations

from types import SimpleNamespace

from typing import Optional

from marsdog_control.config.schema import RuntimeConfig
from marsdog_control.core.units import m_to_mm, rad_to_deg, s_to_ms


def default_runtime_config() -> RuntimeConfig:
    return RuntimeConfig()


def cli_defaults_from_schema(config: Optional[RuntimeConfig] = None) -> SimpleNamespace:
    """CLI-unit defaults mirrored from ``RuntimeConfig`` (mm/ms/deg where needed)."""
    c = config if config is not None else RuntimeConfig()
    f, g, ctrl, imu, safety, dm = (
        c.features, c.gait, c.control, c.imu, c.safety, c.dm_tarsus,
    )
    return SimpleNamespace(
        height=g.body_height_m,
        period=g.period_s,
        step_h=g.step_height_m,
        amp_front=g.amp_front_m,
        amp_rear=g.amp_rear_m,
        stance=g.stance_ratio,
        hip_abd=g.hip_abduction_rad,
        ramp=g.ramp_s,
        fade=g.fade_s,
        natural_soft_trot=g.natural_soft_trot_enabled,
        leg_kp_scale=ctrl.leg_kp_scale,
        kp_scale=ctrl.kp_scale,
        td_kp_scale=ctrl.td_kp_scale,
        swing_kp_scale=ctrl.swing_kp_scale,
        td_window=ctrl.td_window_s,
        grav_scale=ctrl.gravity_scale,
        max_corr_mm=m_to_mm(ctrl.max_correction_m),
        imu_slew_mm_s=m_to_mm(ctrl.imu_slew_m_s),
        yaw_hold_kp=ctrl.yaw_hold_kp,
        yaw_hold_kd=ctrl.yaw_hold_kd,
        yaw_hold_limit=ctrl.yaw_hold_limit,
        yaw_hold=f.yaw_hold_enabled,
        gravity_comp=f.gravity_comp_enabled,
        vmc=f.vmc_enabled,
        var_impedance=f.variable_impedance_enabled,
        dm_dq_feedforward=f.dm_dq_feedforward_enabled,
        ff_decouple=f.ff_decouple_enabled,
        smooth_gait=f.smooth_gait_enabled,
        imu_predict_ms=s_to_ms(imu.predict_s),
        imu_predict_max_ms=s_to_ms(imu.predict_max_s),
        imu_gyro_max_age_ms=s_to_ms(imu.gyro_max_age_s),
        dynamic_imu_predict=imu.dynamic_predict_enabled,
        imu_angle_tau_ms=s_to_ms(imu.angle_tau_s),
        imu_gyro_tau_ms=s_to_ms(imu.gyro_tau_s),
        imu_kp=imu.kp,
        imu_softstart_s=imu.softstart_s,
        roll_trim_mm=m_to_mm(imu.roll_trim_m),
        pitch_trim_mm=m_to_mm(imu.pitch_trim_m),
        auto_trim=imu.auto_trim_enabled,
        auto_trim_rate=imu.auto_trim_rate_m_rad_s,
        auto_trim_limit_mm=m_to_mm(imu.auto_trim_limit_m),
        trim_phases=imu.trim_phases,
        imu_phase_gate=imu.phase_gate_enabled,
        imu_phase_td_gain=imu.phase_td_gain,
        imu_phase_swing_gain=imu.phase_swing_gain,
        bench_max_error_deg=rad_to_deg(safety.bench_max_error_rad),
        bench_max_tilt_deg=rad_to_deg(safety.bench_max_tilt_rad),
        bench_max_torque_nm=safety.bench_max_torque_nm,
        dm_kp_fl=dm.kp_fl,
        dm_kp_fr=dm.kp_fr,
        dm_kd_fl=dm.kd_fl,
        dm_kd_fr=dm.kd_fr,
        tarsus_lead_fl_ms=s_to_ms(dm.lead_fl_s),
        tarsus_lead_fr_ms=s_to_ms(dm.lead_fr_s),
        tarsus_lead_max_deg=rad_to_deg(dm.lead_max_rad),
        dm_dq_max_rps=dm.dq_max_rad_s,
    )


# Module-level snapshot for argparse default= (schema is SSOT).
CLI = cli_defaults_from_schema()


__all__ = [
    "CLI",
    "cli_defaults_from_schema",
    "default_runtime_config",
]
