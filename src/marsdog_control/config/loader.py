"""Configuration loaders.

CLI ``argparse.Namespace`` is converted once into typed ``RuntimeConfig``.
Defaults for missing fields come from ``schema.RuntimeConfig`` via
``cli_defaults_from_schema`` — not hand-copied magic numbers.
There is no write-back onto ``args``.
"""

from __future__ import annotations

from argparse import Namespace
from dataclasses import replace
from typing import Any, Optional

from marsdog_control.config.defaults import cli_defaults_from_schema
from marsdog_control.config.schema import (
    ControlConfig,
    DevToolsConfig,
    DmTarsusConfig,
    DynamicsConfig,
    FeatureFlags,
    GaitConfig,
    HardwareConfig,
    ImuConfig,
    LoggingConfig,
    RuntimeConfig,
    SafetyConfig,
)
from marsdog_control.core.units import deg_to_rad, mm_to_m, ms_to_s

_D = RuntimeConfig()
_CLI = cli_defaults_from_schema(_D)


def _get(args: Namespace, name: str, default: Any) -> Any:
    return getattr(args, name, default)


def _optional_float(args: Namespace, name: str) -> Optional[float]:
    value = getattr(args, name, None)
    return None if value is None else float(value)


def runtime_config_from_args(args: Namespace) -> RuntimeConfig:
    """Build typed config from the walking CLI namespace (one-way)."""

    natural_soft_trot = bool(_get(
        args, "natural_soft_trot", _CLI.natural_soft_trot))
    natural_trot = bool(_get(args, "natural_trot", False)) or natural_soft_trot
    # 达妙零点由操作约定保证(上电前手动归零), 自然步态即主动驱动 tarsus。
    dm_active = bool(natural_trot or natural_soft_trot)
    imu_enabled = not bool(_get(args, "no_imu", False))

    features = FeatureFlags(
        imu_enabled=imu_enabled,
        imu_feedback_enabled=imu_enabled,
        gravity_comp_enabled=bool(_get(args, "gravity_comp", _CLI.gravity_comp)),
        vmc_enabled=bool(_get(args, "vmc", _CLI.vmc)),
        wbc_enabled=bool(_get(args, "wbc", False)),
        variable_impedance_enabled=bool(_get(
            args, "var_impedance", _CLI.var_impedance)),
        yaw_hold_enabled=bool(_get(args, "yaw_hold", _CLI.yaw_hold)),
        tail_enabled=not bool(_get(args, "no_tail", False)),
        gamepad_enabled=not bool(_get(args, "no_gamepad", False)),
        logging_enabled=not bool(_get(args, "no_log", False)),
        dm_tarsus_active=dm_active,
        dm_dq_feedforward_enabled=bool(_get(
            args, "dm_dq_feedforward", _CLI.dm_dq_feedforward)),
        ff_decouple_enabled=bool(_get(args, "ff_decouple", _CLI.ff_decouple)),
        smooth_gait_enabled=bool(_get(args, "smooth_gait", _CLI.smooth_gait)),
    )
    # WBC 覆盖 VMC（互斥）
    if features.wbc_enabled and features.vmc_enabled:
        features = replace(features, vmc_enabled=False)

    gait = GaitConfig(
        body_height_m=float(_get(args, "height", _CLI.height)),
        period_s=float(_get(args, "period", _CLI.period)),
        step_height_m=float(_get(args, "step_h", _CLI.step_h)),
        front_step_height_m=_optional_float(args, "step_h_front"),
        amp_front_m=float(_get(args, "amp_front", _CLI.amp_front)),
        amp_rear_m=float(_get(args, "amp_rear", _CLI.amp_rear)),
        stance_ratio=float(_get(args, "stance", _CLI.stance)),
        hip_abduction_rad=float(_get(args, "hip_abd", _CLI.hip_abd)),
        ramp_s=float(_get(args, "ramp", _CLI.ramp)),
        fade_s=float(_get(args, "fade", _CLI.fade)),
        natural_trot_enabled=natural_trot,
        natural_soft_trot_enabled=natural_soft_trot,
    )

    control = ControlConfig(
        leg_kp_scale=float(_get(args, "leg_kp_scale", _CLI.leg_kp_scale)),
        kp_scale=float(_get(args, "kp_scale", _CLI.kp_scale)),
        td_kp_scale=float(_get(args, "td_kp_scale", _CLI.td_kp_scale)),
        swing_kp_scale=float(_get(args, "swing_kp_scale", _CLI.swing_kp_scale)),
        td_window_s=float(_get(args, "td_window", _CLI.td_window)),
        gravity_scale=float(_get(args, "grav_scale", _CLI.grav_scale)),
        max_correction_m=mm_to_m(float(_get(args, "max_corr_mm", _CLI.max_corr_mm))),
        imu_slew_m_s=mm_to_m(float(_get(args, "imu_slew_mm_s", _CLI.imu_slew_mm_s))),
        yaw_hold_kp=float(_get(args, "yaw_hold_kp", _CLI.yaw_hold_kp)),
        yaw_hold_kd=float(_get(args, "yaw_hold_kd", _CLI.yaw_hold_kd)),
        yaw_hold_limit=float(_get(args, "yaw_hold_limit", _CLI.yaw_hold_limit)),
    )

    if features.wbc_enabled:
        # 跟踪与稳姿折中：0.75 半杆 roll ptp~11°；0.70 保留部分 q_err 改善
        control = replace(
            control,
            leg_kp_scale=0.70,
            td_kp_scale=max(control.td_kp_scale, 0.35),
            swing_kp_scale=max(control.swing_kp_scale, 0.85),
        )
    elif features.vmc_enabled:
        control = replace(control, leg_kp_scale=0.5)

    from marsdog_control.config.schema import DynamicsConfig
    _D_dyn = DynamicsConfig()
    dynamics = DynamicsConfig(
        urdf_path=str(_get(args, "urdf_path", _D_dyn.urdf_path)),
        mu=float(_get(args, "wbc_mu", _D_dyn.mu)),
        f_min=float(_get(args, "wbc_f_min", _D_dyn.f_min)),
        f_max=float(_get(args, "wbc_f_max", _D_dyn.f_max)),
        mpc_horizon=int(_get(args, "mpc_horizon", _D_dyn.mpc_horizon)),
        mpc_dt=float(_get(args, "mpc_dt", _D_dyn.mpc_dt)),
        tau_limit_nm=float(_get(args, "wbc_tau_limit", _D_dyn.tau_limit_nm)),
        kp_base_z=float(_get(args, "kp_base_z", _D_dyn.kp_base_z)),
        kd_base_z=float(_get(args, "kd_base_z", _D_dyn.kd_base_z)),
        kp_base_roll=float(_get(args, "kp_base_roll", _D_dyn.kp_base_roll)),
        kd_base_roll=float(_get(args, "kd_base_roll", _D_dyn.kd_base_roll)),
        kp_base_pitch=float(_get(args, "kp_base_pitch", _D_dyn.kp_base_pitch)),
        kd_base_pitch=float(_get(args, "kd_base_pitch", _D_dyn.kd_base_pitch)),
        base_estimate_mode=str(
            _get(args, "base_estimate_mode", _D_dyn.base_estimate_mode)
        ),
        force_lpf_alpha=float(
            _get(args, "force_lpf_alpha", _D_dyn.force_lpf_alpha)
        ),
        contact_edge_blend=float(
            _get(args, "contact_edge_blend", _D_dyn.contact_edge_blend)
        ),
        disable_imu_foot_balance=bool(
            _get(args, "disable_imu_foot_balance", _D_dyn.disable_imu_foot_balance)
        ),
        swing_foot_kp=float(_get(args, "swing_foot_kp", _D_dyn.swing_foot_kp)),
        swing_foot_kd=float(_get(args, "swing_foot_kd", _D_dyn.swing_foot_kd)),
        max_df_dt=float(_get(args, "max_df_dt", _D_dyn.max_df_dt)),
        lateral_vel_damp=float(
            _get(args, "lateral_vel_damp", _D_dyn.lateral_vel_damp)
        ),
        com_y_shift_m=float(_get(args, "com_y_shift_m", _D_dyn.com_y_shift_m)),
        measure_force_weight=float(
            _get(args, "measure_force_weight", _D_dyn.measure_force_weight)
        ),
    )

    imu = ImuConfig(
        predict_s=ms_to_s(float(_get(args, "imu_predict_ms", _CLI.imu_predict_ms))),
        predict_max_s=ms_to_s(float(_get(
            args, "imu_predict_max_ms", _CLI.imu_predict_max_ms))),
        gyro_max_age_s=ms_to_s(float(_get(
            args, "imu_gyro_max_age_ms", _CLI.imu_gyro_max_age_ms))),
        dynamic_predict_enabled=bool(_get(
            args, "dynamic_imu_predict", _CLI.dynamic_imu_predict)),
        angle_tau_s=ms_to_s(float(_get(
            args, "imu_angle_tau_ms", _CLI.imu_angle_tau_ms))),
        gyro_tau_s=ms_to_s(float(_get(
            args, "imu_gyro_tau_ms", _CLI.imu_gyro_tau_ms))),
        kp=float(_get(args, "imu_kp", _CLI.imu_kp)),
        softstart_s=float(_get(args, "imu_softstart_s", _CLI.imu_softstart_s)),
        roll_trim_m=mm_to_m(float(_get(args, "roll_trim_mm", _CLI.roll_trim_mm))),
        pitch_trim_m=mm_to_m(float(_get(args, "pitch_trim_mm", _CLI.pitch_trim_mm))),
        auto_trim_enabled=bool(_get(args, "auto_trim", _CLI.auto_trim)),
        auto_trim_rate_m_rad_s=float(_get(
            args, "auto_trim_rate", _CLI.auto_trim_rate)),
        auto_trim_limit_m=mm_to_m(float(_get(
            args, "auto_trim_limit_mm", _CLI.auto_trim_limit_mm))),
        trim_phases=int(_get(args, "trim_phases", _CLI.trim_phases)),
        phase_gate_enabled=bool(_get(args, "imu_phase_gate", _CLI.imu_phase_gate)),
        phase_td_gain=float(_get(args, "imu_phase_td_gain", _CLI.imu_phase_td_gain)),
        phase_swing_gain=float(_get(
            args, "imu_phase_swing_gain", _CLI.imu_phase_swing_gain)),
    )

    safety = SafetyConfig(
        bench_max_error_rad=deg_to_rad(float(_get(
            args, "bench_max_error_deg", _CLI.bench_max_error_deg))),
        bench_max_tilt_rad=deg_to_rad(float(_get(
            args, "bench_max_tilt_deg", _CLI.bench_max_tilt_deg))),
        bench_max_torque_nm=float(_get(
            args, "bench_max_torque_nm", _CLI.bench_max_torque_nm)),
    )

    dm_tarsus = DmTarsusConfig(
        kp_fl=float(_get(args, "dm_kp_fl", _CLI.dm_kp_fl)),
        kp_fr=float(_get(args, "dm_kp_fr", _CLI.dm_kp_fr)),
        kd_fl=float(_get(args, "dm_kd_fl", _CLI.dm_kd_fl)),
        kd_fr=float(_get(args, "dm_kd_fr", _CLI.dm_kd_fr)),
        lead_fl_s=ms_to_s(float(_get(
            args, "tarsus_lead_fl_ms", _CLI.tarsus_lead_fl_ms))),
        lead_fr_s=ms_to_s(float(_get(
            args, "tarsus_lead_fr_ms", _CLI.tarsus_lead_fr_ms))),
        lead_max_rad=deg_to_rad(float(_get(
            args, "tarsus_lead_max_deg", _CLI.tarsus_lead_max_deg))),
        dq_max_rad_s=float(_get(args, "dm_dq_max_rps", _CLI.dm_dq_max_rps)),
    )

    devtools = DevToolsConfig(
        hip_abduction_test_rad=_optional_float(args, "hip_abd_test"),
        leg_pitch_test_rad=_optional_float(args, "leg_pitch_test"),
        calf_pitch_test_rad=_optional_float(args, "calf_pitch_test"),
        capture_lie_pose=bool(_get(args, "capture_lie_pose", False)),
        bench_tarsus_side=getattr(args, "bench_tarsus_side", None),
    )

    return RuntimeConfig(
        features=features,
        hardware=HardwareConfig(),
        gait=gait,
        control=control,
        dynamics=dynamics,
        imu=imu,
        safety=safety,
        dm_tarsus=dm_tarsus,
        logging=LoggingConfig(enabled=features.logging_enabled),
        devtools=devtools,
    )


__all__ = ["runtime_config_from_args"]
