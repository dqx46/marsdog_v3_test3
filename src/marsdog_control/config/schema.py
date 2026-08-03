"""Typed runtime configuration for Marsdog control.

``RuntimeConfig`` (and its nested dataclasses) are the single source of truth
for migrated knobs. The walk CLI converts ``argparse.Namespace`` → config once
at the boundary; there is no write-back onto ``args``.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

from marsdog_control.config.devices import DeviceConfig, get_device_config
from marsdog_control.core.units import deg_to_rad


def _default_urdf_path() -> str:
    import os
    # schema.py lives in src/marsdog_control/config/ → repo root is ../../..
    return os.path.abspath(
        os.path.join(
            os.path.dirname(__file__),
            "../../../marsdog/urdf/marsdog.urdf",
        )
    )


@dataclass(frozen=True)
class FeatureFlags:
    # TEMP: 默认关闭 IMU 足高补偿；需要时 CLI 传 --imu
    imu_enabled: bool = False
    imu_feedback_enabled: bool = False
    gravity_comp_enabled: bool = True
    vmc_enabled: bool = False
    wbc_enabled: bool = False
    variable_impedance_enabled: bool = False
    yaw_hold_enabled: bool = False
    tail_enabled: bool = True
    gamepad_enabled: bool = True
    logging_enabled: bool = True
    dm_tarsus_active: bool = True
    dm_dq_feedforward_enabled: bool = False
    ff_decouple_enabled: bool = False
    smooth_gait_enabled: bool = False


@dataclass(frozen=True)
class HardwareConfig:
    devices: DeviceConfig = field(default_factory=get_device_config)
    include_dm: bool = True


@dataclass(frozen=True)
class GaitConfig:
    body_height_m: float = 0.25
    # SoftTrot SSOT (NATURAL_SOFT_TROT): 2026-08-03 lock D
    period_s: float = 1.05
    step_height_m: float = 0.024
    front_step_height_m: Optional[float] = 0.020
    amp_front_m: float = 0.022
    amp_rear_m: float = 0.030
    stance_ratio: float = 0.72
    hip_abduction_rad: float = 0.08
    ramp_s: float = 3.5
    fade_s: float = 3.0
    natural_trot_enabled: bool = True
    natural_soft_trot_enabled: bool = True


@dataclass(frozen=True)
class ControlConfig:
    # SoftTrot real default 0.90 (lock D): slight compliance vs raw JOINT_GAINS.
    # Jump / spot may still override transiently; brand tables stay at native units.
    leg_kp_scale: float = 0.90
    kp_scale: float = 1.0
    td_kp_scale: float = 0.4
    swing_kp_scale: float = 0.7
    td_window_s: float = 0.15
    gravity_scale: float = 1.0
    max_correction_m: float = 0.020
    imu_slew_m_s: float = 0.0
    yaw_hold_kp: float = 0.03
    yaw_hold_kd: float = 0.010
    yaw_hold_limit: float = 0.4


@dataclass(frozen=True)
class DynamicsConfig:
    """SRB-MPC + WBC knobs (simulation-first dynamics stack)."""

    urdf_path: str = field(default_factory=_default_urdf_path)
    mu: float = 0.6
    f_min: float = 2.0
    f_max: float = 80.0
    mpc_horizon: int = 10
    mpc_dt: float = 0.03
    # MPC 求解周期(s): 0=每 tick 解(200Hz, 计算重, 仿真会被拖成慢放);
    # 0.02=50Hz, 中间 hold+LPF。仿真/真机(RK3588)统一用它保证 1× 实时,
    # 从而开/不开 WBC 的仿真步频一致(见 ForcePlanner)。
    mpc_period_s: float = 0.02
    tau_limit_nm: float = 25.0
    # WBC 关节力矩输出增益（QP 限幅之后再乘；实机先用 0.5 软化）
    wbc_tau_scale: float = 0.5
    kp_base_z: float = 30.0
    kd_base_z: float = 10.0
    kp_base_roll: float = 85.0  # roll-first; cut diagonal-trot sway (target pk≤5.5°)
    kd_base_roll: float = 24.0
    kp_base_pitch: float = 35.0
    kd_base_pitch: float = 8.0
    # Default estimator for sim/real parity; "truth" = MuJoCo vel_xyz (debug)
    base_estimate_mode: str = "estimator"
    # Force smoothing / contact edge blend (lower alpha = heavier LPF)
    # Soft-land: wider edge + heavier LPF + tighter rate limit cut TD/LO dFz
    # (telemetry: rear edge |dFz| was ~1100 N/s @ edge=0.06 / alpha=0.18 / max_df=2500)
    force_lpf_alpha: float = 0.10
    contact_edge_blend: float = 0.14  # wider LO/TD blend → less roll kick at diagonal switch
    disable_imu_foot_balance: bool = True  # avoid dual attitude loops with WBC
    swing_foot_kp: float = 45.0
    swing_foot_kd: float = 6.0
    max_df_dt: float = 650.0  # slightly softer force edges
    lateral_vel_damp: float = 18.0
    com_y_shift_m: float = 0.0  # zero CoM kick; diagonal shift was feeding roll
    measure_force_weight: float = 0.15  # kinematic meas → force_scale only

@dataclass(frozen=True)
class ImuConfig:
    predict_s: float = 0.0
    predict_max_s: float = 0.080
    gyro_max_age_s: float = 0.030
    dynamic_predict_enabled: bool = False
    angle_tau_s: float = 0.025
    gyro_tau_s: float = 0.015
    kp: float = 0.05
    softstart_s: float = 0.0
    roll_trim_m: float = 0.0
    pitch_trim_m: float = 0.0
    # auto-trim/ILC 整机调平学习已移除；字段保留仅兼容 CLI。
    auto_trim_enabled: bool = False
    auto_trim_rate_m_rad_s: float = 0.08
    auto_trim_limit_m: float = 0.012
    trim_phases: int = 1
    phase_gate_enabled: bool = False
    phase_td_gain: float = 0.35
    phase_swing_gain: float = 0.70


@dataclass(frozen=True)
class SafetyConfig:
    bench_max_error_rad: float = deg_to_rad(8.0)
    bench_max_tilt_rad: float = deg_to_rad(8.0)
    bench_max_torque_nm: float = 5.0


@dataclass(frozen=True)
class DmTarsusConfig:
    kp_fl: float = 220.0
    kp_fr: float = 220.0
    kd_fl: float = 10.0
    kd_fr: float = 10.0
    lead_fl_s: float = 0.0
    lead_fr_s: float = 0.0
    lead_max_rad: float = deg_to_rad(3.0)
    dq_max_rad_s: float = 3.0


@dataclass(frozen=True)
class LoggingConfig:
    enabled: bool = True
    directory: Optional[str] = None


@dataclass(frozen=True)
class DevToolsConfig:
    hip_abduction_test_rad: Optional[float] = None
    leg_pitch_test_rad: Optional[float] = None
    calf_pitch_test_rad: Optional[float] = None
    capture_lie_pose: bool = False
    bench_tarsus_side: Optional[str] = None


@dataclass(frozen=True)
class RuntimeConfig:
    features: FeatureFlags = field(default_factory=FeatureFlags)
    hardware: HardwareConfig = field(default_factory=HardwareConfig)
    gait: GaitConfig = field(default_factory=GaitConfig)
    control: ControlConfig = field(default_factory=ControlConfig)
    dynamics: DynamicsConfig = field(default_factory=DynamicsConfig)
    imu: ImuConfig = field(default_factory=ImuConfig)
    safety: SafetyConfig = field(default_factory=SafetyConfig)
    dm_tarsus: DmTarsusConfig = field(default_factory=DmTarsusConfig)
    logging: LoggingConfig = field(default_factory=LoggingConfig)
    devtools: DevToolsConfig = field(default_factory=DevToolsConfig)


__all__ = [
    "ControlConfig",
    "DynamicsConfig",
    "DevToolsConfig",
    "DmTarsusConfig",
    "FeatureFlags",
    "GaitConfig",
    "HardwareConfig",
    "ImuConfig",
    "LoggingConfig",
    "RuntimeConfig",
    "SafetyConfig",
]
