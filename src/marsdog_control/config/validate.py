"""Startup validation for :class:`RuntimeConfig`.

The goal is to catch dangerous or inconsistent parameter combinations before
the robot moves. Validation is split into two levels:

- ``errors``: unsafe or contradictory values that should stop startup.
- ``warnings``: unusual values that are allowed but worth flagging.

This addresses the "parameters are easy to get wrong" problem by turning silent
misconfiguration into an explicit, early, human-readable report.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List

from marsdog_control.config.schema import RuntimeConfig
from marsdog_control.core.units import deg_to_rad


class ConfigValidationError(ValueError):
    """Raised when a config has fatal problems and ``raise_on_error`` is set."""


@dataclass
class ValidationResult:
    errors: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return not self.errors

    def error(self, message: str) -> None:
        self.errors.append(message)

    def warn(self, message: str) -> None:
        self.warnings.append(message)


def _check_positive(result: ValidationResult, name: str, value: float) -> None:
    if value <= 0.0:
        result.error(f"{name} 必须 > 0，当前 {value}")


def _check_range(result: ValidationResult, name: str, value: float,
                 low: float, high: float) -> None:
    if not (low <= value <= high):
        result.error(f"{name} 应在 [{low}, {high}]，当前 {value}")


def _warn_range(result: ValidationResult, name: str, value: float,
                low: float, high: float) -> None:
    if not (low <= value <= high):
        result.warn(f"{name} 建议在 [{low}, {high}]，当前 {value}")


def validate_runtime_config(config: RuntimeConfig,
                            raise_on_error: bool = False) -> ValidationResult:
    """Validate a runtime config. Units are internal SI (m, rad, s)."""

    result = ValidationResult()

    gait = config.gait
    _check_positive(result, "gait.body_height_m", gait.body_height_m)
    _check_positive(result, "gait.period_s", gait.period_s)
    _check_range(result, "gait.stance_ratio", gait.stance_ratio, 0.05, 0.95)
    if gait.amp_front_m < 0.0:
        result.error(f"gait.amp_front_m 不能为负，当前 {gait.amp_front_m}")
    if gait.amp_rear_m < 0.0:
        result.error(f"gait.amp_rear_m 不能为负，当前 {gait.amp_rear_m}")
    if gait.step_height_m < 0.0:
        result.error(f"gait.step_height_m 不能为负，当前 {gait.step_height_m}")
    if gait.front_step_height_m is not None and gait.front_step_height_m < 0.0:
        result.error(
            f"gait.front_step_height_m 不能为负，当前 {gait.front_step_height_m}")
    if gait.ramp_s < 0.0:
        result.error(f"gait.ramp_s 不能为负，当前 {gait.ramp_s}")
    if gait.fade_s < 0.0:
        result.error(f"gait.fade_s 不能为负，当前 {gait.fade_s}")
    _warn_range(result, "gait.body_height_m", gait.body_height_m, 0.12, 0.30)
    _warn_range(result, "gait.period_s", gait.period_s, 0.4, 1.6)
    _warn_range(result, "gait.step_height_m", gait.step_height_m, 0.0, 0.06)
    _warn_range(result, "gait.amp_front_m", gait.amp_front_m, 0.0, 0.06)
    _warn_range(result, "gait.amp_rear_m", gait.amp_rear_m, 0.0, 0.08)

    control = config.control
    _check_range(result, "control.leg_kp_scale", control.leg_kp_scale, 0.0, 3.0)
    _check_range(result, "control.kp_scale", control.kp_scale, 0.0, 3.0)
    _check_range(result, "control.td_kp_scale", control.td_kp_scale, 0.0, 2.0)
    _check_range(result, "control.swing_kp_scale", control.swing_kp_scale, 0.0, 2.0)
    _check_positive(result, "control.td_window_s", control.td_window_s)
    _check_range(result, "control.yaw_hold_limit", control.yaw_hold_limit, 0.0, 1.0)
    if control.max_correction_m < 0.0:
        result.error(
            f"control.max_correction_m 不能为负，当前 {control.max_correction_m}")
    if control.imu_slew_m_s < 0.0:
        result.error(f"control.imu_slew_m_s 不能为负，当前 {control.imu_slew_m_s}")
    _warn_range(result, "control.max_correction_m", control.max_correction_m,
                0.0, 0.05)
    _warn_range(result, "control.gravity_scale", control.gravity_scale, -1.5, 1.5)

    imu = config.imu
    if imu.predict_s < 0.0:
        result.error(f"imu.predict_s 不能为负，当前 {imu.predict_s}")
    if imu.predict_max_s < imu.predict_s:
        result.error(
            f"imu.predict_max_s ({imu.predict_max_s}) 不能小于 "
            f"imu.predict_s ({imu.predict_s})")
    _check_positive(result, "imu.gyro_max_age_s", imu.gyro_max_age_s)
    if imu.angle_tau_s < 0.0:
        result.error(f"imu.angle_tau_s 不能为负，当前 {imu.angle_tau_s}")
    if imu.gyro_tau_s < 0.0:
        result.error(f"imu.gyro_tau_s 不能为负，当前 {imu.gyro_tau_s}")
    if imu.softstart_s < 0.0:
        result.error(f"imu.softstart_s 不能为负，当前 {imu.softstart_s}")
    if imu.auto_trim_limit_m < 0.0:
        result.error(f"imu.auto_trim_limit_m 不能为负，当前 {imu.auto_trim_limit_m}")
    if imu.trim_phases < 1:
        result.error(f"imu.trim_phases 必须 >= 1，当前 {imu.trim_phases}")
    _warn_range(result, "imu.predict_s", imu.predict_s, 0.0, 0.05)
    _warn_range(result, "imu.kp", imu.kp, 0.0, 0.2)
    _warn_range(result, "imu.angle_tau_s", imu.angle_tau_s, 0.0, 0.20)
    _warn_range(result, "imu.gyro_tau_s", imu.gyro_tau_s, 0.0, 0.20)

    dm = config.dm_tarsus
    for gain_name, gain in (("kp_fl", dm.kp_fl), ("kp_fr", dm.kp_fr)):
        _check_range(result, f"dm_tarsus.{gain_name}", gain, 0.0, 500.0)
    for gain_name, gain in (("kd_fl", dm.kd_fl), ("kd_fr", dm.kd_fr)):
        _check_range(result, f"dm_tarsus.{gain_name}", gain, 0.0, 50.0)
    if dm.lead_fl_s < 0.0:
        result.error(f"dm_tarsus.lead_fl_s 不能为负，当前 {dm.lead_fl_s}")
    if dm.lead_fr_s < 0.0:
        result.error(f"dm_tarsus.lead_fr_s 不能为负，当前 {dm.lead_fr_s}")
    _check_range(result, "dm_tarsus.lead_max_rad", dm.lead_max_rad, 0.0, deg_to_rad(15.0))
    if dm.dq_max_rad_s < 0.0:
        result.error(f"dm_tarsus.dq_max_rad_s 不能为负，当前 {dm.dq_max_rad_s}")
    _warn_range(result, "dm_tarsus.lead_fl_s", dm.lead_fl_s, 0.0, 0.10)
    _warn_range(result, "dm_tarsus.lead_fr_s", dm.lead_fr_s, 0.0, 0.10)
    _warn_range(result, "dm_tarsus.dq_max_rad_s", dm.dq_max_rad_s, 0.0, 10.0)

    safety = config.safety
    _check_positive(result, "safety.bench_max_error_rad", safety.bench_max_error_rad)
    _check_positive(result, "safety.bench_max_tilt_rad", safety.bench_max_tilt_rad)
    _check_positive(result, "safety.bench_max_torque_nm", safety.bench_max_torque_nm)

    features = config.features
    if features.imu_feedback_enabled and not features.imu_enabled:
        result.warn("imu_feedback_enabled=True 但 imu_enabled=False，反馈将无数据")

    if raise_on_error and result.errors:
        raise ConfigValidationError("; ".join(result.errors))

    return result


__all__ = [
    "ConfigValidationError",
    "ValidationResult",
    "validate_runtime_config",
]
