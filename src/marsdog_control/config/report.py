"""Human-readable summary of a :class:`RuntimeConfig`.

Printing the effective, typed config at startup gives operators visibility into
what is actually in effect (units included) and makes runs reproducible.
"""

from __future__ import annotations

from dataclasses import asdict
from typing import Dict

from marsdog_control.config.schema import RuntimeConfig
from marsdog_control.config.validate import ValidationResult


def runtime_config_to_dict(config: RuntimeConfig) -> Dict:
    """Return a plain dict of the config, dropping non-serializable devices."""
    data = asdict(config)
    data.get("hardware", {}).pop("devices", None)
    return data


def _enabled_flags(config: RuntimeConfig) -> str:
    flags = config.features
    on = [name for name, value in asdict(flags).items() if value]
    return ", ".join(on) if on else "(none)"


def runtime_config_summary(config: RuntimeConfig,
                           result: ValidationResult | None = None) -> str:
    """Build a compact multi-line summary string (internal SI units)."""
    gait = config.gait
    control = config.control
    imu = config.imu
    dm = config.dm_tarsus

    lines = [
        "===== Marsdog RuntimeConfig (SI: m/rad/s) =====",
        f"features    : {_enabled_flags(config)}",
        (f"gait        : height={gait.body_height_m:.3f}m period={gait.period_s:.3f}s "
         f"step_h={gait.step_height_m:.3f}m stance={gait.stance_ratio:.2f}"),
        (f"control     : leg_kp_scale={control.leg_kp_scale:.2f} "
         f"grav_scale={control.gravity_scale:.2f} "
         f"max_corr={control.max_correction_m*1000:.1f}mm"),
        (f"imu         : predict={imu.predict_s*1000:.1f}ms "
         f"kp={imu.kp:.3f} softstart={imu.softstart_s:.2f}s "
         f"auto_trim={imu.auto_trim_enabled}"),
        (f"dm_tarsus   : kp=({dm.kp_fl:.0f},{dm.kp_fr:.0f}) "
         f"kd=({dm.kd_fl:.1f},{dm.kd_fr:.1f}) "
         f"dq_max={dm.dq_max_rad_s:.2f}rad/s"),
    ]

    if result is not None:
        for warning in result.warnings:
            lines.append(f"  [warn]  {warning}")
        for error in result.errors:
            lines.append(f"  [ERROR] {error}")
    lines.append("===============================================")
    return "\n".join(lines)


__all__ = ["runtime_config_summary", "runtime_config_to_dict"]
