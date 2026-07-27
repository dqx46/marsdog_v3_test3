"""Configuration exports for joints, buses, devices, and runtime config."""

from marsdog_control.config.bootstrap import (
    ConfigBootstrapResult,
    bootstrap_runtime_config,
)
from marsdog_control.config.devices import DeviceConfig, get_device_config
from marsdog_control.config.defaults import CLI, default_runtime_config, cli_defaults_from_schema
from marsdog_control.config.gait_tuning import GAIT, GaitCliDefaults, print_tuning_banner
from marsdog_control.config.gains import JOINT_GAINS
from marsdog_control.config.legacy_bridge import apply_runtime_config_to_legacy_args
from marsdog_control.config.loader import runtime_config_from_args
from marsdog_control.config.report import (
    runtime_config_summary,
    runtime_config_to_dict,
)
from marsdog_control.config.schema import (
    ControlConfig,
    DevToolsConfig,
    DmTarsusConfig,
    FeatureFlags,
    GaitConfig,
    HardwareConfig,
    ImuConfig,
    LoggingConfig,
    RuntimeConfig,
    SafetyConfig,
)
from marsdog_control.config.validate import (
    ConfigValidationError,
    ValidationResult,
    validate_runtime_config,
)

__all__ = [
    "CLI",
    "ConfigBootstrapResult",
    "ConfigValidationError",
    "ControlConfig",
    "DevToolsConfig",
    "DeviceConfig",
    "DmTarsusConfig",
    "FeatureFlags",
    "GAIT",
    "GaitCliDefaults",
    "GaitConfig",
    "HardwareConfig",
    "ImuConfig",
    "JOINT_GAINS",
    "LoggingConfig",
    "RuntimeConfig",
    "SafetyConfig",
    "ValidationResult",
    "apply_runtime_config_to_legacy_args",
    "bootstrap_runtime_config",
    "cli_defaults_from_schema",
    "default_runtime_config",
    "get_device_config",
    "print_tuning_banner",
    "runtime_config_from_args",
    "runtime_config_summary",
    "runtime_config_to_dict",
    "validate_runtime_config",
]
