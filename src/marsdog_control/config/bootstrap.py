"""Runtime config bootstrap for walk and tooling entry points.

Single-source flow (no legacy write-back):

1. Build ``RuntimeConfig`` from CLI ``Namespace`` (boundary only).
2. Validate the typed config.
3. Print the effective config summary.
4. Attach ``args._runtime_config`` for the boundary; do **not** mirror
   fields back onto ``args``. Downstream must read ``RuntimeConfig``.

Validation or build failure is FATAL for walk bring-up.
"""

from __future__ import annotations

from argparse import Namespace
from dataclasses import dataclass
from typing import Callable, Optional

from marsdog_control.config.loader import runtime_config_from_args
from marsdog_control.config.report import runtime_config_summary
from marsdog_control.config.schema import RuntimeConfig
from marsdog_control.config.validate import ValidationResult, validate_runtime_config


@dataclass(frozen=True)
class ConfigBootstrapResult:
    config: Optional[RuntimeConfig]
    validation: Optional[ValidationResult]
    fatal: bool = False
    failed: bool = False
    error: Optional[Exception] = None


def bootstrap_runtime_config(
    args: Namespace,
    emit: Callable[[str], None] = print,
) -> ConfigBootstrapResult:
    """Build, validate, and report runtime config (one-way CLI → config).

    On success, sets ``args._runtime_config``. Does not mutate CLI field values.
    On validation errors or exceptions, returns ``fatal=True`` so callers abort.
    """

    try:
        config = runtime_config_from_args(args)
        validation = validate_runtime_config(config)
        emit(runtime_config_summary(config, validation))

        setattr(args, "_runtime_config_error", bool(validation.errors))
        setattr(args, "_runtime_config", config)

        if validation.errors:
            emit("[CONFIG] 检测到致命配置问题，请在上机前修正上述 [ERROR] 项。")
            return ConfigBootstrapResult(
                config=config,
                validation=validation,
                fatal=True,
            )

        return ConfigBootstrapResult(config=config, validation=validation)
    except Exception as exc:  # noqa: BLE001
        setattr(args, "_runtime_config_error", True)
        setattr(args, "_runtime_config", None)
        emit(f"[CONFIG] typed config 构建失败（启动中止）: {exc}")
        return ConfigBootstrapResult(
            config=None,
            validation=None,
            fatal=True,
            failed=True,
            error=exc,
        )


__all__ = ["ConfigBootstrapResult", "bootstrap_runtime_config"]
