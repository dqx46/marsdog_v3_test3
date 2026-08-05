"""Dynamics mixins aggregator — keep import path stable.

Prefer importing leaf modules:
``executor_pin`` / ``executor_wbc_{contact,qp,apply}`` / ``executor_vmc`` /
``executor_telemetry``. ``ExecutorDynamicsMixin`` remains the single base for
``CommandExecutor``; ``executor_wbc`` is a thin pipeline orchestrator.
"""

from __future__ import annotations

from marsdog_control.control.executor_pin import ExecutorPinMixin
from marsdog_control.control.executor_telemetry import ExecutorTelemetryMixin
from marsdog_control.control.executor_vmc import ExecutorVmcMixin
from marsdog_control.control.executor_wbc import ExecutorWbcMixin


class ExecutorDynamicsMixin(
    ExecutorPinMixin,
    ExecutorWbcMixin,
    ExecutorVmcMixin,
    ExecutorTelemetryMixin,
):
    """Composition of pin / WBC / VMC / telemetry private methods."""


__all__ = ["ExecutorDynamicsMixin"]
