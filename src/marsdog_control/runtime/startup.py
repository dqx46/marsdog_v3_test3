"""Startup helpers for the new runtime architecture."""

from __future__ import annotations

from marsdog_control.hardware import RobotHardware


def start_hardware(hardware: RobotHardware) -> RobotHardware:
    hardware.start()
    return hardware


__all__ = ["start_hardware"]
