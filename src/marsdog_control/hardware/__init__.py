"""Hardware aggregation and low-level drivers.

Keep this package import lightweight: diagnostics tools (static_test / usb_probe)
only need motors.can_serial and must not pull pinocchio via board/executor.
"""

from __future__ import annotations

__all__ = [
    "BoardOptions",
    "MotorBoard",
    "RkMotorBoard",
    "HardwareOptions",
    "RobotHardware",
]


def __getattr__(name: str):
    if name in ("BoardOptions", "MotorBoard", "RkMotorBoard"):
        from marsdog_control.hardware.board import (
            BoardOptions,
            MotorBoard,
            RkMotorBoard,
        )
        return {
            "BoardOptions": BoardOptions,
            "MotorBoard": MotorBoard,
            "RkMotorBoard": RkMotorBoard,
        }[name]
    if name in ("HardwareOptions", "RobotHardware"):
        from marsdog_control.hardware.robot_hw import HardwareOptions, RobotHardware
        return {
            "HardwareOptions": HardwareOptions,
            "RobotHardware": RobotHardware,
        }[name]
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
