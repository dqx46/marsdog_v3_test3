"""Shared real-hardware bring-up for walk + menu tools.

SSOT path (same as ``run_walk``):
  ``get_device_config()`` / ``bus_config``
  → ``bringup_motors_and_board``
  → ``WalkServices`` bound to ``WalkRuntimeState``
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any, Optional

from marsdog_control.compat import legacy_dir
from marsdog_control.config.devices import get_device_config
from marsdog_control.config.joints import (
    ALL_IDS,
    DM_MASTER_ID_BY_SLAVE,
    INCOS_CAN_IDS,
    JOINT_BY_ID,
    JOINT_MAP,
)
from marsdog_control.hardware.board import RkMotorBoard
from marsdog_control.hardware.motors.damiao import MotorDamiao
from marsdog_control.hardware.motors.evo import MotorEvo
from marsdog_control.hardware.motors.incos import MotorIncos
from marsdog_control.hardware.motors.lingzu import MotorLz
from marsdog_control.runtime.walk_bringup import (
    HardwareSession,
    bringup_imu,
    bringup_motors_and_board,
)
from marsdog_control.runtime.walk_services import WalkServices
from marsdog_control.runtime.walk_state import WalkRuntimeState


@dataclass
class WalkHardwareBundle:
    """Live handles after a successful ``open_walk_hardware``."""

    lz: Any
    evo: Any
    dm: Any
    incos: Any
    board: Any
    online: list
    svc: WalkServices
    session: HardwareSession
    imu: Any = None
    imu_ok: bool = False


def open_walk_hardware(
    runtime_state: WalkRuntimeState,
    *,
    clear_fault: bool = False,
    control_hz: float = 200.0,
    with_imu: bool = False,
    require_imu: bool = False,
    clock=time,
) -> Optional[WalkHardwareBundle]:
    """Open the five walk buses and bind ``WalkServices`` to ``runtime_state``.

    Returns ``None`` when bring-up finds no online motors (bringup already
    attempted shutdown). Device paths come from ``get_device_config()``.
    """
    dev = get_device_config()
    real_joints = [j for j in JOINT_MAP if j.bus != "none"]
    dm_joints = [j for j in JOINT_MAP if j.mtype == "dm"]

    svc = WalkServices(
        runtime_state=runtime_state,
        real_joints=real_joints,
        resource_dir=str(legacy_dir()),
        control_hz=float(control_hz),
        clock=clock,
    )

    imu = None
    imu_ok = False
    if with_imu:
        from marsdog_control.hardware.sensors.imu_wt901 import ImuWT901

        imu, imu_ok = bringup_imu(
            imu_cls=ImuWT901,
            imu_device=dev.imu,
            imu_baud=dev.imu_baud,
            angle_tau_s=0.0,
            gyro_tau_s=0.0,
            require_imu=require_imu,
        )

    hw = bringup_motors_and_board(
        motor_lz_cls=MotorLz,
        motor_evo_cls=MotorEvo,
        motor_damiao_cls=MotorDamiao,
        motor_incos_cls=MotorIncos,
        board_cls=RkMotorBoard,
        lz_serial_device=dev.lz_can_b,
        lz_can1_device=dev.lz_can_a,
        evo_can0_device=dev.evo_can,
        dm_can_device=dev.dm_can,
        incos_can_device=dev.incos_can,
        baud=dev.baud,
        joint_map=JOINT_MAP,
        dm_joints=dm_joints,
        dm_master_id_by_slave=DM_MASTER_ID_BY_SLAVE,
        incos_can_ids=INCOS_CAN_IDS,
        joint_by_id=JOINT_BY_ID,
        all_ids=ALL_IDS,
        shutdown_motors=svc.shutdown_motors,
        clock=clock,
        clear_fault=clear_fault,
    )
    if hw is None:
        return None

    svc.board = hw.board
    runtime_state.board = hw.board
    runtime_state.dm.fixed_targets.clear()
    runtime_state.dm.fixed_targets.update(hw.dm_fixed_targets)

    return WalkHardwareBundle(
        lz=hw.lz,
        evo=hw.evo,
        dm=hw.dm,
        incos=hw.incos,
        board=hw.board,
        online=hw.online,
        svc=svc,
        session=hw,
        imu=imu,
        imu_ok=imu_ok,
    )


__all__ = [
    "WalkHardwareBundle",
    "open_walk_hardware",
]
