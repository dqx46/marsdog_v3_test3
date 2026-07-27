"""Device path configuration.

The legacy ``bus_config`` module still performs discovery. This wrapper
centralizes access so runtime code no longer imports scattered constants.
"""

from __future__ import annotations

from dataclasses import dataclass

from marsdog_control.config import bus_config as _legacy


@dataclass(frozen=True)
class DeviceConfig:
    lz_can_a: str
    lz_can_b: str
    incos_can: str
    evo_can: str
    dm_can: str
    imu: str
    tail_485: str
    mouth: str
    gamepad: str
    speaker_alsa: str
    baud: int
    imu_baud: int


def get_device_config() -> DeviceConfig:
    return DeviceConfig(
        lz_can_a=_legacy.LZ_CAN_A_DEVICE,
        lz_can_b=_legacy.LZ_CAN_B_DEVICE,
        incos_can=_legacy.INCOS_CAN_DEVICE,
        evo_can=_legacy.EVO_CAN_DEVICE,
        dm_can=_legacy.DM_CAN_DEVICE,
        imu=_legacy.IMU_DEVICE,
        tail_485=_legacy.TAIL_485_DEVICE,
        mouth=_legacy.MOUTH_DEVICE,
        gamepad=_legacy.GAMEPAD_DEVICE,
        speaker_alsa=_legacy.SPEAKER_ALSA_DEVICE,
        baud=_legacy.BAUD,
        imu_baud=_legacy.IMU_BAUD,
    )


__all__ = ["DeviceConfig", "get_device_config"]
