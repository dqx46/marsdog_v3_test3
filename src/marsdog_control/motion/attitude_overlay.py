"""Runtime gate for Soft kinematic attitude overlays.

``AttitudeOwner.WBC`` owns body attitude via QP/base PD. Soft ``anti_roll`` /
``trot_roll_ff`` / spine / IMU ``swing_level`` must not stack on top.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

from marsdog_control.config.control_policies import AttitudeOwner


@dataclass
class AttitudeOverlayGate:
    """Session attitude owner → which Soft roll / spine / IMU patches may run."""

    attitude: AttitudeOwner

    def allows_kinematic_roll_patch(self) -> bool:
        """anti_roll / trot_roll_ff foot-Z patches (open-loop posture)."""
        # WBC already regulates roll; stacking Soft anti_roll fights it.
        return self.attitude is not AttitudeOwner.WBC

    def allows_spine(self) -> bool:
        """Torso spine yaw/roll — same mutex as Soft roll patches (off under WBC)."""
        return self.allows_kinematic_roll_patch()

    def allows_imu_prelevel(self) -> bool:
        """swing_level IMU prelevel weight — only with AttitudeOwner.IMU."""
        return self.attitude is AttitudeOwner.IMU

    def allows_ff_decouple(self) -> bool:
        """Expected-roll feedforward subtract for IMU residual loop."""
        return self.attitude is AttitudeOwner.IMU

    def gate_anti_roll(self, value: float) -> float:
        return float(value) if self.allows_kinematic_roll_patch() else 0.0

    def gate_roll_ff_deg(self, neg_deg: float, pos_deg: float) -> Tuple[float, float]:
        if self.allows_kinematic_roll_patch():
            return (float(neg_deg), float(pos_deg))
        return (0.0, 0.0)

    def gate_swing_level(self, value: float) -> float:
        return float(value) if self.allows_imu_prelevel() else 0.0

    def gate_spine_deg(self, yaw_deg: float, roll_deg: float) -> Tuple[float, float]:
        if self.allows_spine():
            return (float(yaw_deg), float(roll_deg))
        return (0.0, 0.0)

    def attach_to(self, *gaits) -> None:
        """Bind this gate via ``GaitController.bind_ownership`` when available."""
        for gait in gaits:
            if gait is None:
                continue
            if hasattr(gait, "bind_ownership"):
                planner = getattr(gait, "_lateral_planner", None)
                gait.bind_ownership(
                    lateral_planner=planner, attitude_gate=self)
            else:
                gait._attitude_overlay_gate = self


def bind_ownership(*, lateral_planner, attitude_gate, gaits) -> None:
    """Inject ownership into gait handles (method preferred over setattr)."""
    for gait in gaits:
        if gait is None:
            continue
        if hasattr(gait, "bind_ownership"):
            gait.bind_ownership(
                lateral_planner=lateral_planner,
                attitude_gate=attitude_gate,
            )
        else:
            gait._lateral_planner = lateral_planner
            gait._attitude_overlay_gate = attitude_gate


__all__ = ["AttitudeOverlayGate", "bind_ownership"]
