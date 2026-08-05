"""Physical-quantity ownership for the walk control stack.

One owner per quantity — illegal combinations fail at construction / startup,
not by runtime if-wipe alone. Assembled once into ``WalkSessionConfig.policies``.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import TYPE_CHECKING, Optional

if TYPE_CHECKING:
    from marsdog_control.config.schema import RuntimeConfig


class AttitudeOwner(str, Enum):
    """Who may correct body attitude this session."""

    NONE = "none"
    IMU = "imu"      # kinematic foot-height IMU loop
    WBC = "wbc"      # whole-body base PD / force tasks


class LateralOwner(str, Enum):
    """Who may command lateral CoM / weight shift."""

    NONE = "none"
    COM_SHIFT = "com_shift"   # Soft event-type kinematic shift (default Soft)
    SWAY = "sway"             # half-sine lateral_sway (kinematic only)
    WALK_COM = "walk_com"     # Walk abd-Y + MPC y pair (single intentional owner)
    SPOT = "spot"             # SpotYawStepper support-triangle shift
    FORCE_Y = "force_y"       # DynamicsConfig.com_y_shift / MPC y bias


class ForceMode(str, Enum):
    """Exclusive torque feed-forward path.

    Joint PD always runs underneath; its Soft softening is ``ImpedanceAssist``
    (orthogonal — not a fourth ForceMode).
    """

    IMPEDANCE = "impedance"  # gravity_comp and/or plain MIT
    VMC = "vmc"
    WBC = "wbc"


class ControlPolicyError(ValueError):
    """Illegal ownership combination."""


@dataclass(frozen=True)
class ImpedanceAssist:
    """Joint-PD softening layer — orthogonal to ``ForceMode`` τ_ff ownership.

    Soft session default ``leg_kp_scale=0.90`` softens MIT kp so WBC/VMC τ_ff
    can dominate. ``enabled=False`` forces scale 1.0 (native brand tables).
    Jump phase / Spot abd boost are transient overlays in
    ``control.impedance_overlay`` — they never mutate this session value.
    """

    enabled: bool = True
    leg_kp_scale: float = 1.0

    def effective_leg_kp_scale(self) -> float:
        return float(self.leg_kp_scale) if self.enabled else 1.0


@dataclass(frozen=True)
class ControlPolicies:
    """Locked ownership snapshot for one walk session."""

    attitude: AttitudeOwner
    lateral: LateralOwner
    force: ForceMode
    impedance: ImpedanceAssist = ImpedanceAssist()

    def __post_init__(self) -> None:
        if self.attitude is AttitudeOwner.IMU and self.force is ForceMode.WBC:
            raise ControlPolicyError(
                "AttitudeOwner.IMU cannot run with ForceMode.WBC "
                "(dual attitude loops)")
        if self.attitude is AttitudeOwner.WBC and self.force is not ForceMode.WBC:
            raise ControlPolicyError(
                "AttitudeOwner.WBC requires ForceMode.WBC")
        if self.force is ForceMode.WBC and self.attitude is AttitudeOwner.IMU:
            raise ControlPolicyError(
                "ForceMode.WBC forbids AttitudeOwner.IMU")

    @property
    def apply_imu_foot_balance(self) -> bool:
        """True iff the kinematic IMU foot-height correction may run."""
        return self.attitude is AttitudeOwner.IMU

    @property
    def apply_soft_attitude_overlay(self) -> bool:
        """True iff Soft anti_roll / trot_roll_ff may run (not under WBC)."""
        return self.attitude is not AttitudeOwner.WBC

    @property
    def apply_kinematic_lateral(self) -> bool:
        """True iff Soft/trot kinematic CoM-Y may run."""
        return self.lateral in (
            LateralOwner.COM_SHIFT, LateralOwner.SWAY, LateralOwner.WALK_COM)

    @property
    def apply_force_y(self) -> bool:
        """True iff Dynamics / Walk MPC y-bias may run (not Pace SWAY)."""
        return self.lateral in (LateralOwner.FORCE_Y, LateralOwner.WALK_COM)

    @classmethod
    def from_runtime(
        cls,
        runtime: "RuntimeConfig",
        *,
        lateral: Optional[LateralOwner] = None,
    ) -> "ControlPolicies":
        """Derive policies from typed RuntimeConfig features / dynamics.

        Soft default lateral is ``COM_SHIFT`` when Soft balance uses com_shift
        and sway is off; callers may override after Soft pour inspection.
        """
        feat = runtime.features
        dyn = runtime.dynamics
        wbc = bool(feat.wbc_enabled)
        vmc = bool(feat.vmc_enabled)
        if wbc and vmc:
            raise ControlPolicyError(
                "ForceMode conflict: wbc_enabled and vmc_enabled both True")

        if wbc:
            force = ForceMode.WBC
            # WBC owns attitude unless dynamics explicitly keeps IMU foot loop.
            if bool(getattr(dyn, "disable_imu_foot_balance", True)):
                attitude = AttitudeOwner.WBC
            elif bool(feat.imu_enabled) and bool(feat.imu_feedback_enabled):
                raise ControlPolicyError(
                    "WBC with IMU foot balance requires "
                    "dynamics.disable_imu_foot_balance=True "
                    "(AttitudeOwner mutex)")
            else:
                attitude = AttitudeOwner.WBC
        elif vmc:
            force = ForceMode.VMC
            attitude = (
                AttitudeOwner.IMU
                if (feat.imu_enabled and feat.imu_feedback_enabled)
                else AttitudeOwner.NONE
            )
        else:
            force = ForceMode.IMPEDANCE
            attitude = (
                AttitudeOwner.IMU
                if (feat.imu_enabled and feat.imu_feedback_enabled)
                else AttitudeOwner.NONE
            )

        lat = lateral if lateral is not None else LateralOwner.COM_SHIFT
        assist = ImpedanceAssist(
            enabled=True,
            leg_kp_scale=float(runtime.control.leg_kp_scale),
        )
        return cls(
            attitude=attitude, lateral=lat, force=force, impedance=assist)

    @classmethod
    def soft_default_lateral(
        cls,
        *,
        com_shift_m: float,
        lateral_sway: float,
    ) -> LateralOwner:
        """Pick Soft lateral owner; refuse dual active Soft overlays."""
        shift_on = abs(float(com_shift_m)) > 1e-9
        sway_on = abs(float(lateral_sway)) > 1e-9
        if shift_on and sway_on:
            raise ControlPolicyError(
                "Soft lateral conflict: com_shift_m and lateral_sway both "
                f"nonzero ({com_shift_m=}, {lateral_sway=}); pick one owner")
        if shift_on:
            return LateralOwner.COM_SHIFT
        if sway_on:
            return LateralOwner.SWAY
        return LateralOwner.NONE

    def banner_line(self) -> str:
        return (
            f"policies=attitude:{self.attitude.value} "
            f"force:{self.force.value} "
            f"lateral:{self.lateral.value} "
            f"imp_kp×{self.impedance.effective_leg_kp_scale():.2f}"
        )


def resolve_force_mode(wbc_enabled: bool, vmc_enabled: bool) -> ForceMode:
    """Pure helper for ExecutorConfig — same mutex as ControlPolicies."""
    if wbc_enabled and vmc_enabled:
        raise ControlPolicyError("wbc_enabled and vmc_enabled both True")
    if wbc_enabled:
        return ForceMode.WBC
    if vmc_enabled:
        return ForceMode.VMC
    return ForceMode.IMPEDANCE


__all__ = [
    "AttitudeOwner",
    "ControlPolicies",
    "ControlPolicyError",
    "ForceMode",
    "ImpedanceAssist",
    "LateralOwner",
    "resolve_force_mode",
]
