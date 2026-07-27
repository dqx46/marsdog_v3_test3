"""Explicit runtime knobs for the live walk control path.

Replaces the historical walk.py module globals (VAR_IMPEDANCE, LEG_KP_SCALE,
ACTIVE_DM_*, DM_TARSUS_ACTIVE, ...) with one mutable object owned by the
steady-state loop / RuntimePipeline.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

from marsdog_control.config.joints import DEFAULT_DM_KD, DEFAULT_DM_KP
from marsdog_control.hardware.actuation import ActuationRuntime


@dataclass
class DmTarsusState:
    fixed_targets: dict = field(default_factory=dict)
    active: bool = False
    kp_by_id: dict = field(default_factory=lambda: {4: 60.0, 8: 60.0})
    kd_by_id: dict = field(default_factory=lambda: {4: 3.0, 8: 3.0})
    kp_default: float = 60.0
    kd_default: float = 3.0
    reference_lead_s: dict = field(default_factory=lambda: {4: 0.0, 8: 0.0})
    reference_lead_max_rad: float = math.radians(3.0)
    dq_feedforward: bool = True
    dq_max_rps: float = 1.5


@dataclass
class ImpedanceKnobs:
    enabled: bool = False
    td_kp_scale: float = 0.4
    swing_kp_scale: float = 0.7
    td_window: float = 0.15


@dataclass
class WalkRuntimeState:
    """Authoritative live knobs for actuation, impedance and board handle."""

    joint_gains: dict = field(default_factory=dict)
    leg_kp_scale: float = 1.0
    gravity_comp: bool = False
    gravity_scale: float = 0.5
    vmc_enabled: bool = False
    wbc_enabled: bool = False
    impedance: ImpedanceKnobs = field(default_factory=ImpedanceKnobs)
    dm: DmTarsusState = field(default_factory=DmTarsusState)
    board: Optional[object] = None
    stop_requested: bool = False

    def to_actuation_runtime(self) -> ActuationRuntime:
        d = self.dm
        return ActuationRuntime(
            dm_tarsus_active=d.active,
            dm_fixed_targets=d.fixed_targets,
            dm_reference_lead_s=d.reference_lead_s,
            dm_reference_lead_max_rad=d.reference_lead_max_rad,
            active_dm_kp_by_id=d.kp_by_id,
            active_dm_kp=d.kp_default,
            active_dm_kd_by_id=d.kd_by_id,
            active_dm_kd=d.kd_default,
            default_dm_kp=DEFAULT_DM_KP,
            default_dm_kd=DEFAULT_DM_KD,
            dm_dq_max_rps=d.dq_max_rps,
            dm_dq_feedforward=d.dq_feedforward,
            leg_kp_scale=self.leg_kp_scale,
            joint_gains=self.joint_gains,
        )

    def sync_executor_config(self, executor) -> None:
        cfg = executor.config
        cfg.variable_impedance = self.impedance.enabled
        cfg.td_kp_scale = self.impedance.td_kp_scale
        cfg.swing_kp_scale = self.impedance.swing_kp_scale
        cfg.td_window = self.impedance.td_window
        cfg.gravity_comp = self.gravity_comp
        cfg.gravity_scale = self.gravity_scale
        cfg.leg_kp_scale = self.leg_kp_scale
        cfg.vmc_enabled = self.vmc_enabled
        cfg.wbc_enabled = self.wbc_enabled
        # 热开 VMC/WBC 时补建控制器(装配期已开则 __post_init__ 已创建)
        if self.vmc_enabled and getattr(executor, "_vmc_ctrl", None) is None:
            from marsdog_control.control.vmc import DecoupledVMC, VmcConfig
            executor._vmc_ctrl = DecoupledVMC(VmcConfig())
            
        if self.wbc_enabled and getattr(executor, "_wbc_ctrl", None) is None:
            from marsdog_control.control.wbc import WholeBodyController, WbcConfig
            from marsdog_control.control.srb_mpc import SrbMpc, SrbMpcConfig
            from marsdog_control.control.nmpc_reduced_model import (
                QuadrupedReducedModel,
                default_urdf_path,
            )
            from marsdog_control.control.base_estimator import BaseStateEstimator
            try:
                urdf_path = default_urdf_path()
                reduced = QuadrupedReducedModel(urdf_path)
                wbc_cfg = WbcConfig(urdf_path=urdf_path)
                executor._reduced = reduced
                executor._wbc_ctrl = WholeBodyController(wbc_cfg, reduced=reduced)
                mpc_cfg = SrbMpcConfig(mass=float(reduced.total_mass))
                executor._srb_mpc = SrbMpc(mpc_cfg, inertia=reduced.get_locked_inertia())
                executor._estimator = BaseStateEstimator()
                executor.config.wbc_enabled = True
                executor.config.variable_impedance = True
                print(
                    f"[WalkState] late WBC+MPC init mass={reduced.total_mass:.3f} kg"
                )
            except Exception as e:
                print(f"Late Init SRB-MPC/WBC 失败: {e}")
                executor._wbc_ctrl = None
                executor._srb_mpc = None

    def as_dev_tuning(self):
        from marsdog_control.input.user_input import DevTuningRuntime
        return DevTuningRuntime(
            td_kp_scale=self.impedance.td_kp_scale,
            grav_scale=self.gravity_scale,
        )

    def apply_dev_tuning_result(self, rt) -> None:
        self.impedance.td_kp_scale = rt.td_kp_scale
        self.gravity_scale = rt.grav_scale

    def apply_control_features(
        self,
        *,
        leg_kp_scale: float,
        var_impedance: bool,
        td_kp_scale: float,
        swing_kp_scale: float,
        td_window: float,
        gravity_comp: bool,
        gravity_scale: float,
        vmc_enabled: bool = False,
        wbc_enabled: bool = False,
    ) -> None:
        """Load CLI / RuntimeConfig control knobs into this state object."""
        self.leg_kp_scale = leg_kp_scale
        self.impedance.enabled = bool(var_impedance)
        self.impedance.td_kp_scale = max(0.05, min(1.0, td_kp_scale))
        self.impedance.swing_kp_scale = max(0.05, min(1.0, swing_kp_scale))
        self.impedance.td_window = max(0.02, min(0.30, td_window))
        self.gravity_comp = bool(gravity_comp)
        self.gravity_scale = max(-1.5, min(1.5, gravity_scale))
        self.vmc_enabled = bool(vmc_enabled)
        self.wbc_enabled = bool(wbc_enabled)

    def apply_dm_tarsus(
        self,
        *,
        active: bool,
        kp_fl: float,
        kp_fr: float,
        kd_fl: float,
        kd_fr: float,
        lead_fl_s: float,
        lead_fr_s: float,
        lead_max_s: float,
        lead_max_rad: float,
        dq_feedforward: bool,
        dq_max_rps: float,
    ) -> None:
        """Configure active front-tarsus actuation knobs."""
        self.dm.active = bool(active)
        if not active:
            return
        self.dm.kp_by_id[4] = max(0.0, min(500.0, kp_fl))
        self.dm.kp_by_id[8] = max(0.0, min(500.0, kp_fr))
        self.dm.kd_by_id[4] = max(0.0, min(5.0, kd_fl))
        self.dm.kd_by_id[8] = max(0.0, min(5.0, kd_fr))
        self.dm.reference_lead_s[4] = max(0.0, min(lead_max_s, lead_fl_s))
        self.dm.reference_lead_s[8] = max(0.0, min(lead_max_s, lead_fr_s))
        self.dm.reference_lead_max_rad = max(0.0, min(math.radians(15.0), lead_max_rad))
        self.dm.dq_feedforward = bool(dq_feedforward)
        self.dm.dq_max_rps = max(0.0, min(10.0, dq_max_rps))
        self.dm.kp_default = self.dm.kp_by_id[4]
        self.dm.kd_default = self.dm.kd_by_id[4]


__all__ = [
    "DmTarsusState",
    "ImpedanceKnobs",
    "WalkRuntimeState",
]
