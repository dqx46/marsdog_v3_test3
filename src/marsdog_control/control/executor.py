"""Command execution preparation.

Converts a safe ``MotionTarget`` into ``ControlOutput``. Dynamics path is
orchestrated as:
  ContactSchedule → ForcePlanner(MPC) → WholeBodyController(QP) → Telemetry
with BaseStateEstimator providing sim/real velocity parity.

Split:
  * ``executor_gains`` — brand/phase gain resolve + gravity FF
  * ``executor_dynamics`` — WBC/VMC/pin helpers (mixin)
  * this module — ``ExecutorConfig`` + ``CommandExecutor.build`` orchestration
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Dict, Optional

import numpy as np

from marsdog_control.config.schema import RuntimeConfig
from marsdog_control.config.joints import JOINT_BY_NAME as JBN, JOINT_MAP
from marsdog_control.control.executor_dynamics import ExecutorDynamicsMixin
from marsdog_control.control.executor_gains import (
    _LEG_MOTOR_IDS,
    gravity_trq,
    resolve_gains,
)
from marsdog_control.config.control_policies import ForceMode, resolve_force_mode
from marsdog_control.control.impedance_overlay import (
    apply_spot_abd_kp_boost,
    resolve_impedance_layers,
)
from marsdog_control.core.types import ControlOutput, MotionTarget, RobotState
from marsdog_control.motion.gait_controller import kp_phase_scale
from marsdog_control.control.vmc import DecoupledVMC, VmcConfig
from marsdog_control.control.nmpc_reduced_model import default_urdf_path


_TELEMETRY_MAXLEN = 4000
_LEGS = ("fl", "fr", "rl", "rr")


@dataclass
class ExecutorConfig:
    variable_impedance: bool = False
    gravity_comp: bool = False
    vmc_enabled: bool = False
    wbc_enabled: bool = False
    force_mode: ForceMode = ForceMode.IMPEDANCE
    td_kp_scale: float = 0.4
    swing_kp_scale: float = 0.7
    td_window: float = 0.15
    kp_scale: float = 1.0
    leg_kp_scale: float = 1.0
    gravity_scale: float = 0.5
    # Base PD for WBC task (z/roll/pitch; horizontal owned by MPC)
    kp_base_z: float = 30.0
    kd_base_z: float = 10.0
    kp_base_roll: float = 25.0
    kd_base_roll: float = 8.0
    kp_base_pitch: float = 35.0
    kd_base_pitch: float = 8.0
    urdf_path: str = field(default_factory=default_urdf_path)
    # Default estimator: sim/real share velocity path (truth = debug only)
    base_estimate_mode: str = "estimator"  # truth | estimator
    force_lpf_alpha: float = 0.10
    contact_edge_blend: float = 0.12
    disable_imu_foot_balance: bool = True
    swing_foot_kp: float = 40.0
    swing_foot_kd: float = 6.0
    max_df_dt: float = 700.0
    lateral_vel_damp: float = 9.0
    com_y_shift_m: float = 0.012
    measure_force_weight: float = 0.15
    telemetry_maxlen: int = _TELEMETRY_MAXLEN

    def __post_init__(self) -> None:
        # Keep ForceMode and boolean flags consistent (owner gate).
        if self.force_mode is ForceMode.WBC:
            self.wbc_enabled = True
            self.vmc_enabled = False
        elif self.force_mode is ForceMode.VMC:
            self.vmc_enabled = True
            self.wbc_enabled = False
        else:
            # Impedance: derive force_mode from flags if caller set booleans only.
            if self.wbc_enabled and self.vmc_enabled:
                raise ValueError("ExecutorConfig: wbc and vmc both enabled")
            if self.wbc_enabled:
                self.force_mode = ForceMode.WBC
            elif self.vmc_enabled:
                self.force_mode = ForceMode.VMC
            else:
                self.force_mode = ForceMode.IMPEDANCE
        expected = resolve_force_mode(self.wbc_enabled, self.vmc_enabled)
        if self.force_mode is not expected:
            raise ValueError(
                f"ExecutorConfig force_mode={self.force_mode} inconsistent "
                f"with wbc={self.wbc_enabled} vmc={self.vmc_enabled} "
                f"(expected {expected})"
            )

    @classmethod
    def from_runtime_config(cls, config: RuntimeConfig) -> "ExecutorConfig":
        dyn = getattr(config, "dynamics", None)
        force = resolve_force_mode(
            bool(config.features.wbc_enabled),
            bool(config.features.vmc_enabled),
        )
        return cls(
            variable_impedance=config.features.variable_impedance_enabled,
            gravity_comp=config.features.gravity_comp_enabled,
            vmc_enabled=force is ForceMode.VMC,
            wbc_enabled=force is ForceMode.WBC,
            force_mode=force,
            td_kp_scale=config.control.td_kp_scale,
            swing_kp_scale=config.control.swing_kp_scale,
            td_window=config.control.td_window_s,
            kp_scale=config.control.kp_scale,
            leg_kp_scale=config.control.leg_kp_scale,
            gravity_scale=config.control.gravity_scale,
            kp_base_z=getattr(dyn, "kp_base_z", 30.0) if dyn else 30.0,
            kd_base_z=getattr(dyn, "kd_base_z", 10.0) if dyn else 10.0,
            kp_base_roll=getattr(dyn, "kp_base_roll", 45.0) if dyn else 45.0,
            kd_base_roll=getattr(dyn, "kd_base_roll", 12.0) if dyn else 12.0,
            kp_base_pitch=getattr(dyn, "kp_base_pitch", 35.0) if dyn else 35.0,
            kd_base_pitch=getattr(dyn, "kd_base_pitch", 8.0) if dyn else 8.0,
            urdf_path=getattr(dyn, "urdf_path", default_urdf_path())
            if dyn
            else default_urdf_path(),
            base_estimate_mode=getattr(dyn, "base_estimate_mode", "estimator")
            if dyn
            else "estimator",
            force_lpf_alpha=getattr(dyn, "force_lpf_alpha", 0.10) if dyn else 0.10,
            contact_edge_blend=getattr(dyn, "contact_edge_blend", 0.12)
            if dyn
            else 0.12,
            disable_imu_foot_balance=getattr(dyn, "disable_imu_foot_balance", True)
            if dyn
            else True,
            swing_foot_kp=getattr(dyn, "swing_foot_kp", 40.0) if dyn else 40.0,
            swing_foot_kd=getattr(dyn, "swing_foot_kd", 6.0) if dyn else 6.0,
            max_df_dt=getattr(dyn, "max_df_dt", 700.0) if dyn else 700.0,
            lateral_vel_damp=getattr(dyn, "lateral_vel_damp", 9.0) if dyn else 9.0,
            com_y_shift_m=getattr(dyn, "com_y_shift_m", 0.012) if dyn else 0.012,
            measure_force_weight=getattr(dyn, "measure_force_weight", 0.15)
            if dyn
            else 0.15,
        )


@dataclass
class CommandExecutor(ExecutorDynamicsMixin):
    """Builds motor command context for the hardware layer."""

    config: ExecutorConfig = field(default_factory=ExecutorConfig)
    runtime_config: Optional[RuntimeConfig] = None
    # Lateral ownership gate (injected at assembly; fail-closed if None).
    lateral_planner: Optional[object] = None
    _prev_targets: dict[int, float] = field(default_factory=dict)
    _prev_t: float = field(default_factory=time.time)
    _vmc_ctrl: Optional[DecoupledVMC] = None
    _wbc_ctrl: Optional[object] = None
    _force_planner: Optional[object] = None
    _reduced: Optional[object] = None
    _estimator: Optional[object] = None
    _contact: Optional[object] = None
    _dyn_tel: Optional[object] = None
    _foot_des_prev: Optional[Dict[str, np.ndarray]] = None
    _ctrl_dt: float = 0.005
    # Spot yaw-debt anti-windup limit (rad ~ how far the target may lead measured).
    _SPOT_YAW_DEBT_MAX: float = 0.25

    def __post_init__(self) -> None:
        if self.runtime_config is not None:
            self.config = ExecutorConfig.from_runtime_config(self.runtime_config)
        # Alias for WBC mixins that historically read ``_lateral_planner``.
        self._lateral_planner = self.lateral_planner
        if self.config.vmc_enabled:
            self._vmc_ctrl = DecoupledVMC(VmcConfig())
        if self.config.wbc_enabled:
            self.config.variable_impedance = True
            if self.config.td_kp_scale < 0.35:
                self.config.td_kp_scale = 0.40
            if self.config.swing_kp_scale < 0.7:
                self.config.swing_kp_scale = 0.80

            from marsdog_control.control.wbc import WholeBodyController, WbcConfig
            from marsdog_control.control.srb_mpc import SrbMpc, SrbMpcConfig
            from marsdog_control.control.nmpc_reduced_model import QuadrupedReducedModel
            from marsdog_control.control.base_estimator import BaseStateEstimator
            from marsdog_control.control.contact_schedule import (
                ContactConfig,
                ContactSchedule,
            )
            from marsdog_control.control.force_planner import ForcePlanner
            from marsdog_control.control.dynamics_telemetry import DynamicsTelemetry

            dyn = getattr(self.runtime_config, "dynamics", None) if self.runtime_config else None
            urdf = self.config.urdf_path
            self._reduced = QuadrupedReducedModel(urdf)

            wbc_cfg = WbcConfig(urdf_path=urdf)
            if dyn is not None:
                wbc_cfg.mu = dyn.mu
                wbc_cfg.f_max = dyn.f_max
                wbc_cfg.f_min = dyn.f_min
                wbc_cfg.kp_base_z = dyn.kp_base_z
                wbc_cfg.kd_base_z = dyn.kd_base_z
                wbc_cfg.kp_base_roll = dyn.kp_base_roll
                wbc_cfg.kd_base_roll = dyn.kd_base_roll
                wbc_cfg.kp_base_pitch = dyn.kp_base_pitch
                wbc_cfg.kd_base_pitch = dyn.kd_base_pitch
                wbc_cfg.tau_limit_nm = dyn.tau_limit_nm
                wbc_cfg.tau_scale = float(
                    getattr(dyn, "wbc_tau_scale", wbc_cfg.tau_scale)
                )

            self._wbc_ctrl = WholeBodyController(wbc_cfg, reduced=self._reduced)
            I_base = self._reduced.get_locked_inertia()
            mpc_cfg = SrbMpcConfig(mass=float(self._reduced.total_mass))
            if dyn is not None:
                mpc_cfg.mu = dyn.mu
                mpc_cfg.f_max = dyn.f_max
                mpc_cfg.horizon = dyn.mpc_horizon
                mpc_cfg.dt = dyn.mpc_dt
            mpc = SrbMpc(mpc_cfg, inertia=I_base)
            self._force_planner = ForcePlanner(
                mpc,
                force_lpf_alpha=self.config.force_lpf_alpha,
                max_df_dt=self.config.max_df_dt,
                dt=0.005,
                mpc_period_s=(float(dyn.mpc_period_s) if dyn is not None else 0.02),
            )
            self._estimator = BaseStateEstimator()
            self._contact = ContactSchedule(
                ContactConfig(
                    edge_blend=self.config.contact_edge_blend,
                    measure_force_weight=self.config.measure_force_weight,
                )
            )
            self._dyn_tel = DynamicsTelemetry(self.config.telemetry_maxlen)
            self._foot_des_prev = {leg: np.zeros(3) for leg in _LEGS}
            self._vy_filt = 0.0
            print(
                f"[Executor] WBC+MPC on reduced model mass={self._reduced.total_mass:.3f} kg "
                f"nv={self._reduced.nv} f_max={mpc_cfg.f_max} mu={mpc_cfg.mu} "
                f"force_lpf={self.config.force_lpf_alpha} edge={self.config.contact_edge_blend} "
                f"estimate={self.config.base_estimate_mode} "
                f"tau_scale={wbc_cfg.tau_scale:.2f}"
            )

        from marsdog_control.control.dynamics_telemetry import DynamicsTelemetry

        self._dyn_tel = self._dyn_tel or DynamicsTelemetry(self.config.telemetry_maxlen)
        # Back-compat: sim_walk / plotter expect dict-like ``telemetry``
        self.telemetry = self._dyn_tel

    def reset(self, clock=None) -> None:
        self._prev_targets.clear()
        self._prev_t = clock.time() if clock else time.time()
        if self._estimator is not None:
            self._estimator.reset()
        if self._contact is not None:
            self._contact.reset()
        if self._force_planner is not None:
            self._force_planner.reset()
        if self._dyn_tel is not None:
            self._dyn_tel.reset()
        if self._foot_des_prev is not None:
            self._foot_des_prev = {leg: np.zeros(3) for leg in _LEGS}
        self._vy_filt = 0.0
        self._tel_prev_tau = None
        self._tel_prev_tau_base = None
        self._spot_yaw_debt = 0.0

    def build(
        self,
        state: RobotState,
        motion: MotionTarget,
        fsm,
        active_gait=None,
        t_rel: float = 0.0,
        *,
        clock=None,
    ) -> ControlOutput:
        clock = clock or time
        targets = dict(motion.q)
        velocities, ctrl_dt = self._velocity_feedforward(targets, clock=clock)
        self._ctrl_dt = ctrl_dt
        kp_phase = self._kp_phase(active_gait, t_rel)
        # Session ImpedanceAssist (config.leg_kp_scale) × Jump/Spot overlays.
        layers = resolve_impedance_layers(
            float(self.config.leg_kp_scale), active_gait, t_rel
        )
        if layers.spot_abd_boost_active:
            kp_phase = apply_spot_abd_kp_boost(
                kp_phase,
                session_leg_kp=layers.session_leg_kp,
                joint_by_name=JBN,
            )
            self._spot_swing_kp_boost = True
        else:
            self._spot_swing_kp_boost = False

        jump_gait = (
            active_gait is not None
            and getattr(active_gait, "family", None) == "jump"
        )
        self._jump_flight_swing_boost = bool(
            jump_gait
            and getattr(getattr(active_gait, "phase", None), "value", "") == "flight"
        )

        # Phase-only prior (VMC / fallback). WBC overwrites via ContactSchedule.
        leg_is_stance = {leg: True for leg in _LEGS}
        if active_gait is not None:
            period = active_gait.period
            stance_ratio = active_gait.stance_ratio
            offsets = active_gait._PHASE_OFFSET
            for leg in _LEGS:
                phase = (t_rel / period + offsets[leg]) % 1.0
                leg_is_stance[leg] = phase <= stance_ratio

        trq_ff = {}
        mode = self.config.force_mode
        if mode is ForceMode.WBC:
            trq_ff = self._apply_wbc(
                state, targets, active_gait, leg_is_stance, t_rel
            )
        elif mode is ForceMode.VMC:
            trq_ff = self._apply_vmc(state, targets, None, leg_is_stance)
        elif self.config.gravity_comp:
            trq_ff = gravity_trq(targets, self.config.gravity_scale)

        if getattr(self, "_first_print", True):
            print(
                f"[Executor] force_mode={mode.value} "
                f"wbc={self.config.wbc_enabled} vmc={self.config.vmc_enabled}"
            )
            self._first_print = False

        # Impedance / VMC path: still log gait + IMU (WBC path records inside _apply_wbc).
        if mode is not ForceMode.WBC and self._dyn_tel is not None:
            self._record_baseline_telemetry(
                state, targets, active_gait, leg_is_stance, t_rel, trq_ff
            )

        return ControlOutput(
            target=MotionTarget(
                q=targets, dq=velocities, source_mode=motion.source_mode
            ),
            kp_phase=kp_phase,
            trq_ff=trq_ff,
            kp_scale=self.config.kp_scale,
            leg_kp_scale=layers.effective_leg_kp,
            dm_active=fsm.dm_active() if hasattr(fsm, "dm_active") else False,
            gait_active=active_gait is not None,
            control_period_s=ctrl_dt,
        )

    def _velocity_feedforward(self, targets: dict[int, float], *, clock=time):
        now_t = clock.time()
        ctrl_dt = max(0.001, now_t - self._prev_t)
        velocities: dict[int, float] = {}
        if self._prev_targets:
            for mid, target in targets.items():
                if mid in self._prev_targets:
                    velocities[mid] = (
                        target - self._prev_targets[mid]
                    ) / ctrl_dt
        self._prev_targets = dict(targets)
        self._prev_t = now_t
        return velocities, ctrl_dt

    def _kp_phase(self, active_gait, t_rel: float) -> Optional[dict[int, float]]:
        if not self.config.variable_impedance or active_gait is None:
            return None
        kp_phase: dict[int, float] = {}
        # Spot: follow unitree swing duty (SpotYawStepper), not cruise stagger.
        spot = bool(getattr(active_gait, "spot_turn_active", False))
        walk = getattr(active_gait, "family", None) == "walk"
        jump = getattr(active_gait, "family", None) == "jump"
        stepper = getattr(active_gait, "_spot", None) if spot else None
        period = active_gait.period
        stance_ratio = active_gait.stance_ratio
        offsets = active_gait._PHASE_OFFSET
        # Walk: softer plant only in a short late-swing window.
        # Jump LAND: soft TD window for impedance fade-in.
        td_scale = self.config.td_kp_scale
        swing_scale = self.config.swing_kp_scale
        if jump:
            td_window = min(0.18, max(0.12, float(self.config.td_window)))
        elif walk:
            td_window = min(0.10, float(self.config.td_window))
        else:
            td_window = self.config.td_window
        for mid, leg in _LEG_MOTOR_IDS:
            if jump:
                # Stiff crouch/push; soft flight/land (avoid yanking body down).
                phase_name = getattr(
                    getattr(active_gait, "phase", None), "value", ""
                )
                if phase_name == "land":
                    kp_phase[mid] = 1.0
                elif phase_name == "recover":
                    kp_phase[mid] = 0.70
                elif phase_name == "flight":
                    u = 0.0
                    if hasattr(active_gait, "_phase_u"):
                        try:
                            u = float(active_gait._phase_u(t_rel))
                        except Exception:
                            u = 0.0
                    kp_phase[mid] = 1.60 if u < 0.25 else 0.50
                elif phase_name == "push":
                    kp_phase[mid] = 0.95
                elif phase_name == "crouch":
                    kp_phase[mid] = 1.5
                else:
                    kp_phase[mid] = 1.0
            elif stepper is not None and hasattr(stepper, "in_swing"):
                in_sw = bool(stepper.in_swing(leg))
                kp_phase[mid] = (
                    float(swing_scale) if in_sw
                    else 1.0
                )
            else:
                phase = (t_rel / period + offsets[leg]) % 1.0
                kp_phase[mid] = kp_phase_scale(
                    phase,
                    stance_ratio,
                    td_scale,
                    swing_scale,
                    td_window,
                )
        return kp_phase


__all__ = ["CommandExecutor", "ExecutorConfig", "gravity_trq", "resolve_gains"]
