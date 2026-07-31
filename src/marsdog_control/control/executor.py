"""Command execution preparation.

Converts a safe ``MotionTarget`` into ``ControlOutput``. Dynamics path is
orchestrated as:
  ContactSchedule → ForcePlanner(MPC) → WholeBodyController(QP) → Telemetry
with BaseStateEstimator providing sim/real velocity parity.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Dict, Optional

import numpy as np

from marsdog_control.config.schema import RuntimeConfig
from marsdog_control.config.joints import JOINT_BY_NAME as JBN, JOINT_MAP
from marsdog_control.control.gravity_comp import leg_gravity_ff
from marsdog_control.core.types import ControlOutput, MotionTarget, RobotState
from marsdog_control.motion.gait_controller import kp_phase_scale
from marsdog_control.control.vmc import DecoupledVMC, VmcConfig
from marsdog_control.motion import kinematics as K
from marsdog_control.control.nmpc_reduced_model import default_urdf_path


_GC_LEG_JOINTS = {
    "fl": {"hip_pitch": "fl_hip_pitch", "calf": "fl_calf"},
    "fr": {"hip_pitch": "fr_hip_pitch", "calf": "fr_calf"},
    "rl": {"thigh": "rl_thigh", "calf": "rl_calf"},
    "rr": {"thigh": "rr_thigh", "calf": "rr_calf"},
}

_TELEMETRY_MAXLEN = 4000
_LEGS = ("fl", "fr", "rl", "rr")


def gravity_trq(targets, grav_scale):
    """按当前目标位姿计算腿部 pitch 关节重力补偿前馈 (电机端 Nm)。"""
    out = {}
    for leg, jmap in _GC_LEG_JOINTS.items():
        angles = {}
        ok = True
        for key, jname in jmap.items():
            j = JBN[jname]
            if j.motor_id not in targets:
                ok = False
                break
            angles[key] = targets[j.motor_id]
        if not ok:
            continue
        ff = leg_gravity_ff(leg, angles)
        for key, jname in jmap.items():
            j = JBN[jname]
            out[j.motor_id] = ff[key] * grav_scale
    return out


def _is_leg_joint(name: str) -> bool:
    return name[:3] in ("fl_", "fr_", "rl_", "rr_")


_LEG_MOTOR_IDS = [
    (j.motor_id, j.name[:2]) for j in JOINT_MAP if _is_leg_joint(j.name)
]


def resolve_gains(
    j,
    kp_scale,
    use_joint_gains,
    kp_lz,
    kd_lz,
    kp_evo,
    kd_evo,
    leg_kp_scale,
    joint_gains,
    phase_scale=1.0,
    trq_override=None,
):
    """解析单个关节最终的 (kp, kd, trq) — 电机端量纲。"""
    leg_s = leg_kp_scale if _is_leg_joint(j.name) else 1.0
    leg_s *= phase_scale
    if use_joint_gains:
        g = joint_gains.get(j.name, {"kp": 30.0, "kd": 4.0, "trq_ff": 0.0})
        trq = g["trq_ff"] if trq_override is None else trq_override
        return g["kp"] * kp_scale * leg_s, g["kd"], trq
    kp = (kp_lz if j.mtype == "lz" else kp_evo) * kp_scale * leg_s
    kd = kd_lz if j.mtype == "lz" else kd_evo
    return kp, kd, (0.0 if trq_override is None else trq_override)


@dataclass
class ExecutorConfig:
    variable_impedance: bool = False
    gravity_comp: bool = False
    vmc_enabled: bool = False
    wbc_enabled: bool = False
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

    @classmethod
    def from_runtime_config(cls, config: RuntimeConfig) -> "ExecutorConfig":
        dyn = getattr(config, "dynamics", None)
        return cls(
            variable_impedance=config.features.variable_impedance_enabled,
            gravity_comp=config.features.gravity_comp_enabled,
            vmc_enabled=config.features.vmc_enabled,
            wbc_enabled=config.features.wbc_enabled,
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
class CommandExecutor:
    """Builds motor command context for the hardware layer."""

    config: ExecutorConfig = field(default_factory=ExecutorConfig)
    runtime_config: Optional[RuntimeConfig] = None
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
        # Spot: boost abd joints; swing Z tracking uses WBC swing weight / foot PD.
        if active_gait is not None and getattr(active_gait, "spot_turn_active", False):
            if kp_phase is None:
                kp_phase = {}
            leg_s = max(1e-3, float(self.config.leg_kp_scale))
            boost = 1.4 / leg_s
            for jname in (
                "fl_thigh_roll", "fr_thigh_roll", "rl_hip", "rr_hip",
            ):
                if jname in JBN:
                    mid = JBN[jname].motor_id
                    kp_phase[mid] = max(float(kp_phase.get(mid, 1.0)), boost)
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
        if self.config.wbc_enabled:
            trq_ff = self._apply_wbc(
                state, targets, active_gait, leg_is_stance, t_rel
            )
        elif self.config.vmc_enabled:
            trq_ff = self._apply_vmc(state, targets, None, leg_is_stance)
        elif self.config.gravity_comp:
            trq_ff = gravity_trq(targets, self.config.gravity_scale)

        if getattr(self, "_first_print", True):
            print(
                f"[Executor] config wbc_enabled={self.config.wbc_enabled}, "
                f"vmc_enabled={self.config.vmc_enabled}"
            )
            self._first_print = False

        # Jump: stiff crouch/push for launch; soft flight so fold doesn't yank body.
        leg_kp_out = float(self.config.leg_kp_scale)
        if (
            active_gait is not None
            and getattr(active_gait, "family", None) == "jump"
        ):
            phase_name = getattr(
                getattr(active_gait, "phase", None), "value", ""
            )
            if phase_name == "crouch":
                leg_kp_out = max(leg_kp_out, 1.25)
            elif phase_name == "push":
                # Firm track of extend without burying via excessive kp.
                leg_kp_out = min(max(leg_kp_out, 1.00), 1.20)
            elif phase_name == "flight":
                # Short stiff extract, then soft so fold doesn't yank body down.
                u = 0.0
                if hasattr(active_gait, "_phase_u"):
                    try:
                        u = float(active_gait._phase_u(t_rel))
                    except Exception:
                        u = 0.0
                if u < 0.15:
                    leg_kp_out = min(max(leg_kp_out, 1.40), 1.55)
                else:
                    leg_kp_out = min(max(leg_kp_out, 0.45), 0.65)
            elif phase_name == "land":
                # Stiffer track so legs shorten with falling body (anti dig-bounce).
                leg_kp_out = min(max(leg_kp_out, 0.85), 1.10)
            elif phase_name == "recover":
                leg_kp_out = min(max(leg_kp_out, 0.55), 0.75)

        return ControlOutput(
            target=MotionTarget(
                q=targets, dq=velocities, source_mode=motion.source_mode
            ),
            kp_phase=kp_phase,
            trq_ff=trq_ff,
            kp_scale=self.config.kp_scale,
            leg_kp_scale=leg_kp_out,
            dm_active=fsm.dm_active() if hasattr(fsm, "dm_active") else False,
            gait_active=active_gait is not None,
            control_period_s=ctrl_dt,
        )

    def _assemble_pin_state(
        self,
        state: RobotState,
        base_z: float,
        vel_xyz,
        use_estimator_xy: bool,
        joint_pos: Optional[dict] = None,
    ):
        """Fill reduced-model q/v with IMU, joints, rear-tarsus mimic."""
        q_pin = np.zeros(self._wbc_ctrl.nq)
        v_pin = np.zeros(self._wbc_ctrl.nv)

        q_pin[2] = base_z
        if use_estimator_xy and self._estimator is not None:
            q_pin[0] = self._estimator.p[0]
            q_pin[1] = self._estimator.p[1]

        cr = math.cos(state.roll * 0.5)
        sr = math.sin(state.roll * 0.5)
        cp = math.cos(state.pitch * 0.5)
        sp = math.sin(state.pitch * 0.5)
        cy = math.cos(state.yaw * 0.5)
        sy = math.sin(state.yaw * 0.5)
        q_pin[3] = sr * cp * cy - cr * sp * sy
        q_pin[4] = cr * sp * cy + sr * cp * sy
        q_pin[5] = cr * cp * sy - sr * sp * cy
        q_pin[6] = cr * cp * cy + sr * sp * sy

        v_pin[0] = vel_xyz[0]
        v_pin[1] = vel_xyz[1]
        v_pin[2] = vel_xyz[2]
        v_pin[3] = state.gyro_roll
        v_pin[4] = state.gyro_pitch
        v_pin[5] = state.gyro_yaw

        pos_src = joint_pos if joint_pos is not None else state.joint_pos
        model = self._wbc_ctrl.model
        for jname, j in JBN.items():
            urdf_j = jname + "_joint"
            if not model.existJointName(urdf_j):
                continue
            idx_q = model.joints[model.getJointId(urdf_j)].idx_q
            idx_v = model.joints[model.getJointId(urdf_j)].idx_v
            q_pin[idx_q] = pos_src.get(j.motor_id, 0.0)
            if state.joint_vel:
                v_pin[idx_v] = state.joint_vel.get(j.motor_id, 0.0)

        self._reduced.apply_rear_tarsus_mimic(q_pin, v_pin)
        return q_pin, v_pin

    def _foot_kinematics(self, q_pin, v_pin):
        import pinocchio as pin

        pin.forwardKinematics(self._wbc_ctrl.model, self._wbc_ctrl.data, q_pin, v_pin)
        pin.updateFramePlacements(self._wbc_ctrl.model, self._wbc_ctrl.data)
        pin.computeJointJacobians(self._wbc_ctrl.model, self._wbc_ctrl.data, q_pin)
        foot_z = {}
        foot_vz = {}
        foot_pos = {}
        foot_vel = {}
        for i, fname in enumerate(self._wbc_ctrl.foot_names):
            leg = fname[:2]
            fid = self._wbc_ctrl.foot_ids[i]
            p = np.array(self._wbc_ctrl.data.oMf[fid].translation, dtype=float)
            J = pin.getFrameJacobian(
                self._wbc_ctrl.model,
                self._wbc_ctrl.data,
                fid,
                pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
            )
            v = J[:3, :] @ v_pin
            foot_pos[leg] = p
            foot_vel[leg] = v
            foot_z[leg] = float(p[2])
            foot_vz[leg] = float(v[2])
        return foot_z, foot_vz, foot_pos, foot_vel

    def _swing_acc_des(
        self,
        state: RobotState,
        targets: dict,
        base_z: float,
        vel_xyz,
        use_est_xy: bool,
        leg_is_stance: dict,
        foot_pos_act: dict,
        foot_vel_act: dict,
    ) -> tuple:
        """Cartesian PD on swing feet using joint-target FK as reference.

        Returns ``(swing_acc_des, foot_pos_des)``.
        """
        q_des, v_des = self._assemble_pin_state(
            state, base_z, vel_xyz, use_est_xy, joint_pos=targets
        )
        # Desired joint rates ≈ 0 for accel PD (position track from targets)
        v_des[:] = 0.0
        v_des[0:3] = vel_xyz
        v_des[3:6] = [state.gyro_roll, state.gyro_pitch, state.gyro_yaw]
        _, _, foot_pos_des, _ = self._foot_kinematics(q_des, v_des)

        kp = float(self.config.swing_foot_kp)
        kd = float(self.config.swing_foot_kd)
        if getattr(self, "_spot_swing_kp_boost", False):
            kp *= 1.5
            kd *= 1.25
        if getattr(self, "_jump_flight_swing_boost", False):
            kp *= 1.4
            kd *= 1.2
        dt = max(1e-3, self._ctrl_dt)
        out = {}
        # Always compute PD targets; WBC blends by force_scale (stance→0 weight).
        _ = leg_is_stance
        for leg in _LEGS:
            p_des = foot_pos_des[leg].copy()
            # Jump flight: gently lift desired foot clear of soft-contact dig.
            if getattr(self, "_jump_flight_swing_boost", False):
                p_des[2] = max(float(p_des[2]), 0.012)
            p_prev = self._foot_des_prev.get(leg, p_des)
            v_des_leg = (p_des - p_prev) / dt
            self._foot_des_prev[leg] = p_des.copy()
            p_act = foot_pos_act[leg]
            v_act = foot_vel_act[leg]
            acc = kp * (p_des - p_act) + kd * (v_des_leg - v_act)
            # Jump flight: extra upward pull on still-buried feet (esp. front).
            if getattr(self, "_jump_flight_swing_boost", False):
                z_act = float(p_act[2])
                if z_act < 0.012:
                    pull = 55.0 * (0.012 - z_act)
                    if leg in ("fl", "fr"):
                        pull *= 1.4
                    acc = acc.copy()
                    acc[2] += pull
            out[leg] = acc
        return out, foot_pos_des

    def _apply_wbc(
        self,
        state: RobotState,
        targets: dict[int, float],
        active_gait,
        leg_is_stance: dict[str, bool],
        t_rel: float = 0.0,
    ) -> dict[int, float]:
        if not self._wbc_ctrl:
            return {}

        import pinocchio as pin

        use_est = self.config.base_estimate_mode != "truth"
        # Never rely on MuJoCo truth unless explicitly requested
        if use_est:
            vel_xyz = [0.0, 0.0, 0.0]
        else:
            vel_xyz = list(getattr(state, "vel_xyz", (0.0, 0.0, 0.0)))

        current_base_z = 0.24
        q_tmp, v_tmp = self._assemble_pin_state(
            state, current_base_z, vel_xyz, use_estimator_xy=False
        )
        pin.forwardKinematics(self._wbc_ctrl.model, self._wbc_ctrl.data, q_tmp)
        pin.updateFramePlacements(self._wbc_ctrl.model, self._wbc_ctrl.data)

        stance_z_sum = 0.0
        stance_count = 0
        # Use scheduled stance for initial height (contact not yet measured)
        for i, foot_name in enumerate(self._wbc_ctrl.foot_names):
            leg_key = foot_name[:2]
            if not leg_is_stance.get(leg_key, True):
                continue
            foot_id = self._wbc_ctrl.foot_ids[i]
            foot_z_world = self._wbc_ctrl.data.oMf[foot_id].translation[2]
            foot_z_local = foot_z_world - current_base_z
            stance_z_sum += -foot_z_local
            stance_count += 1
        if stance_count > 0:
            current_base_z = stance_z_sum / stance_count

        q_pin, v_pin = self._assemble_pin_state(
            state, current_base_z, vel_xyz, use_estimator_xy=False
        )
        foot_z, foot_vz, foot_pos, foot_vel = self._foot_kinematics(q_pin, v_pin)

        contact_snap = None
        if self._contact is not None:
            # Spot: trust phase schedule — measurement was keeping 3–4 feet "stance".
            # Walk: sharper LO/TD edges so short swing isn't eaten by SoftTrot blend.
            spot_now = bool(
                active_gait is not None
                and getattr(active_gait, "spot_turn_active", False)
            )
            walk_now = bool(
                active_gait is not None
                and getattr(active_gait, "family", None) == "walk"
            )
            jump_now = bool(
                active_gait is not None
                and getattr(active_gait, "family", None) == "jump"
            )
            cfg_prev = None
            if spot_now or walk_now or jump_now:
                cfg_prev = (
                    float(self._contact.cfg.measure_force_weight),
                    float(self._contact.cfg.edge_blend),
                    float(self._contact.cfg.phase_late_lo),
                    bool(self._contact.cfg.use_relative_z),
                    float(self._contact.cfg.lo_height_m),
                    float(self._contact.cfg.td_height_m),
                    int(self._contact.cfg.hold_steps),
                )
                if spot_now:
                    self._contact.cfg.measure_force_weight = 0.04
                    self._contact.cfg.edge_blend = 0.08
                    self._contact.cfg.phase_late_lo = 0.04
                elif jump_now:
                    # Soft land: wider TD window; absolute z so dig/peel don't
                    # poison relative baselines (front stuck "contact" at +1cm).
                    self._contact.cfg.measure_force_weight = 0.08
                    self._contact.cfg.edge_blend = 0.10
                    self._contact.cfg.phase_late_lo = 0.05
                    self._contact.cfg.use_relative_z = False
                    self._contact.cfg.lo_height_m = 0.008
                    self._contact.cfg.td_height_m = 0.004
                    self._contact.cfg.hold_steps = 3
                else:
                    self._contact.cfg.measure_force_weight = 0.10
                    self._contact.cfg.edge_blend = 0.06
                    self._contact.cfg.phase_late_lo = 0.03
            try:
                contact_snap = self._contact.update(
                    t_rel=t_rel,
                    gait=active_gait,
                    foot_z_world=foot_z,
                    foot_vz_world=foot_vz,
                )
            finally:
                if cfg_prev is not None:
                    (
                        self._contact.cfg.measure_force_weight,
                        self._contact.cfg.edge_blend,
                        self._contact.cfg.phase_late_lo,
                        self._contact.cfg.use_relative_z,
                        self._contact.cfg.lo_height_m,
                        self._contact.cfg.td_height_m,
                        self._contact.cfg.hold_steps,
                    ) = cfg_prev
            leg_is_stance = dict(contact_snap.stance)
            if jump_now:
                # Prefer truth vz (sim) so liftoff isn't stuck on lagged est / late note.
                vz_meas = float(getattr(state, "vel_xyz", (0.0, 0.0, 0.0))[2])
                if hasattr(active_gait, "note_base_vz"):
                    active_gait.note_base_vz(vz_meas)
                if hasattr(active_gait, "note_base_z"):
                    active_gait.note_base_z(float(current_base_z))
                if hasattr(active_gait, "_advance"):
                    active_gait._advance(t_rel)
                in_flight = bool(
                    getattr(active_gait, "in_flight", lambda: False)()
                )
                if hasattr(active_gait, "stance_ratio"):
                    active_gait.stance_ratio = 0.0 if in_flight else 1.0
                phase_u = 0.0
                if hasattr(active_gait, "_phase_u"):
                    try:
                        phase_u = float(active_gait._phase_u(t_rel))
                    except Exception:
                        phase_u = 0.0
                # Same-tick retract: motion targets were computed while still PUSH.
                if in_flight and hasattr(active_gait, "get_targets"):
                    try:
                        targets.update(active_gait.get_targets(t_rel))
                    except Exception:
                        pass
                # Snap force EMA on flight entry — residual Fz on buried soles
                # is the rear "second hop".
                if in_flight and phase_u < 0.06 and self._force_planner is not None:
                    self._force_planner._fc_filt[:] = 0.0
                jfs = float(
                    active_gait.jump_force_scale_at(t_rel)
                    if hasattr(active_gait, "jump_force_scale_at")
                    else (0.0 if in_flight else 1.0)
                )
                for leg in _LEGS:
                    if in_flight:
                        # No grace push — finish impulse in PUSH, then unload.
                        leg_is_stance[leg] = False
                        contact_snap.stance[leg] = False
                        contact_snap.force_scale[leg] = 0.0
                    else:
                        # On-ground jump: full stance.
                        leg_is_stance[leg] = True
                        contact_snap.stance[leg] = True
                        contact_snap.force_scale[leg] = 1.0
            elif spot_now:
                # Stomp FSM owns contact: plant/twist = 4-stance; catch = diagonal.
                stepper = getattr(active_gait, "_spot", None)
                for leg in _LEGS:
                    if stepper is not None and hasattr(stepper, "in_swing"):
                        leg_is_stance[leg] = not bool(stepper.in_swing(leg))
                    else:
                        leg_is_stance[leg] = bool(contact_snap.scheduled[leg])
                    contact_snap.stance[leg] = leg_is_stance[leg]
                    if not leg_is_stance[leg]:
                        contact_snap.force_scale[leg] = min(
                            float(contact_snap.force_scale.get(leg, 0.0)), 0.10
                        )

        if use_est and self._estimator is not None:
            # Joint rates already in v_pin; estimator solves for base linear vel
            est = self._estimator.update(
                reduced=self._reduced,
                q_pin=q_pin,
                v_pin=v_pin,
                roll=state.roll,
                pitch=state.pitch,
                yaw=state.yaw,
                gyro=(state.gyro_roll, state.gyro_pitch, state.gyro_yaw),
                leg_is_stance=leg_is_stance,
                dt=self._ctrl_dt,
                leg_phase=(contact_snap.phase if contact_snap is not None else None),
                stance_ratio=float(
                    getattr(active_gait, "stance_ratio", 1.0) if active_gait else 1.0
                ),
                edge_blend=(
                    float(self._contact.cfg.edge_blend)
                    if self._contact is not None
                    else None
                ),
                force_scale=(
                    contact_snap.force_scale if contact_snap is not None else None
                ),
            )
            vel_xyz = [est.vx, est.vy, est.vz]
            current_base_z = est.z
            q_pin, v_pin = self._assemble_pin_state(
                state, current_base_z, vel_xyz, use_estimator_xy=True
            )
            foot_z, foot_vz, foot_pos, foot_vel = self._foot_kinematics(q_pin, v_pin)

        target_z = active_gait.body_height if active_gait else 0.24
        if active_gait is not None and hasattr(active_gait, "get_target_z"):
            try:
                target_z = float(active_gait.get_target_z(t_rel))
            except TypeError:
                target_z = float(active_gait.get_target_z())
        target_roll = 0.0
        target_pitch = 0.0

        vx_cmd = 0.0
        vy_cmd = 0.0
        wz_cmd = 0.0
        vz_cmd = 0.0
        jump_now = bool(
            active_gait is not None
            and getattr(active_gait, "family", None) == "jump"
        )
        if active_gait:
            if hasattr(active_gait, "vel_cmd"):
                vx_cmd = active_gait.vel_cmd[0]
                vy_cmd = active_gait.vel_cmd[1]
                if len(active_gait.vel_cmd) > 2:
                    wz_cmd = float(active_gait.vel_cmd[2])
            elif hasattr(active_gait, "amp_front") and hasattr(active_gait, "period"):
                avg_amp = (
                    active_gait.amp_front
                    + getattr(active_gait, "amp_rear", active_gait.amp_front)
                ) / 2.0
                vx_cmd = (avg_amp * 2.0) / active_gait.period
            if jump_now and hasattr(active_gait, "desired_vz"):
                vz_cmd = float(active_gait.desired_vz(t_rel))
            if jump_now:
                # Prefer MuJoCo/IMU truth over estimator — est under-reads liftoff vz.
                vz_truth = float(getattr(state, "vel_xyz", (0.0, 0.0, 0.0))[2])
                vz_note = (
                    vz_truth
                    if abs(vz_truth) >= abs(float(v_pin[2]))
                    else float(v_pin[2])
                )
                if hasattr(active_gait, "note_base_vz"):
                    active_gait.note_base_vz(vz_note)
                if hasattr(active_gait, "note_base_z"):
                    active_gait.note_base_z(float(current_base_z))
                vx_cmd = 0.0
                vy_cmd = 0.0
                wz_cmd = 0.0

        cfg = self.config
        # Lateral velocity: light EMA then damp (cuts estimator noise, keeps authority).
        # Do NOT blend MuJoCo truth here — real has no truth; mixing it made sim
        # look stabler than the estimator-only path used on hardware.
        a_vy = 0.25 if use_est else 0.45
        self._vy_filt = (1.0 - a_vy) * getattr(self, "_vy_filt", 0.0) + a_vy * vel_xyz[1]
        vy_for_damp = self._vy_filt if use_est else vel_xyz[1]

        base_acc_des = np.zeros(6)
        # Dog-trot plant undershoots kinematic vx while stance-LS overestimates
        # (front swing drag / slip). Never treat est>cmd as overspeed — that
        # commanded persistent braking and stalled after ~2 strides.
        vx_meas = float(vel_xyz[0])
        if vx_cmd > 0.05:
            vx_for_track = min(vx_meas, vx_cmd)
        elif vx_cmd < -0.05:
            vx_for_track = max(vx_meas, vx_cmd)
        else:
            vx_for_track = vx_meas
        vx_err = vx_cmd - vx_for_track
        # Push when slow; zero ax when est already at/above cmd (no fake brake).
        ax_gain = 3.5 if vx_err * vx_cmd > 1e-6 else 0.0
        spot = bool(getattr(active_gait, "spot_turn_active", False))
        # Prefer larger |v| between estimator and IMU/truth — estimator
        # under-reads soft-contact scrub (~0.02 vs truth ~0.11 while standing).
        vx_raw = float(getattr(state, "vel_xyz", (vx_meas, 0.0, 0.0))[0])
        vy_raw = float(getattr(state, "vel_xyz", (0.0, 0.0, 0.0))[1])
        vx_brake = vx_raw if abs(vx_raw) > abs(vx_meas) else vx_meas
        vy_brake = vy_raw if abs(vy_raw) > abs(float(vel_xyz[1])) else float(vel_xyz[1])
        hold_still = (not spot) and abs(float(vx_cmd)) <= 0.05
        if spot:
            # Kill forward creep hard; estimator under-reads scrubbing vx.
            # Active back-bias cancels residual forward scrub.
            base_acc_des[0] = -36.0 * (vx_brake + 0.03) - 6.0 * np.sign(vx_brake + 0.015)
            base_acc_des[1] = -18.0 * vy_brake - 4.0 * np.sign(vy_brake)
            # Pull CoM into the support triangle (SpotYawStepper), on top of brake.
            if hasattr(active_gait, "get_spot_com_shift"):
                sx, sy = active_gait.get_spot_com_shift(t_rel)
                base_acc_des[0] += 30.0 * float(sx)
                base_acc_des[1] += 40.0 * float(sy)
            vx_cmd = -0.03
            vy_cmd = 0.0
        elif hold_still:
            # Stand / jump hold / zero-cmd: lateral had damp, longitudinal did not
            # → constant forward scrape on soft soles. Same authority as Spot.
            # Launch phases: don't fight vertical impulse with hard XY brake.
            jump_launch = jump_now and getattr(
                getattr(active_gait, "phase", None), "value", ""
            ) in ("crouch", "push", "flight")
            if jump_launch:
                base_acc_des[0] = -8.0 * vx_brake
                base_acc_des[1] = -float(cfg.lateral_vel_damp) * vy_for_damp
            else:
                base_acc_des[0] = (
                    -36.0 * (vx_brake + 0.02) - 6.0 * np.sign(vx_brake + 0.01)
                )
                base_acc_des[1] = (
                    -float(cfg.lateral_vel_damp) * vy_for_damp
                    - 3.0 * np.sign(vy_brake)
                )
        else:
            base_acc_des[0] = ax_gain * vx_err
            base_acc_des[1] = -float(cfg.lateral_vel_damp) * vy_for_damp
        # Jump: track z_ref + desired_vz; Soft/Walk keep original damping on vz.
        if jump_now:
            # Jump-only Z gains from JumpController (recipe JUMP_*), never mutate
            # global DynamicsConfig so SoftTrot keeps schema/CLI kp_base_z.
            kp_z = float(getattr(active_gait, "kp_base_z", cfg.kp_base_z))
            kd_z = float(getattr(active_gait, "kd_base_z", cfg.kd_base_z))
            # During jump, position tracking can fight velocity tracking if the robot
            # didn't crouch deep enough. We prioritize velocity tracking.
            z_err = target_z - current_base_z
            phase_name = getattr(
                getattr(active_gait, "phase", None), "value", ""
            )
            if phase_name == "push":
                z_err = max(0.0, z_err)  # Never push down during PUSH
                vz_truth = float(getattr(state, "vel_xyz", (0.0, 0.0, 0.0))[2])
                vz_use = (
                    vz_truth if abs(vz_truth) > abs(float(v_pin[2])) else float(v_pin[2])
                )
                # Strong upward vz track; never brake a rising hop (cmd lag → dig).
                vz_term = -2.0 * kd_z * (vz_use - vz_cmd)
                vz_term = max(0.0, float(vz_term))
                base_acc_des[2] = 0.20 * kp_z * z_err + vz_term
            elif phase_name in ("land", "recover"):
                # Soft absorb: never bounce by over-braking a downward vz.
                stand_h = float(getattr(active_gait, "stand_height", 0.24))
                z_to_stand = stand_h - current_base_z
                vz = float(v_pin[2])
                if z_to_stand > 0.0:
                    # Below stand — rise gently; don't fight the fall hard.
                    base_acc_des[2] = (
                        0.20 * kp_z * z_to_stand
                        - 0.40 * kd_z * vz
                    )
                else:
                    # Above stand — pull down / brake upward bounce.
                    base_acc_des[2] = (
                        0.90 * kp_z * z_to_stand
                        - 2.5 * kd_z * max(0.0, vz)
                    )
            else:
                base_acc_des[2] = (
                    kp_z * z_err
                    - kd_z * (v_pin[2] - vz_cmd)
                )
            # Kill nose-up so front doesn't peel/plant while rear clears.
            # Slight nose-down bias during push loads the rear feet.
            phase_name = getattr(
                getattr(active_gait, "phase", None), "value", ""
            )
            if phase_name == "push":
                pitch_des = -0.02
                pitch_kp = 5.5
                pitch_kd = 3.0
            elif phase_name == "flight":
                # Nose-down bias unloads front so all four can clear together.
                pitch_des = -0.04
                pitch_kp = 6.5
                pitch_kd = 3.5
            else:
                pitch_des = 0.0
                pitch_kp = 3.0
                pitch_kd = 2.0
            base_acc_des[4] = (
                pitch_kp * cfg.kp_base_pitch * (pitch_des - state.pitch)
                - pitch_kd * cfg.kd_base_pitch * state.gyro_pitch
            )
        else:
            base_acc_des[2] = (
                cfg.kp_base_z * (target_z - current_base_z)
                - cfg.kd_base_z * v_pin[2]
            )
            base_acc_des[4] = (
                cfg.kp_base_pitch * (target_pitch - state.pitch)
                - cfg.kd_base_pitch * state.gyro_pitch
            )
        base_acc_des[3] = (
            cfg.kp_base_roll * (target_roll - state.roll)
            - cfg.kd_base_roll * state.gyro_roll
        )
        # Spot: mild yaw_des track + wz — feet scrub does most of the turn.
        if spot and abs(wz_cmd) > 0.05:
            stepper = getattr(active_gait, "_spot", None)
            yaw_err = float(stepper.yaw_error()) if stepper is not None else 0.0
            yaw_err = max(-0.35, min(0.35, yaw_err))
            base_acc_des[5] = (
                8.0 * yaw_err
                - 4.0 * state.gyro_yaw
                + 3.0 * (wz_cmd - state.gyro_yaw)
            )
            self._spot_yaw_debt = yaw_err
        else:
            self._spot_yaw_debt = 0.0
            if abs(wz_cmd) > 0.05:
                base_acc_des[5] = 6.0 * (wz_cmd - state.gyro_yaw) - 1.5 * state.gyro_yaw
            else:
                base_acc_des[5] = -4.0 * state.gyro_yaw

        # Jump flight swing boost must be set BEFORE _swing_acc_des.
        self._jump_flight_swing_boost = bool(
            jump_now
            and getattr(getattr(active_gait, "phase", None), "value", "") == "flight"
        )

        swing_acc, foot_pos_des = self._swing_acc_des(
            state,
            targets,
            current_base_z,
            vel_xyz,
            use_est,
            leg_is_stance,
            foot_pos,
            foot_vel,
        )
        # Standstill: kill foot world-XY velocity. Stance a=0 would keep a
        # constant scrub once soft contact starts sliding.
        # Use scrub-aware base rates (estimator under-reads ~5× while standing).
        stance_acc = {leg: np.zeros(3) for leg in _LEGS}
        jump_phase = (
            getattr(getattr(active_gait, "phase", None), "value", "")
            if jump_now else ""
        )
        # During crouch/push/flight: don't burn friction cone / QP on XY scrub brake.
        scrub_brake = hold_still and jump_phase not in ("crouch", "push", "flight")
        if scrub_brake:
            v_pin[0] = float(vx_brake)
            v_pin[1] = float(vy_brake)
            _, _, _, foot_vel = self._foot_kinematics(q_pin, v_pin)
            kd_foot = 40.0
            for leg in _LEGS:
                if not leg_is_stance.get(leg, True):
                    continue
                v_f = np.asarray(foot_vel[leg], dtype=float).reshape(3)
                # Fallback to base scrub if FK still under-reads.
                vx_f = float(v_f[0]) if abs(float(v_f[0])) > 0.02 else float(vx_brake)
                vy_f = float(v_f[1]) if abs(float(v_f[1])) > 0.02 else float(vy_brake)
                stance_acc[leg] = np.array(
                    [-kd_foot * vx_f, -kd_foot * vy_f, 0.0]
                )
        elif hold_still:
            # Still feed truth rates into dynamics during launch.
            v_pin[0] = float(vx_brake)
            v_pin[1] = float(vy_brake)

        f_c_des = None
        if self._force_planner is not None:
            x0 = np.zeros(13)
            x0[0] = state.roll
            x0[1] = state.pitch
            x0[2] = state.yaw
            x0[5] = current_base_z
            x0[6] = state.gyro_roll
            x0[7] = state.gyro_pitch
            x0[8] = state.gyro_yaw
            # Spot zeros XY rates. Hold-still feeds scrub-aware vx so MPC brakes.
            if spot:
                x0[9] = 0.0
                x0[10] = 0.0
            elif hold_still:
                x0[9] = float(vx_brake)
                x0[10] = float(
                    vy_brake if abs(vy_brake) > abs(float(vel_xyz[1])) else vel_xyz[1]
                )
            else:
                x0[9] = vx_for_track
                x0[10] = vel_xyz[1]
            x0[11] = float(vz_cmd) if jump_now else vel_xyz[2]
            x0[12] = -9.81

            H = self._force_planner.mpc.cfg.horizon
            dt_mpc = self._force_planner.mpc.cfg.dt
            # Cruise: diagonal CoM kick. Spot: CoM-into-support-triangle from
            # SpotYawStepper (decoupled; zero if gait has no get_spot_com_shift).
            st = leg_is_stance
            diag = (
                (1.0 if st.get("fl", True) else 0.0)
                + (1.0 if st.get("rr", True) else 0.0)
                - (1.0 if st.get("fr", True) else 0.0)
                - (1.0 if st.get("rl", True) else 0.0)
            )
            x_shift = 0.0
            if spot and hasattr(active_gait, "get_spot_com_shift"):
                sx, sy = active_gait.get_spot_com_shift(t_rel)
                x_shift = float(sx)
                y_shift = float(sy)
            elif (
                not spot
                and getattr(active_gait, "family", None) == "walk"
                and hasattr(active_gait, "get_com_y_shift")
            ):
                # Four-beat Walk: phase-locked CoM sway (∞-like lateral), not
                # SoftTrot diagonal kick.
                y_shift = float(active_gait.get_com_y_shift(t_rel))
            else:
                y_shift = 0.0 if spot else float(cfg.com_y_shift_m) * 0.5 * diag

            x_ref = np.zeros(13 * H)
            wz_ref = float(wz_cmd)
            yaw0 = (
                float(getattr(getattr(active_gait, "_spot", None), "yaw_des", state.yaw))
                if spot else state.yaw
            )
            walk_com = (
                not spot
                and getattr(active_gait, "family", None) == "walk"
                and hasattr(active_gait, "get_com_y_shift")
            )
            for k in range(H):
                dt_k = k * dt_mpc
                x_ref[k * 13 + 0] = target_roll
                x_ref[k * 13 + 1] = target_pitch
                x_ref[k * 13 + 2] = yaw0 + wz_ref * dt_k
                x_ref[k * 13 + 8] = wz_ref
                x_ref[k * 13 + 3] = vx_cmd * dt_k + x_shift
                y_k = (
                    float(active_gait.get_com_y_shift(t_rel + dt_k))
                    if walk_com else y_shift
                )
                x_ref[k * 13 + 4] = vy_cmd * dt_k + y_k
                x_ref[k * 13 + 5] = (current_base_z if jump_now else target_z) + (vz_cmd * dt_k if jump_now else 0.0)
                x_ref[k * 13 + 9] = vx_cmd
                x_ref[k * 13 + 10] = vy_cmd
                x_ref[k * 13 + 11] = float(vz_cmd) if jump_now else 0.0
                x_ref[k * 13 + 12] = -9.81

            r_feet = np.zeros((3, 4))
            com = self._wbc_ctrl.data.com[0]
            pin.centerOfMass(self._wbc_ctrl.model, self._wbc_ctrl.data, q_pin, v_pin)
            com = self._wbc_ctrl.data.com[0]
            for i, foot_name in enumerate(self._wbc_ctrl.foot_names):
                foot_id = self._wbc_ctrl.foot_ids[i]
                r_feet[:, i] = self._wbc_ctrl.data.oMf[foot_id].translation - com

            contact_h = self._contact.horizon(
                t_rel=t_rel,
                gait=active_gait,
                horizon=H,
                dt=dt_mpc,
                measured=leg_is_stance,
            )
            # Spot: MPC horizon follows unitree diagonal duty (via SpotYawStepper).
            if spot:
                stepper = getattr(active_gait, "_spot", None)
                per = float(getattr(active_gait, "period", 1.0) or 1.0)
                if stepper is not None and hasattr(stepper, "predict_force_scale"):
                    contact_h = np.zeros(4 * H, dtype=float)
                    for k in range(H):
                        t_k = t_rel + k * dt_mpc
                        for li, leg in enumerate(_LEGS):
                            contact_h[k * 4 + li] = float(
                                stepper.predict_force_scale(leg, t_k, per)
                            )
            elif jump_now:
                # Predict jump force scale across horizon.
                in_flight = bool(
                    getattr(active_gait, "in_flight", lambda: False)()
                )
                phase_u = 0.0
                if hasattr(active_gait, "_phase_u"):
                    try:
                        phase_u = float(active_gait._phase_u(t_rel))
                    except Exception:
                        phase_u = 0.0
                still_any = bool(
                    contact_snap is not None
                    and any(bool(contact_snap.measured.get(leg, False)) for leg in _LEGS)
                )
                vz_meas = float(getattr(state, "vel_xyz", (0.0, 0.0, 0.0))[2])
                # Early flight + slow + still planted: tiny horizon hold only.
                hold_push = (
                    in_flight and still_any and phase_u < 0.10 and vz_meas < 0.40
                )
                contact_h = np.zeros(4 * H, dtype=float)
                for k in range(H):
                    t_k = t_rel + k * dt_mpc
                    jfs = float(
                        active_gait.predict_jump_force_scale(t_k)
                        if hasattr(active_gait, "predict_jump_force_scale")
                        else (0.0 if in_flight else 1.0)
                    )
                    if hold_push:
                        jfs = max(jfs, 0.7)
                    for li in range(4):
                        contact_h[k * 4 + li] = jfs
            force_scale = (
                contact_snap.force_scale
                if contact_snap is not None
                else {leg: 1.0 for leg in _LEGS}
            )
            # Spot: open-dog yaw is the task — default Q barely tracks yaw (4)
            # vs roll (90), so ayaw alone scrubs. Boost yaw/wz, pin XY.
            q_prev = None
            lpf_prev = None
            df_prev = None
            if spot:
                q_prev = np.array(self._force_planner.mpc.cfg.weights_Q, copy=True)
                q = np.array(q_prev, copy=True)
                q[2] = 80.0   # yaw
                q[3] = 40.0   # x hold
                q[4] = 40.0   # y hold
                q[8] = 40.0   # wz
                q[9] = 60.0   # vx → 0
                q[10] = 60.0  # vy → 0
                self._force_planner.mpc.cfg.weights_Q = q
            elif hold_still:
                # Stand / zero-cmd: pin vx so soft soles don't scrape forward.
                q_prev = np.array(self._force_planner.mpc.cfg.weights_Q, copy=True)
                q = np.array(q_prev, copy=True)
                q[3] = max(float(q[3]), 30.0)   # x hold
                q[9] = max(float(q[9]), 55.0)   # vx → 0
                q[10] = max(float(q[10]), 40.0)  # vy → 0
                self._force_planner.mpc.cfg.weights_Q = q
            if jump_now:
                # Fast force ramp — default 700N/s is too slow for a short push.
                lpf_prev = float(self._force_planner.force_lpf_alpha)
                df_prev = float(self._force_planner.max_df_dt)
                self._force_planner.force_lpf_alpha = max(lpf_prev, 0.35)
                self._force_planner.max_df_dt = max(df_prev, 2800.0)
                if q_prev is None:
                    q_prev = np.array(self._force_planner.mpc.cfg.weights_Q, copy=True)
                q = np.array(self._force_planner.mpc.cfg.weights_Q, copy=True)
                q[5] = max(float(q[5]), 80.0)   # z
                q[11] = max(float(q[11]), 50.0)  # vz
                phase_q = getattr(
                    getattr(active_gait, "phase", None), "value", ""
                )
                if phase_q == "push":
                    q[11] = max(float(q[11]), 80.0)  # chase push_vz
                    q[9] = min(float(q[9]), 15.0)    # don't brake launch
                else:
                    q[9] = max(float(q[9]), 55.0)    # kill forward creep
                self._force_planner.mpc.cfg.weights_Q = q
            try:
                f_c_des = self._force_planner.plan(
                    x0=x0,
                    x_ref=x_ref,
                    r_feet=r_feet,
                    contact_horizon=contact_h,
                    force_scale=force_scale,
                    dt=0.005,  # fixed control period; t_rel can reset at gait start
                )
                # Jump PUSH: amplify vertical impulse; soft-land scales Fz only.
                if jump_now:
                    phase_name = getattr(
                        getattr(active_gait, "phase", None), "value", ""
                    )
                    jfs = float(
                        active_gait.jump_force_scale_at(t_rel)
                        if hasattr(active_gait, "jump_force_scale_at")
                        else 1.0
                    )
                    if phase_name in ("land", "recover") and jfs < 0.99:
                        for li in range(4):
                            f_c_des[li * 3 + 2] *= max(0.15, jfs)
                    if phase_name == "push":
                        # ~2× body weight; keep front/rear balanced (rear bias → 二弹).
                        f_cap = 60.0
                        for li, leg in enumerate(_LEGS):
                            s = 1.06 if leg in ("rl", "rr") else 1.02
                            fz = max(float(f_c_des[li * 3 + 2]) * s * 1.80, 32.0 * s)
                            f_c_des[li * 3 + 2] = min(f_cap * s, fz)
                        zmin = min(float(foot_z.get(leg, 0.0)) for leg in _LEGS)
                        if zmin < -0.030:
                            dig = min(1.0, (-0.030 - zmin) / 0.020)
                            fz_s = max(0.80, 1.0 - 0.20 * dig)
                            for li in range(4):
                                f_c_des[li * 3 + 2] *= fz_s
                    elif phase_name == "crouch":
                        for li, leg in enumerate(_LEGS):
                            s = 1.05 if leg in ("rl", "rr") else 0.97
                            f_c_des[li * 3 + 2] *= s
                # Hold-still XY brake — skip during launch (needs friction for Fz).
                if scrub_brake:
                    n_st = sum(
                        1 for leg in _LEGS if leg_is_stance.get(leg, True)
                    ) or 4
                    fx_tot = -80.0 * float(vx_brake) - 10.0 * np.sign(
                        float(vx_brake) + 1e-6
                    )
                    fy_tot = -40.0 * float(vy_brake)
                    share = 1.0 / float(n_st)
                    for li, leg in enumerate(_LEGS):
                        if not leg_is_stance.get(leg, True):
                            continue
                        f_c_des[li * 3 + 0] += fx_tot * share
                        f_c_des[li * 3 + 1] += fy_tot * share
            finally:
                if q_prev is not None:
                    self._force_planner.mpc.cfg.weights_Q = q_prev
                if lpf_prev is not None:
                    self._force_planner.force_lpf_alpha = lpf_prev
                if df_prev is not None:
                    self._force_planner.max_df_dt = df_prev
        else:
            force_scale = (
                contact_snap.force_scale
                if contact_snap is not None
                else {leg: 1.0 for leg in _LEGS}
            )

        # Spot: raise yaw/XY hold + swing tracking; soften stance lock.
        wbc_w_prev = None
        wbc_st_prev = None
        wbc_sw_prev = None
        if (spot or jump_now or hold_still) and self._wbc_ctrl is not None:
            wbc_w_prev = np.array(self._wbc_ctrl.config.weight_base_acc, copy=True)
            wbc_st_prev = float(self._wbc_ctrl.config.weight_stance_acc)
            wbc_sw_prev = float(self._wbc_ctrl.config.weight_swing_acc)
            w = np.array(wbc_w_prev, copy=True)
            wbc_ft_prev = None
            if spot:
                w[0, 0] = 45.0   # ax — kill creep
                w[1, 1] = 45.0   # ay
                w[3, 3] = max(float(w[3, 3]), 70.0)  # keep roll while yawing
                w[5, 5] = 120.0  # ayaw assist (CoM+3-foot do the plant)
                self._wbc_ctrl.config.weight_stance_acc = 8.0
                self._wbc_ctrl.config.weight_swing_acc = max(wbc_sw_prev, 18.0)
            elif hold_still:
                w[0, 0] = max(float(w[0, 0]), 55.0)  # ax — standstill brake
                w[1, 1] = max(float(w[1, 1]), 40.0)
                self._wbc_ctrl.config.weight_stance_acc = max(wbc_st_prev, 40.0)
                wbc_ft_prev = float(self._wbc_ctrl.config.weight_force_tracking)
                self._wbc_ctrl.config.weight_force_tracking = max(wbc_ft_prev, 12.0)
            if jump_now:
                phase_name = getattr(
                    getattr(active_gait, "phase", None), "value", ""
                )
                if phase_name == "push":
                    w[0, 0] = 8.0    # don't steal friction from Fz
                    w[1, 1] = 8.0
                    w[2, 2] = 90.0   # vertical launch
                else:
                    w[0, 0] = max(float(w[0, 0]), 55.0)  # hold XY during hop
                    w[2, 2] = 55.0  # Strongly track Z acceleration for jump
                w[4, 4] = max(float(w[4, 4]), 90.0)  # hold pitch — stop nose-up peel
                wbc_fmax_prev = float(self._wbc_ctrl.config.f_max)
                if phase_name == "push":
                    self._wbc_ctrl.config.f_max = max(wbc_fmax_prev, 140.0)
                else:
                    wbc_fmax_prev = None
                if phase_name == "flight":
                    self._wbc_ctrl.config.weight_stance_acc = 1.0
                    self._wbc_ctrl.config.weight_swing_acc = max(wbc_sw_prev, 28.0)
                elif phase_name == "push":
                    # Softer stance lock so feet can leave while Fz is high.
                    self._wbc_ctrl.config.weight_stance_acc = min(
                        max(wbc_st_prev, 12.0), 18.0
                    )
                else:
                    # Keep stance foot-scrub brake strong on ground.
                    self._wbc_ctrl.config.weight_stance_acc = max(wbc_st_prev, 35.0)
                if wbc_ft_prev is None:
                    wbc_ft_prev = float(self._wbc_ctrl.config.weight_force_tracking)
                self._wbc_ctrl.config.weight_force_tracking = max(
                    wbc_ft_prev, 28.0 if phase_name == "push" else 20.0
                )
            else:
                wbc_fmax_prev = None
                if not hold_still:
                    wbc_ft_prev = None
            self._wbc_ctrl.config.weight_base_acc = w
        else:
            wbc_ft_prev = None
            wbc_fmax_prev = None
        try:
            tau_opt = self._wbc_ctrl.compute_tau(
                q_pin=q_pin,
                v_pin=v_pin,
                base_acc_des=base_acc_des,
                leg_is_stance=leg_is_stance,
                f_c_des=f_c_des,
                swing_acc_des=swing_acc,
                force_scale=force_scale,
                stance_acc_des=stance_acc,
            )
        finally:
            if wbc_w_prev is not None:
                self._wbc_ctrl.config.weight_base_acc = wbc_w_prev
                self._wbc_ctrl.config.weight_stance_acc = wbc_st_prev
            if wbc_sw_prev is not None:
                self._wbc_ctrl.config.weight_swing_acc = wbc_sw_prev
            if wbc_ft_prev is not None:
                self._wbc_ctrl.config.weight_force_tracking = wbc_ft_prev
            if wbc_fmax_prev is not None:
                self._wbc_ctrl.config.f_max = wbc_fmax_prev

        # Light gravity assist blended under WBC torque (swing + stance)
        grav = gravity_trq(targets, 0.20)
        # Jump PUSH: amplify τ_ff so Jc^T F is not drowned by residual PD.
        jump_tau_scale = 1.0
        if jump_now:
            phase_name = getattr(
                getattr(active_gait, "phase", None), "value", ""
            )
            if phase_name == "push":
                jump_tau_scale = 3.0
            elif phase_name == "crouch":
                jump_tau_scale = 1.5
            elif phase_name in ("land", "recover"):
                jump_tau_scale = 0.7  # don't smash back up to stand
        out_trq = {}
        for i, jname_joint in enumerate(self._wbc_ctrl.actuated_joint_names):
            jname = jname_joint.replace("_joint", "")
            if jname not in JBN:
                continue
            mid = JBN[jname].motor_id
            if mid not in targets:
                continue
            # Always use WBC tau (includes swing tracking); mild grav blend
            out_trq[mid] = float(tau_opt[i]) * jump_tau_scale + grav.get(mid, 0.0)

        foot_pos_flat = np.zeros(12)
        foot_des_flat = np.zeros(12)
        foot_z_arr = np.zeros(4)
        foot_vz_arr = np.zeros(4)
        for i, leg in enumerate(_LEGS):
            foot_pos_flat[i * 3 : i * 3 + 3] = foot_pos[leg]
            foot_des_flat[i * 3 : i * 3 + 3] = foot_pos_des[leg]
            foot_z_arr[i] = float(foot_z[leg])
            foot_vz_arr[i] = float(foot_vz[leg])

        if self._dyn_tel is not None:
            truth = getattr(state, "vel_xyz", (0.0, 0.0, 0.0))
            fs = force_scale if force_scale is not None else {leg: 1.0 for leg in _LEGS}
            force_scale_arr = [float(fs.get(leg, 1.0)) for leg in _LEGS]

            phase_arr = [0.0, 0.0, 0.0, 0.0]
            amp_front = 0.0
            amp_rear = 0.0
            period = 0.0
            stance_ratio = 0.0
            speed_frac = 0.0
            ramp_frac = 1.0
            if active_gait is not None:
                period = float(getattr(active_gait, "period", 0.0) or 0.0)
                stance_ratio = float(getattr(active_gait, "stance_ratio", 0.0) or 0.0)
                amp_front = float(getattr(active_gait, "amp_front", 0.0) or 0.0)
                amp_rear = float(getattr(active_gait, "amp_rear", amp_front) or 0.0)
                speed_frac = float(getattr(active_gait, "speed_frac", 0.0) or 0.0)
                offsets = getattr(active_gait, "_PHASE_OFFSET", None)
                if period > 1e-6 and isinstance(offsets, dict):
                    for i, leg in enumerate(_LEGS):
                        phase_arr[i] = (t_rel / period + float(offsets.get(leg, 0.0))) % 1.0
                ramp_dur = float(getattr(active_gait, "ramp_duration", 0.0) or 0.0)
                if ramp_dur > 1e-6 and t_rel < ramp_dur:
                    s = t_rel / ramp_dur
                    ramp_frac = s * s * (3.0 - 2.0 * s)
                else:
                    ramp_frac = 1.0

            # Joint tracking RMS (targets vs measured, actuated WBC joints)
            q_err_sq = 0.0
            q_err_n = 0
            for jname_joint in self._wbc_ctrl.actuated_joint_names:
                jname = jname_joint.replace("_joint", "")
                if jname not in JBN:
                    continue
                mid = JBN[jname].motor_id
                if mid not in targets:
                    continue
                q_act = float(state.joint_pos.get(mid, targets[mid]))
                dq = float(targets[mid]) - q_act
                q_err_sq += dq * dq
                q_err_n += 1
            q_err_rms = math.sqrt(q_err_sq / q_err_n) if q_err_n else 0.0

            prev_tau = getattr(self, "_tel_prev_tau", None)
            if prev_tau is not None and prev_tau.shape == tau_opt.shape:
                dtau_max = float(np.max(np.abs(tau_opt - prev_tau)))
            else:
                dtau_max = 0.0
            self._tel_prev_tau = tau_opt.copy()

            self._dyn_tel.record(
                t=t_rel,
                roll=state.roll,
                pitch=state.pitch,
                z=current_base_z,
                vx=vel_xyz[0],
                vy=vel_xyz[1],
                wz=v_pin[5],
                vx_truth=float(truth[0]),
                vy_truth=float(truth[1]),
                vz_truth=float(truth[2]),
                fc_des=f_c_des.copy() if f_c_des is not None else np.zeros(12),
                tau_opt=tau_opt.copy(),
                contact_state=[1.0 if leg_is_stance.get(l, True) else 0.0 for l in _LEGS],
                contact_measured=(
                    [1.0 if contact_snap.measured[l] else 0.0 for l in _LEGS]
                    if contact_snap
                    else [0.0] * 4
                ),
                contact_scheduled=(
                    [1.0 if contact_snap.scheduled[l] else 0.0 for l in _LEGS]
                    if contact_snap
                    else [0.0] * 4
                ),
                force_scale=force_scale_arr,
                phase=phase_arr,
                amp_front=amp_front,
                amp_rear=amp_rear,
                period=period,
                stance_ratio=stance_ratio,
                speed_frac=speed_frac,
                ramp_frac=ramp_frac,
                vx_cmd=vx_cmd,
                vy_cmd=vy_cmd,
                base_acc_des=base_acc_des.copy(),
                foot_pos_actual=foot_pos_flat,
                foot_pos_des=foot_des_flat,
                foot_z=foot_z_arr,
                foot_vz=foot_vz_arr,
                q_err_rms=q_err_rms,
                dtau_max=dtau_max,
                mpc_ok=bool(self._force_planner.last_ok) if self._force_planner else False,
                wbc_ok=bool(self._wbc_ctrl.last_solve_ok),
                estimate_mode=self.config.base_estimate_mode,
            )

        return out_trq

    def _apply_vmc(
        self,
        state: RobotState,
        targets: dict[int, float],
        trq_ff: Optional[dict[int, float]],
        leg_is_stance: dict[str, bool],
    ) -> dict[int, float]:
        if not self._vmc_ctrl:
            return trq_ff or {}

        out_trq = dict(trq_ff) if trq_ff else {}
        urdf_targets = {"fl": {}, "fr": {}, "rl": {}, "rr": {}}
        urdf_current = {"fl": {}, "fr": {}, "rl": {}, "rr": {}}

        for leg in ["fl", "fr"]:
            for key, jname in [
                ("hip_pitch", f"{leg}_hip_pitch"),
                ("calf", f"{leg}_calf"),
                ("tarsus", f"{leg}_tarsus"),
            ]:
                j = JBN[jname]
                urdf_targets[leg][key] = targets.get(j.motor_id, 0.0)
                urdf_current[leg][key] = state.joint_pos.get(j.motor_id, 0.0)
        for leg in ["rl", "rr"]:
            for key, jname in [
                ("thigh", f"{leg}_thigh"),
                ("calf", f"{leg}_calf"),
            ]:
                j = JBN[jname]
                urdf_targets[leg][key] = targets.get(j.motor_id, 0.0)
                urdf_current[leg][key] = state.joint_pos.get(j.motor_id, 0.0)

        leg_z_targets = {}
        leg_z_current = {}
        for leg in ["fl", "fr"]:
            _, leg_z_targets[leg] = K.fk_front_3link(
                urdf_targets[leg]["hip_pitch"],
                urdf_targets[leg]["calf"],
                urdf_targets[leg].get("tarsus", 0.0),
            )
            _, leg_z_current[leg] = K.fk_front_3link(
                urdf_current[leg]["hip_pitch"],
                urdf_current[leg]["calf"],
                urdf_current[leg].get("tarsus", 0.0),
            )
        for leg in ["rl", "rr"]:
            _, leg_z_targets[leg] = K.fk_rear_2d(
                urdf_targets[leg]["thigh"], urdf_targets[leg]["calf"]
            )
            _, leg_z_current[leg] = K.fk_rear_2d(
                urdf_current[leg]["thigh"], urdf_current[leg]["calf"]
            )

        leg_vz_current = {}
        fz_cmd = self._vmc_ctrl.compute_fz(
            leg_z_targets=leg_z_targets,
            leg_z_current=leg_z_current,
            leg_vz_current=leg_vz_current,
            roll=state.roll,
            roll_rate=state.gyro_roll,
            leg_is_stance=leg_is_stance,
        )
        urdf_torques = self._vmc_ctrl.compute_joint_torques(fz_cmd, urdf_current)

        for leg in ["fl", "fr"]:
            for key, jname in [
                ("hip_pitch", f"{leg}_hip_pitch"),
                ("calf", f"{leg}_calf"),
            ]:
                j = JBN[jname]
                if j.motor_id in targets:
                    out_trq[j.motor_id] = urdf_torques[leg].get(key, 0.0)
        for leg in ["rl", "rr"]:
            for key, jname in [
                ("thigh", f"{leg}_thigh"),
                ("calf", f"{leg}_calf"),
            ]:
                j = JBN[jname]
                if j.motor_id in targets:
                    out_trq[j.motor_id] = urdf_torques[leg].get(key, 0.0)
        return out_trq

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
