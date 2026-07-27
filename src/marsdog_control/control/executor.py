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
                f"estimate={self.config.base_estimate_mode}"
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

        return ControlOutput(
            target=MotionTarget(
                q=targets, dq=velocities, source_mode=motion.source_mode
            ),
            kp_phase=kp_phase,
            trq_ff=trq_ff,
            kp_scale=self.config.kp_scale,
            leg_kp_scale=self.config.leg_kp_scale,
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
        dt = max(1e-3, self._ctrl_dt)
        out = {}
        # Always compute PD targets; WBC blends by force_scale (stance→0 weight).
        _ = leg_is_stance
        for leg in _LEGS:
            p_des = foot_pos_des[leg]
            p_prev = self._foot_des_prev.get(leg, p_des)
            v_des_leg = (p_des - p_prev) / dt
            self._foot_des_prev[leg] = p_des.copy()
            p_act = foot_pos_act[leg]
            v_act = foot_vel_act[leg]
            out[leg] = kp * (p_des - p_act) + kd * (v_des_leg - v_act)
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
            contact_snap = self._contact.update(
                t_rel=t_rel,
                gait=active_gait,
                foot_z_world=foot_z,
                foot_vz_world=foot_vz,
            )
            leg_is_stance = dict(contact_snap.stance)

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
        target_roll = 0.0
        target_pitch = 0.0

        vx_cmd = 0.0
        vy_cmd = 0.0
        if active_gait:
            if hasattr(active_gait, "vel_cmd"):
                vx_cmd = active_gait.vel_cmd[0]
                vy_cmd = active_gait.vel_cmd[1]
            elif hasattr(active_gait, "amp_front") and hasattr(active_gait, "period"):
                avg_amp = (
                    active_gait.amp_front
                    + getattr(active_gait, "amp_rear", active_gait.amp_front)
                ) / 2.0
                vx_cmd = (avg_amp * 2.0) / active_gait.period

        cfg = self.config
        # Lateral velocity: light EMA then damp (cuts estimator noise, keeps authority)
        a_vy = 0.25 if use_est else 0.45
        self._vy_filt = (1.0 - a_vy) * getattr(self, "_vy_filt", 0.0) + a_vy * vel_xyz[1]
        # In sim, blend a bit of truth lateral into the damp signal
        truth_vy = float(getattr(state, "vel_xyz", (0.0, 0.0, 0.0))[1])
        if use_est and abs(truth_vy) + abs(self._vy_filt) > 1e-6:
            vy_for_damp = 0.65 * self._vy_filt + 0.35 * truth_vy
        else:
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
        base_acc_des[0] = ax_gain * vx_err
        base_acc_des[1] = -float(cfg.lateral_vel_damp) * vy_for_damp
        base_acc_des[2] = cfg.kp_base_z * (target_z - current_base_z) - cfg.kd_base_z * v_pin[2]
        base_acc_des[3] = (
            cfg.kp_base_roll * (target_roll - state.roll)
            - cfg.kd_base_roll * state.gyro_roll
        )
        base_acc_des[4] = (
            cfg.kp_base_pitch * (target_pitch - state.pitch)
            - cfg.kd_base_pitch * state.gyro_pitch
        )
        # Light yaw rate damping (wz peak was ~1.1 rad/s)
        base_acc_des[5] = -4.0 * state.gyro_yaw

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
            x0[9] = vx_for_track
            x0[10] = vel_xyz[1]
            x0[11] = vel_xyz[2]
            x0[12] = -9.81

            H = self._force_planner.mpc.cfg.horizon
            dt_mpc = self._force_planner.mpc.cfg.dt
            # Diagonal stance: (FL+RR) - (FR+RL). Shift CoM toward support side
            # to oppose gait-locked roll (corr(roll,diag) was strongly negative).
            st = leg_is_stance
            diag = (
                (1.0 if st.get("fl", True) else 0.0)
                + (1.0 if st.get("rr", True) else 0.0)
                - (1.0 if st.get("fr", True) else 0.0)
                - (1.0 if st.get("rl", True) else 0.0)
            )
            # Positive diag (FL+RR) → positive y shift (robot +y = left)
            y_shift = float(cfg.com_y_shift_m) * 0.5 * diag

            x_ref = np.zeros(13 * H)
            for k in range(H):
                dt_k = k * dt_mpc
                x_ref[k * 13 + 0] = target_roll
                x_ref[k * 13 + 1] = target_pitch
                x_ref[k * 13 + 2] = state.yaw
                x_ref[k * 13 + 3] = vx_cmd * dt_k
                x_ref[k * 13 + 4] = vy_cmd * dt_k + y_shift
                x_ref[k * 13 + 5] = target_z
                x_ref[k * 13 + 9] = vx_cmd
                x_ref[k * 13 + 10] = vy_cmd
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
            force_scale = (
                contact_snap.force_scale
                if contact_snap is not None
                else {leg: 1.0 for leg in _LEGS}
            )
            f_c_des = self._force_planner.plan(
                x0=x0,
                x_ref=x_ref,
                r_feet=r_feet,
                contact_horizon=contact_h,
                force_scale=force_scale,
                dt=0.005,  # fixed control period; t_rel can reset at gait start
            )
        else:
            force_scale = (
                contact_snap.force_scale
                if contact_snap is not None
                else {leg: 1.0 for leg in _LEGS}
            )

        tau_opt = self._wbc_ctrl.compute_tau(
            q_pin=q_pin,
            v_pin=v_pin,
            base_acc_des=base_acc_des,
            leg_is_stance=leg_is_stance,
            f_c_des=f_c_des,
            swing_acc_des=swing_acc,
            force_scale=force_scale,
        )

        # Light gravity assist blended under WBC torque (swing + stance)
        grav = gravity_trq(targets, 0.20)
        out_trq = {}
        for i, jname_joint in enumerate(self._wbc_ctrl.actuated_joint_names):
            jname = jname_joint.replace("_joint", "")
            if jname not in JBN:
                continue
            mid = JBN[jname].motor_id
            if mid not in targets:
                continue
            # Always use WBC tau (includes swing tracking); mild grav blend
            out_trq[mid] = float(tau_opt[i]) + grav.get(mid, 0.0)

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
        period = active_gait.period
        stance_ratio = active_gait.stance_ratio
        offsets = active_gait._PHASE_OFFSET
        for mid, leg in _LEG_MOTOR_IDS:
            phase = (t_rel / period + offsets[leg]) % 1.0
            kp_phase[mid] = kp_phase_scale(
                phase,
                stance_ratio,
                self.config.td_kp_scale,
                self.config.swing_kp_scale,
                self.config.td_window,
            )
        return kp_phase


__all__ = ["CommandExecutor", "ExecutorConfig", "gravity_trq", "resolve_gains"]
