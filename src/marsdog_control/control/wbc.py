"""Weighted whole-body QP on the reduced Marsdog model (legs only)."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Optional

import numpy as np
import pinocchio as pin
import qpsolvers

from marsdog_control.control.nmpc_reduced_model import (
    LEG_ACTUATED_JOINT_NAMES,
    QuadrupedReducedModel,
    default_urdf_path,
)


@dataclass
class WbcConfig:
    urdf_path: str = field(default_factory=default_urdf_path)
    mu: float = 0.6
    f_min: float = 2.0
    f_max: float = 80.0  # ~3 * mg/4 for 10.5 kg

    # Track z/roll/pitch + lateral; roll weight dominant vs diagonal rock
    weight_base_acc: np.ndarray = field(
        default_factory=lambda: np.diag([2.0, 16.0, 22.0, 48.0, 15.0, 2.0])
    )
    weight_stance_acc: float = 50.0  # firmer stance feet
    weight_swing_acc: float = 12.0  # less aggressive swing chase → less roll kick
    weight_posture: float = 0.5
    weight_force: float = 1e-3
    weight_force_tracking: float = 3.5
    weight_torque: float = 1e-3
    tau_limit_nm: float = 25.0
    # Post-QP joint torque gain (1.0=full; <1 softens feedforward on real robot).
    tau_scale: float = 0.5
    # Below this contact weight, enforce F≈0 hard (deep swing). Above: soft blend.
    soft_contact_eps: float = 0.05

    kp_base_z: float = 30.0
    kd_base_z: float = 10.0
    kp_base_roll: float = 85.0
    kd_base_roll: float = 24.0
    kp_base_pitch: float = 35.0
    kd_base_pitch: float = 8.0

class WholeBodyController:
    """QP: x = [v_dot (nv), F_c (12), tau (n_actuated_legs)]."""

    def __init__(
        self,
        config: Optional[WbcConfig] = None,
        reduced: Optional[QuadrupedReducedModel] = None,
    ):
        self.config = config or WbcConfig()
        self.reduced = reduced or QuadrupedReducedModel(self.config.urdf_path)
        self.model = self.reduced.model
        self.data = self.reduced.data

        self.nv = self.model.nv
        self.nq = self.model.nq

        self.actuated_joint_names = []
        for jname in LEG_ACTUATED_JOINT_NAMES:
            if self.model.existJointName(jname):
                self.actuated_joint_names.append(jname)
            else:
                print(f"[WBC] Warning: actuated joint {jname} missing in reduced model")

        self.n_actuated = len(self.actuated_joint_names)
        self.S_T = np.zeros((self.nv, self.n_actuated))
        for i, jname in enumerate(self.actuated_joint_names):
            idx_v = self.model.joints[self.model.getJointId(jname)].idx_v
            self.S_T[idx_v, i] = 1.0

        self.foot_names = ["fl_foot_link", "fr_foot_link", "rl_foot_link", "rr_foot_link"]
        self.foot_ids = [self.model.getFrameId(n) for n in self.foot_names]

        self._last_tau = np.zeros(self.n_actuated)
        self.last_solve_ok = False
        self.fail_count = 0

    def compute_tau(
        self,
        q_pin: np.ndarray,
        v_pin: np.ndarray,
        base_acc_des: np.ndarray,
        leg_is_stance: dict[str, bool],
        f_c_des: np.ndarray = None,
        swing_acc_des: dict[str, np.ndarray] = None,
        postural_acc_des: np.ndarray = None,
        force_scale: dict[str, float] = None,
        stance_acc_des: dict[str, np.ndarray] = None,
    ) -> np.ndarray:
        """Solve WBC QP.

        ``force_scale`` (preferred) continuously blends stance↔swing so LO/TD
        does not hard-flip QP structure. Falls back to binary ``leg_is_stance``
        when omitted (unit tests / legacy callers).

        ``stance_acc_des``: per-leg world-frame foot accel for stance (default 0).
        Needed to kill soft-contact scrub — pure a=0 keeps a constant slide.
        """
        if swing_acc_des is None:
            swing_acc_des = {}
        if stance_acc_des is None:
            stance_acc_des = {}

        # Continuous contact weight s∈[0,1]: 1=full stance, 0=full swing.
        # Binary stance alone caused ~30× torque steps at schedule flips.
        scales: dict[str, float] = {}
        for foot_name in self.foot_names:
            leg = foot_name[:2]
            if force_scale is not None and leg in force_scale:
                scales[leg] = float(np.clip(force_scale[leg], 0.0, 1.0))
            else:
                scales[leg] = 1.0 if leg_is_stance.get(leg, True) else 0.0

        pin.computeAllTerms(self.model, self.data, q_pin, v_pin)
        pin.updateFramePlacements(self.model, self.data)

        M = self.data.M
        h = self.data.nle

        J_c_list = []
        for i in range(4):
            J_foot = pin.getFrameJacobian(
                self.model,
                self.data,
                self.foot_ids[i],
                pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
            )
            J_c_list.append(J_foot[:3, :])
        J_c = np.vstack(J_c_list)

        nx = self.nv + 12 + self.n_actuated

        # Dynamics equality
        A_dyn = np.hstack([M, -J_c.T, -self.S_T])
        b_dyn = -h
        A_eq = [A_dyn]
        b_eq = [b_dyn]

        # Deep swing only: hard F=0. Near edges, keep force free under soft bounds.
        eps = float(self.config.soft_contact_eps)
        a_drift_list = []
        for i, foot_name in enumerate(self.foot_names):
            leg_key = foot_name[:2]
            a_drift = pin.getFrameClassicalAcceleration(
                self.model,
                self.data,
                self.foot_ids[i],
                pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
            ).linear
            a_drift_list.append(a_drift)
            if scales[leg_key] < eps:
                A_f0 = np.zeros((3, nx))
                A_f0[:, self.nv + 3 * i : self.nv + 3 * i + 3] = np.eye(3)
                A_eq.append(A_f0)
                b_eq.append(np.zeros(3))

        A_eq = np.vstack(A_eq)
        b_eq = np.concatenate(b_eq)

        # Friction cone inequalities (scaled by continuous contact weight)
        G_ineq = []
        h_ineq = []
        mu = self.config.mu
        for i in range(4):
            leg_key = self.foot_names[i][:2]
            s = scales[leg_key]
            if s < eps:
                f_min_i = 0.0
                f_max_i = 0.0
            else:
                f_min_i = self.config.f_min * s
                f_max_i = self.config.f_max * s
            G_f = np.zeros((6, nx))
            idx = self.nv + 3 * i
            G_f[0, idx] = 1.0
            G_f[0, idx + 2] = -mu
            G_f[1, idx] = -1.0
            G_f[1, idx + 2] = -mu
            G_f[2, idx + 1] = 1.0
            G_f[2, idx + 2] = -mu
            G_f[3, idx + 1] = -1.0
            G_f[3, idx + 2] = -mu
            G_f[4, idx + 2] = -1.0
            G_f[5, idx + 2] = 1.0
            h_f = np.array([0.0, 0.0, 0.0, 0.0, -f_min_i, f_max_i])
            G_ineq.append(G_f)
            h_ineq.append(h_f)
        G_ineq = np.vstack(G_ineq)
        h_ineq = np.concatenate(h_ineq)

        P = np.zeros((nx, nx))
        q_obj = np.zeros(nx)

        P[:6, :6] = self.config.weight_base_acc
        q_obj[:6] = -self.config.weight_base_acc @ base_acc_des

        P[6 : self.nv, 6 : self.nv] = np.eye(self.nv - 6) * self.config.weight_posture
        if postural_acc_des is not None:
            q_obj[6 : self.nv] = -self.config.weight_posture * postural_acc_des

        # Soft blend: stance tracks a_st (default 0); swing tracks a_sw.
        w_st = self.config.weight_stance_acc
        w_sw = self.config.weight_swing_acc
        for i, foot_name in enumerate(self.foot_names):
            leg_key = foot_name[:2]
            Ji = J_c_list[i]
            s = scales[leg_key]
            a_sw = np.asarray(
                swing_acc_des.get(leg_key, np.zeros(3)), dtype=float
            ).reshape(3)
            a_st = np.asarray(
                stance_acc_des.get(leg_key, np.zeros(3)), dtype=float
            ).reshape(3)
            a_des = (1.0 - s) * a_sw + s * a_st
            w = s * w_st + (1.0 - s) * w_sw
            P[: self.nv, : self.nv] += w * (Ji.T @ Ji)
            q_obj[: self.nv] += w * (Ji.T @ (a_drift_list[i] - a_des))

        if f_c_des is not None:
            P[self.nv : self.nv + 12, self.nv : self.nv + 12] = (
                np.eye(12) * self.config.weight_force_tracking
            )
            q_obj[self.nv : self.nv + 12] = (
                -self.config.weight_force_tracking * f_c_des
            )
        else:
            P[self.nv : self.nv + 12, self.nv : self.nv + 12] = (
                np.eye(12) * self.config.weight_force
            )

        P[self.nv + 12 :, self.nv + 12 :] = (
            np.eye(self.n_actuated) * self.config.weight_torque
        )

        P += np.eye(nx) * 1e-6
        P = 0.5 * (P + P.T)

        try:
            import warnings

            # Prefer clarabel (sim parity); fall back to whatever qpsolvers has.
            available = set(qpsolvers.available_solvers)
            solver = next(
                (s for s in ("clarabel", "osqp", "proxqp", "daqp", "scs") if s in available),
                None,
            )
            if solver is None:
                raise RuntimeError(
                    f"no QP solver available (found: {sorted(available)})"
                )
            with warnings.catch_warnings():
                warnings.simplefilter("ignore")
                x_opt = qpsolvers.solve_qp(
                    P, q_obj, G_ineq, h_ineq, A_eq, b_eq, solver=solver
                )
        except Exception as e:
            print("[WBC] QP Solver error:", e)
            x_opt = None

        if x_opt is None:
            self.last_solve_ok = False
            self.fail_count += 1
            return self._last_tau.copy()

        self.last_solve_ok = True
        self.fail_count = 0
        tau_opt = np.clip(
            x_opt[self.nv + 12 :],
            -self.config.tau_limit_nm,
            self.config.tau_limit_nm,
        )
        tau_opt = tau_opt * float(self.config.tau_scale)
        self._last_tau = tau_opt.copy()
        return tau_opt
