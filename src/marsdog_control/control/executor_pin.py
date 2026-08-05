"""Pinocchio state / foot FK / swing Cartesian PD for CommandExecutor."""

from __future__ import annotations

import math
from typing import Dict, Optional

import numpy as np

from marsdog_control.config.joints import JOINT_BY_NAME as JBN
from marsdog_control.core.types import RobotState
from marsdog_control.motion import kinematics as K


_LEGS = ("fl", "fr", "rl", "rr")


class ExecutorPinMixin:
    """Assemble pin q/v, foot kinematics, swing acceleration targets."""

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

