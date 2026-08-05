"""Lock waist/head/tail into the floating base; keep asymmetric legs for WBC/MPC."""

from __future__ import annotations

import os
from typing import Dict, List, Optional, Tuple

import numpy as np
import pinocchio as pin


def default_urdf_path() -> str:
    """Repo-relative URDF: ``marsdog/urdf/marsdog.urdf``.

    Prefer :mod:`marsdog_control.control.paths` as the shared helper; this
    wrapper stays for historical imports from WBC / reduced-model callers.
    """
    from marsdog_control.control.paths import default_urdf_path as _shared

    return _shared()


# WBC / MIT 主动关节：四肢 only；前腿含主动 tarsus；后腿 tarsus 被动 mimic，不进 S。
LEG_ACTUATED_JOINT_NAMES: List[str] = [
    "fl_hip_pitch_joint",
    "fl_thigh_roll_joint",
    "fl_calf_joint",
    "fl_tarsus_joint",
    "fr_hip_pitch_joint",
    "fr_thigh_roll_joint",
    "fr_calf_joint",
    "fr_tarsus_joint",
    "rl_hip_joint",
    "rl_thigh_joint",
    "rl_calf_joint",
    "rr_hip_joint",
    "rr_thigh_joint",
    "rr_calf_joint",
]

# URDF mimic: rear tarsus = -calf (Pinocchio 不会自动施加)
REAR_TARSUS_MIMIC: Dict[str, Tuple[str, float]] = {
    "rl_tarsus_joint": ("rl_calf_joint", -1.0),
    "rr_tarsus_joint": ("rr_calf_joint", -1.0),
}


class QuadrupedReducedModel:
    """降维四足模型：头/颈/尾/腰锁死，质量折入 base；保留前后不对称腿。"""

    def __init__(self, urdf_path: Optional[str] = None):
        urdf_path = urdf_path or default_urdf_path()
        self.urdf_path = urdf_path
        self.full_model = pin.buildModelFromUrdf(urdf_path, pin.JointModelFreeFlyer())

        self.joints_to_lock = [
            "neck_pitch_joint", "head_roll_joint", "head_yaw_joint", "head_pitch_joint",
            "tail1_pitch_joint", "tail1_yaw_joint",
            "tail2_pitch_joint", "tail2_yaw_joint",
            "tail3_pitch_joint", "tail3_yaw_joint",
            "tail4_pitch_joint", "tail4_yaw_joint",
            "tail5_pitch_joint", "tail5_yaw_joint",
            "tail6_pitch_joint", "tail6_yaw_joint",
            "tail7_pitch_joint", "tail7_yaw_joint",
            "tail8_pitch_joint", "tail8_yaw_joint",
            "tail9_pitch_joint", "tail9_yaw_joint",
            "tail10_pitch_joint", "tail10_yaw_joint",
            "tail11_pitch_joint", "tail11_yaw_joint",
            "tail12_pitch_joint", "tail12_yaw_joint",
            "waist_roll_joint", "waist_pitch_joint", "waist_yaw_joint",
        ]

        lock_ids = []
        for j_name in self.joints_to_lock:
            if self.full_model.existJointName(j_name):
                lock_ids.append(self.full_model.getJointId(j_name))

        q_ref = pin.neutral(self.full_model)
        self.model = pin.buildReducedModel(self.full_model, lock_ids, q_ref)
        self.data = self.model.createData()

        self.retained_joint_names = [
            name for name in self.model.names if name not in ("universe", "root_joint")
        ]

        pin.computeTotalMass(self.model, self.data)
        self.total_mass = float(self.data.mass[0])

        # Precompute joint index maps for state assembly
        self._joint_idx_q: Dict[str, int] = {}
        self._joint_idx_v: Dict[str, int] = {}
        for jname in self.retained_joint_names:
            jid = self.model.getJointId(jname)
            self._joint_idx_q[jname] = self.model.joints[jid].idx_q
            self._joint_idx_v[jname] = self.model.joints[jid].idx_v

    @property
    def nq(self) -> int:
        return self.model.nq

    @property
    def nv(self) -> int:
        return self.model.nv

    def update_kinematics_and_dynamics(self, q: np.ndarray, v: np.ndarray) -> None:
        pin.computeAllTerms(self.model, self.data, q, v)
        pin.updateFramePlacements(self.model, self.data)

    def get_mass_matrix(self) -> np.ndarray:
        return self.data.M

    def get_locked_inertia(self) -> np.ndarray:
        """Base link rotational inertia (after locking head/waist/tail mass in)."""
        return np.array(self.model.inertias[1].inertia, dtype=float, copy=True)

    def get_nonlinear_effects(self) -> np.ndarray:
        return self.data.nle

    def get_com(self) -> np.ndarray:
        return np.array(self.data.com[0], dtype=float, copy=True)

    def get_foot_jacobian(self, foot_frame_name: str, q: np.ndarray) -> np.ndarray:
        pin.computeJointJacobians(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        frame_id = self.model.getFrameId(foot_frame_name)
        J = pin.getFrameJacobian(
            self.model, self.data, frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
        )
        return J[:3, :]

    def apply_rear_tarsus_mimic(self, q: np.ndarray, v: Optional[np.ndarray] = None) -> None:
        """In-place: rear tarsus = multiplier * calf (URDF mimic)."""
        for tarsus, (calf, mult) in REAR_TARSUS_MIMIC.items():
            if tarsus not in self._joint_idx_q or calf not in self._joint_idx_q:
                continue
            iq_t, iq_c = self._joint_idx_q[tarsus], self._joint_idx_q[calf]
            q[iq_t] = mult * q[iq_c]
            if v is not None:
                iv_t, iv_c = self._joint_idx_v[tarsus], self._joint_idx_v[calf]
                v[iv_t] = mult * v[iv_c]

    def print_diagnostics(self) -> None:
        print("====== 降维模型诊断 ======")
        print(f"URDF: {self.urdf_path}")
        print(f"原模型 nv: {self.full_model.nv}")
        print(f"降维后 nq/nv: {self.model.nq}/{self.model.nv}")
        print(f"折叠后总质量: {self.total_mass:.3f} kg")
        print("保留关节:")
        for name in self.retained_joint_names:
            print(f" - {name}")
        print("WBC 主动关节 (S):")
        for name in LEG_ACTUATED_JOINT_NAMES:
            ok = self.model.existJointName(name)
            print(f" - {name} {'OK' if ok else 'MISSING'}")


if __name__ == "__main__":
    rm = QuadrupedReducedModel()
    rm.print_diagnostics()
