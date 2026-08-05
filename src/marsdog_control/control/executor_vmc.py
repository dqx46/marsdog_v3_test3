"""Decoupled VMC torque path for CommandExecutor."""

from __future__ import annotations

import math
from typing import Dict, Optional

import numpy as np

from marsdog_control.config.joints import JOINT_BY_NAME as JBN
from marsdog_control.core.types import RobotState
from marsdog_control.motion import kinematics as K


_LEGS = ("fl", "fr", "rl", "rr")


class ExecutorVmcMixin:
    """Per-leg VMC overlay when WBC is off."""

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

