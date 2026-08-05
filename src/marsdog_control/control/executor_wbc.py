"""WBC + ForcePlanner QP torque path for CommandExecutor.

Pipeline (typed ``WbcTickCtx``):
  pin height → contact → estimate → cmds → base/foot acc → force QP → solve/map.
"""

from __future__ import annotations

from marsdog_control.control.executor_wbc_apply import ExecutorWbcApplyMixin
from marsdog_control.control.executor_wbc_contact import ExecutorWbcContactMixin
from marsdog_control.control.executor_wbc_ctx import WbcTickCtx, _LEGS
from marsdog_control.control.executor_wbc_qp import ExecutorWbcQpMixin
from marsdog_control.core.types import RobotState


class ExecutorWbcMixin(
    ExecutorWbcContactMixin,
    ExecutorWbcQpMixin,
    ExecutorWbcApplyMixin,
):
    """Contact schedule → MPC forces → WBC QP → joint torque FF."""

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

        ctx = WbcTickCtx(
            state=state,
            targets=targets,
            active_gait=active_gait,
            t_rel=t_rel,
            use_est=use_est,
            vel_xyz=vel_xyz,
            current_base_z=current_base_z,
            q_pin=q_pin,
            v_pin=v_pin,
            foot_z=foot_z,
            foot_vz=foot_vz,
            foot_pos=foot_pos,
            foot_vel=foot_vel,
            leg_is_stance=dict(leg_is_stance),
        )
        self._wbc_update_contact(ctx)
        self._wbc_maybe_estimate(ctx)
        self._wbc_fill_velocity_cmds(ctx)
        self._wbc_compute_base_and_foot_acc(ctx)
        self._wbc_plan_forces(ctx)
        return self._wbc_solve_map_and_log(ctx)


__all__ = ["ExecutorWbcMixin"]
