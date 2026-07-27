"""Walk app hardware/log service layer.

Collects the thin I/O wrappers that used to live as module-level functions in
``apps/walk.py`` (send / read / diagnostics / smooth-transition / logging) into
one explicit object that owns the live handles (board, runtime state, joint set,
resource dir). ``apps/walk`` keeps same-named module shims delegating here so the
parity seam is now anchored directly on these class methods (patched by
``tests/parity/loop_harness.py``); the old ``walk.send_all`` / ``walk.read_state``
module delegators have been removed.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Optional

from marsdog_control.config.joints import (
    DEFAULT_EVO_KD,
    DEFAULT_EVO_KP,
    DEFAULT_LZ_KD,
    DEFAULT_LZ_KP,
)
from marsdog_control.hardware.actuation import send_all as _send_all_impl
from marsdog_control.hardware.diagnostics import (
    check_motors as _check_motors_impl,
    check_motors_board as _check_motors_board,
    find_lz_recoverable_faults as _find_lz_recoverable_faults,
    recover_lz_stand_faults as _recover_lz_stand_faults,
    smooth_transition as _smooth_transition_impl,
)
from marsdog_control.hardware.robot_hw import (
    read_robot_positions as _read_robot_positions,
    read_robot_state as _read_robot_state,
)
from marsdog_control.io.logging import (
    LogRuntime,
    WriteLogRuntime,
    setup_log as _setup_log_impl,
    write_log as _write_log_impl,
)
from marsdog_control.runtime.walk_state import WalkRuntimeState


@dataclass
class WalkServices:
    """Owns live hardware handles + runtime knobs for the walk app's I/O path."""

    runtime_state: WalkRuntimeState
    real_joints: list
    resource_dir: str
    control_hz: float = 200.0
    board: Optional[object] = None
    stop: bool = False
    clock: object = time

    @property
    def dm_fixed_targets(self) -> dict:
        return self.runtime_state.dm.fixed_targets

    # ── actuation ────────────────────────────────────────────────────────────
    def actuation_runtime(self):
        """Snapshot actuation knobs from WalkRuntimeState (no ad-hoc globals)."""
        return self.runtime_state.to_actuation_runtime()

    def send_all(self, lz, evo, dm, incos, targets, kp_scale=1.0,
                 use_joint_gains=True,
                 kp_lz=DEFAULT_LZ_KP, kd_lz=DEFAULT_LZ_KD,
                 kp_evo=DEFAULT_EVO_KP, kd_evo=DEFAULT_EVO_KD,
                 kp_dm=None, kd_dm=None,
                 velocities=None, kp_phase=None, trq_ff=None,
                 dm_reference_lead_active=False):
        board = self.runtime_state.board if self.runtime_state.board is not None else self.board
        act = self.runtime_state.to_actuation_runtime()
        if board is not None:
            board.send_angles(
                targets, act,
                kp_scale=kp_scale, use_joint_gains=use_joint_gains,
                kp_lz=kp_lz, kd_lz=kd_lz, kp_evo=kp_evo, kd_evo=kd_evo,
                kp_dm=kp_dm, kd_dm=kd_dm,
                velocities=velocities, kp_phase=kp_phase, trq_ff=trq_ff,
                dm_reference_lead_active=dm_reference_lead_active)
            return
        _send_all_impl(
            lz, evo, dm, incos, targets, act,
            kp_scale=kp_scale, use_joint_gains=use_joint_gains,
            kp_lz=kp_lz, kd_lz=kd_lz, kp_evo=kp_evo, kd_evo=kd_evo,
            kp_dm=kp_dm, kd_dm=kd_dm,
            velocities=velocities, kp_phase=kp_phase, trq_ff=trq_ff,
            dm_reference_lead_active=dm_reference_lead_active)

    # ── state read ───────────────────────────────────────────────────────────
    def read_positions(self, lz, evo, incos):
        if self.board is not None:
            return self.board.get_angles(include_dm=False)
        return _read_robot_positions(lz, evo, incos, self.real_joints)

    def read_state(self, lz, evo, dm, incos, imu, online):
        return _read_robot_state(
            lz, evo, dm, incos, imu, online,
            self.dm_fixed_targets, self.real_joints)

    # ── diagnostics ──────────────────────────────────────────────────────────
    def check_motors(self, lz, evo, dm, incos, label=""):
        if self.board is not None:
            return _check_motors_board(self.board, self.real_joints, label=label)
        return _check_motors_impl(
            lz, evo, dm, incos, self.real_joints, self.dm_fixed_targets,
            label=label)

    def find_lz_recoverable_faults(self, lz, joints, targets, *,
                                   max_error_rad=math.radians(15.0),
                                   low_torque_nm=0.10):
        return _find_lz_recoverable_faults(
            lz, joints, targets,
            max_error_rad=max_error_rad, low_torque_nm=low_torque_nm)

    def recover_lz_stand_faults(self, lz, evo, dm, incos, online, stand_pos, *,
                                attempts=2,
                                max_error_rad=math.radians(15.0),
                                low_torque_nm=0.10):
        board = self.board
        if board is not None:
            lz, evo, dm, incos = board.lz, board.evo, board.dm, board.incos
        return _recover_lz_stand_faults(
            lz, evo, dm, incos, online, stand_pos,
            real_joints=self.real_joints,
            dm_fixed_targets=self.dm_fixed_targets,
            read_positions_fn=self.read_positions,
            smooth_transition_fn=self.smooth_transition,
            attempts=attempts,
            max_error_rad=max_error_rad,
            low_torque_nm=low_torque_nm)

    def smooth_transition(self, lz, evo, dm, incos, from_pos, to_pos, duration,
                          label="fade"):
        def _send(lz_arg, evo_arg, dm_arg, incos_arg, cur, kp_s):
            self.send_all(lz_arg, evo_arg, dm_arg, incos_arg, cur,
                          use_joint_gains=True, kp_scale=kp_s)

        return _smooth_transition_impl(
            lz, evo, dm, incos, from_pos, to_pos, duration, label=label,
            send_fn=_send, control_hz=self.control_hz,
            stop_check=lambda: self.stop, clock=self.clock)

    def shutdown_motors(self, lz, evo, dm=None, incos=None):
        """Close motor drivers in bus-owner order.

        Incos shares LZ CAN-A, so it must release before ``lz.end()`` closes the
        shared serial adapter.
        """
        if self.board is not None:
            self.board.close()
            self.board = None
            self.runtime_state.board = None
            return
        if incos is not None:
            incos.end()
        lz.end()
        evo.end()
        if dm is not None:
            dm.end()

    # ── logging ──────────────────────────────────────────────────────────────
    def setup_log(self, enabled, args=None):
        rt = self.runtime_state
        return _setup_log_impl(
            enabled, args, base_dir=self.resource_dir,
            runtime=LogRuntime(
                active_dm_kp_by_id=rt.dm.kp_by_id,
                active_dm_kd_by_id=rt.dm.kd_by_id,
                dm_reference_lead_s=rt.dm.reference_lead_s,
                dm_reference_lead_max_rad=rt.dm.reference_lead_max_rad,
                dm_dq_feedforward=rt.dm.dq_feedforward,
                dm_dq_max_rps=rt.dm.dq_max_rps,
                leg_kp_scale=rt.leg_kp_scale,
                var_impedance=rt.impedance.enabled,
            ),
        )

    def _write_log_runtime(self) -> WriteLogRuntime:
        rt = self.runtime_state
        return WriteLogRuntime(
            real_joints=self.real_joints,
            dm_fixed_targets=rt.dm.fixed_targets,
            joint_gains=rt.joint_gains,
            leg_kp_scale=rt.leg_kp_scale,
            default_lz_kp=DEFAULT_LZ_KP,
            default_lz_kd=DEFAULT_LZ_KD,
            default_evo_kp=DEFAULT_EVO_KP,
            default_evo_kd=DEFAULT_EVO_KD,
        )

    def write_log(self, writer, t_s, mode, lz, evo, dm, incos, targets, dt_ms,
                  trot, throttle, imu=None, imu_dz=None,
                  imu_ctrl=None, ramp_frac=0.0, kp_phase=None, trq_ff=None,
                  run_t_s=0.0, fsm_mode="", gait_active=False,
                  controller_name="", input_vx=0.0, input_turn=0.0,
                  input_has_stick=False, input_request_mode="",
                  control_period_ms=0.0):
        return _write_log_impl(
            writer, t_s, mode, lz, evo, dm, incos, targets, dt_ms,
            trot, throttle, self._write_log_runtime(),
            imu=imu, imu_dz=imu_dz, imu_ctrl=imu_ctrl, ramp_frac=ramp_frac,
            kp_phase=kp_phase, trq_ff=trq_ff, run_t_s=run_t_s, fsm_mode=fsm_mode,
            gait_active=gait_active, controller_name=controller_name,
            input_vx=input_vx, input_turn=input_turn,
            input_has_stick=input_has_stick, input_request_mode=input_request_mode,
            control_period_ms=control_period_ms,
            feedback=(self.board.get_feedback() if self.board is not None else None))


__all__ = ["WalkServices"]
