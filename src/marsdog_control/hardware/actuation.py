"""Motor command dispatch — the write half of the Mapping layer.

Two responsibilities, kept separate:

- ``marsdog_control.hardware.mapping.build_board_command_batches`` is the *pure*
  conversion (motor-frame angles → per-driver command batches, the diagram's
  ``cvAngles2Encoder`` / ``send_ids``). No I/O, fully unit-testable.
- ``dispatch_batches`` is the *single* impure seam that pushes those batches to
  the concrete drivers (``send_id`` / ``send_ids``). It is shared by both
  ``send_all`` (raw-handle callers) and ``hardware.board`` (``send_angles``), so
  the "batches → driver write" path has exactly one implementation.

``send_all`` = build (mapping) + dispatch. Runtime-mutable DM/tarsus knobs and
gain context come in via ``ActuationRuntime`` (snapshotted by the caller each
cycle) instead of module globals, so ``src`` no longer depends on ``walk``.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass

from marsdog_control.config.joints import (
    DEFAULT_LZ_KP, DEFAULT_LZ_KD,
    DEFAULT_EVO_KP, DEFAULT_EVO_KD,
)
from marsdog_control.hardware.mapping import build_board_command_batches


@dataclass
class ActuationRuntime:
    """运行期可变旋钮的一帧快照(调用方按当前状态填入, 保持 send_all 纯粹)。"""
    dm_tarsus_active: bool
    dm_fixed_targets: dict
    dm_reference_lead_s: dict
    dm_reference_lead_max_rad: float
    active_dm_kp_by_id: dict
    active_dm_kp: float
    active_dm_kd_by_id: dict
    active_dm_kd: float
    default_dm_kp: float
    default_dm_kd: float
    dm_dq_max_rps: float
    dm_dq_feedforward: bool
    leg_kp_scale: float
    joint_gains: dict


def dispatch_batches(lz, evo, dm, incos, batches):
    """Push pre-built per-driver batches to the motor drivers.

    Single dispatch seam shared by ``send_all`` and ``hardware.board``.

    IncOS 与灵足 can1 已分属独立 USB-CAN，可与 CAN-B / DM / EVO 一样并行下发。
    （旧共享 CAN-A 时的「先 IncOS 再 LZ」串行调度已不再需要。）

    Driver handles may be ``None`` (partial hardware / offline motors) — the
    corresponding batch is then simply skipped.
    """
    def _do_can1():
        if lz is not None and batches.lz_can1:
            b = batches.lz_can1
            lz.mit_controls_can1(b.ids, b.positions, b.velocities,
                                 b.kps, b.kds, b.torques)

    def _do_serial():
        if lz is not None and batches.lz_serial:
            b = batches.lz_serial
            lz.mit_controls_serial(b.ids, b.positions, b.velocities,
                                   b.kps, b.kds, b.torques)

    def _do_incos():
        if incos is not None and batches.incos:
            b = batches.incos
            incos.mit_controls(b.ids, b.positions, b.velocities,
                               b.kps, b.kds, b.torques)

    t_serial = threading.Thread(target=_do_serial, daemon=True)
    t_can1 = threading.Thread(target=_do_can1, daemon=True)
    t_incos = threading.Thread(target=_do_incos, daemon=True)
    t_serial.start()
    t_can1.start()
    t_incos.start()

    if dm is not None and batches.dm:
        if dm.worker_running:
            dm.set_commands(batches.dm)
        else:
            for mid, command in batches.dm.items():
                dm.control_mit(mid, *command)
    if evo is not None and batches.evo:
        b = batches.evo
        evo.ptm_controls(b.ids, b.positions, b.velocities, b.kps, b.kds, b.torques)

    t_serial.join()
    t_can1.join()
    t_incos.join()


def send_all(lz, evo, dm, incos, targets, rt, kp_scale=1.0, use_joint_gains=True,
             kp_lz=DEFAULT_LZ_KP, kd_lz=DEFAULT_LZ_KD,
             kp_evo=DEFAULT_EVO_KP, kd_evo=DEFAULT_EVO_KD,
             kp_dm=None, kd_dm=None,
             velocities=None, kp_phase=None, trq_ff=None,
             dm_reference_lead_active=False):
    """Build (mapping) + dispatch for raw-handle callers. Board uses the same seam."""
    batches = build_board_command_batches(
        targets, rt,
        kp_scale=kp_scale, use_joint_gains=use_joint_gains,
        kp_lz=kp_lz, kd_lz=kd_lz, kp_evo=kp_evo, kd_evo=kd_evo,
        kp_dm=kp_dm, kd_dm=kd_dm,
        velocities=velocities, kp_phase=kp_phase, trq_ff=trq_ff,
        dm_reference_lead_active=dm_reference_lead_active)
    dispatch_batches(lz, evo, dm, incos, batches)


__all__ = ["ActuationRuntime", "dispatch_batches", "send_all"]
