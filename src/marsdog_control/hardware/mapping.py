"""Mapping layer: motor-frame angles -> per-driver command batches.

This is the software version of the diagram's Mapping block. It owns the
static joint grouping, limit/lead handling, and gain resolution needed before a
Board can talk to concrete motor drivers.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Mapping, Optional

from marsdog_control.config.joints import (
    DEFAULT_EVO_KD,
    DEFAULT_EVO_KP,
    DEFAULT_LZ_KD,
    DEFAULT_LZ_KP,
    JOINT_MAP,
)
from marsdog_control.control.executor import resolve_gains
from marsdog_control.core.types import MotorCommandFrame


REAL_JOINTS = [j for j in JOINT_MAP if j.bus != "none"]
CAN1_JOINTS = [j for j in JOINT_MAP if j.mtype == "lz" and j.bus == "lz_can_a"]
SERIAL_JOINTS = [j for j in JOINT_MAP if j.mtype == "lz" and j.bus == "lz_can_b"]
EVO_JOINTS = [j for j in JOINT_MAP if j.mtype == "evo"]
DM_JOINTS = [j for j in JOINT_MAP if j.mtype == "dm"]
INCOS_JOINTS = [j for j in JOINT_MAP if j.mtype == "incos"]


@dataclass
class MitBatch:
    ids: list[int] = field(default_factory=list)
    positions: list[float] = field(default_factory=list)
    velocities: list[float] = field(default_factory=list)
    kps: list[float] = field(default_factory=list)
    kds: list[float] = field(default_factory=list)
    torques: list[float] = field(default_factory=list)

    def append(self, mid: int, q: float, dq: float, kp: float, kd: float, tau: float) -> None:
        self.ids.append(mid)
        self.positions.append(q)
        self.velocities.append(dq)
        self.kps.append(kp)
        self.kds.append(kd)
        self.torques.append(tau)

    def __bool__(self) -> bool:
        return bool(self.ids)


@dataclass
class BoardCommandBatches:
    lz_can1: MitBatch = field(default_factory=MitBatch)
    lz_serial: MitBatch = field(default_factory=MitBatch)
    evo: MitBatch = field(default_factory=MitBatch)
    incos: MitBatch = field(default_factory=MitBatch)
    dm: dict[int, tuple[float, float, float, float, float]] = field(default_factory=dict)
    recorder: MotorCommandFrame = field(default_factory=MotorCommandFrame)


def build_board_command_batches(
    targets: Mapping[int, float],
    rt,
    *,
    kp_scale: float = 1.0,
    use_joint_gains: bool = True,
    kp_lz: float = DEFAULT_LZ_KP,
    kd_lz: float = DEFAULT_LZ_KD,
    kp_evo: float = DEFAULT_EVO_KP,
    kd_evo: float = DEFAULT_EVO_KD,
    kp_dm: Optional[float] = None,
    kd_dm: Optional[float] = None,
    velocities: Optional[Mapping[int, float]] = None,
    kp_phase: Optional[Mapping[int, float]] = None,
    trq_ff: Optional[Mapping[int, float]] = None,
    dm_reference_lead_active: bool = False,
) -> BoardCommandBatches:
    """Build per-driver command batches from motor-frame targets.

    ``rt`` is intentionally duck-typed so the legacy ``ActuationRuntime`` can
    remain the caller-side snapshot of mutable runtime knobs.
    """
    out = BoardCommandBatches()
    velocities = velocities or {}
    kp_phase = kp_phase or {}
    trq_ff = trq_ff or {}

    def _record(mid: int, q: float, dq: float, kp: float, kd: float, tau: float) -> None:
        out.recorder.target_q[mid] = q
        out.recorder.target_dq[mid] = dq
        out.recorder.kp[mid] = kp
        out.recorder.kd[mid] = kd
        out.recorder.torque_ff[mid] = tau

    for j in DM_JOINTS:
        mid = j.motor_id
        q = targets.get(mid) if rt.dm_tarsus_active else None
        if q is None:
            q = rt.dm_fixed_targets.get(mid)
        if q is None:
            continue
        dq = velocities.get(mid, 0.0)
        if rt.dm_tarsus_active and dm_reference_lead_active:
            lead_delta = dq * rt.dm_reference_lead_s.get(mid, 0.0)
            lead_delta = max(-rt.dm_reference_lead_max_rad,
                             min(rt.dm_reference_lead_max_rad, lead_delta))
            q += lead_delta
            q = max(j.limit_lo, min(j.limit_hi, q))
        kp = (kp_dm if kp_dm is not None
              else (rt.active_dm_kp_by_id.get(mid, rt.active_dm_kp)
                    if rt.dm_tarsus_active else rt.default_dm_kp))
        kd = (kd_dm if kd_dm is not None
              else (rt.active_dm_kd_by_id.get(mid, rt.active_dm_kd)
                    if rt.dm_tarsus_active else rt.default_dm_kd))
        dq_cmd = max(-rt.dm_dq_max_rps, min(rt.dm_dq_max_rps, dq))
        dq_send = dq_cmd if rt.dm_dq_feedforward else 0.0
        # JOINT_GAINS 按关节空间刚度标定；外置减速 N 时电机端 kp/kd = K_j / N^2
        gr = float(getattr(j, "gear_ratio", 1.0) or 1.0)
        gr2 = gr * gr if gr != 0.0 else 1.0
        kp_send = kp * kp_scale / gr2
        kd_send = kd * kp_scale / gr2
        out.dm[mid] = (kp_send, kd_send, q, dq_send, 0.0)
        _record(mid, q, dq_send, kp_send, kd_send, 0.0)

    def _append_group(joints, batch: MitBatch) -> None:
        for j in joints:
            mid = j.motor_id
            if mid not in targets:
                continue
            ps = kp_phase.get(mid, 1.0)
            to = trq_ff.get(mid) if mid in trq_ff else None
            kp, kd, tau = resolve_gains(
                j, kp_scale, use_joint_gains,
                kp_lz, kd_lz, kp_evo, kd_evo,
                rt.leg_kp_scale, rt.joint_gains, ps, to)
            q = targets[mid]
            dq = velocities.get(mid, 0.0)
            batch.append(mid, q, dq, kp, kd, tau)
            _record(mid, q, dq, kp, kd, tau)

    _append_group(CAN1_JOINTS, out.lz_can1)
    _append_group(SERIAL_JOINTS, out.lz_serial)
    _append_group(EVO_JOINTS, out.evo)
    _append_group(INCOS_JOINTS, out.incos)
    return out


__all__ = [
    "REAL_JOINTS",
    "CAN1_JOINTS",
    "SERIAL_JOINTS",
    "EVO_JOINTS",
    "DM_JOINTS",
    "INCOS_JOINTS",
    "MitBatch",
    "BoardCommandBatches",
    "build_board_command_batches",
]
