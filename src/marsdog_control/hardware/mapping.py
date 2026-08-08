"""Mapping layer: motor-frame angles -> per-driver command batches.

This is the software version of the diagram's Mapping block. It owns the
static joint grouping, limit/lead handling, and gain resolution needed before a
Board can talk to concrete motor drivers.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable, Mapping, Optional

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

# 仅实机接线极性（× 在 sign×gear 之外）。仿真 / JointDesc.sign 不动。
# send / read / CSV 日志必须同源，否则会出现 act≈−cmd、err≈−2×cmd 的假「发软」。
REAL_WIRE_POLARITY: dict[int, float] = {
    19: -1.0,  # waist_yaw
}


def wire_polarity(motor_id: int) -> float:
    return float(REAL_WIRE_POLARITY.get(int(motor_id), 1.0))


def urdf_to_motor_wired(joint, urdf_angle: float) -> float:
    """URDF → 实机电机角（含接线极性）。"""
    from marsdog_control.motion.kinematics import urdf_to_motor
    return float(urdf_to_motor(joint, urdf_angle)) * wire_polarity(joint.motor_id)


def motor_to_urdf_wired(joint, motor_val: float) -> float:
    """实机电机角 → URDF（与 urdf_to_motor_wired 互逆）。"""
    from marsdog_control.motion.kinematics import motor_to_urdf
    pol = wire_polarity(joint.motor_id)
    return float(motor_to_urdf(joint, float(motor_val) / pol))


def sync_dm_fixed_targets(dm_fixed: Optional[dict], motor_pose: Mapping[int, float]) -> list[int]:
    """把达妙 fixed_targets 同步为当前站立/目标电机角，避免 hold 打回开机探测角。"""
    if not dm_fixed or not motor_pose:
        return []
    updated: list[int] = []
    for j in DM_JOINTS:
        mid = j.motor_id
        if mid not in motor_pose:
            continue
        dm_fixed[mid] = float(motor_pose[mid])
        updated.append(mid)
    return updated


def dm_wire_gains(joint) -> tuple[float, float, float]:
    """JOINT_GAINS (关节空间) → 达妙线端 kp/kd（外置减速 /N²，与 batch 路径同源）。"""
    from marsdog_control.config.gains import BRAND_GAIN_SCALE, JOINT_GAINS
    g = JOINT_GAINS.get(joint.name, {})
    sc = BRAND_GAIN_SCALE.get("dm", {"kp": 1.0, "kd": 1.0})
    kp = float(g.get("kp", 30.0)) * float(sc.get("kp", 1.0))
    kd = float(g.get("kd", 0.5)) * float(sc.get("kd", 1.0))
    tau = float(g.get("trq_ff", 0.0))
    gr = float(getattr(joint, "gear_ratio", 1.0) or 1.0)
    gr2 = gr * gr if gr != 0.0 else 1.0
    return kp / gr2, kd / gr2, tau


def hold_dm_at(dm, joint_map: Iterable, targets: Mapping[int, float]) -> None:
    """在探测角上 MIT 保位（历史安全模式，见 ``bench_dm_latency``）。

    ``probe()`` 用 disable(0xFD) 读遥测；仅 ``enable`` 而不立刻 MIT 会让达妙
    处于「已使能但未锁」——起立 fade 前空窗表现为乱动。
    """
    if dm is None or not targets:
        return
    cmds = {}
    for j in joint_map:
        if getattr(j, "mtype", None) != "dm":
            continue
        mid = j.motor_id
        if mid not in targets:
            continue
        kp, kd, tau = dm_wire_gains(j)
        cmds[mid] = (kp, kd, float(targets[mid]), 0.0, tau)
    if not cmds:
        return
    if getattr(dm, "worker_running", False):
        dm.set_commands(cmds)
    else:
        for mid, (kp, kd, q, dq, tau) in cmds.items():
            dm.control_mit(mid, kp, kd, q, dq, tau)


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
    "REAL_WIRE_POLARITY",
    "wire_polarity",
    "urdf_to_motor_wired",
    "motor_to_urdf_wired",
    "sync_dm_fixed_targets",
    "MitBatch",
    "BoardCommandBatches",
    "build_board_command_batches",
    "dm_wire_gains",
    "hold_dm_at",
]
