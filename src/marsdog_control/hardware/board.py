"""Board abstraction for the current RK3588 motor stack.

The future STM32 firmware can implement the same ``MotorBoard`` contract. Until
then, ``RkMotorBoard`` is the software Board layer that owns the concrete
LZ/EVO/DM/Incos drivers and presents one uniform API to runtime code.
"""

from __future__ import annotations

import sys
import time
from dataclasses import dataclass, field
from typing import Iterable, Mapping, Optional, Protocol

from marsdog_control.compat import ensure_legacy_path
from marsdog_control.config.devices import DeviceConfig, get_device_config
from marsdog_control.config.joints import (
    DM_CAN_IDS,
    DM_MASTER_ID_BY_SLAVE,
    INCOS_CAN_IDS,
    JOINT_BY_ID,
    JOINT_MAP,
)
from marsdog_control.core.types import MotorCommandFrame, MotorFeedbackFrame, MotorSample
from marsdog_control.hardware.actuation import dispatch_batches
from marsdog_control.hardware.mapping import REAL_JOINTS, build_board_command_batches


@dataclass
class BoardOptions:
    include_dm: bool = True


class MotorBoard(Protocol):
    def start(self) -> None: ...
    def online_ids(self) -> set[int]: ...
    def get_angles(self, ids: Optional[Iterable[int]] = None) -> dict[int, float]: ...
    def get_feedback(self, ids: Optional[Iterable[int]] = None) -> MotorFeedbackFrame: ...
    def send_angles(self, targets: Mapping[int, float], rt, **kwargs) -> MotorCommandFrame: ...
    def soft_disable(self, hold_target: Mapping[int, float], rt, *,
                     duration_s: float, control_hz: float, stop_check=None) -> bool: ...
    def disable(self, ids: Optional[Iterable[int]] = None) -> None: ...
    def close(self) -> None: ...


@dataclass
class RkMotorBoard:
    """RK3588-hosted Board implementation backed by local Python motor drivers."""

    devices: DeviceConfig = field(default_factory=get_device_config)
    options: BoardOptions = field(default_factory=BoardOptions)
    lz: Optional[object] = None
    evo: Optional[object] = None
    dm: Optional[object] = None
    incos: Optional[object] = None
    dm_fixed_targets: dict[int, float] = field(default_factory=dict)
    online: set[int] = field(default_factory=set)
    last_command: MotorCommandFrame = field(default_factory=MotorCommandFrame)

    @classmethod
    def from_existing(cls, lz, evo, dm=None, incos=None, *,
                      dm_fixed_targets: Optional[dict[int, float]] = None) -> "RkMotorBoard":
        board = cls(lz=lz, evo=evo, dm=dm, incos=incos)
        if dm_fixed_targets is not None:
            board.dm_fixed_targets = dm_fixed_targets
        board.online = board.online_ids()
        return board

    def start(self) -> None:
        ensure_legacy_path()
        from marsdog_control.hardware.motors.damiao import MotorDamiao
        from marsdog_control.hardware.motors.evo import MotorEvo
        from marsdog_control.hardware.motors.lingzu import MotorLz

        self.lz = MotorLz()
        self.evo = MotorEvo()

        self.lz.init_serial(self.devices.lz_can_b, self.devices.baud)
        self.lz.init_can1_serial(self.devices.lz_can_a, self.devices.baud)
        self.evo.init_serial(self.devices.evo_can, self.devices.baud)

        try:
            from marsdog_control.hardware.motors.incos import MotorIncos
            incos = MotorIncos()
            if incos.begin(self.devices.incos_can, INCOS_CAN_IDS, self.devices.baud):
                self.incos = incos
            elif getattr(incos, "_running", False):
                # 适配器已打开、recv 线程已起，只是电机未应答(接线中/未上电)
                print(f"  [WARNING] {self.devices.incos_can} 已打开, "
                      f"但因克斯 ID{list(INCOS_CAN_IDS)} 无应答(未上电?)")
                self.incos = incos
            else:
                print(f"  [WARNING] {self.devices.incos_can} 打开失败, "
                      "因克斯小腿本次不可用")
                self.incos = None
        except Exception:
            self.incos = None

        if self.options.include_dm:
            dm = MotorDamiao()
            if dm.begin(self.devices.dm_can, self.devices.baud):
                time.sleep(1.5)
                for mid in DM_CAN_IDS:
                    dm.add_motor(mid, master_id=DM_MASTER_ID_BY_SLAVE.get(mid))
                for mid in DM_CAN_IDS:
                    online, pos, _err, _link_ok = dm.probe(mid)
                    if online:
                        self.dm_fixed_targets[mid] = pos
                        dm.enable(mid)
                        time.sleep(0.02)
                dm.start_worker()
                self.dm = dm

        for joint in JOINT_MAP:
            if joint.mtype == "lz" and self.lz.is_connected[joint.motor_id - 1]:
                self.lz.enable(joint.motor_id)
                time.sleep(0.002)

        time.sleep(0.05)
        for _attempt in range(5):
            pending = []
            for joint in JOINT_MAP:
                if joint.mtype == "evo" and self.evo.is_connected[joint.motor_id - 1]:
                    idx = joint.motor_id - 1
                    if self.evo.status[idx] != 0x02:
                        pending.append(joint)
                        self.evo.enter_motor_state(joint.motor_id)
                        time.sleep(0.005)
            if not pending:
                break
            time.sleep(0.05)
        time.sleep(0.4)
        # EVO 保活短窗可能把刚发现的电机误标离线；按 init 名单恢复一次。
        if self.evo is not None:
            for mid in getattr(self.evo, "_active_ids", []):
                idx = mid - 1
                if 0 <= idx < len(self.evo.is_connected):
                    self.evo.is_connected[idx] = True
                    self.evo._loss_count[idx] = 0
        self.online = self.online_ids()

    def online_ids(self) -> set[int]:
        out: set[int] = set()
        if self.lz is None or self.evo is None:
            return out
        for joint in REAL_JOINTS:
            mid = joint.motor_id
            if joint.mtype == "lz":
                connected = self.lz.is_connected[mid - 1]
            elif joint.mtype == "incos":
                connected = self.incos is not None and self.incos.is_connected[mid - 1]
            elif joint.mtype == "dm":
                connected = mid in self.dm_fixed_targets
            else:
                connected = self.evo.is_connected[mid - 1]
            if connected:
                out.add(mid)
        return out

    def missing_ids(self) -> list[int]:
        online = self.online_ids()
        return [j.motor_id for j in REAL_JOINTS if j.motor_id not in online]

    def get_angles(self, ids: Optional[Iterable[int]] = None, *,
                   include_dm: bool = True) -> dict[int, float]:
        wanted = set(ids) if ids is not None else {j.motor_id for j in REAL_JOINTS}
        out: dict[int, float] = {}
        for mid in sorted(wanted):
            joint = JOINT_BY_ID.get(mid)
            if joint is None or joint.bus == "none":
                continue
            if joint.mtype == "dm":
                if not include_dm:
                    continue
                p = self.dm.get_position(mid) if self.dm is not None else None
                out[mid] = p if p is not None else self.dm_fixed_targets.get(mid, 0.0)
            elif joint.mtype == "lz":
                p = self.lz.get_position(mid) if self.lz is not None else None
                out[mid] = p if p is not None else 0.0
            elif joint.mtype == "incos":
                p = self.incos.get_position(mid) if self.incos is not None else None
                out[mid] = p if p is not None else 0.0
            else:
                p = self.evo.get_position(mid) if self.evo is not None else None
                out[mid] = p if p is not None else 0.0
        return out

    def get_feedback(self, ids: Optional[Iterable[int]] = None) -> MotorFeedbackFrame:
        now = time.monotonic()
        wanted = set(ids) if ids is not None else {j.motor_id for j in REAL_JOINTS}
        frame = MotorFeedbackFrame(t=now)
        for mid in sorted(wanted):
            joint = JOINT_BY_ID.get(mid)
            if joint is None or joint.bus == "none":
                continue
            idx = mid - 1
            timing = {}
            command_q = self.last_command.target_q.get(mid)
            command_dq = self.last_command.target_dq.get(mid)
            command_kp = self.last_command.kp.get(mid)
            command_kd = self.last_command.kd.get(mid)
            command_tau = self.last_command.torque_ff.get(mid)
            if joint.mtype == "lz":
                pos = self.lz.get_position(mid) if self.lz is not None else 0.0
                torque = self._safe_call(self.lz, "get_torque", mid, default=0.0)
                enabled = bool(self.lz.is_enabled[idx]) if self.lz is not None else False
                fault = int(self.lz.fault[idx]) if self.lz is not None else -1
            elif joint.mtype == "incos":
                pos = self.incos.get_position(mid) if self.incos is not None else 0.0
                torque = self._safe_call(self.incos, "get_torque", mid, default=0.0)
                enabled = bool(self.incos.is_enabled[idx]) if self.incos is not None else False
                fault = int(self.incos.fault[idx]) if self.incos is not None else -1
            elif joint.mtype == "dm":
                pos = self.dm.get_position(mid) if self.dm is not None else self.dm_fixed_targets.get(mid, 0.0)
                torque = self._safe_call(self.dm, "get_torque", mid, default=0.0)
                enabled = mid in self.dm_fixed_targets
                fault = int(self._safe_call(self.dm, "get_error", mid, default=-1))
                if self.dm is not None:
                    timing = dict(self._safe_call(self.dm, "get_timing", mid, default={}) or {})
                    command_q = timing.get("command_q", command_q)
                    command_dq = timing.get("command_dq", command_dq)
                    command_kp = timing.get("command_kp", command_kp)
                    command_kd = timing.get("command_kd", command_kd)
                    command_tau = timing.get("command_tau", command_tau)
            else:
                pos = self.evo.get_position(mid) if self.evo is not None else 0.0
                torque = self._safe_call(self.evo, "get_torque", mid, default=0.0)
                enabled = bool(self.evo.status[idx] == 0x02) if self.evo is not None else False
                fault = int(self.evo.fault[idx]) if self.evo is not None else -1
            frame.samples[mid] = MotorSample(
                motor_id=mid,
                name=joint.name,
                position=pos if pos is not None else 0.0,
                torque=torque if torque is not None else 0.0,
                enabled=enabled,
                fault=fault,
                command_q=command_q,
                command_dq=command_dq,
                command_kp=command_kp,
                command_kd=command_kd,
                command_tau=command_tau,
                timing=timing,
            )
        return frame

    def send_angles(self, targets: Mapping[int, float], rt, **kwargs) -> MotorCommandFrame:
        batches = build_board_command_batches(targets, rt, **kwargs)
        self._dispatch_batches(batches)
        self.last_command = batches.recorder
        return batches.recorder

    def soft_disable(self, hold_target: Mapping[int, float], rt, *,
                     duration_s: float, control_hz: float, stop_check=None,
                     clock=None) -> bool:
        """Ramp gains to zero over ``duration_s`` wall-clock seconds.

        ``kp`` starts at 10 and falls linearly to 0. Alpha follows wall time so
        a slow bus cannot stretch the fade past the requested duration.
        """
        from marsdog_control.config.joints import DEFAULT_DM_KD, DEFAULT_DM_KP

        clock = clock or time
        duration_s = max(1e-3, float(duration_s))
        dt = 1.0 / max(1.0, float(control_hz))
        kp_start = 10.0
        kd_start = 0.5
        t0 = clock.monotonic()
        while True:
            if stop_check is not None and stop_check():
                return False
            elapsed = clock.monotonic() - t0
            alpha = min(1.0, elapsed / duration_s)
            kp = kp_start * (1.0 - alpha)
            kd = kd_start * (1.0 - alpha)
            dm_kp_base = getattr(rt, "active_dm_kp", DEFAULT_DM_KP) if rt.dm_tarsus_active else DEFAULT_DM_KP
            dm_kd_base = getattr(rt, "active_dm_kd", DEFAULT_DM_KD) if rt.dm_tarsus_active else DEFAULT_DM_KD
            self.send_angles(
                hold_target, rt,
                use_joint_gains=False,
                kp_lz=kp, kd_lz=kd,
                kp_evo=kp, kd_evo=kd,
                kp_dm=dm_kp_base * (1.0 - alpha),
                kd_dm=dm_kd_base * (1.0 - alpha),
            )
            sys.stdout.write(f"\r  [disable] {int(alpha*100):3d}%  kp={kp:.2f}   ")
            sys.stdout.flush()
            if alpha >= 1.0:
                break
            # Pace toward control_hz, but never schedule past the deadline.
            next_t = min(t0 + duration_s, clock.monotonic() + dt)
            sleep_t = next_t - clock.monotonic()
            if sleep_t > 0:
                clock.sleep(sleep_t)
        sys.stdout.write("\n")
        return True

    def disable(self, ids: Optional[Iterable[int]] = None) -> None:
        wanted = set(ids) if ids is not None else {j.motor_id for j in REAL_JOINTS}
        if self.dm is not None:
            self._safe_call(self.dm, "stop_worker", default=None)
        for mid in sorted(wanted):
            joint = JOINT_BY_ID.get(mid)
            if joint is None or joint.bus == "none":
                continue
            if joint.mtype == "lz" and self.lz is not None:
                self.lz.disable(mid)
            elif joint.mtype == "incos" and self.incos is not None:
                self.incos.disable(mid)
            elif joint.mtype == "dm" and self.dm is not None:
                self.dm.disable(mid)
            elif joint.mtype == "evo" and self.evo is not None:
                self.evo.enter_rest_state(mid)
            time.sleep(0.002)
        if self.evo is not None:
            for _ in range(3):
                time.sleep(0.05)
                for joint in JOINT_MAP:
                    if joint.mtype == "evo":
                        self.evo.enter_rest_state(joint.motor_id)
                        time.sleep(0.002)

    def close(self) -> None:
        if self.incos is not None:
            self.incos.end()
        if self.lz is not None:
            self.lz.end()
        if self.evo is not None:
            self.evo.end()
        if self.dm is not None:
            self.dm.end()

    def _dispatch_batches(self, batches) -> None:
        # Single dispatch seam shared with hardware.actuation.send_all.
        dispatch_batches(self.lz, self.evo, self.dm, self.incos, batches)

    @staticmethod
    def _safe_call(obj, name: str, *args, default=None):
        if obj is None or not hasattr(obj, name):
            return default
        try:
            return getattr(obj, name)(*args)
        except Exception:
            return default


__all__ = ["BoardOptions", "MotorBoard", "RkMotorBoard"]
