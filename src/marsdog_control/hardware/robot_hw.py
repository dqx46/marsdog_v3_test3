"""Hardware aggregation boundary for the Marsdog runtime."""

from __future__ import annotations

import os
import time
from dataclasses import dataclass, field
from typing import Optional

from marsdog_control.config.devices import DeviceConfig, get_device_config
from marsdog_control.config.joints import ALL_IDS, JOINT_MAP
from marsdog_control.config.schema import RuntimeConfig
from marsdog_control.core.types import ControlOutput, RobotState
from marsdog_control.hardware.actuation import send_all as _send_all
from marsdog_control.hardware.board import BoardOptions, RkMotorBoard
from marsdog_control.hardware.diagnostics import smooth_transition as _smooth_transition


# 实际接线的关节(排除 bus=="none" 的预留位)。随只读状态估计从 walk.py 下沉,
# 单一真源在 src(与 walk._REAL_JOINTS 同源, 都由同一份 JOINT_MAP 派生)。
REAL_JOINTS = [j for j in JOINT_MAP if j.bus != "none"]
_MISSING_INCOS = object()


def read_robot_positions(lz, evo, incos=_MISSING_INCOS, real_joints=REAL_JOINTS):
    """读取 lz/evo 电机当前位置 (不含达妙 tarsus — 那个固定角度另外单独读取,
    不参与这里的站立/步态位置混合, 避免被意外插值带动)。"""
    time.sleep(0.3)
    pos = {}
    for j in real_joints:
        mid = j.motor_id
        if j.mtype == "dm":
            continue
        if j.mtype == "lz":
            p = lz.get_position(mid)
        elif j.mtype == "incos":
            if incos is _MISSING_INCOS:
                p = evo.get_position(mid)  # legacy tests before Incos split
            else:
                p = incos.get_position(mid) if incos is not None else None
        else:
            p = evo.get_position(mid)
        pos[mid] = p if p is not None else 0.0
    return pos


def read_robot_state(lz, evo, dm, incos, imu=None, online=None, dm_fixed_targets=None,
                     real_joints=REAL_JOINTS):
    """State estimation: 采一帧传感器快照 -> RobotState(只读事实, 不含决策)。

    ``dm_fixed_targets`` 显式传入(达妙已使能的固定角映射), 使本函数与运行期模块
    全局解耦、可用假电机离线单测。get_position/is_enabled 读的都是驱动层后台缓存。
    """
    if dm_fixed_targets is None:
        # Backward-compatible call shape:
        # read_robot_state(lz, evo, dm, imu, online, dm_fixed_targets)
        dm_fixed_targets = online
        online = imu
        imu = incos
        incos = _MISSING_INCOS
    now = time.monotonic()
    st = RobotState(t=now, online=set(online))
    for j in real_joints:
        mid = j.motor_id
        idx = mid - 1
        vel = 0.0
        if j.mtype == "lz":
            p = lz.get_position(mid)
            en = lz.is_enabled[idx]
            try:
                vel = float(lz.get_velocity(mid))
            except Exception:
                vel = 0.0
        elif j.mtype == "incos":
            if incos is _MISSING_INCOS:
                p = evo.get_position(mid)
                en = (evo.status[idx] == 0x02)
                try:
                    vel = float(evo.get_velocity(mid))
                except Exception:
                    vel = 0.0
            else:
                p = incos.get_position(mid) if incos is not None else 0.0
                en = incos.is_enabled[idx] if incos is not None else False
                try:
                    vel = float(incos.get_velocity(mid)) if incos is not None else 0.0
                except Exception:
                    vel = 0.0
        elif j.mtype == "dm":
            p = dm.get_position(mid) if dm is not None else 0.0
            en = mid in dm_fixed_targets
            try:
                vel = float(dm.get_velocity(mid)) if dm is not None else 0.0
            except Exception:
                vel = 0.0
        else:
            p = evo.get_position(mid)
            en = (evo.status[idx] == 0x02)
            try:
                vel = float(evo.get_velocity(mid))
            except Exception:
                vel = 0.0
        st.joint_pos[mid] = p if p is not None else 0.0
        st.joint_vel[mid] = vel
        st.joint_enabled[mid] = en
    if imu is not None and imu.connected:
        st.imu_connected = True
        st.roll = imu.roll
        st.pitch = imu.pitch
        st.yaw = imu.yaw
        st.gyro_roll = imu.gyro_roll
        st.gyro_pitch = imu.gyro_pitch
        st.gyro_yaw = imu.gyro[2] if len(imu.gyro) > 2 else 0.0
        st.imu_age_s = ((now - imu.angle_timestamp)
                        if imu.angle_timestamp else float("inf"))
    return st


@dataclass
class HardwareOptions:
    include_imu: bool = True
    include_dm: bool = True
    imu_angle_tau_s: float = 0.0
    imu_gyro_tau_s: float = 0.0


@dataclass
class RobotHardware:
    """Owns all hardware drivers used by the runtime."""

    config: Optional[RuntimeConfig] = None
    devices: DeviceConfig = field(default_factory=get_device_config)
    options: HardwareOptions = field(default_factory=HardwareOptions)
    lz: Optional[object] = None
    evo: Optional[object] = None
    dm: Optional[object] = None
    incos: Optional[object] = None
    imu: Optional[object] = None
    board: Optional[RkMotorBoard] = None
    online: list[int] = field(default_factory=list)
    dm_fixed_targets: dict[int, float] = field(default_factory=dict)
    # Authoritative live knobs snapshot source (WalkRuntimeState). Owned here so
    # send/transition no longer reverse-import the legacy ``walk`` module.
    runtime_state: Optional[object] = None
    control_hz: float = 200.0

    def __post_init__(self) -> None:
        if self.config is None:
            return
        self.devices = self.config.hardware.devices
        self.options = HardwareOptions(
            include_imu=self.config.features.imu_enabled,
            include_dm=self.config.hardware.include_dm,
            imu_angle_tau_s=self.config.imu.angle_tau_s,
            imu_gyro_tau_s=self.config.imu.gyro_tau_s,
        )

    def start(self) -> None:
        """Initialize IMU and motor buses using the legacy-proven sequence."""
        self._start_imu()
        self._start_motors()
        self.online = self._detect_online()
        if not self.online:
            self.shutdown("no online motors")
            raise RuntimeError("无在线电机")

    def _start_imu(self) -> None:
        if not self.options.include_imu or not os.path.exists(self.devices.imu):
            self.imu = None
            return
        from marsdog_control.hardware.sensors.imu_wt901 import ImuWT901

        imu = ImuWT901(
            self.devices.imu,
            self.devices.imu_baud,
            angle_tau_s=self.options.imu_angle_tau_s,
            gyro_tau_s=self.options.imu_gyro_tau_s,
        )
        self.imu = imu if imu.begin() else None

    def _start_motors(self) -> None:
        self.board = RkMotorBoard(
            devices=self.devices,
            options=BoardOptions(include_dm=self.options.include_dm),
            dm_fixed_targets=self.dm_fixed_targets,
        )
        self.board.start()
        self.lz = self.board.lz
        self.evo = self.board.evo
        self.dm = self.board.dm
        self.incos = self.board.incos
        self.dm_fixed_targets = self.board.dm_fixed_targets

    def _detect_online(self) -> list[int]:
        if self.board is not None:
            return sorted(self.board.online_ids())
        online: list[int] = []
        if self.lz is None or self.evo is None:
            return online
        for joint in JOINT_MAP:
            mid = joint.motor_id
            if joint.bus == "none":
                continue
            if joint.mtype == "lz":
                connected = self.lz.is_connected[mid - 1]
            elif joint.mtype == "incos":
                connected = self.incos is not None and self.incos.is_connected[mid - 1]
            elif joint.mtype == "dm":
                connected = mid in self.dm_fixed_targets
            else:
                connected = self.evo.is_connected[mid - 1]
            if connected:
                online.append(mid)
        return online

    def read_positions(self) -> dict[int, float]:
        if self.board is not None:
            return self.board.get_angles()
        if self.lz is None or self.evo is None:
            return {}
        positions = read_robot_positions(self.lz, self.evo, self.incos)
        positions.update(self.dm_fixed_targets)
        return positions

    def read_state(self) -> RobotState:
        if self.board is not None:
            now_state = RobotState(t=time.monotonic(), online=set(self.online))
            feedback = self.board.get_feedback()
            for mid, sample in feedback.samples.items():
                now_state.joint_pos[mid] = sample.position
                now_state.joint_vel[mid] = sample.velocity
                now_state.joint_enabled[mid] = sample.enabled
            if self.imu is not None and self.imu.connected:
                now_state.imu_connected = True
                now_state.roll = self.imu.roll
                now_state.pitch = self.imu.pitch
                now_state.yaw = self.imu.yaw
                now_state.gyro_roll = self.imu.gyro_roll
                now_state.gyro_pitch = self.imu.gyro_pitch
                now_state.gyro_yaw = self.imu.gyro[2] if len(self.imu.gyro) > 2 else 0.0
                now_state.imu_age_s = (
                    (now_state.t - self.imu.angle_timestamp)
                    if self.imu.angle_timestamp else float("inf"))
            return now_state
        if self.lz is None or self.evo is None:
            return RobotState()
        return read_robot_state(
            self.lz, self.evo, self.dm, self.incos, self.imu, self.online,
            self.dm_fixed_targets)

    def _actuation(self):
        """Snapshot the live actuation knobs (no dependency on legacy ``walk``)."""
        rs = self.runtime_state
        if rs is None:
            # Fallback for dry assembly without wired runtime state.
            from marsdog_control.runtime.walk_state import WalkRuntimeState

            rs = WalkRuntimeState()
            rs.dm.fixed_targets = self.dm_fixed_targets
        return rs.to_actuation_runtime()

    def send(self, output: ControlOutput) -> None:
        act = self._actuation()
        if self.board is not None:
            self.board.send_angles(
                output.target.q, act,
                kp_scale=output.kp_scale,
                use_joint_gains=True,
                velocities=output.target.dq,
                kp_phase=output.kp_phase,
                trq_ff=output.trq_ff,
                dm_reference_lead_active=output.dm_active,
            )
            return
        if self.lz is None or self.evo is None:
            return
        _send_all(
            self.lz, self.evo, self.dm, self.incos, output.target.q, act,
            kp_scale=output.kp_scale,
            use_joint_gains=True,
            velocities=output.target.dq,
            kp_phase=output.kp_phase,
            trq_ff=output.trq_ff,
            dm_reference_lead_active=output.dm_active,
        )

    def transition(self, target: dict[int, float], duration_s: float,
                   label: str = "transition") -> bool:
        if self.lz is None or self.evo is None:
            return False
        current = self.read_positions()

        def _send(lz, evo, dm, incos, cur, kp_s):
            act = self._actuation()
            if self.board is not None:
                self.board.send_angles(
                    cur, act, use_joint_gains=True, kp_scale=kp_s)
            else:
                _send_all(lz, evo, dm, incos, cur, act,
                          use_joint_gains=True, kp_scale=kp_s)

        return _smooth_transition(
            self.lz, self.evo, self.dm, self.incos, current, target, duration_s,
            label=label, send_fn=_send,
            control_hz=self.control_hz,
            stop_check=lambda: False,
        )

    def shutdown(self, reason: str = "") -> None:
        if self.board is not None:
            self.board.close()
        else:
            # Incos shares LZ CAN-A; release it before LZ closes the shared adapter.
            if self.incos is not None:
                self.incos.end()
            if self.lz is not None:
                self.lz.end()
            if self.evo is not None:
                self.evo.end()
            if self.dm is not None:
                self.dm.end()
        if self.imu is not None:
            try:
                self.imu.close()
            except AttributeError:
                pass

    @property
    def online_count(self) -> int:
        return len(self.online)

    @property
    def expected_motor_count(self) -> int:
        return len(ALL_IDS)
