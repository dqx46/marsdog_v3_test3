from typing import Dict, Optional, Set

from marsdog_control.backends import RobotBackend
from marsdog_control.config.joints import JOINT_BY_ID, JOINT_MAP
from marsdog_control.core.types import ControlOutput, RobotState
from marsdog_control.runtime.walk_services import WalkServices

_REAL_JOINTS = [j for j in JOINT_MAP if j.bus != "none"]


class RealRobotBackend(RobotBackend):
    """
    实机后端实现。
    持有 WalkServices 和底层句柄，负责:
    1. 从底层物理传感器读取，映射回纯 URDF 空间 (RobotState)
    2. 从纯 URDF 空间的 ControlOutput，映射为实机电机方向 (j.sign) 和物理限位，并下发

    契约: ``joint_pos`` / ``joint_vel`` 均为 URDF 空间；``vel_xyz`` 由动力学估计器填充
    （实机无 MoCap 时保持 0）。
    """

    def __init__(self, services: WalkServices, lz, evo, dm, incos, imu):
        self.svc = services
        self.lz = lz
        self.evo = evo
        self.dm = dm
        self.incos = incos
        self.imu = imu
        self._prev_pos: Dict[int, float] = {}
        self._prev_t: Optional[float] = None

    def _motor_velocity(self, j) -> float:
        """Read motor-frame velocity [rad/s] from the owning driver."""
        mid = j.motor_id
        try:
            if j.mtype == "lz" and self.lz is not None:
                return float(self.lz.get_velocity(mid))
            if j.mtype == "evo" and self.evo is not None:
                return float(self.evo.get_velocity(mid))
            if j.mtype == "dm" and self.dm is not None:
                return float(self.dm.get_velocity(mid))
            if j.mtype == "incos" and self.incos is not None:
                return float(self.incos.get_velocity(mid))
        except Exception:
            return 0.0
        return 0.0

    def read_state(self, online_ids: Set[int]) -> RobotState:
        # 调用底层的 read_state，此时拿到的是电机空间的值
        raw_state = self.svc.read_state(
            self.lz, self.evo, self.dm, self.incos, self.imu, online_ids
        )

        # 转换为 URDF 空间
        urdf_pos = {}
        urdf_vel = {}
        for mid, pos in raw_state.joint_pos.items():
            j = JOINT_BY_ID.get(mid)
            if j is not None and j.sign != 0:
                urdf_pos[mid] = pos / j.sign
            else:
                urdf_pos[mid] = 0.0

        # Prefer driver velocity; fall back to finite difference on URDF pos
        dt = 0.005
        if self._prev_t is not None and raw_state.t > self._prev_t:
            dt = max(1e-3, raw_state.t - self._prev_t)

        for j in _REAL_JOINTS:
            mid = j.motor_id
            if mid not in urdf_pos:
                continue
            motor_v = self._motor_velocity(j)
            if j.sign != 0 and abs(motor_v) > 1e-9:
                urdf_vel[mid] = motor_v / j.sign
            elif mid in self._prev_pos:
                urdf_vel[mid] = (urdf_pos[mid] - self._prev_pos[mid]) / dt
            else:
                urdf_vel[mid] = 0.0

        self._prev_pos = dict(urdf_pos)
        self._prev_t = raw_state.t

        return RobotState(
            t=raw_state.t,
            joint_pos=urdf_pos,
            joint_vel=urdf_vel,
            joint_enabled=raw_state.joint_enabled,
            online=raw_state.online,
            imu_connected=raw_state.imu_connected,
            roll=raw_state.roll,
            pitch=raw_state.pitch,
            yaw=raw_state.yaw,
            gyro_roll=raw_state.gyro_roll,
            gyro_pitch=raw_state.gyro_pitch,
            gyro_yaw=raw_state.gyro_yaw,
            vel_xyz=(0.0, 0.0, 0.0),  # filled by BaseStateEstimator in Executor
            imu_age_s=raw_state.imu_age_s,
        )

    def send(self, output: ControlOutput) -> None:
        # 转换为电机空间
        motor_targets = {}
        motor_velocities = {}
        motor_trq_ff = {}

        for mid, urdf_q in output.target.q.items():
            j = JOINT_BY_ID.get(mid)
            if j is None:
                continue

            # 方向映射
            m_q = urdf_q * j.sign

            # 电机硬限位钳制 (因为 DM 前瞻等可能会略微超出，或者做最后兜底)
            m_q = max(j.limit_lo, min(j.limit_hi, m_q))

            motor_targets[mid] = m_q

            # 速度和力矩的方向映射 (注意阻抗和力矩前馈的正负号)
            if mid in output.target.dq:
                motor_velocities[mid] = output.target.dq[mid] * j.sign

            if output.trq_ff and mid in output.trq_ff:
                # 力矩与位置同一方向变换 (做功 P = tau * dq 保持不变)
                motor_trq_ff[mid] = output.trq_ff[mid] * j.sign

        # 下发到服务层
        self.svc.send_all(
            self.lz, self.evo, self.dm, self.incos,
            targets=motor_targets,
            use_joint_gains=True,
            kp_scale=output.kp_scale,
            velocities=motor_velocities,
            kp_phase=output.kp_phase,
            trq_ff=motor_trq_ff,
            dm_reference_lead_active=output.gait_active
        )

    def shutdown(self, reason: str = "") -> None:
        self.svc.shutdown(reason)
