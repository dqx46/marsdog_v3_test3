from typing import Dict, Optional, Set

from marsdog_control.backends import RobotBackend
from marsdog_control.config.joints import JOINT_BY_ID, JOINT_MAP
from marsdog_control.core.types import ControlOutput, RobotState
from marsdog_control.motion.kinematics import motor_to_urdf, urdf_to_motor
from marsdog_control.runtime.walk_services import WalkServices

_REAL_JOINTS = [j for j in JOINT_MAP if j.bus != "none"]

# 仅实机接线极性（乘在 sign×gear 之外）。仿真 / JointDesc.sign / URDF 约定不动。
# 2026-08-07: waist_yaw(ID19) 电机正转与 URDF +yaw 相反。
_REAL_WIRE_POLARITY: Dict[int, float] = {
    19: -1.0,  # waist_yaw
}


def _wire_polarity(motor_id: int) -> float:
    return float(_REAL_WIRE_POLARITY.get(int(motor_id), 1.0))


def _joint_scale(j) -> float:
    """电机角 = URDF角 × scale；scale = sign × gear_ratio × 实机接线极性。"""
    gr = float(getattr(j, "gear_ratio", 1.0) or 1.0)
    return float(j.sign) * gr * _wire_polarity(j.motor_id)


def urdf_pose_to_motor(urdf_q: Dict[int, float]) -> Dict[int, float]:
    """URDF 空间关节位姿 -> 实机电机空间位姿。

    基映射与 :func:`kinematics.urdf_to_motor` 同源（``sign × gear_ratio``），
    再乘 ``_REAL_WIRE_POLARITY``（仅实机）。fade / recover / shutdown /
    RealRobotBackend.send 共用此函数。
    """
    out: Dict[int, float] = {}
    for mid, urdf in urdf_q.items():
        j = JOINT_BY_ID.get(mid)
        if j is None:
            continue
        m_q = urdf_to_motor(j, urdf) * _wire_polarity(mid)
        m_q = max(j.limit_lo, min(j.limit_hi, m_q))
        out[mid] = m_q
    return out


def motor_pose_to_urdf(motor_q: Dict[int, float]) -> Dict[int, float]:
    """实机电机空间 -> URDF 空间（与 send / ``urdf_pose_to_motor`` 互逆）。"""
    out: Dict[int, float] = {}
    for mid, pos in motor_q.items():
        j = JOINT_BY_ID.get(mid)
        if j is not None and _joint_scale(j) != 0:
            # motor = urdf * sign * gear * pol  →  urdf = motor_to_urdf(motor / pol)
            out[mid] = motor_to_urdf(j, float(pos) / _wire_polarity(mid))
        else:
            out[mid] = 0.0
    return out


class RealRobotBackend(RobotBackend):
    """
    实机后端实现。
    持有 WalkServices 和底层句柄，负责:
    1. 从底层物理传感器读取，映射回纯 URDF 空间 (RobotState)
    2. 从纯 URDF 空间的 ControlOutput，映射为实机电机方向 (j.sign × gear_ratio)
       和物理限位，并下发

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

        # 转换为 URDF 空间 (唯一真源映射, 与 send 互逆)
        urdf_pos = motor_pose_to_urdf(raw_state.joint_pos)
        urdf_vel = {}

        # Prefer driver velocity; fall back to finite difference on URDF pos
        dt = 0.005
        if self._prev_t is not None and raw_state.t > self._prev_t:
            dt = max(1e-3, raw_state.t - self._prev_t)

        for j in _REAL_JOINTS:
            mid = j.motor_id
            if mid not in urdf_pos:
                continue
            motor_v = self._motor_velocity(j)
            scale = _joint_scale(j)
            if scale != 0 and abs(motor_v) > 1e-9:
                urdf_vel[mid] = motor_v / scale
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
        # 位置: 走唯一真源映射, 保证与 fade/recover/shutdown 完全同源
        motor_targets = urdf_pose_to_motor(output.target.q)
        motor_velocities = {}
        motor_trq_ff = {}

        for mid in output.target.q:
            j = JOINT_BY_ID.get(mid)
            if j is None:
                continue

            # 速度/力矩与位置同一 scale (sign×gear_ratio)；
            # 力矩 ÷ gear 保持做功 P=τ·ω 在减速两侧守恒。
            scale = _joint_scale(j)
            if mid in output.target.dq:
                motor_velocities[mid] = output.target.dq[mid] * scale

            if output.trq_ff and mid in output.trq_ff:
                gr = float(getattr(j, "gear_ratio", 1.0) or 1.0)
                # τ_motor = τ_joint / gear_ratio, 再乘 sign×接线极性对齐方向
                pol = _wire_polarity(mid)
                motor_trq_ff[mid] = (
                    output.trq_ff[mid] * float(j.sign) * pol / gr
                    if gr != 0 else 0.0
                )

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
