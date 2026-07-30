"""Motion planning boundary — real implementation.

This layer owns the "compute joint targets" step of the fixed control pipeline:
turn the active gait / stand controller (carried by the FSM) plus the IMU
foot-height deltas into a ``MotionTarget`` (motor-frame), including stand/gait
blend and the fine anti stick-slip rate limit. IMU only enters as a foot-height
delta (``imu_dz``); this layer decides how to use it — IMU never edits legs
directly.

Moved verbatim out of the monolithic ``mocap_to_real/walk.py`` during the
decoupling refactor. Dependencies now come from the sunk ``src`` modules
(kinematics / gait_controller / joint table / core contracts). ``walk.py``
imports these back so the legacy loop keeps calling the exact same code.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Optional

from marsdog_control.config.schema import RuntimeConfig
from marsdog_control.config.joints import JOINT_BY_ID, JOINT_BY_NAME as JBN
from marsdog_control.core.types import MotionTarget, RobotMode, RobotState
from marsdog_control.motion.kinematics import (
    ik_front_3link_foot_orient,
    ik_rear_leg_2d,
)
from marsdog_control.motion.lie_down import build_lie_down_target
from marsdog_control.motion.gait_controller import (
    StablePace,
    _FRONT_HIP_OFFSET,
    _REAR_HIP_OFFSET,
)


# roll/hip 目标速率限制的电机 id (fl/fr_thigh_roll, rl/rr_hip)。抗 stick-slip 的
# 细粒度限速属 motion 层步态平滑, 与 SafetySupervisor 的安全闸是两回事。
_RATELIMIT_IDS = {2, 6, 9, 12}
_HIP_ABDUCTION_TEST_IDS = (2, 6, 9, 12)
_LEG_PITCH_TEST_JOINTS = (
    "fl_hip_pitch", "fr_hip_pitch",  # 前腿大腿向前
    "rl_thigh", "rr_thigh",          # 后腿大腿向后
)
# 每个测试轴的 URDF 角增量方向(× amplitude)。语义: 前腿向前 / 后腿向后。
# 前腿 hip_pitch: 负 URDF 角 = 向前(实机已验证)。
# 后腿 thigh: 按模型约定(kinematics.compute_joint_check_pose)负角=向前, 故"向后"取正号。
#   2026-07-11 实机: 之前四轴统一取负, 后腿因此也向前(错), 现把后腿翻成 +1。
_LEG_PITCH_TEST_DELTA_SIGN = {
    "fl_hip_pitch": -1.0,
    "fr_hip_pitch": -1.0,
    "rl_thigh":     +1.0,
    "rr_thigh":     +1.0,
}
# 后腿大腿视觉行程比前腿小(几何关系), 单独放大后腿幅度便于肉眼判断方向。
# URDF 上限 rl/rr_thigh 为 ±0.71, 2.0×0.30=0.60 仍在限位内, 不会被静默钳。
_LEG_PITCH_REAR_SCALE = 2.0
_CALF_PITCH_TEST_JOINTS = (
    "fl_calf", "fr_calf",  # 前腿小腿向前
    "rl_calf", "rr_calf",  # 后腿小腿向前
)
# 逐轴语义: 四个小腿都向前。按主控 FK, 固定大腿只动 calf 时 URDF 负角让足端 +x。
# 最终仍只经 urdf_to_motor()/joint.sign 映射, 不在测试模式复制电机符号。
_CALF_PITCH_TEST_DELTA_SIGN = {
    "fl_calf": -1.0,
    "fr_calf": -1.0,
    "rl_calf": -1.0,
    "rr_calf": -1.0,
}


def build_motion_target(fsm, state, imu_dz, imu_state, online, cur_pos,
                        smooth_tgt) -> MotionTarget:
    """Motion planner: 由 active_gait/stand 生成关节目标, 含 blend + 细限速。

    这里承接旧主循环的"计算目标"块; IMU 的 imu_dz 作为落脚点增量传入, 由本层决定
    怎么用(不再让 IMU 直接改腿)。
    """
    gait = fsm.active_gait
    if gait is not None:
        if hasattr(fsm, "clock"):
            t_rel = fsm.clock.time() - fsm.t_gait
        else:
            t_rel = time.time() - fsm.t_gait
        # Jump liftoff uses measured vz/z — note before get_targets advances FSM.
        if getattr(gait, "family", None) == "jump":
            vz = float(getattr(state, "vel_xyz", (0.0, 0.0, 0.0))[2])
            if hasattr(gait, "note_base_vz"):
                gait.note_base_vz(vz)
            # Prefer IMU-integrated height if planner passed it via imu_state.
            if imu_state and "base_z" in imu_state and hasattr(gait, "note_base_z"):
                gait.note_base_z(float(imu_state["base_z"]))
        targets = gait.get_targets(t_rel, imu_dz=imu_dz, imu_state=imu_state)
    else:
        targets = fsm.stand.get_targets(0)
        if imu_dz:
            _apply_stand_imu_dz(fsm.stand, targets, imu_dz)

    for mid in online:
        if mid not in targets:
            targets[mid] = cur_pos.get(mid, 0.0)

    # blend(站立/步态切换位置混合)
    if fsm.blend_active:
        if hasattr(fsm, "clock"):
            elapsed = fsm.clock.time() - fsm.blend_start
        else:
            elapsed = time.time() - fsm.blend_start
        if elapsed >= fsm.blend_dur:
            fsm.blend_active = False
        else:
            s = elapsed / fsm.blend_dur
            s = s * s * (3.0 - 2.0 * s)
            for mid in fsm.blend_from:
                if mid in targets:
                    targets[mid] = fsm.blend_from[mid] * (1.0 - s) + targets[mid] * s

    # 细粒度抗 stick-slip 限速(步态平滑, 非安全闸; 保留在 motion 层)
    if gait is not None and getattr(gait, "family", None) == "jump":
        rl_max = 1.0
    elif gait is not None and type(gait).__name__ == "StablePace":
        rl_max = 0.0175
    else:
        rl_max = 0.0087
    for mid in _RATELIMIT_IDS:
        if mid in targets:
            if mid in smooth_tgt:
                delta = targets[mid] - smooth_tgt[mid]
                if delta > rl_max:
                    delta = rl_max
                elif delta < -rl_max:
                    delta = -rl_max
                targets[mid] = smooth_tgt[mid] + delta
            smooth_tgt[mid] = targets[mid]

    src = fsm.mode
    return MotionTarget(q=targets, dq={}, source_mode=src)


def build_hip_abduction_test_target(stand, held_positions, online, *,
                                    elapsed_s: float, duration_s: float) -> MotionTarget:
    """主控内置的四髋外展方向测试目标。

    四个测试关节的终点**直接**取当前主程序实际使用的
    ``StandController.get_targets(0)`` 输出；因此它完整复用了：

        hip_abduction -> URDF 轴/运动学 -> urdf_to_motor -> joint.sign

    绝不复制或猜测任何电机正负号。其余在线关节冻结为启动时读取的位置，避免方向
    测试把狗带入完整站姿。起始/过渡目标在同一 MotionTarget -> SafetySupervisor ->
    send_all 主控制管线中处理。
    """
    missing = [mid for mid in _HIP_ABDUCTION_TEST_IDS if mid not in online]
    if missing:
        names = ", ".join(JOINT_BY_ID[mid].name for mid in missing)
        raise ValueError(f"髋外展方向测试要求四轴均在线，缺少: {names}")

    stand_targets = stand.get_targets(0)  # 必须是主控同一 StandController 的真实输出
    missing = [mid for mid in _HIP_ABDUCTION_TEST_IDS if mid not in stand_targets]
    if missing:
        raise ValueError(f"StandController 未产出髋外展目标: {missing}")

    # smoothstep: 仅给四个髋从开机位置柔和过渡到主控的站姿目标。
    if duration_s <= 1e-6:
        alpha = 1.0
    else:
        u = max(0.0, min(1.0, elapsed_s / duration_s))
        alpha = u * u * (3.0 - 2.0 * u)

    targets = {mid: held_positions.get(mid, 0.0) for mid in online}
    for mid in _HIP_ABDUCTION_TEST_IDS:
        start = held_positions[mid]
        target = stand_targets[mid]
        # alpha 已完成时保留 StandController 输出的原始数值；除了避免浮点舍入，
        # 也保证测试终点就是主控站姿的同一份目标，而非“等价的重新计算值”。
        targets[mid] = target if alpha >= 1.0 else start + (target - start) * alpha

    return MotionTarget(q=targets, dq={}, source_mode=RobotMode.ZEROING)


def build_leg_pitch_direction_test_target(held_positions, online, *,
                                          amplitude_rad: float,
                                          elapsed_s: float = float("inf"),
                                          duration_s: float = 0.0) -> MotionTarget:
    """主控内置的大腿前后摆方向测试目标。

    目标语义是“前腿大腿向前、后腿大腿向后”。四轴的 URDF 增量都按当前模型约定
    取负值；最终电机目标必须通过主控同一 ``urdf_to_motor(joint, delta)`` 生成。
    因而测试和步态/IK 的 joint.sign 映射完全同源，非测试轴冻结在启动读数。
    """
    test_ids = tuple(JBN[name].motor_id for name in _LEG_PITCH_TEST_JOINTS)
    missing = [mid for mid in test_ids if mid not in online]
    if missing:
        names = ", ".join(JOINT_BY_ID[mid].name for mid in missing)
        raise ValueError(f"大腿方向测试要求四轴均在线，缺少: {names}")

    targets = {mid: held_positions.get(mid, 0.0) for mid in online}
    if duration_s <= 1e-6:
        alpha = 1.0
    else:
        u = max(0.0, min(1.0, elapsed_s / duration_s))
        alpha = u * u * (3.0 - 2.0 * u)
    for name in _LEG_PITCH_TEST_JOINTS:
        joint = JBN[name]
        # 前腿向前(负角)、后腿向后(正角)，逐轴显式指定 URDF 增量方向；
        # 最终电机目标仍走主控同一 urdf_to_motor(joint, delta)，与步态/IK 同源。
        urdf_delta = _LEG_PITCH_TEST_DELTA_SIGN[name] * amplitude_rad
        if name in ("rl_thigh", "rr_thigh"):
            urdf_delta *= _LEG_PITCH_REAR_SCALE
        start = held_positions[joint.motor_id]
        targets[joint.motor_id] = start + urdf_delta * alpha

    return MotionTarget(q=targets, dq={}, source_mode=RobotMode.ZEROING)


def build_calf_pitch_direction_test_target(held_positions, online, *,
                                           amplitude_rad: float,
                                           elapsed_s: float = float("inf"),
                                           duration_s: float = 0.0) -> MotionTarget:
    """主控内置的小腿前摆方向测试目标。

    目标语义是“四个小腿都向前”。前/后腿 calf 的 URDF 增量都取负值；最终电机
    目标必须通过主控同一 ``urdf_to_motor(joint, delta)`` 生成。因而测试和步态/IK
    的 joint.sign 映射完全同源，非测试轴冻结在启动读数。
    """
    test_ids = tuple(JBN[name].motor_id for name in _CALF_PITCH_TEST_JOINTS)
    missing = [mid for mid in test_ids if mid not in online]
    if missing:
        names = ", ".join(JOINT_BY_ID[mid].name for mid in missing)
        raise ValueError(f"小腿方向测试要求四轴均在线，缺少: {names}")

    targets = {mid: held_positions.get(mid, 0.0) for mid in online}
    if duration_s <= 1e-6:
        alpha = 1.0
    else:
        u = max(0.0, min(1.0, elapsed_s / duration_s))
        alpha = u * u * (3.0 - 2.0 * u)
    for name in _CALF_PITCH_TEST_JOINTS:
        joint = JBN[name]
        urdf_delta = _CALF_PITCH_TEST_DELTA_SIGN[name] * amplitude_rad
        start = held_positions[joint.motor_id]
        targets[joint.motor_id] = start + urdf_delta * alpha

    return MotionTarget(q=targets, dq={}, source_mode=RobotMode.ZEROING)


def _apply_stand_imu_dz(stand, targets, imu_dz):
    """站立IMU调平；前腿保持三连杆足段朝向，不退化成旧两连杆。"""
    _h = stand.body_height
    _xf = stand.x_offset_front
    _xr = stand.x_offset_rear
    _zf = -(_h - _FRONT_HIP_OFFSET)
    _zr = -(_h - _REAR_HIP_OFFSET)
    front_foot_pitch_rad = math.radians(stand.front_stand_foot_pitch_deg)
    for leg in ("fl", "fr"):
        dz = imu_dz.get(leg, 0.0)
        if abs(dz) > 0.0001:
            hp_u, ca_u, ta_u = ik_front_3link_foot_orient(
                _xf, _zf + dz, front_foot_pitch_rad)
            for suffix, value in (
                ("hip_pitch", hp_u), ("calf", ca_u), ("tarsus", ta_u),
            ):
                joint = JBN[f"{leg}_{suffix}"]
                targets[joint.motor_id] = value

    for leg, hp_name, ca_name in (
        ("rl", "rl_thigh", "rl_calf"),
        ("rr", "rr_thigh", "rr_calf"),
    ):
        dz = imu_dz.get(leg, 0.0)
        if abs(dz) > 0.0001:
            hp_u, ca_u = ik_rear_leg_2d(_xr, _zr + dz)
            for joint_name, value in ((hp_name, hp_u), (ca_name, ca_u)):
                joint = JBN[joint_name]
                targets[joint.motor_id] = value


@dataclass
class MotionPlanner:
    """Builds ``MotionTarget`` objects from runtime state and gait context."""

    config: Optional[RuntimeConfig] = None
    smooth_targets: dict[int, float] = field(default_factory=dict)

    def reset_smoothing(self) -> None:
        self.smooth_targets.clear()

    def plan(self, fsm, state: RobotState, imu_dz: Optional[dict],
             imu_state: Optional[dict], online, current_positions) -> MotionTarget:
        return build_motion_target(
            fsm, state, imu_dz, imu_state, online, current_positions,
            self.smooth_targets,
        )

    def lie_down_target(self, online, pose_path: Optional[str] = None) -> dict[int, float]:
        if pose_path is None:
            raise ValueError("lie_down_target requires pose_path (JSON under mocap_to_real/)")
        return build_lie_down_target(online, pose_path=pose_path)

    def hip_abduction_test(self, stand, held_positions, online, *,
                           elapsed_s: float, duration_s: float) -> MotionTarget:
        return build_hip_abduction_test_target(
            stand, held_positions, online,
            elapsed_s=elapsed_s, duration_s=duration_s)

    def leg_pitch_direction_test(self, held_positions, online, *,
                                 amplitude_rad: float, elapsed_s: float,
                                 duration_s: float) -> MotionTarget:
        return build_leg_pitch_direction_test_target(
            held_positions, online,
            amplitude_rad=amplitude_rad, elapsed_s=elapsed_s, duration_s=duration_s)

    def calf_pitch_direction_test(self, held_positions, online, *,
                                  amplitude_rad: float, elapsed_s: float,
                                  duration_s: float) -> MotionTarget:
        return build_calf_pitch_direction_test_target(
            held_positions, online,
            amplitude_rad=amplitude_rad, elapsed_s=elapsed_s, duration_s=duration_s)


__all__ = [
    "MotionPlanner",
    "build_motion_target",
    "build_hip_abduction_test_target",
    "build_leg_pitch_direction_test_target",
    "build_calf_pitch_direction_test_target",
    "_apply_stand_imu_dz",
    "_RATELIMIT_IDS",
    "_HIP_ABDUCTION_TEST_IDS",
    "_LEG_PITCH_TEST_JOINTS",
    "_LEG_PITCH_TEST_DELTA_SIGN",
    "_LEG_PITCH_REAR_SCALE",
    "_CALF_PITCH_TEST_JOINTS",
    "_CALF_PITCH_TEST_DELTA_SIGN",
]
