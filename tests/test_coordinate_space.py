"""坐标空间(URDF ↔ 电机)契约回归测试。

守护本轮修复的一整类 bug:
- URDF→电机→URDF 往返恒等(含前腿 tarsus 外置 1:2 gear、左侧 sign=-1 镜像)。
- 非稳态路径(fade / recover / shutdown / lie-down)所用的 ``urdf_pose_to_motor``
  与主循环 ``RealRobotBackend.send`` 的位置映射**逐关节一致** → 起立/回站立/趴下
  不会再出现左腿反向或 tarsus 半幅/倍幅。
- ``urdf_pose_to_motor`` 与运动学唯一真源 ``kinematics.urdf_to_motor`` 同源。

不 import pytest, 保证在未安装 pytest 的环境也能被 ``__main__`` 直接跑。
"""

from marsdog_control.config.joints import JOINT_MAP, JOINT_BY_NAME
from marsdog_control.motion.kinematics import urdf_to_motor
from marsdog_control.backends.real import (
    urdf_pose_to_motor,
    motor_pose_to_urdf,
    RealRobotBackend,
)
from marsdog_control.core.types import ControlOutput, MotionTarget

_REAL = [j for j in JOINT_MAP if j.bus != "none"]


def _scale(j) -> float:
    return float(j.sign) * float(getattr(j, "gear_ratio", 1.0) or 1.0)


def _urdf_at_motor_midpoint(j) -> float:
    """限位是电机空间; 取限位中点对应的 URDF 角(必落在可达范围内, 不会被钳)。"""
    m_mid = 0.5 * (j.limit_lo + j.limit_hi)
    s = _scale(j)
    return m_mid / s if s else 0.0


def test_roundtrip_identity():
    for j in _REAL:
        u = _urdf_at_motor_midpoint(j)
        back = motor_pose_to_urdf(urdf_pose_to_motor({j.motor_id: u}))[j.motor_id]
        assert abs(back - u) < 1e-9, (j.name, u, back)


def test_matches_kinematics_with_clamp():
    for j in _REAL:
        for u in (-0.3, 0.0, 0.2, 0.5):
            expect = max(j.limit_lo, min(j.limit_hi, urdf_to_motor(j, u)))
            got = urdf_pose_to_motor({j.motor_id: u})[j.motor_id]
            assert abs(got - expect) < 1e-12, (j.name, u, got, expect)


def test_front_tarsus_gear_applied():
    for name in ("fl_tarsus", "fr_tarsus"):
        j = JOINT_BY_NAME[name]
        assert (j.gear_ratio or 1.0) == 2.0, name
        u = 0.2
        got = urdf_pose_to_motor({j.motor_id: u})[j.motor_id]
        assert abs(got - u * j.sign * 2.0) < 1e-12, (name, got)


def test_left_sign_joints_reverse():
    # 这些左侧关节 sign=-1: URDF 正角必须映射成负的电机角(方向镜像)。
    for name in ("fl_hip_pitch", "fl_calf", "fl_tarsus", "rl_thigh", "rl_calf"):
        j = JOINT_BY_NAME[name]
        assert j.sign == -1, name
        got = urdf_pose_to_motor({j.motor_id: 0.3})[j.motor_id]
        assert got < 0.0, (name, got)


def test_backend_send_matches_urdf_pose_to_motor():
    """主循环下发路径必须与 fade/shutdown/lie 用的映射同源, 否则又会分叉。"""
    captured = {}

    class _FakeSvc:
        def send_all(self, lz, evo, dm, incos, targets, **kw):
            captured["targets"] = dict(targets)

    be = RealRobotBackend(_FakeSvc(), None, None, None, None, None)
    q = {j.motor_id: 0.15 for j in _REAL}
    be.send(ControlOutput(target=MotionTarget(q=q, dq={})))
    assert captured["targets"] == urdf_pose_to_motor(q)


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in fns:
        fn()
        print(f"PASS {fn.__name__}")
    print(f"\nOK: {len(fns)} tests passed")
