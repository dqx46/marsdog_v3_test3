"""站姿/步态起点契约检查。

这些检查不参与控制计算，只用于在入口处和调试脚本中尽早发现：
- foot orientation tracking 开启但站姿仍是 2-link/tarsus=0；
- StandController 与 gait(t=0) 不是同一个姿态；
- 调试工具显示的姿态和真实仿真入口用的姿态不一致。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable

from marsdog_control.config.joints import JOINT_BY_NAME


FRONT_POSE_JOINTS = (
    "fl_hip_pitch",
    "fl_calf",
    "fl_tarsus",
    "fr_hip_pitch",
    "fr_calf",
    "fr_tarsus",
)

LEG_POSE_JOINTS = FRONT_POSE_JOINTS + (
    "rl_hip",
    "rl_thigh",
    "rl_calf",
    "rr_hip",
    "rr_thigh",
    "rr_calf",
)


@dataclass(frozen=True)
class PoseDiff:
    joint_name: str
    stand_deg: float
    gait_deg: float
    diff_deg: float


def _targets_diff(
    stand_targets: dict[int, float],
    gait_targets: dict[int, float],
    joint_names: Iterable[str],
) -> list[PoseDiff]:
    diffs: list[PoseDiff] = []
    for name in joint_names:
        joint = JOINT_BY_NAME[name]
        if joint.motor_id not in stand_targets or joint.motor_id not in gait_targets:
            continue
        stand_deg = math.degrees(stand_targets[joint.motor_id])
        gait_deg = math.degrees(gait_targets[joint.motor_id])
        diffs.append(
            PoseDiff(
                joint_name=name,
                stand_deg=stand_deg,
                gait_deg=gait_deg,
                diff_deg=gait_deg - stand_deg,
            )
        )
    return diffs


def format_pose_diffs(diffs: Iterable[PoseDiff]) -> str:
    lines = []
    for d in diffs:
        lines.append(
            f"{d.joint_name:14s}: stand={d.stand_deg:+8.3f}deg "
            f"gait0={d.gait_deg:+8.3f}deg diff={d.diff_deg:+8.4f}deg"
        )
    return "\n".join(lines)


def assert_foot_tracking_requires_tarsus(
    *,
    front_foot_track_deg: float | None,
    stand_controller,
    context: str = "pose contract",
) -> None:
    if front_foot_track_deg is None:
        return
    if not getattr(stand_controller, "use_tarsus", False):
        raise RuntimeError(
            f"{context}: front_foot_track_deg is enabled but "
            "StandController(use_tarsus=True) is not. This would make stand "
            "and gait(t=0) use different front-leg kinematics."
        )


def assert_stand_matches_gait_start(
    stand_controller,
    gait_controller,
    *,
    tolerance_deg: float = 0.10,
    joint_names: Iterable[str] = LEG_POSE_JOINTS,
    context: str = "pose contract",
) -> None:
    stand_targets = stand_controller.get_targets(0.0)
    gait_targets = gait_controller.get_targets(0.0)
    diffs = _targets_diff(stand_targets, gait_targets, joint_names)
    bad = [d for d in diffs if abs(d.diff_deg) > tolerance_deg]
    if bad:
        raise RuntimeError(
            f"{context}: StandController and {gait_controller.__class__.__name__}"
            f"(t=0) differ by more than {tolerance_deg:.3f}deg:\n"
            f"{format_pose_diffs(bad)}"
        )


def print_targets(
    targets: dict[int, float],
    *,
    joint_names: Iterable[str] = LEG_POSE_JOINTS,
) -> None:
    for name in joint_names:
        joint = JOINT_BY_NAME[name]
        if joint.motor_id not in targets:
            continue
        print(f"  {name:14s}: motor={math.degrees(targets[joint.motor_id]):+8.2f}deg")


def print_qpos_from_robot(robot, *, joint_names: Iterable[str] = LEG_POSE_JOINTS) -> None:
    for name in joint_names:
        qadr = robot._joint_qpos.get(name)
        if qadr is None:
            print(f"  {name:14s}: qpos=<missing>")
            continue
        print(f"  {name:14s}: qpos ={math.degrees(robot.data.qpos[qadr]):+8.2f}deg")
