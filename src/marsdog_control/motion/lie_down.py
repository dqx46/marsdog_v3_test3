"""Lie-down pose load/save and target construction (pure data + JSON I/O)."""

from __future__ import annotations

import csv
import datetime
import json
import math
import os
from typing import Iterable, Mapping, Optional


# 头部 + 脖子；脖子后续走独立任务通道
NON_BODY_LIE_MOTOR_IDS = {15, 16, 17, 18}

# fallback: 旧日志里的姿势。实际使用应优先通过 --capture-lie-pose 从实机当前姿势
# 保存 lie_down_pose.json；这个 fallback 仅避免文件缺失时报错。
LIE_DOWN_TARGETS_RAD = {
    1: -0.687625,
    2: -0.077685,
    3: -1.371986,
    4: -0.716981,
    5: +0.835315,
    6: +0.082292,
    7: +1.389247,
    8: +0.712792,
    9: +0.090984,
    10: +0.192004,
    11: -0.529952,
    12: -0.089832,
    13: -0.243404,
    14: +0.643503,
    18: -0.002094,
    19: -0.000192,
    20: +0.033371,
    21: -0.012462,
}


def default_lie_down_pose_path(base_dir: str) -> str:
    return os.path.join(base_dir, "lie_down_pose.json")


def default_sit_pose_path(base_dir: str) -> str:
    return os.path.join(base_dir, "sit_pose.json")


def load_lie_down_pose_from_log(path: str) -> dict:
    """从 walk CSV 最后一帧读取趴下姿势(电机帧 rad), 排除头/脖子 15/16/17/18。"""
    pose = {}
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            mid = int(row["motor_id"])
            if mid in NON_BODY_LIE_MOTOR_IDS:
                continue
            pose[mid] = math.radians(float(row["actual_deg"]))
    return pose


def save_lie_down_pose(path: str, pose: Mapping, *, pose_name: str = "lie_down") -> None:
    """保存姿势 JSON（趴下/坐下共用）。键为电机 ID 字符串, 值为电机帧 rad。"""
    clean = {
        str(int(mid)): float(q)
        for mid, q in pose.items()
        if int(mid) not in NON_BODY_LIE_MOTOR_IDS
    }
    with open(path, "w") as f:
        json.dump({
            "source": "captured-live-motor-position",
            "pose_name": str(pose_name),
            "saved_at": datetime.datetime.now().isoformat(timespec="seconds"),
            "units": "motor_frame_rad",
            "excluded_motor_ids": sorted(NON_BODY_LIE_MOTOR_IDS),
            "targets": clean,
        }, f, indent=2, sort_keys=True)


def load_lie_down_pose(path: str) -> dict:
    """读取趴下姿势 JSON；不存在则返回空 dict。"""
    if not os.path.exists(path):
        return {}
    with open(path) as f:
        data = json.load(f)
    raw = data.get("targets", data)
    return {
        int(mid): float(q)
        for mid, q in raw.items()
        if int(mid) not in NON_BODY_LIE_MOTOR_IDS
    }


def build_lie_down_target(online: Iterable[int], pose_path: str,
                          fallback: Optional[Mapping[int, float]] = None) -> dict:
    """构造趴下目标: 优先使用实机捕获 JSON, 否则 fallback 到内置旧目标。"""
    if fallback is None:
        fallback = LIE_DOWN_TARGETS_RAD
    pose = load_lie_down_pose(pose_path) or dict(fallback)
    online_set = set(online)
    return {
        mid: q for mid, q in pose.items()
        if mid in online_set and mid not in NON_BODY_LIE_MOTOR_IDS
    }


def build_sit_target(online: Iterable[int], pose_path: str,
                     fallback: Optional[Mapping[int, float]] = None) -> dict:
    """构造坐下目标: 仅用 sit_pose.json（无内置 fallback，缺文件则空）。"""
    pose = load_lie_down_pose(pose_path) or dict(fallback or {})
    online_set = set(online)
    return {
        mid: q for mid, q in pose.items()
        if mid in online_set and mid not in NON_BODY_LIE_MOTOR_IDS
    }


def build_zero_target(online: Iterable[int]) -> dict:
    """构造平滑回零目标: 在线关节电机角全部到 0（含头/腰/达妙）。"""
    return {int(mid): 0.0 for mid in online}


__all__ = [
    "NON_BODY_LIE_MOTOR_IDS",
    "LIE_DOWN_TARGETS_RAD",
    "default_lie_down_pose_path",
    "default_sit_pose_path",
    "load_lie_down_pose_from_log",
    "save_lie_down_pose",
    "load_lie_down_pose",
    "build_lie_down_target",
    "build_sit_target",
    "build_zero_target",
]
