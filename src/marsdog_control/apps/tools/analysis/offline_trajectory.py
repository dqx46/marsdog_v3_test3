#!/usr/bin/env python3
"""无机器开环轨迹导出。

这个脚本把 parity harness 从“单元测试内部工具”变成现场可直接运行的
轨迹导出工具。它会用 fake drivers/fake clock 启动真实 walk.main(), 记录主循环
中 send_all() 收到的目标, 不打开串口/CAN, 不下发真实电机点位。
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
TESTS_PARITY = PROJECT_ROOT / "tests" / "parity"
SRC_DIR = PROJECT_ROOT / "src"
for path in (PROJECT_ROOT, TESTS_PARITY, SRC_DIR, PROJECT_ROOT / "mocap_to_real"):
    p = str(path)
    if p not in sys.path:
        sys.path.insert(0, p)

from loop_harness import run_legacy_loop  # noqa: E402
from marsdog_control.compat import ensure_legacy_path  # noqa: E402

ensure_legacy_path()
from marsdog_control.config.joints import JOINT_BY_ID  # noqa: E402


def _deg(x):
    return math.degrees(float(x)) if x is not None and math.isfinite(float(x)) else float("nan")


def run(extra_args: list[str], ticks: int):
    stream = run_legacy_loop(extra_argv=extra_args, n_ticks=ticks)
    rows = []
    for tick, send in enumerate(stream):
        targets = send.get("targets") or {}
        velocities = send.get("velocities") or {}
        kp_phase = send.get("kp_phase") or {}
        trq_ff = send.get("trq_ff") or {}
        for mid_text in sorted(targets, key=lambda x: int(x)):
            mid = int(mid_text)
            joint = JOINT_BY_ID.get(mid)
            rows.append({
                "tick": tick,
                "motor_id": mid,
                "name": joint.name if joint else "",
                "target_deg": _deg(targets[mid_text]),
                "velocity_rad_s": velocities.get(mid_text, float("nan")),
                "kp_phase": kp_phase.get(mid_text, float("nan")),
                "trq_ff_nm": trq_ff.get(mid_text, float("nan")),
            })
    return rows


def main():
    ap = argparse.ArgumentParser(description="无机器开环导出重构版 walk 目标轨迹")
    ap.add_argument("--ticks", type=int, default=240, help="记录多少个主循环tick")
    ap.add_argument("--output", default=str(PROJECT_ROOT / "mocap_to_real" / "log" / "offline_trajectory.csv"))
    ap.add_argument("walk_args", nargs=argparse.REMAINDER,
                    help="传给 walk.py 的额外参数, 例如: -- --trot --natural-soft-trot")
    args = ap.parse_args()
    extra = args.walk_args[1:] if args.walk_args[:1] == ["--"] else args.walk_args
    rows = run(extra, args.ticks)
    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", newline="") as fh:
        writer = csv.DictWriter(
            fh,
            fieldnames=["tick", "motor_id", "name", "target_deg",
                        "velocity_rad_s", "kp_phase", "trq_ff_nm"],
        )
        writer.writeheader()
        writer.writerows(rows)
    print(f"[offline] wrote {out} rows={len(rows)} ticks={args.ticks}")


if __name__ == "__main__":
    main()
