#!/usr/bin/env python3
"""一键离线跑老程序/重构版并比较目标轨迹。

默认流程:
  1) 调用老程序 mocap_to_real/offline_trajectory.py 导出 old.csv
  2) 调用当前重构版 mocap_to_real/offline_trajectory.py 导出 refactor.csv
  3) 按 (tick, motor_id) 对齐 target_deg, 输出逐点 diff 和按电机汇总

整个过程只跑 fake hardware, 不打开串口/CAN, 不下发真实点位。
"""

from __future__ import annotations

import argparse
import csv
import math
import subprocess
import sys
from collections import defaultdict
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OLD_SCRIPT = Path("/home/cat/公共的/20260712_1628/mocap_to_real/offline_trajectory.py")


def _run_export(script: Path, ticks: int, output: Path, walk_args: list[str]) -> None:
    cmd = [sys.executable, str(script), "--ticks", str(ticks), "--output", str(output)]
    if walk_args:
        cmd += ["--"] + walk_args
    subprocess.run(cmd, check=True)


def _read_csv(path: Path):
    rows = {}
    names = {}
    with path.open(newline="") as fh:
        for row in csv.DictReader(fh):
            key = (int(row["tick"]), int(row["motor_id"]))
            rows[key] = row
            names[int(row["motor_id"])] = row.get("name", "")
    return rows, names


def _float(row, key):
    try:
        return float(row[key])
    except Exception:
        return float("nan")


def _rms(values):
    vals = [v for v in values if math.isfinite(v)]
    if not vals:
        return float("nan")
    return math.sqrt(sum(v * v for v in vals) / len(vals))


def _fmt(x):
    return "" if not math.isfinite(x) else f"{x:.6f}"


def compare(old_csv: Path, new_csv: Path, diff_csv: Path, summary_csv: Path):
    old, old_names = _read_csv(old_csv)
    new, new_names = _read_csv(new_csv)
    keys = sorted(set(old) | set(new))
    by_motor = defaultdict(list)
    missing_old = missing_new = 0

    diff_csv.parent.mkdir(parents=True, exist_ok=True)
    with diff_csv.open("w", newline="") as fh:
        writer = csv.DictWriter(
            fh,
            fieldnames=["tick", "motor_id", "name", "old_target_deg",
                        "refactor_target_deg", "diff_deg", "abs_diff_deg"],
        )
        writer.writeheader()
        for tick, mid in keys:
            o = old.get((tick, mid))
            n = new.get((tick, mid))
            if o is None:
                missing_old += 1
            if n is None:
                missing_new += 1
            old_target = _float(o, "target_deg") if o else float("nan")
            new_target = _float(n, "target_deg") if n else float("nan")
            diff = new_target - old_target if math.isfinite(old_target) and math.isfinite(new_target) else float("nan")
            name = new_names.get(mid) or old_names.get(mid) or ""
            by_motor[mid].append((old_target, new_target, diff))
            writer.writerow({
                "tick": tick,
                "motor_id": mid,
                "name": name,
                "old_target_deg": _fmt(old_target),
                "refactor_target_deg": _fmt(new_target),
                "diff_deg": _fmt(diff),
                "abs_diff_deg": _fmt(abs(diff) if math.isfinite(diff) else float("nan")),
            })

    summaries = []
    for mid in sorted(by_motor):
        triples = by_motor[mid]
        old_vals = [x[0] for x in triples if math.isfinite(x[0])]
        new_vals = [x[1] for x in triples if math.isfinite(x[1])]
        diffs = [x[2] for x in triples if math.isfinite(x[2])]
        max_abs = max((abs(x) for x in diffs), default=float("nan"))
        summaries.append({
            "motor_id": mid,
            "name": new_names.get(mid) or old_names.get(mid) or "",
            "samples": len(diffs),
            "rms_diff_deg": _rms(diffs),
            "max_abs_diff_deg": max_abs,
            "old_min_deg": min(old_vals, default=float("nan")),
            "old_max_deg": max(old_vals, default=float("nan")),
            "refactor_min_deg": min(new_vals, default=float("nan")),
            "refactor_max_deg": max(new_vals, default=float("nan")),
        })
    summaries.sort(key=lambda r: (-(r["max_abs_diff_deg"] if math.isfinite(r["max_abs_diff_deg"]) else -1), r["motor_id"]))

    with summary_csv.open("w", newline="") as fh:
        writer = csv.DictWriter(
            fh,
            fieldnames=["motor_id", "name", "samples", "rms_diff_deg",
                        "max_abs_diff_deg", "old_min_deg", "old_max_deg",
                        "refactor_min_deg", "refactor_max_deg"],
        )
        writer.writeheader()
        for row in summaries:
            writer.writerow({k: (_fmt(v) if isinstance(v, float) else v) for k, v in row.items()})
    return summaries, missing_old, missing_new


def main():
    ap = argparse.ArgumentParser(description="无机器开环跑老程序/重构版并比较目标轨迹")
    ap.add_argument("--ticks", type=int, default=240)
    ap.add_argument("--old-script", default=str(DEFAULT_OLD_SCRIPT))
    ap.add_argument("--new-script", default=str(PROJECT_ROOT / "mocap_to_real" / "offline_trajectory.py"))
    ap.add_argument("--output-dir", default=str(PROJECT_ROOT / "mocap_to_real" / "log" / "offline_compare"))
    ap.add_argument("--skip-run", action="store_true", help="不重新导出, 直接比较 output-dir 下已有 CSV")
    ap.add_argument("walk_args", nargs=argparse.REMAINDER,
                    help="传给两边 walk.py 的额外参数, 例如: -- --trot --natural-soft-trot")
    args = ap.parse_args()

    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    old_csv = out_dir / "old.csv"
    new_csv = out_dir / "refactor.csv"
    diff_csv = out_dir / "diff.csv"
    summary_csv = out_dir / "summary.csv"
    walk_args = args.walk_args[1:] if args.walk_args[:1] == ["--"] else args.walk_args

    if not args.skip_run:
        _run_export(Path(args.old_script), args.ticks, old_csv, walk_args)
        _run_export(Path(args.new_script), args.ticks, new_csv, walk_args)

    summaries, missing_old, missing_new = compare(old_csv, new_csv, diff_csv, summary_csv)
    print(f"[compare] old={old_csv}")
    print(f"[compare] refactor={new_csv}")
    print(f"[compare] diff={diff_csv}")
    print(f"[compare] summary={summary_csv}")
    if missing_old or missing_new:
        print(f"[compare] missing rows: old={missing_old}, refactor={missing_new}")
    print("[compare] top drift motors:")
    for row in summaries[:8]:
        print(
            f"  id={row['motor_id']:>2} {row['name']:<14} "
            f"max={_fmt(row['max_abs_diff_deg'])}deg rms={_fmt(row['rms_diff_deg'])}deg"
        )


if __name__ == "__main__":
    main()
