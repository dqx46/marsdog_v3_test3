"""Thin CSV logger for motor tracking benches (plot_tracking compatible)."""

from __future__ import annotations

import csv
import datetime
import math
from pathlib import Path
from typing import Iterable, Mapping, Optional, TextIO

HEADER = [
    "t_s",
    "run_t_s",
    "mode",
    "motor_id",
    "name",
    "target_deg",
    "actual_deg",
    "error_deg",
    "torque_nm",
    "actual_kp",
    "actual_kd",
]


def setup_motor_track_log(
    log_dir: Path,
    *,
    mode: str,
    enabled: bool = True,
) -> tuple[Optional[TextIO], Optional[csv.writer], Optional[Path]]:
    if not enabled:
        return None, None, None
    log_dir.mkdir(parents=True, exist_ok=True)
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    path = log_dir / f"bench_motor_{mode}_{ts}.csv"
    fh = path.open("w", newline="", encoding="utf-8")
    writer = csv.writer(fh)
    writer.writerow(HEADER)
    fh.flush()
    return fh, writer, path


def write_motor_track_rows(
    writer: Optional[csv.writer],
    *,
    t_s: float,
    run_t_s: float,
    mode: str,
    motor_ids: Iterable[int],
    names: Mapping[int, str],
    targets_rad: Mapping[int, float],
    actuals_rad: Mapping[int, float],
    kp_by_id: Mapping[int, float],
    kd_by_id: Mapping[int, float],
    torque_by_id: Optional[Mapping[int, float]] = None,
) -> None:
    if writer is None:
        return
    torque_by_id = torque_by_id or {}
    for mid in motor_ids:
        if mid not in targets_rad or mid not in actuals_rad:
            continue
        tgt = math.degrees(targets_rad[mid])
        act = math.degrees(actuals_rad[mid])
        writer.writerow([
            f"{t_s:.6f}",
            f"{run_t_s:.6f}",
            mode,
            mid,
            names.get(mid, f"ID{mid}"),
            f"{tgt:.6f}",
            f"{act:.6f}",
            f"{tgt - act:.6f}",
            f"{float(torque_by_id.get(mid, float('nan'))):.6f}",
            f"{float(kp_by_id.get(mid, float('nan'))):.4f}",
            f"{float(kd_by_id.get(mid, float('nan'))):.4f}",
        ])


def close_log(fh: Optional[TextIO]) -> None:
    if fh is not None:
        fh.flush()
        fh.close()
