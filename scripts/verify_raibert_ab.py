#!/usr/bin/env python3
"""SoftTrot Raibert A/B: log raibert_dx / live amp / step vs CoM per period.

Usage:
  PYTHONPATH=src /path/to/python scripts/verify_raibert_ab.py
"""

from __future__ import annotations

import csv
import math
import os
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PY = os.environ.get(
    "MARDOG_PYTHON",
    "/home/z/miniforge3/envs/gmr/bin/python",
)
OUT = ROOT / "docs" / "baselines"
OUT.mkdir(parents=True, exist_ok=True)


def _run(label: str, extra: list[str], dest: Path) -> Path:
    cmd = [
        PY, "-m", "marsdog_control.apps.sim.sim_walk",
        "--vx", "0.10", "--turn", "0",
        "--headless", "--duration", "10",
        *extra,
    ]
    env = os.environ.copy()
    env["PYTHONPATH"] = str(ROOT / "src")
    print(f"\n=== {label}: {' '.join(extra) if extra else '(default)'} ===")
    r = subprocess.run(cmd, cwd=str(ROOT), env=env, capture_output=True, text=True)
    for ln in (r.stdout or "").splitlines():
        if "[Raibert]" in ln or "[Tel]" in ln or "Test finished" in ln:
            print(ln)
    if r.returncode != 0:
        print((r.stderr or r.stdout or "")[-2000:])
        raise SystemExit(r.returncode)
    src = ROOT / "telemetry.csv"
    shutil.copy2(src, dest)
    return dest


def _analyze(path: Path) -> dict:
    rows = list(csv.DictReader(path.open()))
    if not rows:
        return {}

    def col(k):
        return [float(r[k]) for r in rows if k in r and r[k] != ""]

    t = col("t")
    vx = col("vx_truth") if "vx_truth" in rows[0] else col("vx")
    vx_cmd = col("vx_cmd")
    period = col("period")
    r_on = col("raibert_on") if "raibert_on" in rows[0] else [0.0] * len(t)
    r_dx = col("raibert_dx") if "raibert_dx" in rows[0] else [0.0] * len(t)
    r_af = col("raibert_amp_front") if "raibert_amp_front" in rows[0] else [0.0] * len(t)
    r_ar = col("raibert_amp_rear") if "raibert_amp_rear" in rows[0] else [0.0] * len(t)
    af = col("amp_front")
    ar = col("amp_rear")
    roll = col("roll")

    # Engage window: body accelerating, Raibert dx should be most visible.
    eng = [(i, ti) for i, ti in enumerate(t) if 1.0 <= ti <= 3.5]
    cruise = [(i, ti) for i, ti in enumerate(t) if ti >= 4.0]

    def _mean(idxs, arr):
        if not idxs:
            return 0.0
        return sum(arr[i] for i, _ in idxs) / len(idxs)

    def _p95_abs(idxs, arr):
        if not idxs:
            return 0.0
        vals = sorted(abs(arr[i]) for i, _ in idxs)
        return vals[int(0.95 * (len(vals) - 1))]

    on_eng = [i for i, _ in eng if r_on[i] > 0.5]
    live_af = [r_af[i] if r_on[i] > 0.5 else af[i] for i in range(len(t))]
    live_ar = [r_ar[i] if r_on[i] > 0.5 else ar[i] for i in range(len(t))]
    step = [abs(live_af[i]) + abs(live_ar[i]) for i in range(len(t))]
    T = _mean(cruise, period) or 1.2
    vx_c = _mean(cruise, vx)
    # Classic identity: step_length ≈ vx * period  (foot hip-frame stroke ↔ CoM travel)
    step_c = _mean(cruise, step)
    com_per_T = vx_c * T

    # Correlation dx vs (v - v*) in engage (should ≈ kx when on)
    if on_eng:
        xs = [vx[i] - vx_cmd[i] for i in on_eng]
        ys = [r_dx[i] for i in on_eng]
        mx = sum(xs) / len(xs)
        my = sum(ys) / len(ys)
        num = sum((a - mx) * (b - my) for a, b in zip(xs, ys))
        denx = math.sqrt(sum((a - mx) ** 2 for a in xs)) or 1e-12
        deny = math.sqrt(sum((b - my) ** 2 for b in ys)) or 1e-12
        corr = num / (denx * deny)
        # slope ≈ kx
        den = sum((a - mx) ** 2 for a in xs) or 1e-12
        slope = num / den
    else:
        corr = slope = 0.0

    return {
        "path": str(path),
        "raibert_on_pct": 100.0 * (sum(1 for x in r_on if x > 0.5) / len(r_on)),
        "eng_dx_mean_mm": 1000.0 * _mean(eng, r_dx),
        "eng_dx_p95_mm": 1000.0 * _p95_abs(eng, r_dx),
        "cruise_dx_mean_mm": 1000.0 * _mean(cruise, r_dx),
        "eng_vx_err": _mean(eng, [vx[i] - vx_cmd[i] for i in range(len(t))]),
        "dx_vs_verr_corr_eng": corr,
        "dx_vs_verr_slope_eng": slope,
        "cruise_vx": vx_c,
        "cruise_vx_cmd": _mean(cruise, vx_cmd),
        "step_len_m": step_c,
        "com_dx_per_T_m": com_per_T,
        "step_vs_com_err_cm": 100.0 * (step_c - com_per_T),
        "roll_pk_deg": math.degrees(max(abs(x) for x in roll)),
        "live_amp_cm": (
            100.0 * _mean(cruise, live_af),
            100.0 * _mean(cruise, live_ar),
        ),
    }


def _print(label: str, s: dict) -> None:
    print(f"\n--- {label} ---")
    if not s:
        print("  (empty)")
        return
    print(f"  Raibert on: {s['raibert_on_pct']:.0f}%")
    print(
        f"  engage dx mean/p95: {s['eng_dx_mean_mm']:+.2f} / "
        f"{s['eng_dx_p95_mm']:.2f} mm"
    )
    print(f"  cruise dx mean: {s['cruise_dx_mean_mm']:+.2f} mm")
    print(
        f"  engage dx~(v-v*) corr/slope: {s['dx_vs_verr_corr_eng']:+.3f} / "
        f"{s['dx_vs_verr_slope_eng']:+.3f} (slope≈kx)"
    )
    print(
        f"  cruise vx/cmd: {s['cruise_vx']:.4f} / {s['cruise_vx_cmd']:.4f} m/s"
    )
    print(
        f"  step_len={100*s['step_len_m']:.2f}cm  "
        f"CoM/T={100*s['com_dx_per_T_m']:.2f}cm  "
        f"Δ={s['step_vs_com_err_cm']:+.2f}cm"
    )
    print(
        f"  live amp F/R: {s['live_amp_cm'][0]:.2f}/{s['live_amp_cm'][1]:.2f}cm  "
        f"roll_pk={s['roll_pk_deg']:.1f}°"
    )


def main() -> None:
    off = _run("OFF", ["--no-raibert"], OUT / "tel_raibert_off.csv")
    on = _run(
        "ON kx=0.20",
        ["--raibert", "--raibert-kx", "0.20", "--raibert-dx-max", "0.03"],
        OUT / "tel_raibert_on_kx020.csv",
    )
    s_off = _analyze(off)
    s_on = _analyze(on)
    _print("OFF", s_off)
    _print("ON kx=0.20", s_on)
    print("\n=== 判读 ===")
    print(
        "1) ON 时 engage |dx| 应明显大于 OFF；corr(dx, v-v*) 应接近 +1，slope≈kx。"
    )
    print(
        "2) step≈|af|+|ar| 与 CoM/T=vx·T 接近，是开环运动学身份（名义落点），"
        "不是反馈 dx。"
    )
    print(
        "3) cruise 若 vx≈vx_cmd，dx→0 正常：Raibert 反馈只在速度误差时工作。"
    )


if __name__ == "__main__":
    main()
