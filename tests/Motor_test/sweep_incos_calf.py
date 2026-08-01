#!/usr/bin/env python3
"""Grid sine-sweep for Incos front calves (IDs 3,7) — brand-native kp/kd.

Safety: dog MUST be hung / stand-supported. Hands near e-stop.

Writes one CSV per (kp,kd) cell under tests/Motor_test/log/sweep_incos_*.
Prints RMS tracking error so you can pick a symmetric JOINT_GAINS entry.

Example:
  cd /home/cat/marsdog_v3_test3
  PYTHONPATH=src:mocap_to_real python3 tests/Motor_test/sweep_incos_calf.py \\
      --kp 40,50,60 --kd 2.0,2.5,3.0 --sec 6 --amp-deg 4
"""

from __future__ import annotations

import argparse
import csv
import math
import subprocess
import sys
from pathlib import Path

_HERE = Path(__file__).resolve().parent
_REPO = _HERE.parents[1]
_BENCH = _HERE / "bench_motor_track.py"
_LOG = _HERE / "log"


def _parse_floats(text: str) -> list[float]:
    return [float(x.strip()) for x in text.split(",") if x.strip()]


def _rms_error(csv_path: Path, motor_ids: tuple[int, ...]) -> dict[int, float]:
    by: dict[int, list[float]] = {mid: [] for mid in motor_ids}
    with csv_path.open() as fh:
        for row in csv.DictReader(fh):
            mid = int(float(row["motor_id"]))
            if mid in by and row.get("error_deg"):
                by[mid].append(float(row["error_deg"]))
    out = {}
    for mid, errs in by.items():
        if not errs:
            out[mid] = float("nan")
        else:
            out[mid] = math.sqrt(sum(e * e for e in errs) / len(errs))
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--ids", default="3,7")
    ap.add_argument("--kp", default="40,50,60", help="comma kp grid")
    ap.add_argument("--kd", default="2.0,2.5,3.0", help="comma kd grid")
    ap.add_argument("--amp-deg", type=float, default=4.0)
    ap.add_argument("--period", type=float, default=2.0)
    ap.add_argument("--sec", type=float, default=6.0)
    ap.add_argument("--hz", type=float, default=200.0)
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    ids = tuple(int(x.strip()) for x in args.ids.split(",") if x.strip())
    kps = _parse_floats(args.kp)
    kds = _parse_floats(args.kd)
    _LOG.mkdir(parents=True, exist_ok=True)

    print("=" * 60)
    print("Incos calf sweep — HANG the dog. Ctrl-C aborts.")
    print(f"ids={ids}  kp={kps}  kd={kds}  ±{args.amp-deg}°  T={args.period}s")
    print("=" * 60)

    results = []
    for kp in kps:
        for kd in kds:
            tag = f"kp{kp:g}_kd{kd:g}"
            cmd = [
                sys.executable, str(_BENCH), "sine",
                "--ids", ",".join(str(i) for i in ids),
                "--amp-deg", str(args.amp_deg),
                "--period", str(args.period),
                "--kp", str(kp),
                "--kd", str(kd),
                "--hz", str(args.hz),
                "--sec", str(args.sec),
                "--log-dir", str(_LOG),
            ]
            print(f"\n>>> {tag}")
            if args.dry_run:
                print(" ", " ".join(cmd))
                continue
            proc = subprocess.run(cmd, cwd=str(_REPO), env={
                **dict(**{k: v for k, v in __import__("os").environ.items()}),
                "PYTHONPATH": f"{_REPO / 'src'}:{_REPO / 'mocap_to_real'}",
            })
            if proc.returncode != 0:
                print(f"[FAIL] {tag} exit={proc.returncode}")
                results.append((kp, kd, None, proc.returncode))
                continue
            # newest matching log
            logs = sorted(_LOG.glob("bench_motor_sine_*.csv"), key=lambda p: p.stat().st_mtime)
            if not logs:
                print(f"[FAIL] {tag} no log")
                results.append((kp, kd, None, -1))
                continue
            latest = logs[-1]
            rms = _rms_error(latest, ids)
            print(f"  log={latest.name}  RMS_deg=" +
                  ", ".join(f"id{m}={rms[m]:.2f}" for m in ids))
            results.append((kp, kd, rms, 0))

    print("\n======= summary (pick lowest RMS without squeal / fault) =======")
    print(f"{'kp':>6} {'kd':>6}  " + " ".join(f"rms{m:>3}" for m in ids))
    for kp, kd, rms, rc in results:
        if rms is None:
            print(f"{kp:6g} {kd:6g}  FAIL rc={rc}")
        else:
            print(f"{kp:6g} {kd:6g}  " +
                  " ".join(f"{rms[m]:6.2f}" for m in ids))
    print("\nWrite the chosen symmetric pair into "
          "src/marsdog_control/config/gains.py  fl_calf / fr_calf.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
