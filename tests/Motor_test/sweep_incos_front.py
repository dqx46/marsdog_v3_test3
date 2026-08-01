#!/usr/bin/env python3
"""Sine-sweep the four front Incos motors (IDs 2,3,6,7).

Protocol (ENCOS V1.19 力位混控):
  KP 0..500, KD 0..5 (9-bit); empty-load ref KP≈15, KD≈0.5.
  Fault bits: 1 overheat, 2 overcurrent, 3/4 voltage, 5 encoder, ...

Safety: dog MUST be hung / stand-supported. Hands near e-stop.

Example:
  cd /home/cat/marsdog_v3_test3
  ./run_with_env.sh python tests/Motor_test/sweep_incos_front.py
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

_HERE = Path(__file__).resolve().parent
_REPO = _HERE.parents[1]
if str(_HERE) not in sys.path:
    sys.path.insert(0, str(_HERE))
if str(_REPO / "src") not in sys.path:
    sys.path.insert(0, str(_REPO / "src"))

from marsdog_control.compat import ensure_legacy_path  # noqa: E402

ensure_legacy_path()

from marsdog_control.config.joints import JOINT_BY_ID  # noqa: E402
from marsdog_control.hardware.board import RkMotorBoard  # noqa: E402

from bench_motor_track import run_loop, send_hold  # noqa: E402
from marsdog_control.hardware.actuation import ActuationRuntime  # noqa: E402
from marsdog_control.config.joints import DEFAULT_DM_KD, DEFAULT_DM_KP  # noqa: E402
from marsdog_control.config.gains import JOINT_GAINS  # noqa: E402

DEFAULT_IDS = (2, 3, 6, 7)  # fl_thigh_roll, fl_calf, fr_thigh_roll, fr_calf
DEFAULT_LOG = _HERE / "log"
# Manual empty-load 15/0.5; climb toward previous walk numbers carefully.
DEFAULT_KP = (15.0, 25.0, 35.0, 45.0)
DEFAULT_KD = (0.8, 1.5, 2.5)


def _parse_floats(text: str) -> tuple[float, ...]:
    return tuple(float(x.strip()) for x in text.split(",") if x.strip())


def _parse_ids(text: str) -> tuple[int, ...]:
    return tuple(int(x.strip(), 0) for x in text.split(",") if x.strip())


def _faults(board: RkMotorBoard, ids: tuple[int, ...]) -> dict[int, int]:
    out = {}
    incos = board.incos
    if incos is None:
        return {mid: -1 for mid in ids}
    for mid in ids:
        idx = mid - 1
        out[mid] = int(incos.fault[idx]) if 0 <= idx < len(incos.fault) else -1
    return out


def _enabled(board: RkMotorBoard, ids: tuple[int, ...]) -> dict[int, bool]:
    out = {}
    incos = board.incos
    if incos is None:
        return {mid: False for mid in ids}
    for mid in ids:
        idx = mid - 1
        out[mid] = bool(incos.is_enabled[idx]) if 0 <= idx < len(incos.is_enabled) else False
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--ids", type=_parse_ids, default=DEFAULT_IDS)
    ap.add_argument("--kp", type=_parse_floats, default=DEFAULT_KP)
    ap.add_argument("--kd", type=_parse_floats, default=DEFAULT_KD)
    ap.add_argument("--amp-deg", type=float, default=3.0,
                    help="sine amplitude (deg); keep small for first pass")
    ap.add_argument("--period", type=float, default=2.0)
    ap.add_argument("--sec", type=float, default=5.0)
    ap.add_argument("--hz", type=float, default=200.0)
    ap.add_argument("--settle", type=float, default=1.0,
                    help="hold q0 between cells (s)")
    ap.add_argument("--log-dir", type=Path, default=DEFAULT_LOG)
    ap.add_argument("--stop-on-fault", action=argparse.BooleanOptionalAction,
                    default=True)
    args = ap.parse_args()

    for mid in args.ids:
        j = JOINT_BY_ID.get(mid)
        if j is None or j.mtype != "incos":
            raise SystemExit(f"[FATAL] id {mid} is not an Incos joint")

    amp = math.radians(args.amp_deg)
    omega = 2.0 * math.pi / max(1e-3, args.period)
    args.log_dir.mkdir(parents=True, exist_ok=True)

    print("=" * 64)
    print("ENCOS front-leg MIT sweep (力位混控)")
    print("  Manual empty-load ref: KP≈15  KD≈0.5  |  KP∈[0,500] KD∈[0,5]")
    print(f"  ids={args.ids}")
    print(f"  names={[JOINT_BY_ID[m].name for m in args.ids]}")
    print(f"  kp grid={args.kp}  kd grid={args.kd}")
    print(f"  ±{args.amp_deg}°  T={args.period}s  cell={args.sec}s")
    print("  HANG THE DOG. Ctrl-C aborts.")
    print("=" * 64)

    board = RkMotorBoard()
    results = []
    try:
        board.start()
        online = board.online_ids()
        print(f"[init] online {len(online)} motors")
        offline = [m for m in args.ids if m not in online]
        if offline:
            print(f"[FATAL] offline: {offline}")
            return 1
        for mid in args.ids:
            q = board.get_angles((mid,)).get(mid, float("nan"))
            print(f"  ID{mid} {JOINT_BY_ID[mid].name:16s} "
                  f"q0={math.degrees(q):+.2f}°")

        for kp in args.kp:
            for kd in args.kd:
                tag = f"kp{kp:g}_kd{kd:g}"
                print(f"\n>>> {tag}")
                # Abort early if already faulted from previous cell
                faults0 = _faults(board, args.ids)
                if any(f > 0 for f in faults0.values()):
                    print(f"[ABORT] pre-cell fault={faults0}")
                    results.append((kp, kd, None, faults0, "pre_fault"))
                    if args.stop_on_fault:
                        break

                def q_cmd(mid, t, q0, _amp=amp, _w=omega):
                    return q0 + _amp * math.sin(_w * t)

                def dq_cmd(mid, t, q0, _amp=amp, _w=omega):
                    return _amp * _w * math.cos(_w * t)

                try:
                    path = run_loop(
                        board,
                        mode="sine",
                        test_ids=args.ids,
                        kp=float(kp),
                        kd=float(kd),
                        hz=float(args.hz),
                        duration_s=float(args.sec),
                        q_cmd_fn=q_cmd,
                        dq_cmd_fn=dq_cmd,
                        log_dir=args.log_dir,
                        no_log=False,
                    )
                except Exception as exc:
                    print(f"[FAIL] {tag}: {exc}")
                    results.append((kp, kd, None, _faults(board, args.ids), str(exc)))
                    if args.stop_on_fault:
                        break
                    continue

                faults = _faults(board, args.ids)
                en = _enabled(board, args.ids)
                # Recompute RMS from last path quickly
                rms = {}
                if path is not None and path.exists():
                    import csv
                    buckets = {m: [] for m in args.ids}
                    with path.open() as fh:
                        for row in csv.DictReader(fh):
                            mid = int(float(row["motor_id"]))
                            if mid in buckets and row.get("error_deg"):
                                buckets[mid].append(float(row["error_deg"]))
                    for mid, errs in buckets.items():
                        rms[mid] = (
                            math.sqrt(sum(e * e for e in errs) / len(errs))
                            if errs else float("nan"))
                print(
                    "  RMS_deg="
                    + ", ".join(
                        f"{JOINT_BY_ID[m].name}={rms.get(m, float('nan')):.2f}"
                        for m in args.ids)
                )
                print(f"  fault={faults}  enabled={en}")
                bad = any(f > 0 for f in faults.values()) or not all(en.values())
                results.append((kp, kd, rms, faults, "fault" if bad else "ok"))
                if bad and args.stop_on_fault:
                    print("[ABORT] fault or lost enable")
                    break

                # settle at current pose with mild MIT gains (manual empty-load band)
                q_hold = board.get_angles(sorted(board.online_ids()))
                rt = ActuationRuntime(
                    dm_tarsus_active=False,
                    dm_fixed_targets=dict(board.dm_fixed_targets),
                    dm_reference_lead_s={4: 0.0, 8: 0.0},
                    dm_reference_lead_max_rad=0.0,
                    active_dm_kp_by_id={4: DEFAULT_DM_KP, 8: DEFAULT_DM_KP},
                    active_dm_kp=DEFAULT_DM_KP,
                    active_dm_kd_by_id={4: DEFAULT_DM_KD, 8: DEFAULT_DM_KD},
                    active_dm_kd=DEFAULT_DM_KD,
                    default_dm_kp=DEFAULT_DM_KP,
                    default_dm_kd=DEFAULT_DM_KD,
                    dm_dq_max_rps=3.0,
                    dm_dq_feedforward=False,
                    leg_kp_scale=1.0,
                    joint_gains=JOINT_GAINS,
                )
                t_s = time.monotonic()
                while time.monotonic() - t_s < args.settle:
                    send_hold(board, rt, q_hold, kp=15.0, kd=0.8)
                    time.sleep(0.01)
            else:
                continue
            break

    finally:
        try:
            board.disable()
        except Exception as exc:
            print(f"[cleanup] disable: {exc}")
        try:
            board.close()
        except Exception as exc:
            print(f"[cleanup] close: {exc}")
        print("[cleanup] done")

    print("\n======= summary (ENCOS front Incos) =======")
    hdr = f"{'kp':>6} {'kd':>6}  status  " + " ".join(
        f"{JOINT_BY_ID[m].name[:10]:>10}" for m in args.ids)
    print(hdr)
    for kp, kd, rms, faults, status in results:
        if rms is None:
            print(f"{kp:6g} {kd:6g}  {status:6s}  faults={faults}")
        else:
            print(
                f"{kp:6g} {kd:6g}  {status:6s}  "
                + " ".join(f"{rms.get(m, float('nan')):10.2f}" for m in args.ids)
                + f"  fault={faults}"
            )
    print(
        "\nPick lowest-RMS cell without fault/squeal; write the SAME kp/kd "
        "into gains.py for fl/fr_thigh_roll and/or fl/fr_calf (L/R matched)."
    )
    print("Manual tip: start near KP=15 KD=0.5 empty-load; raise slowly under load.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
