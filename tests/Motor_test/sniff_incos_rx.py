#!/usr/bin/env python3
"""Raw CAN-A standard-frame sniffer for Incos ID3 vs ID7.

Counts frames *before* MotorIncos parsing, on the same shared bus as walk.
If the wire already has almost no ID=3 frames, the driver is not the culprit.
If the wire has many ID=3 frames but MotorIncos.rx_count stays low, then
driver/handler is dropping them.

Usage (repo root, dog hung):
  cd /home/cat/project/marsdogv3_test1
  export PYTHONPATH=src:mocap_to_real
  python3 tests/Motor_test/sniff_incos_rx.py
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from collections import Counter
from pathlib import Path

_HERE = Path(__file__).resolve().parent
_REPO = _HERE.parents[1]
sys.path.insert(0, str(_HERE))
sys.path.insert(0, str(_REPO / "src"))

from marsdog_control.compat import ensure_legacy_path

ensure_legacy_path()

from marsdog_control.hardware.board import RkMotorBoard
from marsdog_control.hardware.motors.can_serial import CAN_EFF_FLAG

from bench_motor_track import build_actuation_runtime, send_hold


def main() -> int:
    ap = argparse.ArgumentParser(description="Sniff raw std CAN ids on Incos bus")
    ap.add_argument("--sec", type=float, default=5.0)
    ap.add_argument("--ids", default="3,7",
                    help="comma ids to command (offline ones skipped)")
    ap.add_argument("--amp-deg", type=float, default=5.0)
    ap.add_argument("--period", type=float, default=2.0)
    ap.add_argument("--kp", type=float, default=50.0)
    ap.add_argument("--kd", type=float, default=1.0)
    ap.add_argument("--hz", type=float, default=200.0)
    args = ap.parse_args()
    args.ids = tuple(int(x) for x in args.ids.split(",") if x.strip())

    board = RkMotorBoard()
    raw_ids: Counter[int] = Counter()
    state = {"ext": 0}

    def sniff(can_id, dlc, data):
        if can_id & CAN_EFF_FLAG:
            state["ext"] += 1
            return
        raw_ids[can_id & 0x7FF] += 1

    try:
        print("[init] starting RkMotorBoard...")
        board.start()
        if board.lz is None or board.incos is None:
            print("[FATAL] lz/incos not up")
            return 1

        # Register AFTER start so we sit alongside the Incos handler.
        board.lz.add_can1_standard_handler(sniff)

        ap_ids = getattr(args, "ids", (3, 7))
        online = board.online_ids()
        test_ids = tuple(mid for mid in ap_ids if mid in online)
        if not test_ids:
            print(f"[FATAL] none of {ap_ids} online; online={sorted(online)}")
            return 1
        missing = [mid for mid in ap_ids if mid not in online]
        if missing:
            print(f"[warn] offline (skipped): {missing}")

        rx0 = {mid: int(board.incos.rx_count[mid - 1]) for mid in test_ids}
        tx0 = {mid: int(board.incos.tx_count[mid - 1]) for mid in test_ids}

        q0 = board.get_angles(online)
        hold = dict(q0)
        # Only command --ids (others get no MIT → 500ms heartbeat stop).
        # This isolates "ID7 on the wire" vs "ID7 also being commanded".
        amp = math.radians(args.amp_deg)
        omega = 2.0 * math.pi / max(0.2, args.period)
        dt = 1.0 / max(1.0, args.hz)
        rt = build_actuation_runtime(board, dm_active=False, kp=args.kp, kd=args.kd)

        print(f"[sniff] command_only={test_ids} online_incos="
              f"{sorted(m for m in (3, 7) if m in online)} "
              f"amp={args.amp_deg}° hz={args.hz} for {args.sec:.1f}s")
        t0 = time.monotonic()
        step = 0
        while time.monotonic() - t0 < args.sec:
            t = time.monotonic() - t0
            targets = {}
            for mid in test_ids:
                targets[mid] = hold[mid] + amp * math.sin(omega * t)
            send_hold(board, rt, targets, kp=args.kp, kd=args.kd)
            step += 1
            sleep_t = t0 + (step + 1) * dt - time.monotonic()
            if sleep_t > 0:
                time.sleep(sleep_t)

        rx1 = {mid: int(board.incos.rx_count[mid - 1]) for mid in test_ids}
        tx1 = {mid: int(board.incos.tx_count[mid - 1]) for mid in test_ids}

        print("\n=== Raw standard CAN frames on CAN-A (before Incos parse) ===")
        for mid in sorted(raw_ids):
            mark = "  <-- Incos" if mid in (3, 7) else ""
            print(f"  CAN_ID={mid:3d}  frames={raw_ids[mid]}{mark}")
        if not raw_ids:
            print("  (none)")
        print(f"  (extended frames not counted here: {state['ext']})")

        print("\n=== MotorIncos counters (delta) ===")
        for mid in test_ids:
            dtx = tx1[mid] - tx0[mid]
            drx = rx1[mid] - rx0[mid]
            ratio = (drx / dtx) if dtx else 0.0
            print(f"  ID{mid}: txΔ={dtx}  rxΔ={drx}  rx/tx={ratio:.3f}  "
                  f"raw_std_id{mid}={raw_ids.get(mid, 0)}")

        print("\n=== Verdict ===")
        for mid in test_ids:
            dtx = tx1[mid] - tx0[mid]
            drx = rx1[mid] - rx0[mid]
            raw = raw_ids.get(mid, 0)
            if dtx <= 0:
                continue
            ratio = drx / dtx
            if ratio < 0.1:
                print(f"  ID{mid}: 回报稀疏 ({ratio:.1%}) — MIT 几乎不回")
            elif ratio > 0.8:
                print(f"  ID{mid}: 回报正常 ({ratio:.1%})")
            else:
                print(f"  ID{mid}: 回报偏少 ({ratio:.1%})")
            if raw and abs(raw - drx) > max(5, raw * 0.2):
                print(f"  ID{mid}: raw≠rx — 驱动可能漏解析")
        return 0
    finally:
        try:
            board.disable()
        except Exception:
            pass
        try:
            board.close()
        except Exception:
            pass
        print("[cleanup] done")


if __name__ == "__main__":
    raise SystemExit(main())
