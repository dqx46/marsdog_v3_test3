#!/usr/bin/env python3
"""ENCOS/Incos param probe on the walk shared CAN-A (no official USB-CAN app).

Per ENCOS manual V1.19:
  - Broadcast ID query on 0x7FF: FF FF 00 82
  - Param query on Motor ID: E1 <code>  (mode=7 query)
      23 KP range, 24 KD, 25 POS, 26 SPD, 27 TOR, 31 timeout, 22 Kt, 30 version
  - Can Baud Rate is ONLY in VESC/USB UI — not exposed as a CAN query code.
    We infer bus compatibility: if a motor answers our 1Mbps classical-CAN
    queries, its baud matches the adapter; if it never answers, baud/TX/ID
    may be wrong.

Static: no sine. Dog should be hung.
"""

from __future__ import annotations

import argparse
import struct
import sys
import time
from collections import defaultdict
from pathlib import Path

_HERE = Path(__file__).resolve().parent
_REPO = _HERE.parents[1]
sys.path.insert(0, str(_REPO / "src"))

from marsdog_control.compat import ensure_legacy_path

ensure_legacy_path()

from marsdog_control.hardware.board import RkMotorBoard
from marsdog_control.hardware.motors.can_serial import CAN_EFF_FLAG

# Manual §9.3 query codes
Q_POS = 1
Q_KT = 22
Q_KP = 23
Q_KD = 24
Q_POS_RANGE = 25
Q_SPD = 26
Q_TOR = 27
Q_CUR = 28
Q_TIMEOUT = 31
Q_VERSION = 30

QUERY_NAMES = {
    Q_KP: "KP_range",
    Q_KD: "KD_range",
    Q_POS_RANGE: "POS_range",
    Q_SPD: "SPD_range",
    Q_TOR: "TOR_range",
    Q_CUR: "CUR_range",
    Q_TIMEOUT: "timeout_ms",
    Q_KT: "Kt",
    Q_VERSION: "version",
    Q_POS: "position_deg",
}


def _u16(b: bytes, i: int = 0) -> int:
    return struct.unpack_from(">H", b, i)[0]


def _i16(b: bytes, i: int = 0) -> int:
    return struct.unpack_from(">h", b, i)[0]


def parse_type5(data: bytes):
    if len(data) < 2:
        return None
    if (data[0] >> 5) != 5:
        return None
    code = data[1]
    payload = data[2:]
    err = data[0] & 0x1F
    return code, err, payload


def decode_payload(code: int, payload: bytes):
    try:
        if code in (Q_KP, Q_KD) and len(payload) >= 4:
            return {"min": _u16(payload, 0), "max": _u16(payload, 2)}
        if code == Q_POS_RANGE and len(payload) >= 4:
            return {"min_rad": _i16(payload, 0) / 100.0,
                    "max_rad": _i16(payload, 2) / 100.0}
        if code == Q_SPD and len(payload) >= 4:
            return {"min_rad_s": _i16(payload, 0) / 100.0,
                    "max_rad_s": _i16(payload, 2) / 100.0}
        if code in (Q_TOR, Q_CUR) and len(payload) >= 4:
            scale = 10.0
            unit = "Nm" if code == Q_TOR else "A"
            return {f"min_{unit}": _i16(payload, 0) / scale,
                    f"max_{unit}": _i16(payload, 2) / scale}
        if code == Q_TIMEOUT and len(payload) >= 2:
            return {"ms": _u16(payload, 0)}
        if code == Q_KT and len(payload) >= 2:
            return {"Kt": _u16(payload, 0) / 100.0}
        if code == Q_POS and len(payload) >= 4:
            return {"deg": struct.unpack_from(">f", payload, 0)[0]}
        if code == Q_VERSION and len(payload) >= 2:
            return {"raw": payload.hex()}
    except Exception as exc:
        return {"parse_error": str(exc), "raw": payload.hex()}
    return {"raw": payload.hex()}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--ids", default="3,7", help="motor ids to query")
    ap.add_argument("--retries", type=int, default=5)
    ap.add_argument("--wait", type=float, default=0.08,
                    help="seconds to wait for each reply")
    args = ap.parse_args()
    ids = tuple(int(x) for x in args.ids.split(",") if x.strip())

    board = RkMotorBoard()
    # can_id -> list of raw frames (dlc, data)
    inbox = defaultdict(list)

    def on_std(can_id, dlc, data):
        if can_id & CAN_EFF_FLAG:
            return
        inbox[can_id & 0x7FF].append(bytes(data[:dlc]))

    try:
        print("[init] opening RkMotorBoard (shared CAN-A)...")
        board.start()
        if board.lz is None or board.incos is None:
            print("[FATAL] lz/incos not available")
            return 1
        board.lz.add_can1_standard_handler(on_std)

        # Hold still: zero-gain MIT so we don't fight the joint while querying.
        online = board.online_ids()
        print(f"[init] online motors: {sorted(online)}")
        q0 = board.get_angles(online) if online else {}
        if board.incos is not None:
            for mid in ids:
                if mid in online:
                    board.incos.disable(mid)
        time.sleep(0.05)
        inbox.clear()

        # ---- 1) Broadcast ID query (manual §8.1) ----
        print("\n=== 1) 广播查 CAN ID  (0x7FF / FF FF 00 82) ===")
        found_ids = set()
        for attempt in range(args.retries):
            with board.lz._can1_lock:
                board.lz._can1_serial.send_msg(bytes([0xFF, 0xFF, 0x00, 0x82]), 4, 0x7FF)
            t_end = time.monotonic() + args.wait
            while time.monotonic() < t_end:
                time.sleep(0.002)
            for frame in inbox.get(0x7FF, []):
                # success: FF FF 01 ID_HI ID_LO  (dlc=5)
                if len(frame) >= 5 and frame[0] == 0xFF and frame[1] == 0xFF and frame[2] == 0x01:
                    mid = (frame[3] << 8) | frame[4]
                    found_ids.add(mid)
                    print(f"  [reply] Motor ID = {mid}")
                elif len(frame) >= 4 and frame[0] == 0x80:
                    print(f"  [reply] query failed frame: {frame.hex()}")
                else:
                    print(f"  [reply] 0x7FF raw: {frame.hex()}")
            inbox[0x7FF].clear()
            if found_ids:
                break
            time.sleep(0.05)
        if not found_ids:
            print("  [!] 0x7FF 无成功应答（总线忙/电机不处理广播时也可能发生）")
            print("      回退：用已 online 的 Incos ID 继续查参")
            found_ids = {mid for mid in ids if mid in online}
        print(f"  => 发现的电机 ID: {sorted(found_ids)}")

        # ---- 2) Baud note + per-motor range queries ----
        print("\n=== 2) Can Baud Rate ===")
        print("  手册：波特率只在 VESC/USB「Can Baud Rate」里改，")
        print("  CAN 查询码表(§9.3)没有读波特率的指令。")
        print("  间接结论：能在本机 1Mbps 经典 CAN 上应答 ⇒ 波特率匹配；")
        print("  完全不应答 ⇒ 可能波特率不对 / TX 坏 / ID 不对。")

        print("\n=== 3) 力位混控范围 + 超时 (E1 <code>) ===")
        print(f"  对照驱动假定: KP 0..500, KD 0..5, POS ±12.5rad, "
              f"SPD ±18rad/s, TOR ±12Nm (encode)")
        results = {mid: {} for mid in ids}

        for mid in ids:
            print(f"\n  --- Motor ID {mid} ---")
            if mid not in online and mid not in found_ids:
                print("  [SKIP] 不在 online/广播结果里")
                continue
            for code in (Q_KP, Q_KD, Q_POS_RANGE, Q_SPD, Q_TOR, Q_TIMEOUT, Q_KT, Q_POS):
                name = QUERY_NAMES[code]
                got = None
                for attempt in range(args.retries):
                    inbox[mid].clear()
                    # E1 <code> — same as MotorIncos.query_parameter
                    with board.lz._can1_lock:
                        board.lz._can1_serial.send_msg(
                            bytes([0xE1, code]), 2, mid)
                    t_end = time.monotonic() + args.wait
                    while time.monotonic() < t_end:
                        time.sleep(0.002)
                    for frame in inbox[mid]:
                        parsed = parse_type5(frame)
                        if parsed is None:
                            continue
                        pcode, err, payload = parsed
                        if pcode != code:
                            continue
                        got = decode_payload(code, payload)
                        got["err"] = err
                        got["raw_frame"] = frame.hex()
                        break
                    if got is not None:
                        break
                    time.sleep(0.02)
                results[mid][name] = got
                if got is None:
                    print(f"  {name:12s}: NO REPLY")
                else:
                    show = {k: v for k, v in got.items()
                            if k not in ("raw_frame",)}
                    print(f"  {name:12s}: {show}")

        # ---- 4) Compare 3 vs 7 ----
        print("\n=== 4) ID3 vs ID7 对照 ===")
        if 3 in results and 7 in results:
            keys = ["KP_range", "KD_range", "POS_range", "SPD_range",
                    "TOR_range", "timeout_ms", "Kt"]
            for key in keys:
                a, b = results[3].get(key), results[7].get(key)
                if a is None and b is None:
                    status = "两边都无应答"
                elif a is None:
                    status = "仅 ID3 无应答  <--"
                elif b is None:
                    status = "仅 ID7 无应答  <--"
                else:
                    aa = {k: v for k, v in a.items() if k not in ("err", "raw_frame")}
                    bb = {k: v for k, v in b.items() if k not in ("err", "raw_frame")}
                    status = "一致" if aa == bb else f"不一致  ID3={aa}  ID7={bb}"
                print(f"  {key:12s}: {status}")

        print("\n=== 5) 结论提示 ===")
        r3 = results.get(3, {})
        r7 = results.get(7, {})
        n3 = sum(1 for v in r3.values() if v is not None)
        n7 = sum(1 for v in r7.values() if v is not None)
        print(f"  ID3 查询应答 {n3}/{len(QUERY_NAMES)-1} 项, "
              f"ID7 查询应答 {n7}/{len(QUERY_NAMES)-1} 项")
        if n3 == 0 and n7 > 0:
            print("  → ID3 在问答查询上几乎死寂，和 MIT 稀疏回报同源：")
            print("    电机 TX / ID / 波特率(无法CAN直读) / 该电机本体。")
        elif n3 > 0 and n7 > 0:
            print("  → 两边都能答查询；请看上面范围是否不一致。")
            print("    若范围一致仍 MIT 稀疏，则更像 ID3 控制回报链路异常。")
        print("  波特率无法经 CAN 读出；若需改回 1M，必须用 VESC/官方 APP。")
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
