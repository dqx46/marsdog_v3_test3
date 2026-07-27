#!/usr/bin/env python3
"""On-robot motor tracking bench via RkMotorBoard (same path as walk).

No gait / FSM / IMU. Exercises Board.send_angles → mapping → drivers.

Examples (from repo root):
  PYTHONPATH=src:mocap_to_real python3 tests/Motor_test/bench_motor_track.py \\
      probe --ids 3,7,4,8
  PYTHONPATH=src:mocap_to_real python3 tests/Motor_test/bench_motor_track.py \\
      step --ids 4,8 --deg 3 --kp 40 --sec 2
  PYTHONPATH=src:mocap_to_real python3 tests/Motor_test/bench_motor_track.py \\
      sine --ids 3,7 --amp-deg 5 --period 2 --kp 50 --kd 1 --sec 10
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

from marsdog_control.config.gains import JOINT_GAINS  # noqa: E402
from marsdog_control.config.joints import (  # noqa: E402
    DEFAULT_DM_KD,
    DEFAULT_DM_KP,
    JOINT_BY_ID,
)
from marsdog_control.hardware.actuation import ActuationRuntime  # noqa: E402
from marsdog_control.hardware.board import RkMotorBoard  # noqa: E402

from motor_track_log import (  # noqa: E402
    close_log,
    setup_motor_track_log,
    write_motor_track_rows,
)

DEFAULT_LOG_DIR = _HERE / "log"


def parse_ids(text: str) -> tuple[int, ...]:
    ids = tuple(int(x.strip(), 0) for x in text.split(",") if x.strip())
    if not ids:
        raise argparse.ArgumentTypeError("empty --ids")
    return ids


def validate_ids(ids: tuple[int, ...]) -> None:
    for mid in ids:
        j = JOINT_BY_ID.get(mid)
        if j is None:
            raise SystemExit(f"[FATAL] unknown motor id {mid}")
        if j.bus == "none":
            raise SystemExit(f"[FATAL] motor {mid} ({j.name}) is passive (bus=none)")


def build_actuation_runtime(board: RkMotorBoard, *, dm_active: bool,
                            kp: float, kd: float,
                            use_joint_gains: bool = False,
                            vel_ff: bool = False,
                            dm_kp_fl: float | None = None,
                            dm_kd_fl: float | None = None,
                            dm_kp_fr: float | None = None,
                            dm_kd_fr: float | None = None) -> ActuationRuntime:
    if use_joint_gains:
        fl_kp = float(JOINT_GAINS.get("fl_tarsus", {}).get("kp", kp))
        fl_kd = float(JOINT_GAINS.get("fl_tarsus", {}).get("kd", kd))
        fr_kp = float(JOINT_GAINS.get("fr_tarsus", {}).get("kp", kp))
        fr_kd = float(JOINT_GAINS.get("fr_tarsus", {}).get("kd", kd))
    else:
        fl_kp = fr_kp = kp
        fl_kd = fr_kd = kd
    if dm_kp_fl is not None:
        fl_kp = dm_kp_fl
    if dm_kd_fl is not None:
        fl_kd = dm_kd_fl
    if dm_kp_fr is not None:
        fr_kp = dm_kp_fr
    if dm_kd_fr is not None:
        fr_kd = dm_kd_fr
    return ActuationRuntime(
        dm_tarsus_active=dm_active,
        dm_fixed_targets=dict(board.dm_fixed_targets),
        dm_reference_lead_s={4: 0.0, 8: 0.0},
        dm_reference_lead_max_rad=math.radians(3.0),
        active_dm_kp_by_id={4: fl_kp, 8: fr_kp},
        active_dm_kp=kp,
        active_dm_kd_by_id={4: fl_kd, 8: fr_kd},
        active_dm_kd=kd,
        default_dm_kp=DEFAULT_DM_KP,
        default_dm_kd=DEFAULT_DM_KD,
        dm_dq_max_rps=3.0,
        dm_dq_feedforward=bool(vel_ff or use_joint_gains),
        leg_kp_scale=1.0,
        joint_gains=JOINT_GAINS,
    )


def needs_dm_active(ids: tuple[int, ...]) -> bool:
    return any(JOINT_BY_ID[mid].mtype == "dm" for mid in ids)


def names_for(ids) -> dict[int, str]:
    return {mid: JOINT_BY_ID[mid].name for mid in ids}


def torque_for(board: RkMotorBoard, mid: int) -> float:
    j = JOINT_BY_ID[mid]
    drv = {"lz": board.lz, "evo": board.evo, "dm": board.dm,
           "incos": board.incos}.get(j.mtype)
    if drv is None:
        return float("nan")
    try:
        return float(drv.get_torque(mid))
    except Exception:
        return float("nan")


def send_hold(board: RkMotorBoard, rt: ActuationRuntime, targets: dict,
              *, kp: float, kd: float, velocities: dict | None = None,
              use_joint_gains: bool = False) -> None:
    """Send via the same Board path walk uses.

    DM kp/kd always come from ``rt.active_dm_*`` (filled by
    ``build_actuation_runtime`` from CLI / JOINT_GAINS / --dm-kp-*).
    """
    board.send_angles(
        targets, rt,
        use_joint_gains=use_joint_gains,
        kp_lz=kp, kd_lz=kd,
        kp_evo=kp, kd_evo=kd,
        kp_dm=None, kd_dm=None,
        kp_scale=1.0,
        velocities=velocities or {},
    )


def print_safety(ids: tuple[int, ...]) -> None:
    print("=" * 60)
    print("  Motor_test bench — NO gait; uses RkMotorBoard.send_angles")
    print("  SAFETY: dog should be HUNG / supported; e-stop ready")
    print(f"  Test IDs: {', '.join(str(i) for i in ids)} "
          f"({', '.join(JOINT_BY_ID[i].name for i in ids)})")
    print("  Other online motors: held at boot angle each tick")
    print("=" * 60)


def cmd_probe(board: RkMotorBoard, ids: tuple[int, ...]) -> int:
    online = board.online_ids()
    print(f"[probe] online {len(online)} motors")
    angles = board.get_angles(ids)
    rc = 0
    for mid in ids:
        j = JOINT_BY_ID[mid]
        on = mid in online
        q = angles.get(mid)
        qdeg = math.degrees(q) if q is not None else float("nan")
        flag = "ONLINE" if on else "OFFLINE"
        if not on:
            rc = 1
        print(f"  ID{mid:2d} {j.name:16s} {j.mtype:5s}  {flag:7s}  "
              f"q={qdeg:+8.2f} deg")
    return rc


def _base_targets(board: RkMotorBoard) -> dict[int, float]:
    online = sorted(board.online_ids())
    return board.get_angles(online)


def run_loop(
    board: RkMotorBoard,
    *,
    mode: str,
    test_ids: tuple[int, ...],
    kp: float,
    kd: float,
    hz: float,
    duration_s: float,
    q_cmd_fn,
    log_dir: Path,
    no_log: bool,
    dq_cmd_fn=None,
    use_joint_gains: bool = False,
    dm_kp_fl: float | None = None,
    dm_kd_fl: float | None = None,
    dm_kp_fr: float | None = None,
    dm_kd_fr: float | None = None,
) -> Path | None:
    dm_active = needs_dm_active(test_ids)
    vel_ff = dq_cmd_fn is not None
    rt = build_actuation_runtime(
        board, dm_active=dm_active, kp=kp, kd=kd,
        use_joint_gains=use_joint_gains, vel_ff=vel_ff,
        dm_kp_fl=dm_kp_fl, dm_kd_fl=dm_kd_fl,
        dm_kp_fr=dm_kp_fr, dm_kd_fr=dm_kd_fr)
    q0 = _base_targets(board)
    missing = [mid for mid in test_ids if mid not in q0]
    if missing:
        raise SystemExit(f"[FATAL] test ids not readable/online: {missing}")

    hold = dict(q0)
    online_ids = tuple(sorted(hold.keys()))
    names = names_for(online_ids)
    # Logged kp/kd: uniform CLI values, or JOINT_GAINS / DM overrides.
    kp_map = {mid: kp for mid in online_ids}
    kd_map = {mid: kd for mid in online_ids}
    if use_joint_gains:
        for mid in online_ids:
            j = JOINT_BY_ID.get(mid)
            g = JOINT_GAINS.get(j.name) if j is not None else None
            if g:
                kp_map[mid] = float(g["kp"])
                kd_map[mid] = float(g["kd"])
    for mid, kpv, kdv in (
        (4, rt.active_dm_kp_by_id.get(4), rt.active_dm_kd_by_id.get(4)),
        (8, rt.active_dm_kp_by_id.get(8), rt.active_dm_kd_by_id.get(8)),
    ):
        if mid in kp_map and kpv is not None:
            kp_map[mid] = float(kpv)
            kd_map[mid] = float(kdv)

    fh, writer, path = setup_motor_track_log(
        log_dir, mode=mode, enabled=not no_log)
    if path is not None:
        print(f"[log] {path}")
    print(f"[bench] use_joint_gains={use_joint_gains} "
          f"vel_ff={'on' if vel_ff else 'off'} "
          f"dm_kp=({rt.active_dm_kp_by_id.get(4)},{rt.active_dm_kp_by_id.get(8)}) "
          f"dm_kd=({rt.active_dm_kd_by_id.get(4)},{rt.active_dm_kd_by_id.get(8)})")

    dt = 1.0 / max(1.0, hz)
    t0 = time.monotonic()
    t_wall0 = time.time()
    err_acc = {mid: [] for mid in test_ids}
    step = 0
    try:
        while True:
            now = time.monotonic()
            run_t = now - t0
            if run_t >= duration_s:
                break
            targets = dict(hold)
            velocities = {mid: 0.0 for mid in online_ids}
            for mid in test_ids:
                targets[mid] = q_cmd_fn(mid, run_t, hold[mid])
                if dq_cmd_fn is not None:
                    velocities[mid] = dq_cmd_fn(mid, run_t, hold[mid])
            send_hold(board, rt, targets, kp=kp, kd=kd,
                      velocities=velocities,
                      use_joint_gains=use_joint_gains)
            actual = board.get_angles(test_ids)
            for mid in test_ids:
                if mid in actual:
                    err_acc[mid].append(
                        math.degrees(targets[mid] - actual[mid]))
            torques = {mid: torque_for(board, mid) for mid in test_ids}
            write_motor_track_rows(
                writer,
                t_s=time.time() - t_wall0,
                run_t_s=run_t,
                mode=mode,
                motor_ids=test_ids,
                names=names,
                targets_rad={m: targets[m] for m in test_ids},
                actuals_rad=actual,
                kp_by_id=kp_map,
                kd_by_id=kd_map,
                torque_by_id=torques,
            )
            step += 1
            sleep_t = t0 + (step + 1) * dt - time.monotonic()
            if sleep_t > 0:
                time.sleep(sleep_t)
    except KeyboardInterrupt:
        print("\n[abort] KeyboardInterrupt — returning to q0")
    finally:
        # Return test axes to q0, then soft-disable.
        try:
            for _ in range(int(hz * 0.5)):
                send_hold(board, rt, hold, kp=kp, kd=kd,
                          use_joint_gains=use_joint_gains)
                time.sleep(dt)
            board.soft_disable(
                hold, rt, duration_s=2.0, control_hz=hz)
        except Exception as exc:
            print(f"[cleanup] soft_disable warning: {exc}")
        close_log(fh)

    # Incos/shared-bus: sparse RX looks like huge tracking error even when the
    # shaft moves — print driver counters so we don't chase kp/kd first.
    if board.incos is not None:
        for mid in test_ids:
            j = JOINT_BY_ID.get(mid)
            if j is None or j.mtype != "incos":
                continue
            idx = mid - 1
            rx = int(board.incos.rx_count[idx])
            tx = int(board.incos.tx_count[idx])
            fault = int(board.incos.fault[idx])
            print(f"[incos] ID{mid} {j.name}: tx={tx} rx={rx} "
                  f"rx/tx={rx / tx if tx else 0:.3f} fault={fault}")

    if board.evo is not None:
        for mid in test_ids:
            j = JOINT_BY_ID.get(mid)
            if j is None or j.mtype != "evo":
                continue
            idx = mid - 1
            fault = int(board.evo.fault[idx])
            status = int(board.evo.status[idx])
            temp = float(board.evo.temperature[idx])
            print(f"[evo] ID{mid} {j.name}: fault={fault} status={status} "
                  f"temp={temp:.0f}°C")

    print("[stats] tracking error (test ids only):")
    for mid in test_ids:
        errs = err_acc[mid]
        if not errs:
            print(f"  ID{mid}: no samples")
            continue
        abs_e = [abs(e) for e in errs]
        rms = math.sqrt(sum(e * e for e in errs) / len(errs))
        print(f"  ID{mid} {JOINT_BY_ID[mid].name}: n={len(errs)}  "
              f"RMS={rms:.2f}°  max={max(abs_e):.2f}°  "
              f"kp={kp_map[mid]:.0f} kd={kd_map[mid]:.0f}")
    return path


def _dm_kwargs(args) -> dict:
    return dict(
        dm_kp_fl=getattr(args, "dm_kp_fl", None),
        dm_kd_fl=getattr(args, "dm_kd_fl", None),
        dm_kp_fr=getattr(args, "dm_kp_fr", None),
        dm_kd_fr=getattr(args, "dm_kd_fr", None),
    )


def cmd_hold(board, args) -> int:
    print_safety(args.ids)

    def q_fn(_mid, _t, q0):
        return q0

    run_loop(
        board, mode="hold", test_ids=args.ids, kp=args.kp, kd=args.kd,
        hz=args.hz, duration_s=args.sec, q_cmd_fn=q_fn,
        log_dir=args.log_dir, no_log=args.no_log,
        use_joint_gains=args.use_joint_gains, **_dm_kwargs(args))
    return 0


def cmd_step(board, args) -> int:
    print_safety(args.ids)
    delta = math.radians(args.deg)
    half = max(0.1, args.sec * 0.5)

    def q_fn(_mid, t, q0):
        return q0 + (delta if t < half else 0.0)

    run_loop(
        board, mode="step", test_ids=args.ids, kp=args.kp, kd=args.kd,
        hz=args.hz, duration_s=args.sec, q_cmd_fn=q_fn,
        log_dir=args.log_dir, no_log=args.no_log,
        use_joint_gains=args.use_joint_gains, **_dm_kwargs(args))
    return 0


def cmd_sine(board, args) -> int:
    print_safety(args.ids)
    amp = math.radians(args.amp_deg)
    period = max(0.2, args.period)
    omega = 2.0 * math.pi / period

    def q_fn(_mid, t, q0):
        return q0 + amp * math.sin(omega * t)

    dq_fn = None
    if args.vel_ff:
        def dq_fn(_mid, t, _q0):
            return amp * omega * math.cos(omega * t)

    run_loop(
        board, mode="sine", test_ids=args.ids, kp=args.kp, kd=args.kd,
        hz=args.hz, duration_s=args.sec, q_cmd_fn=q_fn,
        log_dir=args.log_dir, no_log=args.no_log,
        dq_cmd_fn=dq_fn, use_joint_gains=args.use_joint_gains,
        **_dm_kwargs(args))
    return 0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Motor tracking bench via RkMotorBoard (walk driver path)")
    sub = p.add_subparsers(dest="cmd", required=True)

    def add_common(sp, *, need_motion: bool):
        sp.add_argument("--ids", type=parse_ids, required=True,
                        help="comma-separated motor ids, e.g. 3,7,4,8")
        if need_motion:
            sp.add_argument("--kp", type=float, default=40.0)
            sp.add_argument("--kd", type=float, default=2.0)
            sp.add_argument("--hz", type=float, default=200.0)
            sp.add_argument("--sec", type=float, default=5.0)
            sp.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
            sp.add_argument("--no-log", action="store_true")
            sp.add_argument("--use-joint-gains", action="store_true",
                            help="use JOINT_GAINS like walk (ignore uniform "
                                 "--kp/--kd for named joints)")
            sp.add_argument("--vel-ff", action="store_true",
                            help="sine: send analytical dq=Aωcos(ωt) "
                                 "(walk also sends velocity FF)")
            sp.add_argument("--dm-kp-fl", type=float, default=None,
                            help="override fl_tarsus (ID4) kp")
            sp.add_argument("--dm-kd-fl", type=float, default=None,
                            help="override fl_tarsus (ID4) kd")
            sp.add_argument("--dm-kp-fr", type=float, default=None,
                            help="override fr_tarsus (ID8) kp")
            sp.add_argument("--dm-kd-fr", type=float, default=None,
                            help="override fr_tarsus (ID8) kd")

    sp = sub.add_parser("probe", help="print online status and angles")
    add_common(sp, need_motion=False)

    sp = sub.add_parser("hold", help="hold boot angles (enable/feedback check)")
    add_common(sp, need_motion=True)

    sp = sub.add_parser("step", help="relative step then return")
    add_common(sp, need_motion=True)
    sp.add_argument("--deg", type=float, default=3.0,
                    help="step size in degrees relative to q0")

    sp = sub.add_parser("sine", help="relative sine tracking")
    add_common(sp, need_motion=True)
    sp.add_argument("--amp-deg", type=float, default=3.0)
    sp.add_argument("--period", type=float, default=2.0)
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    validate_ids(args.ids)

    print("[init] starting RkMotorBoard (same stack as walk)...")
    board = RkMotorBoard()
    try:
        board.start()
        online = board.online_ids()
        print(f"[init] online {len(online)} motors")
        offline = [mid for mid in args.ids if mid not in online]
        if offline and args.cmd != "probe":
            print(f"[FATAL] test ids offline: {offline}")
            return 1

        if args.cmd == "probe":
            return cmd_probe(board, args.ids)
        if args.cmd == "hold":
            return cmd_hold(board, args)
        if args.cmd == "step":
            return cmd_step(board, args)
        if args.cmd == "sine":
            return cmd_sine(board, args)
        return 1
    finally:
        try:
            board.disable()
        except Exception as exc:
            print(f"[cleanup] disable warning: {exc}")
        try:
            board.close()
        except Exception as exc:
            print(f"[cleanup] close warning: {exc}")
        print("[cleanup] done")


if __name__ == "__main__":
    raise SystemExit(main())
