#!/usr/bin/env python3
"""Headless SoftTrot impedance tune: score roll / lateral / bounce, pick recipe.

Usage::

    PYTHONPATH=src NO_VIEWER=1 python scripts/tune_soft_trot_impedance.py
"""

from __future__ import annotations

import csv
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

os.environ.setdefault("NO_VIEWER", "1")

_ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(_ROOT / "src"), str(_ROOT)]

from marsdog_control.apps.sim.sim_walk import (  # noqa: E402
    DummyMotor,
    SimClock,
    SimImuFake,
    make_controllers,
)
from marsdog_control.backends.sim import SimPhysicsOptions, SimRobotBackend  # noqa: E402
from marsdog_control.config.gains import SIM_JOINT_GAINS  # noqa: E402
from marsdog_control.config.joints import JOINT_MAP  # noqa: E402
from marsdog_control.core.types import RobotMode, UserCommand  # noqa: E402
from marsdog_control.input.hal import WalkInputHAL  # noqa: E402
from marsdog_control.input.user_input import InputState, KeyReader  # noqa: E402
from marsdog_control.runtime.walk_assembly import assemble_walk_loop_context  # noqa: E402
from marsdog_control.runtime.walk_loop import LoopHardware, tick_walk_loop  # noqa: E402
from marsdog_control.runtime.walk_state import WalkRuntimeState  # noqa: E402


@dataclass(frozen=True)
class Case:
    name: str
    period: float
    stance: float
    step_h: float
    step_h_front: float
    amp_front: float
    amp_rear: float
    com_shift_m: float
    touchdown_compress: float = 0.003
    vx: float = 0.10


def _build_args(case: Case):
    from marsdog_control.apps.walk_cli import parse_args

    old = sys.argv
    sys.argv = [
        old[0],
        "--natural-soft-trot",
        "--no-wbc",
        "--no-vmc",
        "--gait-period",
        f"{case.period:.4g}",
        "--stance",
        f"{case.stance:.4g}",
        "--step-h",
        f"{case.step_h:.4g}",
        "--step-h-front",
        f"{case.step_h_front:.4g}",
        "--fwd-front-lift",
        f"{case.step_h_front:.4g}",
        "--amp-front",
        f"{case.amp_front:.4g}",
        "--amp-rear",
        f"{case.amp_rear:.4g}",
        "--nat-amp-front",
        f"{case.amp_front:.4g}",
        "--nat-amp-rear",
        f"{case.amp_rear:.4g}",
        "--nat-step-h",
        f"{case.step_h:.4g}",
        "--com-shift",
        f"{case.com_shift_m:.4g}",
        "--cruise-vx",
        f"{case.vx:.4g}",
    ]
    args = parse_args()
    sys.argv = old
    # touchdown not always CLI; patch after parse if present
    if hasattr(args, "touchdown_compress"):
        args.touchdown_compress = case.touchdown_compress
    return args


def run_case(case: Case, *, duration_s: float = 7.0, stand_ticks: int = 160) -> dict:
    from marsdog_control.runtime.walk_startup import prepare_walk_startup

    args = _build_args(case)
    runtime_state = WalkRuntimeState()
    startup = prepare_walk_startup(
        args, runtime_state=runtime_state, joint_gains=SIM_JOINT_GAINS
    )
    runtime_state.dm.fixed_targets[4] = 0.0
    runtime_state.dm.fixed_targets[8] = 0.0

    stack = make_controllers(args, startup)
    stand, fsm, safety, imu_ctrl = (
        stack.stand,
        stack.fsm,
        stack.safety,
        stack.imu_ctrl,
    )
    # Soft landing if controller exposes it
    if hasattr(stack.fsm.nat_fwd, "touchdown_compress"):
        stack.fsm.nat_fwd.touchdown_compress = case.touchdown_compress

    backend = SimRobotBackend(
        stand_controller=stand, physics_options=SimPhysicsOptions()
    )
    clock = SimClock(backend)
    fsm.clock = clock
    fsm.t_gait = clock.time()
    fsm.drive = fsm.drive.__class__(
        **{**fsm.drive.__dict__, "cruise_vx": abs(case.vx)}
    )
    imu = SimImuFake(backend)
    online_ids = set(j.motor_id for j in JOINT_MAP)
    dummy = DummyMotor()
    hw = LoopHardware(
        imu=imu, online=online_ids, lz=dummy, evo=dummy, dm=dummy, incos=dummy
    )
    keyboard = KeyReader()
    input_hal = WalkInputHAL(None, keyboard, InputState(), runtime_state)
    state = {"tick": 0}

    def fake_poll(_fsm):
        cmd = UserCommand()
        t = state["tick"]
        if t == stand_ticks:
            cmd.request_mode = RobotMode.NATURAL
        if t > stand_ticks:
            cmd.vx = 1.0
            cmd.turn = 0.0
            cmd.has_stick = True
        state["tick"] += 1
        return cmd, None

    input_hal.poll = fake_poll
    targets = dict(stand.get_targets(0.0))
    ctx = assemble_walk_loop_context(
        startup=startup,
        runtime_state=runtime_state,
        hw=hw,
        fsm=fsm,
        input_hal=input_hal,
        stand=stand,
        safety=safety,
        imu_ctrl=imu_ctrl,
        targets=targets,
        cur_pos=targets.copy(),
        smooth_tgt={},
        real_joints=[j for j in JOINT_MAP],
        joint_map=JOINT_MAP,
        direction_test_base=targets.copy(),
        direction_test_start=0.0,
        control_hz=200.0,
        clock=clock,
        write_log=lambda x: None,
        log_writer=None,
        bark_with_mouth=lambda: None,
        build_lie_down_target=lambda *a, **k: targets.copy(),
        read_positions=lambda *a, **k: targets.copy(),
        smooth_transition=lambda *a, **k: targets.copy(),
        backend=backend,
    )

    rows: List[dict] = []
    keyboard.start()
    max_ticks = stand_ticks + int(duration_s * 200)
    try:
        while ctx.running and state["tick"] < max_ticks:
            if not tick_walk_loop(ctx):
                break
            backend.step()
            if state["tick"] <= stand_ticks:
                continue
            t_since = (state["tick"] - stand_ticks) * 0.005
            roll_deg, pitch_deg, _ = backend.base_rpy
            pos = backend.base_pos
            qvel = backend.data.qvel
            base = backend._base_dof
            vx = float(qvel[base + 0]) if base >= 0 else 0.0
            vy = float(qvel[base + 1]) if base >= 0 else 0.0
            vz = float(qvel[base + 2]) if base >= 0 else 0.0
            gait = fsm.active_gait
            com_cmd = 0.0
            if gait is not None and hasattr(gait, "_lateral_offset"):
                com_cmd = float(gait._lateral_offset(t_since))
            rows.append(
                {
                    "t": t_since,
                    "roll": roll_deg,
                    "pitch": pitch_deg,
                    "z": float(pos[2]),
                    "vx": vx,
                    "vy": vy,
                    "vz": vz,
                    "com_cmd": com_cmd,
                }
            )
    finally:
        keyboard.stop()
        backend.close()

    return score_rows(rows, case)


def score_rows(rows: List[dict], case: Case) -> dict:
    if len(rows) < 80:
        return {"name": case.name, "error": "too_few", "score": 1e9}
    t = np.array([r["t"] for r in rows])
    m = t >= 1.2
    R = [r for r, ok in zip(rows, m) if ok] or rows
    roll = np.array([r["roll"] for r in R])
    pitch = np.array([r["pitch"] for r in R])
    vy = np.array([r["vy"] for r in R])
    vz = np.array([r["vz"] for r in R])
    vx = np.array([r["vx"] for r in R])
    z = np.array([r["z"] for r in R])
    com = np.array([r["com_cmd"] for r in R])

    roll_rms = float(np.sqrt(np.mean(roll**2)))
    roll_pk = float(np.max(np.abs(roll)))
    pitch_pk = float(np.max(np.abs(pitch)))
    vy_rms = float(np.sqrt(np.mean(vy**2)))
    vy_pk = float(np.max(np.abs(vy)))
    vz_rms = float(np.sqrt(np.mean(vz**2)))
    vz_pk = float(np.max(np.abs(vz)))
    z_std = float(np.std(z))
    vx_mean = float(np.mean(vx))
    vx_err = abs(vx_mean - case.vx)
    com_pk = float(np.max(np.abs(com)))

    # Weighted cost: stability + looks (vy) + bounce (vz/z) + speed tracking.
    # Prefer real-friendly cadence mildly (period away from 0.58).
    cadence_pen = 0.0
    if case.period < 0.70:
        cadence_pen = (0.70 - case.period) * 8.0  # discourage too-fast for real
    score = (
        1.20 * roll_rms
        + 0.55 * roll_pk
        + 18.0 * vy_rms
        + 6.0 * vy_pk
        + 25.0 * vz_rms
        + 8.0 * vz_pk
        + 80.0 * z_std
        + 4.0 * vx_err
        + 0.35 * pitch_pk
        + cadence_pen
    )
    return {
        "name": case.name,
        "period": case.period,
        "stance": case.stance,
        "step_h": case.step_h,
        "step_h_front": case.step_h_front,
        "amp_front": case.amp_front,
        "amp_rear": case.amp_rear,
        "com_shift_m": case.com_shift_m,
        "roll_rms": roll_rms,
        "roll_pk": roll_pk,
        "pitch_pk": pitch_pk,
        "vy_rms": vy_rms,
        "vy_pk": vy_pk,
        "vz_rms": vz_rms,
        "vz_pk": vz_pk,
        "z_std": z_std,
        "vx_mean": vx_mean,
        "vx_err": vx_err,
        "com_pk": com_pk,
        "score": score,
        "n": len(R),
    }


def build_grid() -> List[Case]:
    cases: List[Case] = []
    # Baselines
    cases.append(
        Case(
            "old_fast_25mm",
            period=0.58,
            stance=0.56,
            step_h=0.048,
            step_h_front=0.045,
            amp_front=0.050,
            amp_rear=0.068,
            com_shift_m=0.025,
            touchdown_compress=0.0,
        )
    )
    cases.append(
        Case(
            "curr_ugly_10mm",
            period=0.88,
            stance=0.66,
            step_h=0.032,
            step_h_front=0.028,
            amp_front=0.048,
            amp_rear=0.062,
            com_shift_m=0.010,
            touchdown_compress=0.003,
        )
    )

    # Stage: real-friendly cadence × stance × lift × com
    for period in (0.72, 0.78, 0.84):
        for stance in (0.60, 0.64, 0.68):
            for step_h, step_hf in (
                (0.034, 0.030),
                (0.038, 0.034),
                (0.042, 0.038),
            ):
                for com in (0.0, 0.008, 0.012, 0.016, 0.020):
                    # Amp scaled lightly with period so kinematic vx≈0.10 mid-cruise
                    # vx≈2*avg_amp/T → avg_amp≈0.05*T/1 → keep rear-drive bias
                    scale = period / 0.78
                    af = round(0.042 * scale, 4)
                    ar = round(0.056 * scale, 4)
                    name = (
                        f"T{period:.2f}_st{stance:.2f}_h{step_h*100:.0f}"
                        f"_c{com*1000:.0f}"
                    )
                    cases.append(
                        Case(
                            name,
                            period=period,
                            stance=stance,
                            step_h=step_h,
                            step_h_front=step_hf,
                            amp_front=af,
                            amp_rear=ar,
                            com_shift_m=com,
                            touchdown_compress=0.003,
                        )
                    )
    return cases


def main() -> int:
    duration = 7.0
    for i, a in enumerate(sys.argv):
        if a == "--duration" and i + 1 < len(sys.argv):
            duration = float(sys.argv[i + 1])
        if a == "--quick":
            # smaller grid for smoke
            pass

    cases = build_grid()
    if "--quick" in sys.argv:
        cases = [
            c
            for c in cases
            if c.name in ("old_fast_25mm", "curr_ugly_10mm")
            or (
                c.period in (0.78, 0.84)
                and c.stance in (0.64,)
                and c.step_h in (0.038,)
                and c.com_shift_m in (0.0, 0.012, 0.016)
            )
        ]

    out_dir = _ROOT / "sim_soft_trot_tune"
    out_dir.mkdir(exist_ok=True)
    results: List[dict] = []
    print(f"Running {len(cases)} cases, duration={duration}s ...")
    for i, case in enumerate(cases, 1):
        print(f"[{i}/{len(cases)}] {case.name}", flush=True)
        try:
            s = run_case(case, duration_s=duration)
        except Exception as e:
            s = {"name": case.name, "error": str(e), "score": 1e9}
            print(f"  FAIL {e}")
        results.append(s)
        if "error" not in s:
            print(
                f"  score={s['score']:.2f} roll={s['roll_rms']:.1f}/{s['roll_pk']:.1f} "
                f"vy={s['vy_rms']:.3f}/{s['vy_pk']:.3f} "
                f"vz={s['vz_rms']:.3f}/{s['vz_pk']:.3f} "
                f"zstd={s['z_std']*1000:.1f}mm vx={s['vx_mean']:.3f}"
            )

    results_ok = [r for r in results if "error" not in r]
    results_ok.sort(key=lambda r: r["score"])
    csv_path = out_dir / "tune_results.csv"
    if results_ok:
        with csv_path.open("w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(results_ok[0].keys()))
            w.writeheader()
            w.writerows(results_ok)

    print("\n========== TOP 12 ==========")
    hdr = (
        f"{'name':<28} {'score':>7} {'roll_rms':>8} {'roll_pk':>7} "
        f"{'vy_rms':>7} {'vz_rms':>7} {'z_mm':>6} {'vx':>6} {'com':>5}"
    )
    print(hdr)
    for s in results_ok[:12]:
        print(
            f"{s['name']:<28} {s['score']:7.2f} {s['roll_rms']:8.2f} "
            f"{s['roll_pk']:7.2f} {s['vy_rms']:7.3f} {s['vz_rms']:7.3f} "
            f"{s['z_std']*1000:6.1f} {s['vx_mean']:6.3f} "
            f"{s['com_shift_m']*1000:5.0f}"
        )

    # Also print baselines
    print("\n========== BASELINES ==========")
    for name in ("old_fast_25mm", "curr_ugly_10mm"):
        for s in results_ok:
            if s["name"] == name:
                print(
                    f"{name}: score={s['score']:.2f} roll_pk={s['roll_pk']:.1f} "
                    f"vy_pk={s['vy_pk']:.3f} vz_pk={s['vz_pk']:.3f}"
                )

    if results_ok:
        best = results_ok[0]
        print("\nBEST → apply to NATURAL_SOFT_TROT_WBC:")
        for k in (
            "period",
            "stance",
            "step_h",
            "step_h_front",
            "amp_front",
            "amp_rear",
            "com_shift_m",
        ):
            print(f"  {k}: {best[k]}")
        print(f"CSV: {csv_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
