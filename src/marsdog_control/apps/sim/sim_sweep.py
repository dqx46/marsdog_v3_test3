"""Headless WBC schedule sweeps: multi-vx and turn cases.

Usage::

    PYTHONPATH=src NO_VIEWER=1 python -m marsdog_control.apps.sim.sim_sweep
    PYTHONPATH=src NO_VIEWER=1 python -m marsdog_control.apps.sim.sim_sweep --duration 6

Gates (cruise): roll_peak≤5°, vy_peak≤0.18, mpc/wbc ok≥0.99
Turn cases: roll≤7°, vy≤0.25, |wz| peak reported (info).
"""

from __future__ import annotations

import json
import math
import os
import sys
import time
from dataclasses import dataclass, asdict
from typing import Dict, List, Optional, Tuple

import numpy as np

os.environ.setdefault("NO_VIEWER", "1")

from marsdog_control.apps.sim.sim_walk import (
    DummyMotor,
    SimClock,
    SimImuFake,
    make_controllers,
)
from marsdog_control.backends.sim import SimPhysicsOptions, SimRobotBackend
from marsdog_control.config.joints import JOINT_MAP
from marsdog_control.core.types import RobotMode, UserCommand
from marsdog_control.input.hal import WalkInputHAL
from marsdog_control.input.user_input import InputState, KeyReader
from marsdog_control.runtime.walk_assembly import assemble_walk_loop_context
from marsdog_control.runtime.walk_loop import LoopHardware, tick_walk_loop
from marsdog_control.runtime.walk_state import WalkRuntimeState


@dataclass
class SweepCase:
    name: str
    vx: float
    turn: float
    kind: str  # "cruise" | "turn"


@dataclass
class SweepResult:
    name: str
    vx: float
    turn: float
    kind: str
    roll_peak_deg: float
    roll_rms_deg: float
    vy_peak: float
    wz_peak: float
    fz_peak: float
    dfz_peak: float
    mpc_ok: float
    wbc_ok: float
    amp_cm: float
    period_s: float
    stance: float
    vx_cmd_mean: float
    passed: bool
    reason: str = ""


DEFAULT_CASES: List[SweepCase] = [
    SweepCase("vx0.35", 0.35, 0.0, "cruise"),
    SweepCase("vx0.50", 0.50, 0.0, "cruise"),
    SweepCase("vx0.70", 0.70, 0.0, "cruise"),
    SweepCase("vx1.00", 1.00, 0.0, "cruise"),
    SweepCase("turn0.20", 0.50, 0.20, "turn"),
    SweepCase("turn0.40", 0.50, 0.40, "turn"),
    SweepCase("turn0.60", 0.50, 0.60, "turn"),
]


def _parse_duration(argv: List[str], default: float = 6.0) -> float:
    for i, a in enumerate(argv):
        if a == "--duration" and i + 1 < len(argv):
            try:
                return float(argv[i + 1])
            except ValueError:
                pass
        if a.startswith("--duration="):
            try:
                return float(a.split("=", 1)[1])
            except ValueError:
                pass
    return default


def _build_args():
    from marsdog_control.apps.walk_cli import parse_args

    old = sys.argv
    filtered = []
    i = 1
    while i < len(old):
        a = old[i]
        if a in ("--headless", "--duration") or a.startswith("--duration="):
            if a == "--duration":
                i += 2
            else:
                i += 1
            continue
        if a in ("--natural-soft-trot", "--vmc", "--no-vmc", "--wbc", "--no-wbc"):
            i += 1
            continue
        filtered.append(a)
        i += 1
    sys.argv = [old[0], "--natural-soft-trot", "--wbc", "--no-vmc"] + filtered
    args = parse_args()
    sys.argv = old
    return args


def run_episode(
    *,
    vx: float,
    turn: float,
    duration_s: float = 6.0,
    stand_ticks: int = 200,
    quiet: bool = True,
) -> Tuple[dict, SweepResult]:
    from marsdog_control.runtime.walk_startup import prepare_walk_startup
    from marsdog_control.config.gains import JOINT_GAINS

    args = _build_args()
    runtime_state = WalkRuntimeState()
    startup = prepare_walk_startup(args, runtime_state=runtime_state, joint_gains=JOINT_GAINS)
    runtime_state.dm.fixed_targets[4] = 0.0
    runtime_state.dm.fixed_targets[8] = 0.0

    stack = make_controllers(args, startup)
    stand, fsm, safety, imu_ctrl = stack.stand, stack.fsm, stack.safety, stack.imu_ctrl
    backend = SimRobotBackend(
        stand_controller=stand, physics_options=SimPhysicsOptions()
    )
    clock = SimClock(backend)
    fsm.clock = clock
    fsm.t_gait = clock.time()
    imu = SimImuFake(backend)
    online_ids = set(j.motor_id for j in JOINT_MAP)
    dummy = DummyMotor()
    hw = LoopHardware(
        imu=imu, online=online_ids, lz=dummy, evo=dummy, dm=dummy, incos=dummy
    )
    keyboard = KeyReader()
    input_hal = WalkInputHAL(None, keyboard, InputState(), runtime_state)

    state = {"tick": 0, "amp": 0.0, "period": 1.05, "stance": 0.8}

    def fake_poll(_fsm):
        cmd = UserCommand()
        t = state["tick"]
        if t == stand_ticks:
            cmd.request_mode = RobotMode.NATURAL
            if not quiet:
                print(f"[Sweep] start NATURAL vx={vx:.2f} turn={turn:.2f}")
        if t > stand_ticks:
            cmd.vx = float(vx)
            cmd.turn = float(turn)
            cmd.has_stick = True
            g = _fsm.active_gait
            if g is not None:
                state["amp"] = abs(getattr(g, "amp_front", 0.0))
                state["period"] = float(getattr(g, "period", 1.05))
                state["stance"] = float(getattr(g, "stance_ratio", 0.8))
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
        build_lie_down_target=lambda a, b, c, d, e, f, g: targets.copy(),
        read_positions=lambda a, b, c, d: targets.copy(),
        smooth_transition=lambda a, b, c, d, e: targets.copy(),
        backend=backend,
    )

    keyboard.start()
    max_ticks = stand_ticks + int(duration_s * 200)
    try:
        while ctx.running and state["tick"] < max_ticks:
            if not tick_walk_loop(ctx):
                break
            backend.step()
    finally:
        keyboard.stop()

    tel = {k: list(v) for k, v in ctx.executor.telemetry.items()}
    return tel, _score_episode(
        tel,
        vx=vx,
        turn=turn,
        amp=state["amp"],
        period=state["period"],
        stance=state["stance"],
        name=f"vx{vx:.2f}_t{turn:.2f}",
        kind="turn" if abs(turn) > 1e-6 else "cruise",
    )


def _score_episode(
    tel: dict,
    *,
    vx: float,
    turn: float,
    amp: float,
    period: float,
    stance: float,
    name: str,
    kind: str,
) -> SweepResult:
    roll = np.asarray(tel.get("roll", []), dtype=float)
    vy = np.asarray(tel.get("vy", []), dtype=float)
    wz = np.asarray(tel.get("wz", []), dtype=float)
    vx_cmd = np.asarray(tel.get("vx_cmd", []), dtype=float)
    fc = np.asarray(tel.get("fc_des", []), dtype=float)
    mpc = np.asarray(tel.get("mpc_ok", []), dtype=float)
    wbc = np.asarray(tel.get("wbc_ok", []), dtype=float)
    cs = np.asarray(tel.get("contact_scheduled", []), dtype=float)

    n = len(roll)
    if n == 0 or len(vx_cmd) != n:
        return SweepResult(
            name, vx, turn, kind, 99, 99, 9, 9, 0, 0, 0, 0, amp * 100, period, stance, 0,
            False, "empty telemetry",
        )

    mask = vx_cmd > 0.005
    if not np.any(mask):
        mask = np.ones(n, dtype=bool)

    def pk_rms(x):
        x = x[mask]
        return float(np.max(np.abs(x))), float(np.sqrt(np.mean(x ** 2)))

    r_pk, r_rms = pk_rms(roll)
    vy_pk, _ = pk_rms(vy)
    wz_pk = float(np.max(np.abs(wz[mask]))) if len(wz) == n else 0.0
    fz_peak = 0.0
    dfz_peak = 0.0
    if fc.ndim == 2 and fc.shape[0] > 1:
        fz = fc[:, [2, 5, 8, 11]]
        fz_peak = float(np.max(np.abs(fz)))
        dfz_peak = float(np.max(np.abs(np.diff(fz, axis=0) / 0.005)))

    mpc_ok = float(np.mean(mpc[mask])) if len(mpc) == n else 0.0
    wbc_ok = float(np.mean(wbc[mask])) if len(wbc) == n else 0.0
    vx_mean = float(np.mean(vx_cmd[mask]))

    if kind == "cruise":
        ok = (
            math.degrees(r_pk) <= 5.0
            and vy_pk <= 0.18
            and mpc_ok >= 0.99
            and wbc_ok >= 0.99
        )
        reason = ""
        if math.degrees(r_pk) > 5.0:
            reason = "roll>5"
        elif vy_pk > 0.18:
            reason = "vy>0.18"
        elif mpc_ok < 0.99 or wbc_ok < 0.99:
            reason = "solver"
    else:
        ok = (
            math.degrees(r_pk) <= 7.0
            and vy_pk <= 0.25
            and mpc_ok >= 0.99
            and wbc_ok >= 0.99
        )
        reason = ""
        if math.degrees(r_pk) > 7.0:
            reason = "roll>7"
        elif vy_pk > 0.25:
            reason = "vy>0.25"
        elif mpc_ok < 0.99 or wbc_ok < 0.99:
            reason = "solver"

    return SweepResult(
        name=name,
        vx=vx,
        turn=turn,
        kind=kind,
        roll_peak_deg=math.degrees(r_pk),
        roll_rms_deg=math.degrees(r_rms),
        vy_peak=vy_pk,
        wz_peak=wz_pk,
        fz_peak=fz_peak,
        dfz_peak=dfz_peak,
        mpc_ok=mpc_ok,
        wbc_ok=wbc_ok,
        amp_cm=amp * 100.0,
        period_s=period,
        stance=stance,
        vx_cmd_mean=vx_mean,
        passed=ok,
        reason=reason,
    )


def main():
    duration = _parse_duration(sys.argv, 6.0)
    # Quiet startup spam from prepare_walk_startup
    import contextlib
    import io

    print(f"[Sweep] WBC schedule sweep  duration={duration:.1f}s / case")
    print(
        f"{'case':<12} {'vx':>4} {'turn':>5} {'amp_cm':>6} {'T':>5} {'st':>4} "
        f"{'roll°':>6} {'vy':>5} {'wz':>5} {'vx_cmd':>6} {'ok':>4}"
    )
    results: List[SweepResult] = []
    for case in DEFAULT_CASES:
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            _tel, res = run_episode(
                vx=case.vx, turn=case.turn, duration_s=duration, quiet=True
            )
        res.name = case.name
        res.kind = case.kind
        if case.kind == "cruise":
            res.passed = (
                res.roll_peak_deg <= 5.0
                and res.vy_peak <= 0.18
                and res.mpc_ok >= 0.99
                and res.wbc_ok >= 0.99
            )
            if not res.passed:
                if res.roll_peak_deg > 5.0:
                    res.reason = "roll>5"
                elif res.vy_peak > 0.18:
                    res.reason = "vy>0.18"
                else:
                    res.reason = "solver"
        else:
            res.passed = (
                res.roll_peak_deg <= 7.0
                and res.vy_peak <= 0.25
                and res.mpc_ok >= 0.99
                and res.wbc_ok >= 0.99
            )
            if not res.passed:
                if res.roll_peak_deg > 7.0:
                    res.reason = "roll>7"
                elif res.vy_peak > 0.25:
                    res.reason = "vy>0.25"
                else:
                    res.reason = "solver"
        results.append(res)
        flag = "PASS" if res.passed else f"FAIL:{res.reason}"
        print(
            f"{res.name:<12} {res.vx:4.2f} {res.turn:5.2f} {res.amp_cm:6.2f} "
            f"{res.period_s:5.2f} {res.stance:4.2f} {res.roll_peak_deg:6.2f} "
            f"{res.vy_peak:5.3f} {res.wz_peak:5.3f} {res.vx_cmd_mean:6.4f} {flag}"
        )

    n_pass = sum(1 for r in results if r.passed)
    print(f"\n[Sweep] {n_pass}/{len(results)} passed")
    out = "sweep_results.json"
    with open(out, "w") as f:
        json.dump([asdict(r) for r in results], f, indent=2)
    print(f"[Sweep] wrote {out}")
    if n_pass < len(results):
        sys.exit(1)


if __name__ == "__main__":
    main()
