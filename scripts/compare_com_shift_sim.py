#!/usr/bin/env python3
"""Headless A/B: SoftTrot com_shift ON vs OFF — dump CSV + score correctness.

Usage::

    PYTHONPATH=src NO_VIEWER=1 python scripts/compare_com_shift_sim.py
    PYTHONPATH=src NO_VIEWER=1 python scripts/compare_com_shift_sim.py --duration 8
"""

from __future__ import annotations

import csv
import math
import os
import sys
from pathlib import Path
from typing import Dict, List, Tuple

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


def _parse_duration(default: float = 8.0) -> float:
    argv = sys.argv
    for i, a in enumerate(argv):
        if a == "--duration" and i + 1 < len(argv):
            return float(argv[i + 1])
        if a.startswith("--duration="):
            return float(a.split("=", 1)[1])
    return default


def _build_args(com_shift_m: float):
    from marsdog_control.apps.walk_cli import parse_args

    old = sys.argv
    # Strip script-only flags; force SoftTrot baseline (no WBC).
    filtered = []
    i = 1
    while i < len(old):
        a = old[i]
        if a in ("--headless", "--duration") or a.startswith("--duration="):
            i += 2 if a == "--duration" else 1
            if a == "--duration" and i <= len(old) and not str(old[i - 1]).startswith("--"):
                pass
            continue
        if a in ("--natural-soft-trot", "--vmc", "--no-vmc", "--wbc", "--no-wbc",
                 "--com-shift", "--com-shift-blend"):
            if a in ("--com-shift", "--com-shift-blend"):
                i += 2
            else:
                i += 1
            continue
        if a.startswith("--com-shift"):
            i += 1
            continue
        filtered.append(a)
        i += 1
    sys.argv = [
        old[0], "--natural-soft-trot", "--no-wbc", "--no-vmc",
        "--com-shift", str(com_shift_m),
    ] + filtered
    args = parse_args()
    sys.argv = old
    return args


def run_case(
    *,
    com_shift_m: float,
    vx: float = 0.100,  # SI m/s teleop cruise (SoftTrot mid)
    duration_s: float = 8.0,
    stand_ticks: int = 200,
) -> Tuple[List[dict], dict]:
    from marsdog_control.runtime.walk_startup import prepare_walk_startup

    args = _build_args(com_shift_m)
    runtime_state = WalkRuntimeState()
    startup = prepare_walk_startup(
        args, runtime_state=runtime_state, joint_gains=SIM_JOINT_GAINS)
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
    fsm.drive = fsm.drive.__class__(**{**fsm.drive.__dict__, "cruise_vx": abs(vx)})
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
            cmd.vx = 1.0  # engage; cruise_vx fixed
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
            gait = fsm.active_gait
            t_rel = float(getattr(fsm, "t_gait", clock.time()) - getattr(fsm, "t0", 0.0))
            # Prefer gait-relative time used by get_targets
            try:
                t_gait = float(clock.time() - fsm.t_gait) if hasattr(fsm, "t_gait") else clock.time()
            except Exception:
                t_gait = clock.time()
            # Walk loop uses fsm-relative; match get_targets via gait period phase
            period = float(getattr(gait, "period", 0.58) or 0.58) if gait else 0.58
            com_cmd = 0.0
            if gait is not None and hasattr(gait, "_lateral_offset"):
                # Use same clock the gait sees: WalkLoop uses t_rel from fsm
                # Approximate with sim time since NATURAL start
                t_since = (state["tick"] - stand_ticks) * 0.005
                com_cmd = float(gait._lateral_offset(t_since))
            roll_deg, pitch_deg, yaw_deg = backend.base_rpy
            pos = backend.base_pos
            com = backend.com_xy
            qvel = backend.data.qvel
            vy = float(qvel[backend._base_dof + 1]) if backend._base_dof >= 0 else 0.0
            vx_w = float(qvel[backend._base_dof + 0]) if backend._base_dof >= 0 else 0.0
            phase = (t_since / period) % 1.0 if period > 1e-9 else 0.0
            rows.append({
                "t": t_since,
                "phase": phase,
                "com_cmd_y_m": com_cmd,
                "base_y_m": float(pos[1]),
                "com_y_m": float(com[1]),
                "base_x_m": float(pos[0]),
                "vx": vx_w,
                "vy": vy,
                "roll_deg": float(roll_deg),
                "pitch_deg": float(pitch_deg),
                "yaw_deg": float(yaw_deg),
                "z_m": float(pos[2]),
            })
    finally:
        keyboard.stop()
        backend.close()

    return rows, _score(rows, com_shift_m=com_shift_m, period=period)


def _score(rows: List[dict], *, com_shift_m: float, period: float) -> dict:
    if len(rows) < 50:
        return {"n": len(rows), "error": "too few samples"}
    # Drop first 1s after walk start (ramp)
    ts = np.array([r["t"] for r in rows])
    mask = ts >= 1.0
    R = [r for r, m in zip(rows, mask) if m]
    if len(R) < 50:
        R = rows
    com_cmd = np.array([r["com_cmd_y_m"] for r in R])
    roll = np.array([r["roll_deg"] for r in R])
    pitch = np.array([r["pitch_deg"] for r in R])
    vy = np.array([r["vy"] for r in R])
    vx = np.array([r["vx"] for r in R])
    z = np.array([r["z_m"] for r in R])
    phase = np.array([r["phase"] for r in R])

    # Correctness: commanded sign vs diagonal half (post polarity flip)
    # phase∈[0,0.5) FL+RR → −com; [0.5,1) FR+RL → +com
    expected = np.where(phase < 0.5, -1.0, 1.0)
    if abs(com_shift_m) > 1e-6:
        # Plateau vs diagonal half; negative amp flips expected sign.
        plateau = np.abs(com_cmd) > 0.85 * abs(com_shift_m)
        exp = expected * np.sign(com_shift_m)
        sign_ok = float(np.mean(
            (np.sign(com_cmd[plateau]) == exp[plateau]) if np.any(plateau)
            else [0.0]
        ))
        amp_mean = float(np.mean(np.abs(com_cmd[plateau]))) if np.any(plateau) else 0.0
    else:
        sign_ok = float(np.mean(np.abs(com_cmd) < 1e-6))
        amp_mean = float(np.mean(np.abs(com_cmd)))

    # Body response: roll should anti-correlate with com_cmd
    # (lat_offset>0 body left → feet right → often slight + or - roll; measure corr)
    if np.std(com_cmd) > 1e-6 and np.std(roll) > 1e-6:
        corr_roll = float(np.corrcoef(com_cmd, roll)[0, 1])
    else:
        corr_roll = float("nan")

    # Zero-crossing rate of com_cmd ≈ 2 / period
    zc = np.where(np.diff(np.sign(com_cmd + 1e-12)))[0]
    if len(zc) >= 2:
        dt = float(np.mean(np.diff(ts[mask] if np.sum(mask) == len(com_cmd)
                                   else np.array([r["t"] for r in R])[zc])))
        # better: mean interval between crossings
        t_arr = np.array([r["t"] for r in R])
        zc_dt = float(np.mean(np.diff(t_arr[zc]))) if len(zc) > 1 else float("nan")
        zc_hz = 1.0 / zc_dt if zc_dt > 1e-6 else float("nan")
    else:
        zc_hz = 0.0

    return {
        "n": len(R),
        "com_shift_m": com_shift_m,
        "com_cmd_amp_mean_m": amp_mean,
        "com_cmd_amp_peak_m": float(np.max(np.abs(com_cmd))),
        "com_sign_match": sign_ok,
        "com_zc_hz": zc_hz,
        "com_zc_hz_expected": 2.0 / period if period > 1e-9 else float("nan"),
        "roll_peak_deg": float(np.max(np.abs(roll))),
        "roll_rms_deg": float(np.sqrt(np.mean(roll ** 2))),
        "pitch_peak_deg": float(np.max(np.abs(pitch))),
        "vy_peak": float(np.max(np.abs(vy))),
        "vy_rms": float(np.sqrt(np.mean(vy ** 2))),
        "vx_mean": float(np.mean(vx)),
        "z_mean": float(np.mean(z)),
        "z_std": float(np.std(z)),
        "corr_com_cmd_roll": corr_roll,
    }


def _write_csv(path: Path, rows: List[dict]) -> None:
    if not rows:
        path.write_text("")
        return
    with path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)


def main() -> int:
    duration = _parse_duration(8.0)
    out_dir = _ROOT / "sim_com_shift_compare"
    out_dir.mkdir(exist_ok=True)

    cases = [
        ("off_0mm", 0.0),
        ("pos_12mm", 0.012),
        ("neg_12mm", -0.012),
        ("pos_25mm", 0.025),
        ("neg_25mm", -0.025),
    ]
    scores: Dict[str, dict] = {}
    for name, shift in cases:
        print(f"\n=== Running {name} com_shift={shift} duration={duration}s ===")
        rows, score = run_case(com_shift_m=shift, duration_s=duration)
        csv_path = out_dir / f"com_shift_{name}.csv"
        _write_csv(csv_path, rows)
        scores[name] = score
        print(f"  wrote {csv_path} ({len(rows)} rows)")
        for k, v in score.items():
            if isinstance(v, float):
                print(f"  {k}: {v:.4g}")
            else:
                print(f"  {k}: {v}")

    # Verdict: pick lowest roll_peak / vy among signed cases
    print("\n========== SIGN A/B VERDICT ==========")
    print(f"{'case':<12} {'roll_pk':>8} {'roll_rms':>8} {'vy_pk':>8} {'vy_rms':>8} {'corr':>7}")
    order = ["off_0mm", "pos_12mm", "neg_12mm", "pos_25mm", "neg_25mm"]
    for name in order:
        s = scores[name]
        corr = s.get("corr_com_cmd_roll", float("nan"))
        corr_s = f"{corr:.3f}" if corr == corr else "nan"
        print(
            f"{name:<12} {s.get('roll_peak_deg', 0):8.2f} "
            f"{s.get('roll_rms_deg', 0):8.2f} "
            f"{s.get('vy_peak', 0):8.3f} "
            f"{s.get('vy_rms', 0):8.3f} {corr_s:>7}"
        )

    ranked = sorted(
        order,
        key=lambda n: (
            scores[n].get("roll_peak_deg", 99.0),
            scores[n].get("vy_peak", 99.0),
        ),
    )
    best = ranked[0]
    print(f"\n按 roll_peak→vy_peak 最优: {best}")
    off_r = scores["off_0mm"]["roll_peak_deg"]
    pos_r = scores["pos_12mm"]["roll_peak_deg"]
    neg_r = scores["neg_12mm"]["roll_peak_deg"]
    if neg_r + 0.3 < pos_r and neg_r <= off_r + 0.5:
        print(
            "结论: 负号（符号反相）优于正号；建议默认 com_shift 取负，"
            "或把 trot_weight_shift_sign 整体取反后保持正幅值。"
        )
    elif pos_r + 0.3 < neg_r and pos_r <= off_r + 0.5:
        print("结论: 正号仍优于负号；叠滚问题可能来自幅值/策略而非符号。")
    elif best == "off_0mm":
        print(
            "结论: OFF 仍最优——纯横向移重两边都不稳增益；"
            "需改策略（对角线向的 xy 移重），不是简单翻符号。"
        )
    else:
        print(f"结论: 最优为 {best}；看上表决定是否改默认符号。")
    print(f"CSV 目录: {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
