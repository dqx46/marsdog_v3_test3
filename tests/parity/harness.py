"""Parity harness for the Marsdog decoupling refactor.

Purpose
-------
The decoupling plan physically relocates algorithm code out of the monolithic
``mocap_to_real/walk.py`` into ``src/marsdog_control`` and eventually hands the
main loop to ``RuntimePipeline``. Because the robot cannot be re-validated on
every step, this harness pins the *observable output* of the control math so
any behavior drift during migration is caught offline on a dev machine.

Design
------
- Inputs (gait controllers + FSM) are built through the legacy construction
  path -- those are the deterministic gait/kinematics modules.
- Outputs are produced through the ``src`` boundaries (``MotionPlanner`` and
  ``CommandExecutor``). As those boundaries stop delegating to legacy code in
  later phases, this harness automatically exercises the migrated code and the
  golden snapshot must stay identical.
- Determinism: any clock-dependent path is driven with a patched ``time.time``
  and first-call executor state, so snapshots are reproducible.

This module imports the legacy ``walk`` module, so it installs inert stubs for
POSIX-only modules (``termios``/``tty``/``fcntl``) to run on Windows/CI. Those
stubs are no-ops on the Linux robot and never affect on-target behavior.
"""

from __future__ import annotations

import contextlib
import os
import sys
import time
from typing import Iterator

_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
for _p in (_ROOT, os.path.join(_ROOT, "src")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from marsdog_control.compat import (  # noqa: E402
    ensure_legacy_path,
    install_offline_stubs,
)

ROUND = 6


@contextlib.contextmanager
def _fixed_clock(value: float) -> Iterator[None]:
    """Freeze ``time.time()`` process-wide so gait/executor output is stable.

    Patching the attribute on the ``time`` module affects every ``time.time()``
    caller regardless of which module owns the control math, so the harness is
    robust across the migration.
    """
    original = time.time
    time.time = lambda: value  # type: ignore[assignment]
    try:
        yield
    finally:
        time.time = original  # type: ignore[assignment]


def _round_map(d: dict) -> dict:
    return {int(k): round(float(v), ROUND) for k, v in sorted(d.items())}


def _round_opt_map(d):
    return None if d is None else _round_map(d)


# --- motor-space mapping (parity golden is validated motor-space output) -----
# MotionTarget/ControlOutput now carry pure-URDF (pre-sign) values; the backend
# applies ``urdf * joint.sign`` + motor-limit clamp right before dispatch. The
# committed golden pins the *validated motor-space* command stream, so the
# harness maps planner output through that exact backend mapping before
# snapshotting. This keeps real-robot behavior byte-identical across the URDF
# refactor.
def _to_motor_pos(d: dict) -> dict:
    from marsdog_control.config.joints import JOINT_BY_ID
    out = {}
    for k, v in sorted(d.items()):
        mid = int(k)
        j = JOINT_BY_ID.get(mid)
        if j is None:
            out[mid] = round(float(v), ROUND)
            continue
        m = float(v) * j.sign
        m = max(j.limit_lo, min(j.limit_hi, m))
        out[mid] = round(m, ROUND)
    return out


def _to_motor_signed(d: dict) -> dict:
    """Velocity / torque feed-forward: direction maps with sign, no clamp."""
    from marsdog_control.config.joints import JOINT_BY_ID
    out = {}
    for k, v in sorted(d.items()):
        mid = int(k)
        j = JOINT_BY_ID.get(mid)
        s = j.sign if j is not None else 1
        out[mid] = round(float(v) * s, ROUND)
    return out


def _to_motor_signed_opt(d):
    return None if d is None else _to_motor_signed(d)


def build_context():
    """Build the legacy gait controllers + FSM used as parity inputs."""
    install_offline_stubs()
    ensure_legacy_path()

    import gait_controller  # noqa: E402
    import walk  # noqa: E402
    from gait_recipes import build_controller_set  # noqa: E402
    from runtime_fsm import RobotMode, RuntimeStateMachine  # noqa: E402
    from marsdog_control.config.stack_build import FsmDriveConfig  # noqa: E402

    argv_backup = sys.argv
    sys.argv = ["walk"]
    try:
        args = walk.parse_args()
    finally:
        sys.argv = argv_backup

    controllers = build_controller_set(
        args,
        front_x0=gait_controller._FRONT_X0,
        rear_x0=gait_controller._REAR_X0,
        natural_params=None,
        pace_use_stand_offsets=False,
    )
    stand, trot_fwd, trot_bwd, pace_fwd, pace_bwd, nat_fwd = controllers.as_tuple()

    fsm = RuntimeStateMachine(
        controllers, FsmDriveConfig.from_args(args),
        height=args.height,
        fwd_amp_front=args.amp_front, fwd_amp_rear=args.amp_rear,
        natural_configured=False, start_mode=RobotMode.STAND,
    )

    return {
        "walk": walk,
        "args": args,
        "controllers": controllers,
        "stand": stand,
        "trot_fwd": trot_fwd,
        "trot_bwd": trot_bwd,
        "pace_fwd": pace_fwd,
        "pace_bwd": pace_bwd,
        "nat_fwd": nat_fwd,
        "fsm": fsm,
        "RobotMode": RobotMode,
    }


def run_scenarios() -> dict:
    """Compute the full parity snapshot through the ``src`` boundaries."""
    from marsdog_control.control.executor import CommandExecutor, ExecutorConfig
    from marsdog_control.motion.motion_planner import MotionPlanner

    ctx = build_context()
    fsm = ctx["fsm"]
    stand = ctx["stand"]

    online = set(stand.get_targets(0).keys())
    cur_pos = dict(stand.get_targets(0))
    held = dict(stand.get_targets(0))

    planner = MotionPlanner()
    snapshot: dict = {}

    # --- STAND (deterministic; active_gait is None) --------------------------
    fsm.active_gait = None
    planner.reset_smoothing()
    snapshot["stand_default"] = _to_motor_pos(
        planner.plan(fsm, None, None, None, online, cur_pos).q)

    imu_dz = {"fl": 0.006, "fr": -0.004, "rl": 0.003, "rr": -0.002}
    planner.reset_smoothing()
    snapshot["stand_imu_dz"] = _to_motor_pos(
        planner.plan(fsm, None, imu_dz, None, online, cur_pos).q)

    # --- gait ticks through the planner (patched clock) ----------------------
    for name, gait in (("trot_fwd", ctx["trot_fwd"]),
                       ("pace_fwd", ctx["pace_fwd"])):
        fsm.active_gait = gait
        fsm.t_gait = 0.0
        for t_rel in (0.05, 0.25, 0.5, 0.8):
            planner.reset_smoothing()
            with _fixed_clock(t_rel):
                q = planner.plan(fsm, None, None, None, online, cur_pos).q
            snapshot[f"{name}@{t_rel}"] = _to_motor_pos(q)
    fsm.active_gait = None

    # --- direction-test builders --------------------------------------------
    snapshot["dir_hip_abd"] = _to_motor_pos(
        planner.hip_abduction_test(
            stand, held, online, elapsed_s=0.5, duration_s=1.0).q)
    snapshot["dir_leg_pitch"] = _to_motor_pos(
        planner.leg_pitch_direction_test(
            held, online, amplitude_rad=0.20, elapsed_s=0.5, duration_s=1.0).q)
    snapshot["dir_calf_pitch"] = _to_motor_pos(
        planner.calf_pitch_direction_test(
            held, online, amplitude_rad=0.20, elapsed_s=0.5, duration_s=1.0).q)

    # --- executor: gravity compensation trq_ff (clock-independent) -----------
    grav_exec = CommandExecutor(
        config=ExecutorConfig(gravity_comp=True, variable_impedance=False))
    grav_out = grav_exec.build(None, planner.plan(
        _stand_fsm(fsm), None, None, None, online, cur_pos), fsm)
    snapshot["exec_gravity_trq_ff"] = _to_motor_signed_opt(grav_out.trq_ff)
    snapshot["exec_gravity_velocities"] = _to_motor_signed(grav_out.target.dq)

    # --- executor: variable-impedance kp_phase -------------------------------
    imp_exec = CommandExecutor(
        config=ExecutorConfig(
            gravity_comp=False, variable_impedance=True,
            td_kp_scale=0.4, swing_kp_scale=0.7, td_window=0.15))
    fsm.active_gait = ctx["trot_fwd"]
    fsm.t_gait = 0.0
    with _fixed_clock(0.25):
        imp_out = imp_exec.build(
            None,
            planner.plan(fsm, None, None, None, online, cur_pos),
            fsm, active_gait=ctx["trot_fwd"], t_rel=0.25)
    snapshot["exec_kp_phase"] = _round_opt_map(imp_out.kp_phase)
    fsm.active_gait = None

    return snapshot


def _stand_fsm(fsm):
    fsm.active_gait = None
    return fsm
