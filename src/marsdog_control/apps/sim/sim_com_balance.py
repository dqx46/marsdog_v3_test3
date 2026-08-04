"""CoM balance test — WASD shift body with feet planted; Q/E lift FL–RR.

Simulation (default)::

    PYTHONPATH=src python -m marsdog_control.apps.sim.sim_com_balance
    PYTHONPATH=src python -m marsdog_control.apps.sim.sim_com_balance --headless --duration 2

Real robot::

    PYTHONPATH=src python -m marsdog_control.apps.sim.sim_com_balance --real
    PYTHONPATH=src python -m marsdog_control.apps.sim.sim_com_balance --real --allow-lift
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time
from typing import Dict, Optional


def _prefer_x11_for_mujoco_viewer() -> None:
    """Same Wayland HiDPI workaround as sim_walk."""
    if os.environ.get("MARDOG_ALLOW_WAYLAND") == "1":
        return
    if os.environ.get("PYGLFW_LIBRARY"):
        return
    try:
        import importlib.util

        spec = importlib.util.find_spec("glfw")
        if spec is None or not spec.submodule_search_locations:
            return
        root = spec.submodule_search_locations[0]
        x11_so = os.path.join(root, "x11", "libglfw.so")
        if os.path.isfile(x11_so):
            os.environ["PYGLFW_LIBRARY"] = x11_so
            os.environ.setdefault("GDK_BACKEND", "x11")
            os.environ.setdefault("QT_QPA_PLATFORM", "xcb")
    except Exception:
        return


_prefer_x11_for_mujoco_viewer()

try:
    if os.environ.get("NO_VIEWER") == "1":
        HAS_VIEWER = False
    else:
        import mujoco.viewer

        HAS_VIEWER = True
except ImportError:
    HAS_VIEWER = False

from marsdog_control.apps.sim.sim_walk import make_controllers  # noqa: E402
from marsdog_control.config.gains import JOINT_GAINS, SIM_JOINT_GAINS  # noqa: E402
from marsdog_control.config.joints import JOINT_MAP  # noqa: E402
from marsdog_control.control.executor import gravity_trq  # noqa: E402
from marsdog_control.core.types import ControlOutput, MotionTarget, RobotMode  # noqa: E402
from marsdog_control.input.user_input import KeyReader  # noqa: E402
from marsdog_control.motion.balance_stand import (  # noqa: E402
    BalanceStandPlanner,
    pinocchio_com_in_base,
)
from marsdog_control.runtime.walk_startup import prepare_walk_startup  # noqa: E402
from marsdog_control.runtime.walk_state import WalkRuntimeState  # noqa: E402

CONTROL_HZ = 200.0
CONTROL_DT = 1.0 / CONTROL_HZ

_TOOL_FLAGS = (
    "--headless",
    "--smoke-keys",
    "--real",
    "--allow-lift",
    "--no-log-joints",
    "--duration",
    "--com-step",
    "--lift-z",
    "--log-hz",
    "--max-tilt-deg",
    "--fade-s",
)

# 日志里打印的主动关节（跳过未接线 tarsus）
_LOG_JOINT_IDS = tuple(j.motor_id for j in JOINT_MAP if j.bus != "none")

_HELP = """
Keys (this tool only — not walk hotkeys):
  W/S  com_x ± step (forward / back)
  A/D  com_y ± step (left / right)
  Q    lift FL+RR diagonal  (real: needs --allow-lift)
  E    plant FL+RR (quad support)
  R    reset com_x/y to 0
  X or Ctrl+C  quit
"""


def _build_args():
    from marsdog_control.apps.walk_cli import parse_args

    old = sys.argv
    filtered: list[str] = []
    skip_next = False
    for a in old[1:]:
        if skip_next:
            skip_next = False
            continue
        if a in _TOOL_FLAGS or a.startswith(
            ("--duration=", "--com-step=", "--lift-z=", "--log-hz=",
             "--max-tilt-deg=", "--fade-s=")
        ):
            if a in (
                "--duration",
                "--com-step",
                "--lift-z",
                "--log-hz",
                "--max-tilt-deg",
                "--fade-s",
            ):
                skip_next = True
            continue
        if a in (
            "--natural-soft-trot",
            "--vmc",
            "--no-vmc",
            "--wbc",
            "--no-wbc",
        ):
            continue
        filtered.append(a)
    sys.argv = [
        old[0],
        "--natural-soft-trot",
        "--no-wbc",
        "--no-vmc",
    ] + filtered
    args = parse_args()
    sys.argv = old
    return args


def _parse_tool_flags(argv: list[str]):
    p = argparse.ArgumentParser(
        description="CoM balance: WASD shift body, Q/E lift FL–RR (sim or --real)",
    )
    p.add_argument("--headless", action="store_true")
    p.add_argument(
        "--real",
        action="store_true",
        help="Use real robot (walk bring-up + RealRobotBackend)",
    )
    p.add_argument(
        "--allow-lift",
        action="store_true",
        help="Allow Q diagonal lift on --real (disabled by default for safety)",
    )
    p.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Auto-stop after N seconds (0 = until quit; sim=sim-time, real=wall)",
    )
    p.add_argument("--com-step", type=float, default=0.005, help="WASD step (m)")
    p.add_argument("--lift-z", type=float, default=0.025, help="FL–RR lift height (m)")
    p.add_argument("--log-hz", type=float, default=5.0, help="Status print rate")
    p.add_argument(
        "--no-log-joints",
        action="store_true",
        help="Do not print joint angles (tgt/fb deg) with status lines",
    )
    p.add_argument(
        "--max-tilt-deg",
        type=float,
        default=20.0,
        help="Abort if |roll| or |pitch| exceeds this (real; 0=disable)",
    )
    p.add_argument(
        "--fade-s",
        type=float,
        default=3.0,
        help="Real: fade-to-stand duration (s)",
    )
    p.add_argument(
        "--smoke-keys",
        action="store_true",
        help="Headless sim: replay W/A/Q/E/R once to exercise key path",
    )
    return p.parse_known_args(argv)


def _handle_key(
    key: str | None,
    planner: BalanceStandPlanner,
    step: float,
    *,
    allow_lift: bool,
    lift_blocked_msg: list,
) -> bool:
    """Update planner from one key. Return False to quit."""
    if key is None:
        return True
    if key in ("x", "X", "\x03"):
        return False
    if key in ("w", "W"):
        planner.nudge_com(dx=+step)
    elif key in ("s", "S"):
        planner.nudge_com(dx=-step)
    elif key in ("a", "A"):
        planner.nudge_com(dy=+step)
    elif key in ("d", "D"):
        planner.nudge_com(dy=-step)
    elif key in ("q", "Q"):
        if not allow_lift:
            if not lift_blocked_msg:
                print("[CoMBal] Q ignored on --real without --allow-lift")
                lift_blocked_msg.append(True)
        else:
            planner.lift_diag()
    elif key in ("e", "E"):
        planner.plant_diag()
    elif key in ("r", "R"):
        planner.reset_com()
    return True


def _control_output(
    targets: Dict[int, float],
    trq_ff: Optional[Dict[int, float]],
) -> ControlOutput:
    return ControlOutput(
        target=MotionTarget(q=dict(targets), source_mode=RobotMode.STAND),
        trq_ff=dict(trq_ff) if trq_ff else {},
        control_period_s=CONTROL_DT,
        gait_active=False,
        dm_active=False,
    )


def _format_joint_deg_line(
    label: str,
    q: Dict[int, float],
    *,
    ids: tuple = _LOG_JOINT_IDS,
) -> str:
    """One compact line: ``[q] tgt(deg) fl_hip=+45.6 ...``."""
    from marsdog_control.config.joints import JOINT_BY_ID

    parts = []
    for mid in ids:
        j = JOINT_BY_ID.get(mid)
        if j is None or mid not in q:
            continue
        name = j.name
        for prefix in ("fl_", "fr_", "rl_", "rr_"):
            if name.startswith(prefix):
                name = prefix[:2] + "_" + name[len(prefix):]
                break
        # shorten common suffixes
        name = (
            name.replace("hip_pitch", "hip")
            .replace("thigh_roll", "roll")
            .replace("waist_pitch", "w_p")
            .replace("waist_yaw", "w_y")
            .replace("waist_roll", "w_r")
            .replace("neck_pitch", "neck")
            .replace("head_pitch", "h_p")
            .replace("head_yaw", "h_y")
            .replace("head_roll", "h_r")
        )
        parts.append(f"{name}={math.degrees(q[mid]):+.1f}")
    return f"[q] {label}(deg) " + " ".join(parts)


def _print_joint_angles(
    targets: Dict[int, float],
    measured: Optional[Dict[int, float]] = None,
) -> None:
    print(_format_joint_deg_line("tgt", targets))
    if measured:
        print(_format_joint_deg_line("fb ", measured))


def _bootstrap_stand(tool):
    """Shared SoftTrot stand + planner (sim or real gains).

    Caller must set ``sys.argv`` to process-name + walk_cli remainder before call.
    """
    gains = JOINT_GAINS if tool.real else SIM_JOINT_GAINS
    args = _build_args()

    runtime_state = WalkRuntimeState(joint_gains=gains)
    startup = prepare_walk_startup(
        args, runtime_state=runtime_state, joint_gains=gains
    )
    if startup is None:
        return None
    runtime_state.dm.fixed_targets.setdefault(4, 0.0)
    runtime_state.dm.fixed_targets.setdefault(8, 0.0)

    stack = make_controllers(args, startup)
    stand = stack.stand
    grav_scale = float(getattr(runtime_state, "gravity_scale", 1.0) or 1.0)
    gravity_on = bool(getattr(runtime_state, "gravity_comp", True))
    planner = BalanceStandPlanner(stand, lift_z_m=tool.lift_z, com_x_m=0.040)
    pin_com = pinocchio_com_in_base(stand)

    print(
        f"[CoMBal] mode={'REAL' if tool.real else 'SIM'}  "
        f"SoftTrot stand + MIT + gravity_comp={gravity_on} scale={grav_scale:.2f}"
    )
    print(
        f"[CoMBal] stand H={stand.body_height:.3f}  "
        f"x_off F/R={stand.x_offset_front:.4f}/{stand.x_offset_rear:.4f}  "
        f"abd={stand.hip_abduction:.3f}  com_step={tool.com_step:.4f}m"
    )
    if pin_com is not None:
        print(
            f"[CoMBal] Pinocchio CoM @ stand (base≈origin): "
            f"({pin_com[0]:+.4f}, {pin_com[1]:+.4f}, {pin_com[2]:+.4f}) m"
        )
    if tool.real and not tool.allow_lift:
        print("[CoMBal] real safety: Q lift disabled (pass --allow-lift to enable)")
    print(_HELP.strip())
    return {
        "args": args,
        "runtime_state": runtime_state,
        "startup": startup,
        "stand": stand,
        "planner": planner,
        "grav_scale": grav_scale,
        "gravity_on": gravity_on,
    }


def _run_sim(tool, ctx) -> int:
    from marsdog_control.backends.sim import SimPhysicsOptions, SimRobotBackend

    stand = ctx["stand"]
    planner = ctx["planner"]
    grav_scale = ctx["grav_scale"]
    gravity_on = ctx["gravity_on"]
    allow_lift = True  # sim always allows Q
    lift_blocked_msg: list = []

    is_headless = bool(tool.headless) or os.environ.get("NO_VIEWER") == "1"
    if is_headless:
        os.environ["NO_VIEWER"] = "1"

    backend = SimRobotBackend(
        stand_controller=stand, physics_options=SimPhysicsOptions()
    )
    backend.set_xy_hold_damp(150.0)

    keyboard = KeyReader()
    keyboard.start()
    log_every = max(1, int(CONTROL_HZ / max(0.1, tool.log_hz)))
    max_ticks = int(tool.duration * CONTROL_HZ) if tool.duration > 0 else 0
    tick = 0
    wall0 = time.time()
    sim0 = float(backend.sim_time)
    running = True
    smoke_at = (
        {80: "w", 120: "a", 160: "q", 220: "e", 280: "r"} if tool.smoke_keys else {}
    )

    log_joints = not bool(tool.no_log_joints)

    def _log(targets: Optional[Dict[int, float]] = None) -> None:
        contacts = sorted(backend.foot_contacts())
        com = backend.com_xy
        pos = backend.base_pos
        print(
            f"[CoMBal] {planner.describe()}  "
            f"mujoco_com=({com[0]:+.4f},{com[1]:+.4f})  "
            f"base=({pos[0]:+.4f},{pos[1]:+.4f},z={pos[2]:.3f})  "
            f"contact={contacts or '-'}"
        )
        if log_joints:
            tgt = targets if targets is not None else planner.get_targets()
            fb = None
            try:
                st = backend.read_state(set(_LOG_JOINT_IDS))
                fb = st.joint_pos
            except Exception:
                fb = None
            _print_joint_angles(tgt, fb)

    def _tick_once() -> bool:
        nonlocal tick, running
        while True:
            key = keyboard.get()
            if key is None:
                break
            if not _handle_key(
                key, planner, tool.com_step,
                allow_lift=allow_lift, lift_blocked_msg=lift_blocked_msg,
            ):
                running = False
                return False
        if tick in smoke_at:
            k = smoke_at[tick]
            print(f"[CoMBal] smoke-key '{k}'")
            _handle_key(
                k, planner, tool.com_step,
                allow_lift=allow_lift, lift_blocked_msg=lift_blocked_msg,
            )
        targets = planner.get_targets()
        trq = gravity_trq(targets, grav_scale) if gravity_on else {}
        backend.send(_control_output(targets, trq))
        backend.step()
        tick += 1
        if tick % log_every == 0:
            _log(targets)
        if max_ticks and tick >= max_ticks:
            print(f"[CoMBal] duration reached ({tool.duration:.1f}s)")
            running = False
            return False
        return True

    try:
        if HAS_VIEWER and not is_headless:
            with mujoco.viewer.launch_passive(backend.model, backend.data) as viewer:
                while viewer.is_running() and running:
                    if not _tick_once():
                        break
                    if tick % 4 == 0:
                        pos = backend.base_pos
                        viewer.cam.lookat[0] = pos[0]
                        viewer.cam.lookat[1] = pos[1]
                        viewer.cam.lookat[2] = 0.15
                        viewer.sync()
                    target_wall = wall0 + (float(backend.sim_time) - sim0)
                    sleep_t = target_wall - time.time()
                    if sleep_t > 0:
                        time.sleep(sleep_t)
        else:
            while running:
                if not _tick_once():
                    break
    except KeyboardInterrupt:
        print("\n[CoMBal] interrupted")
    finally:
        keyboard.stop()
        _log(planner.get_targets())
        print(
            f"[CoMBal] done  ticks={tick}  final {planner.describe()}  "
            f"(record this com_x/y if still balanced)"
        )
    return 0


def _run_real(tool, ctx) -> int:
    from marsdog_control.backends.real import RealRobotBackend
    from marsdog_control.compat import legacy_dir
    from marsdog_control.config.bus_config import (
        BAUD,
        DM_CAN_DEVICE,
        EVO_CAN0_DEVICE,
        IMU_BAUD,
        IMU_DEVICE,
        INCOS_CAN_DEVICE,
        LZ_CAN1_DEVICE,
        LZ_SERIAL_DEVICE,
    )
    from marsdog_control.config.joints import (
        ALL_IDS,
        DM_MASTER_ID_BY_SLAVE,
        INCOS_CAN_IDS,
        JOINT_BY_ID,
        JOINT_MAP as _JM,
    )
    from marsdog_control.hardware.board import RkMotorBoard
    from marsdog_control.hardware.motors.damiao import MotorDamiao
    from marsdog_control.hardware.motors.evo import MotorEvo
    from marsdog_control.hardware.motors.incos import MotorIncos
    from marsdog_control.hardware.motors.lingzu import MotorLz
    from marsdog_control.hardware.sensors.imu_wt901 import ImuWT901
    from marsdog_control.runtime.walk_bringup import (
        bringup_imu,
        bringup_motors_and_board,
        fade_to_stand,
    )
    from marsdog_control.runtime.walk_services import WalkServices

    stand = ctx["stand"]
    planner = ctx["planner"]
    grav_scale = ctx["grav_scale"]
    gravity_on = ctx["gravity_on"]
    runtime_state = ctx["runtime_state"]
    allow_lift = bool(tool.allow_lift)
    lift_blocked_msg: list = []
    max_tilt = math.radians(max(0.0, float(tool.max_tilt_deg)))

    real_joints = [j for j in _JM if j.bus != "none"]
    dm_joints = [j for j in _JM if j.mtype == "dm"]
    svc = WalkServices(
        runtime_state=runtime_state,
        real_joints=real_joints,
        resource_dir=str(legacy_dir()),
        control_hz=CONTROL_HZ,
        clock=time,
    )

    imu, imu_ok = bringup_imu(
        imu_cls=ImuWT901,
        imu_device=IMU_DEVICE,
        imu_baud=IMU_BAUD,
        angle_tau_s=0.0,
        gyro_tau_s=0.0,
        require_imu=False,
    )
    hw = bringup_motors_and_board(
        motor_lz_cls=MotorLz,
        motor_evo_cls=MotorEvo,
        motor_damiao_cls=MotorDamiao,
        motor_incos_cls=MotorIncos,
        board_cls=RkMotorBoard,
        lz_serial_device=LZ_SERIAL_DEVICE,
        lz_can1_device=LZ_CAN1_DEVICE,
        evo_can0_device=EVO_CAN0_DEVICE,
        dm_can_device=DM_CAN_DEVICE,
        incos_can_device=INCOS_CAN_DEVICE,
        baud=BAUD,
        joint_map=JOINT_MAP,
        dm_joints=dm_joints,
        dm_master_id_by_slave=DM_MASTER_ID_BY_SLAVE,
        incos_can_ids=INCOS_CAN_IDS,
        joint_by_id=JOINT_BY_ID,
        all_ids=ALL_IDS,
        shutdown_motors=svc.shutdown_motors,
        clock=time,
    )
    if hw is None:
        print("[CoMBal] motor bring-up failed")
        return 1

    lz, evo, dm, incos = hw.lz, hw.evo, hw.dm, hw.incos
    svc.board = hw.board
    runtime_state.board = hw.board
    runtime_state.dm.fixed_targets.clear()
    runtime_state.dm.fixed_targets.update(hw.dm_fixed_targets)
    online = hw.online

    print("[CoMBal] reading positions...")
    cur_pos = svc.read_positions(lz, evo, incos)
    if dm is not None:
        cur_pos.update(runtime_state.dm.fixed_targets)

    ready = fade_to_stand(
        stand=stand,
        cur_pos=cur_pos,
        online=online,
        lz=lz, evo=evo, dm=dm, incos=incos,
        fade_s=float(tool.fade_s),
        smooth_transition=svc.smooth_transition,
        recover_lz_stand_faults=svc.recover_lz_stand_faults,
        shutdown_motors=svc.shutdown_motors,
    )
    if not ready.ok:
        print("[CoMBal] fade-to-stand failed")
        return 1

    if imu_ok:
        try:
            print("[IMU] brief calibrate after stand...")
            imu.calibrate(1.0)
        except Exception as exc:
            print(f"[IMU] calibrate skipped: {exc}")

    backend = RealRobotBackend(svc, lz, evo, dm, incos, imu)
    keyboard = KeyReader()
    keyboard.start()
    log_every = max(1, int(CONTROL_HZ / max(0.1, tool.log_hz)))
    max_ticks = int(tool.duration * CONTROL_HZ) if tool.duration > 0 else 0
    tick = 0
    wall0 = time.time()
    running = True
    abort_reason = ""

    log_joints = not bool(tool.no_log_joints)

    def _log(
        roll: float,
        pitch: float,
        targets: Optional[Dict[int, float]] = None,
        measured: Optional[Dict[int, float]] = None,
    ) -> None:
        print(
            f"[CoMBal] {planner.describe()}  "
            f"imu_rpy=({math.degrees(roll):+.1f},{math.degrees(pitch):+.1f})deg  "
            f"allow_lift={allow_lift}"
        )
        if log_joints:
            tgt = targets if targets is not None else planner.get_targets()
            _print_joint_angles(tgt, measured)

    try:
        print("[CoMBal] real loop — WASD / E / R / X  (Q only with --allow-lift)")
        if log_joints:
            print("[CoMBal] joint log ON  (tgt=command URDF deg, fb=feedback URDF deg; "
                  "--no-log-joints to disable)")
        while running:
            t_loop = time.time()
            while True:
                key = keyboard.get()
                if key is None:
                    break
                if not _handle_key(
                    key, planner, tool.com_step,
                    allow_lift=allow_lift, lift_blocked_msg=lift_blocked_msg,
                ):
                    running = False
                    abort_reason = "quit key"
                    break
            if not running:
                break

            state = backend.read_state(online)
            if max_tilt > 1e-9 and (
                abs(state.roll) > max_tilt or abs(state.pitch) > max_tilt
            ):
                print(
                    f"[CoMBal] TILT ABORT  "
                    f"roll={math.degrees(state.roll):+.1f}deg  "
                    f"pitch={math.degrees(state.pitch):+.1f}deg  "
                    f"(limit ±{tool.max_tilt_deg:.1f}deg)"
                )
                planner.plant_diag()
                planner.reset_com()
                backend.send(
                    _control_output(
                        planner.get_targets(),
                        gravity_trq(planner.get_targets(), grav_scale)
                        if gravity_on else {},
                    )
                )
                abort_reason = "tilt"
                running = False
                break

            targets = planner.get_targets()
            trq = gravity_trq(targets, grav_scale) if gravity_on else {}
            backend.send(_control_output(targets, trq))
            tick += 1
            if tick % log_every == 0:
                _log(state.roll, state.pitch, targets, state.joint_pos)
            if max_ticks and tick >= max_ticks:
                print(f"[CoMBal] duration reached ({tool.duration:.1f}s wall)")
                abort_reason = "duration"
                break

            # Wall-clock pace (~200 Hz).
            sleep_t = CONTROL_DT - (time.time() - t_loop)
            if sleep_t > 0:
                time.sleep(sleep_t)
    except KeyboardInterrupt:
        print("\n[CoMBal] interrupted")
        abort_reason = "KeyboardInterrupt"
    finally:
        keyboard.stop()
        try:
            planner.plant_diag()
            targets = planner.get_targets()
            trq = gravity_trq(targets, grav_scale) if gravity_on else {}
            backend.send(_control_output(targets, trq))
            time.sleep(0.05)
            if log_joints:
                try:
                    st = backend.read_state(online)
                    _print_joint_angles(targets, st.joint_pos)
                except Exception:
                    _print_joint_angles(targets)
        except Exception:
            pass
        print(
            f"[CoMBal] done  ticks={tick}  final {planner.describe()}  "
            f"reason={abort_reason or 'ok'}  "
            f"(record this com_x/y if still balanced)"
        )
        # svc.shutdown_motors(lz, evo, dm, incos)
        print("[CoMBal] motors shut down")
    return 0 if abort_reason in ("", "quit key", "duration", "ok") else 1


def main(argv: list[str] | None = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    tool, remainder = _parse_tool_flags(argv)

    # Feed remainder (+ keep process name) into walk_cli via _build_args.
    old_argv = sys.argv
    sys.argv = [old_argv[0]] + list(remainder)
    try:
        ctx = _bootstrap_stand(tool)
    finally:
        sys.argv = old_argv
    if ctx is None:
        return 1

    if tool.real:
        return _run_real(tool, ctx)
    return _run_sim(tool, ctx)


if __name__ == "__main__":
    raise SystemExit(main())
