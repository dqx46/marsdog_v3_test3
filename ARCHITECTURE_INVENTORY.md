# Marsdog Control Architecture Inventory

This file records the current ownership of the walking control application.
It is a migration aid for the `marsdog_control` package and does not change
runtime behavior.

Status after the decoupling pass:

- Real implementations for the independent modules now live under
  `src/marsdog_control/**`.
- The matching `mocap_to_real/*.py` files are compatibility aliases that
  bootstrap `src` and point to the same module objects. This preserves legacy
  imports, underscore symbols, and mutable module globals such as
  `gait_controller.ABD_LEGACY`.
- The walking application implementation now lives in
  `src/marsdog_control/apps/walk.py`. The old `mocap_to_real/walk.py` file is a
  compatibility launcher/module alias only, so existing commands and flat
  imports keep working while implementation ownership is in `src`.
- The hardware boundary now has an explicit Board layer:
  `marsdog_control/hardware/board.py` (`RkMotorBoard`) plus
  `marsdog_control/hardware/mapping.py`. The current RK3588 process still owns
  the concrete LZ/EVO/DM/Incos drivers, but runtime code goes through the Board
  contract so a future STM32 board can replace that implementation.
- Offline parity coverage lives in `tests/parity/`:
  - motion/executor outputs are pinned against
    `tests/parity/golden_motion.json`;
  - the fake-hardware full-loop harness records the real `walk.main()`
    command stream against `tests/parity/golden_loop.json`, giving Phase 3 a
    deterministic off-robot target before any real-robot switch.

## Application Entry

- `src/marsdog_control/apps/walk.py`
  - CLI parsing and preset application: `parse_args()`,
    `apply_preset_preserving_cli()`.
  - Runtime lifecycle: `main()`, startup validation, hardware init, stand-up,
    IMU calibration, main loop, shutdown ramp.
- `mocap_to_real/walk.py`
  - Compatibility launcher only. `import walk` resolves to the same module
    object as `marsdog_control.apps.walk`.

## Input

Current home:

- `KeyReader`, `InputState`, `poll_user_command()`, and
  `apply_dev_tuning()` now live in `marsdog_control/input/user_input.py`.
- `walk.py` keeps compatibility names (`KeyReader`, `_InputState`,
  `poll_user_command`, `apply_dev_tuning`) and forwards the runtime hotkey
  state (`TD_KP_SCALE`, `GRAV_SCALE`) through `DevTuningRuntime`.
- Offline coverage: `tests/test_user_input.py` pins keyboard/gamepad edge
  parsing and development hotkey side effects.

## Hardware And Actuation

Real driver implementations now live in:

- Lingzu motors: `marsdog_control/hardware/motors/lingzu.py`
- EVO motors: `marsdog_control/hardware/motors/evo.py`
- Damiao motors: `marsdog_control/hardware/motors/damiao.py`
- CAN helpers: `marsdog_control/hardware/motors/can_bus.py`,
  `marsdog_control/hardware/motors/can_serial.py`
- WT901 IMU: `marsdog_control/hardware/sensors/imu_wt901.py`
- Tail / audio behavior channels:
  `marsdog_control/hardware/behavior/tail.py`,
  `marsdog_control/hardware/behavior/audio.py`
- Device and joint configuration:
  `marsdog_control/config/bus_config.py`,
  `marsdog_control/config/devices.py`,
  `marsdog_control/config/joints.py`

Sunk into `src` (implementation lives there; `walk` keeps a byte-identical thin
wrapper that forwards the current runtime knobs, so behavior is unchanged):

- `send_all()` → `marsdog_control/hardware/actuation.py` (`send_all`); the
  runtime-mutable DM/tarsus/leg knobs are passed in via `ActuationRuntime`
  instead of read from `walk` globals.
- Board abstraction:
  - `marsdog_control/hardware/board.py` (`MotorBoard`, `RkMotorBoard`) owns
    driver lifecycle, unified feedback, soft disable, shutdown ordering, and
    the current software Board implementation.
  - `marsdog_control/hardware/mapping.py` owns per-bus command batch creation
    from motor-frame angle targets.
- `_resolve_gains()` → `marsdog_control/control/executor.py` (`resolve_gains`,
  pure; `leg_kp_scale`/`joint_gains` passed in).
- `read_positions()` → `marsdog_control/hardware/robot_hw.py`
  (`read_robot_positions`).
- DM tarsus fixed-target and lead handling now lives inside
  `hardware/actuation.send_all` (driven by `ActuationRuntime`).

Compatibility wrappers remain in `walk.py`, but the real runtime path now
prefers Board-backed send/read/diagnostic/cleanup. LZ-specific recovery still
uses the LZ driver internally through the RK Board implementation because that
fault mode is not a portable future-STM32 contract.

Package boundaries already exist at:

- `marsdog_control/hardware/robot_hw.py`
- `marsdog_control/control/executor.py`
- `marsdog_control/runtime/startup.py`
- `marsdog_control/runtime/shutdown.py`

## State Estimation

Current home:

- WT901 driver implementation: `marsdog_control/hardware/sensors/imu_wt901.py`.
- IMU closed-loop controller implementation:
  `marsdog_control/control/imu_balance.py`.
- `read_state()` → `marsdog_control/hardware/robot_hw.py` (`read_robot_state`);
  the `RobotState` snapshot math is now in `src` and covered by an offline
  fake-motor parity test (`tests/test_robot_hw_readstate.py`). `walk` keeps a
  one-line wrapper forwarding `DM_FIXED_TARGETS` / `_REAL_JOINTS`.
- Loop-specific IMU policy now lives in `marsdog_control/control/balance.py`
  (`RuntimeBalanceController`): soft-start, D ramp, touchdown freeze, phase
  gate, trim ramp, gait-phase calculation, and stable auto-trim sampling.
  `apps/walk.py` only wires the component and consumes `BalanceOutput`.

## Motion

Real implementations now live in:

- Kinematics: `marsdog_control/motion/kinematics.py`
- Gait controllers: `marsdog_control/motion/gait_controller.py`
- Gait presets / recipes: `marsdog_control/motion/gait_recipes.py`
- Motion target construction:
  `marsdog_control/motion/motion_planner.py`

Moved from `walk.py` into `motion_planner.py`:

- `build_motion_target()`
- `_apply_stand_imu_dz()`
- `build_hip_abduction_test_target()`
- `build_leg_pitch_direction_test_target()`
- `build_calf_pitch_direction_test_target()`
- Blend state consumption from `RuntimeStateMachine`
- Fine rate limiting with `_smooth_tgt`

Still legacy-owned:

- `build_lie_down_target()`, because it depends on captured pose resources and
  shutdown sequencing that should move with the lifecycle code.

## Control

Real implementations now live in:

- IMU attitude controller and phase gain:
  `marsdog_control/control/imu_balance.py`
- Gravity feed-forward model:
  `marsdog_control/control/gravity_comp.py`
- Executor boundary and velocity feed-forward:
  `marsdog_control/control/executor.py`
- Impedance exports:
  `marsdog_control/control/impedance.py`
- Smoothstep soft-start helpers:
  `marsdog_control/control/ramps.py`

Moved from `walk.py` into `executor.py`:

- Gravity torque calculation, exposed as `gravity_trq(targets, grav_scale)`.
  `walk._gravity_trq()` is now a thin wrapper that passes the runtime
  `GRAV_SCALE`.
- Gain resolution, exposed as `resolve_gains(...)`. `walk._resolve_gains()`
  forwards `LEG_KP_SCALE` and `JOINT_GAINS` so behavior stays unchanged.
- IMU/trim smoothstep soft-start math, exposed as `softstart_gain(...)` in
  `control/ramps.py` and covered by `tests/test_ramps.py`.

Still app-owned:

- CLI / preset / FSM+IMU-controller construction wiring in `apps/walk.py` `main()`.
- Default per-joint gains: `config/gains.py` (`JOINT_GAINS`); app re-exports the name.
- Live knobs live in `runtime/walk_state.py` (`WalkRuntimeState`); CLI writes
  them first, then mirrors to legacy module globals for remaining readers.
- Hardware open/enable/board, stand fade, post-stand IMU cal, operator inputs:
  `runtime/walk_bringup.py`.
- Steady-state control is no longer inlined in the app: the sole live path is
  `RuntimeApp(pipeline=...).run()` → `RuntimePipeline.tick()` →
  `tick_walk_loop` (`runtime/walk_loop.py`). Component wiring lives in
  `runtime/walk_assembly.py` (`assemble_walk_loop_context`).
  `run_steady_state_loop(ctx)` remains a thin compat wrapper around the same
  pipeline tick.

## Safety

- Real `SafetySupervisor` implementation:
  `marsdog_control/safety/supervisor.py`
- ESTOP handling around `SafetyReport.triggered_estop`

The legacy `mocap_to_real/safety_supervisor.py` file is now a compatibility
alias to the `src` implementation.

## Logging And Diagnostics

Current home:

- `setup_log()` now lives in `marsdog_control/io/logging.py`; `walk.py` keeps a
  thin wrapper that passes the legacy log directory and current metadata knobs
  through `LogRuntime`.
- Offline coverage: `tests/test_logging_setup.py` pins the CSV header and meta
  JSON shape.

- `write_log()` row construction now accepts a Board `MotorFeedbackFrame`; the
  legacy wrapper passes `board.get_feedback()` so logging no longer has to own
  concrete motor routing in the main runtime.
- Runtime log cadence and per-row context assembly now live in
  `marsdog_control/io/recorder.py` (`RecorderRuntime`). The app loop no longer
  owns log cycle counting, `run_t_s`, ramp fraction calculation, or controller
  name selection.
- Periodic terminal status and disabled-motor health checks now live in
  `marsdog_control/runtime/status.py` (`RuntimeStatusDisplay`). Health checks
  prefer Board feedback, so Incos motors are not misclassified as EVO motors.
- Walk shutdown cleanup now lives in `marsdog_control/runtime/shutdown.py`
  (`WalkShutdownContext`, `run_walk_shutdown`): input/tail close, auto-trim
  persistence, return-to-stand selection, Board soft-disable, driver release,
  log close, and scope teardown.

## Tools And Legacy

- Calibration: `set_zero_all.py`, `go_zero.py`, `dm_switch_mode.py`,
  IMU setup scripts.
- Diagnostics and benchmarks: `diag.py`, `static_test.py`, `test_*`,
  `bench_*`, `comm_bench.py`, `freq_loss_bench.py`.
- Analysis: `plot_*`, `analyze_*`, `phase_analysis.py`.
- Legacy: `walk2.py`, `motor_lz.py`, `*.bak_pre_merge`.

Target home: wrappers under `marsdog_control/apps/tools/`, with the original
files retained until the new package fully replaces the flat layout.

## Current Verification

- `python -m unittest discover -s tests -p "test_*.py"` passes on the
  development machine.
- The parity harness imports the legacy loop off-target and verifies that the
  moved motion/executor outputs still match the committed golden snapshot.
- The full-loop fake-hardware harness boots `walk.main()` off-target and
  verifies the recorded `send_all` stream still matches
  `tests/parity/golden_loop.json`.
- Direct import identity has been checked for the sunk modules so legacy flat
  imports and new `src` imports share the same module objects.
