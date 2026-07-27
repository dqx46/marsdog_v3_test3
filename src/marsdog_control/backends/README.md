# Backends

This directory is reserved for future runtime backends. It intentionally does
not define backend classes yet.

Future boundaries:

- `RealRobotBackend` owns physical motors, sensors, timing, and device startup.
- `SimRobotBackend` owns simulator stepping and simulator-specific state reads.
- `ReplayBackend` owns deterministic playback of recorded robot state.
- Every backend must expose robot state in the same internal units: meters,
  radians, seconds, radians per second, and meters per second.
- Backend outputs must still flow through `MotionTarget -> SafetySupervisor ->
  CommandExecutor`; no backend may bypass safety.

The current hardware implementation remains in `marsdog_control.hardware` while
the legacy walking runtime is migrated gradually.
