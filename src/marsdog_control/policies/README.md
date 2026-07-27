# Policies

This directory is reserved for future learned or scripted policies. It
intentionally does not define an RL policy interface yet.

Future policy rules:

- A policy may output high-level gait commands or residual corrections.
- A policy must not write motor commands directly.
- Policy output must be adapted into `MotionTarget` first.
- Every `MotionTarget` must pass through `SafetySupervisor` before
  `CommandExecutor` builds motor commands.
- The first RL integration should prefer residual control on top of the
  traditional gait planner, not a full replacement of the proven safety path.
- Policy inputs and outputs must use internal SI units: meters, radians, and
  seconds.

Concrete `Observation` and `PolicyAction` types are intentionally deferred until
the simulator, logging, and real hardware data contracts are stable.
