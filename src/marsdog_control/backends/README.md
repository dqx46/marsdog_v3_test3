# Backends

Shared I/O boundary for simulation and hardware. Controllers speak only
`RobotState` / `ControlOutput` (URDF frame); backends own devices and stepping.

## Implemented

| Class | Module | Role |
|---|---|---|
| `RobotBackend` | `base.py` | Protocol: `read_state` / `send` / `shutdown` |
| `SimRobotBackend` | `sim.py` | MuJoCo step + MIT-equivalent actuators |
| `RealRobotBackend` | `real.py` | Motors / IMU via `WalkServices` (sign → URDF) |

Units are meters, radians, seconds. `joint_pos` / `joint_vel` are always URDF
space. Base linear velocity for control comes from `BaseStateEstimator`
(`--base-estimate-mode estimator`); MuJoCo `vel_xyz` truth is debug / telemetry
only (`truth` mode).

## Contract

```
MotionTarget → SafetySupervisor → CommandExecutor → ControlOutput
                                                      │
                                              RobotBackend.send()
```

No backend may bypass safety. Replay / RL backends remain future work; do not
add device-specific branches inside gait or WBC.
