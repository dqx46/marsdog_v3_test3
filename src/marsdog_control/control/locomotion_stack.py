"""Locomotion stack seams (sim-first).

Data flow (WBC path)::

    UserCommand / VelocityCommand
           │
           ▼
    SoftTrotSchedule  →  GaitScheduleOutput
           │
           ▼
    GaitController (amps, period, stance, turn, vel_cmd)
           │
           ├─────────────► MotionTarget (q, dq)     [kinematic shape]
           │
           ▼
    ContactSchedule → ForcePlanner(MPC) → WholeBodyController
           │
           ▼
    ControlOutput (q, dq, kp*, trq_ff) → RobotBackend

Dynamics ownership today:
  - Stance forces + swing foot accel + base PD: WBC/MPC (trq_ff)
  - Foot Cartesian shape / phasing: gait kinematics (q)
  - Joint MIT kp: softened under WBC (leg_kp_scale) so τ_ff can dominate

Real/sim parity: Backend fills joint_pos/joint_vel in URDF frame;
BaseStateEstimator fills vel_xyz (default). Do not use MuJoCo truth in the
control path except ``base_estimate_mode=truth`` for debug.
"""

from __future__ import annotations

from marsdog_control.control.contact_schedule import ContactSchedule
from marsdog_control.control.dynamics_telemetry import DynamicsTelemetry
from marsdog_control.control.force_planner import ForcePlanner
from marsdog_control.motion.gait_schedule import (
    SoftTrotSchedule,
    VelocityCommand,
    apply_schedule_to_gait,
)

__all__ = [
    "ContactSchedule",
    "DynamicsTelemetry",
    "ForcePlanner",
    "SoftTrotSchedule",
    "VelocityCommand",
    "apply_schedule_to_gait",
]
