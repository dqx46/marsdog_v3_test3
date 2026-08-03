"""Teleop stick → SI VelocityCommand (decoupled from gait schedule)."""

from __future__ import annotations

from marsdog_control.input.teleop_policy import (
    DEFAULT_CRUISE_VX_MPS,
    TeleopPolicy,
    stick_to_body_velocity,
)
from marsdog_control.motion.gait_schedule import SoftTrotSchedule, VelocityCommand


def test_engage_cruise_ignores_stick_depth():
    p = TeleopPolicy(cruise_vx_mps=0.134, engage_threshold=0.15)
    a = stick_to_body_velocity(0.2, 0.0, policy=p)
    b = stick_to_body_velocity(1.0, 0.0, policy=p)
    assert abs(a.vx - 0.134) < 1e-9
    assert abs(b.vx - 0.134) < 1e-9


def test_below_threshold_is_stop():
    p = TeleopPolicy(cruise_vx_mps=0.134, engage_threshold=0.15)
    cmd = stick_to_body_velocity(0.10, 0.0, policy=p)
    assert cmd.vx == 0.0


def test_yaw_stick_to_rad_s():
    p = TeleopPolicy(yaw_rate_max=0.40, deadzone=0.12)
    cmd = stick_to_body_velocity(0.0, 1.0, policy=p)
    assert abs(cmd.yaw_rate - 0.40) < 1e-9
    idle = stick_to_body_velocity(0.0, 0.05, policy=p)
    assert idle.yaw_rate == 0.0


def test_schedule_consumes_si_not_stick():
    """Half cruise m/s is a first-class SI command (no stick floor)."""
    sch = SoftTrotSchedule()
    full_cruise = DEFAULT_CRUISE_VX_MPS
    half = stick_to_body_velocity(
        1.0, 0.0, policy=TeleopPolicy(cruise_vx_mps=0.5 * full_cruise)
    )
    out = sch.map(VelocityCommand(vx=half.vx))
    assert out.speed_frac > 0.0
    assert abs(out.vel_cmd[0]) < abs(sch.map(VelocityCommand(vx=full_cruise)).vel_cmd[0])
