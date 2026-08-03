"""Stick is engage-only: depth must not change SoftTrot cruise (SI m/s)."""

from __future__ import annotations

from marsdog_control.config.stack_build import FsmDriveConfig
from marsdog_control.core.types import Direction, RobotMode, RobotState, UserCommand
from marsdog_control.input.teleop_policy import DEFAULT_CRUISE_VX_MPS
from marsdog_control.motion.gait_recipes import ControllerSet
from marsdog_control.motion.gait_schedule import SoftTrotSchedule, VelocityCommand
from marsdog_control.runtime.fsm import RuntimeStateMachine


class _Gait:
    family = "soft"
    period = 0.58
    stance_ratio = 0.56
    amp_front = 0.05
    amp_rear = 0.068
    step_height = 0.048
    step_height_front = 0.045
    _PHASE_OFFSET = {"fl": 0.0, "fr": 0.5, "rl": 0.5, "rr": 0.0}
    spot_turn_active = False
    vel_cmd = (0.0, 0.0, 0.0)
    turn_cmd = 0.0
    turn_y_gain = 0.0
    _reactive_filtered = 0.0

    def set_height(self, h):
        pass

    def set_period(self, p):
        self.period = p


def _fsm(cruise_mps: float = DEFAULT_CRUISE_VX_MPS) -> RuntimeStateMachine:
    g = _Gait()
    controllers = ControllerSet(
        stand=g, fwd=g, bwd=g, pace_fwd=g, pace_bwd=g,
        nat_fwd=g, walk_fwd=g, jump_fwd=g,
    )
    drive = FsmDriveConfig(
        gp_trot_threshold=0.15,
        gp_deadzone=0.12,
        cruise_vx=cruise_mps,
        throttle_min_scale=0.45,
    )
    return RuntimeStateMachine(
        controllers,
        drive,
        height=0.24,
        fwd_amp_front=0.05,
        fwd_amp_rear=0.068,
        natural_configured=True,
    )


def test_stick_depth_maps_to_fixed_cruise_vx_mps():
    fsm = _fsm(0.134)
    assert abs(fsm._stick_cruise_vx(0.2) - 0.134) < 1e-9
    assert abs(fsm._stick_cruise_vx(1.0) - 0.134) < 1e-9
    assert abs(fsm._stick_cruise_vx(-0.8) + 0.134) < 1e-9


def test_natural_stick_half_and_full_same_schedule():
    cruise = 0.134
    fsm = _fsm(cruise)
    state = RobotState()
    fsm.request_transition(RobotMode.NATURAL, Direction.FWD, targets_now={})

    cmd_half = UserCommand(vx=0.51, has_stick=True)
    fsm.update(state, cmd_half, last_targets={})
    amp_half = float(fsm.nat_fwd.amp_front)
    period_half = float(fsm.nat_fwd.period)
    th_half = float(fsm.throttle)

    cmd_full = UserCommand(vx=1.0, has_stick=True)
    fsm.update(state, cmd_full, last_targets={})
    amp_full = float(fsm.nat_fwd.amp_front)
    period_full = float(fsm.nat_fwd.period)
    th_full = float(fsm.throttle)

    assert abs(th_half - cruise) < 1e-9
    assert abs(th_full - cruise) < 1e-9
    assert abs(amp_half - amp_full) < 1e-9
    assert abs(period_half - period_full) < 1e-9

    expect = SoftTrotSchedule(fsm._nat_schedule.env).map(VelocityCommand(vx=cruise))
    assert abs(amp_full - expect.amp_front) < 1e-9
    assert abs(period_full - expect.period) < 1e-9
