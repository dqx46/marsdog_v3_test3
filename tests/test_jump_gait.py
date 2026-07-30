"""JumpController phase machine + recipe isolation tests."""

from __future__ import annotations

from marsdog_control.motion.gait_controller import JumpController, JumpPhase
from marsdog_control.motion.gait_recipes import (
    JUMP_REAL,
    JUMP_WBC,
    NATURAL_SOFT_TROT_WBC,
    NATURAL_WALK_WBC,
)
from marsdog_control.motion.gait_schedule import (
    JumpSchedule,
    VelocityCommand,
    WalkSchedule,
    SoftTrotSchedule,
    GaitEnvelope,
    apply_jump_schedule,
    apply_schedule_to_gait,
)

def test_jump_phase_sequence_crouch_to_idle():
    j = JumpController(
        body_height=0.24,
        crouch_s=0.10,
        push_s=0.10,
        flight_s=0.10,
        land_s=0.10,
        recover_s=0.10,
    )
    assert j.family == "jump"
    assert j.spot_turn_active is False
    assert j.phase is JumpPhase.IDLE

    j.request_jump(True)
    j.get_targets(0.0)
    assert j.phase is JumpPhase.CROUCH

    # Advance through each phase by exceeding duration.
    t = 0.0
    expected = [
        JumpPhase.CROUCH,
        JumpPhase.PUSH,
        JumpPhase.FLIGHT,
        JumpPhase.LAND,
        JumpPhase.RECOVER,
        JumpPhase.IDLE,
    ]
    seen = [j.phase]
    for _ in range(5):
        t += 0.11
        j.get_targets(t)
        seen.append(j.phase)
    assert seen == expected
    assert j.in_flight() is False


def test_jump_never_enables_spot():
    j = JumpController(body_height=0.24)
    j.spot_turn_active = True
    j.request_jump(True)
    _ = j.get_targets(0.0)
    assert j.spot_turn_active is False


def test_jump_flight_zero_force_scale():
    j = JumpController(
        body_height=0.24,
        crouch_s=0.05,
        push_s=0.05,
        flight_s=0.20,
        land_s=0.05,
        recover_s=0.05,
    )
    j.request_jump(True)
    j.get_targets(0.0)
    j.get_targets(0.06)  # PUSH
    j.get_targets(0.12)  # FLIGHT
    assert j.phase is JumpPhase.FLIGHT
    assert j.jump_force_scale_at(0.12) == 0.0
    assert j.stance_ratio == 0.0


def test_jump_schedule_trigger_and_isolation():
    sched = JumpSchedule(vx_deadzone=0.12)
    out = sched.map(VelocityCommand(vx=0.5, yaw_rate=0.9))
    assert out.trigger is True
    assert out.auto_rejump is True
    # yaw ignored
    idle = sched.map(VelocityCommand(vx=0.0, yaw_rate=0.9))
    assert idle.trigger is False

    j = JumpController(body_height=0.24)
    apply_jump_schedule(j, out)
    assert j.trigger is True
    assert j.auto_rejump is True
    assert j.spot_turn_active is False

    # Soft schedule must not mutate Jump (apply_schedule_to_gait early-return).
    soft_env = GaitEnvelope.from_wbc_soft_trot(
        amp_front=0.05, amp_rear=0.06, period=0.6, stance=0.55,
    )
    soft_out = SoftTrotSchedule(soft_env).map(VelocityCommand(vx=0.8, yaw_rate=0.0))
    before = (j.amp_front, j.period, j.stance_ratio)
    apply_schedule_to_gait(j, soft_out)
    assert (j.amp_front, j.period, j.stance_ratio) == before


def test_jump_recipe_isolation_from_soft_and_walk():
    soft_before = dict(NATURAL_SOFT_TROT_WBC)
    walk_before = dict(NATURAL_WALK_WBC)
    jump = dict(JUMP_WBC)
    jump["crouch_depth"] = 0.099
    jump["push_vz"] = 1.23

    j = JumpController(
        body_height=jump["height"],
        crouch_depth=jump["crouch_depth"],
        push_vz=jump["push_vz"],
    )
    assert abs(j.crouch_depth - 0.099) < 1e-9
    assert abs(j.push_vz - 1.23) < 1e-9
    assert NATURAL_SOFT_TROT_WBC == soft_before
    assert NATURAL_WALK_WBC == walk_before
    assert JUMP_REAL["crouch_depth"] != 0.099
    assert JUMP_WBC["crouch_depth"] != 0.099


def test_walk_and_soft_schedules_still_green():
    walk = WalkSchedule(
        GaitEnvelope.from_walk(
            amp_front=0.05, amp_rear=0.06, period=1.0, stance=0.75,
        )
    )
    out = walk.map(VelocityCommand(vx=0.6, yaw_rate=0.5))
    assert out.spot_turn is False
    assert out.turn_cmd == 0.0
    assert out.amp_front > 0.0

    soft = SoftTrotSchedule(
        GaitEnvelope.from_wbc_soft_trot(
            amp_front=0.05, amp_rear=0.06, period=0.6, stance=0.55,
        )
    )
    spot = soft.map(VelocityCommand(vx=0.0, yaw_rate=0.8))
    assert spot.spot_turn is True
