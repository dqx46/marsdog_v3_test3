"""NaturalWalk four-beat phase + liveliness trajectory tests."""

from __future__ import annotations

from marsdog_control.motion.foot_trajectory import (
    lateral_offset_walk,
    natural_soft_trot_x,
    natural_walk_x,
    walk_weight_shift_sign,
)
from marsdog_control.motion.gait_controller import (
    NaturalSoftTrot,
    NaturalWalk,
    StablePace,
    WALK_PHASE_OFFSET,
)
from marsdog_control.motion.gait_recipes import (
    NATURAL_SOFT_TROT_WBC,
    NATURAL_WALK_WBC,
)
from marsdog_control.motion.gait_schedule import (
    GaitEnvelope,
    VelocityCommand,
    WalkSchedule,
    apply_schedule_to_gait,
)


def _lift_frac(offset: float, stance: float) -> float:
    return (stance - offset) % 1.0


def test_walk_phases_are_four_beat():
    assert WALK_PHASE_OFFSET == {
        "fl": 0.50, "fr": 0.00, "rl": 0.75, "rr": 0.25,
    }
    walk = NaturalWalk(
        amp_front=0.054, amp_rear=0.062, step_height=0.036,
        period=1.0, stance_ratio=0.73,
    )
    assert walk.family == "walk"
    assert walk._PHASE_OFFSET == WALK_PHASE_OFFSET

    soft = NaturalSoftTrot(
        amp_front=0.05, amp_rear=0.068, step_height=0.048,
        period=0.58, stance_ratio=0.56,
    )
    assert soft._PHASE_OFFSET != walk._PHASE_OFFSET

    pace = StablePace(amp_front=0.03, amp_rear=0.03, period=0.8, stance_ratio=0.7)
    assert pace._PHASE_OFFSET["fl"] == pace._PHASE_OFFSET["rl"]


def test_walk_lift_order_is_lh_lf_rh_rf():
    stance = 0.75  # offsets designed for this duty
    lifts = {
        leg: _lift_frac(off, stance)
        for leg, off in WALK_PHASE_OFFSET.items()
    }
    order = sorted(lifts.keys(), key=lambda leg: lifts[leg])
    assert order == ["rl", "fl", "rr", "fr"]


def test_walk_uses_linear_stance_not_soft_mj():
    """Walk stance X is linear; SoftTrot MJ differs away from mid-stance."""
    amp, cx, sr = 0.05, 0.0, 0.75
    phase = 0.25 * sr  # quarter stance — MJ ≠ linear
    x_walk, sw_w, _ = natural_walk_x(phase, amp, cx, sr, 0.03, 0.22)
    x_soft, sw_s, _ = natural_soft_trot_x(phase, amp, cx, sr, 0.03, 0.40)
    assert sw_w is False and sw_s is False
    assert abs(x_walk - (cx + amp * (1.0 - 2.0 * 0.25))) < 1e-9
    assert abs(x_walk - x_soft) > 1e-4


def test_walk_sway_holds_before_left_lift():
    """At RL lift (phase≈0) CoM already on right (negative)."""
    assert walk_weight_shift_sign(0.0) < 0.0
    assert walk_weight_shift_sign(0.20) < 0.0
    assert walk_weight_shift_sign(0.70) > 0.0
    period, sway = 1.0, 0.01
    assert lateral_offset_walk(0.0, period, sway) < 0.0
    walk = NaturalWalk(
        amp_front=0.054, amp_rear=0.062, step_height=0.036,
        period=period, stance_ratio=0.73, lateral_sway=sway, com_sway_m=0.026,
    )
    assert walk.get_com_y_shift(0.0) < 0.0
    assert walk.get_com_y_shift(0.7 * period) > 0.0


def test_walk_never_enables_spot():
    walk = NaturalWalk(
        amp_front=0.054, amp_rear=0.062, step_height=0.036,
        period=1.0, stance_ratio=0.73,
    )
    walk.spot_turn_active = True
    _ = walk.get_targets(0.1)
    assert walk.spot_turn_active is False
    # Walk _leg_xz path used (no crash) and phases intact
    x, z = walk._leg_xz("rl", 0.05)
    assert isinstance(x, float) and isinstance(z, float)
    assert walk._PHASE_OFFSET == WALK_PHASE_OFFSET


def test_walk_recipe_isolated_from_soft_trot():
    soft_amp = NATURAL_SOFT_TROT_WBC["amp_front"]
    soft_period = NATURAL_SOFT_TROT_WBC["period"]
    walk = dict(NATURAL_WALK_WBC)
    walk["amp_front"] = 0.999
    assert NATURAL_SOFT_TROT_WBC["amp_front"] == soft_amp
    assert NATURAL_SOFT_TROT_WBC["period"] == soft_period
    assert NATURAL_WALK_WBC["period"] == 1.00
    assert NATURAL_WALK_WBC["stance"] == 0.75
    assert NATURAL_WALK_WBC["amp_front"] == 0.054
    assert NATURAL_WALK_WBC["step_h"] == 0.036
    assert NATURAL_WALK_WBC["com_sway_m"] >= 0.024


def test_walk_schedule_scales_forward_only():
    sch = WalkSchedule(
        GaitEnvelope.from_walk(
            amp_front=0.054, amp_rear=0.062, period=1.0, stance=0.75,
            throttle_min_scale=0.5,
        )
    )
    full = sch.map(VelocityCommand(vx=sch.max_forward_vx(), yaw_rate=-0.4))
    assert full.spot_turn is False
    assert full.turn_cmd == 0.0
    assert abs(full.amp_front - 0.054) < 1e-9
    assert 0.88 <= full.period <= 1.25
