"""Unit tests for ω×r scrub SpotYawStepper (diagonal trot)."""

from marsdog_control.motion.spot_yaw_step import (
    CATCH0_LEGS, CATCH1_LEGS, PHASE_OFFSET, SpotYawStepConfig, SpotYawStepper,
)


def _hip(leg):
    hx = 0.1694 if leg.startswith("f") else -0.1437
    hy = 0.040 if leg.endswith("l") else -0.040
    if not leg.startswith("f"):
        hy = 0.034 if leg.endswith("l") else -0.034
    return hx, hy


def test_diagonal_phases_not_pace():
    assert PHASE_OFFSET["fl"] == PHASE_OFFSET["rr"] == 0.0
    assert PHASE_OFFSET["fr"] == PHASE_OFFSET["rl"] == 0.5
    assert PHASE_OFFSET["fl"] != PHASE_OFFSET["rl"]


def test_diagonal_swing_pairs():
    s = SpotYawStepper(cfg=SpotYawStepConfig(stance_ratio=0.55), hip_xy=_hip)
    period = 0.85
    t = 0.75 * period
    s.update_pose(t, yaw=0.0, base_xy=(0.0, 0.0))
    s.tick(t, turn=-0.8, period=period, stance_ratio=0.55)
    for lg in CATCH0_LEGS:
        assert s.in_swing(lg), lg
    for lg in CATCH1_LEGS:
        assert not s.in_swing(lg), lg


def test_scrub_is_yaw_shear():
    """Late-stance body-Y: front/rear oppose (dy ∝ hip_x)."""
    cfg = SpotYawStepConfig(yaw_step_rad=0.45, stance_ratio=0.55, scrub_x_scale=1.0)
    s = SpotYawStepper(cfg=cfg, hip_xy=_hip)
    period = 0.85
    t = 0.50 * period  # late FL/RR stance
    s.update_pose(t, yaw=0.0, base_xy=(0.0, 0.0))
    s.tick(t, turn=-1.0, period=period, stance_ratio=0.55)
    y_fl = s.cached_xy("fl")[1]
    y_rr = s.cached_xy("rr")[1]
    # Body-Y shear: front and rear means oppose.
    assert abs(y_fl) > 0.015 and abs(y_rr) > 0.015
    assert y_fl * y_rr < 0.0, (y_fl, y_rr)


def test_ipsilateral_abd_not_same_sign():
    """Same-side legs must NOT share abd sign (that would be pace/sway)."""
    cfg = SpotYawStepConfig(yaw_step_rad=0.45, stance_ratio=0.55)
    s = SpotYawStepper(cfg=cfg, hip_xy=_hip)
    period = 0.85
    t = 0.40 * period
    s.update_pose(t, yaw=0.0, base_xy=(0.0, 0.0))
    s.tick(t, turn=-1.0, period=period, stance_ratio=0.55)

    def abd(leg, y):
        side = 1.0 if leg.endswith("l") else -1.0
        return side * y / 0.24

    afl = abd("fl", s.cached_xy("fl")[1])
    arl = abd("rl", s.cached_xy("rl")[1])
    assert afl * arl <= 0.0 or abs(afl) < 1e-3 or abs(arl) < 1e-3


def test_yaw_des_never_recentres():
    """Measured yaw drifting must not drag yaw_des back toward yaw.

    Projection clamps ``yaw_des ∈ [yaw±lead]`` recentre the setpoint when
    pose updates the wrong way; freeze-at-lead anti-windup must not.
    """
    s = SpotYawStepper(
        cfg=SpotYawStepConfig(yaw_step_rad=0.45, yaw_lead_max=0.5),
        hip_xy=_hip,
    )
    period = 0.85
    s.update_pose(0.0, yaw=0.0, base_xy=(0.0, 0.0))
    s.tick(0.0, turn=-1.0, period=period, stance_ratio=0.55)
    s.update_pose(period, yaw=0.05, base_xy=(0.0, 0.0))
    s.tick(period, turn=-1.0, period=period, stance_ratio=0.55)
    assert s.yaw_des < -0.25
    held = s.yaw_des
    # Measured yaw moves opposite the turn command (wrong-way drift).
    s.update_pose(1.5 * period, yaw=0.10, base_xy=(0.0, 0.0))
    s.tick(1.5 * period, turn=-1.0, period=period, stance_ratio=0.55)
    # May freeze at lead limit, but must never recentre toward measured yaw.
    assert s.yaw_des <= held + 1e-12
    assert s.yaw_des < s.yaw - 0.25


def test_yaw_des_integrates_while_within_lead():
    s = SpotYawStepper(
        cfg=SpotYawStepConfig(yaw_step_rad=0.45, yaw_lead_max=0.5),
        hip_xy=_hip,
    )
    period = 0.85
    s.update_pose(0.0, yaw=0.0, base_xy=(0.0, 0.0))
    s.tick(0.0, turn=-1.0, period=period, stance_ratio=0.55)
    # Measured yaw tracks the command (more negative).
    s.update_pose(0.5 * period, yaw=-0.10, base_xy=(0.0, 0.0))
    s.tick(0.5 * period, turn=-1.0, period=period, stance_ratio=0.55)
    mid = s.yaw_des
    assert mid < -0.15
    s.update_pose(period, yaw=-0.20, base_xy=(0.0, 0.0))
    s.tick(period, turn=-1.0, period=period, stance_ratio=0.55)
    assert s.yaw_des < mid


def test_predict_force_scale_matches_diag():
    s = SpotYawStepper(cfg=SpotYawStepConfig(stance_ratio=0.55), hip_xy=_hip)
    period = 0.85
    s.update_pose(0.0, yaw=0.0, base_xy=(0.0, 0.0))
    s.tick(0.05, turn=-0.8, period=period, stance_ratio=0.55)
    t = 0.75 * period
    assert s.predict_force_scale("fl", t, period) == 0.0
    assert s.predict_force_scale("rr", t, period) == 0.0
    assert s.predict_force_scale("fr", t, period) == 1.0
    assert s.predict_force_scale("rl", t, period) == 1.0
