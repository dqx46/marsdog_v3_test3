"""Unit tests for VelocityCommand → SoftTrotSchedule."""

from __future__ import annotations

from marsdog_control.motion.gait_schedule import (
    GaitEnvelope,
    SoftTrotSchedule,
    VelocityCommand,
    apply_schedule_to_gait,
)


def test_schedule_scales_amp_period_stance_with_speed():
    env = GaitEnvelope(
        amp_front_max=0.032,
        amp_rear_max=0.036,
        step_h_front_max=0.024,
        step_h_rear_max=0.034,
        step_h_front_floor=0.014,
        step_h_rear_floor=0.022,
        period_nom=1.05,
        period_min=0.95,
        period_max=1.20,
        stance_nom=0.80,
        stance_min=0.76,
        stance_max=0.84,
        throttle_min_scale=0.5,
        vx_engage=0.30,
        vx_deadzone=0.12,
    )
    sch = SoftTrotSchedule(env)

    crawl = sch.map(VelocityCommand(vx=0.31))  # mid stick (linear from deadzone)
    mid = sch.map(VelocityCommand(vx=0.55))
    full = sch.map(VelocityCommand(vx=1.0))
    stop = sch.map(VelocityCommand(vx=0.0))

    assert crawl.speed_frac >= 0.5
    assert mid.speed_frac > crawl.speed_frac
    assert full.speed_frac == 1.0
    assert abs(full.amp_front) == env.amp_front_max
    assert abs(crawl.amp_front) < abs(full.amp_front)
    assert full.period <= crawl.period  # faster cadence at full
    assert full.stance_ratio <= crawl.stance_ratio
    assert stop.speed_frac == 0.0
    assert stop.amp_front == 0.0
    assert full.vel_cmd[0] > mid.vel_cmd[0] > crawl.vel_cmd[0] > 0.0
    assert full.step_height == env.step_h_rear_max
    # Crawl must keep clearance floor (anti-limp), not pure speed_frac*max
    assert crawl.step_height >= env.step_h_rear_floor
    assert crawl.step_height_front >= env.step_h_front_floor
    assert stop.step_height == 0.0
    # Mid-stick must keep meaningful authority (old engage-based map
    # compressed 0.55→1.0 into ~20% of amp span).
    assert mid.vel_cmd[0] >= 0.55 * full.vel_cmd[0]


def test_schedule_turn_and_reverse():
    sch = SoftTrotSchedule(GaitEnvelope())
    left = sch.map(VelocityCommand(vx=0.8, yaw_rate=-0.5))
    right = sch.map(VelocityCommand(vx=0.8, yaw_rate=0.5))
    back = sch.map(VelocityCommand(vx=-0.8))
    assert left.turn_cmd < 0
    assert right.turn_cmd > 0
    assert back.amp_front < 0
    assert back.vel_cmd[0] < 0


def test_apply_schedule_sets_vel_cmd():
    class _Gait:
        def __init__(self):
            self.amp_front = 0.0
            self.amp_rear = 0.0
            self.step_height = 0.0
            self.step_height_front = 0.0
            self.period = 1.0
            self.stance_ratio = 0.7
            self.turn_cmd = 0.0
            self.turn_y_gain = 0.0
            self.vel_cmd = (0.0, 0.0, 0.0)

        def set_period(self, p):
            self.period = p

    g = _Gait()
    sched = SoftTrotSchedule().map(VelocityCommand(vx=1.0, yaw_rate=0.2))
    apply_schedule_to_gait(g, sched)
    assert g.amp_front == sched.amp_front
    assert g.step_height == sched.step_height
    assert g.step_height_front == sched.step_height_front
    assert g.period == sched.period
    assert g.stance_ratio == sched.stance_ratio
    assert g.vel_cmd == sched.vel_cmd
    assert g.speed_frac == sched.speed_frac


def test_schedule_spot_turn_is_abduction_led():
    """vx≈0 + yaw → Unitree spot: amp=0, abduct budget, real wz."""
    env = GaitEnvelope.from_wbc_soft_trot(
        amp_front=0.050,
        amp_rear=0.068,
        period=0.58,
        stance=0.56,
        step_h_front=0.045,
        step_h_rear=0.048,
        turn_y_amp=0.040,
        turn_amp_diff=0.012,
        vx_deadzone=0.12,
    )
    sch = SoftTrotSchedule(env)
    spot = sch.map(VelocityCommand(vx=0.0, yaw_rate=0.8))
    assert spot.spot_turn is True
    assert spot.amp_front == 0.0 and spot.amp_rear == 0.0
    assert spot.turn_amp_diff == 0.0
    assert abs(spot.spot_yaw_step - 0.45) < 1e-9
    assert spot.spot_y_hold_max >= 0.04
    assert abs(spot.period - 0.85) < 1e-9
    assert abs(spot.stance_ratio - 0.55) < 1e-9
    assert 0.035 <= spot.step_height_front <= 0.055
    assert abs(spot.vel_cmd[0]) < 1e-9
    assert spot.vel_cmd[2] > 0.15
    assert spot.spot_dx_scale == 0.0
    assert spot.turn_cmd > 0

    left = sch.map(VelocityCommand(vx=0.0, yaw_rate=-0.8))
    assert left.turn_cmd < 0
    assert left.vel_cmd[2] < 0

    idle = sch.map(VelocityCommand(vx=0.0, yaw_rate=0.05))
    assert idle.spot_turn is False
    assert idle.amp_front == 0.0


def test_apply_schedule_sets_turn_geometry():
    class _Gait:
        def __init__(self):
            self.amp_front = 0.0
            self.amp_rear = 0.0
            self.step_height = 0.0
            self.step_height_front = 0.0
            self.period = 1.0
            self.stance_ratio = 0.7
            self.turn_cmd = 0.0
            self.turn_y_gain = 0.0
            self.vel_cmd = (0.0, 0.0, 0.0)
            self.max_turn_y_amp = 0.01
            self.max_turn_amp_diff = 0.02
            self.spot_turn_active = False
            self.spot_yaw_step_rad = 0.1
            self.spot_dx_scale = 0.3
            self._PHASE_OFFSET = {"fl": 0.0, "rr": 0.06, "fr": 0.50, "rl": 0.56}
            self._PHASE_OFFSET_CRUISE = dict(self._PHASE_OFFSET)

        def set_period(self, p):
            self.period = p

        def _clear_spot_state(self):
            pass

    g = _Gait()
    sched = SoftTrotSchedule(
        GaitEnvelope(vx_deadzone=0.12)
    ).map(VelocityCommand(vx=0.0, yaw_rate=1.0))
    apply_schedule_to_gait(g, sched)
    assert g.max_turn_amp_diff == 0.0
    assert g.vel_cmd[2] != 0.0
    assert g.spot_turn_active is True
    assert g.spot_yaw_step_rad == sched.spot_yaw_step
    # Diagonal trot phases for Unitree turn.
    assert abs(g._PHASE_OFFSET["fl"] - 0.00) < 1e-9
    assert abs(g._PHASE_OFFSET["rr"] - 0.00) < 1e-9
    assert abs(g._PHASE_OFFSET["fr"] - 0.50) < 1e-9
    assert abs(g._PHASE_OFFSET["rl"] - 0.50) < 1e-9
    idle = SoftTrotSchedule(GaitEnvelope(vx_deadzone=0.12)).map(
        VelocityCommand(vx=0.0, yaw_rate=0.0)
    )
    apply_schedule_to_gait(g, idle)
    assert abs(g._PHASE_OFFSET["rr"] - 0.06) < 1e-9
    assert abs(g._PHASE_OFFSET["rl"] - 0.56) < 1e-9


def test_schedule_vel_cmd_includes_scrub_offset():
    from marsdog_control.control.velocity_model import VX_SCRUB_OFFSET_MPS

    env = GaitEnvelope(
        amp_front_max=0.058,
        amp_rear_max=0.065,
        period_nom=0.92,
        period_min=0.83,
        period_max=1.06,
        stance_nom=0.79,
        throttle_min_scale=0.35,
        vx_deadzone=0.12,
    )
    sched = SoftTrotSchedule(env)
    full = sched.map(VelocityCommand(vx=1.0))
    avg_amp = 0.5 * (abs(full.amp_front) + abs(full.amp_rear))
    vx_kin = 2.0 * avg_amp / full.period
    assert abs(full.vel_cmd[0] - (vx_kin + VX_SCRUB_OFFSET_MPS)) < 1e-6
