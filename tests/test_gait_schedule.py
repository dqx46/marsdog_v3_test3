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
