"""Unit tests for SI VelocityCommand → SoftTrotSchedule."""

from __future__ import annotations

from marsdog_control.motion.gait_schedule import (
    GaitEnvelope,
    SoftTrotSchedule,
    VelocityCommand,
    apply_schedule_to_gait,
)


def _env(**kwargs) -> GaitEnvelope:
    base = dict(
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
    base.update(kwargs)
    return GaitEnvelope(**base)


def test_schedule_scales_amp_period_stance_with_speed():
    env = _env()
    sch = SoftTrotSchedule(env)
    vmax = sch.max_forward_vx()
    assert vmax > 0.05

    crawl = sch.map(VelocityCommand(vx=0.35 * vmax))
    mid = sch.map(VelocityCommand(vx=0.55 * vmax))
    full = sch.map(VelocityCommand(vx=vmax))
    stop = sch.map(VelocityCommand(vx=0.0))

    assert crawl.speed_frac >= 0.5
    assert mid.speed_frac > crawl.speed_frac
    assert abs(full.speed_frac - 1.0) < 1e-6
    assert abs(abs(full.amp_front) - env.amp_front_max) < 1e-9
    assert abs(crawl.amp_front) < abs(full.amp_front)
    assert full.period <= crawl.period + 1e-9
    assert full.stance_ratio <= crawl.stance_ratio + 1e-9
    assert stop.speed_frac == 0.0
    assert stop.amp_front == 0.0
    assert full.vel_cmd[0] > mid.vel_cmd[0] > crawl.vel_cmd[0] > 0.0
    assert abs(full.step_height - env.step_h_rear_max) < 1e-9
    # Crawl: lift tracks speed_frac (scuff floor), not the high anti-limp floor
    assert crawl.step_height <= env.step_h_rear_max * crawl.speed_frac + 1e-9
    assert crawl.step_height >= 0.008
    assert crawl.step_height_front >= 0.006
    assert stop.step_height == 0.0
    assert mid.vel_cmd[0] >= 0.50 * full.vel_cmd[0]


def test_schedule_turn_and_reverse():
    sch = SoftTrotSchedule(_env())
    vmax = sch.max_forward_vx()
    left = sch.map(VelocityCommand(vx=0.8 * vmax, yaw_rate=-0.2))
    right = sch.map(VelocityCommand(vx=0.8 * vmax, yaw_rate=0.2))
    back = sch.map(VelocityCommand(vx=-0.8 * vmax))
    assert left.turn_cmd < 0
    assert right.turn_cmd > 0
    assert abs(left.vel_cmd[2] + 0.2) < 1e-9
    assert abs(right.vel_cmd[2] - 0.2) < 1e-9
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
    sch = SoftTrotSchedule(_env())
    sched = sch.map(VelocityCommand(vx=sch.max_forward_vx(), yaw_rate=0.2))
    apply_schedule_to_gait(g, sched)
    assert g.amp_front == sched.amp_front
    assert g.step_height == sched.step_height
    assert g.step_height_front == sched.step_height_front
    assert g.period == sched.period
    assert g.stance_ratio == sched.stance_ratio
    assert g.vel_cmd == sched.vel_cmd
    assert g.speed_frac == sched.speed_frac


def test_schedule_spot_turn_is_abduction_led():
    """vx≈0 + yaw_rate → Unitree spot: amp=0; Spot 几何与前进/走+转解耦。"""
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
        # Spot 显式独立值（故意不同于前进 step_h / period）
        spot_period=1.60,
        spot_stance=0.55,
        spot_step_h_front=0.022,
        spot_step_h_rear=0.022,
        spot_yaw_step_rad=0.45,
    )
    sch = SoftTrotSchedule(env)
    spot = sch.map(VelocityCommand(vx=0.0, yaw_rate=0.32))
    assert spot.spot_turn is True
    assert spot.amp_front == 0.0 and spot.amp_rear == 0.0
    assert spot.turn_amp_diff == 0.0
    assert abs(spot.spot_yaw_step - 0.45) < 1e-9
    assert spot.spot_y_hold_max >= 0.04
    # Spot 用自己的 period/step_h，不吃前进 0.58 / 0.045
    assert abs(spot.period - 1.60) < 1e-9
    assert abs(spot.stance_ratio - 0.55) < 1e-9
    assert abs(spot.step_height_front - 0.022) < 1e-9
    assert abs(spot.step_height - 0.022) < 1e-9
    assert abs(spot.vel_cmd[0]) < 1e-9
    assert spot.vel_cmd[2] > 0.15
    assert spot.spot_dx_scale == 0.0
    assert spot.turn_cmd > 0

    left = sch.map(VelocityCommand(vx=0.0, yaw_rate=-0.32))
    assert left.turn_cmd < 0
    assert left.vel_cmd[2] < 0

    idle = sch.map(VelocityCommand(vx=0.0, yaw_rate=0.04))
    assert idle.spot_turn is False
    assert idle.amp_front == 0.0

    # 走+转仍吃 cruise turn_*；周期走前进层，绝不是 Spot 的 1.60
    cruise = sch.map(VelocityCommand(vx=0.10, yaw_rate=0.20))
    assert cruise.spot_turn is False
    assert abs(cruise.turn_amp_diff - 0.012) < 1e-9
    assert abs(cruise.turn_y_amp - 0.040) < 1e-9
    assert abs(cruise.period - 1.60) > 0.2



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
    ).map(VelocityCommand(vx=0.0, yaw_rate=0.40))
    apply_schedule_to_gait(g, sched)
    assert g.max_turn_amp_diff == 0.0
    assert g.vel_cmd[2] != 0.0
    assert g.spot_turn_active is True
    assert g.spot_yaw_step_rad == sched.spot_yaw_step
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
    full = sched.map(VelocityCommand(vx=sched.max_forward_vx()))
    avg_amp = 0.5 * (abs(full.amp_front) + abs(full.amp_rear))
    vx_kin = 2.0 * avg_amp / full.period
    assert abs(full.vel_cmd[0] - (vx_kin + VX_SCRUB_OFFSET_MPS)) < 1e-6


def test_legacy_norm_maps_to_si_default_cruise():
    env = GaitEnvelope.from_wbc_soft_trot(
        amp_front=0.050,
        amp_rear=0.068,
        period=0.58,
        stance=0.56,
        throttle_min_scale=0.45,
    )
    sch = SoftTrotSchedule(env)
    vx = sch.vx_at_legacy_norm(0.5)
    assert 0.12 < vx < 0.15
