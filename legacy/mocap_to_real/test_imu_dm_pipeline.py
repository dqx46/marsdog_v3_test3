import argparse
import math
import struct
import time
import unittest
from types import SimpleNamespace

from imu_controller import ImuAttitudeController
from imu_wt901 import _ema_alpha
from gait_controller import NaturalSoftTrot, StandController
from kinematics import front_foot_pitch, motor_to_urdf
from joint_config import JOINT_BY_NAME
from motor_damiao import (
    DM_S2325_DQMAX,
    DM_S2325_PMAX,
    DM_S2325_TAUMAX,
    MotorDamiao,
    _float_to_uint,
)
from robot_types import Direction, RobotMode
import walk
from walk import apply_preset_preserving_cli, tarsus_bench_reference


class TestTimeInvariantImu(unittest.TestCase):
    def test_dt_filter_is_sample_rate_independent(self):
        def run(dt):
            y = 0.0
            for _ in range(round(1.0 / dt)):
                a = _ema_alpha(dt, 0.025)
                y += a * (1.0 - y)
            return y

        self.assertAlmostEqual(run(0.005), run(0.010), delta=0.004)

    def test_integrator_and_leak_are_dt_invariant(self):
        def run(dt):
            c = ImuAttitudeController(
                kp_roll=0.0, kp_pitch=0.0, ki_roll=0.01, ki_pitch=0.0,
                kd_roll=0.0, kd_pitch=0.0, deadzone_deg=0.0,
                max_correction=1.0, predict_lead_s=0.0,
            )
            c.enable()
            for _ in range(round(1.0 / dt)):
                c.update(0.1, 0.0, 0.0, 0.0, dt_s=dt)
            return c.roll_out

        self.assertAlmostEqual(run(0.005), run(0.010), delta=2e-5)

    def test_dynamic_prediction_clamps_and_rejects_stale_gyro(self):
        c = ImuAttitudeController(
            kp_roll=1.0, kp_pitch=0.0, ki_roll=0.0, ki_pitch=0.0,
            kd_roll=0.0, kd_pitch=0.0, deadzone_deg=0.0,
            max_correction=1.0, predict_lead_s=0.010,
            prediction_max_s=0.050, gyro_max_age_s=0.030,
        )
        c.enable()
        c.update(0.0, 0.0, 1.0, 0.0, angle_age_s=0.2, gyro_age_s=0.01)
        self.assertAlmostEqual(c.prediction_lead_s, 0.050)
        self.assertAlmostEqual(c.roll_out, 0.050, places=6)
        c.update(0.0, 0.0, 1.0, 0.0, angle_age_s=0.2, gyro_age_s=0.04)
        self.assertEqual(c.prediction_lead_s, 0.0)
        self.assertAlmostEqual(c.roll_out, 0.0, places=6)


class RecordingDamiao(MotorDamiao):
    def __init__(self):
        super().__init__()
        self.sent = []

    def control_mit(self, slave_id, kp, kd, q, dq, tau):
        self.sent.append((slave_id, kp, kd, q, dq, tau))


class TestDamiaoWorker(unittest.TestCase):
    def test_latest_command_overwrites_before_send(self):
        dm = RecordingDamiao()
        dm.add_motor(4, master_id=0x63)
        dm.add_motor(8, master_id=0x63)
        dm.set_commands({4: (10, 1, 1.0, 0.1, 0.0)})
        dm.set_commands({4: (20, 2, 2.0, 0.2, 0.0)})
        dm.start_worker()
        time.sleep(0.03)
        dm.stop_worker()
        self.assertTrue(dm.sent)
        self.assertEqual(dm.sent[-1][3], 2.0)

    def test_expected_id_owns_feedback_with_shared_master_id(self):
        dm = MotorDamiao()
        dm.add_motor(4, master_id=0x63)
        dm.add_motor(8, master_id=0x63)
        q = _float_to_uint(0.5, -DM_S2325_PMAX, DM_S2325_PMAX, 16)
        dq = _float_to_uint(0.0, -DM_S2325_DQMAX, DM_S2325_DQMAX, 12)
        tau = _float_to_uint(0.0, -DM_S2325_TAUMAX, DM_S2325_TAUMAX, 12)
        data = bytearray(8)
        data[1], data[2] = (q >> 8) & 0xFF, q & 0xFF
        data[3] = dq >> 4
        data[4] = ((dq & 0x0F) << 4) | ((tau >> 8) & 0x0F)
        data[5] = tau & 0xFF
        self.assertEqual(dm._process_feedback(0x63, data, expected_slave_id=8), 8)
        self.assertNotEqual(dm.get_position(8), 0.0)
        self.assertEqual(dm.get_position(4), 0.0)
        self.assertGreater(dm.get_timing(8)["feedback_seq"], 0)


class TestPresetPrecedence(unittest.TestCase):
    def test_explicit_cli_wins_over_preset(self):
        args = argparse.Namespace(height=0.27, nat_period=0.8,
                                  _explicit_cli={"height"})
        changed = apply_preset_preserving_cli(
            args, {"height": 0.24, "nat_period": 0.9})
        self.assertEqual(changed, ["height"])
        self.assertEqual(args.height, 0.27)
        self.assertEqual(args.nat_period, 0.9)

    def test_bench_reference_starts_and_ends_at_zero(self):
        frequencies = [0.5, 1.0]
        self.assertEqual(
            tarsus_bench_reference(0.5, frequencies, 0.1, cycles=2, settle_s=1)[0],
            0.0,
        )
        total = (1 + 2 / 0.5) + (1 + 2 / 1.0)
        delta, frequency, done = tarsus_bench_reference(
            total, frequencies, 0.1, cycles=2, settle_s=1)
        self.assertEqual(delta, 0.0)
        self.assertEqual(frequency, 0.0)
        self.assertTrue(done)


class TestRuntimeFacts(unittest.TestCase):
    def test_rate_limit_ids_match_documented_roll_and_rear_hip_joints(self):
        self.assertEqual(walk._RATELIMIT_IDS, {2, 6, 9, 12})

    def test_front_foot_orientation_survives_natural_visual_layers(self):
        gait = NaturalSoftTrot(
            body_height=0.24, period=0.9, stance_ratio=0.66,
            amp_front=0.026, amp_rear=0.026,
            step_height=0.016, step_height_front=0.014,
            ramp_duration=0.0,
            front_foot_track_deg=-78.0,
            front_foot_stance_push_deg=10.0,
            front_foot_swing_track=0.0,
            front_stand_foot_pitch_deg=-90.0,
            thigh_swing_front_deg=8.0,
            tarsus_swing_deg=4.0,
            retract_front=0.018,
        )
        pitches = []
        for index in range(200):
            targets = gait.get_targets(gait.period * index / 200.0)
            for leg in ("fl", "fr"):
                joints = [
                    JOINT_BY_NAME[f"{leg}_hip_pitch"],
                    JOINT_BY_NAME[f"{leg}_calf"],
                    JOINT_BY_NAME[f"{leg}_tarsus"],
                ]
                urdf = [motor_to_urdf(joint, targets[joint.motor_id])
                        for joint in joints]
                pitches.append(math.degrees(front_foot_pitch(*urdf)))
        self.assertGreaterEqual(min(pitches), -90.01)
        self.assertLessEqual(max(pitches), -77.99)

    def test_stand_imu_correction_preserves_front_foot_pitch(self):
        stand = StandController(
            body_height=0.24, front_stand_foot_pitch_deg=-90.0)
        targets = stand.get_targets(0.0)
        walk._apply_stand_imu_dz(
            stand, targets,
            {"fl": -0.010, "fr": 0.010, "rl": 0.0, "rr": 0.0},
        )
        for leg in ("fl", "fr"):
            values = {
                JOINT_BY_NAME[name].motor_id:
                    targets[JOINT_BY_NAME[name].motor_id]
                for name in (
                    f"{leg}_hip_pitch", f"{leg}_calf", f"{leg}_tarsus",
                )
            }
            self.assertAlmostEqual(
                walk.front_foot_pitch_from_motor(leg, values),
                -90.0,
                places=6,
            )

    def test_mode_string_uses_actual_controller(self):
        nat = object()
        fsm = SimpleNamespace(
            mode=RobotMode.STAND,
            active_gait=nat,
            nat_fwd=nat,
            pace_fwd=object(),
            pace_bwd=object(),
            trot_fwd=object(),
            trot_bwd=object(),
            direction=Direction.FWD,
        )
        self.assertEqual(walk._mode_str(fsm), "natural_fwd")
        fsm.active_gait = None
        fsm.mode = RobotMode.NATURAL
        self.assertEqual(walk._mode_str(fsm), "stand")

    # NOTE: the former ``test_tarsus_lead_is_disabled_outside_gait`` moved to
    # ``tests/test_dm_reference_lead.py`` — it now exercises the live
    # ``hardware.actuation.send_all`` + ``WalkRuntimeState`` path instead of the
    # removed ``walk.DM_*`` module globals / pre-incos ``send_all`` signature.


if __name__ == "__main__":
    unittest.main()
