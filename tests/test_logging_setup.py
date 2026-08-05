"""Offline tests for CSV log setup."""

import csv
import json
import os
import sys
import tempfile
import unittest
from argparse import Namespace

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (_ROOT, os.path.join(_ROOT, "src")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from marsdog_control.io.logging import (  # noqa: E402
    LOG_HEADER, WALK_RECORDER_CSV, WALK_RECORDER_META, LogRuntime, setup_log,
)


class SetupLogTest(unittest.TestCase):
    def test_disabled_returns_empty_handles(self):
        self.assertEqual(setup_log(False, base_dir="."), (None, None, None))

    def test_writes_header_and_metadata(self):
        with tempfile.TemporaryDirectory() as tmp:
            args = Namespace(foo=1, bar={2, 1}, _explicit_cli={"foo"})
            runtime = LogRuntime(
                active_dm_kp_by_id={4: 60.0},
                active_dm_kd_by_id={4: 3.0},
                dm_reference_lead_s={4: 0.01},
                dm_reference_lead_max_rad=0.1,
                dm_dq_feedforward=True,
                dm_dq_max_rps=1.5,
                leg_kp_scale=0.8,
                var_impedance=True,
            )
            fh, writer, path = setup_log(True, args, base_dir=tmp, runtime=runtime)
            self.assertIsNotNone(writer)
            fh.close()

            self.assertEqual(os.path.basename(path), WALK_RECORDER_CSV)
            with open(path, newline="") as csv_file:
                rows = list(csv.reader(csv_file))
            self.assertEqual(rows[0], LOG_HEADER)

            meta_path = os.path.join(os.path.dirname(path), WALK_RECORDER_META)
            with open(meta_path, encoding="utf-8") as meta_file:
                meta = json.load(meta_file)
            self.assertEqual(meta["explicit_cli"], ["foo"])
            self.assertEqual(meta["final_args"]["bar"], [1, 2])
            self.assertEqual(meta["dm"]["kp_by_id"], {"4": 60.0})
            self.assertEqual(meta["leg_kp_scale"], 0.8)
            self.assertTrue(meta["var_impedance"])


class WriteLogFrameTest(unittest.TestCase):
    """target 是 URDF、反馈是电机角时，日志误差必须先做 motor_to_urdf。"""

    def test_sign_and_gear_converted_before_error(self):
        import io
        import math
        from marsdog_control.config.joints import JOINT_BY_ID
        from marsdog_control.core.types import MotorFeedbackFrame, MotorSample
        from marsdog_control.io.logging import WriteLogRuntime, write_log
        from marsdog_control.motion.kinematics import urdf_to_motor

        # fl_calf sign=-1；fl_tarsus sign=-1 gear=2
        joints = [JOINT_BY_ID[3], JOINT_BY_ID[4]]
        targets = {
            3: math.radians(-80.0),
            4: math.radians(45.0),
        }
        # 理想跟踪：电机角 = urdf * sign * gear
        samples = {
            mid: MotorSample(
                motor_id=mid,
                name=JOINT_BY_ID[mid].name,
                position=urdf_to_motor(JOINT_BY_ID[mid], q),
                velocity=0.0,
                torque=0.0,
                timing=(
                    {
                        "command_q": urdf_to_motor(JOINT_BY_ID[mid], q),
                        "command_dq": 0.0,
                        "command_kp": 1.0,
                        "command_kd": 0.1,
                        "command_tau": 0.0,
                    }
                    if JOINT_BY_ID[mid].mtype == "dm"
                    else {}
                ),
            )
            for mid, q in targets.items()
        }
        buf = io.StringIO()
        writer = csv.writer(buf)
        runtime = WriteLogRuntime(
            real_joints=joints,
            dm_fixed_targets={},
            joint_gains={},
            leg_kp_scale=1.0,
        )
        write_log(
            writer, 0.0, "stand", None, None, None, None, targets, 5.0,
            None, 0.0, runtime,
            feedback=MotorFeedbackFrame(samples=samples),
        )
        from marsdog_control.io.logging import LOG_HEADER
        buf2 = io.StringIO(",".join(LOG_HEADER) + "\n" + buf.getvalue())
        rows = {int(r["motor_id"]): r for r in csv.DictReader(buf2)}
        for mid, q in targets.items():
            self.assertAlmostEqual(float(rows[mid]["target_deg"]), math.degrees(q), places=2)
            self.assertAlmostEqual(float(rows[mid]["actual_deg"]), math.degrees(q), places=2)
            self.assertAlmostEqual(float(rows[mid]["error_deg"]), 0.0, places=2)
            # 若未换算，error 会到百度级
            raw_err = math.degrees(samples[mid].position - q)
            self.assertGreater(abs(raw_err), 50.0)


if __name__ == "__main__":
    unittest.main()
