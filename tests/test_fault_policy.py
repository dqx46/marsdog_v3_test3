"""Offline unit tests for `safety/fault_policy.py` (motor fault tiering).

Pins the P1 roadmap item "故障分级策略": losing a leg-bearing motor must
ABORT bring-up (the leg would collapse under load), losing a non-bearing
motor (head/neck/waist) must only DEGRADE (matches the real-machine case
where head_pitch/head_yaw were offline and walking was unaffected).
"""

import os
import sys
import unittest

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (_ROOT, os.path.join(_ROOT, "src")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from marsdog_control.config.joints import JOINT_BY_ID  # noqa: E402
from marsdog_control.safety.fault_policy import (  # noqa: E402
    LEG_CRITICAL_JOINT_NAMES,
    MotorFaultTier,
    classify_motor_fault,
)


def _id_of(name: str) -> int:
    for mid, j in JOINT_BY_ID.items():
        if j.name == name:
            return mid
    raise KeyError(name)


class ClassifyMotorFaultTest(unittest.TestCase):
    def test_no_missing_is_ok(self):
        report = classify_motor_fault([], JOINT_BY_ID)
        self.assertEqual(report.tier, MotorFaultTier.OK)
        self.assertTrue(report.ok_to_stand)

    def test_head_and_neck_offline_is_degraded_not_abort(self):
        # Exactly the real-machine case seen in this session's terminal log:
        # head_pitch(15)/head_yaw(16) offline, walk proceeded fine.
        missing = [_id_of("head_pitch"), _id_of("head_yaw")]
        report = classify_motor_fault(missing, JOINT_BY_ID)
        self.assertEqual(report.tier, MotorFaultTier.DEGRADED)
        self.assertTrue(report.ok_to_stand)
        self.assertEqual(sorted(report.missing_noncritical), sorted(missing))
        self.assertEqual(report.missing_critical, [])

    def test_waist_roll_offline_is_degraded(self):
        report = classify_motor_fault([_id_of("waist_roll")], JOINT_BY_ID)
        self.assertEqual(report.tier, MotorFaultTier.DEGRADED)

    def test_single_leg_motor_offline_aborts(self):
        for name in ("fl_hip_pitch", "fl_calf", "fl_tarsus", "rr_hip", "rr_thigh"):
            with self.subTest(name=name):
                report = classify_motor_fault([_id_of(name)], JOINT_BY_ID)
                self.assertEqual(report.tier, MotorFaultTier.ABORT)
                self.assertFalse(report.ok_to_stand)
                self.assertEqual(report.missing_critical, [_id_of(name)])

    def test_mixed_critical_and_noncritical_still_aborts(self):
        missing = [_id_of("head_pitch"), _id_of("rl_calf")]
        report = classify_motor_fault(missing, JOINT_BY_ID)
        self.assertEqual(report.tier, MotorFaultTier.ABORT)
        self.assertFalse(report.ok_to_stand)
        self.assertIn(_id_of("rl_calf"), report.missing_critical)
        self.assertIn(_id_of("head_pitch"), report.missing_noncritical)

    def test_describe_mentions_joint_names(self):
        report = classify_motor_fault([_id_of("fr_calf")], JOINT_BY_ID)
        text = report.describe(JOINT_BY_ID)
        self.assertIn("fr_calf", text)
        self.assertIn("ABORT", text)

    def test_leg_critical_set_matches_all_load_bearing_joints(self):
        # Every leg-critical name must actually resolve to a real joint id
        # (catches typos immediately, without touching hardware).
        for name in LEG_CRITICAL_JOINT_NAMES:
            self.assertIsNotNone(_id_of(name))
        # 4 legs; front legs have an active tarsus (4 joints), rear legs don't (3).
        self.assertEqual(len(LEG_CRITICAL_JOINT_NAMES), 14)


if __name__ == "__main__":
    unittest.main()
