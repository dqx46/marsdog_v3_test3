"""Sim vs real gain tables must stay decoupled."""

from __future__ import annotations

import os
import sys
import unittest

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (_ROOT, os.path.join(_ROOT, "src")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from marsdog_control.config.gains import (  # noqa: E402
    JOINT_GAINS,
    SIM_JOINT_GAINS,
    joint_gains_for,
)


class SimJointGainsTest(unittest.TestCase):
    def test_tables_cover_same_joints(self):
        self.assertEqual(set(JOINT_GAINS), set(SIM_JOINT_GAINS))

    def test_incos_soft_real_not_copied_to_sim_roll(self):
        # Real Incos load gains must not soften MuJoCo thigh_roll.
        self.assertAlmostEqual(JOINT_GAINS["fl_thigh_roll"]["kp"], 55.0)
        self.assertAlmostEqual(JOINT_GAINS["fl_thigh_roll"]["kd"], 3.2)
        self.assertAlmostEqual(JOINT_GAINS["fl_calf"]["kp"], 65.0)
        self.assertAlmostEqual(JOINT_GAINS["fl_calf"]["kd"], 3.2)
        self.assertGreater(SIM_JOINT_GAINS["fl_thigh_roll"]["kp"], 60.0)
        self.assertGreaterEqual(SIM_JOINT_GAINS["fl_thigh_roll"]["kd"], 4.0)

    def test_joint_gains_for_selector(self):
        self.assertIs(joint_gains_for("sim"), SIM_JOINT_GAINS)
        self.assertIs(joint_gains_for("real"), JOINT_GAINS)
        self.assertIs(joint_gains_for("anything"), JOINT_GAINS)


if __name__ == "__main__":
    unittest.main()
