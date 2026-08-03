"""Pin default joint gains (especially front calf / abd compliance)."""

import unittest

from marsdog_control.config.gains import JOINT_GAINS


class JointGainsTest(unittest.TestCase):
    def test_incos_front_symmetric(self):
        for name in ("fl_thigh_roll", "fr_thigh_roll"):
            g = JOINT_GAINS[name]
            self.assertAlmostEqual(g["kp"], 55.0)
            self.assertAlmostEqual(g["kd"], 3.2)
        for name in ("fl_calf", "fr_calf"):
            g = JOINT_GAINS[name]
            self.assertAlmostEqual(g["kp"], 65.0)
            self.assertAlmostEqual(g["kd"], 3.2)
            self.assertAlmostEqual(g["trq_ff"], 0.35)

    def test_lz_rear_sagittal_raised(self):
        for name in ("rl_thigh", "rr_thigh"):
            g = JOINT_GAINS[name]
            self.assertAlmostEqual(g["kp"], 105.0)
            self.assertAlmostEqual(g["kd"], 5.5)
        for name in ("rl_calf", "rr_calf"):
            g = JOINT_GAINS[name]
            self.assertAlmostEqual(g["kp"], 95.0)
            self.assertAlmostEqual(g["kd"], 5.5)

    def test_evo_hips_unchanged(self):
        for name in ("rl_hip", "rr_hip"):
            g = JOINT_GAINS[name]
            self.assertAlmostEqual(g["kp"], 78.0)
            self.assertAlmostEqual(g["kd"], 10.0)


if __name__ == "__main__":
    unittest.main()
