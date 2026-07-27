"""Pin default joint gains (especially front calf compliance)."""

import unittest

from marsdog_control.config.gains import JOINT_GAINS


class JointGainsTest(unittest.TestCase):
    def test_front_calf_soft_gains(self):
        g_fl = JOINT_GAINS["fl_calf"]
        self.assertAlmostEqual(g_fl["kp"], 70.0)
        self.assertAlmostEqual(g_fl["kd"], 1.5)
        self.assertAlmostEqual(g_fl["trq_ff"], 0.35)
        
        g_fr = JOINT_GAINS["fr_calf"]
        self.assertAlmostEqual(g_fr["kp"], 90.0)
        self.assertAlmostEqual(g_fr["kd"], 2.0)
        self.assertAlmostEqual(g_fr["trq_ff"], 0.40)


if __name__ == "__main__":
    unittest.main()
