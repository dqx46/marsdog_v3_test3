"""SoftTrot position-layer CoM weight-shift wiring."""

from __future__ import annotations

import os
import sys
import unittest

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (_ROOT, os.path.join(_ROOT, "src")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from marsdog_control.motion.foot_trajectory import (  # noqa: E402
    lateral_offset_soft_trot_com,
    lateral_offset_trot,
)
from marsdog_control.motion.gait_controller import NaturalSoftTrot  # noqa: E402
from marsdog_control.motion.gait_recipes import NATURAL_SOFT_TROT_WBC  # noqa: E402


class SoftTrotComShiftTest(unittest.TestCase):
    def test_recipe_defaults_impedance_overlays(self):
        # Minimal SoftTrot: com_shift only; conflicting overlays off.
        self.assertAlmostEqual(float(NATURAL_SOFT_TROT_WBC["com_shift_m"]), 0.012)
        self.assertAlmostEqual(float(NATURAL_SOFT_TROT_WBC["height"]), 0.25)
        self.assertAlmostEqual(float(NATURAL_SOFT_TROT_WBC["step_h"]), 0.024)
        self.assertAlmostEqual(float(NATURAL_SOFT_TROT_WBC["stance"]), 0.74)
        self.assertAlmostEqual(
            float(NATURAL_SOFT_TROT_WBC.get("lateral_sway", 0.0)), 0.0)
        self.assertAlmostEqual(
            float(NATURAL_SOFT_TROT_WBC.get("rear_clearance_m", 0.0)), 0.0)
        self.assertAlmostEqual(
            float(NATURAL_SOFT_TROT_WBC.get("spine_yaw_deg", 0.0)), 0.0)
        self.assertAlmostEqual(
            float(NATURAL_SOFT_TROT_WBC.get("front_foot_swing_track", 1.0)), 1.0)
        self.assertAlmostEqual(
            float(NATURAL_SOFT_TROT_WBC.get("swing_level", 1.0)), 0.0)

    def test_controller_uses_event_com_not_half_sine(self):
        sway = 0.012
        g = NaturalSoftTrot(
            period=1.0, stance_ratio=0.56,
            lateral_sway=0.0025,  # must be ignored when com_shift_m > 0
            com_shift_m=sway, com_shift_blend=0.12,
        )
        t_mid = 0.25  # FL+RR half → −Y after polarity flip
        got = g._lateral_offset(t_mid)
        self.assertAlmostEqual(got, -sway)
        self.assertAlmostEqual(
            got, lateral_offset_soft_trot_com(t_mid, 1.0, sway))
        self.assertNotAlmostEqual(
            got, lateral_offset_trot(t_mid, 1.0, 0.56, 0.0025))

    def test_com_shift_zero_falls_back_to_lateral_sway(self):
        g = NaturalSoftTrot(
            period=1.0, stance_ratio=0.56,
            lateral_sway=0.008, com_shift_m=0.0,
        )
        t = 0.28
        self.assertAlmostEqual(
            g._lateral_offset(t),
            lateral_offset_trot(t, 1.0, 0.56, 0.008),
        )

    def test_negative_com_shift_flips_sign(self):
        g_pos = NaturalSoftTrot(period=1.0, com_shift_m=0.012)
        g_neg = NaturalSoftTrot(period=1.0, com_shift_m=-0.012)
        t = 0.25  # FL+RR: pos→−Y, neg→+Y
        self.assertAlmostEqual(g_pos._lateral_offset(t), -0.012)
        self.assertAlmostEqual(g_neg._lateral_offset(t), 0.012)


if __name__ == "__main__":
    unittest.main()
