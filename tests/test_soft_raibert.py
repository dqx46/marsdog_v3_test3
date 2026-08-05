"""SoftTrot sagittal Raibert placement wiring."""

import os
import sys
import unittest

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (_ROOT, os.path.join(_ROOT, "src")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from marsdog_control.motion.gait_controller import NaturalSoftTrot  # noqa: E402


class SoftRaibertWiringTest(unittest.TestCase):
    def _soft(self, **kwargs):
        defaults = dict(
            amp_front=0.022,
            amp_rear=0.030,
            period=1.20,
            stance_ratio=0.72,
            raibert_enabled=True,
            raibert_kx=0.05,
            raibert_dx_max=0.03,
        )
        defaults.update(kwargs)
        return NaturalSoftTrot(**defaults)

    def test_raibert_amp_from_vel_cmd_not_schedule_amp(self):
        g = self._soft()
        g.amp_front = 0.0  # schedule idle must not zero Raibert amp
        g.amp_rear = 0.0
        g.vel_cmd = (0.10, 0.0, 0.0)
        g._update_raibert_placement({"vel_xyz": (0.10, 0.0, 0.0)})
        self.assertTrue(g._raibert_use_amp)
        self.assertAlmostEqual(g._raibert_amp_front, 0.022, places=5)
        self.assertAlmostEqual(g._raibert_amp_rear, 0.030, places=5)
        self.assertAlmostEqual(g._raibert_dx, 0.0, places=9)

    def test_raibert_dx_from_speed_error(self):
        g = self._soft()
        g.vel_cmd = (0.10, 0.0, 0.0)
        g._update_raibert_placement({"vel_xyz": (0.20, 0.0, 0.0)})
        self.assertAlmostEqual(g._raibert_dx, 0.005, places=6)

    def test_raibert_disabled_skips_placement(self):
        g = self._soft(raibert_enabled=False)
        g.vel_cmd = (0.10, 0.0, 0.0)
        g._update_raibert_placement({"vel_xyz": (0.20, 0.0, 0.0)})
        self.assertFalse(g._raibert_use_amp)
        self.assertEqual(g._raibert_dx, 0.0)

    def test_get_targets_uses_raibert_without_legacy_double_dx(self):
        g = self._soft()
        g.vel_cmd = (0.10, 0.0, 0.0)
        g.ramp_duration = 0.0
        # Over-speed → positive dx_td inside Soft X; legacy path must stay 0.
        targets = g.get_targets(
            0.5, imu_state={"roll": 0.0, "gyro_roll": 0.0, "vel_xyz": (0.20, 0.0, 0.0)})
        self.assertTrue(g._raibert_use_amp)
        self.assertAlmostEqual(g._raibert_dx, 0.005, places=6)
        self.assertIsInstance(targets, dict)
        self.assertGreater(len(targets), 0)


if __name__ == "__main__":
    unittest.main()
