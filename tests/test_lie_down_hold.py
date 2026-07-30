"""Unit tests for sit/lie hold handoff helpers."""

from __future__ import annotations

import unittest

from marsdog_control.runtime.lie_down_session import LieDownSession


class LieDownHoldGravBlendTest(unittest.TestCase):
    def _session(self) -> LieDownSession:
        return LieDownSession(
            build_target=lambda online: {},
            read_positions=lambda *a, **k: {},
            smooth_transition=lambda *a, **k: True,
            dm_fixed_targets={},
            grav_ramp_s=1.0,
        )

    def test_blend_ramps_after_enter_hold(self):
        s = self._session()
        self.assertEqual(s.hold_grav_blend(100.0), 1.0)  # not holding
        s._enter_hold("lie_down", {1: 0.1}, mono=10.0)
        self.assertAlmostEqual(s.hold_grav_blend(10.0), 0.0)
        mid = s.hold_grav_blend(10.5)
        self.assertTrue(0.0 < mid < 1.0)
        self.assertAlmostEqual(s.hold_grav_blend(11.0), 1.0)
        self.assertAlmostEqual(s.hold_grav_blend(12.0), 1.0)

    def test_clear_hold_resets_blend(self):
        s = self._session()
        s._enter_hold("sit", {1: 0.2}, mono=5.0)
        s._clear_hold()
        self.assertFalse(s.hold)
        self.assertIsNone(s.hold_t0)
        self.assertEqual(s.hold_grav_blend(6.0), 1.0)


if __name__ == "__main__":
    unittest.main()
