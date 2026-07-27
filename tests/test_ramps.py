"""Offline unit tests for the control soft-start ramps.

Pins `softstart_gain` against the exact inline smoothstep formula that used to
live in walk.py's main loop (ss_gain / trim_gain), proving the extraction is
byte-identical and needs no robot to verify.
"""

import os
import sys
import unittest

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (_ROOT, os.path.join(_ROOT, "src")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from marsdog_control.control.ramps import smoothstep, softstart_gain  # noqa: E402


def _legacy_ss_gain(elapsed, duration):
    """The original inline formula from walk.main (ss_gain / trim_gain)."""
    gain = 1.0
    ss = elapsed / duration
    if ss < 1.0:
        ss = max(0.0, ss)
        gain = 3 * ss * ss - 2 * ss * ss * ss
    return gain


class SmoothstepTest(unittest.TestCase):
    def test_endpoints_and_mid(self):
        self.assertAlmostEqual(smoothstep(0.0), 0.0)
        self.assertAlmostEqual(smoothstep(1.0), 1.0)
        self.assertAlmostEqual(smoothstep(0.5), 0.5)


class SoftstartGainTest(unittest.TestCase):
    def test_disabled_when_duration_nonpositive(self):
        self.assertEqual(softstart_gain(0.0, 0.0), 1.0)
        self.assertEqual(softstart_gain(0.5, -1.0), 1.0)

    def test_full_authority_after_duration(self):
        self.assertEqual(softstart_gain(2.0, 1.5), 1.0)
        self.assertEqual(softstart_gain(1.5, 1.5), 1.0)

    def test_clamped_below_zero(self):
        self.assertEqual(softstart_gain(-0.3, 1.5), 0.0)

    def test_matches_legacy_inline_formula(self):
        for elapsed in (0.0, 0.15, 0.375, 0.75, 1.1, 1.49):
            for duration in (1.5, 3.0, 0.9):
                self.assertAlmostEqual(
                    softstart_gain(elapsed, duration),
                    _legacy_ss_gain(elapsed, duration),
                    msg=f"elapsed={elapsed} duration={duration}")


if __name__ == "__main__":
    unittest.main()
