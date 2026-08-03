"""Stand abduction-flare toggle for joint-direction verification."""

from __future__ import annotations

import unittest
from types import SimpleNamespace
from unittest import mock

from marsdog_control.runtime.walk_loop import clear_abd_flare, toggle_abd_flare


class _Stand:
    def __init__(self, hip_abduction: float = 0.08):
        self.hip_abduction = hip_abduction
        self.updated = 0

    def set_hip_abduction(self, hip_abduction: float):
        self.hip_abduction = float(hip_abduction)
        self.updated += 1


def _ctx(*, gait=None, hold=False, abd=0.08):
    return SimpleNamespace(
        stand=_Stand(abd),
        fsm=SimpleNamespace(active_gait=gait),
        lie_down_session=SimpleNamespace(hold=hold),
        abd_flare_active=False,
        abd_flare_base=None,
        abd_flare_rad=0.16,
    )


class AbdFlareTest(unittest.TestCase):
    def test_toggle_on_off_in_stand(self):
        ctx = _ctx()
        with mock.patch("builtins.print"):
            toggle_abd_flare(ctx)
        self.assertTrue(ctx.abd_flare_active)
        self.assertAlmostEqual(ctx.abd_flare_base, 0.08)
        self.assertAlmostEqual(ctx.stand.hip_abduction, 0.16)
        with mock.patch("builtins.print"):
            toggle_abd_flare(ctx)
        self.assertFalse(ctx.abd_flare_active)
        self.assertIsNone(ctx.abd_flare_base)
        self.assertAlmostEqual(ctx.stand.hip_abduction, 0.08)

    def test_ignored_while_walking(self):
        ctx = _ctx(gait=object())
        with mock.patch("builtins.print"):
            toggle_abd_flare(ctx)
        self.assertFalse(ctx.abd_flare_active)
        self.assertAlmostEqual(ctx.stand.hip_abduction, 0.08)

    def test_clear_on_gait_entry(self):
        ctx = _ctx()
        with mock.patch("builtins.print"):
            toggle_abd_flare(ctx)
            clear_abd_flare(ctx, reason="进入步态")
        self.assertFalse(ctx.abd_flare_active)
        self.assertAlmostEqual(ctx.stand.hip_abduction, 0.08)


if __name__ == "__main__":
    unittest.main()
