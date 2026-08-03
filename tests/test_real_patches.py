"""Tests for real-patch inventory and --sim-parity."""

from __future__ import annotations

import sys
import unittest
from types import SimpleNamespace

from marsdog_control.apps.walk_cli import parse_args
from marsdog_control.config.real_patches import (
    PATCHES,
    apply_sim_parity,
    patch_status,
)


class RealPatchesTest(unittest.TestCase):
    def test_sim_parity_disables_learning_and_lead(self):
        old = sys.argv
        try:
            sys.argv = [
                "walk", "--natural-soft-trot", "--sim-parity",
                "--no-wbc", "--no-vmc",
            ]
            args = parse_args()
        finally:
            sys.argv = old

        self.assertTrue(args.sim_parity)
        self.assertFalse(args.auto_trim)
        self.assertFalse(args.imu_phase_gate)
        self.assertFalse(args.td_imu_freeze_i)
        self.assertFalse(args.dm_dq_feedforward)
        self.assertFalse(args.ff_decouple)
        self.assertFalse(args.load_trim_cal)
        self.assertFalse(args.save_trim_cal)
        self.assertEqual(float(args.imu_softstart_s), 0.0)
        self.assertEqual(float(args.imu_predict_ms), 0.0)
        self.assertEqual(float(args.tarsus_lead_fl_ms), 0.0)
        self.assertEqual(float(args.tarsus_lead_fr_ms), 0.0)
        self.assertEqual(float(args.swing_level), 0.0)

        # SoftTrot preset must not revive patches (explicit after sim-parity).
        from marsdog_control.apps.walk_cli import apply_preset_preserving_cli
        from marsdog_control.motion.gait_recipes import NATURAL_SOFT_TROT_WBC
        apply_preset_preserving_cli(args, dict(NATURAL_SOFT_TROT_WBC))
        self.assertFalse(args.auto_trim)
        self.assertFalse(args.imu_phase_gate)
        self.assertEqual(float(args.tarsus_lead_fl_ms), 0.0)
        self.assertEqual(float(args.com_shift_m), 0.0)
        self.assertEqual(float(args.rear_clearance_m), 0.0)
        self.assertAlmostEqual(float(args.front_foot_swing_track), 1.0)

        on = {k for k, is_on, _ in patch_status(args) if is_on}
        self.assertEqual(on, set())

    def test_sim_parity_keeps_explicit_reenable(self):
        old = sys.argv
        try:
            sys.argv = [
                "walk", "--natural-soft-trot", "--sim-parity",
                "--auto-trim", "--no-wbc",
            ]
            args = parse_args()
        finally:
            sys.argv = old
        self.assertTrue(args.auto_trim)
        self.assertFalse(args.imu_phase_gate)

    def test_apply_sim_parity_noop_without_flag(self):
        args = SimpleNamespace(
            sim_parity=False,
            auto_trim=True,
            _explicit_cli=set(),
        )
        self.assertEqual(apply_sim_parity(args), [])
        self.assertTrue(args.auto_trim)

    def test_patch_inventory_nonempty(self):
        self.assertGreaterEqual(len(PATCHES), 10)


if __name__ == "__main__":
    unittest.main()
