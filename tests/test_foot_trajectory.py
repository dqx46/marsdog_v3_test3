"""Offline unit tests for `motion/foot_trajectory.py`'s pure trajectory functions.

Phase O pulled these out of `StableTrot`/`NaturalTrot`/`NaturalSoftTrot`
precisely so they could be pinned down and reasoned about without building a
full gait controller instance. These tests check continuity/boundary
properties that the numeric before/after sweep (see REFACTOR_STATUS.md Phase
O) already proved are behavior-preserving; here we pin the *shape* contracts
so a future edit that breaks C0/C1 continuity at swing/stance boundaries
fails fast, locally, without needing the robot.
"""

import os
import sys
import unittest

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (_ROOT, os.path.join(_ROOT, "src")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from marsdog_control.motion import foot_trajectory as ft  # noqa: E402


class SwingZTest(unittest.TestCase):
    def test_three_phase_swing_z_endpoints_zero(self):
        self.assertAlmostEqual(ft.three_phase_swing_z(0.0, 0.02), 0.0)
        self.assertAlmostEqual(ft.three_phase_swing_z(1.0, 0.02), 0.0)

    def test_three_phase_swing_z_cruise_plateau(self):
        # Between rise_end=0.4 and cruise_end=0.7, height is exactly step_h.
        for swing_t in (0.4, 0.5, 0.6, 0.69):
            self.assertAlmostEqual(ft.three_phase_swing_z(swing_t, 0.03), 0.03)

    def test_sin2_swing_z_endpoints_zero_and_peak_at_half(self):
        self.assertAlmostEqual(ft.sin2_swing_z(0.0, 0.02), 0.0)
        self.assertAlmostEqual(ft.sin2_swing_z(1.0, 0.02), 0.0)
        self.assertAlmostEqual(ft.sin2_swing_z(0.5, 0.02), 0.02)

    def test_minimum_jerk_bump_endpoints_zero(self):
        self.assertAlmostEqual(ft.minimum_jerk_bump(0.0, 0.45), 0.0)
        self.assertAlmostEqual(ft.minimum_jerk_bump(1.0, 0.45), 0.0)

    def test_minimum_jerk_swing_z_peak_matches_step_height(self):
        peak = 0.45
        z_at_peak = ft.minimum_jerk_swing_z(peak, 0.02, peak)
        self.assertAlmostEqual(z_at_peak, 0.02)


class MinimumJerkTest(unittest.TestCase):
    def test_monotonic_and_endpoint_derivative_zero(self):
        # minimum_jerk(u) endpoints = 0/1; near-endpoint slope should be tiny
        # (5th-order polynomial, zero 1st+2nd derivative at 0 and 1).
        self.assertAlmostEqual(ft.minimum_jerk(0.0), 0.0)
        self.assertAlmostEqual(ft.minimum_jerk(1.0), 1.0)
        eps = 1e-4
        self.assertLess(ft.minimum_jerk(eps), eps ** 2)  # far below linear
        prev = -1.0
        for i in range(11):
            u = i / 10.0
            v = ft.minimum_jerk(u)
            self.assertGreaterEqual(v, prev)
            prev = v

    def test_clamped_outside_unit_interval(self):
        self.assertAlmostEqual(ft.minimum_jerk(-1.0), 0.0)
        self.assertAlmostEqual(ft.minimum_jerk(2.0), 1.0)


class AntiRollAndLateralTest(unittest.TestCase):
    def test_anti_roll_diag_scale_picks_neg_then_pos(self):
        # sr < 0.5 so there's a genuine "gap" phase between the two diagonal
        # stance windows where the scale falls back to 1.0 (symmetric).
        period, sr = 1.0, 0.4
        self.assertEqual(ft.anti_roll_diag_scale(0.1, period, sr, 1.3, 0.8), 1.3)
        self.assertEqual(ft.anti_roll_diag_scale(0.45, period, sr, 1.3, 0.8), 1.0)
        self.assertEqual(ft.anti_roll_diag_scale(0.6, period, sr, 1.3, 0.8), 0.8)
        self.assertEqual(ft.anti_roll_diag_scale(0.95, period, sr, 1.3, 0.8), 1.0)

    def test_lateral_offset_trot_zero_at_stance_boundaries(self):
        period, sr, sway = 1.0, 0.6, 0.008
        self.assertAlmostEqual(ft.lateral_offset_trot(0.0, period, sr, sway), 0.0)
        self.assertAlmostEqual(ft.lateral_offset_trot(sr, period, sr, sway), 0.0, places=6)

    def test_soft_trot_com_holds_amplitude_mid_diagonal(self):
        """Event-type CoM stays loaded mid-stance (unlike half-sine → 0 at TD)."""
        period, sway = 1.0, 0.012
        # Mid FL+RR half → full −Y (sim-validated anti-roll polarity)
        self.assertAlmostEqual(
            ft.lateral_offset_soft_trot_com(0.25, period, sway), -sway)
        # Mid FR+RL half → full +Y
        self.assertAlmostEqual(
            ft.lateral_offset_soft_trot_com(0.75, period, sway), sway)
        # Near diagonal switch: magnitude below full (blend zone)
        mid = ft.lateral_offset_soft_trot_com(0.50, period, sway, blend=0.12)
        self.assertAlmostEqual(mid, 0.0, places=5)

    def test_trot_weight_shift_sign_plateaus(self):
        self.assertEqual(ft.trot_weight_shift_sign(0.2, blend=0.10), -1.0)
        self.assertEqual(ft.trot_weight_shift_sign(0.7, blend=0.10), 1.0)

    def test_lateral_offset_pace_is_full_period_cosine(self):
        period, sr, sway = 1.0, 0.6, 0.01
        # phase == stance_ratio/2 -> cos(0) == 1 -> full amplitude.
        val = ft.lateral_offset_pace(sr / 2.0, period, sr, sway)
        self.assertAlmostEqual(val, sway)


class GateAndTurnTest(unittest.TestCase):
    def test_stance_weight_is_swing_level_during_swing(self):
        sr, taper = 0.6, 0.06
        self.assertEqual(ft.stance_weight(0.8, sr, 0.3, taper), 0.3)
        self.assertEqual(ft.stance_weight(sr, sr, 0.0, taper), 0.0)

    def test_stance_weight_peaks_at_one_mid_stance(self):
        sr, taper = 0.6, 0.06
        self.assertAlmostEqual(ft.stance_weight(sr / 2.0, sr, 0.0, taper), 1.0)

    def test_foot_track_gate_floor_and_stance(self):
        sr, taper, floor = 0.6, 0.06, 0.4
        self.assertEqual(ft.foot_track_gate(0.9, sr, floor, taper), floor)
        self.assertAlmostEqual(ft.foot_track_gate(sr / 2.0, sr, floor, taper), 1.0)

    def test_leg_y_turn_zero_when_turn_near_zero(self):
        self.assertEqual(ft.leg_y_turn('fl', 0.3, 0.0005, 0.6, 0.025, 1.0), 0.0)

    def test_leg_y_turn_front_and_rear_opposite_sign(self):
        front = ft.leg_y_turn('fl', 0.2, 0.5, 0.6, 0.025, 1.0)
        rear = ft.leg_y_turn('rl', 0.2, 0.5, 0.6, 0.025, 1.0)
        self.assertLess(front * rear, 0.0)


class LegXZShapeTest(unittest.TestCase):
    def test_stable_trot_x_swing_flag_matches_stance_ratio(self):
        sr = 0.6
        _, is_swing_before, _ = ft.stable_trot_x(sr - 0.01, 0.02, 0.1, sr, False)
        _, is_swing_after, _ = ft.stable_trot_x(sr + 0.01, 0.02, 0.1, sr, False)
        self.assertFalse(is_swing_before)
        self.assertTrue(is_swing_after)

    def test_stable_trot_x_smooth_and_cosine_agree_at_swing_start(self):
        # At swing_t=0 both Hermite(smooth) and cosine variants start at x=cx-amp.
        sr, amp, cx = 0.6, 0.02, 0.1
        x_smooth, _, _ = ft.stable_trot_x(sr + 1e-6, amp, cx, sr, True)
        x_cos, _, _ = ft.stable_trot_x(sr + 1e-6, amp, cx, sr, False)
        self.assertAlmostEqual(x_smooth, cx - amp, places=3)
        self.assertAlmostEqual(x_cos, cx - amp, places=3)

    def test_natural_trot_x_retract_pulls_inward_mid_swing(self):
        sr, amp, cx, retract = 0.6, 0.02, 0.1, 0.03
        x_no_retract, _, _ = ft.natural_trot_x(sr + (1 - sr) / 2.0, amp, cx, sr, 0.0)
        x_retracted, _, _ = ft.natural_trot_x(sr + (1 - sr) / 2.0, amp, cx, sr, retract)
        self.assertLess(x_retracted, x_no_retract)

    def test_natural_soft_trot_x_swing_flag(self):
        sr = 0.66
        _, is_swing, _ = ft.natural_soft_trot_x(sr + 0.01, 0.02, 0.1, sr, 0.03, 0.38)
        self.assertTrue(is_swing)


class FlourishAndTarsusTest(unittest.TestCase):
    def test_swing_flourish_hann_zero_in_stance(self):
        self.assertEqual(ft.swing_flourish_hann('fl', 0.3, 0.6, 5.0, 3.0), 0.0)

    def test_swing_flourish_hann_front_rear_opposite_sign(self):
        front = ft.swing_flourish_hann('fl', 0.8, 0.6, 5.0, 3.0)
        rear = ft.swing_flourish_hann('rl', 0.8, 0.6, 5.0, 3.0)
        self.assertGreater(front, 0.0)
        self.assertLess(rear, 0.0)

    def test_tarsus_swing_delta_hann_rear_leg_always_zero(self):
        self.assertEqual(ft.tarsus_swing_delta_hann('rl', 0.9, 0.6, 12.0), 0.0)
        self.assertEqual(ft.tarsus_swing_delta_hann('rr', 0.9, 0.6, 12.0), 0.0)

    def test_tarsus_swing_delta_mj_front_leg_nonzero_mid_swing(self):
        val = ft.tarsus_swing_delta_mj('fl', 0.8, 0.6, 12.0, peak=0.42)
        self.assertGreater(val, 0.0)


if __name__ == "__main__":
    unittest.main()
