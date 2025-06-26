"""Unit tests for sit/lie hold handoff helpers."""

from __future__ import annotations

import math
import unittest

from marsdog_control.backends.real import motor_pose_to_urdf, urdf_pose_to_motor
from marsdog_control.config.joints import JOINT_BY_ID
from marsdog_control.runtime.lie_down_session import LieDownSession


class LieDownHoldGravBlendTest(unittest.TestCase):
    def _session(self, **kwargs) -> LieDownSession:
        defaults = dict(
            build_target=lambda online: {},
            read_positions=lambda *a, **k: {},
            smooth_transition=lambda *a, **k: True,
            dm_fixed_targets={},
            grav_ramp_s=1.0,
            kp_ramp_s=1.0,
            kp_start=0.40,
        )
        defaults.update(kwargs)
        return LieDownSession(**defaults)

    def test_blend_ramps_after_enter_hold(self):
        s = self._session()
        self.assertEqual(s.hold_grav_blend(100.0), 1.0)  # not holding
        s._enter_hold("lie_down", {1: 0.1}, mono=10.0)
        self.assertAlmostEqual(s.hold_grav_blend(10.0), 0.0)
        mid = s.hold_grav_blend(10.5)
        self.assertTrue(0.0 < mid < 1.0)
        self.assertAlmostEqual(s.hold_grav_blend(11.0), 1.0)
        self.assertAlmostEqual(s.hold_grav_blend(12.0), 1.0)

    def test_kp_blend_starts_soft(self):
        s = self._session(kp_ramp_s=1.0, kp_start=0.40)
        self.assertEqual(s.hold_kp_blend(100.0), 1.0)
        s._enter_hold("sit", {1: 0.2}, mono=10.0)
        self.assertAlmostEqual(s.hold_kp_blend(10.0), 0.40)
        mid = s.hold_kp_blend(10.5)
        self.assertTrue(0.40 < mid < 1.0)
        self.assertAlmostEqual(s.hold_kp_blend(11.0), 1.0)

    def test_clear_hold_resets_blend(self):
        s = self._session()
        s._enter_hold("sit", {1: 0.2}, mono=5.0)
        s._clear_hold()
        self.assertFalse(s.hold)
        self.assertIsNone(s.hold_t0)
        self.assertEqual(s.hold_grav_blend(6.0), 1.0)
        self.assertEqual(s.hold_kp_blend(6.0), 1.0)

    def test_close_tracking_gap_skips_when_within_tol(self):
        calls = []

        def _read(*_a, **_k):
            return {1: 0.10, 9: 1.0}

        def _smooth(*args, **kwargs):
            calls.append((args, kwargs))
            return True

        s = self._session(
            read_positions=_read,
            smooth_transition=_smooth,
            settle_err_tol_deg=3.5,
        )
        ok = s._close_tracking_gap(
            lz=None, evo=None, dm=None, incos=None, board=None,
            dm_tarsus_active=False,
            target_motor={1: 0.10 + math.radians(2.0), 9: 1.0},
            label="sit",
        )
        self.assertTrue(ok)
        self.assertEqual(calls, [])

    def test_close_tracking_gap_runs_settle_when_lagging(self):
        calls = []

        def _read(*_a, **_k):
            return {1: 0.0, 9: math.radians(60.0)}

        def _smooth(lz, evo, dm, incos, from_pos, to_pos, duration, label="fade",
                    **kwargs):
            calls.append({
                "from": dict(from_pos),
                "to": dict(to_pos),
                "duration": float(duration),
                "label": label,
                "kp_end": kwargs.get("kp_end"),
            })
            return True

        s = self._session(
            read_positions=_read,
            smooth_transition=_smooth,
            settle_err_tol_deg=3.5,
            settle_s_min=0.7,
            settle_s_max=1.8,
            settle_s_per_rad=4.0,
            kp_start=0.40,
        )
        target = {1: 0.0, 9: math.radians(44.0)}
        ok = s._close_tracking_gap(
            lz=None, evo=None, dm=None, incos=None, board=None,
            dm_tarsus_active=False,
            target_motor=target,
            label="lie_down",
        )
        self.assertTrue(ok)
        self.assertEqual(len(calls), 1)
        self.assertEqual(calls[0]["label"], "lie_down-settle")
        self.assertAlmostEqual(calls[0]["kp_end"], 0.40)
        self.assertAlmostEqual(calls[0]["from"][9], math.radians(60.0), places=5)
        self.assertAlmostEqual(calls[0]["to"][9], math.radians(44.0), places=5)
        self.assertGreaterEqual(calls[0]["duration"], 0.7)
        self.assertLessEqual(calls[0]["duration"], 1.8)

    def test_canonicalize_preserves_sit_waist_pitch(self):
        """腰 pitch 捕获负角不得再被钳成 0（旧限位 [0,0.40] 的接缝根因）。"""
        waist = math.radians(-11.49)
        motor, urdf = LieDownSession._canonicalize_motor_pose(
            {20: waist}, label="sit")
        self.assertAlmostEqual(motor[20], waist, places=5)
        self.assertAlmostEqual(urdf[20], waist, places=5)
        # hold 下发路径 roundtrip 一致
        back = urdf_pose_to_motor(motor_pose_to_urdf({20: waist}))
        self.assertAlmostEqual(back[20], waist, places=5)

    def test_captured_lie_hips_roundtrip(self):
        pose = {
            9: math.radians(63.83),
            12: math.radians(-45.01),
            20: math.radians(-12.58),
        }
        motor, _urdf = LieDownSession._canonicalize_motor_pose(pose, label="lie")
        for mid, q in pose.items():
            self.assertAlmostEqual(
                motor[mid], q, places=4,
                msg=f"{JOINT_BY_ID[mid].name} clamped unexpectedly")


if __name__ == "__main__":
    unittest.main()
