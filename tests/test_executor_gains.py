"""Offline unit tests for `resolve_gains` (sunk out of walk.py).

`resolve_gains` is pure math (no hardware), so we can pin its behavior exactly:
joint-gain lookup, leg kp scaling, phase scaling, torque override, and the
non-joint-gains lz/evo branch.
"""

import os
import sys
import unittest
from types import SimpleNamespace

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (_ROOT, os.path.join(_ROOT, "src")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from marsdog_control.control.executor import resolve_gains  # noqa: E402

_GAINS = {
    "fl_hip_pitch": {"kp": 40.0, "kd": 4.0, "trq_ff": 1.5},
    "waist_yaw": {"kp": 30.0, "kd": 3.0, "trq_ff": 0.0},
}


def _j(name, mtype="lz"):
    return SimpleNamespace(name=name, mtype=mtype)


class ResolveGainsTest(unittest.TestCase):
    def test_leg_joint_scaled_by_leg_and_phase(self):
        kp, kd, trq = resolve_gains(
            _j("fl_hip_pitch"), kp_scale=2.0, use_joint_gains=True,
            kp_lz=0, kd_lz=0, kp_evo=0, kd_evo=0,
            leg_kp_scale=0.5, joint_gains=_GAINS, phase_scale=0.4)
        self.assertAlmostEqual(kp, 40.0 * 2.0 * 0.5 * 0.4)
        self.assertAlmostEqual(kd, 4.0)
        self.assertAlmostEqual(trq, 1.5)

    def test_non_leg_joint_ignores_leg_scale(self):
        kp, kd, trq = resolve_gains(
            _j("waist_yaw"), kp_scale=1.0, use_joint_gains=True,
            kp_lz=0, kd_lz=0, kp_evo=0, kd_evo=0,
            leg_kp_scale=0.5, joint_gains=_GAINS)
        self.assertAlmostEqual(kp, 30.0)
        self.assertAlmostEqual(trq, 0.0)

    def test_trq_override_wins(self):
        _, _, trq = resolve_gains(
            _j("fl_hip_pitch"), kp_scale=1.0, use_joint_gains=True,
            kp_lz=0, kd_lz=0, kp_evo=0, kd_evo=0,
            leg_kp_scale=1.0, joint_gains=_GAINS, trq_override=-2.0)
        self.assertAlmostEqual(trq, -2.0)

    def test_unknown_joint_uses_fallback_gains(self):
        kp, kd, trq = resolve_gains(
            _j("mystery"), kp_scale=1.0, use_joint_gains=True,
            kp_lz=0, kd_lz=0, kp_evo=0, kd_evo=0,
            leg_kp_scale=1.0, joint_gains=_GAINS)
        self.assertAlmostEqual(kp, 30.0)
        self.assertAlmostEqual(kd, 4.0)
        self.assertAlmostEqual(trq, 0.0)

    def test_no_joint_gains_lz_vs_evo(self):
        kp_lz, _, _ = resolve_gains(
            _j("fl_calf", "lz"), kp_scale=1.0, use_joint_gains=False,
            kp_lz=12.0, kd_lz=1.0, kp_evo=99.0, kd_evo=9.0,
            leg_kp_scale=1.0, joint_gains=_GAINS)
        self.assertAlmostEqual(kp_lz, 12.0)
        kp_evo, kd_evo, _ = resolve_gains(
            _j("rr_thigh", "evo"), kp_scale=1.0, use_joint_gains=False,
            kp_lz=12.0, kd_lz=1.0, kp_evo=99.0, kd_evo=9.0,
            leg_kp_scale=1.0, joint_gains=_GAINS)
        self.assertAlmostEqual(kp_evo, 99.0)
        self.assertAlmostEqual(kd_evo, 9.0)


if __name__ == "__main__":
    unittest.main()
