"""WBC reduced-model gravity vector smoke."""

from __future__ import annotations

import os
import sys
import unittest

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (_ROOT, os.path.join(_ROOT, "src")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

try:
    import pinocchio as pin  # noqa: F401
except ImportError:
    pin = None


@unittest.skipIf(pin is None, "pinocchio not installed")
class WbcHTest(unittest.TestCase):
    def test_nle_finite_at_neutral_pose(self):
        import numpy as np
        import pinocchio as pin
        from marsdog_control.control.nmpc_reduced_model import (
            QuadrupedReducedModel,
            default_urdf_path,
        )
        from marsdog_control.control.wbc import WholeBodyController, WbcConfig

        urdf = default_urdf_path()
        if not os.path.isfile(urdf):
            self.skipTest(f"URDF missing: {urdf}")
        cfg = WbcConfig(urdf_path=urdf)
        wbc = WholeBodyController(cfg, reduced=QuadrupedReducedModel(urdf))
        q = pin.neutral(wbc.model)
        q[2] = 0.25
        pin.computeAllTerms(wbc.model, wbc.data, q, np.zeros(wbc.nv))
        h = np.asarray(wbc.data.nle, dtype=float).reshape(-1)
        self.assertEqual(h.shape[0], wbc.nv)
        self.assertTrue(np.all(np.isfinite(h)))


if __name__ == "__main__":
    unittest.main()
