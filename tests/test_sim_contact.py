"""Sim contact: foot friction options apply to foot bodies."""

from __future__ import annotations

import os
import sys
import unittest

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (_ROOT, os.path.join(_ROOT, "src")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

try:
    import mujoco  # noqa: F401
except ImportError:
    mujoco = None


@unittest.skipIf(mujoco is None, "mujoco not installed")
class SimContactConfigTest(unittest.TestCase):
    def test_foot_friction_option_applied(self):
        import mujoco
        from marsdog_control.backends.sim import (
            SimPhysicsOptions,
            SimRobotBackend,
            _FOOT_BODIES,
        )
        from marsdog_control.motion.gait_controller import StandController

        ff = (2.5, 0.01, 0.001)
        backend = SimRobotBackend(
            stand_controller=StandController(0.25),
            physics_options=SimPhysicsOptions(foot_friction=ff),
        )
        m = backend.model
        gnd = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "ground")
        self.assertGreaterEqual(gnd, 0)

        n_foot = 0
        for fn in _FOOT_BODIES:
            bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, fn)
            self.assertGreaterEqual(bid, 0, msg=f"missing body {fn}")
            for gid in range(m.ngeom):
                if int(m.geom_bodyid[gid]) != bid:
                    continue
                n_foot += 1
                self.assertAlmostEqual(float(m.geom_friction[gid][0]), ff[0], places=3)
                self.assertAlmostEqual(float(m.geom_friction[gid][1]), ff[1], places=5)
                self.assertAlmostEqual(float(m.geom_friction[gid][2]), ff[2], places=5)
        self.assertGreaterEqual(n_foot, 4)


if __name__ == "__main__":
    unittest.main()
