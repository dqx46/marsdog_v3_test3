"""DM front-tarsus reference-lead / dq-clamp behavior (current actuation path).

Ports the old ``mocap_to_real/test_imu_dm_pipeline.py::TestTarsusLead`` — which
had gone stale (referenced deleted ``walk.DM_*`` module globals and the pre-incos
``send_all`` signature) — onto the live ``hardware.actuation.send_all`` +
``WalkRuntimeState`` path so it actually runs offline and pins the DM lead math.
"""

import math
import unittest

from marsdog_control.hardware.actuation import send_all
from marsdog_control.runtime.walk_state import WalkRuntimeState


class _FakeDm:
    worker_running = True

    def set_commands(self, commands):
        self.commands = commands


def _rt():
    s = WalkRuntimeState()
    s.dm.active = True
    s.dm.reference_lead_s = {4: 0.04}
    s.dm.reference_lead_max_rad = math.radians(3.0)
    s.dm.dq_max_rps = 1.5
    s.dm.dq_feedforward = True
    return s.to_actuation_runtime()


class DmReferenceLeadTest(unittest.TestCase):
    def test_lead_disabled_when_flag_off(self):
        dm = _FakeDm()
        send_all(None, None, dm, None, {4: -0.5}, _rt(),
                 velocities={4: 0.5}, dm_reference_lead_active=False)
        self.assertAlmostEqual(dm.commands[4][2], -0.5)

    def test_lead_applied_within_limit(self):
        dm = _FakeDm()
        send_all(None, None, dm, None, {4: -0.5}, _rt(),
                 velocities={4: 0.5}, dm_reference_lead_active=True)
        self.assertAlmostEqual(dm.commands[4][2], -0.48)

    def test_lead_and_dq_are_clamped(self):
        dm = _FakeDm()
        send_all(None, None, dm, None, {4: -0.5}, _rt(),
                 velocities={4: 10.0}, dm_reference_lead_active=True)
        self.assertAlmostEqual(dm.commands[4][2], -0.5 + math.radians(3.0))
        self.assertAlmostEqual(dm.commands[4][3], 1.5)


if __name__ == "__main__":
    unittest.main()
