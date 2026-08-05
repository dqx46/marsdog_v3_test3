"""RuntimeStateMachine clock wiring smoke (typed build_runtime_fsm API)."""

from __future__ import annotations

import unittest

from marsdog_control.config.stack_build import FsmDriveConfig
from marsdog_control.core.types import RobotMode
from marsdog_control.runtime.walk_controllers import ForwardGaitAmps, build_runtime_fsm


class _DummyStand:
    def set_height(self, h: float) -> None:
        self.height = float(h)

    def get_targets(self, t: float = 0.0):
        return {}


class _DummyControllers:
    def __init__(self):
        self.stand = _DummyStand()
        self.trot_fwd = object()
        self.trot_bwd = object()
        self.pace_fwd = object()
        self.pace_bwd = object()
        self.nat_fwd = object()
        self.walk_fwd = object()
        self.jump_fwd = object()

    def as_tuple(self):
        return (
            self.stand, self.trot_fwd, self.trot_bwd,
            self.pace_fwd, self.pace_bwd, self.nat_fwd,
            self.walk_fwd, self.jump_fwd,
        )


class FsmClockTest(unittest.TestCase):
    def test_build_runtime_fsm_returns_machine(self):
        fsm = build_runtime_fsm(
            _DummyControllers(),
            FsmDriveConfig(),
            fwd=ForwardGaitAmps(
                amp_front=0.02, amp_rear=0.03, step_h=0.02,
                step_h_front=0.02, period=1.0,
            ),
            natural_configured=True,
            start_mode=RobotMode.STAND,
            height=0.25,
        )
        self.assertIsNotNone(fsm.clock)
        self.assertEqual(fsm.mode, RobotMode.STAND)
        self.assertAlmostEqual(fsm.stand.height, 0.25)


if __name__ == "__main__":
    unittest.main()
