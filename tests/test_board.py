"""Offline tests for the RK Board abstraction."""

import os
import sys
import unittest

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (_ROOT, os.path.join(_ROOT, "src"), os.path.join(_ROOT, "tests", "parity")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from fake_hardware import FakeDm, FakeEvo, FakeIncos, FakeLz  # noqa: E402
from marsdog_control.hardware.actuation import ActuationRuntime  # noqa: E402
from marsdog_control.hardware.board import RkMotorBoard  # noqa: E402


def _rt():
    return ActuationRuntime(
        dm_tarsus_active=True,
        dm_fixed_targets={4: 0.0, 8: 0.0},
        dm_reference_lead_s={4: 0.0, 8: 0.0},
        dm_reference_lead_max_rad=0.0,
        active_dm_kp_by_id={},
        active_dm_kp=60.0,
        active_dm_kd_by_id={},
        active_dm_kd=3.0,
        default_dm_kp=30.0,
        default_dm_kd=0.5,
        dm_dq_max_rps=1.5,
        dm_dq_feedforward=True,
        leg_kp_scale=1.0,
        joint_gains={},
    )


class BoardTest(unittest.TestCase):
    def test_close_releases_incos_before_lz_shared_bus(self):
        order = []
        lz = FakeLz()
        evo = FakeEvo()
        dm = FakeDm()
        incos = FakeIncos()
        lz.end = lambda: order.append("lz")
        evo.end = lambda: order.append("evo")
        dm.end = lambda: order.append("dm")
        incos.end = lambda: order.append("incos")

        board = RkMotorBoard.from_existing(
            lz, evo, dm, incos, dm_fixed_targets={4: 0.0, 8: 0.0})
        board.close()

        self.assertEqual(order, ["incos", "lz", "evo", "dm"])

    def test_send_angles_updates_command_frame_and_feedback(self):
        board = RkMotorBoard.from_existing(
            FakeLz(), FakeEvo(), FakeDm(), FakeIncos(),
            dm_fixed_targets={4: 0.0, 8: 0.0})

        command = board.send_angles({1: 0.1, 3: -0.2, 4: 0.3, 9: 0.4}, _rt())
        feedback = board.get_feedback({1, 3, 4, 9})

        self.assertAlmostEqual(command.target_q[1], 0.1)
        self.assertAlmostEqual(command.target_q[3], -0.2)
        self.assertAlmostEqual(command.target_q[4], 0.3)
        self.assertAlmostEqual(command.target_q[9], 0.4)
        self.assertAlmostEqual(feedback.samples[1].position, 0.1)
        self.assertAlmostEqual(feedback.samples[3].position, -0.2)
        self.assertAlmostEqual(feedback.samples[4].position, 0.3)
        self.assertAlmostEqual(feedback.samples[9].position, 0.4)

    def test_soft_disable_finishes_with_zero_gains(self):
        board = RkMotorBoard.from_existing(
            FakeLz(), FakeEvo(), FakeDm(), FakeIncos(),
            dm_fixed_targets={4: 0.0, 8: 0.0})
        board.soft_disable({1: 0.1, 3: -0.2, 4: 0.3, 9: 0.4}, _rt(),
                           duration_s=0.01, control_hz=10.0)
        self.assertAlmostEqual(board.last_command.kp[1], 0.0)
        self.assertAlmostEqual(board.last_command.kd[1], 0.0)
        self.assertAlmostEqual(board.last_command.kp[3], 0.0)
        self.assertAlmostEqual(board.last_command.kd[3], 0.0)
        self.assertAlmostEqual(board.last_command.kp[4], 0.0)
        self.assertAlmostEqual(board.last_command.kd[4], 0.0)


if __name__ == "__main__":
    unittest.main()
