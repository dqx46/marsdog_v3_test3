import unittest

from marsdog_control.core.types import RobotMode
from marsdog_control.motion.mode_select import select_motion_target
from marsdog_control.motion.tarsus_bench import tarsus_bench_reference


class TarsusBenchReferenceTest(unittest.TestCase):
    def test_settle_then_sweep_then_done(self):
        delta, freq, done = tarsus_bench_reference(
            0.5, [1.0], 0.1, cycles=2.0, settle_s=1.0)
        self.assertEqual(delta, 0.0)
        self.assertEqual(freq, 1.0)
        self.assertFalse(done)

        delta, freq, done = tarsus_bench_reference(
            1.25, [1.0], 0.1, cycles=2.0, settle_s=1.0)
        self.assertAlmostEqual(abs(delta), 0.1, places=6)
        self.assertEqual(freq, 1.0)
        self.assertFalse(done)

        delta, freq, done = tarsus_bench_reference(
            4.0, [1.0], 0.1, cycles=2.0, settle_s=1.0)
        self.assertEqual(delta, 0.0)
        self.assertEqual(freq, 0.0)
        self.assertTrue(done)


class ModeSelectTest(unittest.TestCase):
    def test_lie_down_hold_wins(self):
        motion = select_motion_target(
            fsm=None, state=None, imu_dz=None, imu_state=None,
            online=set(), cur_pos={}, smooth_tgt={}, stand=None,
            lie_down_hold=True, lie_down_targets={3: 0.5},
        )
        self.assertEqual(motion.q[3], 0.5)
        self.assertEqual(motion.source_mode, RobotMode.STAND)


if __name__ == "__main__":
    unittest.main()
