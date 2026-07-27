"""Offline tests for the input parsing layer sunk out of walk.py."""

import os
import sys
import unittest
from types import SimpleNamespace

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (_ROOT, os.path.join(_ROOT, "src")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from marsdog_control.core.types import Direction, RobotMode  # noqa: E402
from marsdog_control.input.user_input import (  # noqa: E402
    DevTuningRuntime,
    InputState,
    apply_dev_tuning,
    poll_user_command,
)


class _FakeKb:
    def __init__(self, key=None):
        self.key = key

    def get(self):
        return self.key


class _FakeGp:
    connected = True

    def __init__(self, state):
        self._state = state

    def get_state(self):
        return self._state


class _FakeFsm:
    def __init__(self):
        self.drive = SimpleNamespace(turn_sign=1.0)
        self.trot_fwd = SimpleNamespace(period=1.0)
        self.height = 0.20
        self.periods = []
        self.heights = []
        self.fwd_amp_adjust = []

    def set_period(self, value):
        self.periods.append(value)
        self.trot_fwd.period = value

    def set_height(self, value):
        self.heights.append(value)
        self.height = value

    def adjust_fwd_amp(self, value):
        self.fwd_amp_adjust.append(value)


class _FakeImuCtrl:
    def __init__(self):
        self.p_boost = 1.0
        self.kp_roll = 0.02
        self.kp_pitch = 0.02
        self.roll_trim = 0.0


def _state(**kw):
    base = dict(
        select=False, b=False, start=False, lb=False, rb=False,
        lt=0.0, rt=0.0, ly=0.0, rx=0.0,
        dpad_up=False, dpad_down=False,
    )
    base.update(kw)
    return SimpleNamespace(**base)


class PollUserCommandTest(unittest.TestCase):
    def _poll(self, gp=None, kb=None, inp=None, fsm=None):
        return poll_user_command(
            gp, kb, fsm or _FakeFsm(), inp or InputState(),
            gp_period_step=0.05,
            gp_lie_down_lt_threshold=0.60,
            gp_bark_rt_threshold=0.60,
            gp_deadzone=0.12,
        )

    def test_keyboard_quit_and_toggle(self):
        cmd, dev = self._poll(kb=_FakeKb("q"))
        self.assertTrue(cmd.quit)
        self.assertIsNone(dev)

        cmd, dev = self._poll(kb=_FakeKb(" "))
        self.assertEqual(cmd.request_mode, RobotMode.TROT)
        self.assertIsNone(dev)

    def test_keyboard_dev_key_passthrough(self):
        cmd, dev = self._poll(kb=_FakeKb("+"))
        self.assertFalse(cmd.quit)
        self.assertEqual(dev, "+")

    def test_gamepad_start_edge_triggers_once(self):
        inp = InputState()
        gp = _FakeGp(_state(start=True))
        cmd, _ = self._poll(gp=gp, inp=inp)
        self.assertEqual(cmd.request_mode, RobotMode.TROT)

        cmd, _ = self._poll(gp=gp, inp=inp)
        self.assertIsNone(cmd.request_mode)

    def test_gamepad_sticks_and_dpad(self):
        cmd, _ = self._poll(gp=_FakeGp(_state(ly=-0.5, rx=0.3, dpad_down=True)))
        self.assertTrue(cmd.has_stick)
        self.assertAlmostEqual(cmd.vx, 0.5)
        self.assertAlmostEqual(cmd.turn, -0.3)
        self.assertTrue(cmd.pace)
        self.assertEqual(cmd.request_dir, Direction.BWD)


class ApplyDevTuningTest(unittest.TestCase):
    def test_runtime_hotkeys_update_explicit_runtime(self):
        fsm = _FakeFsm()
        imu = _FakeImuCtrl()
        rt = DevTuningRuntime(td_kp_scale=0.4, grav_scale=0.5)
        self.assertTrue(apply_dev_tuning(".", fsm, imu, None, None, None, rt,
                                         check_motors=lambda *a, **k: None))
        self.assertAlmostEqual(rt.td_kp_scale, 0.45)
        self.assertTrue(apply_dev_tuning("m", fsm, imu, None, None, None, rt,
                                         check_motors=lambda *a, **k: None))
        self.assertAlmostEqual(rt.grav_scale, 0.6)

    def test_controller_hotkeys_still_mutate_targets(self):
        fsm = _FakeFsm()
        imu = _FakeImuCtrl()
        rt = DevTuningRuntime(td_kp_scale=0.4, grav_scale=0.5)
        self.assertTrue(apply_dev_tuning("u", fsm, imu, None, None, None, rt,
                                         check_motors=lambda *a, **k: None))
        self.assertAlmostEqual(fsm.height, 0.21)
        self.assertTrue(apply_dev_tuning("]", fsm, imu, None, None, None, rt,
                                         check_motors=lambda *a, **k: None))
        self.assertAlmostEqual(imu.p_boost, 1.5)


if __name__ == "__main__":
    unittest.main()
