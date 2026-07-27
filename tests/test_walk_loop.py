import unittest
from types import SimpleNamespace

from marsdog_control.control.executor import CommandExecutor, ExecutorConfig
from marsdog_control.core.types import MotionTarget, RobotMode, UserCommand
from marsdog_control.runtime.walk_loop import (
    LoopHardware,
    WalkLoopContext,
    tick_walk_loop,
)
from marsdog_control.runtime.walk_state import WalkRuntimeState


class _Clock:
    def __init__(self):
        self.t = 10.0

    def time(self):
        return self.t

    def monotonic(self):
        return self.t

    def sleep(self, dt):
        self.t += max(0.0, dt)


class _FakeFsm:
    mode = SimpleNamespace(value="stand")
    active_gait = None
    t_gait = 0.0
    height = 0.24
    throttle = 0.0
    direction = None

    def update(self, state, cmd, targets):
        return None

    def consume_just_switched(self):
        return False

    def dm_active(self):
        return False

    def request_transition(self, *args, **kwargs):
        return None


class _FakeSafety:
    def filter(self, state, motion):
        report = SimpleNamespace(triggered_estop=False, reason="")
        return motion, report


class _FakeBalance:
    def update(self, **kwargs):
        return SimpleNamespace(imu_dz=None, imu_state=None, t_rel=0.0)


class _FakeLieDown:
    hold = False
    targets = {}


class _FakeRecorder:
    def maybe_record(self, **kwargs):
        return None


class _FakeStatus:
    def update(self, **kwargs):
        return None


class WalkLoopTickTest(unittest.TestCase):
    def test_quit_stops_loop(self):
        sends = []
        clock = _Clock()

        class _FakeInputHAL:
            def poll(self, fsm):
                return UserCommand(quit=True), None

            def apply_dev_tuning(self, *a, **k):
                return None

        def read_state(*args):
            return SimpleNamespace(
                joint_pos={}, roll=0.0, pitch=0.0, online=set())

        def send_all(*args, **kwargs):
            sends.append(kwargs)
            return None

        def select_motion(**kwargs):
            return MotionTarget(q={1: 0.0}, source_mode=RobotMode.STAND)

        ctx = WalkLoopContext(
            hw=LoopHardware(online=set()),
            runtime_state=WalkRuntimeState(),
            fsm=_FakeFsm(), input_hal=_FakeInputHAL(),
            bark_with_mouth=lambda: None,
            stand=None, safety=_FakeSafety(), imu_ctrl=None,
            balance_runtime=_FakeBalance(),
            executor=CommandExecutor(config=ExecutorConfig()),
            lie_down_session=_FakeLieDown(),
            recorder=_FakeRecorder(),
            status_display=_FakeStatus(),
            backend=None,
            targets={1: 0.0},
            cur_pos={1: 0.0},
            smooth_tgt={},
            dm_tarsus_active=False,
            joint_direction_test=False,
            hip_abd_test=False,
            leg_pitch_test=False,
            direction_test_start=0.0,
            direction_test_duration_s=1.0,
            control_hz=200.0,
            clock=clock,
        )
        self.assertFalse(tick_walk_loop(ctx))
        self.assertEqual(sends, [])


if __name__ == "__main__":
    unittest.main()
