import contextlib
import io
import unittest

from marsdog_control.config.joints import JOINT_BY_ID
from marsdog_control.core.types import MotorFeedbackFrame, MotorSample
from marsdog_control.runtime.status import RuntimeStatusDisplay


class _Clock:
    def time(self):
        return 100.0

    def monotonic(self):
        return 100.0


class _Board:
    def get_feedback(self, ids):
        frame = MotorFeedbackFrame(t=100.0)
        for mid in ids:
            frame.samples[mid] = MotorSample(
                motor_id=mid,
                name=JOINT_BY_ID[mid].name,
                position=0.0,
                torque=0.0,
                enabled=(mid in {3, 7}),
                fault=0,
            )
        return frame


class RuntimeStatusDisplayTest(unittest.TestCase):
    def test_board_health_does_not_report_enabled_incos_as_evo_disabled(self):
        display = RuntimeStatusDisplay(
            [JOINT_BY_ID[3], JOINT_BY_ID[7], JOINT_BY_ID[15]],
            {},
            clock=_Clock(),
        )
        display.next_print = 999.0
        display.next_health = 0.0

        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            display.update(
                mode="stand",
                height=0.24,
                active_gait=None,
                cmd=None,
                imu=None,
                imu_dz=None,
                lie_down_hold=False,
                joint_direction_test=False,
                hip_abd_test=False,
                leg_pitch_test=False,
                direction_test_start=0.0,
                direction_test_duration_s=1.0,
                lz=None,
                evo=None,
                incos=None,
                board=_Board(),
            )

        text = out.getvalue()
        self.assertIn("M15(head_pitch)", text)
        self.assertNotIn("M3(fl_calf)", text)
        self.assertNotIn("M7(fr_calf)", text)


if __name__ == "__main__":
    unittest.main()
