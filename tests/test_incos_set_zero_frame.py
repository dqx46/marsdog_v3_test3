"""Unit tests for Incos/ENCOS set-zero frame encoding (no hardware)."""

from __future__ import annotations

import os
import sys
import unittest

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (_ROOT, os.path.join(_ROOT, "src")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from marsdog_control.hardware.motors.incos import MotorIncos  # noqa: E402


class IncosSetZeroFrameTest(unittest.TestCase):
    def test_encode_matches_manual_example_id1(self):
        # Manual V1.19 §7.2 example: MotorID=1 → 00 01 00 03 on CAN 0x7FF
        self.assertEqual(
            MotorIncos.encode_set_zero_frame(1),
            bytes([0x00, 0x01, 0x00, 0x03]),
        )

    def test_encode_front_leg_ids(self):
        self.assertEqual(
            MotorIncos.encode_set_zero_frame(2),
            bytes([0x00, 0x02, 0x00, 0x03]),
        )
        self.assertEqual(
            MotorIncos.encode_set_zero_frame(7),
            bytes([0x00, 0x07, 0x00, 0x03]),
        )

    def test_encode_can_timeout_matches_manual(self):
        # Manual V1.19 §9.2.10 example: C00B01F4 → 500ms
        self.assertEqual(
            MotorIncos.encode_can_timeout_frame(500),
            bytes([0xC0, 0x0B, 0x01, 0xF4]),
        )
        # 0 = disable CAN timeout (keep last MIT after disconnect)
        self.assertEqual(
            MotorIncos.encode_can_timeout_frame(0),
            bytes([0xC0, 0x0B, 0x00, 0x00]),
        )


if __name__ == "__main__":
    unittest.main()
