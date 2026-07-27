"""Offline parity test for the read-only state-estimation path.

`read_robot_state` / `read_robot_positions` were sunk out of
`mocap_to_real/walk.py` into `marsdog_control.hardware.robot_hw`. They only read
driver caches, so we can exercise them with fake motors/IMU on Windows/CI and
assert the produced ``RobotState`` matches the legacy math field-for-field.
"""

import os
import sys
import unittest

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (_ROOT, os.path.join(_ROOT, "src")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from marsdog_control.hardware.robot_hw import (  # noqa: E402
    REAL_JOINTS,
    read_robot_positions,
    read_robot_state,
)


class _FakeLz:
    def __init__(self, n=64):
        self.is_enabled = [True] * n

    def get_position(self, mid):
        return mid * 0.01


class _FakeEvo:
    def __init__(self, n=64):
        self.status = [0x02] * n

    def get_position(self, mid):
        return mid * 0.02


class _FakeDm:
    def get_position(self, mid):
        return mid * 0.03


class _FakeImu:
    connected = True
    roll = 0.1
    pitch = -0.2
    yaw = 0.3
    gyro_roll = 0.01
    gyro_pitch = -0.02
    gyro = (0.0, 0.0, 0.05)
    angle_timestamp = 0.0  # -> inf age; deterministic without a clock


class ReadStateParityTest(unittest.TestCase):
    def setUp(self):
        self.lz = _FakeLz()
        self.evo = _FakeEvo()
        self.dm = _FakeDm()
        self.imu = _FakeImu()
        self.online = {1, 2, 3}
        self.dm_fixed = {4: 0.5, 8: 0.5}

    def test_positions_skip_dm(self):
        pos = read_robot_positions(self.lz, self.evo)
        for j in REAL_JOINTS:
            if j.mtype == "dm":
                self.assertNotIn(j.motor_id, pos)
            elif j.mtype == "lz":
                self.assertAlmostEqual(pos[j.motor_id], j.motor_id * 0.01)
            else:
                self.assertAlmostEqual(pos[j.motor_id], j.motor_id * 0.02)

    def test_state_matches_legacy_math(self):
        st = read_robot_state(
            self.lz, self.evo, self.dm, self.imu, self.online, self.dm_fixed)
        self.assertEqual(st.online, self.online)
        for j in REAL_JOINTS:
            mid = j.motor_id
            if j.mtype == "lz":
                self.assertAlmostEqual(st.joint_pos[mid], mid * 0.01)
                self.assertTrue(st.joint_enabled[mid])
            elif j.mtype == "dm":
                self.assertAlmostEqual(st.joint_pos[mid], mid * 0.03)
                self.assertEqual(st.joint_enabled[mid], mid in self.dm_fixed)
            else:
                self.assertAlmostEqual(st.joint_pos[mid], mid * 0.02)
                self.assertTrue(st.joint_enabled[mid])
        self.assertTrue(st.imu_connected)
        self.assertAlmostEqual(st.roll, 0.1)
        self.assertAlmostEqual(st.pitch, -0.2)
        self.assertAlmostEqual(st.yaw, 0.3)
        self.assertAlmostEqual(st.gyro_yaw, 0.05)
        self.assertEqual(st.imu_age_s, float("inf"))

    def test_state_without_imu(self):
        st = read_robot_state(
            self.lz, self.evo, self.dm, None, self.online, self.dm_fixed)
        self.assertFalse(st.imu_connected)


if __name__ == "__main__":
    unittest.main()
