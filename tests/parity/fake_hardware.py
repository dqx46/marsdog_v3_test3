"""Deterministic fake drivers so `walk.main` can boot with no robot.

These stand in for MotorLz / MotorEvo / MotorDamiao / ImuWT901 / KeyReader /
TailController during the offline full-loop parity harness. They implement only
the surface `walk.main` actually touches, and every motor behaves as a *perfect
tracker*: whatever position we command, `get_position` reports back. That keeps
the control loop well-defined and fully deterministic (no hardware, no noise),
which is exactly what we need to capture a byte-stable golden command stream.
"""

from __future__ import annotations

import os
import sys

_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
for _p in (_ROOT, os.path.join(_ROOT, "src"), os.path.join(_ROOT, "mocap_to_real"),
           os.path.join(_ROOT, "legacy", "mocap_to_real")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

try:
    from marsdog_control.config.joints import JOINT_MAP  # noqa: E402
except ImportError:  # pragma: no cover - legacy shim path
    from joint_config import JOINT_MAP  # noqa: E402

_N = max(j.motor_id for j in JOINT_MAP) + 2  # +2 slack for 1-based indexing


class FakeLz:
    def __init__(self):
        self.is_connected = [True] * _N
        self.is_enabled = [False] * _N
        self.fault = [0] * _N
        self.torque = [0.0] * _N
        self.mode = [0] * _N
        self._pos = {}
        self._can1_serial = object()
        self._can1_lock = None

    def init_serial(self, dev, baud):
        pass

    def init_can1_serial(self, dev, baud):
        pass

    def add_can1_standard_handler(self, handler):
        pass

    def enable(self, mid):
        self.is_enabled[mid - 1] = True

    def re_enable(self, mid):
        self.is_enabled[mid - 1] = True
        self.fault[mid - 1] = 0

    def disable(self, mid):
        self.is_enabled[mid - 1] = False

    def get_position(self, mid):
        return self._pos.get(mid, 0.0)

    def _track(self, ids, pos):
        for mid, p in zip(ids, pos):
            self._pos[mid] = p

    def mit_controls_can1(self, ids, pos, vel, kp, kd, trq):
        self._track(ids, pos)

    def mit_controls_serial(self, ids, pos, vel, kp, kd, trq):
        self._track(ids, pos)

    def end(self):
        pass


class FakeEvo:
    def __init__(self):
        self.is_connected = [True] * _N
        self.status = [0x00] * _N
        self.fault = [0] * _N
        self._pos = {}

    def init_serial(self, dev, baud):
        pass

    def enter_motor_state(self, mid):
        self.status[mid - 1] = 0x02

    def exit_motor_state(self, mid):
        self.status[mid - 1] = 0x00

    def enter_rest_state(self, mid):
        self.status[mid - 1] = 0x00

    def disable(self, mid):
        self.status[mid - 1] = 0x00

    def get_position(self, mid):
        return self._pos.get(mid, 0.0)

    def ptm_controls(self, ids, pos, vel, kp, kd, trq):
        for mid, p in zip(ids, pos):
            self._pos[mid] = p

    def end(self):
        pass


class FakeDm:
    def __init__(self):
        self.worker_running = False
        self._pos = {}

    def begin(self, dev, baud):
        return True

    def add_motor(self, mid, master_id=None):
        pass

    def probe(self, mid):
        # (online, position, error, link_ok)
        return True, 0.0, 0, True

    def enable(self, mid):
        pass

    def start_worker(self):
        self.worker_running = True

    def stop_worker(self):
        self.worker_running = False

    def disable(self, mid):
        pass

    def get_position(self, mid):
        return self._pos.get(mid, 0.0)

    def get_error(self, mid):
        return 0

    def set_commands(self, commands):
        for mid, cmd in commands.items():
            self._pos[mid] = cmd[2]  # (kp, kd, q, dq, tau)

    def control_mit(self, mid, kp, kd, q, dq, tau):
        self._pos[mid] = q

    def end(self):
        pass


class FakeIncos:
    def __init__(self):
        self.position = [0.0] * _N
        self.velocity = [0.0] * _N
        self.torque = [0.0] * _N
        self.fault = [0] * _N
        self.is_connected = [True] * _N
        self.is_enabled = [False] * _N

    def begin(self, device, motor_ids=(3, 7), baud=921600):
        for mid in motor_ids:
            self.is_connected[mid - 1] = True
        self._running = True
        return True

    def begin_shared(self, serial_obj, lock, motor_ids=(3, 7), register_handler=None):
        return self.begin("shared", motor_ids)

    def start_keepalive(self):
        pass

    def stop_keepalive(self, timeout_s=1.0):
        pass

    def get_position(self, mid):
        return self.position[mid - 1]

    def get_torque(self, mid):
        return self.torque[mid - 1]

    def mit_control(self, mid, pos, vel=0.0, kp=0.0, kd=0.0, torque=0.0):
        self.position[mid - 1] = pos
        self.is_enabled[mid - 1] = kp > 0.0 or kd > 0.0
        return True

    def mit_controls(self, ids, pos, vel, kp, kd, trq):
        for mid, q, kpi, kdi in zip(ids, pos, kp, kd):
            self.position[mid - 1] = q
            self.is_enabled[mid - 1] = kpi > 0.0 or kdi > 0.0
        return True

    def disable(self, mid):
        self.is_enabled[mid - 1] = False
        return True

    def end(self):
        pass


class FakeImu:
    """Level, connected IMU with zero rates and fresh frames (deterministic)."""

    def __init__(self, *args, **kwargs):
        self.connected = True
        self.update_count = 100
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.gyro_roll = 0.0
        self.gyro_pitch = 0.0
        self.gyro = (0.0, 0.0, 0.0)
        self.angle_timestamp = 0.0

    def begin(self):
        return True

    def calibrate(self, seconds=0.0):
        pass

    def frame_ages(self, now):
        return {"angle": 0.0, "gyro": 0.0}

    def close(self):
        pass


class FakeKeyReader:
    def start(self):
        pass

    def stop(self):
        pass

    def flush(self):
        pass

    def get(self):
        return None


class FakeTail:
    """begin() returns False so walk drops the tail channel entirely."""

    def begin(self):
        return False

    def close(self):
        pass
