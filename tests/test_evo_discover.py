"""MotorEvo sequential discover must not miss IDs the way batch probe did."""

from __future__ import annotations

import unittest

from marsdog_control.hardware.motors.evo import (
    CMD_REST_STATE,
    MEVO_KNOWN_IDS,
    MotorEvo,
    STATUS_PTM,
)


def _fake_fb(status: int = STATUS_PTM) -> bytes:
    return bytes([status, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 30])


class EvoDiscoverTest(unittest.TestCase):
    def test_probe_one_registers_target(self):
        evo = MotorEvo()
        evo._use_serial = True
        probe = bytes([0xFF] * 7 + [CMD_REST_STATE])
        sent = []
        replies = [(19, 8, _fake_fb())]

        evo._send_raw = lambda data, dlc, can_id: sent.append(can_id) or True
        evo._recv_raw = lambda: replies.pop(0) if replies else None
        evo._flush_raw = lambda: None

        ok = evo._probe_one(19, probe, timeout_s=0.02)
        self.assertTrue(ok)
        self.assertIn(19, evo._active_ids)
        self.assertTrue(evo.is_connected[18])
        self.assertEqual(sent, [19])

    def test_discover_all_retries_missed_id(self):
        """Simulates walk bug: first sweep misses 19, retry finds it."""
        evo = MotorEvo()
        evo._use_serial = True
        probe = bytes([0xFF] * 7 + [CMD_REST_STATE])
        fails_left = {19: 1}  # fail ID19 once

        def probe_one(mid, probe_bytes, *, timeout_s=0.050):
            if fails_left.get(mid, 0) > 0:
                fails_left[mid] -= 1
                return False
            evo._register_online(mid, _fake_fb(), tag="test")
            return True

        evo._probe_one = probe_one  # type: ignore[method-assign]
        ok = evo._discover_all(probe, tag="test", rounds=2, timeout_s=0.02)
        self.assertTrue(ok)
        self.assertEqual(sorted(evo._active_ids), sorted(MEVO_KNOWN_IDS))
        self.assertEqual(fails_left.get(19, 0), 0)


if __name__ == "__main__":
    unittest.main()
