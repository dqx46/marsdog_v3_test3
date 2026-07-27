"""Offline tests for CSV log setup."""

import csv
import json
import os
import sys
import tempfile
import unittest
from argparse import Namespace

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (_ROOT, os.path.join(_ROOT, "src")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from marsdog_control.io.logging import LOG_HEADER, LogRuntime, setup_log  # noqa: E402


class SetupLogTest(unittest.TestCase):
    def test_disabled_returns_empty_handles(self):
        self.assertEqual(setup_log(False, base_dir="."), (None, None, None))

    def test_writes_header_and_metadata(self):
        with tempfile.TemporaryDirectory() as tmp:
            args = Namespace(foo=1, bar={2, 1}, _explicit_cli={"foo"})
            runtime = LogRuntime(
                active_dm_kp_by_id={4: 60.0},
                active_dm_kd_by_id={4: 3.0},
                dm_reference_lead_s={4: 0.01},
                dm_reference_lead_max_rad=0.1,
                dm_dq_feedforward=True,
                dm_dq_max_rps=1.5,
                leg_kp_scale=0.8,
                var_impedance=True,
            )
            fh, writer, path = setup_log(True, args, base_dir=tmp, runtime=runtime)
            self.assertIsNotNone(writer)
            fh.close()

            with open(path, newline="") as csv_file:
                rows = list(csv.reader(csv_file))
            self.assertEqual(rows[0], LOG_HEADER)

            meta_path = path.replace(".csv", ".meta.json")
            with open(meta_path, encoding="utf-8") as meta_file:
                meta = json.load(meta_file)
            self.assertEqual(meta["explicit_cli"], ["foo"])
            self.assertEqual(meta["final_args"]["bar"], [1, 2])
            self.assertEqual(meta["dm"]["kp_by_id"], {"4": 60.0})
            self.assertEqual(meta["leg_kp_scale"], 0.8)
            self.assertTrue(meta["var_impedance"])


if __name__ == "__main__":
    unittest.main()
