"""SoftTrot shape keys stay aligned between preset and GaitCliDefaults."""

import os
import sys
import unittest

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (_ROOT, os.path.join(_ROOT, "src")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from marsdog_control.config.gait_tuning import GAIT, soft_trot_shape_keys  # noqa: E402
from marsdog_control.motion.gait_recipes import NATURAL_SOFT_TROT_REAL  # noqa: E402


class SoftTrotShapeSyncTest(unittest.TestCase):
    def test_gait_cli_defaults_match_soft_preset_for_shared_keys(self):
        keys = soft_trot_shape_keys()
        mismatches = []
        for key in sorted(keys):
            if key not in NATURAL_SOFT_TROT_REAL:
                mismatches.append(f"{key}: missing from NATURAL_SOFT_TROT_REAL")
                continue
            expected = NATURAL_SOFT_TROT_REAL[key]
            actual = getattr(GAIT, key)
            if actual != expected:
                mismatches.append(
                    f"{key}: GAIT={actual!r} vs NATURAL_SOFT_TROT_REAL={expected!r}")
        self.assertFalse(
            mismatches,
            msg="GaitCliDefaults 与 SoftTrot 预设漂移，请只改 NATURAL_SOFT_TROT_REAL "
                "后同步 GAIT 默认：\n" + "\n".join(mismatches))


if __name__ == "__main__":
    unittest.main()
