"""Golden parity test for the decoupling refactor.

Runs the parity harness through the ``src`` control boundaries and asserts the
output matches a committed golden snapshot. Any behavior drift while relocating
algorithm code out of ``walk.py`` fails here on the dev machine, long before a
real-robot session.

Regenerate the golden intentionally (only when a change is *meant* to alter
output) with::

    MARSDOG_WRITE_GOLDEN=1 python -m unittest tests.parity.test_parity_baseline
"""

from __future__ import annotations

import json
import os
import sys
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from harness import run_scenarios  # noqa: E402

GOLDEN_PATH = os.path.join(_HERE, "golden_motion.json")


class ParityBaselineTest(unittest.TestCase):
    def test_matches_golden(self):
        # Round-trip through JSON so int motor-id keys normalize to strings,
        # matching what ``json.load`` yields for the committed golden file.
        current = json.loads(json.dumps(run_scenarios(), sort_keys=True))

        if os.environ.get("MARSDOG_WRITE_GOLDEN") == "1" or not os.path.exists(GOLDEN_PATH):
            with open(GOLDEN_PATH, "w", encoding="utf-8") as fh:
                json.dump(current, fh, indent=1, ensure_ascii=False, sort_keys=True)
                fh.write("\n")
            self.skipTest(f"golden written to {GOLDEN_PATH}")

        with open(GOLDEN_PATH, encoding="utf-8") as fh:
            golden = json.load(fh)

        # Compare scenario-by-scenario for a readable failure.
        self.assertEqual(set(golden), set(current),
                         "parity scenario set changed")
        for name in sorted(golden):
            self.assertEqual(golden[name], current[name],
                             f"parity drift in scenario '{name}'")


if __name__ == "__main__":
    unittest.main()
