"""Full-loop golden parity test (offline, no robot).

Boots the real ``walk.main`` control loop with fake drivers + a fake clock (see
``loop_harness``) and records the exact ``send_all`` command stream for a few
scenarios. That stream is locked against a committed golden. This is the
reference the refactored ``RuntimePipeline`` must reproduce byte-for-byte, and it
turns "does the whole loop still behave identically" into a deterministic
dev-machine check instead of a real-robot gamble.

Regenerate the golden intentionally (only when a change is *meant* to alter the
command stream) with::

    MARSDOG_WRITE_GOLDEN=1 python -m unittest tests.parity.test_loop_parity
"""

from __future__ import annotations

import json
import os
import sys
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from loop_harness import run_legacy_loop  # noqa: E402

GOLDEN_PATH = os.path.join(_HERE, "golden_loop.json")

_SCENARIOS = {
    "stand": {"extra_argv": [], "n_ticks": 6},
    "trot": {"extra_argv": ["--trot"], "n_ticks": 6},
    # DM-active front-tarsus path (id 4/8 driven by gait, not held fixed). Guards
    # the WalkRuntimeState.dm knobs after the Phase C module-global removal.
    "natural_soft_trot": {
        "extra_argv": ["--natural-soft-trot"],
        "n_ticks": 6,
    },
}


def _collect():
    return {name: run_legacy_loop(**kw) for name, kw in _SCENARIOS.items()}


class LoopParityTest(unittest.TestCase):
    def test_deterministic(self):
        first = json.dumps(_collect(), sort_keys=True)
        second = json.dumps(_collect(), sort_keys=True)
        self.assertEqual(first, second, "offline loop is not deterministic")

    def test_matches_golden(self):
        current = json.loads(json.dumps(_collect(), sort_keys=True))

        if os.environ.get("MARSDOG_WRITE_GOLDEN") == "1" or not os.path.exists(GOLDEN_PATH):
            with open(GOLDEN_PATH, "w", encoding="utf-8") as fh:
                json.dump(current, fh, indent=1, ensure_ascii=False, sort_keys=True)
                fh.write("\n")
            self.skipTest(f"golden written to {GOLDEN_PATH}")

        with open(GOLDEN_PATH, encoding="utf-8") as fh:
            golden = json.load(fh)

        self.assertEqual(set(golden), set(current), "loop scenario set changed")
        for name in sorted(golden):
            self.assertEqual(len(golden[name]), len(current[name]),
                             f"tick count drift in scenario '{name}'")
            for i, (g, c) in enumerate(zip(golden[name], current[name])):
                self.assertEqual(g, c, f"loop drift in '{name}' tick {i}")


if __name__ == "__main__":
    unittest.main()
