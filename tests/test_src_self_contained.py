"""Boundary guard: ``src/marsdog_control`` must be import-self-contained.

Phase G of the refactor sank every real driver into ``src`` and repointed all
internal imports to canonical ``marsdog_control.*`` paths. From then on, the
``src`` package must NEVER reach back into the legacy flat namespace
(``from joint_config import ...``, ``import gait_controller``, ...). Those flat
names only exist as thin compat shims in ``mocap_to_real/`` for the legacy tool
ecosystem; if ``src`` imported them again it would silently re-couple the new
package to the old project root and require ``mocap_to_real`` on ``sys.path``.

This test fails loudly the moment such a regression is introduced. If you are
adding a genuinely new module, import it via its ``marsdog_control.*`` path.
"""

import os
import re
import unittest

_SRC = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "src", "marsdog_control")

# The legacy flat module names that used to live in mocap_to_real/ and now have
# real homes inside marsdog_control.*. `src` must import them by canonical path.
_FORBIDDEN = (
    "joint_config", "bus_config", "kinematics", "runtime_fsm",
    "safety_supervisor", "motor_lz_v2", "motor_evo", "motor_damiao",
    "motor_incos", "imu_wt901", "gamepad", "audio_behavior", "tail_behavior",
    "imu_controller", "robot_types", "gait_controller", "gait_recipes",
    "gravity_comp", "pose_contract", "can_serial", "can_bus",
)

# Matches a *bare* flat import only:
#   from <name> import ...
#   import <name>[ as x][, ...]
# but NOT `from marsdog_control.motion import kinematics` (dotted / package form).
_names = "|".join(re.escape(n) for n in _FORBIDDEN)
_PATTERN = re.compile(
    rf"^\s*(?:from\s+({_names})\s+import\b|import\s+({_names})(?:\s|,|$))")


class SrcSelfContainedTest(unittest.TestCase):
    def test_no_flat_legacy_imports_in_src(self):
        offenders = []
        for root, _dirs, files in os.walk(_SRC):
            if "__pycache__" in root:
                continue
            for fn in files:
                if not fn.endswith(".py"):
                    continue
                path = os.path.join(root, fn)
                with open(path, encoding="utf-8") as fh:
                    for lineno, line in enumerate(fh, 1):
                        if _PATTERN.match(line):
                            rel = os.path.relpath(path, _SRC)
                            offenders.append(f"{rel}:{lineno}: {line.strip()}")
        self.assertFalse(
            offenders,
            msg="src/marsdog_control 出现扁平 legacy import（应改为 "
                "marsdog_control.* 规范路径）：\n" + "\n".join(offenders))


if __name__ == "__main__":
    unittest.main()
