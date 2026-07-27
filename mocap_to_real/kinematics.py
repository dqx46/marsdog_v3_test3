"""Compat alias — the real kinematics now live in
``marsdog_control.motion.kinematics``.

Single module identity so ``kinematics.ABD_LEGACY = ...`` set from the loop is
seen by the kinematics functions. Bootstraps ``src`` onto ``sys.path``.
"""

import os
import sys

_SRC = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src")
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from marsdog_control.motion import kinematics as _real  # noqa: E402

sys.modules[__name__] = _real
