"""Compat alias — real gait controllers live in
``marsdog_control.motion.gait_controller``.

Bootstraps ``src`` onto ``sys.path``. Per-controller flags
(``swing_level`` / ``smooth_gait``) are instance attrs set via
``GaitStackConfig`` / ``GaitParams`` — not module globals.
"""

import os
import sys

_SRC = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "src")
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from marsdog_control.motion import gait_controller as _real  # noqa: E402

sys.modules[__name__] = _real
