"""Compat alias — the real gait controllers now live in
``marsdog_control.motion.gait_controller``.

Single module identity so externally-set module globals
(``gait_controller.ABD_LEGACY`` / ``SWING_LEVEL`` / ``SMOOTH_GAIT``) reach the
gait math. Bootstraps ``src`` onto ``sys.path``.
"""

import os
import sys

_SRC = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src")
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from marsdog_control.motion import gait_controller as _real  # noqa: E402

sys.modules[__name__] = _real
