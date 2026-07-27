"""Compat alias — the real gait presets now live in
``marsdog_control.motion.gait_recipes``.

Bootstraps ``src`` onto ``sys.path`` and preserves single module identity.
"""

import os
import sys

_SRC = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src")
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from marsdog_control.motion import gait_recipes as _real  # noqa: E402

sys.modules[__name__] = _real
