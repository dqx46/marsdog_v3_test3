"""Compat alias — the real Incos driver now lives in
``marsdog_control.hardware.motors.incos``.

Kept so legacy flat imports (``from motor_incos import MotorIncos``) resolve to
the exact same objects as the ``src`` package (single module identity).
Bootstraps ``src`` onto ``sys.path`` so direct launches work.
"""

import os
import sys

_SRC = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src")
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from marsdog_control.hardware.motors import incos as _real  # noqa: E402

sys.modules[__name__] = _real
