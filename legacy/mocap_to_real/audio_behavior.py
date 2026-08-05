"""Compat alias — the real audio/mouth behavior now lives in
``marsdog_control.hardware.behavior.audio``.

Bootstraps ``src`` onto ``sys.path`` and preserves single module identity.
"""

import os
import sys

_SRC = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "src")
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from marsdog_control.hardware.behavior import audio as _real  # noqa: E402

sys.modules[__name__] = _real
