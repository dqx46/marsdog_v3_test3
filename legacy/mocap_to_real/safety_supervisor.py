"""Compat alias — the real safety supervisor now lives in
``marsdog_control.safety.supervisor``.

Bootstraps ``src`` onto ``sys.path`` and preserves single module identity.
"""

import os
import sys

_SRC = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "src")
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from marsdog_control.safety import supervisor as _real  # noqa: E402

sys.modules[__name__] = _real
