"""Compat alias — the real gravity feed-forward now lives in
``marsdog_control.control.gravity_comp``.

Bootstraps ``src`` onto ``sys.path`` and preserves single module identity.
"""

import os
import sys

_SRC = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "src")
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from marsdog_control.control import gravity_comp as _real  # noqa: E402

sys.modules[__name__] = _real
