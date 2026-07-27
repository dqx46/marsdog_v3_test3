"""Compat alias — the real layer contracts now live in
``marsdog_control.core.types``.

Kept so legacy flat imports (``from robot_types import ...``) resolve to the
exact same class objects as the ``src`` package (single module identity),
during and after the decoupling migration. Self-bootstraps ``src`` onto
``sys.path`` so direct ``python walk.py`` / tool-script launches keep working.
"""

import os
import sys

_SRC = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src")
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from marsdog_control.core import types as _real  # noqa: E402

sys.modules[__name__] = _real
