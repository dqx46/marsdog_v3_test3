"""Compat alias — the real joint table now lives in
``marsdog_control.config.joints``.

Kept so legacy flat imports (``from joint_config import ...``) resolve to the
exact same objects as the ``src`` package (single module identity). Bootstraps
``src`` onto ``sys.path`` so direct ``python walk.py`` / tool launches work.
"""

import os
import sys

_SRC = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "src")
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from marsdog_control.config import joints as _real  # noqa: E402

sys.modules[__name__] = _real
