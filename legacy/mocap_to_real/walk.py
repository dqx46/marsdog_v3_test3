#!/usr/bin/env python3
"""Compatibility launcher for the real Marsdog walk app.

The implementation lives in ``marsdog_control.apps.walk``. This file remains so
existing commands such as ``python3 mocap_to_real/walk.py`` and old flat imports
continue to work during the migration.
"""

from __future__ import annotations

import os
import sys

_SRC = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "src")
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from marsdog_control.apps import walk as _real  # noqa: E402

if __name__ == "__main__":
    _real.main()
else:
    sys.modules[__name__] = _real
