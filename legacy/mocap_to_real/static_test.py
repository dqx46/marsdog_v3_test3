#!/usr/bin/env python3
"""Compat launcher — real static bus probe lives in
``marsdog_control.apps.tools.diagnostics.static_test``.

Kept so ``cd mocap_to_real && python3 static_test.py`` still works.
"""

import os
import sys

_SRC = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "src")
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from marsdog_control.apps.tools.diagnostics.static_test import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main() or 0)
