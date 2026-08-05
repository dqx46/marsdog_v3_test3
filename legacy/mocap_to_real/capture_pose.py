#!/usr/bin/env python3
"""Compatibility launcher for interactive pose capture.

Usage:
  python3 mocap_to_real/capture_pose.py
"""

from __future__ import annotations

import os
import sys

_SRC = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "src")
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from marsdog_control.apps.tools.calibration.capture_pose import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main())
