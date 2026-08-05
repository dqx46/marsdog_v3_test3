#!/usr/bin/env python3
"""Compat: provide ``serial.Serial`` when ``mocap_to_real`` is on ``sys.path``.

Prefer the real ``pyserial`` package from site-packages. If it is missing,
fall back to ``marsdog_control.apps.tools.misc.serial_fallback``.

IMPORTANT: this file must NOT be a runpy launcher. Naming it ``serial.py``
means ``import serial`` (used by IMU / tail / mouth) resolves here first —
a launcher would shadow pyserial and break hardware open with
``module 'serial' has no attribute 'Serial'``.
"""
from __future__ import annotations

import importlib
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.join(os.path.dirname(_HERE), "src")


def _load_serial_module():
    """Load real pyserial, or the project fallback, without recursing into this file."""
    saved_path = list(sys.path)
    # Hide this directory so importlib finds site-packages (or src fallback).
    sys.path = [
        p for p in saved_path
        if os.path.abspath(p or ".") != os.path.abspath(_HERE)
    ]
    # Drop any half-initialized self-reference.
    sys.modules.pop("serial", None)
    try:
        try:
            return importlib.import_module("serial")
        except ImportError:
            if _SRC not in sys.path:
                sys.path.insert(0, _SRC)
            return importlib.import_module(
                "marsdog_control.apps.tools.misc.serial_fallback")
    finally:
        sys.path[:] = saved_path


sys.modules[__name__] = _load_serial_module()
