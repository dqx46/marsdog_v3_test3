#!/usr/bin/env python3
"""Compat launcher — real file: manual_tests/legacy/test_rr_hip.py"""
import os, sys, runpy
_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_SRC = os.path.join(_ROOT, "src")
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)
runpy.run_path(os.path.join(_ROOT, "manual_tests", "legacy", "test_rr_hip.py"), run_name="__main__")
