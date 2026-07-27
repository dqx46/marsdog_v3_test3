#!/usr/bin/env python3
"""Compat launcher — real module: marsdog_control.apps.tools.legacy_apps.walk2"""
import os, sys, runpy
_SRC = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src")
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)
runpy.run_module("marsdog_control.apps.tools.legacy_apps.walk2", run_name="__main__")
