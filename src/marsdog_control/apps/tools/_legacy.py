"""Run legacy CLI scripts from organized tool wrappers."""

from __future__ import annotations

import runpy

from marsdog_control.compat import ensure_legacy_path, legacy_dir


def run_legacy_script(script_name: str) -> None:
    ensure_legacy_path()
    runpy.run_path(str(legacy_dir() / script_name), run_name="__main__")


__all__ = ["run_legacy_script"]
