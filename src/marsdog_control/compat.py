"""Compatibility helpers for the legacy flat ``mocap_to_real`` layout."""

from __future__ import annotations

import importlib
import sys
from pathlib import Path
from types import ModuleType


def _find_project_root() -> Path:
    for parent in Path(__file__).resolve().parents:
        if (parent / "mocap_to_real").is_dir():
            return parent
    return Path(__file__).resolve().parents[2]


_PROJECT_ROOT = _find_project_root()
_LEGACY_DIR = _PROJECT_ROOT / "mocap_to_real"
_SRC_DIR = _PROJECT_ROOT / "src"


def ensure_legacy_path() -> None:
    """Make both the ``src`` package and legacy flat modules importable.

    Sunk ``src`` modules keep their byte-identical flat imports (e.g.
    ``from joint_config import ...``), which resolve to the flat compat aliases;
    those aliases in turn import the real ``src`` modules. Putting ``src`` and
    ``mocap_to_real`` on the path here makes that resolution work no matter how
    a module is first imported. On the robot this only prepends existing dirs,
    so it never changes behavior.
    """
    for path in (str(_PROJECT_ROOT), str(_SRC_DIR), str(_LEGACY_DIR)):
        if path not in sys.path:
            sys.path.insert(0, path)


def _make_inert_stub(name: str) -> ModuleType:
    """A module whose every missing attribute resolves to a harmless placeholder.

    POSIX-tty/serial helpers reference constants like ``termios.B115200`` at
    import time; returning ``0`` for any unknown attribute lets those modules
    import off-target while keeping the objects unusable at runtime (which is
    fine because offline parity never touches real serial/tty I/O).
    """
    stub = ModuleType(name)
    stub.__dict__["__marsdog_offline_stub__"] = True

    def _missing(_attr: str):  # PEP 562 module __getattr__
        return 0

    stub.__dict__["__getattr__"] = _missing
    return stub


def install_offline_stubs() -> None:
    """Register dummy POSIX modules so legacy ``walk`` imports off-target.

    ``mocap_to_real/walk.py`` (and its serial/imu helpers) import
    ``termios``/``tty``/``fcntl`` at module top. Those modules only exist on
    POSIX, so importing the legacy loop for offline parity/tests fails on
    Windows dev machines.

    This helper is a no-op wherever the real modules are importable (e.g. the
    Linux robot), so it never changes on-target behavior. It only injects inert
    stand-ins for offline analysis where interactive keyboard/serial I/O is
    unused.
    """
    for name in ("termios", "tty", "fcntl"):
        if name in sys.modules:
            continue
        try:
            importlib.import_module(name)
        except Exception:  # noqa: BLE001 - only stub when genuinely absent
            sys.modules[name] = _make_inert_stub(name)


def import_legacy_module(name: str) -> ModuleType:
    """Import a module from the legacy flat layout."""
    ensure_legacy_path()
    return importlib.import_module(name)


def project_root() -> Path:
    return _PROJECT_ROOT


def legacy_dir() -> Path:
    return _LEGACY_DIR
