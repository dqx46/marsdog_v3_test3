"""Compatibility helpers for the legacy flat ``mocap_to_real`` layout."""

from __future__ import annotations

import importlib
import sys
from pathlib import Path
from types import ModuleType


def _find_project_root() -> Path:
    here = Path(__file__).resolve()
    for parent in here.parents:
        if (parent / "src" / "marsdog_control").is_dir() and (
            (parent / "mocap_to_real").is_dir()
            or (parent / "legacy" / "mocap_to_real").is_dir()
        ):
            return parent
    return here.parents[2]


_PROJECT_ROOT = _find_project_root()
_SRC_DIR = _PROJECT_ROOT / "src"


def _legacy_shim_dir() -> Path:
    """Python flat aliases: prefer archived tree, fall back to resources dir."""
    archived = _PROJECT_ROOT / "legacy" / "mocap_to_real"
    if archived.is_dir():
        return archived
    return _PROJECT_ROOT / "mocap_to_real"


_LEGACY_DIR = _legacy_shim_dir()
# Deploy resources (calib / sounds / poses) remain at repo ``mocap_to_real/``.
_RESOURCE_DIR = _PROJECT_ROOT / "mocap_to_real"


def ensure_legacy_path() -> None:
    """Make ``src`` and legacy flat shim modules importable.

    Sunk ``src`` modules keep flat imports (e.g. ``from joint_config import …``),
    which resolve to aliases under ``legacy/mocap_to_real/`` that re-export the
    real ``marsdog_control.*`` modules. Resource files stay in root
    ``mocap_to_real/`` and are also on the path for relative open() helpers.
    """
    for path in (
        str(_PROJECT_ROOT),
        str(_SRC_DIR),
        str(_LEGACY_DIR),
        str(_RESOURCE_DIR),
    ):
        if path and path not in sys.path:
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

    ``legacy/mocap_to_real/walk.py`` (and its serial/imu helpers) import
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


def legacy_resource_dir() -> Path:
    """Directory with motor_calib.json / sounds / pose JSON (not Python shims)."""
    return _RESOURCE_DIR


def legacy_dir() -> Path:
    """Deploy resource directory (``mocap_to_real/``).

    Historical name used by bus_config / audio / motor calib loaders. Python
    flat shims live under ``legacy/mocap_to_real/`` via ``ensure_legacy_path``.
    """
    return _RESOURCE_DIR


__all__ = [
    "ensure_legacy_path",
    "install_offline_stubs",
    "import_legacy_module",
    "legacy_dir",
    "legacy_resource_dir",
    "project_root",
]
