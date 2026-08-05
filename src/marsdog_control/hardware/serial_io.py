"""Serial port binding without legacy ``sys.path`` shims.

Prefer site-packages ``pyserial``; fall back to the in-tree POSIX wrapper so
simulation / offline imports do not require ``ensure_legacy_path``.
"""

from __future__ import annotations

try:
    import serial
except ImportError:  # pragma: no cover - depends on host packaging
    from marsdog_control.apps.tools.misc import serial_fallback as serial

__all__ = ["serial"]
