"""Compatibility shim for the src-layout package.

The implementation lives in ``src/marsdog_control``. This shim keeps imports
working when running directly from the repository root before ``pip install -e``.
"""

from __future__ import annotations

from pathlib import Path

_SRC_PACKAGE = Path(__file__).resolve().parents[1] / "src" / "marsdog_control"
if _SRC_PACKAGE.exists():
    __path__.append(str(_SRC_PACKAGE))

__all__ = ["__version__"]

__version__ = "0.1.0"
