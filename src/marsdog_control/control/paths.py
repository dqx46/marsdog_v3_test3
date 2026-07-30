"""Lightweight path helpers (no pinocchio / heavy deps)."""

from __future__ import annotations

import os


def default_urdf_path() -> str:
    """Repo-relative URDF: ``marsdog/urdf/marsdog.urdf``.

    ``paths.py`` lives in ``src/marsdog_control/control/`` → repo root is
    ``../../..`` (not ``../../../..``; that wrongly escapes into the parent
    of the project and breaks WBC unless ``--urdf-path`` is passed).
    """
    return os.path.abspath(
        os.path.join(
            os.path.dirname(__file__),
            "../../../marsdog/urdf/marsdog.urdf",
        )
    )
