"""Compat alias — the real device-path config now lives in
``marsdog_control.config.bus_config``.

The device map cache (``usb_device_map.json``) stays here with the deployment;
the sunk module anchors its data dir back to this folder, so discovery behaves
identically. Bootstraps ``src`` onto ``sys.path``; single module identity.
"""

import os
import sys

_SRC = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src")
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from marsdog_control.config import bus_config as _real  # noqa: E402

sys.modules[__name__] = _real
