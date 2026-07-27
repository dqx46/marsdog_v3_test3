"""Input HAL — unified keyboard / gamepad (bluetooth reserved) adapter.

The steady-state loop takes a single ``WalkInputHAL`` component instead of bare
``poll_user_command`` / ``apply_dev_tuning`` callables + raw ``gamepad``/``keyboard``
handles. The app assembles the HAL once and the tick only sees ``poll(fsm)`` and
``apply_dev_tuning(...)`` — no knowledge of device details or dev-tuning wiring.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional

from marsdog_control.core.protocols import KeyReaderLike
from marsdog_control.input.user_input import (
    InputState,
    apply_dev_tuning as _apply_dev_tuning_impl,
    poll_user_command as _poll_user_command_impl,
)
from marsdog_control.runtime.walk_state import WalkRuntimeState


@dataclass
class WalkInputHAL:
    """Keyboard + gamepad boundary → (``UserCommand``, dev_key) + dev-tuning.

    Thresholds default to the canonical ``user_input`` constants; override only for
    tuning. ``check_motors`` is the diagnostics hook the ``p`` dev key prints with.

    ``gamepad`` stays ``object`` on purpose: it's a real HAL seam (device may be
    absent -> ``None``, or swapped for a fake in tests) with a wide attribute
    surface that isn't worth capturing in a Protocol here.
    """

    gamepad: object
    keyboard: Optional[KeyReaderLike]
    state: InputState
    runtime_state: WalkRuntimeState
    check_motors: Optional[Callable] = None

    def poll(self, fsm) -> tuple:
        """Return ``(UserCommand, dev_key)`` for one control cycle."""
        return _poll_user_command_impl(self.gamepad, self.keyboard, fsm, self.state)

    def apply_dev_tuning(self, dev_key, fsm, imu_ctrl, lz, evo, dm, incos) -> bool:
        """Apply a developer hot-key against the explicit runtime state."""
        rt = self.runtime_state.as_dev_tuning()

        def _check(lz_arg, evo_arg, dm_arg, label=""):
            if self.check_motors is None:
                return None
            return self.check_motors(lz_arg, evo_arg, dm_arg, incos, label=label)

        handled = _apply_dev_tuning_impl(
            dev_key, fsm, imu_ctrl, lz, evo, dm, rt, check_motors=_check)
        self.runtime_state.apply_dev_tuning_result(rt)
        return handled


__all__ = ["WalkInputHAL"]
