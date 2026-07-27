"""Input boundary for keyboard/gamepad commands."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from marsdog_control.config.schema import RuntimeConfig
from marsdog_control.core.types import UserCommand
from marsdog_control.input.user_input import poll_user_command


@dataclass
class InputManager:
    """Keyboard/gamepad boundary → ``UserCommand`` (no dependency on ``walk``)."""

    gamepad: object = None
    keyboard: object = None
    fsm: object = None
    state: object = None
    config: Optional[RuntimeConfig] = None

    def poll(self) -> UserCommand:
        if self.config is not None and not self.config.features.gamepad_enabled:
            return UserCommand()
        if self.fsm is None or self.state is None:
            return UserCommand()
        command, _dev_key = poll_user_command(
            self.gamepad, self.keyboard, self.fsm, self.state)
        return command


__all__ = ["InputManager"]
