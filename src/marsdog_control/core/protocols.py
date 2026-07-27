"""Structural (``typing.Protocol``) contracts for seams that are intentionally
duck-typed — real driver *and* test fake both satisfy the interface without
either one inheriting from the other.

These are for the handful of components that legitimately have more than one
implementation at runtime (real hardware vs. parity-harness fakes, real
terminal vs. headless test double). They exist so dataclass fields like
``WalkLoopContext.clock`` stop being bare ``object`` — the type checker /
IDE can now tell you the exact method surface a replacement must satisfy,
instead of "whatever the convention says".

Components with exactly one real implementation (``RuntimeStateMachine``,
``SafetySupervisor``, ``CommandExecutor``, ...) should be typed with that
concrete class directly instead of a Protocol here — a Protocol only earns
its keep when structural substitution is the actual intent.
"""

from __future__ import annotations

from typing import Optional, Protocol


class ClockLike(Protocol):
    """Minimal time source the steady-state loop needs.

    Satisfied by the real ``time`` module *and* by
    ``tests/parity/loop_harness.py::_FakeClock`` without either declaring the
    other as a base class.
    """

    def time(self) -> float: ...
    def monotonic(self) -> float: ...
    def sleep(self, seconds: float) -> None: ...


class KeyReaderLike(Protocol):
    """Keyboard input seam. Satisfied by ``input.user_input.KeyReader`` and by
    ``tests/parity/fake_hardware.py::FakeKeyReader``.
    """

    def start(self) -> None: ...
    def stop(self) -> None: ...
    def flush(self) -> None: ...
    def get(self) -> Optional[str]: ...


__all__ = ["ClockLike", "KeyReaderLike"]
