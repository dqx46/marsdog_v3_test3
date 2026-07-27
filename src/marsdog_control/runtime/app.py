"""Runtime application boundary."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Optional, Union

from marsdog_control.config.schema import RuntimeConfig
from marsdog_control.runtime.walk_loop import (
    WalkLoopContext,
    WalkLoopResult,
    tick_walk_loop,
)


@dataclass
class RuntimePipeline:
    """Live control pipeline. ``tick()`` is the sole steady-state control path.

    Production walk wires a ``WalkLoopContext``; dry assembly tests may leave
    ``walk_loop=None`` and use injected hardware/planner stubs instead.
    """

    hardware: object = None
    input_manager: object = None
    fsm: object = None
    balance_controller: object = None
    motion_planner: object = None
    safety: object = None
    executor: object = None
    logger: Optional[object] = None
    status: Optional[object] = None
    lie_down: Optional[object] = None
    bench: Optional[object] = None
    config: RuntimeConfig = field(default_factory=RuntimeConfig)
    walk_loop: Optional[WalkLoopContext] = None
    running: bool = True

    def tick(self) -> bool:
        if self.walk_loop is not None:
            self.walk_loop.running = self.running
            return tick_walk_loop(self.walk_loop)
        # Dry skeleton for unit assembly only.
        state = self.hardware.read_state()
        command = self.input_manager.poll()
        self.fsm.update(state, command, None)
        balance = self.balance_controller.update(state, self.fsm)
        motion = self.motion_planner.plan(
            self.fsm, state, balance.imu_dz, balance.imu_state,
            state.online, state.joint_pos)
        safe_motion, report = self.safety.filter(state, motion)
        if report.triggered_estop:
            return False
        output = self.executor.build(
            state, safe_motion, self.fsm,
            active_gait=getattr(self.fsm, "active_gait", None),
            t_rel=balance.t_rel,
        )
        self.hardware.send(output)
        if self.logger is not None:
            self.logger.write(state, command, output, report)
        return self.running


def run_pipeline_loop(pipeline: RuntimePipeline) -> Optional[WalkLoopResult]:
    """Drive ``pipeline.tick()`` until quit/estop/bench/safety/KeyboardInterrupt."""
    ctx = pipeline.walk_loop
    try:
        while pipeline.tick():
            pass
    except KeyboardInterrupt:
        pass
    if ctx is None:
        return None
    return WalkLoopResult(
        estopped_fall=ctx.estopped_fall,
        dm_tarsus_active=ctx.dm_tarsus_active,
        lie_down_hold=ctx.lie_down_session.hold,
    )


class RuntimeApp:
    """Owns the lifecycle of one Marsdog runtime application."""

    def __init__(self, entrypoint: Optional[Callable[[], None]] = None,
                 pipeline: Optional[RuntimePipeline] = None,
                 walk_loop: Optional[WalkLoopContext] = None,
                 config: Optional[RuntimeConfig] = None):
        self._entrypoint = entrypoint
        if pipeline is None and walk_loop is not None:
            pipeline = RuntimePipeline(walk_loop=walk_loop)
        self._pipeline = pipeline
        self.config = config or (
            pipeline.config if pipeline is not None else RuntimeConfig())

    def run(self) -> Optional[Union[WalkLoopResult, None]]:
        if self._pipeline is not None:
            return run_pipeline_loop(self._pipeline)
        if self._entrypoint is None:
            raise RuntimeError(
                "RuntimeApp requires a pipeline (preferred) or entrypoint")
        self._entrypoint()
        return None
