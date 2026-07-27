"""Runtime orchestration for Marsdog applications."""

from marsdog_control.runtime.app import RuntimeApp, RuntimePipeline, run_pipeline_loop
from marsdog_control.runtime.fsm import RuntimeStateMachine
from marsdog_control.runtime.lie_down_session import LieDownSession, LieDownSessionResult
from marsdog_control.runtime.shutdown import WalkShutdownContext, run_walk_shutdown
from marsdog_control.runtime.shadow import (
    ShadowDiff,
    compare_command_frames,
    compare_output_to_board,
)
from marsdog_control.runtime.status import RuntimeStatusDisplay
from marsdog_control.runtime.walk_loop import (
    WalkLoopContext,
    WalkLoopResult,
    run_steady_state_loop,
    tick_walk_loop,
)
from marsdog_control.runtime.walk_assembly import assemble_walk_loop_context
from marsdog_control.runtime.walk_state import WalkRuntimeState
from marsdog_control.runtime.walk_bringup import (
    HardwareSession,
    OperatorInputs,
    StandReadyResult,
    bringup_imu,
    bringup_motors_and_board,
    calibrate_imu_after_stand,
    fade_to_stand,
    open_operator_inputs,
)

__all__ = [
    "HardwareSession",
    "LieDownSession",
    "LieDownSessionResult",
    "OperatorInputs",
    "RuntimeApp",
    "RuntimePipeline",
    "RuntimeStateMachine",
    "RuntimeStatusDisplay",
    "ShadowDiff",
    "StandReadyResult",
    "WalkLoopContext",
    "WalkLoopResult",
    "WalkRuntimeState",
    "WalkShutdownContext",
    "assemble_walk_loop_context",
    "bringup_imu",
    "bringup_motors_and_board",
    "calibrate_imu_after_stand",
    "compare_command_frames",
    "compare_output_to_board",
    "fade_to_stand",
    "open_operator_inputs",
    "run_pipeline_loop",
    "run_steady_state_loop",
    "run_walk_shutdown",
    "tick_walk_loop",
]
