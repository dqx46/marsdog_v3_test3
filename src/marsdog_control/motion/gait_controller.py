"""Backward-compatible facade for gait controllers.

Prefer importing from the split modules (gait_base, stable_trot, natural_gait,
jump_gait); this module re-exports the historical public surface.
"""
from marsdog_control.motion.gait_base import (  # noqa: F401
    GaitController,
    StandController,
    _FRONT_HIP_OFFSET,
    _REAR_HIP_OFFSET,
    _FRONT_X0,
    _REAR_X0,
    _clamp,
    _cmd,
    kp_phase_scale,
    _smoothstep,
)
from marsdog_control.motion.stable_trot import (  # noqa: F401
    StableTrot,
    StablePace,
)
from marsdog_control.motion.natural_gait import (  # noqa: F401
    NaturalTrot,
    NaturalSoftTrot,
    NaturalWalk,
    WALK_PHASE_OFFSET,
)
from marsdog_control.motion.jump_gait import (  # noqa: F401
    JumpPhase,
    JumpController,
)

__all__ = [
    "GaitController",
    "StandController",
    "StableTrot",
    "StablePace",
    "NaturalTrot",
    "NaturalSoftTrot",
    "NaturalWalk",
    "WALK_PHASE_OFFSET",
    "JumpPhase",
    "JumpController",
    "kp_phase_scale",
    "_smoothstep",
    "_FRONT_HIP_OFFSET",
    "_REAR_HIP_OFFSET",
    "_FRONT_X0",
    "_REAR_X0",
    "_clamp",
    "_cmd",
]
