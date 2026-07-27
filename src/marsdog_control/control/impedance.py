"""Impedance and feed-forward control exports."""

from __future__ import annotations

from marsdog_control.motion.gait_controller import kp_phase_scale
from marsdog_control.control.gravity_comp import leg_gravity_ff

__all__ = ["kp_phase_scale", "leg_gravity_ff"]
