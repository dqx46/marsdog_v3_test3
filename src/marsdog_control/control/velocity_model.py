"""Shared velocity model constants (estimator ↔ gait schedule ↔ WBC).

Dog-trot (2026-07): plant *undershoots* kinematic+scrub (truth≈0.12 vs
cmd≈0.20) while stance-foot LS *over-reads* (~0.25). The old soft-trot
``+0.055`` scrub prior then made WBC/MPC brake every step after the
first — robot only "walks out" for ~2s then crawls in place.

Scrub offset is therefore 0 until an adaptive residual prior exists.
"""

from __future__ import annotations

# Steady scrub along travel (m/s). 0 = none (see module doc).
VX_SCRUB_OFFSET_MPS: float = 0.0


__all__ = ["VX_SCRUB_OFFSET_MPS"]
