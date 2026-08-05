"""Compose body-Y foot overlays (sway + reactive + turn) for abd conversion.

Keeps StableTrot.get_targets from inlining the same three-term sum twice.
"""

from __future__ import annotations

from typing import Callable


def compose_foot_body_y(
    *,
    lat_offset: float,
    reactive: float,
    body_height: float,
    reactive_active: bool,
    y_turn: float,
) -> float:
    """Body-frame foot Y (+Y = left).

    ``lat_offset`` > 0 (body left) → feet move right (−Y).
    ``reactive`` from roll PD; applied only when ``reactive_active``.
    """
    y_sway = -float(lat_offset)
    if reactive_active:
        y_reactive = -float(reactive) * float(body_height)
    else:
        y_reactive = 0.0
    return y_sway + y_reactive + float(y_turn)


def body_y_to_abd_delta(
    *,
    leg: str,
    y_total: float,
    y_to_abd: Callable[[str, float], float],
) -> float:
    """Convert body-Y overlay sum into abduction joint delta (URDF rad)."""
    return float(y_to_abd(leg, float(y_total)))


__all__ = ["body_y_to_abd_delta", "compose_foot_body_y"]
