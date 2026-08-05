"""Spot ramp / waist yaw and Natural spine helpers."""

from __future__ import annotations

import math

from marsdog_control.motion.spot_orchestration import (
    gait_entry_ramp,
    waist_yaw_turn_cmd,
)
from marsdog_control.motion.spine_overlay import (
    expected_spine_roll_deg,
    spine_yaw_roll_osc,
)


def test_spot_uses_short_blend_not_full_ramp():
    assert gait_entry_ramp(0.0, spot_turn_active=True, ramp_duration=1.0) == 0.0
    mid = gait_entry_ramp(0.125, spot_turn_active=True, ramp_duration=1.0)
    assert 0.4 < mid < 0.6
    assert gait_entry_ramp(0.3, spot_turn_active=True, ramp_duration=1.0) == 1.0
    # Cruise still uses long ramp_duration.
    assert gait_entry_ramp(0.3, spot_turn_active=False, ramp_duration=1.0) < 0.5


def test_spine_freezes_under_spot():
    yaw, roll = spine_yaw_roll_osc(
        t=0.25, period=1.0, ramp=1.0,
        spine_yaw_deg=3.0, spine_roll_deg=1.5,
        spine_phase_deg=0.0, spine_roll_phase_deg=0.0,
        spot_turn_active=True,
    )
    assert yaw == 0.0 and roll == 0.0
    yaw2, _ = spine_yaw_roll_osc(
        t=0.25, period=1.0, ramp=1.0,
        spine_yaw_deg=3.0, spine_roll_deg=1.5,
        spine_phase_deg=0.0, spine_roll_phase_deg=0.0,
        spot_turn_active=False,
    )
    assert abs(yaw2 - math.radians(3.0)) < 1e-9


def test_waist_yaw_spot_bias_and_cruise():
    spot = waist_yaw_turn_cmd(
        t=0.0, period=1.0, turn_filtered=1.0, ramp=1.0,
        spot_turn_active=True,
        waist_yaw_offset=0.0, waist_yaw_turn_sign=1.0,
        max_turn_waist_yaw=0.2,
        spot_waist_yaw_rad=0.1, spot_waist_yaw_pulse_rad=0.05,
    )
    assert abs(spot - 0.1) < 1e-9  # bias only at t=0 (pulse=0)
    cruise = waist_yaw_turn_cmd(
        t=0.0, period=1.0, turn_filtered=1.0, ramp=1.0,
        spot_turn_active=False,
        waist_yaw_offset=0.01, waist_yaw_turn_sign=1.0,
        max_turn_waist_yaw=0.2,
        spot_waist_yaw_rad=0.1, spot_waist_yaw_pulse_rad=0.05,
    )
    assert abs(cruise - 0.21) < 1e-9


def test_expected_spine_roll_contrib():
    c = expected_spine_roll_deg(
        t=0.25, period=1.0, ramp=1.0,
        spine_roll_deg=2.0, spine_roll_phase_deg=0.0,
    )
    assert abs(c - 2.0) < 1e-9
