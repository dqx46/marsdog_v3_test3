"""Jump/Spot transient impedance layered on session ImpedanceAssist."""

from __future__ import annotations

from types import SimpleNamespace

from marsdog_control.control.impedance_overlay import (
    apply_spot_abd_kp_boost,
    jump_phase_leg_kp,
    resolve_impedance_layers,
)


def test_jump_phases_layer_on_session_soft():
    session = 0.90
    crouch = SimpleNamespace(family="jump", phase=SimpleNamespace(value="crouch"))
    assert jump_phase_leg_kp(session, crouch, 0.0) == 1.25

    flight = SimpleNamespace(
        family="jump",
        phase=SimpleNamespace(value="flight"),
        _phase_u=lambda _t: 0.5,
    )
    assert 0.45 <= jump_phase_leg_kp(session, flight, 0.1) <= 0.65

    layers = resolve_impedance_layers(session, crouch, 0.0)
    assert layers.session_leg_kp == session
    assert layers.effective_leg_kp == 1.25
    assert not layers.spot_abd_boost_active


def test_spot_abd_boost_uses_session_not_jump_effective():
    class _J:
        def __init__(self, mid):
            self.motor_id = mid

    jbn = {
        "fl_thigh_roll": _J(1),
        "fr_thigh_roll": _J(2),
        "rl_hip": _J(3),
        "rr_hip": _J(4),
    }
    out = apply_spot_abd_kp_boost({}, session_leg_kp=0.90, joint_by_name=jbn)
    # 1.4 / 0.90 ≈ 1.555…
    assert abs(out[1] - 1.4 / 0.90) < 1e-9

    spot = SimpleNamespace(family="trot", spot_turn_active=True)
    layers = resolve_impedance_layers(0.90, spot, 0.0)
    assert layers.spot_abd_boost_active
    assert layers.effective_leg_kp == 0.90
