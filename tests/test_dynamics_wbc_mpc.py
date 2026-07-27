"""Unit tests for reduced-model WBC/MPC dynamics stack."""

from __future__ import annotations

import numpy as np
import pytest

pinocchio = pytest.importorskip("pinocchio")
pytest.importorskip("osqp")
pytest.importorskip("qpsolvers")


def test_reduced_model_mimic_and_mass():
    from marsdog_control.control.nmpc_reduced_model import (
        LEG_ACTUATED_JOINT_NAMES,
        QuadrupedReducedModel,
        default_urdf_path,
    )

    rm = QuadrupedReducedModel(default_urdf_path())
    assert rm.total_mass > 8.0
    assert rm.nv == 22  # 6 fb + 16 leg joints (incl. rear tarsus)
    assert rm.model.existJointName("rl_tarsus_joint")
    assert rm.model.existJointName("fl_tarsus_joint")
    for jn in LEG_ACTUATED_JOINT_NAMES:
        assert rm.model.existJointName(jn), jn
    # rear tarsus must NOT be in S list
    assert "rl_tarsus_joint" not in LEG_ACTUATED_JOINT_NAMES

    q = np.zeros(rm.nq)
    v = np.zeros(rm.nv)
    # set calf then mimic
    iq_c = rm._joint_idx_q["rl_calf_joint"]
    iq_t = rm._joint_idx_q["rl_tarsus_joint"]
    q[iq_c] = 0.3
    rm.apply_rear_tarsus_mimic(q, v)
    assert abs(q[iq_t] - (-0.3)) < 1e-9


def test_mpc_osqp_updates_with_yaw_change():
    from marsdog_control.control.srb_mpc import SrbMpc, SrbMpcConfig

    cfg = SrbMpcConfig(f_max=80.0, mu=0.6, mass=10.47)
    I = np.diag([0.05, 0.1, 0.1])
    mpc = SrbMpc(cfg, I)

    x0 = np.zeros(13)
    x0[12] = -9.81
    x0[5] = 0.24
    H = cfg.horizon
    x_ref = np.zeros(13 * H)
    for i in range(H):
        x_ref[i * 13 + 5] = 0.24
        x_ref[i * 13 + 12] = -9.81
    r_feet = np.array(
        [
            [0.2, 0.2, -0.2, -0.2],
            [0.1, -0.1, 0.1, -0.1],
            [-0.24, -0.24, -0.24, -0.24],
        ]
    )
    contact = np.ones(4 * H)

    f1 = mpc.solve(x0, x_ref, r_feet, contact)
    assert mpc.last_solve_ok
    assert f1[2] > 10.0  # gravity share ~25N
    assert f1[2] < cfg.f_max + 1e-6

    # Change yaw / foot placement — must still solve (P rebuilt)
    x0[2] = 0.4
    r_feet[0, 0] += 0.05
    f2 = mpc.solve(x0, x_ref, r_feet, contact)
    assert mpc.last_solve_ok
    assert np.linalg.norm(f2) > 1.0


def test_wbc_fallback_keeps_last_tau():
    from marsdog_control.control.nmpc_reduced_model import (
        QuadrupedReducedModel,
        default_urdf_path,
    )
    from marsdog_control.control.wbc import WholeBodyController, WbcConfig

    rm = QuadrupedReducedModel(default_urdf_path())
    wbc = WholeBodyController(WbcConfig(urdf_path=default_urdf_path()), reduced=rm)
    wbc._last_tau = np.ones(wbc.n_actuated) * 1.23

    # Force failure path by monkeypatching solve
    import marsdog_control.control.wbc as wbc_mod

    orig = wbc_mod.qpsolvers.solve_qp

    def boom(*args, **kwargs):
        raise RuntimeError("forced")

    wbc_mod.qpsolvers.solve_qp = boom
    try:
        q = np.zeros(wbc.nq)
        q[2] = 0.24
        q[6] = 1.0  # qw
        v = np.zeros(wbc.nv)
        tau = wbc.compute_tau(
            q,
            v,
            np.zeros(6),
            {"fl": True, "fr": True, "rl": True, "rr": True},
        )
        assert np.allclose(tau, 1.23)
        assert not wbc.last_solve_ok
        assert wbc.fail_count >= 1
    finally:
        wbc_mod.qpsolvers.solve_qp = orig


def test_wbc_stance_solve_smoke():
    from marsdog_control.control.nmpc_reduced_model import (
        QuadrupedReducedModel,
        default_urdf_path,
    )
    from marsdog_control.control.wbc import WholeBodyController, WbcConfig

    rm = QuadrupedReducedModel(default_urdf_path())
    wbc = WholeBodyController(WbcConfig(f_max=80.0, mu=0.6), reduced=rm)
    q = np.zeros(wbc.nq)
    q[2] = 0.24
    q[6] = 1.0
    rm.apply_rear_tarsus_mimic(q, np.zeros(wbc.nv))
    v = np.zeros(wbc.nv)
    base_acc = np.zeros(6)
    base_acc[2] = 1.0
    f_des = np.zeros(12)
    for i in range(4):
        f_des[i * 3 + 2] = rm.total_mass * 9.81 / 4
    tau = wbc.compute_tau(
        q,
        v,
        base_acc,
        {"fl": True, "fr": True, "rl": True, "rr": True},
        f_c_des=f_des,
    )
    assert wbc.last_solve_ok
    assert tau.shape == (wbc.n_actuated,)
    assert np.all(np.abs(tau) <= WbcConfig().tau_limit_nm + 1e-6)


def test_contact_schedule_phase_and_measure():
    from marsdog_control.control.contact_schedule import ContactSchedule, ContactConfig

    class _Gait:
        period = 1.0
        stance_ratio = 0.7
        _PHASE_OFFSET = {"fl": 0.0, "fr": 0.5, "rl": 0.5, "rr": 0.0}

    cs = ContactSchedule(
        ContactConfig(
            hold_steps=1,
            td_height_m=0.02,
            lo_height_m=0.03,
            use_relative_z=False,
        )
    )
    # t=0.35: FL phase=0.35 stance; FR phase=0.85 swing
    snap = cs.update(
        t_rel=0.35,
        gait=_Gait(),
        foot_z_world={"fl": 0.005, "fr": 0.05, "rl": 0.05, "rr": 0.005},
        foot_vz_world={"fl": 0.0, "fr": 0.1, "rl": 0.1, "rr": 0.0},
    )
    assert snap.scheduled["fl"] is True
    assert snap.scheduled["fr"] is False
    assert snap.force_scale["fl"] >= 0.0

    H = cs.horizon(
        t_rel=0.35,
        gait=_Gait(),
        horizon=5,
        dt=0.03,
        measured={"fl": False, "fr": False, "rl": False, "rr": False},
    )
    assert H.shape == (20,)
    # Horizon is phase-only continuous weights: mid-stance ≈1, mid-swing ≈0
    assert H[0] >= 0.99  # FL scheduled stance at k=0
    assert H[1] <= 0.01  # FR scheduled swing
    # Measured hard-cuts must NOT clear scheduled stance mid-support
    assert H[0] > 0.5


def test_contact_relative_z_releases_low_world_z():
    """Rear feet with negative world-z baseline must still LO via relative height."""
    from marsdog_control.control.contact_schedule import ContactSchedule, ContactConfig

    class _Gait:
        period = 1.0
        stance_ratio = 0.7
        _PHASE_OFFSET = {"fl": 0.0, "fr": 0.5, "rl": 0.5, "rr": 0.0}

    cs = ContactSchedule(
        ContactConfig(
            hold_steps=1,
            use_relative_z=True,
            rel_td_m=0.006,
            rel_lo_m=0.012,
            z_ref_ema=1.0,
            edge_blend=0.12,
        )
    )
    # Seed mid-stance baseline for RR at z=-0.008 (world)
    for _ in range(3):
        cs.update(
            t_rel=0.35,  # RR phase=0.35 mid-stance
            gait=_Gait(),
            foot_z_world={"fl": 0.01, "fr": 0.05, "rl": 0.05, "rr": -0.008},
            foot_vz_world={"fl": 0.0, "fr": 0.1, "rl": 0.1, "rr": 0.0},
        )
    assert cs._z_ref_ready["rr"]
    # Swing: RR phase at t=0.85 → 0.85, z lifts +18 mm relative → LO
    snap = cs.update(
        t_rel=0.85,
        gait=_Gait(),
        foot_z_world={"fl": 0.05, "fr": 0.005, "rl": 0.005, "rr": 0.012},
        foot_vz_world={"fl": 0.1, "fr": 0.0, "rl": 0.0, "rr": 0.15},
    )
    assert snap.scheduled["rr"] is False
    assert snap.measured["rr"] is False


def test_estimator_mid_stance_gate_prefers_mid_feet():
    from marsdog_control.control.base_estimator import BaseStateEstimator
    from marsdog_control.control.nmpc_reduced_model import (
        QuadrupedReducedModel,
        default_urdf_path,
    )

    rm = QuadrupedReducedModel(default_urdf_path())
    est = BaseStateEstimator(ema=1.0, force_scale_min=0.8, slip_thresh=0.08)
    q = np.zeros(rm.nq)
    q[2] = 0.24
    q[6] = 1.0
    v = np.zeros(rm.nv)
    # All stance scheduled; only mid-phase + high force_scale used preferentially
    out = est.update(
        reduced=rm,
        q_pin=q,
        v_pin=v,
        roll=0.0,
        pitch=0.0,
        yaw=0.0,
        gyro=(0.0, 0.0, 0.0),
        leg_is_stance={"fl": True, "fr": True, "rl": True, "rr": True},
        leg_phase={"fl": 0.35, "fr": 0.05, "rl": 0.05, "rr": 0.35},
        stance_ratio=0.79,
        edge_blend=0.12,
        force_scale={"fl": 1.0, "fr": 0.3, "rl": 0.3, "rr": 1.0},
        dt=0.005,
    )
    assert np.isfinite(out.vx)


def test_wbc_soft_force_scale_continuity():
    """Binary stance flip must not produce huge tau jumps when force_scale is continuous."""
    from marsdog_control.control.nmpc_reduced_model import (
        QuadrupedReducedModel,
        default_urdf_path,
    )
    from marsdog_control.control.wbc import WholeBodyController, WbcConfig

    rm = QuadrupedReducedModel(default_urdf_path())
    wbc = WholeBodyController(WbcConfig(f_max=80.0, mu=0.6), reduced=rm)
    q = np.zeros(wbc.nq)
    q[2] = 0.24
    q[6] = 1.0
    rm.apply_rear_tarsus_mimic(q, np.zeros(wbc.nv))
    v = np.zeros(wbc.nv)
    base_acc = np.zeros(6)
    base_acc[2] = 1.0
    f_des = np.zeros(12)
    for i in range(4):
        f_des[i * 3 + 2] = rm.total_mass * 9.81 / 4

    # Soft path: force_scale ramps across LO while boolean stance flips.
    tau_s = []
    for s, stance_fl in [(0.55, True), (0.45, False)]:
        tau_s.append(
            wbc.compute_tau(
                q,
                v,
                base_acc,
                {"fl": stance_fl, "fr": True, "rl": True, "rr": True},
                f_c_des=f_des * np.array([s, s, s, 1, 1, 1, 1, 1, 1, 1, 1, 1]),
                force_scale={"fl": s, "fr": 1.0, "rl": 1.0, "rr": 1.0},
            )
        )
        assert wbc.last_solve_ok
    soft_jump = float(np.max(np.abs(tau_s[1] - tau_s[0])))

    # Hard path: same states but no force_scale → binary QP structure flip.
    tau_h = []
    for stance_fl in (True, False):
        tau_h.append(
            wbc.compute_tau(
                q,
                v,
                base_acc,
                {"fl": stance_fl, "fr": True, "rl": True, "rr": True},
                f_c_des=f_des,
            )
        )
        assert wbc.last_solve_ok
    hard_jump = float(np.max(np.abs(tau_h[1] - tau_h[0])))

    assert soft_jump < 2.0, soft_jump
    assert soft_jump < 0.5 * hard_jump + 0.5, (soft_jump, hard_jump)


def test_estimator_accepts_v_pin():
    from marsdog_control.control.base_estimator import BaseStateEstimator
    from marsdog_control.control.nmpc_reduced_model import (
        QuadrupedReducedModel,
        default_urdf_path,
    )

    rm = QuadrupedReducedModel(default_urdf_path())
    est = BaseStateEstimator(ema=1.0)
    q = np.zeros(rm.nq)
    q[2] = 0.24
    q[6] = 1.0
    v = np.zeros(rm.nv)
    out = est.update(
        reduced=rm,
        q_pin=q,
        v_pin=v,
        roll=0.0,
        pitch=0.0,
        yaw=0.0,
        gyro=(0.0, 0.0, 0.0),
        leg_is_stance={"fl": True, "fr": True, "rl": True, "rr": True},
        dt=0.005,
    )
    assert out.z > 0.0
