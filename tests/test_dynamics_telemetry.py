"""Unit tests for DynamicsTelemetry export / summary."""

from __future__ import annotations

import os
import tempfile

import numpy as np

from marsdog_control.control.dynamics_telemetry import DynamicsTelemetry


def _fill(tel: DynamicsTelemetry, n: int = 40) -> None:
    prev_tau = np.zeros(6)
    for i in range(n):
        t = 0.005 * i
        contact = [1.0, 0.0, 0.0, 1.0] if (i // 10) % 2 == 0 else [0.0, 1.0, 1.0, 0.0]
        tau = prev_tau + (0.5 if i % 10 == 0 else 0.05) * np.ones(6)
        dtau = float(np.max(np.abs(tau - prev_tau))) if i else 0.0
        prev_tau = tau.copy()
        tel.record(
            t=t,
            roll=0.01 * np.sin(t),
            pitch=0.005,
            z=0.24,
            vx=0.08,
            vy=0.0,
            wz=0.0,
            vx_truth=0.10,
            vy_truth=0.0,
            vz_truth=0.0,
            fc_des=np.array([0, 0, 30, 0, 0, 30, 0, 0, 30, 0, 0, 30], dtype=float),
            tau_opt=tau,
            contact_state=contact,
            contact_measured=contact,
            contact_scheduled=contact,
            force_scale=[float(c) for c in contact],
            phase=[0.1, 0.6, 0.6, 0.1],
            amp_front=0.039,
            amp_rear=0.043,
            period=0.95,
            stance_ratio=0.79,
            speed_frac=0.55,
            ramp_frac=1.0,
            vx_cmd=0.087,
            vy_cmd=0.0,
            base_acc_des=np.zeros(6),
            foot_pos_actual=np.zeros(12),
            foot_pos_des=np.zeros(12),
            foot_z=np.array([0.01, 0.02, 0.01, 0.02]),
            foot_vz=np.zeros(4),
            q_err_rms=0.02,
            dtau_max=dtau,
            mpc_ok=True,
            wbc_ok=True,
            estimate_mode="estimator",
        )


def test_summary_and_csv():
    tel = DynamicsTelemetry(maxlen=100)
    _fill(tel, 40)
    s = tel.summary()
    assert s["n"] == 40
    assert s["duration_s"] > 0
    assert s["vx_cmd_mean"] > 0.08
    assert s["amp_front_mean_cm"] > 3.0
    assert s["dtau_p95"] > 0.0
    assert s["dtau_flip_p95"] > 0.0
    assert "vx_est_minus_cmd_mean" in s
    assert s["estimate_mode"] == "estimator"
    text = tel.format_summary()
    assert "vx cmd/est/truth" in text

    with tempfile.TemporaryDirectory() as td:
        path = os.path.join(td, "tel.csv")
        n = tel.write_csv(path)
        assert n == 40
        with open(path) as f:
            header = f.readline()
        assert "force_scale_fl" in header
        assert "amp_front" in header
        assert "dtau_max" in header
        summary_path = os.path.join(td, "summary.json")
        payload = tel.write_summary_json(summary_path, extra={"source": "unit"})
        assert payload["source"] == "unit"
        assert os.path.isfile(summary_path)


def test_as_lists_includes_new_keys():
    tel = DynamicsTelemetry()
    _fill(tel, 3)
    data = tel.as_lists()
    for key in (
        "force_scale",
        "phase",
        "amp_front",
        "foot_pos_des",
        "foot_z",
        "q_err_rms",
        "dtau_max",
    ):
        assert key in data
        assert len(data[key]) == 3
