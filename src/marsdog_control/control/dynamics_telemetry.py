"""Ring-buffer telemetry for WBC/MPC diagnostics.

Stores per-tick dynamics + gait context; supports JSON / flat CSV export and
an end-of-run summary for terminal / analysis gate checks.
"""

from __future__ import annotations

import csv
import math
from collections import deque
from typing import Any, Deque, Dict, List, Optional, Sequence

import numpy as np


_LEGS = ("fl", "fr", "rl", "rr")


class DynamicsTelemetry:
    """Append-only ring buffers; safe to JSON-serialize via ``as_lists``."""

    KEYS = (
        "t",
        "roll",
        "pitch",
        "z",
        "vx",
        "vy",
        "wz",
        "vx_truth",
        "vy_truth",
        "vz_truth",
        "fc_des",
        "tau_opt",
        "contact_state",
        "contact_measured",
        "contact_scheduled",
        "force_scale",
        "phase",
        "amp_front",
        "amp_rear",
        "period",
        "stance_ratio",
        "speed_frac",
        "ramp_frac",
        "vx_cmd",
        "vy_cmd",
        "base_acc_des",
        "foot_pos_actual",
        "foot_pos_des",
        "foot_z",
        "foot_vz",
        "q_err_rms",
        "dtau_max",
        "mpc_ok",
        "wbc_ok",
        "estimate_mode",
    )

    def __init__(self, maxlen: int = 4000):
        self.maxlen = int(maxlen)
        self.buffers: Dict[str, Deque] = {
            k: deque(maxlen=self.maxlen) for k in self.KEYS
        }

    def reset(self) -> None:
        for buf in self.buffers.values():
            buf.clear()

    def record(self, **kwargs: Any) -> None:
        for key, value in kwargs.items():
            if key not in self.buffers:
                continue
            self.buffers[key].append(value)

    def as_lists(self) -> Dict[str, list]:
        out: Dict[str, list] = {}
        for key, buf in self.buffers.items():
            items = []
            for v in buf:
                if isinstance(v, np.ndarray):
                    items.append(v.tolist())
                else:
                    items.append(v)
            out[key] = items
        return out

    def __getitem__(self, key: str) -> Deque:
        return self.buffers[key]

    def get(self, key: str, default=None):
        return self.buffers.get(key, default)

    def items(self):
        return self.buffers.items()

    def keys(self):
        return self.buffers.keys()

    def __len__(self) -> int:
        return len(self.buffers["t"])

    # ------------------------------------------------------------------
    # Analysis helpers
    # ------------------------------------------------------------------

    def summary(self) -> Dict[str, float]:
        """Scalar end-of-run metrics for gates / terminal print."""
        n = len(self)
        empty = {
            "n": 0.0,
            "duration_s": 0.0,
            "roll_peak_deg": 0.0,
            "roll_p95_deg": 0.0,
            "pitch_peak_deg": 0.0,
            "vx_cmd_mean": 0.0,
            "vx_est_mean": 0.0,
            "vx_truth_mean": 0.0,
            "dtau_p95": 0.0,
            "dtau_flip_p95": 0.0,
            "contact_mismatch_pct": 0.0,
            "mpc_ok_pct": 0.0,
            "wbc_ok_pct": 0.0,
            "amp_front_mean_cm": 0.0,
            "amp_rear_mean_cm": 0.0,
            "period_mean_s": 0.0,
            "speed_frac_mean": 0.0,
            "q_err_rms_mean_deg": 0.0,
            "foot_z_min_m": 0.0,
            "fz_peak_n": 0.0,
        }
        if n < 2:
            return empty

        t = np.asarray(list(self.buffers["t"]), dtype=float)
        roll = np.asarray(list(self.buffers["roll"]), dtype=float)
        pitch = np.asarray(list(self.buffers["pitch"]), dtype=float)
        vx = np.asarray(list(self.buffers["vx"]), dtype=float)
        vx_cmd = np.asarray(list(self.buffers["vx_cmd"]), dtype=float)
        vx_truth = np.asarray(list(self.buffers["vx_truth"]), dtype=float)
        dtau = np.asarray(list(self.buffers["dtau_max"]), dtype=float)
        amp_f = np.asarray(list(self.buffers["amp_front"]), dtype=float)
        amp_r = np.asarray(list(self.buffers["amp_rear"]), dtype=float)
        period = np.asarray(list(self.buffers["period"]), dtype=float)
        speed_frac = np.asarray(list(self.buffers["speed_frac"]), dtype=float)
        q_err = np.asarray(list(self.buffers["q_err_rms"]), dtype=float)
        mpc_ok = np.asarray(list(self.buffers["mpc_ok"]), dtype=float)
        wbc_ok = np.asarray(list(self.buffers["wbc_ok"]), dtype=float)

        c_sched = np.asarray(list(self.buffers["contact_scheduled"]), dtype=float)
        c_meas = np.asarray(list(self.buffers["contact_measured"]), dtype=float)
        fc = np.asarray(list(self.buffers["fc_des"]), dtype=float)
        foot_z = np.asarray(list(self.buffers["foot_z"]), dtype=float)

        # Contact flip mask: any leg scheduled 0↔1
        flip = np.zeros(n, dtype=bool)
        if c_sched.ndim == 2 and c_sched.shape[0] >= 2:
            flip[1:] = np.any(np.abs(np.diff(c_sched, axis=0)) > 0.5, axis=1)

        dtau_all = dtau[1:] if n > 1 else dtau
        dtau_flip = dtau[flip] if np.any(flip) else np.array([0.0])

        mismatch = 0.0
        if c_sched.ndim == 2 and c_meas.ndim == 2 and c_sched.size:
            mismatch = 100.0 * float(np.mean(c_sched != c_meas))

        fz_peak = 0.0
        if fc.ndim == 2 and fc.shape[1] >= 12:
            fz_peak = float(np.max(np.abs(fc[:, [2, 5, 8, 11]])))

        foot_z_min = float(np.min(foot_z)) if foot_z.size else 0.0

        def _p95(a: np.ndarray) -> float:
            if a.size == 0:
                return 0.0
            return float(np.percentile(np.abs(a), 95))

        return {
            "n": float(n),
            "duration_s": float(t[-1] - t[0]) if n > 1 else 0.0,
            "roll_peak_deg": float(np.degrees(np.max(np.abs(roll)))),
            "roll_p95_deg": float(np.degrees(_p95(roll))),
            "pitch_peak_deg": float(np.degrees(np.max(np.abs(pitch)))),
            "vx_cmd_mean": float(np.mean(vx_cmd)),
            "vx_est_mean": float(np.mean(vx)),
            "vx_truth_mean": float(np.mean(vx_truth)),
            "dtau_p95": _p95(dtau_all),
            "dtau_flip_p95": _p95(dtau_flip),
            "contact_mismatch_pct": mismatch,
            "mpc_ok_pct": 100.0 * float(np.mean(mpc_ok)) if mpc_ok.size else 0.0,
            "wbc_ok_pct": 100.0 * float(np.mean(wbc_ok)) if wbc_ok.size else 0.0,
            "amp_front_mean_cm": 100.0 * float(np.mean(np.abs(amp_f))),
            "amp_rear_mean_cm": 100.0 * float(np.mean(np.abs(amp_r))),
            "period_mean_s": float(np.mean(period[period > 0])) if np.any(period > 0) else 0.0,
            "speed_frac_mean": float(np.mean(speed_frac)),
            "q_err_rms_mean_deg": float(np.degrees(np.mean(q_err))) if q_err.size else 0.0,
            "foot_z_min_m": foot_z_min,
            "fz_peak_n": fz_peak,
        }

    def format_summary(self, prefix: str = "[Tel]") -> str:
        s = self.summary()
        if s["n"] < 2:
            return f"{prefix} no samples"
        return (
            f"{prefix} n={int(s['n'])} dur={s['duration_s']:.2f}s | "
            f"roll_pk/p95={s['roll_peak_deg']:.1f}/{s['roll_p95_deg']:.1f}° "
            f"pitch_pk={s['pitch_peak_deg']:.1f}° | "
            f"vx cmd/est/truth={s['vx_cmd_mean']:.3f}/{s['vx_est_mean']:.3f}/{s['vx_truth_mean']:.3f} | "
            f"|Δτ| p95={s['dtau_p95']:.2f} flip_p95={s['dtau_flip_p95']:.2f}Nm | "
            f"amp={s['amp_front_mean_cm']:.1f}/{s['amp_rear_mean_cm']:.1f}cm "
            f"T={s['period_mean_s']:.2f}s frac={s['speed_frac_mean']:.2f} | "
            f"mismatch={s['contact_mismatch_pct']:.1f}% "
            f"mpc/wbc_ok={s['mpc_ok_pct']:.0f}/{s['wbc_ok_pct']:.0f}% "
            f"q_err={s['q_err_rms_mean_deg']:.2f}° Fz_pk={s['fz_peak_n']:.0f}N "
            f"z_foot_min={s['foot_z_min_m']:.3f}m"
        )

    def write_csv(self, path: str) -> int:
        """Write a flat CSV; returns row count."""
        n = len(self)
        if n == 0:
            return 0

        rows: List[Dict[str, Any]] = []
        for i in range(n):
            row: Dict[str, Any] = {}
            for key in self.KEYS:
                buf = self.buffers[key]
                if i >= len(buf):
                    continue
                val = buf[i]
                if isinstance(val, np.ndarray):
                    val = val.tolist()
                if isinstance(val, (list, tuple)):
                    flat = _flatten_named(key, val)
                    row.update(flat)
                else:
                    row[key] = val
            rows.append(row)

        # Stable column order: scalars first, then discovered flats
        fieldnames: List[str] = []
        seen = set()
        preferred = [
            "t", "roll", "pitch", "z", "vx", "vy", "wz",
            "vx_truth", "vy_truth", "vz_truth", "vx_cmd", "vy_cmd",
            "amp_front", "amp_rear", "period", "stance_ratio",
            "speed_frac", "ramp_frac", "q_err_rms", "dtau_max",
            "mpc_ok", "wbc_ok", "estimate_mode",
        ]
        for name in preferred:
            if any(name in r for r in rows) and name not in seen:
                fieldnames.append(name)
                seen.add(name)
        for r in rows:
            for k in r:
                if k not in seen:
                    fieldnames.append(k)
                    seen.add(k)

        with open(path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(rows)
        return n


def _flatten_named(key: str, val: Sequence) -> Dict[str, Any]:
    """Expand vector channels into CSV-friendly columns."""
    out: Dict[str, Any] = {}
    if key in ("force_scale", "phase", "contact_state",
               "contact_measured", "contact_scheduled", "foot_z", "foot_vz"):
        for i, leg in enumerate(_LEGS):
            if i < len(val):
                out[f"{key}_{leg}"] = val[i]
        return out
    if key in ("foot_pos_actual", "foot_pos_des", "fc_des"):
        xyz = ("x", "y", "z")
        for i, leg in enumerate(_LEGS):
            for j, ax in enumerate(xyz):
                idx = i * 3 + j
                if idx < len(val):
                    out[f"{key}_{leg}_{ax}"] = val[idx]
        return out
    if key == "base_acc_des":
        labels = ("ax", "ay", "az", "aroll", "apitch", "ayaw")
        for i, lab in enumerate(labels):
            if i < len(val):
                out[f"base_acc_{lab}"] = val[i]
        return out
    if key == "tau_opt":
        for i, v in enumerate(val):
            out[f"tau_{i}"] = v
        return out
    for i, v in enumerate(val):
        out[f"{key}_{i}"] = v
    return out


__all__ = ["DynamicsTelemetry"]
