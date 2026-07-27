"""Gait-phase prior + kinematic touchdown/liftoff contact schedule.

Design (anti-chatter):
  - MPC horizon contact weights follow the **phase schedule** (continuous edge).
  - Measurement only modulates continuous ``force_scale``.
  - WBC blends stance/swing tasks by ``force_scale`` (no hard boolean QP flip).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Mapping, Optional

import numpy as np

_LEGS = ("fl", "fr", "rl", "rr")


@dataclass
class ContactConfig:
    """Kinematic contact thresholds (SI: m, m/s, phase fraction)."""

    td_height_m: float = 0.010  # acquire contact below this (absolute fallback)
    lo_height_m: float = 0.030  # release only above this (absolute fallback)
    vz_td_max: float = 0.25
    vz_lo_min: float = 0.08
    phase_early_td: float = 0.06
    phase_late_lo: float = 0.10
    measure_force_weight: float = 0.15  # kinematic meas → force_scale only
    edge_blend: float = 0.12  # force ramp window (phase fraction)
    hold_steps: int = 6  # debounce ~30ms @200Hz
    # Soft WBC: reserved; stance flags follow schedule for QP stability
    wbc_follow_measure: bool = False
    # Relative z vs per-leg mid-stance baseline (WHY: rear world-z ≈ −8 mm
    # never clears absolute lo=30 mm → stuck measured-stance / mismatch ~20%).
    use_relative_z: bool = True
    rel_td_m: float = 0.006  # acquire when (z − z_ref) below this
    rel_lo_m: float = 0.012  # release when (z − z_ref) above this
    z_ref_ema: float = 0.08  # mid-stance baseline EMA


@dataclass
class ContactSnapshot:
    """One-step contact view for WBC/MPC/telemetry."""

    phase: Dict[str, float] = field(
        default_factory=lambda: {leg: 0.0 for leg in _LEGS}
    )
    scheduled: Dict[str, bool] = field(
        default_factory=lambda: {leg: True for leg in _LEGS}
    )
    measured: Dict[str, bool] = field(
        default_factory=lambda: {leg: True for leg in _LEGS}
    )
    # Effective contact for WBC stance/swing tasks
    stance: Dict[str, bool] = field(
        default_factory=lambda: {leg: True for leg in _LEGS}
    )
    force_scale: Dict[str, float] = field(
        default_factory=lambda: {leg: 1.0 for leg in _LEGS}
    )


class ContactSchedule:
    """Maintains per-leg contact with phase prior + kinematic detection."""

    def __init__(self, config: Optional[ContactConfig] = None):
        self.cfg = config or ContactConfig()
        self._measured = {leg: True for leg in _LEGS}
        self._flip_count = {leg: 0 for leg in _LEGS}
        self._force_scale_filt = {leg: 1.0 for leg in _LEGS}
        self._z_ref = {leg: 0.0 for leg in _LEGS}
        self._z_ref_ready = {leg: False for leg in _LEGS}

    def reset(self) -> None:
        self._measured = {leg: True for leg in _LEGS}
        self._flip_count = {leg: 0 for leg in _LEGS}
        self._force_scale_filt = {leg: 1.0 for leg in _LEGS}
        self._z_ref = {leg: 0.0 for leg in _LEGS}
        self._z_ref_ready = {leg: False for leg in _LEGS}

    @staticmethod
    def phase_of(leg: str, t_rel: float, gait) -> float:
        if gait is None:
            return 0.0
        period = max(1e-6, float(gait.period))
        offset = gait._PHASE_OFFSET[leg]
        return (t_rel / period + offset) % 1.0

    def update(
        self,
        *,
        t_rel: float,
        gait,
        foot_z_world: Mapping[str, float],
        foot_vz_world: Mapping[str, float],
    ) -> ContactSnapshot:
        snap = ContactSnapshot()
        cfg = self.cfg
        stance_ratio = float(getattr(gait, "stance_ratio", 1.0)) if gait else 1.0

        for leg in _LEGS:
            phase = self.phase_of(leg, t_rel, gait) if gait is not None else 0.0
            snap.phase[leg] = phase
            scheduled = phase <= stance_ratio if gait is not None else True
            snap.scheduled[leg] = scheduled

            z = float(foot_z_world.get(leg, 0.05))
            vz = float(foot_vz_world.get(leg, 0.0))
            candidate = self._measured[leg]

            # Mid-stance baseline for relative height (rear world-z offset)
            in_mid = (
                gait is not None
                and scheduled
                and cfg.edge_blend < phase < (stance_ratio - cfg.edge_blend)
            )
            if in_mid:
                if not self._z_ref_ready[leg]:
                    self._z_ref[leg] = z
                    self._z_ref_ready[leg] = True
                else:
                    a = float(cfg.z_ref_ema)
                    self._z_ref[leg] = (1.0 - a) * self._z_ref[leg] + a * z

            use_rel = bool(cfg.use_relative_z) and self._z_ref_ready[leg]
            if use_rel:
                z_cmp = z - self._z_ref[leg]
                td_thr = float(cfg.rel_td_m)
                lo_thr = float(cfg.rel_lo_m)
            else:
                z_cmp = z
                td_thr = float(cfg.td_height_m)
                lo_thr = float(cfg.lo_height_m)

            # Height hysteresis: different thresholds for TD vs LO
            if candidate:
                want_lo = z_cmp > lo_thr and (
                    vz > cfg.vz_lo_min or phase > stance_ratio
                )
                # Only unload if schedule agrees OR clearly past stance
                if want_lo and (not scheduled or phase > stance_ratio + cfg.phase_late_lo):
                    candidate = False
            else:
                in_td_window = scheduled or phase > (1.0 - cfg.phase_early_td)
                want_td = in_td_window and z_cmp < td_thr and abs(vz) < cfg.vz_td_max
                if want_td:
                    candidate = True

            if candidate != self._measured[leg]:
                self._flip_count[leg] += 1
                if self._flip_count[leg] >= cfg.hold_steps:
                    self._measured[leg] = candidate
                    self._flip_count[leg] = 0
            else:
                self._flip_count[leg] = 0

            measured = self._measured[leg]
            # Mid-stance: trust schedule (kills false mid-stance unloads)
            if (
                gait is not None
                and scheduled
                and cfg.edge_blend < phase < (stance_ratio - cfg.edge_blend)
            ):
                measured = True
                self._measured[leg] = True
                self._flip_count[leg] = 0

            snap.measured[leg] = measured

            # Binary stance kept for estimator / CoM diagnostics.
            # WBC QP now soft-blends via force_scale (avoids LO/TD torque jumps).
            snap.stance[leg] = scheduled if gait is not None else measured

            # Continuous force scale: phase edge × soft measured confidence
            edge = self._phase_edge_scale(phase, stance_ratio)
            meas_s = 1.0 if measured else 0.0
            w = float(cfg.measure_force_weight)
            raw = edge * ((1.0 - w) * (1.0 if scheduled else 0.0) + w * meas_s)
            # Extra: if scheduled stance but measured unload, bleed force down
            if scheduled and not measured:
                raw = min(raw, 0.25 * edge)
            if not scheduled and measured:
                raw = max(raw, 0.20 * edge)

            # EMA force scale (~15Hz equivalent at 200Hz)
            a = 0.12
            prev = self._force_scale_filt[leg]
            filt = (1.0 - a) * prev + a * raw
            self._force_scale_filt[leg] = filt
            snap.force_scale[leg] = float(max(0.0, min(1.0, filt)))

        return snap

    def _phase_edge_scale(self, phase: float, stance_ratio: float) -> float:
        edge = float(self.cfg.edge_blend)
        if edge <= 1e-9:
            return 1.0 if phase <= stance_ratio else 0.0
        if phase <= stance_ratio:
            if phase < edge:
                return max(0.0, min(1.0, phase / edge))
            if phase > stance_ratio - edge:
                return max(0.0, min(1.0, (stance_ratio - phase) / edge))
            return 1.0
        rem = 1.0 - phase
        if rem < edge:
            # approaching TD from swing
            return max(0.0, min(1.0, 1.0 - rem / edge)) * 0.5
        return 0.0

    def horizon(
        self,
        *,
        t_rel: float,
        gait,
        horizon: int,
        dt: float,
        measured: Mapping[str, bool],
    ) -> np.ndarray:
        """Build 4*H continuous contact weights for MPC — **phase schedule only**.

        Values are in [0, 1] via ``_phase_edge_scale`` so SRB-MPC can scale
        ``f_max`` near TD/LO instead of hard 0/1 flips (main dFz spike source).
        Measured contact must not hard-cut future horizon.
        ``measured`` kept for API compatibility / future soft bias.
        """
        del measured  # intentionally unused for hard flags
        H = int(horizon)
        out = np.zeros(4 * H, dtype=float)
        if gait is None:
            out[:] = 1.0
            return out

        stance_ratio = float(gait.stance_ratio)
        for k in range(H):
            t_k = t_rel + k * dt
            for li, leg in enumerate(_LEGS):
                phase = self.phase_of(leg, t_k, gait)
                out[k * 4 + li] = float(
                    self._phase_edge_scale(phase, stance_ratio)
                )
        return out


__all__ = ["ContactConfig", "ContactSnapshot", "ContactSchedule"]
