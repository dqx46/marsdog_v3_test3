"""SRB-MPC force planning + edge scale + EMA + rate limit.

MPC can run slower than the 200 Hz impedance loop via ``mpc_period_s``
(e.g. 0.02 → 50 Hz on RK3588). Default 0 = solve every call (sim / full rate).
"""

from __future__ import annotations

from typing import Mapping, Optional

import numpy as np

from marsdog_control.control.srb_mpc import SrbMpc


class ForcePlanner:
    """Owns contact-force reference for WBC force-tracking task."""

    def __init__(
        self,
        mpc: SrbMpc,
        *,
        force_lpf_alpha: float = 0.10,
        max_df_dt: float = 700.0,  # N/s per force component
        dt: float = 0.005,
        # Solve MPC at this period; hold+LPF between solves. 0 = every call.
        # Sim / pre-RK3588 default: every tick. Real RK3588 can pass 0.02.
        mpc_period_s: float = 0.0,
    ):
        self.mpc = mpc
        self.force_lpf_alpha = float(force_lpf_alpha)
        self.max_df_dt = float(max_df_dt)
        self.dt = float(dt)
        self.mpc_period_s = float(mpc_period_s)
        self._fc_filt = np.zeros(12)
        self._fc_raw = np.zeros(12)
        self._since_mpc_s = 1e9  # force first plan() to solve
        self.last_ok = False
        self.last_mpc_ms = 0.0
        self.last_did_solve = False

    def reset(self) -> None:
        self._fc_filt[:] = 0.0
        self._fc_raw[:] = 0.0
        self._since_mpc_s = 1e9
        self.last_ok = False
        self.last_did_solve = False

    def plan(
        self,
        *,
        x0: np.ndarray,
        x_ref: np.ndarray,
        r_feet: np.ndarray,
        contact_horizon: np.ndarray,
        force_scale: Mapping[str, float],
        dt: Optional[float] = None,
    ) -> np.ndarray:
        step_dt = float(dt) if dt is not None else self.dt
        self._since_mpc_s += step_dt
        do_mpc = (
            self.mpc_period_s <= 1e-9
            or self._since_mpc_s >= self.mpc_period_s
        )
        self.last_did_solve = do_mpc
        if do_mpc:
            f_raw = self.mpc.solve(x0, x_ref, r_feet, contact_horizon)
            self.last_ok = bool(self.mpc.last_solve_ok)
            self.last_mpc_ms = float(getattr(self.mpc, "last_solve_ms", 0.0))
            self._fc_raw = np.asarray(f_raw, dtype=float).copy()
            self._since_mpc_s = 0.0
        else:
            # Hold last MPC solution; still apply current edge scale below.
            self.last_mpc_ms = 0.0

        f = self._fc_raw.copy()
        for li, leg in enumerate(("fl", "fr", "rl", "rr")):
            s = float(force_scale.get(leg, 1.0))
            f[li * 3 : li * 3 + 3] *= max(0.0, min(1.0, s))

        # Low alpha = heavier smoothing (cut chatter)
        a = max(0.05, min(0.5, self.force_lpf_alpha))
        target = a * f + (1.0 - a) * self._fc_filt

        # Rate limit each component
        max_step = self.max_df_dt * max(1e-4, step_dt)
        delta = np.clip(target - self._fc_filt, -max_step, max_step)
        self._fc_filt = self._fc_filt + delta
        return self._fc_filt.copy()


__all__ = ["ForcePlanner"]
