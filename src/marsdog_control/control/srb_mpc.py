"""Convex single-rigid-body MPC (MIT-style) with per-step dynamics update."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import numpy as np
import osqp
import scipy.sparse as sparse


@dataclass
class SrbMpcConfig:
    horizon: int = 10
    dt: float = 0.03
    mass: float = 10.473
    mu: float = 0.6
    f_max: float = 80.0  # ~3 * mg / 4

    # Q: [r,p,y, x,y,z, wx,wy,wz, vx,vy,vz, g]
    # Roll / lateral heavily weighted — diagonal trot rock is the main complaint.
    weights_Q: np.ndarray = field(
        default_factory=lambda: np.array(
            [
                90.0,  # roll
                22.0,  # pitch
                4.0,   # yaw
                1.0,   # x
                14.0,  # y
                60.0,  # z
                14.0,  # wx
                8.0,   # wy
                2.0,   # wz
                18.0,  # vx
                48.0,  # vy
                6.0,   # vz
                0.0,
            ]
        )
    )
    weights_R: np.ndarray = field(default_factory=lambda: np.ones(12) * 2.0e-3)


class SrbMpc:
    """SRB-MPC over contact forces; rebuilds OSQP each solve (correct P/A)."""

    def __init__(self, config: SrbMpcConfig, inertia: np.ndarray):
        self.cfg = config
        self.I = np.asarray(inertia, dtype=float)
        self.I_inv = np.linalg.inv(self.I)

        self.H = config.horizon
        self.nx = 13
        self.nu = 12

        self.A_qp = np.zeros((self.nx * self.H, self.nx))
        self.B_qp = np.zeros((self.nx * self.H, self.nu * self.H))
        self.C_friction = self._build_friction_matrix()

        self._last_forces = np.zeros(12)
        self.last_solve_ok = False
        self.fail_count = 0
        self._gravity_share: Optional[np.ndarray] = None

    def _gravity_fallback(self, contact_state: np.ndarray) -> np.ndarray:
        """Vertical support weighted by continuous contact schedule (step 0)."""
        f = np.zeros(12)
        w = np.clip(np.asarray(contact_state[:4], dtype=float), 0.0, 1.0)
        wsum = float(np.sum(w))
        if wsum < 1e-6:
            return self._last_forces.copy() if np.any(self._last_forces) else f
        mg = self.cfg.mass * 9.81
        for i in range(4):
            if w[i] > 1e-6:
                f[i * 3 + 2] = mg * (w[i] / wsum)
        return f

    def _get_continuous_dynamics(
        self, yaw: float, r_feet: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        Ac = np.zeros((13, 13))
        Bc = np.zeros((13, 12))

        cos_y = np.cos(yaw)
        sin_y = np.sin(yaw)
        R_yaw = np.array(
            [[cos_y, -sin_y, 0.0], [sin_y, cos_y, 0.0], [0.0, 0.0, 1.0]]
        )

        Ac[0:3, 6:9] = R_yaw
        Ac[3:6, 9:12] = np.eye(3)
        Ac[11, 12] = 1.0

        for i in range(4):
            r = r_feet[:, i]
            r_skew = np.array(
                [
                    [0.0, -r[2], r[1]],
                    [r[2], 0.0, -r[0]],
                    [-r[1], r[0], 0.0],
                ]
            )
            Bc[6:9, i * 3 : (i + 1) * 3] = self.I_inv @ R_yaw.T @ r_skew
            Bc[9:12, i * 3 : (i + 1) * 3] = np.eye(3) / self.cfg.mass

        return Ac, Bc

    def _build_friction_matrix(self) -> np.ndarray:
        C_leg = np.array(
            [
                [1.0, 0.0, -self.cfg.mu],
                [-1.0, 0.0, -self.cfg.mu],
                [0.0, 1.0, -self.cfg.mu],
                [0.0, -1.0, -self.cfg.mu],
                [0.0, 0.0, -1.0],
                [0.0, 0.0, 1.0],
            ]
        )
        C = np.zeros((24, 12))
        for i in range(4):
            C[i * 6 : (i + 1) * 6, i * 3 : (i + 1) * 3] = C_leg
        return C

    def solve(
        self,
        x0: np.ndarray,
        x_ref: np.ndarray,
        r_feet: np.ndarray,
        contact_state: np.ndarray,
    ) -> np.ndarray:
        yaw = float(x0[2])
        Ac, Bc = self._get_continuous_dynamics(yaw, r_feet)

        Ad = np.eye(13) + Ac * self.cfg.dt
        Bd = Bc * self.cfg.dt

        A_power = np.eye(13)
        self.B_qp.fill(0.0)
        for i in range(self.H):
            A_power = Ad @ A_power
            self.A_qp[i * 13 : (i + 1) * 13, :] = A_power
            for j in range(i + 1):
                self.B_qp[
                    i * 13 : (i + 1) * 13, j * 12 : (j + 1) * 12
                ] = np.linalg.matrix_power(Ad, i - j) @ Bd

        Q_bar = np.diag(np.tile(self.cfg.weights_Q, self.H))
        R_bar = np.diag(np.tile(self.cfg.weights_R, self.H))

        H_qp = self.B_qp.T @ Q_bar @ self.B_qp + R_bar
        q_qp = self.B_qp.T @ Q_bar @ (self.A_qp @ x0 - x_ref)

        lb = np.zeros(24 * self.H)
        ub = np.zeros(24 * self.H)
        C_qp = np.zeros((24 * self.H, 12 * self.H))
        for i in range(self.H):
            C_qp[i * 24 : (i + 1) * 24, i * 12 : (i + 1) * 12] = self.C_friction
            for leg in range(4):
                # Continuous schedule weight ∈[0,1]: soft TD/LO via f_max scale
                c = float(np.clip(contact_state[i * 4 + leg], 0.0, 1.0))
                idx = i * 24 + leg * 6
                if c > 1e-3:
                    lb[idx : idx + 5] = -np.inf
                    ub[idx : idx + 5] = 0.0
                    lb[idx + 5] = 0.0
                    ub[idx + 5] = self.cfg.f_max * c
                else:
                    lb[idx : idx + 6] = 0.0
                    ub[idx : idx + 6] = 0.0

        P_sparse = sparse.triu(sparse.csc_matrix(H_qp), format="csc")
        A_sparse = sparse.csc_matrix(C_qp)

        # Rebuild each step so P reflects current yaw / r_feet (correctness > micro-opt).
        solver = osqp.OSQP()
        solver.setup(
            P=P_sparse,
            q=q_qp,
            A=A_sparse,
            l=lb,
            u=ub,
            verbose=False,
            warm_starting=True,
            polishing=False,
            eps_abs=1e-3,
            eps_rel=1e-3,
            max_iter=4000,
        )
        if np.any(self._last_forces):
            x0_ws = np.tile(self._last_forces, self.H)
            solver.warm_start(x=x0_ws)

        res = solver.solve()

        if res.info.status_val == osqp.constant("OSQP_SOLVED") or res.info.status_val == osqp.constant(
            "OSQP_SOLVED_INACCURATE"
        ):
            optimal_forces = np.asarray(res.x[:12], dtype=float).copy()
            self._last_forces = optimal_forces
            self.last_solve_ok = True
            self.fail_count = 0
            return optimal_forces

        print(f"[SRB-MPC] solve failed: {res.info.status}")
        self.last_solve_ok = False
        self.fail_count += 1
        if np.any(self._last_forces):
            return self._last_forces.copy()
        return self._gravity_fallback(contact_state)
