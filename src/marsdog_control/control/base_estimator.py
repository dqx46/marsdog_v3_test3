"""Stance-foot + IMU base velocity/height estimator (sim/real parity).

Uses least-squares on stance-foot Jacobians with known gyro + joint rates.
Lateral velocity is more aggressively filtered — it drives WBC Ay damping.

Mid-stance window (WHY): edge TD/LO feet violate no-slip → corrupt LS.
Slip weights (WHY): residual ‖A v − b‖ large ⇒ foot sliding; downweight.
Scrub offset (WHY): soft-trot no-slip LS tracks gait cmd (~0.08) while
  truth overruns by ~0.05–0.06 m/s (kinetic scrub). Additive prior closes
  mean(vx_truth−vx) toward <0.03 until IMU linear-accel fusion exists.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Mapping, Optional, Tuple

import numpy as np
import pinocchio as pin

from marsdog_control.control.velocity_model import VX_SCRUB_OFFSET_MPS


@dataclass
class BaseEstimate:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.24
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0


class BaseStateEstimator:
    """Estimate base linear velocity from stance-foot no-slip + IMU gyro."""

    def __init__(
        self,
        ema: float = 0.12,
        ema_lat: float = 0.06,
        v_clip: float = 1.2,
        v_lat_clip: float = 0.45,
        foot_names=None,
        edge_blend: float = 0.12,
        force_scale_min: float = 0.80,
        slip_thresh: float = 0.08,
        # Forward scrub prior (m/s) — must match gait_schedule vel_cmd bias
        scrub_offset_x: float = VX_SCRUB_OFFSET_MPS,
        scrub_engage_vx: float = 0.02,
    ):
        self.ema = float(ema)
        self.ema_lat = float(ema_lat)
        self.v_clip = float(v_clip)
        self.v_lat_clip = float(v_lat_clip)
        self.edge_blend = float(edge_blend)
        self.force_scale_min = float(force_scale_min)
        self.slip_thresh = float(slip_thresh)
        self.scrub_offset_x = float(scrub_offset_x)
        self.scrub_engage_vx = float(scrub_engage_vx)
        self.foot_names = foot_names or [
            "fl_foot_link",
            "fr_foot_link",
            "rl_foot_link",
            "rr_foot_link",
        ]
        self.p = np.zeros(3)
        self.v = np.zeros(3)
        self._initialized = False

    def reset(self) -> None:
        self.p[:] = 0.0
        self.v[:] = 0.0
        self._initialized = False

    @staticmethod
    def _in_mid_stance(phase: float, stance_ratio: float, edge: float) -> bool:
        return edge < phase < (stance_ratio - edge)

    def update(
        self,
        *,
        reduced,
        q_pin: np.ndarray,
        v_pin: np.ndarray,
        roll: float,
        pitch: float,
        yaw: float,
        gyro: Tuple[float, float, float],
        leg_is_stance: Dict[str, bool],
        dt: float = 0.005,
        leg_phase: Optional[Mapping[str, float]] = None,
        stance_ratio: float = 1.0,
        edge_blend: Optional[float] = None,
        force_scale: Optional[Mapping[str, float]] = None,
    ) -> BaseEstimate:
        del roll, pitch, yaw  # orientation already baked into q_pin / gyro
        model = reduced.model
        data = reduced.data

        pin.forwardKinematics(model, data, q_pin, v_pin)
        pin.updateFramePlacements(model, data)
        pin.computeJointJacobians(model, data, q_pin)

        base_z_assumed = float(q_pin[2]) if q_pin.shape[0] > 2 else 0.24
        z_sum = 0.0
        n_st = 0
        A_rows = []
        b_rows = []

        w = np.array(gyro, dtype=float)
        v_known = np.array(v_pin, dtype=float, copy=True)
        v_known[0:3] = 0.0
        v_known[3:6] = w

        edge = float(self.edge_blend if edge_blend is None else edge_blend)
        sr = float(stance_ratio)
        slip_th = max(1e-4, float(self.slip_thresh))

        candidates = []
        for fname in self.foot_names:
            leg = fname[:2]
            if not leg_is_stance.get(leg, True):
                continue
            phase_ok = True
            if leg_phase is not None:
                ph = float(leg_phase.get(leg, 0.0))
                phase_ok = self._in_mid_stance(ph, sr, edge)
            fs_ok = True
            if force_scale is not None:
                fs_ok = float(force_scale.get(leg, 1.0)) >= self.force_scale_min
            candidates.append((fname, leg, phase_ok and fs_ok))

        use_mid = any(ok for _, _, ok in candidates)
        selected = [
            (fname, leg)
            for fname, leg, ok in candidates
            if (ok if use_mid else True)
        ]

        for fname, leg in selected:
            fid = model.getFrameId(fname)
            foot_z = data.oMf[fid].translation[2]
            z_sum += -(foot_z - base_z_assumed)
            n_st += 1

            J = pin.getFrameJacobian(
                model, data, fid, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
            )
            J_lin = J[:3, :]
            A_i = J_lin[:, 0:3]
            b_i = -(J_lin[:, 3:] @ v_known[3:])

            if self._initialized:
                r = A_i @ self.v - b_i
                r_n = float(np.linalg.norm(r))
                wt = 1.0 / (1.0 + (r_n / slip_th) ** 2)
            else:
                wt = 1.0
            A_rows.append(wt * A_i)
            b_rows.append(wt * b_i)

        if n_st > 0:
            z_est = z_sum / n_st
            A = np.vstack(A_rows)
            b = np.concatenate(b_rows)
            try:
                v_raw, *_ = np.linalg.lstsq(A, b, rcond=1e-4)
            except np.linalg.LinAlgError:
                v_raw = self.v.copy()
            v_raw = np.asarray(v_raw, dtype=float).reshape(3)
        else:
            z_est = base_z_assumed
            v_raw = 0.92 * self.v

        # Friction scrub prior: only when mid-stance LS sees forward motion
        if (
            use_mid
            and abs(float(v_raw[0])) >= self.scrub_engage_vx
            and self.scrub_offset_x > 1e-9
        ):
            v_raw[0] = float(v_raw[0] + np.sign(v_raw[0]) * self.scrub_offset_x)

        v_raw[0] = float(np.clip(v_raw[0], -self.v_clip, self.v_clip))
        v_raw[1] = float(np.clip(v_raw[1], -self.v_lat_clip, self.v_lat_clip))
        v_raw[2] = float(np.clip(v_raw[2], -self.v_clip, self.v_clip))

        if not self._initialized:
            self.p[:] = [0.0, 0.0, z_est]
            self.v[:] = v_raw
            self._initialized = True
        else:
            ax, az = self.ema, self.ema
            ay = self.ema_lat
            self.v[0] = (1.0 - ax) * self.v[0] + ax * v_raw[0]
            self.v[1] = (1.0 - ay) * self.v[1] + ay * v_raw[1]
            self.v[2] = (1.0 - az) * self.v[2] + az * v_raw[2]
            self.p[0] += self.v[0] * dt
            self.p[1] += self.v[1] * dt
            self.p[2] = (1.0 - ax) * self.p[2] + ax * z_est

        return BaseEstimate(
            x=float(self.p[0]),
            y=float(self.p[1]),
            z=float(self.p[2]),
            vx=float(self.v[0]),
            vy=float(self.v[1]),
            vz=float(self.v[2]),
        )


__all__ = ["BaseEstimate", "BaseStateEstimator"]
