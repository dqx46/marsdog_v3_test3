"""Baseline (non-WBC) dynamics telemetry for CommandExecutor."""

from __future__ import annotations

import math
from typing import Dict, Optional

import numpy as np

from marsdog_control.config.joints import JOINT_BY_NAME as JBN
from marsdog_control.core.types import RobotState
from marsdog_control.motion import kinematics as K


_LEGS = ("fl", "fr", "rl", "rr")


class ExecutorTelemetryMixin:
    """Record impedance/VMC-path telemetry frames."""

    def _record_baseline_telemetry(
        self,
        state: RobotState,
        targets: dict[int, float],
        active_gait,
        leg_is_stance: dict[str, bool],
        t_rel: float,
        trq_ff: dict[int, float],
    ) -> None:
        """Record gait/IMU telemetry when WBC is off (impedance / VMC baseline)."""
        if self._dyn_tel is None:
            return
        if state is None:
            # Parity / offline harness may call build(None, ...); skip IMU/q_err.
            return
        truth = getattr(state, "vel_xyz", (0.0, 0.0, 0.0))
        vel = truth
        phase_arr = [0.0, 0.0, 0.0, 0.0]
        amp_front = amp_rear = period = stance_ratio = speed_frac = 0.0
        ramp_frac = 1.0
        vx_cmd = vy_cmd = 0.0
        z = float(getattr(active_gait, "height", 0.0) or 0.0) if active_gait else 0.0
        if active_gait is not None:
            period = float(getattr(active_gait, "period", 0.0) or 0.0)
            stance_ratio = float(getattr(active_gait, "stance_ratio", 0.0) or 0.0)
            amp_front = float(getattr(active_gait, "amp_front", 0.0) or 0.0)
            amp_rear = float(getattr(active_gait, "amp_rear", amp_front) or 0.0)
            speed_frac = float(getattr(active_gait, "speed_frac", 0.0) or 0.0)
            vc = getattr(active_gait, "vel_cmd", None)
            if isinstance(vc, (tuple, list)) and len(vc) >= 2:
                vx_cmd = float(vc[0])
                vy_cmd = float(vc[1])
            offsets = getattr(active_gait, "_PHASE_OFFSET", None)
            if period > 1e-6 and isinstance(offsets, dict):
                for i, leg in enumerate(_LEGS):
                    phase_arr[i] = (t_rel / period + float(offsets.get(leg, 0.0))) % 1.0
            ramp_dur = float(getattr(active_gait, "ramp_duration", 0.0) or 0.0)
            if ramp_dur > 1e-6 and t_rel < ramp_dur:
                s = t_rel / ramp_dur
                ramp_frac = s * s * (3.0 - 2.0 * s)

        # Joint tracking RMS (cmd vs measured)
        q_err_sq = 0.0
        q_err_n = 0
        for mid, q_des in targets.items():
            if mid not in state.joint_pos:
                continue
            dq = float(q_des) - float(state.joint_pos[mid])
            q_err_sq += dq * dq
            q_err_n += 1
        q_err_rms = math.sqrt(q_err_sq / q_err_n) if q_err_n else 0.0

        tau_vec = np.zeros(max(1, len(trq_ff)), dtype=float)
        if trq_ff:
            tau_vec = np.asarray(list(trq_ff.values()), dtype=float)
        prev_tau = getattr(self, "_tel_prev_tau_base", None)
        if prev_tau is not None and prev_tau.shape == tau_vec.shape:
            dtau_max = float(np.max(np.abs(tau_vec - prev_tau))) if tau_vec.size else 0.0
        else:
            dtau_max = 0.0
        self._tel_prev_tau_base = tau_vec.copy()

        self._dyn_tel.record(
            t=t_rel,
            roll=float(state.roll),
            pitch=float(state.pitch),
            z=z,
            vx=float(vel[0]),
            vy=float(vel[1]),
            wz=float(state.gyro_yaw),
            vx_truth=float(truth[0]),
            vy_truth=float(truth[1]),
            vz_truth=float(truth[2]),
            fc_des=np.zeros(12),
            tau_opt=tau_vec,
            contact_state=[1.0 if leg_is_stance.get(l, True) else 0.0 for l in _LEGS],
            contact_measured=[0.0] * 4,
            contact_scheduled=[1.0 if leg_is_stance.get(l, True) else 0.0 for l in _LEGS],
            force_scale=[1.0] * 4,
            phase=phase_arr,
            amp_front=amp_front,
            amp_rear=amp_rear,
            period=period,
            stance_ratio=stance_ratio,
            speed_frac=speed_frac,
            ramp_frac=ramp_frac,
            vx_cmd=vx_cmd,
            vy_cmd=vy_cmd,
            base_acc_des=np.zeros(6),
            foot_pos_actual=np.zeros(12),
            foot_pos_des=np.zeros(12),
            foot_z=np.zeros(4),
            foot_vz=np.zeros(4),
            q_err_rms=q_err_rms,
            dtau_max=dtau_max,
            mpc_ok=False,
            wbc_ok=False,
            estimate_mode="impedance",
        )

