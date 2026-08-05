"""ForcePlanner MPC (contact forces) for WBC path."""

from __future__ import annotations

import numpy as np

from marsdog_control.control.executor_wbc_ctx import WbcTickCtx, _LEGS


class ExecutorWbcQpMixin:
    """Horizon contact + ForcePlanner.plan → f_c_des."""

    def _wbc_plan_forces(self, ctx: WbcTickCtx) -> None:
        """Run ForcePlanner MPC; sets ctx.f_c_des / ctx.force_scale."""
        import pinocchio as pin

        cfg = self.config
        ctx.f_c_des = None
        if self._force_planner is not None:
            x0 = np.zeros(13)
            x0[0] = ctx.state.roll
            x0[1] = ctx.state.pitch
            x0[2] = ctx.state.yaw
            x0[5] = ctx.current_base_z
            x0[6] = ctx.state.gyro_roll
            x0[7] = ctx.state.gyro_pitch
            x0[8] = ctx.state.gyro_yaw
            # Spot zeros XY rates. Hold-still feeds scrub-aware vx so MPC brakes.
            if ctx.spot:
                x0[9] = 0.0
                x0[10] = 0.0
            elif ctx.hold_still:
                x0[9] = float(ctx.vx_brake)
                x0[10] = float(
                    ctx.vy_brake if abs(ctx.vy_brake) > abs(float(ctx.vel_xyz[1])) else ctx.vel_xyz[1]
                )
            else:
                x0[9] = ctx.vx_for_track
                x0[10] = ctx.vel_xyz[1]
            x0[11] = float(ctx.vz_cmd) if ctx.jump_now else ctx.vel_xyz[2]
            x0[12] = -9.81

            H = self._force_planner.mpc.cfg.horizon
            dt_mpc = self._force_planner.mpc.cfg.dt
            # Cruise: diagonal CoM kick. Spot: CoM-into-support-triangle from
            # SpotYawStepper (decoupled; zero if gait has no get_spot_com_shift).
            st = ctx.leg_is_stance
            diag = (
                (1.0 if st.get("fl", True) else 0.0)
                + (1.0 if st.get("rr", True) else 0.0)
                - (1.0 if st.get("fr", True) else 0.0)
                - (1.0 if st.get("rl", True) else 0.0)
            )
            x_shift = 0.0
            planner = getattr(self, "_lateral_planner", None)
            if planner is None:
                planner = getattr(ctx.active_gait, "_lateral_planner", None)
            if planner is not None and ctx.active_gait is not None:
                planner.sync_from_gait(ctx.active_gait)

            if ctx.spot and hasattr(ctx.active_gait, "get_spot_com_shift"):
                sx, sy = ctx.active_gait.get_spot_com_shift(ctx.t_rel)
                x_shift = float(sx)
                y_shift = float(sy)
            elif (
                not ctx.spot
                and getattr(ctx.active_gait, "family", None) == "walk"
                and hasattr(ctx.active_gait, "get_com_y_shift")
            ):
                # Four-beat Walk: phase-locked CoM sway — LateralOwner.WALK_COM.
                y_shift = float(ctx.active_gait.get_com_y_shift(ctx.t_rel))
            else:
                raw_y = 0.0 if ctx.spot else float(cfg.com_y_shift_m) * 0.5 * diag
                # Fail-closed without planner (no ungated Dynamics force_y).
                y_shift = (
                    float(planner.gate_force_y(raw_y)) if planner is not None
                    else 0.0
                )

            x_ref = np.zeros(13 * H)
            wz_ref = float(ctx.wz_cmd)
            yaw0 = (
                float(getattr(getattr(ctx.active_gait, "_spot", None), "yaw_des", ctx.state.yaw))
                if ctx.spot else ctx.state.yaw
            )
            walk_com = (
                not ctx.spot
                and getattr(ctx.active_gait, "family", None) == "walk"
                and hasattr(ctx.active_gait, "get_com_y_shift")
            )
            for k in range(H):
                dt_k = k * dt_mpc
                x_ref[k * 13 + 0] = ctx.target_roll
                x_ref[k * 13 + 1] = ctx.target_pitch
                x_ref[k * 13 + 2] = yaw0 + wz_ref * dt_k
                x_ref[k * 13 + 8] = wz_ref
                x_ref[k * 13 + 3] = ctx.vx_cmd * dt_k + x_shift
                y_k = (
                    float(ctx.active_gait.get_com_y_shift(ctx.t_rel + dt_k))
                    if walk_com else y_shift
                )
                x_ref[k * 13 + 4] = ctx.vy_cmd * dt_k + y_k
                x_ref[k * 13 + 5] = (ctx.current_base_z if ctx.jump_now else ctx.target_z) + (ctx.vz_cmd * dt_k if ctx.jump_now else 0.0)
                x_ref[k * 13 + 9] = ctx.vx_cmd
                x_ref[k * 13 + 10] = ctx.vy_cmd
                x_ref[k * 13 + 11] = float(ctx.vz_cmd) if ctx.jump_now else 0.0
                x_ref[k * 13 + 12] = -9.81

            r_feet = np.zeros((3, 4))
            com = self._wbc_ctrl.data.com[0]
            pin.centerOfMass(self._wbc_ctrl.model, self._wbc_ctrl.data, ctx.q_pin, ctx.v_pin)
            com = self._wbc_ctrl.data.com[0]
            for i, foot_name in enumerate(self._wbc_ctrl.foot_names):
                foot_id = self._wbc_ctrl.foot_ids[i]
                r_feet[:, i] = self._wbc_ctrl.data.oMf[foot_id].translation - com

            contact_h = self._contact.horizon(
                t_rel=ctx.t_rel,
                gait=ctx.active_gait,
                horizon=H,
                dt=dt_mpc,
                measured=ctx.leg_is_stance,
            )
            # Spot: MPC horizon follows unitree diagonal duty (via SpotYawStepper).
            if ctx.spot:
                stepper = getattr(ctx.active_gait, "_spot", None)
                per = float(getattr(ctx.active_gait, "period", 1.0) or 1.0)
                if stepper is not None and hasattr(stepper, "predict_force_scale"):
                    contact_h = np.zeros(4 * H, dtype=float)
                    for k in range(H):
                        t_k = ctx.t_rel + k * dt_mpc
                        for li, leg in enumerate(_LEGS):
                            contact_h[k * 4 + li] = float(
                                stepper.predict_force_scale(leg, t_k, per)
                            )
            elif ctx.jump_now:
                # Predict jump force scale across horizon.
                in_flight = bool(
                    getattr(ctx.active_gait, "in_flight", lambda: False)()
                )
                phase_u = 0.0
                if hasattr(ctx.active_gait, "_phase_u"):
                    try:
                        phase_u = float(ctx.active_gait._phase_u(ctx.t_rel))
                    except Exception:
                        phase_u = 0.0
                still_any = bool(
                    ctx.contact_snap is not None
                    and any(bool(ctx.contact_snap.measured.get(leg, False)) for leg in _LEGS)
                )
                vz_meas = float(getattr(ctx.state, "vel_xyz", (0.0, 0.0, 0.0))[2])
                # Early flight + slow + still planted: tiny horizon hold only.
                hold_push = (
                    in_flight and still_any and phase_u < 0.10 and vz_meas < 0.40
                )
                contact_h = np.zeros(4 * H, dtype=float)
                for k in range(H):
                    t_k = ctx.t_rel + k * dt_mpc
                    jfs = float(
                        ctx.active_gait.predict_jump_force_scale(t_k)
                        if hasattr(ctx.active_gait, "predict_jump_force_scale")
                        else (0.0 if in_flight else 1.0)
                    )
                    if hold_push:
                        jfs = max(jfs, 0.7)
                    for li in range(4):
                        contact_h[k * 4 + li] = jfs
            ctx.force_scale = (
                ctx.contact_snap.force_scale
                if ctx.contact_snap is not None
                else {leg: 1.0 for leg in _LEGS}
            )
            # Spot: open-dog yaw is the task — default Q barely tracks yaw (4)
            # vs roll (90), so ayaw alone scrubs. Boost yaw/wz, pin XY.
            q_prev = None
            lpf_prev = None
            df_prev = None
            if ctx.spot:
                q_prev = np.array(self._force_planner.mpc.cfg.weights_Q, copy=True)
                q = np.array(q_prev, copy=True)
                q[2] = 80.0   # yaw
                q[3] = 40.0   # x hold
                q[4] = 40.0   # y hold
                q[8] = 40.0   # wz
                q[9] = 60.0   # vx → 0
                q[10] = 60.0  # vy → 0
                self._force_planner.mpc.cfg.weights_Q = q
            elif ctx.hold_still:
                # Stand / zero-cmd: pin vx so soft soles don't scrape forward.
                q_prev = np.array(self._force_planner.mpc.cfg.weights_Q, copy=True)
                q = np.array(q_prev, copy=True)
                q[3] = max(float(q[3]), 30.0)   # x hold
                q[9] = max(float(q[9]), 55.0)   # vx → 0
                q[10] = max(float(q[10]), 40.0)  # vy → 0
                self._force_planner.mpc.cfg.weights_Q = q
            if ctx.jump_now:
                # Fast force ramp — default 700N/s is too slow for a short push.
                lpf_prev = float(self._force_planner.force_lpf_alpha)
                df_prev = float(self._force_planner.max_df_dt)
                self._force_planner.force_lpf_alpha = max(lpf_prev, 0.35)
                self._force_planner.max_df_dt = max(df_prev, 2800.0)
                if q_prev is None:
                    q_prev = np.array(self._force_planner.mpc.cfg.weights_Q, copy=True)
                q = np.array(self._force_planner.mpc.cfg.weights_Q, copy=True)
                q[5] = max(float(q[5]), 80.0)   # z
                q[11] = max(float(q[11]), 50.0)  # vz
                phase_q = getattr(
                    getattr(ctx.active_gait, "phase", None), "value", ""
                )
                if phase_q == "push":
                    q[11] = max(float(q[11]), 80.0)  # chase push_vz
                    q[9] = min(float(q[9]), 15.0)    # don't brake launch
                else:
                    q[9] = max(float(q[9]), 55.0)    # kill forward creep
                self._force_planner.mpc.cfg.weights_Q = q
            try:
                ctx.f_c_des = self._force_planner.plan(
                    x0=x0,
                    x_ref=x_ref,
                    r_feet=r_feet,
                    contact_horizon=contact_h,
                    force_scale=ctx.force_scale,
                    dt=0.005,  # fixed control period; t_rel can reset at gait start
                )
                # Jump PUSH: amplify vertical impulse; soft-land scales Fz only.
                if ctx.jump_now:
                    phase_name = getattr(
                        getattr(ctx.active_gait, "phase", None), "value", ""
                    )
                    jfs = float(
                        ctx.active_gait.jump_force_scale_at(ctx.t_rel)
                        if hasattr(ctx.active_gait, "jump_force_scale_at")
                        else 1.0
                    )
                    if phase_name in ("land", "recover") and jfs < 0.99:
                        for li in range(4):
                            ctx.f_c_des[li * 3 + 2] *= max(0.15, jfs)
                    if phase_name == "push":
                        # ~2× body weight; keep front/rear balanced (rear bias → 二弹).
                        f_cap = 60.0
                        for li, leg in enumerate(_LEGS):
                            s = 1.06 if leg in ("rl", "rr") else 1.02
                            fz = max(float(ctx.f_c_des[li * 3 + 2]) * s * 1.80, 32.0 * s)
                            ctx.f_c_des[li * 3 + 2] = min(f_cap * s, fz)
                        zmin = min(float(ctx.foot_z.get(leg, 0.0)) for leg in _LEGS)
                        if zmin < -0.030:
                            dig = min(1.0, (-0.030 - zmin) / 0.020)
                            fz_s = max(0.80, 1.0 - 0.20 * dig)
                            for li in range(4):
                                ctx.f_c_des[li * 3 + 2] *= fz_s
                    elif phase_name == "crouch":
                        for li, leg in enumerate(_LEGS):
                            s = 1.05 if leg in ("rl", "rr") else 0.97
                            ctx.f_c_des[li * 3 + 2] *= s
                # Hold-still XY brake — skip during launch (needs friction for Fz).
                if ctx.scrub_brake:
                    n_st = sum(
                        1 for leg in _LEGS if ctx.leg_is_stance.get(leg, True)
                    ) or 4
                    fx_tot = -80.0 * float(ctx.vx_brake) - 10.0 * np.sign(
                        float(ctx.vx_brake) + 1e-6
                    )
                    fy_tot = -40.0 * float(ctx.vy_brake)
                    share = 1.0 / float(n_st)
                    for li, leg in enumerate(_LEGS):
                        if not ctx.leg_is_stance.get(leg, True):
                            continue
                        ctx.f_c_des[li * 3 + 0] += fx_tot * share
                        ctx.f_c_des[li * 3 + 1] += fy_tot * share
            finally:
                if q_prev is not None:
                    self._force_planner.mpc.cfg.weights_Q = q_prev
                if lpf_prev is not None:
                    self._force_planner.force_lpf_alpha = lpf_prev
                if df_prev is not None:
                    self._force_planner.max_df_dt = df_prev
        else:
            ctx.force_scale = (
                ctx.contact_snap.force_scale
                if ctx.contact_snap is not None
                else {leg: 1.0 for leg in _LEGS}
            )

