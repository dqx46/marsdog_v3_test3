"""WBC base/foot acc, tau solve/map, dyn telemetry."""

from __future__ import annotations

import math
import numpy as np

from marsdog_control.config.joints import JOINT_BY_NAME as JBN
from marsdog_control.control.executor_wbc_ctx import WbcTickCtx, _LEGS


class ExecutorWbcApplyMixin:
    """Desired acc → WBC.compute_tau → joint FF + optional dyn log."""

    def _wbc_maybe_estimate(self, ctx: WbcTickCtx) -> None:
        """Blend base estimator into pin state when enabled."""
        if ctx.use_est and self._estimator is not None:
            # Joint rates already in v_pin; estimator solves for base linear vel
            est = self._estimator.update(
                reduced=self._reduced,
                q_pin=ctx.q_pin,
                v_pin=ctx.v_pin,
                roll=ctx.state.roll,
                pitch=ctx.state.pitch,
                yaw=ctx.state.yaw,
                gyro=(ctx.state.gyro_roll, ctx.state.gyro_pitch, ctx.state.gyro_yaw),
                leg_is_stance=ctx.leg_is_stance,
                dt=self._ctrl_dt,
                leg_phase=(ctx.contact_snap.phase if ctx.contact_snap is not None else None),
                stance_ratio=float(
                    getattr(ctx.active_gait, "stance_ratio", 1.0) if ctx.active_gait else 1.0
                ),
                edge_blend=(
                    float(self._contact.cfg.edge_blend)
                    if self._contact is not None
                    else None
                ),
                force_scale=(
                    ctx.contact_snap.force_scale if ctx.contact_snap is not None else None
                ),
            )
            ctx.vel_xyz = [est.vx, est.vy, est.vz]
            ctx.current_base_z = est.z
            ctx.q_pin, ctx.v_pin = self._assemble_pin_state(
                ctx.state, ctx.current_base_z, ctx.vel_xyz, use_estimator_xy=True
            )
            ctx.foot_z, ctx.foot_vz, ctx.foot_pos, ctx.foot_vel = self._foot_kinematics(ctx.q_pin, ctx.v_pin)

    def _wbc_fill_velocity_cmds(self, ctx: WbcTickCtx) -> None:
        """Resolve vx/vy/wz/vz and target z/rpy from gait."""
        ctx.target_z = ctx.active_gait.body_height if ctx.active_gait else 0.24
        if ctx.active_gait is not None and hasattr(ctx.active_gait, "get_target_z"):
            try:
                ctx.target_z = float(ctx.active_gait.get_target_z(ctx.t_rel))
            except TypeError:
                ctx.target_z = float(ctx.active_gait.get_target_z())
        ctx.target_roll = 0.0
        ctx.target_pitch = 0.0

        ctx.vx_cmd = 0.0
        ctx.vy_cmd = 0.0
        ctx.wz_cmd = 0.0
        ctx.vz_cmd = 0.0
        ctx.jump_now = bool(
            ctx.active_gait is not None
            and getattr(ctx.active_gait, "family", None) == "jump"
        )
        if ctx.active_gait:
            if hasattr(ctx.active_gait, "vel_cmd"):
                ctx.vx_cmd = ctx.active_gait.vel_cmd[0]
                ctx.vy_cmd = ctx.active_gait.vel_cmd[1]
                if len(ctx.active_gait.vel_cmd) > 2:
                    ctx.wz_cmd = float(ctx.active_gait.vel_cmd[2])
            elif hasattr(ctx.active_gait, "amp_front") and hasattr(ctx.active_gait, "period"):
                avg_amp = (
                    ctx.active_gait.amp_front
                    + getattr(ctx.active_gait, "amp_rear", ctx.active_gait.amp_front)
                ) / 2.0
                ctx.vx_cmd = (avg_amp * 2.0) / ctx.active_gait.period
            if ctx.jump_now and hasattr(ctx.active_gait, "desired_vz"):
                ctx.vz_cmd = float(ctx.active_gait.desired_vz(ctx.t_rel))
            if ctx.jump_now:
                # Prefer MuJoCo/IMU truth over estimator — est under-reads liftoff vz.
                vz_truth = float(getattr(ctx.state, "vel_xyz", (0.0, 0.0, 0.0))[2])
                vz_note = (
                    vz_truth
                    if abs(vz_truth) >= abs(float(ctx.v_pin[2]))
                    else float(ctx.v_pin[2])
                )
                if hasattr(ctx.active_gait, "note_base_vz"):
                    ctx.active_gait.note_base_vz(vz_note)
                if hasattr(ctx.active_gait, "note_base_z"):
                    ctx.active_gait.note_base_z(float(ctx.current_base_z))
                ctx.vx_cmd = 0.0
                ctx.vy_cmd = 0.0
                ctx.wz_cmd = 0.0


    def _wbc_compute_base_and_foot_acc(self, ctx: WbcTickCtx) -> None:
        """Base PD + swing/stance Cartesian acc."""
        cfg = self.config
        # Lateral velocity: light EMA then damp (cuts estimator noise, keeps authority).
        # Do NOT blend MuJoCo truth here — real has no truth; mixing it made sim
        # look stabler than the estimator-only path used on hardware.
        a_vy = 0.25 if ctx.use_est else 0.45
        self._vy_filt = (1.0 - a_vy) * getattr(self, "_vy_filt", 0.0) + a_vy * ctx.vel_xyz[1]
        ctx.vy_for_damp = self._vy_filt if ctx.use_est else ctx.vel_xyz[1]

        ctx.base_acc_des = np.zeros(6)
        # Dog-trot plant undershoots kinematic vx while stance-LS overestimates
        # (front swing drag / slip). Never treat est>cmd as overspeed — that
        # commanded persistent braking and stalled after ~2 strides.
        vx_meas = float(ctx.vel_xyz[0])
        if ctx.vx_cmd > 0.05:
            ctx.vx_for_track = min(vx_meas, ctx.vx_cmd)
        elif ctx.vx_cmd < -0.05:
            ctx.vx_for_track = max(vx_meas, ctx.vx_cmd)
        else:
            ctx.vx_for_track = vx_meas
        vx_err = ctx.vx_cmd - ctx.vx_for_track
        # Push when slow; zero ax when est already at/above cmd (no fake brake).
        ax_gain = 3.5 if vx_err * ctx.vx_cmd > 1e-6 else 0.0
        ctx.spot = bool(getattr(ctx.active_gait, "spot_turn_active", False))
        # Prefer larger |v| between estimator and IMU/truth — estimator
        # under-reads soft-contact scrub (~0.02 vs truth ~0.11 while standing).
        vx_raw = float(getattr(ctx.state, "vel_xyz", (vx_meas, 0.0, 0.0))[0])
        vy_raw = float(getattr(ctx.state, "vel_xyz", (0.0, 0.0, 0.0))[1])
        ctx.vx_brake = vx_raw if abs(vx_raw) > abs(vx_meas) else vx_meas
        ctx.vy_brake = vy_raw if abs(vy_raw) > abs(float(ctx.vel_xyz[1])) else float(ctx.vel_xyz[1])
        ctx.hold_still = (not ctx.spot) and abs(float(ctx.vx_cmd)) <= 0.05
        if ctx.spot:
            # Kill forward creep hard; estimator under-reads scrubbing vx.
            # Active back-bias cancels residual forward scrub.
            ctx.base_acc_des[0] = -36.0 * (ctx.vx_brake + 0.03) - 6.0 * np.sign(ctx.vx_brake + 0.015)
            ctx.base_acc_des[1] = -18.0 * ctx.vy_brake - 4.0 * np.sign(ctx.vy_brake)
            # Pull CoM into the support triangle (SpotYawStepper), on top of brake.
            if hasattr(ctx.active_gait, "get_spot_com_shift"):
                sx, sy = ctx.active_gait.get_spot_com_shift(ctx.t_rel)
                ctx.base_acc_des[0] += 30.0 * float(sx)
                ctx.base_acc_des[1] += 40.0 * float(sy)
            ctx.vx_cmd = -0.03
            ctx.vy_cmd = 0.0
        elif ctx.hold_still:
            # Stand / jump hold / zero-cmd: lateral had damp, longitudinal did not
            # → constant forward scrape on soft soles. Same authority as Spot.
            # Launch phases: don't fight vertical impulse with hard XY brake.
            jump_launch = ctx.jump_now and getattr(
                getattr(ctx.active_gait, "phase", None), "value", ""
            ) in ("crouch", "push", "flight")
            if jump_launch:
                ctx.base_acc_des[0] = -8.0 * ctx.vx_brake
                ctx.base_acc_des[1] = -float(cfg.lateral_vel_damp) * ctx.vy_for_damp
            else:
                ctx.base_acc_des[0] = (
                    -36.0 * (ctx.vx_brake + 0.02) - 6.0 * np.sign(ctx.vx_brake + 0.01)
                )
                ctx.base_acc_des[1] = (
                    -float(cfg.lateral_vel_damp) * ctx.vy_for_damp
                    - 3.0 * np.sign(ctx.vy_brake)
                )
        else:
            ctx.base_acc_des[0] = ax_gain * vx_err
            ctx.base_acc_des[1] = -float(cfg.lateral_vel_damp) * ctx.vy_for_damp
        # Jump: track z_ref + desired_vz; Soft/Walk keep original damping on vz.
        if ctx.jump_now:
            # Jump-only Z gains from JumpController (recipe JUMP_*), never mutate
            # global DynamicsConfig so SoftTrot keeps schema/CLI kp_base_z.
            kp_z = float(getattr(ctx.active_gait, "kp_base_z", cfg.kp_base_z))
            kd_z = float(getattr(ctx.active_gait, "kd_base_z", cfg.kd_base_z))
            # During jump, position tracking can fight velocity tracking if the robot
            # didn't crouch deep enough. We prioritize velocity tracking.
            z_err = ctx.target_z - ctx.current_base_z
            phase_name = getattr(
                getattr(ctx.active_gait, "phase", None), "value", ""
            )
            if phase_name == "push":
                z_err = max(0.0, z_err)  # Never push down during PUSH
                vz_truth = float(getattr(ctx.state, "vel_xyz", (0.0, 0.0, 0.0))[2])
                vz_use = (
                    vz_truth if abs(vz_truth) > abs(float(ctx.v_pin[2])) else float(ctx.v_pin[2])
                )
                # Strong upward vz track; never brake a rising hop (cmd lag → dig).
                vz_term = -2.0 * kd_z * (vz_use - ctx.vz_cmd)
                vz_term = max(0.0, float(vz_term))
                ctx.base_acc_des[2] = 0.20 * kp_z * z_err + vz_term
            elif phase_name in ("land", "recover"):
                # Soft absorb: never bounce by over-braking a downward vz.
                stand_h = float(getattr(ctx.active_gait, "stand_height", 0.24))
                z_to_stand = stand_h - ctx.current_base_z
                vz = float(ctx.v_pin[2])
                if z_to_stand > 0.0:
                    # Below stand — rise gently; don't fight the fall hard.
                    ctx.base_acc_des[2] = (
                        0.20 * kp_z * z_to_stand
                        - 0.40 * kd_z * vz
                    )
                else:
                    # Above stand — pull down / brake upward bounce.
                    ctx.base_acc_des[2] = (
                        0.90 * kp_z * z_to_stand
                        - 2.5 * kd_z * max(0.0, vz)
                    )
            else:
                ctx.base_acc_des[2] = (
                    kp_z * z_err
                    - kd_z * (ctx.v_pin[2] - ctx.vz_cmd)
                )
            # Kill nose-up so front doesn't peel/plant while rear clears.
            # Slight nose-down bias during push loads the rear feet.
            phase_name = getattr(
                getattr(ctx.active_gait, "phase", None), "value", ""
            )
            if phase_name == "push":
                pitch_des = -0.02
                pitch_kp = 5.5
                pitch_kd = 3.0
            elif phase_name == "flight":
                # Nose-down bias unloads front so all four can clear together.
                pitch_des = -0.04
                pitch_kp = 6.5
                pitch_kd = 3.5
            else:
                pitch_des = 0.0
                pitch_kp = 3.0
                pitch_kd = 2.0
            ctx.base_acc_des[4] = (
                pitch_kp * cfg.kp_base_pitch * (pitch_des - ctx.state.pitch)
                - pitch_kd * cfg.kd_base_pitch * ctx.state.gyro_pitch
            )
        else:
            ctx.base_acc_des[2] = (
                cfg.kp_base_z * (ctx.target_z - ctx.current_base_z)
                - cfg.kd_base_z * ctx.v_pin[2]
            )
            ctx.base_acc_des[4] = (
                cfg.kp_base_pitch * (ctx.target_pitch - ctx.state.pitch)
                - cfg.kd_base_pitch * ctx.state.gyro_pitch
            )
        ctx.base_acc_des[3] = (
            cfg.kp_base_roll * (ctx.target_roll - ctx.state.roll)
            - cfg.kd_base_roll * ctx.state.gyro_roll
        )
        # Spot: mild yaw_des track + wz — feet scrub does most of the turn.
        if ctx.spot and abs(ctx.wz_cmd) > 0.05:
            stepper = getattr(ctx.active_gait, "_spot", None)
            yaw_err = float(stepper.yaw_error()) if stepper is not None else 0.0
            yaw_err = max(-0.35, min(0.35, yaw_err))
            ctx.base_acc_des[5] = (
                8.0 * yaw_err
                - 4.0 * ctx.state.gyro_yaw
                + 3.0 * (ctx.wz_cmd - ctx.state.gyro_yaw)
            )
            self._spot_yaw_debt = yaw_err
        else:
            self._spot_yaw_debt = 0.0
            if abs(ctx.wz_cmd) > 0.05:
                ctx.base_acc_des[5] = 6.0 * (ctx.wz_cmd - ctx.state.gyro_yaw) - 1.5 * ctx.state.gyro_yaw
            else:
                ctx.base_acc_des[5] = -4.0 * ctx.state.gyro_yaw

        # Jump flight swing boost must be set BEFORE _swing_acc_des.
        self._jump_flight_swing_boost = bool(
            ctx.jump_now
            and getattr(getattr(ctx.active_gait, "phase", None), "value", "") == "flight"
        )

        ctx.swing_acc, ctx.foot_pos_des = self._swing_acc_des(
            ctx.state,
            ctx.targets,
            ctx.current_base_z,
            ctx.vel_xyz,
            ctx.use_est,
            ctx.leg_is_stance,
            ctx.foot_pos,
            ctx.foot_vel,
        )
        # Standstill: kill foot world-XY velocity. Stance a=0 would keep a
        # constant scrub once soft contact starts sliding.
        # Use scrub-aware base rates (estimator under-reads ~5× while standing).
        ctx.stance_acc = {leg: np.zeros(3) for leg in _LEGS}
        jump_phase = (
            getattr(getattr(ctx.active_gait, "phase", None), "value", "")
            if ctx.jump_now else ""
        )
        # During crouch/push/flight: don't burn friction cone / QP on XY scrub brake.
        ctx.scrub_brake = ctx.hold_still and jump_phase not in ("crouch", "push", "flight")
        if ctx.scrub_brake:
            ctx.v_pin[0] = float(ctx.vx_brake)
            ctx.v_pin[1] = float(ctx.vy_brake)
            _, _, _, ctx.foot_vel = self._foot_kinematics(ctx.q_pin, ctx.v_pin)
            kd_foot = 40.0
            for leg in _LEGS:
                if not ctx.leg_is_stance.get(leg, True):
                    continue
                v_f = np.asarray(ctx.foot_vel[leg], dtype=float).reshape(3)
                # Fallback to base scrub if FK still under-reads.
                vx_f = float(v_f[0]) if abs(float(v_f[0])) > 0.02 else float(ctx.vx_brake)
                vy_f = float(v_f[1]) if abs(float(v_f[1])) > 0.02 else float(ctx.vy_brake)
                ctx.stance_acc[leg] = np.array(
                    [-kd_foot * vx_f, -kd_foot * vy_f, 0.0]
                )
        elif ctx.hold_still:
            # Still feed truth rates into dynamics during launch.
            ctx.v_pin[0] = float(ctx.vx_brake)
            ctx.v_pin[1] = float(ctx.vy_brake)

    def _wbc_solve_map_and_log(self, ctx: WbcTickCtx) -> dict[int, float]:
        """Weight-scoped WBC QP, gravity blend, dyn telemetry."""
        # Spot: raise yaw/XY hold + swing tracking; soften stance lock.
        wbc_w_prev = None
        wbc_st_prev = None
        wbc_sw_prev = None
        if (ctx.spot or ctx.jump_now or ctx.hold_still) and self._wbc_ctrl is not None:
            wbc_w_prev = np.array(self._wbc_ctrl.config.weight_base_acc, copy=True)
            wbc_st_prev = float(self._wbc_ctrl.config.weight_stance_acc)
            wbc_sw_prev = float(self._wbc_ctrl.config.weight_swing_acc)
            w = np.array(wbc_w_prev, copy=True)
            wbc_ft_prev = None
            if ctx.spot:
                w[0, 0] = 45.0   # ax — kill creep
                w[1, 1] = 45.0   # ay
                w[3, 3] = max(float(w[3, 3]), 70.0)  # keep roll while yawing
                w[5, 5] = 120.0  # ayaw assist (CoM+3-foot do the plant)
                self._wbc_ctrl.config.weight_stance_acc = 8.0
                self._wbc_ctrl.config.weight_swing_acc = max(wbc_sw_prev, 18.0)
            elif ctx.hold_still:
                w[0, 0] = max(float(w[0, 0]), 55.0)  # ax — standstill brake
                w[1, 1] = max(float(w[1, 1]), 40.0)
                self._wbc_ctrl.config.weight_stance_acc = max(wbc_st_prev, 40.0)
                wbc_ft_prev = float(self._wbc_ctrl.config.weight_force_tracking)
                self._wbc_ctrl.config.weight_force_tracking = max(wbc_ft_prev, 12.0)
            if ctx.jump_now:
                phase_name = getattr(
                    getattr(ctx.active_gait, "phase", None), "value", ""
                )
                if phase_name == "push":
                    w[0, 0] = 8.0    # don't steal friction from Fz
                    w[1, 1] = 8.0
                    w[2, 2] = 90.0   # vertical launch
                else:
                    w[0, 0] = max(float(w[0, 0]), 55.0)  # hold XY during hop
                    w[2, 2] = 55.0  # Strongly track Z acceleration for jump
                w[4, 4] = max(float(w[4, 4]), 90.0)  # hold pitch — stop nose-up peel
                wbc_fmax_prev = float(self._wbc_ctrl.config.f_max)
                if phase_name == "push":
                    self._wbc_ctrl.config.f_max = max(wbc_fmax_prev, 140.0)
                else:
                    wbc_fmax_prev = None
                if phase_name == "flight":
                    self._wbc_ctrl.config.weight_stance_acc = 1.0
                    self._wbc_ctrl.config.weight_swing_acc = max(wbc_sw_prev, 28.0)
                elif phase_name == "push":
                    # Softer stance lock so feet can leave while Fz is high.
                    self._wbc_ctrl.config.weight_stance_acc = min(
                        max(wbc_st_prev, 12.0), 18.0
                    )
                else:
                    # Keep stance foot-scrub brake strong on ground.
                    self._wbc_ctrl.config.weight_stance_acc = max(wbc_st_prev, 35.0)
                if wbc_ft_prev is None:
                    wbc_ft_prev = float(self._wbc_ctrl.config.weight_force_tracking)
                self._wbc_ctrl.config.weight_force_tracking = max(
                    wbc_ft_prev, 28.0 if phase_name == "push" else 20.0
                )
            else:
                wbc_fmax_prev = None
                if not ctx.hold_still:
                    wbc_ft_prev = None
            self._wbc_ctrl.config.weight_base_acc = w
        else:
            wbc_ft_prev = None
            wbc_fmax_prev = None
        try:
            tau_opt = self._wbc_ctrl.compute_tau(
                q_pin=ctx.q_pin,
                v_pin=ctx.v_pin,
                base_acc_des=ctx.base_acc_des,
                leg_is_stance=ctx.leg_is_stance,
                f_c_des=ctx.f_c_des,
                swing_acc_des=ctx.swing_acc,
                force_scale=ctx.force_scale,
                stance_acc_des=ctx.stance_acc,
            )
        finally:
            if wbc_w_prev is not None:
                self._wbc_ctrl.config.weight_base_acc = wbc_w_prev
                self._wbc_ctrl.config.weight_stance_acc = wbc_st_prev
            if wbc_sw_prev is not None:
                self._wbc_ctrl.config.weight_swing_acc = wbc_sw_prev
            if wbc_ft_prev is not None:
                self._wbc_ctrl.config.weight_force_tracking = wbc_ft_prev
            if wbc_fmax_prev is not None:
                self._wbc_ctrl.config.f_max = wbc_fmax_prev

        # ForceMode.WBC owns τ_ff — no ImpedanceAssist gravity blend under WBC.
        # Jump PUSH: amplify τ_ff so Jc^T F is not drowned by residual PD.
        jump_tau_scale = 1.0
        if ctx.jump_now:
            phase_name = getattr(
                getattr(ctx.active_gait, "phase", None), "value", ""
            )
            if phase_name == "push":
                jump_tau_scale = 3.0
            elif phase_name == "crouch":
                jump_tau_scale = 1.5
            elif phase_name in ("land", "recover"):
                jump_tau_scale = 0.7  # don't smash back up to stand
        out_trq = {}
        for i, jname_joint in enumerate(self._wbc_ctrl.actuated_joint_names):
            jname = jname_joint.replace("_joint", "")
            if jname not in JBN:
                continue
            mid = JBN[jname].motor_id
            if mid not in ctx.targets:
                continue
            out_trq[mid] = float(tau_opt[i]) * jump_tau_scale

        foot_pos_flat = np.zeros(12)
        foot_des_flat = np.zeros(12)
        foot_z_arr = np.zeros(4)
        foot_vz_arr = np.zeros(4)
        for i, leg in enumerate(_LEGS):
            foot_pos_flat[i * 3 : i * 3 + 3] = ctx.foot_pos[leg]
            foot_des_flat[i * 3 : i * 3 + 3] = ctx.foot_pos_des[leg]
            foot_z_arr[i] = float(ctx.foot_z[leg])
            foot_vz_arr[i] = float(ctx.foot_vz[leg])

        if self._dyn_tel is not None:
            truth = getattr(ctx.state, "vel_xyz", (0.0, 0.0, 0.0))
            fs = ctx.force_scale if ctx.force_scale is not None else {leg: 1.0 for leg in _LEGS}
            force_scale_arr = [float(fs.get(leg, 1.0)) for leg in _LEGS]

            phase_arr = [0.0, 0.0, 0.0, 0.0]
            amp_front = 0.0
            amp_rear = 0.0
            period = 0.0
            stance_ratio = 0.0
            speed_frac = 0.0
            ramp_frac = 1.0
            if ctx.active_gait is not None:
                period = float(getattr(ctx.active_gait, "period", 0.0) or 0.0)
                stance_ratio = float(getattr(ctx.active_gait, "stance_ratio", 0.0) or 0.0)
                amp_front = float(getattr(ctx.active_gait, "amp_front", 0.0) or 0.0)
                amp_rear = float(getattr(ctx.active_gait, "amp_rear", amp_front) or 0.0)
                speed_frac = float(getattr(ctx.active_gait, "speed_frac", 0.0) or 0.0)
                offsets = getattr(ctx.active_gait, "_PHASE_OFFSET", None)
                if period > 1e-6 and isinstance(offsets, dict):
                    for i, leg in enumerate(_LEGS):
                        phase_arr[i] = (ctx.t_rel / period + float(offsets.get(leg, 0.0))) % 1.0
                ramp_dur = float(getattr(ctx.active_gait, "ramp_duration", 0.0) or 0.0)
                if ramp_dur > 1e-6 and ctx.t_rel < ramp_dur:
                    s = ctx.t_rel / ramp_dur
                    ramp_frac = s * s * (3.0 - 2.0 * s)
                else:
                    ramp_frac = 1.0

            # Joint tracking RMS (targets vs measured, actuated WBC joints)
            q_err_sq = 0.0
            q_err_n = 0
            for jname_joint in self._wbc_ctrl.actuated_joint_names:
                jname = jname_joint.replace("_joint", "")
                if jname not in JBN:
                    continue
                mid = JBN[jname].motor_id
                if mid not in ctx.targets:
                    continue
                q_act = float(ctx.state.joint_pos.get(mid, ctx.targets[mid]))
                dq = float(ctx.targets[mid]) - q_act
                q_err_sq += dq * dq
                q_err_n += 1
            q_err_rms = math.sqrt(q_err_sq / q_err_n) if q_err_n else 0.0

            prev_tau = getattr(self, "_tel_prev_tau", None)
            if prev_tau is not None and prev_tau.shape == tau_opt.shape:
                dtau_max = float(np.max(np.abs(tau_opt - prev_tau)))
            else:
                dtau_max = 0.0
            self._tel_prev_tau = tau_opt.copy()

            self._dyn_tel.record(
                t=ctx.t_rel,
                roll=ctx.state.roll,
                pitch=ctx.state.pitch,
                z=ctx.current_base_z,
                vx=ctx.vel_xyz[0],
                vy=ctx.vel_xyz[1],
                wz=ctx.v_pin[5],
                vx_truth=float(truth[0]),
                vy_truth=float(truth[1]),
                vz_truth=float(truth[2]),
                fc_des=ctx.f_c_des.copy() if ctx.f_c_des is not None else np.zeros(12),
                tau_opt=tau_opt.copy(),
                contact_state=[1.0 if ctx.leg_is_stance.get(l, True) else 0.0 for l in _LEGS],
                contact_measured=(
                    [1.0 if ctx.contact_snap.measured[l] else 0.0 for l in _LEGS]
                    if ctx.contact_snap
                    else [0.0] * 4
                ),
                contact_scheduled=(
                    [1.0 if ctx.contact_snap.scheduled[l] else 0.0 for l in _LEGS]
                    if ctx.contact_snap
                    else [0.0] * 4
                ),
                force_scale=force_scale_arr,
                phase=phase_arr,
                amp_front=amp_front,
                amp_rear=amp_rear,
                period=period,
                stance_ratio=stance_ratio,
                speed_frac=speed_frac,
                ramp_frac=ramp_frac,
                vx_cmd=ctx.vx_cmd,
                vy_cmd=ctx.vy_cmd,
                base_acc_des=ctx.base_acc_des.copy(),
                foot_pos_actual=foot_pos_flat,
                foot_pos_des=foot_des_flat,
                foot_z=foot_z_arr,
                foot_vz=foot_vz_arr,
                q_err_rms=q_err_rms,
                dtau_max=dtau_max,
                mpc_ok=bool(self._force_planner.last_ok) if self._force_planner else False,
                wbc_ok=bool(self._wbc_ctrl.last_solve_ok),
                estimate_mode=self.config.base_estimate_mode,
            )

        return out_trq

