"""WBC contact schedule / measurement overrides."""

from __future__ import annotations

from marsdog_control.control.executor_wbc_ctx import WbcTickCtx, _LEGS


class ExecutorWbcContactMixin:
    """Phase / measurement contact fusion for Soft / Walk / Spot / Jump."""

    def _wbc_update_contact(self, ctx: WbcTickCtx) -> None:
        """Update contact_snap and leg_is_stance; may refresh jump targets."""
        ctx.contact_snap = None
        if self._contact is not None:
            # Spot: trust phase schedule — measurement was keeping 3–4 feet "stance".
            # Walk: sharper LO/TD edges so short swing isn't eaten by SoftTrot blend.
            spot_now = bool(
                ctx.active_gait is not None
                and getattr(ctx.active_gait, "spot_turn_active", False)
            )
            walk_now = bool(
                ctx.active_gait is not None
                and getattr(ctx.active_gait, "family", None) == "walk"
            )
            ctx.jump_now = bool(
                ctx.active_gait is not None
                and getattr(ctx.active_gait, "family", None) == "jump"
            )
            cfg_prev = None
            if spot_now or walk_now or ctx.jump_now:
                cfg_prev = (
                    float(self._contact.cfg.measure_force_weight),
                    float(self._contact.cfg.edge_blend),
                    float(self._contact.cfg.phase_late_lo),
                    bool(self._contact.cfg.use_relative_z),
                    float(self._contact.cfg.lo_height_m),
                    float(self._contact.cfg.td_height_m),
                    int(self._contact.cfg.hold_steps),
                )
                if spot_now:
                    self._contact.cfg.measure_force_weight = 0.04
                    self._contact.cfg.edge_blend = 0.08
                    self._contact.cfg.phase_late_lo = 0.04
                elif ctx.jump_now:
                    # Soft land: wider TD window; absolute z so dig/peel don't
                    # poison relative baselines (front stuck "contact" at +1cm).
                    self._contact.cfg.measure_force_weight = 0.08
                    self._contact.cfg.edge_blend = 0.10
                    self._contact.cfg.phase_late_lo = 0.05
                    self._contact.cfg.use_relative_z = False
                    self._contact.cfg.lo_height_m = 0.008
                    self._contact.cfg.td_height_m = 0.004
                    self._contact.cfg.hold_steps = 3
                else:
                    self._contact.cfg.measure_force_weight = 0.10
                    self._contact.cfg.edge_blend = 0.06
                    self._contact.cfg.phase_late_lo = 0.03
            try:
                ctx.contact_snap = self._contact.update(
                    t_rel=ctx.t_rel,
                    gait=ctx.active_gait,
                    foot_z_world=ctx.foot_z,
                    foot_vz_world=ctx.foot_vz,
                )
            finally:
                if cfg_prev is not None:
                    (
                        self._contact.cfg.measure_force_weight,
                        self._contact.cfg.edge_blend,
                        self._contact.cfg.phase_late_lo,
                        self._contact.cfg.use_relative_z,
                        self._contact.cfg.lo_height_m,
                        self._contact.cfg.td_height_m,
                        self._contact.cfg.hold_steps,
                    ) = cfg_prev
            ctx.leg_is_stance = dict(ctx.contact_snap.stance)
            if ctx.jump_now:
                # Prefer truth vz (sim) so liftoff isn't stuck on lagged est / late note.
                vz_meas = float(getattr(ctx.state, "vel_xyz", (0.0, 0.0, 0.0))[2])
                if hasattr(ctx.active_gait, "note_base_vz"):
                    ctx.active_gait.note_base_vz(vz_meas)
                if hasattr(ctx.active_gait, "note_base_z"):
                    ctx.active_gait.note_base_z(float(ctx.current_base_z))
                if hasattr(ctx.active_gait, "_advance"):
                    ctx.active_gait._advance(ctx.t_rel)
                in_flight = bool(
                    getattr(ctx.active_gait, "in_flight", lambda: False)()
                )
                if hasattr(ctx.active_gait, "stance_ratio"):
                    ctx.active_gait.stance_ratio = 0.0 if in_flight else 1.0
                phase_u = 0.0
                if hasattr(ctx.active_gait, "_phase_u"):
                    try:
                        phase_u = float(ctx.active_gait._phase_u(ctx.t_rel))
                    except Exception:
                        phase_u = 0.0
                # Same-tick retract: motion targets were computed while still PUSH.
                if in_flight and hasattr(ctx.active_gait, "get_targets"):
                    try:
                        ctx.targets.update(ctx.active_gait.get_targets(ctx.t_rel))
                    except Exception:
                        pass
                # Snap force EMA on flight entry — residual Fz on buried soles
                # is the rear "second hop".
                if in_flight and phase_u < 0.06 and self._force_planner is not None:
                    self._force_planner._fc_filt[:] = 0.0
                jfs = float(
                    ctx.active_gait.jump_force_scale_at(ctx.t_rel)
                    if hasattr(ctx.active_gait, "jump_force_scale_at")
                    else (0.0 if in_flight else 1.0)
                )
                for leg in _LEGS:
                    if in_flight:
                        # No grace push — finish impulse in PUSH, then unload.
                        ctx.leg_is_stance[leg] = False
                        ctx.contact_snap.stance[leg] = False
                        ctx.contact_snap.force_scale[leg] = 0.0
                    else:
                        # On-ground jump: full stance.
                        ctx.leg_is_stance[leg] = True
                        ctx.contact_snap.stance[leg] = True
                        ctx.contact_snap.force_scale[leg] = 1.0
            elif spot_now:
                # Stomp FSM owns contact: plant/twist = 4-stance; catch = diagonal.
                stepper = getattr(ctx.active_gait, "_spot", None)
                for leg in _LEGS:
                    if stepper is not None and hasattr(stepper, "in_swing"):
                        ctx.leg_is_stance[leg] = not bool(stepper.in_swing(leg))
                    else:
                        ctx.leg_is_stance[leg] = bool(ctx.contact_snap.scheduled[leg])
                    ctx.contact_snap.stance[leg] = ctx.leg_is_stance[leg]
                    if not ctx.leg_is_stance[leg]:
                        ctx.contact_snap.force_scale[leg] = min(
                            float(ctx.contact_snap.force_scale.get(leg, 0.0)), 0.10
                        )


