"""集中管理站姿配置和步态预设。

目标是把 sim_walk.py / walk.py / 调试工具里散落的关键默认值收敛到这里。
这里不包含控制循环，也不直接依赖 MuJoCo，方便实机和仿真共同导入。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Callable, Optional, Union

from marsdog_control.config.jump_recipe import (
    JUMP_RECIPE,
    JUMP_RECIPE_WBC,
    jump_recipe_dict,
)
from marsdog_control.config.soft_trot_recipe import (
    SOFT_TROT_RECIPE,
    soft_trot_recipe_dict,
)
from marsdog_control.config.stack_build import GaitStackConfig
from marsdog_control.config.walk_recipe import (
    WALK_RECIPE,
    WALK_RECIPE_WBC,
    walk_recipe_dict,
)
from marsdog_control.motion.gait_params import (
    GaitParams,
    NaturalExtras,
    SoftExtras,
    SoftTrotBuild,
)

if TYPE_CHECKING:
    from marsdog_control.motion.gait_controller import GaitController


@dataclass(frozen=True)
class StandingPoseConfig:
    body_height: float
    x_offset_front: float
    x_offset_rear: float
    hip_abduction: float
    use_tarsus: bool
    waist_pitch_offset: float = 0.0
    waist_yaw_offset: float = 0.0
    front_stand_foot_pitch_deg: Optional[float] = None
    front_stand_tarsus_deg: float = 0.0

    # 2026-07-11: 唯一站姿=新三连杆带前腿主动 tarsus 站姿, 脚段默认 -90°(竖直指地)。
    _DEFAULT_FOOT_PITCH_DEG = -90.0

    @classmethod
    def from_config(
        cls, cfg: GaitStackConfig, *, front_x0: float, rear_x0: float,
    ) -> "StandingPoseConfig":
        foot_pitch_deg = cfg.front_stand_foot_pitch_deg
        if foot_pitch_deg is None:
            foot_pitch_deg = cls._DEFAULT_FOOT_PITCH_DEG
        return cls(
            body_height=cfg.height,
            x_offset_front=front_x0 + cfg.x_shift,
            x_offset_rear=rear_x0 + cfg.x_shift,
            hip_abduction=cfg.hip_abd,
            use_tarsus=True,
            waist_pitch_offset=cfg.waist_pitch,
            waist_yaw_offset=cfg.waist_yaw_offset,
            front_stand_foot_pitch_deg=foot_pitch_deg,
            front_stand_tarsus_deg=cfg.front_stand_tarsus_deg,
        )

    @classmethod
    def from_args(cls, args, *, front_x0: float, rear_x0: float) -> "StandingPoseConfig":
        """Compat wrapper — prefer :meth:`from_config`."""
        if isinstance(args, GaitStackConfig):
            return cls.from_config(args, front_x0=front_x0, rear_x0=rear_x0)
        return cls.from_config(
            GaitStackConfig.from_args(args), front_x0=front_x0, rear_x0=rear_x0)

    def build_stand_controller(self, hip_abduction_override: Optional[float] = None):
        from marsdog_control.motion.gait_controller import StandController

        stand = StandController(
            body_height=self.body_height,
            x_offset_front=self.x_offset_front,
            x_offset_rear=self.x_offset_rear,
            hip_abduction=(
                hip_abduction_override if hip_abduction_override is not None
                else self.hip_abduction
            ),
            use_tarsus=self.use_tarsus,
            front_stand_tarsus_deg=self.front_stand_tarsus_deg,
            front_stand_foot_pitch_deg=self.front_stand_foot_pitch_deg,
        )
        stand.waist_pitch_offset = self.waist_pitch_offset
        stand.waist_yaw_offset = self.waist_yaw_offset
        return stand


@dataclass(frozen=True)
class ControllerSet:
    stand: "GaitController"
    fwd: "GaitController"
    bwd: "GaitController"
    pace_fwd: "GaitController"
    pace_bwd: "GaitController"
    nat_fwd: "GaitController"
    walk_fwd: "GaitController"
    jump_fwd: "GaitController"

    def as_tuple(self):
        return (
            self.stand, self.fwd, self.bwd,
            self.pace_fwd, self.pace_bwd, self.nat_fwd, self.walk_fwd,
            self.jump_fwd,
        )

    def bind_ownership(self, *, lateral_planner, attitude_gate) -> None:
        """Inject session ownership into every gait handle."""
        from marsdog_control.motion.attitude_overlay import bind_ownership
        bind_ownership(
            lateral_planner=lateral_planner,
            attitude_gate=attitude_gate,
            gaits=self.as_tuple(),
        )


def apply_turn_params(controller, cfg: GaitStackConfig) -> None:
    controller.max_turn_amp_diff = cfg.turn_amp_diff
    controller.max_turn_y_amp = cfg.turn_y_amp
    controller.turn_filter_alpha = cfg.turn_smooth
    controller.max_turn_waist_yaw = cfg.turn_waist_yaw
    controller.waist_yaw_turn_sign = cfg.waist_yaw_turn_sign


def _set_waist_offsets(controller, cfg: GaitStackConfig) -> None:
    controller.waist_pitch_offset = cfg.waist_pitch
    controller.waist_yaw_offset = cfg.waist_yaw_offset


def _as_gait_stack_config(args_or_cfg: Union[GaitStackConfig, Any]) -> GaitStackConfig:
    if isinstance(args_or_cfg, GaitStackConfig):
        return args_or_cfg
    return GaitStackConfig.from_args(args_or_cfg)


def build_controller_set(
    args_or_cfg,
    *,
    front_x0: float,
    rear_x0: float,
    soft_build: Optional[SoftTrotBuild] = None,
    walk_recipe: Optional[Any] = None,
    jump_recipe: Optional[Any] = None,
    natural_params: Optional[dict] = None,
    walk_params: Optional[dict] = None,
    jump_params: Optional[dict] = None,
    natural_spine_yaw_deg: Optional[float] = None,
    natural_spine_roll_deg: Optional[float] = None,
    apply_turn: bool = True,
    pace_use_stand_offsets: bool = True,
) -> ControllerSet:
    """构造 stand/trot/pace/natural/walk/jump 控制器组。

    Prefer typed ``soft_build`` / ``walk_recipe`` / ``jump_recipe``. Dict kwargs
    remain as CLI-pour compat only.
    """
    from marsdog_control.config.jump_recipe import JumpRecipe, JUMP_RECIPE
    from marsdog_control.config.walk_recipe import WalkRecipe, WALK_RECIPE
    from marsdog_control.motion.gait_controller import (
        StablePace, StableTrot, NaturalTrot, NaturalSoftTrot, NaturalWalk,
        JumpController)

    cfg = _as_gait_stack_config(args_or_cfg)
    stand_cfg = StandingPoseConfig.from_config(cfg, front_x0=front_x0, rear_x0=rear_x0)
    xf = stand_cfg.x_offset_front
    xr = stand_cfg.x_offset_rear

    bwd_scale = cfg.bwd_amp_scale
    if cfg.fwd_use_bwd:
        fwd_amp_front = cfg.amp_rear * bwd_scale
        fwd_amp_rear = cfg.amp_front * bwd_scale
        fwd_step_h = cfg.bwd_step_h
        fwd_step_h_front = cfg.bwd_step_h * 0.75
        fwd_period = cfg.bwd_period
        fwd_hip_abd = cfg.hip_abd + 0.01
    else:
        fwd_amp_front = cfg.amp_front
        fwd_amp_rear = cfg.amp_rear
        fwd_step_h = cfg.step_h
        fwd_step_h_front = cfg.step_h_front if cfg.step_h_front else cfg.step_h * 0.75
        fwd_period = cfg.period
        fwd_hip_abd = cfg.hip_abd

    # stand 的髋外展必须和 fwd/nat_fwd 的起始姿态一致 (fwd_use_bwd 会给 fwd_hip_abd
    # 加 0.01rad), 否则 assert_stand_matches_gait_start 会在 rl_hip/rr_hip 上报不一致。
    stand = stand_cfg.build_stand_controller(hip_abduction_override=fwd_hip_abd)

    fwd_amp_front *= cfg.fwd_front_amp_scale
    if cfg.fwd_front_lift > 1e-6:
        fwd_step_h_front = cfg.fwd_front_lift

    base = cfg.shared_gait_params(
        x_offset_front=xf, x_offset_rear=xr, hip_abduction=fwd_hip_abd)

    fwd = StableTrot.from_params(base.with_overrides(
        amp_front=fwd_amp_front,
        amp_rear=fwd_amp_rear,
        step_height=fwd_step_h,
        step_height_front=fwd_step_h_front,
        period=fwd_period,
        hip_abduction=fwd_hip_abd,
    ))
    bwd = StableTrot.from_params(base.with_overrides(
        amp_front=-cfg.amp_rear * bwd_scale,
        amp_rear=-cfg.amp_front * bwd_scale,
        step_height=cfg.bwd_step_h,
        step_height_front=cfg.bwd_step_h * 0.75,
        period=cfg.bwd_period,
        hip_abduction=cfg.hip_abd + 0.01,
    ))

    pace_params = GaitParams(
        body_height=cfg.height,
        amp_front=cfg.pace_amp,
        amp_rear=cfg.pace_amp,
        step_height=cfg.pace_step_h,
        step_height_front=cfg.pace_step_h,
        period=cfg.pace_period,
        stance_ratio=cfg.pace_stance,
        hip_abduction=cfg.pace_hip_abd,
        ramp_duration=cfg.ramp,
        reactive_kp=cfg.reactive_kp,
        reactive_kd=cfg.reactive_kd,
        lateral_sway=cfg.pace_sway,
        swing_level=cfg.swing_level,
        smooth_gait=cfg.smooth_gait,
        x_offset_front=(xf if pace_use_stand_offsets else None),
        x_offset_rear=(xr if pace_use_stand_offsets else None),
    )
    pace_fwd = StablePace.from_params(pace_params)
    pace_bwd = StablePace.from_params(pace_params.with_overrides(
        amp_front=-cfg.pace_amp, amp_rear=-cfg.pace_amp))

    # ── Soft / Natural: typed SoftTrotBuild is SSOT; dict only if absent ──
    if soft_build is not None:
        natural_params = None
    else:
        import warnings
        if natural_params:
            warnings.warn(
                "build_controller_set(natural_params=...) without soft_build "
                "is deprecated; pass SoftTrotBuild",
                DeprecationWarning,
                stacklevel=2,
            )
        soft_build = SoftTrotBuild.from_gait_stack(
            cfg,
            x_offset_front=xf,
            x_offset_rear=xr,
            hip_abduction=fwd_hip_abd,
            spine_yaw_deg=natural_spine_yaw_deg,
            spine_roll_deg=natural_spine_roll_deg,
        )
        # Legacy dict may still carry Soft pour extras not yet on stack.
        if natural_params:
            soft_build = SoftTrotBuild(
                soft_build.gait.with_overrides(
                    **{
                        k: natural_params[k]
                        for k in (
                            "lateral_sway", "anti_roll",
                            "trot_roll_ff_neg_deg", "trot_roll_ff_pos_deg",
                            "anti_roll_asym_neg", "anti_roll_asym_pos",
                            "front_foot_swing_track",
                            "front_foot_stance_push_deg",
                        )
                        if k in natural_params
                    },
                    amp_front=natural_params.get("amp_front", soft_build.gait.amp_front),
                    amp_rear=natural_params.get("amp_rear", soft_build.gait.amp_rear),
                    period=natural_params.get("period", soft_build.gait.period),
                    step_height=natural_params.get("step_h", soft_build.gait.step_height),
                    step_height_front=natural_params.get(
                        "step_h_front", soft_build.gait.step_height_front),
                    stance_ratio=natural_params.get(
                        "stance", soft_build.gait.stance_ratio),
                    front_stand_foot_pitch_deg=natural_params.get(
                        "front_stand_foot_pitch_deg",
                        soft_build.gait.front_stand_foot_pitch_deg),
                    front_tarsus_push=0.0,
                    front_thrust_gain=1.0,
                    front_thrust_swing_gain=1.0,
                ),
                soft_build.natural,
                SoftExtras.from_mapping(
                    {**soft_build.soft.as_kwargs(), **natural_params}),
            )

    if cfg.natural_soft_trot:
        nat_fwd = NaturalSoftTrot.from_build(soft_build)
    else:
        nat_fwd = NaturalTrot.from_params(soft_build.gait, soft_build.natural)

    # ── NaturalWalk (typed WalkRecipe — no to_dict pour) ──
    if isinstance(walk_recipe, WalkRecipe):
        wr = walk_recipe
    elif walk_params:
        wr = WALK_RECIPE.with_overrides(walk_params)
    else:
        wr = WALK_RECIPE
    walk_fwd = NaturalWalk.from_params(
        base.with_overrides(
            body_height=float(wr.height),
            amp_front=float(wr.amp_front),
            amp_rear=float(wr.amp_rear),
            step_height=float(wr.step_h),
            step_height_front=float(wr.step_h_front),
            period=float(wr.period),
            hip_abduction=fwd_hip_abd,
            stance_ratio=float(wr.stance),
            lateral_sway=float(wr.lateral_sway),
            anti_roll=float(wr.anti_roll),
            trot_roll_ff_neg_deg=float(wr.trot_roll_ff_neg_deg),
            trot_roll_ff_pos_deg=float(wr.trot_roll_ff_pos_deg),
            anti_roll_asym_neg=float(wr.anti_roll_asym_neg),
            anti_roll_asym_pos=float(wr.anti_roll_asym_pos),
            front_tarsus_push=0.0,
            front_thrust_gain=1.0,
            front_thrust_swing_gain=1.0,
            front_foot_swing_track=float(wr.front_foot_swing_track),
            front_stand_foot_pitch_deg=float(wr.front_stand_foot_pitch_deg),
        ),
        NaturalExtras(
            spine_yaw_deg=float(wr.spine_yaw_deg),
            spine_roll_deg=float(wr.spine_roll_deg),
            spine_phase_deg=float(wr.spine_phase_deg),
            thigh_swing_front_deg=float(wr.thigh_swing_front_deg),
            thigh_swing_rear_deg=float(wr.thigh_swing_rear_deg),
            retract_front=float(wr.retract_front),
            retract_rear=float(wr.retract_rear),
            tarsus_swing_deg=float(wr.tarsus_swing_deg),
        ),
        soft=SoftExtras(
            touchdown_compress=float(wr.touchdown_compress),
            anti_roll_soft_scale=float(wr.anti_roll_soft_scale),
            toeoff_lift=float(wr.toeoff_lift),
            retract_peak=float(wr.retract_peak),
            lift_peak=float(wr.lift_peak),
            rear_clearance_m=float(wr.rear_clearance_m),
        ),
        com_sway_m=float(wr.com_sway_m),
    )

    # ── Jump (typed JumpRecipe — no to_dict pour) ──
    if isinstance(jump_recipe, JumpRecipe):
        jr = jump_recipe
    elif jump_params:
        jr = JUMP_RECIPE.with_overrides(jump_params)
    else:
        jr = JUMP_RECIPE
    jump_fwd = JumpController(
        body_height=float(jr.height),
        x_offset_front=xf,
        x_offset_rear=xr,
        hip_abduction=fwd_hip_abd,
        front_stand_tarsus_deg=base.front_stand_tarsus_deg,
        front_stand_foot_pitch_deg=float(jr.front_stand_foot_pitch_deg),
        crouch_depth=float(jr.crouch_depth),
        crouch_s=float(jr.crouch_s),
        push_s=float(jr.push_s),
        flight_s=float(jr.flight_s),
        land_s=float(jr.land_s),
        recover_s=float(jr.recover_s),
        flight_clearance=float(jr.flight_clearance),
        land_compress=float(jr.land_compress),
        push_vz=float(jr.push_vz),
        push_extend=float(jr.push_extend),
        kp_base_z=float(jr.kp_base_z),
        kd_base_z=float(jr.kd_base_z),
    )

    for controller in (fwd, bwd, pace_fwd, pace_bwd, nat_fwd, walk_fwd, jump_fwd):
        _set_waist_offsets(controller, cfg)
    if apply_turn:
        for controller in (fwd, bwd, nat_fwd):
            apply_turn_params(controller, cfg)

    return ControllerSet(
        stand, fwd, bwd, pace_fwd, pace_bwd, nat_fwd, walk_fwd, jump_fwd,
    )


SIM_PREVIEW_BASE = {
    "height": 0.24,
    "leg_kp_scale": 1.0,
    "var_impedance": False,
    "gravity_comp": False,
    "fwd_use_bwd": False,
    "fwd_front_amp_scale": 1.0,
    "lateral_sway": 0.006,
    "stance": 0.60,
    "ff_decouple": True,
    "auto_trim": False,
    "anti_roll": 0.014,
    "anti_roll_asym_neg": 1.30,
    "anti_roll_asym_pos": 0.85,
    "step_h": 0.012,
    "trot_roll_ff_neg_deg": 2.6,
    "trot_roll_ff_pos_deg": 2.2,
    "front_thrust_gain": 1.0,
    "front_thrust_swing_gain": 1.0,
    "front_tarsus_push": 0.0,
    "front_foot_track_deg": -78.0,
    "front_foot_stance_push_deg": 6.0,
    "front_stand_foot_pitch_deg": -90.0,
    "swing_clearance_per_rad": 0.35,
    "imu_kp": 0.035,
    "imu_predict_ms": 10.0,        # 额外执行提前量；总预测由 angle age 动态补足
    "imu_slew_mm_s": 120.0,
    "max_corr_mm": 15.0,
    "imu_phase_gate": True,
    "td_imu_freeze_i": True,
}


SIM_PREVIEW_NATURAL_TROT = {
    "lateral_sway": 0.010,
    "anti_roll": 0.022,
    "spine_yaw_deg": 0.0,
    "spine_roll_deg": 0.0,
    "imu_kp": 0.055,
    "max_corr_mm": 20.0,
    "imu_slew_mm_s": 110.0,
    "imu_phase_td_gain": 0.30,
    "imu_phase_swing_gain": 0.65,
    "nat_step_h": 0.022,
    "fwd_front_lift": 0.020,
    "nat_amp_rear": 0.012,
    "anti_roll_asym_neg": 1.15,
    "anti_roll_asym_pos": 0.92,
    "front_foot_stance_push_deg": 14.0,
}


# SoftTrot **唯一真源** → ``config.soft_trot_recipe.SoftTrotRecipe``。
# 本 dict 仅为兼容出口（apply_preset / natural_params）；勿手写第二套数字。
# ``nat_*`` 由 Recipe.to_dict() 从 amp/period/step 派生；横向只留 com_shift。
NATURAL_SOFT_TROT = soft_trot_recipe_dict()

# 历史别名：同一对象，禁止再写第二套几何。
NATURAL_SOFT_TROT_WBC = NATURAL_SOFT_TROT
NATURAL_SOFT_TROT_REAL = NATURAL_SOFT_TROT

# Sim preview: explicit overrides on SoftTrotRecipe (not a second SSOT).
_SIM_PREVIEW_SOFT_OVERRIDES = {
    "amp_front": 0.026,
    "amp_rear": 0.026,
    "period": 0.90,
    "nat_period": 0.90,
    "nat_amp_front": 0.026,
    "nat_amp_rear": 0.026,
    "nat_step_h": 0.040,
    "step_h_front": 0.040,
    "fwd_front_lift": 0.040,
    "stance": 0.66,
    # Preview uses LateralOwner.SWAY — must not dual-own with com_shift.
    "com_shift_m": 0.0,
    "lateral_sway": 0.004,
    "anti_roll": 0.010,
    "anti_roll_asym_neg": 1.05,
    "anti_roll_asym_pos": 0.98,
    "trot_roll_ff_neg_deg": 1.2,
    "trot_roll_ff_pos_deg": 1.0,
    "front_foot_stance_push_deg": 10.0,
    "front_foot_swing_track": 0.0,
    "spine_yaw_deg": 0.0,
    "spine_roll_deg": 0.0,
    "thigh_swing_front_deg": 0.0,
    "thigh_swing_rear_deg": 12.0,
    "retract_front": 0.018,
    "retract_rear": 0.014,
    "tarsus_swing_deg": 0.0,
    "touchdown_compress": 0.004,
    "anti_roll_soft_scale": 0.35,
    "toeoff_lift": 0.002,
    "retract_peak": 0.36,
    "lift_peak": 0.42,
    # IMU / WBC knobs are schema-owned; Soft pour no longer carries them.
}

SIM_PREVIEW_NATURAL_SOFT_TROT = SOFT_TROT_RECIPE.with_overrides(
    _SIM_PREVIEW_SOFT_OVERRIDES
).to_dict()


TROT_PREVIEW_REAL = {
    "height": 0.24,
    "fwd_use_bwd": False,
    "fwd_front_amp_scale": 1.0,
    "lateral_sway": 0.0,
    "stance": 0.60,
    "ff_decouple": True,
    "anti_roll": 0.010,
    "anti_roll_asym_neg": 1.30,
    "anti_roll_asym_pos": 0.85,
    "step_h": 0.012,
    "trot_roll_ff_neg_deg": 3.2,
    "trot_roll_ff_pos_deg": 1.8,
    "front_thrust_gain": 1.0,
    "front_thrust_swing_gain": 1.0,
    "front_tarsus_push": 0.0,
    "front_foot_track_deg": -78.0,
    "front_foot_stance_push_deg": 6.0,
    "front_stand_foot_pitch_deg": -90.0,
    "swing_clearance_per_rad": 0.35,
    "imu_kp": 0.035,
    "imu_predict_ms": 10.0,        # 100Hz后仅作为额外执行提前量
    "imu_slew_mm_s": 120.0,
    "max_corr_mm": 15.0,
    "imu_phase_gate": True,
    "td_imu_freeze_i": True,
}


NATURAL_TROT_REAL = {
    "amp_front": 0.012,
    "amp_rear": 0.010,
    "period": 0.90,
    "spine_yaw_deg": 0.0,
    "spine_roll_deg": 0.0,
    "lateral_sway": 0.006,
    "anti_roll": 0.014,
    "imu_kp": 0.040,
    "step_h": 0.018,
    "step_h_front": 0.015,
    "trot_roll_ff_neg_deg": 2.5,
    "trot_roll_ff_pos_deg": 1.5,
    "anti_roll_asym_neg": 1.20,
    "anti_roll_asym_pos": 0.90,
    # 注意: 不设 front_stand_foot_pitch_deg。REAL 预设默认不开 front_foot_track_deg
    # (足朝向跟踪), NaturalTrot 在 ramp=0 时的 tarsus 落在"legacy tarsus_push"
    # 中性值(≈0), 若这里强行给 stand 设 -90°(≈49° tarsus)会和步态起始姿态不一致,
    # 触发 assert_stand_matches_gait_start。等真正启用 front_foot_track_deg 时再一起加上。
    "max_corr_mm": 18.0,
    "imu_slew_mm_s": 110.0,
    "imu_phase_td_gain": 0.30,
    "imu_phase_swing_gain": 0.65,
}


# 真狗四拍慢走 — WalkRecipe SSOT；与 SoftTrot 完全解耦。
NATURAL_WALK_REAL = walk_recipe_dict(WALK_RECIPE)
NATURAL_WALK_WBC = walk_recipe_dict(WALK_RECIPE_WBC)

# 原地 hop — JumpRecipe SSOT；kp/kd_base_z 挂 JumpController，不 pour 进 Soft Dynamics。
JUMP_REAL = jump_recipe_dict(JUMP_RECIPE)
JUMP_WBC = jump_recipe_dict(JUMP_RECIPE_WBC)


def apply_values(args, values: dict) -> None:
    for key, value in values.items():
        setattr(args, key, value)


def apply_sim_preview(
    args,
    *,
    physics_options_factory: Optional[Callable[..., object]] = None,
) -> None:
    apply_values(args, SIM_PREVIEW_BASE)
    if physics_options_factory is not None:
        args.sim_physics = physics_options_factory(
            ground_friction=(1.8, 1.2, 0.001),
            foot_friction=(1.5, 1.0, 0.001),
        )
    if getattr(args, "natural_soft_trot", False):
        apply_values(args, SIM_PREVIEW_NATURAL_SOFT_TROT)
    elif getattr(args, "natural_trot", False):
        apply_values(args, SIM_PREVIEW_NATURAL_TROT)


def apply_trot_preview_real(args) -> None:
    apply_values(args, TROT_PREVIEW_REAL)
