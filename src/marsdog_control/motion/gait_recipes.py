"""集中管理站姿配置和步态预设。

目标是把 sim_walk.py / walk.py / 调试工具里散落的关键默认值收敛到这里。
这里不包含控制循环，也不直接依赖 MuJoCo，方便实机和仿真共同导入。
"""

from __future__ import annotations

# [解耦] 真实实现已从 mocap_to_real 下沉到此 src 模块; 函数内的扁平 import
# (from gait_controller import ...) 由 ensure_legacy_path() 保证可解析。
from marsdog_control.compat import ensure_legacy_path as _ensure_legacy_path
_ensure_legacy_path()

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Callable, Optional, Union

from marsdog_control.config.stack_build import GaitStackConfig

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
    natural_params: Optional[dict] = None,
    walk_params: Optional[dict] = None,
    jump_params: Optional[dict] = None,
    natural_spine_yaw_deg: Optional[float] = None,
    natural_spine_roll_deg: Optional[float] = None,
    apply_turn: bool = True,
    pace_use_stand_offsets: bool = True,
) -> ControllerSet:
    """构造 stand/trot/pace/natural/walk/jump 控制器组。

    首选传入 :class:`GaitStackConfig`；仍接受 CLI Namespace（内部一次性快照）。
    这里只负责控制器对象和共享参数，不处理仿真/实机 IO、effort override、
    IMU、键盘/手柄等运行时逻辑。
    """
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

    common = dict(
        body_height=cfg.height,
        stance_ratio=cfg.stance,
        ramp_duration=cfg.ramp,
        reactive_kp=cfg.reactive_kp,
        reactive_kd=cfg.reactive_kd,
        lateral_sway=cfg.lateral_sway,
        front_thrust_gain=cfg.front_thrust_gain,
        front_thrust_swing_gain=cfg.front_thrust_swing_gain,
        front_tarsus_push=cfg.front_tarsus_push,
        front_foot_track_deg=cfg.front_foot_track_deg,
        front_foot_stance_push_deg=cfg.front_foot_stance_push_deg,
        front_foot_swing_track=cfg.front_foot_swing_track,
        front_stand_tarsus_deg=cfg.front_stand_tarsus_deg,
        front_stand_foot_pitch_deg=cfg.front_stand_foot_pitch_deg,
        swing_clearance_per_rad=cfg.swing_clearance_per_rad,
        x_offset_front=xf,
        x_offset_rear=xr,
        anti_roll=cfg.anti_roll,
        trot_roll_ff_neg_deg=cfg.trot_roll_ff_neg_deg,
        trot_roll_ff_pos_deg=cfg.trot_roll_ff_pos_deg,
        anti_roll_asym_neg=cfg.anti_roll_asym_neg,
        anti_roll_asym_pos=cfg.anti_roll_asym_pos,
    )

    fwd = StableTrot(
        amp_front=fwd_amp_front,
        amp_rear=fwd_amp_rear,
        step_height=fwd_step_h,
        step_height_front=fwd_step_h_front,
        period=fwd_period,
        hip_abduction=fwd_hip_abd,
        **common,
    )
    bwd = StableTrot(
        amp_front=-cfg.amp_rear * bwd_scale,
        amp_rear=-cfg.amp_front * bwd_scale,
        step_height=cfg.bwd_step_h,
        step_height_front=cfg.bwd_step_h * 0.75,
        period=cfg.bwd_period,
        hip_abduction=cfg.hip_abd + 0.01,
        **common,
    )

    pace_common = dict(
        body_height=cfg.height,
        step_height=cfg.pace_step_h,
        step_height_front=cfg.pace_step_h,
        period=cfg.pace_period,
        stance_ratio=cfg.pace_stance,
        hip_abduction=cfg.pace_hip_abd,
        ramp_duration=cfg.ramp,
        reactive_kp=cfg.reactive_kp,
        reactive_kd=cfg.reactive_kd,
        lateral_sway=cfg.pace_sway,
    )
    if pace_use_stand_offsets:
        pace_common.update(x_offset_front=xf, x_offset_rear=xr)
    pace_fwd = StablePace(amp_front=cfg.pace_amp, amp_rear=cfg.pace_amp, **pace_common)
    pace_bwd = StablePace(amp_front=-cfg.pace_amp, amp_rear=-cfg.pace_amp, **pace_common)

    np = natural_params
    nat_amp_front = (
        np["amp_front"] if np and "amp_front" in np else cfg.nat_amp_front
    )
    nat_amp_rear = (
        np["amp_rear"] if np and "amp_rear" in np else cfg.nat_amp_rear
    )
    nat_period = (
        np["period"] if np and "period" in np else cfg.nat_period
    )
    nat_step_h = (
        np["step_h"] if np and "step_h" in np else cfg.nat_step_h
    )
    nat_step_h_front = (
        np["step_h_front"]
        if np and "step_h_front" in np
        else (cfg.fwd_front_lift if cfg.fwd_front_lift > 1e-6 else nat_step_h * 0.75)
    )
    nat_common = dict(common)
    if np:
        nat_common.update(
            lateral_sway=np.get("lateral_sway", nat_common["lateral_sway"]),
            anti_roll=np.get("anti_roll", nat_common["anti_roll"]),
            trot_roll_ff_neg_deg=np.get("trot_roll_ff_neg_deg", nat_common["trot_roll_ff_neg_deg"]),
            trot_roll_ff_pos_deg=np.get("trot_roll_ff_pos_deg", nat_common["trot_roll_ff_pos_deg"]),
            anti_roll_asym_neg=np.get("anti_roll_asym_neg", nat_common["anti_roll_asym_neg"]),
            anti_roll_asym_pos=np.get("anti_roll_asym_pos", nat_common["anti_roll_asym_pos"]),
            front_tarsus_push=0.0,
            front_thrust_gain=1.0,
            front_thrust_swing_gain=1.0,
            # 摆动相保留部分足朝向跟踪，避免触地/离地时 foot_pitch 被门控砸到站立角，
            # 经 3-link IK 把 hip_pitch 目标拐成速度反向（日志里的“震一下”）。
            front_foot_swing_track=np.get(
                "front_foot_swing_track", cfg.front_foot_swing_track),
            front_foot_stance_push_deg=np.get(
                "front_foot_stance_push_deg",
                nat_common["front_foot_stance_push_deg"]),
            front_stand_foot_pitch_deg=(
                cfg.front_stand_foot_pitch_deg
                if cfg.front_stand_foot_pitch_deg is not None
                else np.get("front_stand_foot_pitch_deg")
            ),
            stance_ratio=np.get("stance", nat_common["stance_ratio"]),
        )

    natural_cls = NaturalSoftTrot if cfg.natural_soft_trot else NaturalTrot
    natural_extra = {}
    if natural_cls is NaturalSoftTrot:
        natural_extra.update(
            touchdown_compress=(
                np.get("touchdown_compress", cfg.touchdown_compress)
                if np else cfg.touchdown_compress
            ),
            anti_roll_soft_scale=(
                np.get("anti_roll_soft_scale", cfg.anti_roll_soft_scale)
                if np else cfg.anti_roll_soft_scale
            ),
            toeoff_lift=(
                np.get("toeoff_lift", cfg.toeoff_lift)
                if np else cfg.toeoff_lift
            ),
            retract_peak=(
                np.get("retract_peak", cfg.retract_peak)
                if np else cfg.retract_peak
            ),
            lift_peak=(
                np.get("lift_peak", cfg.lift_peak)
                if np else cfg.lift_peak
            ),
            rear_clearance_m=(
                np.get("rear_clearance_m", getattr(cfg, "rear_clearance_m", 0.0))
                if np else getattr(cfg, "rear_clearance_m", 0.0)
            ),
            com_shift_m=(
                np.get("com_shift_m", getattr(cfg, "com_shift_m", 0.0))
                if np else getattr(cfg, "com_shift_m", 0.0)
            ),
            com_shift_blend=(
                np.get("com_shift_blend", getattr(cfg, "com_shift_blend", 0.12))
                if np else getattr(cfg, "com_shift_blend", 0.12)
            ),
        )

    nat_fwd = natural_cls(
        amp_front=nat_amp_front,
        amp_rear=nat_amp_rear,
        step_height=nat_step_h,
        step_height_front=nat_step_h_front,
        period=nat_period,
        hip_abduction=fwd_hip_abd,
        spine_yaw_deg=(
            cfg.spine_yaw_deg
            if natural_spine_yaw_deg is None
            else natural_spine_yaw_deg
        ),
        spine_roll_deg=(
            cfg.spine_roll_deg
            if natural_spine_roll_deg is None
            else natural_spine_roll_deg
        ),
        spine_phase_deg=cfg.spine_phase_deg,
        thigh_swing_front_deg=cfg.thigh_swing_front_deg,
        thigh_swing_rear_deg=cfg.thigh_swing_rear_deg,
        retract_front=cfg.retract_front,
        retract_rear=cfg.retract_rear,
        tarsus_swing_deg=cfg.tarsus_swing_deg,
        **natural_extra,
        **nat_common,
    )

    # ── NaturalWalk: 独立配方，不读 SoftTrot 已灌进 cfg 的数字 ──
    wp = dict(walk_params) if walk_params else dict(NATURAL_WALK_REAL)
    walk_common = dict(common)
    walk_common.update(
        body_height=wp.get("height", common["body_height"]),
        stance_ratio=wp.get("stance", 0.74),
        lateral_sway=wp.get("lateral_sway", 0.008),
        anti_roll=wp.get("anti_roll", 0.0),
        trot_roll_ff_neg_deg=wp.get("trot_roll_ff_neg_deg", 0.0),
        trot_roll_ff_pos_deg=wp.get("trot_roll_ff_pos_deg", 0.0),
        anti_roll_asym_neg=wp.get("anti_roll_asym_neg", 1.0),
        anti_roll_asym_pos=wp.get("anti_roll_asym_pos", 1.0),
        front_tarsus_push=0.0,
        front_thrust_gain=1.0,
        front_thrust_swing_gain=1.0,
        front_foot_swing_track=0.0,
        front_stand_foot_pitch_deg=wp.get(
            "front_stand_foot_pitch_deg",
            common.get("front_stand_foot_pitch_deg"),
        ),
    )
    walk_fwd = NaturalWalk(
        amp_front=float(wp.get("amp_front", 0.032)),
        amp_rear=float(wp.get("amp_rear", 0.036)),
        step_height=float(wp.get("step_h", 0.038)),
        step_height_front=float(wp.get("step_h_front", wp.get("step_h", 0.038))),
        period=float(wp.get("period", 1.00)),
        hip_abduction=fwd_hip_abd,
        spine_yaw_deg=float(wp.get("spine_yaw_deg", 5.0)),
        spine_roll_deg=float(wp.get("spine_roll_deg", 2.4)),
        spine_phase_deg=float(wp.get("spine_phase_deg", 0.0)),
        thigh_swing_front_deg=float(wp.get("thigh_swing_front_deg", 0.0)),
        thigh_swing_rear_deg=float(wp.get("thigh_swing_rear_deg", 12.0)),
        retract_front=float(wp.get("retract_front", 0.032)),
        retract_rear=float(wp.get("retract_rear", 0.036)),
        tarsus_swing_deg=float(wp.get("tarsus_swing_deg", 0.0)),
        touchdown_compress=float(wp.get("touchdown_compress", 0.006)),
        anti_roll_soft_scale=float(wp.get("anti_roll_soft_scale", 0.0)),
        toeoff_lift=float(wp.get("toeoff_lift", 0.008)),
        retract_peak=float(wp.get("retract_peak", 0.22)),
        lift_peak=float(wp.get("lift_peak", 0.26)),
        rear_clearance_m=float(wp.get("rear_clearance_m", 0.020)),
        com_sway_m=float(wp.get("com_sway_m", wp.get("lateral_sway", 0.010) * 1.4)),
        **walk_common,
    )

    # ── Jump: 独立配方，绝不读 Soft/Walk 数字进 Jump，也不把 Jump 灌进 Soft args ──
    jp = dict(jump_params) if jump_params else dict(JUMP_REAL)
    jump_fwd = JumpController(
        body_height=float(jp.get("height", common["body_height"])),
        x_offset_front=common["x_offset_front"],
        x_offset_rear=common["x_offset_rear"],
        hip_abduction=fwd_hip_abd,
        front_stand_tarsus_deg=common.get("front_stand_tarsus_deg", 0.0),
        front_stand_foot_pitch_deg=jp.get(
            "front_stand_foot_pitch_deg",
            common.get("front_stand_foot_pitch_deg"),
        ),
        crouch_depth=float(jp.get("crouch_depth", 0.045)),
        crouch_s=float(jp.get("crouch_s", 0.28)),
        push_s=float(jp.get("push_s", 0.12)),
        flight_s=float(jp.get("flight_s", 0.18)),
        land_s=float(jp.get("land_s", 0.22)),
        recover_s=float(jp.get("recover_s", 0.25)),
        flight_clearance=float(jp.get("flight_clearance", 0.025)),
        land_compress=float(jp.get("land_compress", 0.012)),
        push_vz=float(jp.get("push_vz", 0.55)),
        push_extend=float(jp.get("push_extend", 0.020)),
        kp_base_z=float(jp.get("kp_base_z", 80.0)),
        kd_base_z=float(jp.get("kd_base_z", 10.0)),
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


SIM_PREVIEW_NATURAL_SOFT_TROT = {
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
    # 前腿由三关节 IK 严格保持足朝向；禁止 IK 后单关节 flourish。
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
    "imu_kp": 0.040,
    "imu_kp_pitch": 0.040,
    "max_corr_mm": 14.0,
    "imu_slew_mm_s": 80.0,
    "imu_phase_td_gain": 0.25,
    "imu_phase_swing_gain": 0.50,
}


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


# SoftTrot **唯一真源**（仿真/真机 walk_startup 只灌这一份）。
# 调参：只改本 dict。勿再维护 REAL/WBC 两套几何。
# 横向只留 com_shift；IMU lead/spine/flourish 等互殴叠层默认全关。
NATURAL_SOFT_TROT = {
    # ── 体高 / 节奏 / 支撑相 ──
    # 2026-08-03 真机锁 D：T=1.05 / st=0.72 / kp×0.90（连感+稳，优于 1.20/0.74 机械感）
    "height": 0.25,
    "period": 1.05,
    "nat_period": 1.05,
    "stance": 0.72,
    "amp_front": 0.022,
    "amp_rear": 0.030,
    "nat_amp_front": 0.022,
    "nat_amp_rear": 0.030,
    "step_h": 0.024,
    "nat_step_h": 0.024,
    "step_h_front": 0.020,
    "fwd_front_lift": 0.020,
    "fwd_front_amp_scale": 1.0,
    # SoftTrot 默认柔顺叠加（schema.control 同步）；CLI --leg-kp-scale 可覆盖
    "leg_kp_scale": 0.90,
    # ── 核心足端形状 ──
    # 3.5mm + 加宽 TD 窗：旧 6mm/0.24stance 在短 stance 下后小腿目标 dq 爆
    "touchdown_compress": 0.0035,
    "anti_roll_soft_scale": 0.0,
    "toeoff_lift": 0.0008,
    "retract_peak": 0.42,
    "lift_peak": 0.48,
    "thigh_swing_front_deg": 0.0,
    "thigh_swing_rear_deg": 0.0,
    "retract_front": 0.010,
    "retract_rear": 0.008,
    "tarsus_swing_deg": 0.0,
    "swing_clearance_per_rad": 0.35,
    # ── 横向唯一策略 ──
    # 2026-08-04 真机：com_shift 12→4mm；x_shift 默认 0（见 gait_tuning）
    "com_shift_m": 0.004,
    # 0.15：回绕与半周切换同为 2·blend，触地前完成移重（上限见 foot_trajectory）
    "com_shift_blend": 0.15,
    "rear_clearance_m": 0.0,
    "swing_level": 0.0,
    "front_thrust_gain": 1.0,
    "front_thrust_swing_gain": 1.0,
    "front_tarsus_push": 0.0,
    "front_foot_track_deg": -78.0,
    "front_foot_stance_push_deg": 0.0,
    "front_foot_swing_track": 1.0,
    "front_stand_foot_pitch_deg": -90.0,
    "spine_yaw_deg": 0.0,
    "spine_roll_deg": 0.0,
    "throttle_min_scale": 0.45,
    # ── 互殴叠层：全部关 ──
    "lateral_sway": 0.0,
    "anti_roll": 0.0,
    "anti_roll_asym_neg": 1.0,
    "anti_roll_asym_pos": 1.0,
    "trot_roll_ff_neg_deg": 0.0,
    "trot_roll_ff_pos_deg": 0.0,
    "ff_decouple": False,
    "imu_kp": 0.040,
    "imu_kp_pitch": 0.040,
    "max_corr_mm": 14.0,
    "imu_slew_mm_s": 0.0,
    "imu_predict_ms": 0.0,
    "imu_softstart_s": 0.0,
    "dynamic_imu_predict": False,
    "imu_phase_gate": False,
    "imu_phase_td_gain": 0.25,
    "imu_phase_swing_gain": 0.50,
    "td_imu_freeze_i": False,
    "tarsus_lead_fl_ms": 0.0,
    "tarsus_lead_fr_ms": 0.0,
    "dm_dq_feedforward": False,
    # Spot-turn / cruise turn
    "turn_y_amp": 0.040,
    "turn_amp_diff": 0.012,
    "turn_waist_yaw": 0.40,
    "kp_base_roll": 68.0,
    "kd_base_roll": 20.0,
    "lateral_vel_damp": 14.0,
    "swing_foot_kp": 70.0,
    "com_y_shift_m": 0.0,
}

# 历史别名：同一对象，禁止再写第二套几何。
NATURAL_SOFT_TROT_WBC = NATURAL_SOFT_TROT
NATURAL_SOFT_TROT_REAL = NATURAL_SOFT_TROT

# 真狗四拍慢走 — 与 SoftTrot 完全解耦；只改本 dict，勿动 NATURAL_SOFT_TROT_*。
# 抬腿序 LH→LF→RH→RF；可读走速（非爬行）+ 事件型侧移 + 自有足端曲线。
NATURAL_WALK_REAL = {
    "height": 0.24,
    "period": 1.05,
    "nat_period": 1.05,
    "stance": 0.75,
    "amp_front": 0.050,
    "amp_rear": 0.058,
    "nat_amp_front": 0.050,
    "nat_amp_rear": 0.058,
    "step_h": 0.034,
    "nat_step_h": 0.034,
    "step_h_front": 0.032,
    "fwd_front_lift": 0.032,
    "touchdown_compress": 0.006,
    "anti_roll_soft_scale": 0.0,
    "toeoff_lift": 0.007,
    "retract_peak": 0.22,
    "lift_peak": 0.26,
    "thigh_swing_front_deg": 0.0,
    "thigh_swing_rear_deg": 12.0,
    "retract_front": 0.030,
    "retract_rear": 0.034,
    "tarsus_swing_deg": 0.0,
    "swing_clearance_per_rad": 0.35,
    "front_thrust_gain": 1.0,
    "front_thrust_swing_gain": 1.0,
    "front_tarsus_push": 0.0,
    "front_foot_track_deg": -78.0,
    "front_foot_stance_push_deg": 8.0,
    "front_foot_swing_track": 0.0,
    "front_stand_foot_pitch_deg": -90.0,
    "spine_yaw_deg": 5.5,
    "spine_roll_deg": 2.5,
    "spine_phase_deg": 0.0,
    "lateral_sway": 0.010,
    "com_sway_m": 0.024,
    "anti_roll": 0.0,
    "anti_roll_asym_neg": 1.0,
    "anti_roll_asym_pos": 1.0,
    "trot_roll_ff_neg_deg": 0.0,
    "trot_roll_ff_pos_deg": 0.0,
    "ff_decouple": True,
    "rear_clearance_m": 0.018,
    "throttle_min_scale": 0.55,
}

# WBC 仿真 / 同参真机 — 四拍可读慢走
NATURAL_WALK_WBC = {
    **NATURAL_WALK_REAL,
    "amp_front": 0.054,
    "amp_rear": 0.062,
    "nat_amp_front": 0.054,
    "nat_amp_rear": 0.062,
    "step_h": 0.036,
    "nat_step_h": 0.036,
    "step_h_front": 0.034,
    "fwd_front_lift": 0.034,
    "period": 1.00,
    "nat_period": 1.00,
    "stance": 0.75,
    "lateral_sway": 0.009,
    "com_sway_m": 0.026,
    "spine_yaw_deg": 5.0,
    "spine_roll_deg": 2.4,
    "spine_phase_deg": 0.0,
    "retract_front": 0.032,
    "retract_rear": 0.036,
    "retract_peak": 0.22,
    "lift_peak": 0.26,
    "toeoff_lift": 0.008,
    "touchdown_compress": 0.006,
    "rear_clearance_m": 0.020,
    "throttle_min_scale": 0.50,
    "kp_base_roll": 74.0,
    "kd_base_roll": 22.0,
}

# 原地 hop — 与 SoftTrot/Walk/Spot 完全解耦；只改本 dict。
# kp/kd_base_z 挂在 JumpController 上，executor 仅 jump_now 时读取；
# 绝不 apply_preset 进全局 DynamicsConfig（避免 Soft 被 Jump 增益盖写）。
JUMP_REAL = {
    "height": 0.24,
    "crouch_depth": 0.050,
    "crouch_s": 0.30,
    "push_s": 0.14,
    "flight_s": 0.20,
    "land_s": 0.25,
    "recover_s": 0.28,
    "flight_clearance": 0.030,
    "land_compress": 0.014,
    "push_vz": 0.60,
    "push_extend": 0.022,
    "front_stand_foot_pitch_deg": -90.0,
    "kp_base_z": 80.0,
    "kd_base_z": 10.0,
}

# WBC 仿真 — 稳的一版：单次腾空优先，不过度加后腿力
JUMP_WBC = {
    **JUMP_REAL,
    "crouch_depth": 0.070,
    "crouch_s": 0.24,
    "push_s": 0.18,
    "flight_s": 0.30,
    "land_s": 0.34,
    "recover_s": 0.40,
    "flight_clearance": 0.075,
    "land_compress": 0.022,
    "push_vz": 2.2,
    "push_extend": 0.016,
    "kp_base_z": 140.0,
    "kd_base_z": 12.0,
}


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
