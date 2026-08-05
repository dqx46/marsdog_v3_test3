"""Stable trot / pace gait controllers."""
import math
from typing import Any, Optional

from marsdog_control.motion.kinematics import (
    ik_front_leg_2d,
    front_standing_foot_pitch,
)
from marsdog_control.config.joints import JOINT_BY_NAME
from marsdog_control.motion import foot_trajectory as _ft
from marsdog_control.motion.gait_base import (
    GaitController,
    _FRONT_HIP_OFFSET,
    _REAR_HIP_OFFSET,
    _FRONT_X0,
    _REAR_X0,
    _COM_TO_FRONT,
    _COM_TO_REAR,
    _HALF_TRACK_FRONT,
    _HALF_TRACK_REAR,
    _clamp,
)

class StableTrot(GaitController):
    """稳定 Trot — Raibert 反应式落脚 + 横向 CoM 摆动 + 三段 Z 轨迹。

    核心稳定算法:
      1. Raibert 反应式落脚点 — 摆动腿根据 IMU roll/gyro 动态调整横向落点
         身体左倾 → 所有摆动腿向左偏移 → 扩大支撑面 → "接住"摔倒趋势
         公式: Δθ = kp·roll + kd·gyro_roll (仅作用于摆动腿)
      2. 横向 CoM 摆动 — 支撑相中把重心移向支撑对角线
         FL+RR支撑 → body左移; FR+RL支撑 → body右移
      3. 三段式摆动 Z — sin²快起 + 巡航 + cos²柔落

    相位: FL+RR=0.0, FR+RL=0.5
    """

    _PHASE_OFFSET = {'fl': 0.0, 'rr': 0.0, 'fr': 0.5, 'rl': 0.5}

    _RISE_END  = 0.4
    _CRUISE_END = 0.7

    def __init__(self,
                 body_height:      float = 0.24,
                 amp_front:        float = 0.020,
                 amp_rear:         float = 0.025,
                 step_height:      float = 0.020,
                 step_height_front: float = None,
                 period:           float = 1.0,
                 stance_ratio:     float = 0.65,
                 x_offset_front:   float = None,
                 x_offset_rear:    float = None,
                 hip_abduction:    float = 0.04,
                 lateral_sway:     float = 0.0,
                 anti_roll:        float = 0.003,
                 reactive_kp:      float = 0.0,
                 reactive_kd:      float = 0.0,
                 ramp_duration:    float = 1.0,
                 front_thrust_gain: float = 1.0,
                 front_thrust_swing_gain: float = 1.0,
                 front_tarsus_push: float = 0.0,
                 front_foot_track_deg: float = None,
                 front_foot_stance_push_deg: float = 0.0,
                 front_foot_swing_track: float = 1.0,
                 front_stand_tarsus_deg: float = 0.0,
                 front_stand_foot_pitch_deg: float = None,
                 swing_clearance_per_rad: float = 0.0,
                 trot_roll_ff_neg_deg: float = 0.0,
                 trot_roll_ff_pos_deg: float = 0.0,
                 anti_roll_asym_neg: float = 1.0,
                 anti_roll_asym_pos: float = 1.0,
                 swing_level: float = 0.0,
                 smooth_gait: bool = False,
                 *,
                 lateral_planner=None,
                 attitude_overlay_gate=None):
        super().__init__(
            lateral_planner=lateral_planner,
            attitude_overlay_gate=attitude_overlay_gate,
        )
        self.body_height      = body_height
        self.amp_front        = amp_front
        self.amp_rear         = amp_rear
        self.step_height      = step_height
        self.step_height_front = step_height_front if step_height_front is not None else step_height * 0.75
        self.ramp_duration    = ramp_duration
        self.period           = period
        self.stance_ratio     = stance_ratio
        self.x_offset_front   = x_offset_front if x_offset_front is not None else _FRONT_X0
        self.x_offset_rear    = x_offset_rear if x_offset_rear is not None else _REAR_X0
        self.hip_abduction    = hip_abduction
        self.lateral_sway     = lateral_sway
        self.anti_roll        = anti_roll
        self.reactive_kp      = reactive_kp
        self.reactive_kd      = reactive_kd
        # Per-controller (was module globals mutated by startup).
        self.swing_level = max(0.0, min(1.0, float(swing_level)))
        self.smooth_gait = bool(smooth_gait)
        # 前腿推力增益: hip_pitch(大腿)摆动放大倍数, calf(小腿)只解算高度
        #   1.0 = 等价原全IK; >1 = 大腿摆动更大 → 步幅/推力更大, 小腿近似刚性
        # 支撑相用 front_thrust_gain (可<1 减前腿推进); 摆动相用 front_thrust_swing_gain
        # (默认 1.0=全 IK 抬腿, 避免半摆幅+小腿硬凑高度导致左右抬腿不对称)
        self.front_thrust_gain = front_thrust_gain
        self.front_thrust_swing_gain = front_thrust_swing_gain
        # 前腿脚踝(tarsus)支撑相蹬地幅度 (rad, URDF); 0=中性(等效锁死), >0 支撑相 plantarflex 推进
        self.front_tarsus_push = front_tarsus_push
        # 前腿足朝向跟踪 (deg): 让足段绝对朝向恒定(≈脚尖指地), 复现 RL 策略, 天然协调三关节。
        #   None=关闭(用 tarsus_push 鼓包旧法); -90≈竖直朝下。启用时优先于 tarsus_push。
        self.front_foot_track_deg = front_foot_track_deg
        # 支撑相在跟踪基准上额外前倾(蹬地)的度数, 摆动相回中性 (仅足朝向模式生效)
        self.front_foot_stance_push_deg = front_foot_stance_push_deg
        # 摆动相保留多少足朝向跟踪 [0,1]: 1=全程跟踪(swing 时 tarsus 大幅补偿, 甩动大);
        #   0=摆动相回站立朝向(减少 tarsus 甩动与反作用力矩, 只在触地时脚尖指地)
        self.front_foot_swing_track = front_foot_swing_track
        # 站立位足朝向(ramp 起点), 必须匹配 StandController 的站立 tarsus。
        stand_fp = (
            math.radians(front_stand_foot_pitch_deg)
            if front_stand_foot_pitch_deg is not None
            else None
        )
        self._front_stand_foot_pitch = front_standing_foot_pitch(
            body_height, self.x_offset_front, front_stand_tarsus_deg,
            foot_pitch=stand_fp)
        # 摆动相 roll 低侧额外抬腿: z += low_side_roll * per_rad * body_height
        self.swing_clearance_per_rad = swing_clearance_per_rad
        # 对角 Trot 预期 roll 前馈 (度): FL+RR 支撑负峰 / FR+RL 支撑正峰, 供 ff_decouple 扣除
        self.trot_roll_ff_neg_deg = trot_roll_ff_neg_deg
        self.trot_roll_ff_pos_deg = (
            trot_roll_ff_pos_deg if trot_roll_ff_pos_deg > 1e-6
            else trot_roll_ff_neg_deg * 0.55
        )
        # FL+RR 对角 roll 负峰更大时, 支撑相 asym 缩放 anti_roll (1=对称)
        self.anti_roll_asym_neg = anti_roll_asym_neg
        self.anti_roll_asym_pos = anti_roll_asym_pos
        self._reactive_filtered = 0.0

        # 转向控制 (平滑过渡)
        self._turn_cmd = 0.0
        self._turn_filtered = 0.0
        self.turn_filter_alpha = 0.015  # 时间常数约0.33秒，转向极其柔和"不猛甩"
        self.max_turn_waist_yaw = 0.35  # 腰部扭曲最大角度 (约20度)
        self.max_turn_amp_diff = 0.020  # 转向最大步幅差 2.0cm
        self.max_turn_y_amp    = 0.025  # 转向髋外展跨步幅度 2.5cm
        # 横向跨步(蟹步)增益: 1=原地转(靠横向跨步转身); 0=纯差速旋转(边走边拐弯不横移)
        self.turn_y_gain = 1.0
        # 腰扭转符号(硬件改线后可微调): 1=与腿转向同向(推荐); -1=反向
        self.waist_yaw_turn_sign = 1.0
        # Spot-turn: Unitree continuous trot-turn (SpotYawStepper).
        self.spot_turn_active = False
        self.spot_yaw_step_rad = 0.45  # yaw per cycle at |turn|=1
        self.spot_hip_half_width = 0.075
        self.spot_y_hold_max_m = 0.055
        self.spot_dx_scale = 0.0
        # Spot waist: bias≈23° + pulse≈7° (hw soft-limit ±1.2rad / ±69°).
        self.spot_waist_yaw_rad = 0.40
        self.spot_waist_yaw_pulse_rad = 0.12
        self._PHASE_OFFSET_CRUISE = dict(type(self)._PHASE_OFFSET)
        self._PHASE_OFFSET = dict(self._PHASE_OFFSET_CRUISE)
        from marsdog_control.motion.spot_yaw_step import SpotYawStepConfig, SpotYawStepper
        self._spot = SpotYawStepper(
            cfg=SpotYawStepConfig(y_hold_max_m=self.spot_y_hold_max_m),
            hip_xy=self._spot_hip_xy,
        )
        self._spot_xy_cache = self._spot._xy_cache  # abd / _leg_y_turn read this
        # SI velocity command from SoftTrotSchedule (not reverse-engineered amp).
        self.vel_cmd = (0.0, 0.0, 0.0)
        self.speed_frac = 0.0
        # Sagittal Raibert — Schedule owns amp; placement reads vel_cmd.
        from marsdog_control.motion.sagittal_raibert import attach_raibert
        attach_raibert(
            self,
            enabled=True,
            kx=0.05,
            dx_max=0.03,
            recipe_amp_front=self.amp_front,
            recipe_amp_rear=self.amp_rear,
        )

    @classmethod
    def from_params(cls, params: "GaitParams", **overrides: Any) -> "StableTrot":
        """Build from ``GaitParams`` instead of a long kwargs bag."""
        kw = params.as_stable_trot_kwargs()
        kw.update(overrides)
        return cls(**kw)

    @property
    def turn_cmd(self):
        return self._turn_cmd

    @turn_cmd.setter
    def turn_cmd(self, v):
        self._turn_cmd = _clamp(v, -1.0, 1.0)

    def _swing_z(self, swing_t: float, step_h: float) -> float:
        """三段式摆动 Z: 快起(30%) → 巡航(40%) → 柔落(30%), C1 连续。"""
        return _ft.three_phase_swing_z(swing_t, step_h, self._RISE_END, self._CRUISE_END)

    def _anti_roll_diag_scale(self, t: float) -> float:
        """按对角支撑相缩放 anti_roll: FL+RR 负 roll 大 → 加大伸腿。"""
        return _ft.anti_roll_diag_scale(
            t, self.period, self.stance_ratio,
            self.anti_roll_asym_neg, self.anti_roll_asym_pos)

    def _gated_anti_roll(self) -> float:
        """Soft anti_roll after AttitudeOverlayGate; fail-closed without gate."""
        gate = getattr(self, "_attitude_overlay_gate", None)
        if gate is None:
            return 0.0
        return float(gate.gate_anti_roll(self.anti_roll))

    def _gated_swing_level(self) -> float:
        """IMU prelevel weight — only when AttitudeOwner.IMU; else 0."""
        gate = getattr(self, "_attitude_overlay_gate", None)
        if gate is None:
            return 0.0
        return float(gate.gate_swing_level(self.swing_level))

    def _gated_spine_deg(self) -> tuple:
        """Spine yaw/roll deg after gate (WBC / missing gate → 0)."""
        gate = getattr(self, "_attitude_overlay_gate", None)
        if gate is None:
            return (0.0, 0.0)
        return gate.gate_spine_deg(
            float(self.spine_yaw_deg), float(self.spine_roll_deg))

    def _leg_xz(self, leg: str, t: float, turn: float = 0.0) -> tuple:
        """单腿足端 (x, z_lift), 分段半余弦 X + 三段 Z。

        加入 turn 差速: turn>0(右转) 左外侧步幅大, 右内侧步幅小甚至反向。
        支撑相 lift 含 anti-roll 补偿: 中期腿伸长(负值), 推body远离倾倒方向。
        X 轨迹形状纯几何部分见 `foot_trajectory.stable_trot_x`；摆动相抬腿高度
        走 `self._swing_z`(虚方法, 子类可重写), 保留多态分发。
        """
        phase = (t / self.period + self._PHASE_OFFSET[leg]) % 1.0
        is_front = leg.startswith('f')
        is_left = leg.endswith('l')

        base_amp = self.amp_front if is_front else self.amp_rear

        # Spot: no amp-diff; abduction Y does the turn. Cruise: tank amp-diff.
        if getattr(self, "spot_turn_active", False):
            amp = base_amp
        else:
            turn_amp = turn * self.max_turn_amp_diff
            amp = base_amp + turn_amp if is_left else base_amp - turn_amp

        cx = self.x_offset_front if is_front else self.x_offset_rear
        sh = self.step_height_front if is_front else self.step_height

        x, is_swing, u = _ft.stable_trot_x(
            phase, amp, cx, self.stance_ratio, self.smooth_gait)
        if getattr(self, "spot_turn_active", False):
            dx, _dy = self._spot_foot_xy(leg, t, turn)
            x = cx + dx
            # Unitree SpotYawStepper owns swing (diagonal trot duty).
            is_swing = self._spot.in_swing(leg)
            u = self._spot.swing_progress(leg) if is_swing else 0.0
        if is_swing:
            lift = self._swing_z(u, sh)
        else:
            if getattr(self, "spot_turn_active", False):
                lift = 0.0
            else:
                lift = _ft.stance_anti_roll_lift(
                    u, self._gated_anti_roll(), self._anti_roll_diag_scale(t))
        return x, lift

    def _leg_y_turn(self, leg: str, t: float, turn: float) -> float:
        """返回转向时的 Y 轴跨步偏移量 (正值=向左跨步)。"""
        if getattr(self, "spot_turn_active", False):
            # Cache filled by _leg_xz earlier this tick (avoid double half-step).
            return self._spot_xy_cache.get(leg, (0.0, 0.0))[1]
        phase = (t / self.period + self._PHASE_OFFSET[leg]) % 1.0
        return _ft.leg_y_turn(
            leg, phase, turn, self.stance_ratio,
            self.max_turn_y_amp, self.turn_y_gain)

    def _y_body_to_abd_roll(self, leg: str, y_body: float) -> float:
        """Body-frame foot Y (+left) → abduction joint delta (+ = outward both sides).

        Left foot +Y = abduct; right foot +Y = adduct (−abd). Without this
        side map, the same y command on L/R only changes stance width and
        produces almost no yaw couple.

        ``y_body`` is now a physically-derived ω×r_hip target (SpotYawStepper),
        so the same leg-length lever as cruise sway applies directly — no
        artificial lever shortening needed.
        """
        side = 1.0 if leg.endswith("l") else -1.0
        lever = max(1e-3, float(self.body_height))
        return side * float(y_body) / lever

    def _spot_hip_xy(self, leg: str) -> tuple:
        """Hip position in body frame relative to CoM (for ω×r footholds).

        Must NOT use foot stand x_offset (≈0 under hip) — that kills front dy.
        """
        if leg.startswith("f"):
            hx = float(_COM_TO_FRONT)  # front hip ahead of CoM
            hy = float(_HALF_TRACK_FRONT) if leg.endswith("l") else -float(_HALF_TRACK_FRONT)
        else:
            hx = -float(_COM_TO_REAR)  # rear hip behind CoM
            hy = float(_HALF_TRACK_REAR) if leg.endswith("l") else -float(_HALF_TRACK_REAR)
        return hx, hy

    def _spot_update_pose(self, t: float, imu_state=None) -> None:
        """Pose + one unitree-turn tick. Call once per get_targets."""
        imu_state = imu_state or {}
        yaw = float(imu_state.get("yaw", self._spot.yaw))
        base_xy = None
        vel_xy = None
        if "base_xy" in imu_state:
            bxy = imu_state["base_xy"]
            base_xy = (float(bxy[0]), float(bxy[1]))
        elif "vel_xyz" in imu_state:
            vel_xy = (float(imu_state["vel_xyz"][0]), float(imu_state["vel_xyz"][1]))
        self._spot.cfg.y_hold_max_m = float(getattr(self, "spot_y_hold_max_m", 0.045))
        if hasattr(self, "spot_yaw_step_rad"):
            self._spot.cfg.yaw_step_rad = float(self.spot_yaw_step_rad) or self._spot.cfg.yaw_step_rad
        self._spot.update_pose(t, yaw=yaw, base_xy=base_xy, vel_xy=vel_xy, wz=0.0)
        self._spot.tick(
            t,
            float(self._turn_filtered),
            float(self.period),
            stance_ratio=float(self.stance_ratio),
        )

    def _spot_foot_xy(self, leg: str, t: float, turn: float) -> tuple:
        """Thin adapter → SpotYawStepper cache (tick already ran)."""
        return self._spot.foot_xy(leg, t, turn, self.period)

    def spot_leg_in_swing(self, leg: str) -> bool:
        if not getattr(self, "spot_turn_active", False):
            return False
        return bool(self._spot.in_swing(leg))

    def _spot_abd_from_cache(self, leg: str) -> float:
        """Body-Y cache → abd (both signs — world hold needs adduct too)."""
        y = self._spot.cached_xy(leg)[1]
        return float(self._y_body_to_abd_roll(leg, y))

    def _clear_spot_state(self) -> None:
        self._spot.reset()

    def get_spot_com_shift(self, t: float) -> tuple:
        """Body-frame CoM shift into support triangle. Executor / sway only."""
        if not getattr(self, "spot_turn_active", False):
            return (0.0, 0.0)
        xy = self._spot.com_shift_xy(
            t, self.period, self.stance_ratio, self._PHASE_OFFSET)
        planner = getattr(self, "_lateral_planner", None)
        if planner is None:
            return (0.0, 0.0)
        return planner.gate_spot_com(xy)

    def _lateral_offset(self, t: float) -> float:
        """横向 CoM 偏移。

        Spot: kinematic sway off (Spot CoM via executor only).
        Cruise: diagonal trot sway when LateralOwner.SWAY.
        Fail-closed when no LateralPlanner is bound.
        """
        if getattr(self, "spot_turn_active", False):
            # CoM unload via executor MPC/base_acc only — abd sway on top of
            # world-hold twist was stacking and tipping in catch.
            return 0.0
        planner = getattr(self, "_lateral_planner", None)
        if planner is None:
            return 0.0
        planner.sync_from_gait(self)
        return planner.trot_sway_kinematic(
            t, self.period, self.stance_ratio, self.lateral_sway)

    def _expected_diagonal_roll(self, t: float) -> float:
        """对角 Trot 支撑引起的预期 roll (度), 与 sway 无关。

        FL+RR 支撑 (phase 0~sr): 负 roll; FR+RL 支撑 (phase 0.5~0.5+sr): 正 roll。
        半正弦包络, 支撑中期达峰 — 匹配无 IMU 时测得的 ~2×步频摆动。
        """
        gate = getattr(self, "_attitude_overlay_gate", None)
        if gate is None:
            return 0.0
        neg, pos = gate.gate_roll_ff_deg(
            self.trot_roll_ff_neg_deg, self.trot_roll_ff_pos_deg)
        return _ft.expected_diagonal_roll(
            t, self.period, self.stance_ratio, neg, pos)

    def get_expected_roll(self, t: float) -> float:
        """预期身体侧倾角 (度): lateral_sway 项 + 对角 Trot 动力学项。

        lat_offset > 0 → 左侧变低 → IMU roll 为负。
        ff_decouple 扣除后 IMU 只修残差, 避免与步态周期对着干。
        """
        delay = 0.06
        t_delayed = max(0.0, t - delay)
        lat_offset = self._lateral_offset(t_delayed)
        roll_sway = math.degrees(-lat_offset / self.body_height)
        return roll_sway + self._expected_diagonal_roll(t_delayed)

    def get_expected_pitch(self, t: float) -> float:
        """预期身体俯仰角 (度)。StableTrot 步态本身不做动态俯仰, 返回 0。
        预留前馈钩子: 若将来加入动态前倾, 在此返回预期 pitch 供 walk.py 扣除。
        """
        return 0.0

    _STANCE_TAPER = 0.06   # 支撑相两端平滑过渡比例 (相位)

    def _stance_weight(self, phase: float) -> float:
        """支撑相平滑门控权重 [0,1]: 支撑腿(踩地)接受 IMU Z 修正。

        摆动腿腾空无法推动身体, 但给摆动腿落脚点一点预调平(swing_level)可让它
        按纠正后的姿态落地, 消除"摆动→支撑"切换时纠正突然出现的顿挫(开源做法)。
        swing_level=0 时=原行为(仅支撑腿); >0 时摆动腿获得该比例的纠正, 用 max 保持连续。
        """
        return _ft.stance_weight(
            phase, self.stance_ratio, self._gated_swing_level(), self._STANCE_TAPER)

    def _foot_track_gate(self, phase: float) -> float:
        """足朝向跟踪门控 [floor,1]: 支撑相=1(脚尖指地), 摆动相=front_foot_swing_track。"""
        return _ft.foot_track_gate(
            phase, self.stance_ratio, self.front_foot_swing_track, self._STANCE_TAPER)

    def get_targets(self, t: float, imu_dz: dict = None,
                    imu_state: dict = None) -> dict:
        """生成步态目标。"""
        from marsdog_control.motion.trot_tick import (
            apply_trot_waist,
            prepare_stable_trot_tick,
        )
        from marsdog_control.motion.foot_geometry_engine import (
            solve_front_legs,
            solve_rear_legs,
        )

        targets = self._zero_targets()
        prep = prepare_stable_trot_tick(
            self,
            t,
            imu_state=imu_state,
            front_hip_offset=_FRONT_HIP_OFFSET,
            rear_hip_offset=_REAR_HIP_OFFSET,
            clamp=_clamp,
        )

        # 前腿标称姿态 (推力放大基准): hip 偏离此姿态被放大, calf 仅解算 Z
        hip0_f, _calf0_f = ik_front_leg_2d(
            self.x_offset_front, prep.z_front_base)

        solve_front_legs(
            self,
            targets,
            t=t,
            ramp=prep.ramp,
            lat_offset=prep.lat_offset,
            reactive=prep.reactive,
            dx_raibert=prep.dx_raibert,
            z_front_base=prep.z_front_base,
            hip0_f=hip0_f,
            imu_dz=imu_dz,
            imu_state=imu_state,
        )
        solve_rear_legs(
            self,
            targets,
            t=t,
            ramp=prep.ramp,
            lat_offset=prep.lat_offset,
            reactive=prep.reactive,
            dx_raibert=prep.dx_raibert,
            z_rear_base=prep.z_rear_base,
            imu_dz=imu_dz,
        )
        apply_trot_waist(
            self,
            targets,
            t=t,
            ramp=prep.ramp,
            joint_by_name=JOINT_BY_NAME,
            clamp=_clamp,
        )
        return targets

    def set_period(self, p: float):
        self.period = max(0.2, min(2.0, p))

    def set_height(self, h: float):
        self.body_height = h

    def describe(self) -> str:
        roll_ff = ""
        if self.trot_roll_ff_neg_deg > 1e-6:
            roll_ff = (
                f"  roll_ff={self.trot_roll_ff_neg_deg:.1f}/"
                f"{self.trot_roll_ff_pos_deg:.1f}°"
            )
        return (f"STABLE_TROT  T={self.period:.2f}s  "
                f"H={self.body_height:.3f}m  "
                f"amp(F/R)={self.amp_front*100:.1f}/{self.amp_rear*100:.1f}cm  "
                f"lift(F/R)={self.step_height_front*100:.1f}/{self.step_height*100:.1f}cm  "
                f"stance={self.stance_ratio:.0%}  "
                f"sway={self.lateral_sway*1000:.0f}mm  "
                f"anti_roll={self.anti_roll*1000:.0f}mm"
                f"{roll_ff}  "
                f"react_kp={self.reactive_kp:.2f}")


class StablePace(StableTrot):
    """稳定 Pace (溜步/同侧步) — 左侧前后腿同起同落，右侧前后腿同起同落。

    继承自 StableTrot，重写 _lateral_offset 以适配 Pace 时序。
    Pace 步态需要横向重心转移 (lateral_sway) 才能保持平衡。
    相位: FL+RL=0.0, FR+RR=0.5
    """
    family = "pace"
    _PHASE_OFFSET = {'fl': 0.0, 'rl': 0.0, 'fr': 0.5, 'rr': 0.5}

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.family = "pace"

    def _lateral_offset(self, t: float) -> float:
        """Pace 专用横向偏移 — 使用全周期余弦确保抬腿瞬间已有重心转移。

        FR+RR 单支撑中点: phase = sr/2
        FL+RL 单支撑中点: phase = (sr+1)/2
        在两对腿抬起瞬间，重心已转移 ≥71% 到支撑侧。
        """
        planner = getattr(self, "_lateral_planner", None)
        if planner is None:
            return 0.0
        planner.sync_from_gait(self)
        raw = _ft.lateral_offset_pace(
            t, self.period, self.stance_ratio, self.lateral_sway)
        return planner.gate_kinematic(raw)


