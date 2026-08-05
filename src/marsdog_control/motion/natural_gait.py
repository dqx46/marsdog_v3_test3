"""Natural trot / soft trot / walk gait controllers."""
import math
from typing import Any, Optional

from marsdog_control.motion.kinematics import (
    compute_standing_pose_3link,
    front_thigh_roll_abd_urdf,
    ik_front_leg_2d, ik_rear_leg_2d,
    ik_front_3link, front_tarsus_schedule, solve_front_calf_for_z_3link,
    ik_front_3link_foot_orient, front_standing_foot_pitch,
    fk_front_2d, fk_rear_2d,
    WAIST_Z, FL_HIP_Z, RL_HIP_Z,
)
from marsdog_control.config.joints import JOINT_BY_NAME
from marsdog_control.motion import foot_trajectory as _ft
from marsdog_control.motion.gait_base import (
    _FRONT_HIP_OFFSET,
    _REAR_HIP_OFFSET,
    _FRONT_X0,
    _REAR_X0,
    _clamp,
    _cmd,
)
from marsdog_control.motion.stable_trot import StableTrot

class NaturalTrot(StableTrot):
    """自然 Trot (真狗小跑姿态) — 通过轨迹形状让 IK 自然产生仿生关节运动。

    核心思路: 不靠额外力矩/delta 强扭关节, 而是设计足端轨迹让 IK 几何解
    本身就产生真狗般的关节角度变化。电机只需跟踪 IK 计算的位置即可。

    仿生效果 (由轨迹几何自然产生):
      1. 膝关节折叠: 摆动相足端先回缩(靠近hip)再前伸 → IK 自然弯膝/伸膝
      2. 水滴形轨迹: sin Z弧 + 回缩X → 足端画出椭圆/水滴, 无硬角
      3. 脊柱对角律动: waist_yaw/roll 周期性反扭
      4. 跗关节收放(仅前腿): 摆动相脚尖翻起, 落地前脚尖朝下

    稳定内核(相位/落脚/anti-roll/IMU)完全继承 StableTrot。
    """

    def __init__(self, *args,
                 spine_yaw_deg: float = 3.0,
                 spine_roll_deg: float = 1.5,
                 spine_phase_deg: float = 0.0,
                 spine_roll_phase_deg: float = None,
                 thigh_swing_front_deg: float = 0.0,
                 thigh_swing_rear_deg: float = 0.0,
                 retract_front: float = 0.035,
                 retract_rear: float = 0.025,
                 tarsus_swing_deg: float = 12.0,
                 **kwargs):
        super().__init__(*args, **kwargs)
        self.spine_yaw_deg = spine_yaw_deg
        self.spine_roll_deg = spine_roll_deg
        self.spine_phase_deg = spine_phase_deg
        self.spine_roll_phase_deg = (
            spine_roll_phase_deg if spine_roll_phase_deg is not None
            else spine_phase_deg
        )
        self.thigh_swing_front_deg = thigh_swing_front_deg
        self.thigh_swing_rear_deg = thigh_swing_rear_deg
        # 层1+2: 摆动相足端 X 回缩量 (m) — 中点最大回缩, 让 IK 自然折膝
        self.retract_front = retract_front
        self.retract_rear = retract_rear
        # 层5: 前腿跗关节摆动相收放幅度 (度)
        self.tarsus_swing_deg = tarsus_swing_deg

    @classmethod
    def from_params(
        cls,
        params: "GaitParams",
        extras: Optional["NaturalExtras"] = None,
        **overrides: Any,
    ) -> "NaturalTrot":
        from marsdog_control.motion.gait_params import NaturalExtras as _NE
        kw = params.as_stable_trot_kwargs()
        kw.update((extras or _NE()).as_kwargs())
        kw.update(overrides)
        return cls(**kw)

    def _spine_ramp(self, t: float) -> float:
        """脊柱律动软启动。"""
        from marsdog_control.motion.spine_overlay import spine_ramp
        return spine_ramp(t, float(self.ramp_duration))

    def _swing_z(self, swing_t: float, step_h: float) -> float:
        """C1 摆动 Z 轨迹。

        旧版 sin(pi*t) 在起落脚端竖直速度不为 0，视觉上会像把脚砸到地面。
        sin² 保持同样峰值，但起脚/落脚端速度为 0，更接近软触地。
        """
        return _ft.sin2_swing_z(swing_t, step_h)

    def _leg_xz(self, leg: str, t: float, turn: float = 0.0) -> tuple:
        """仿生足端轨迹: 回缩弧线让 IK 自然产生膝折叠。

        摆动相核心设计: X = 前进运动 - 回缩弧线; Z = self._swing_z(虚方法)。
        回缩弧线 (Hann 窗): 摆动中点足端 X 大幅回缩(靠近 hip 正下方), 使
        hip-to-foot 距离缩短 → IK 几何解自动产生大膝弯角。纯几何部分见
        `foot_trajectory.natural_trot_x`。
        """
        phase = (t / self.period + self._PHASE_OFFSET[leg]) % 1.0
        is_front = leg.startswith('f')
        is_left = leg.endswith('l')

        base_amp = self.amp_front if is_front else self.amp_rear
        if getattr(self, "spot_turn_active", False):
            amp = base_amp
        else:
            turn_amp = turn * self.max_turn_amp_diff
            amp = base_amp + turn_amp if is_left else base_amp - turn_amp

        cx = self.x_offset_front if is_front else self.x_offset_rear
        sh = self.step_height_front if is_front else self.step_height
        retract = self.retract_front if is_front else self.retract_rear
        if getattr(self, "spot_turn_active", False):
            retract = 0.0

        x, is_swing, u = _ft.natural_trot_x(phase, amp, cx, self.stance_ratio, retract)
        if getattr(self, "spot_turn_active", False):
            dx, _dy = self._spot_foot_xy(leg, t, turn)
            x = cx + dx
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

    def _swing_flourish(self, leg: str, t: float) -> float:
        """摆动相大腿 Hann 窗偏置(URDF rad), 支撑相为零。"""
        phase = (t / self.period + self._PHASE_OFFSET[leg]) % 1.0
        return _ft.swing_flourish_hann(
            leg, phase, self.stance_ratio,
            self.thigh_swing_front_deg, self.thigh_swing_rear_deg)

    def _tarsus_swing_delta(self, leg: str, t: float) -> float:
        """前腿跗关节摆动相收放 (URDF rad 增量), 仅前腿。

        摆动前70%: 跗关节微收(脚尖抬起, "翻爪")
        摆动后30%: 跗关节伸展回零(脚尖朝下准备触地)
        """
        phase = (t / self.period + self._PHASE_OFFSET[leg]) % 1.0
        return _ft.tarsus_swing_delta_hann(leg, phase, self.stance_ratio, self.tarsus_swing_deg)

    def get_targets(self, t: float, imu_dz: dict = None,
                    imu_state: dict = None) -> dict:
        targets = super().get_targets(t, imu_dz, imu_state)

        from marsdog_control.motion.spine_overlay import (
            apply_spine_osc_to_targets,
            spine_yaw_roll_osc,
        )
        ramp = self._spine_ramp(t)
        spine_yaw_deg, spine_roll_deg = self._gated_spine_deg()
        yaw_osc, roll_osc = spine_yaw_roll_osc(
            t=t,
            period=float(self.period),
            ramp=ramp,
            spine_yaw_deg=spine_yaw_deg,
            spine_roll_deg=spine_roll_deg,
            spine_phase_deg=float(self.spine_phase_deg),
            spine_roll_phase_deg=float(self.spine_roll_phase_deg),
            spot_turn_active=bool(getattr(self, "spot_turn_active", False)),
        )
        apply_spine_osc_to_targets(
            targets,
            yaw_osc=yaw_osc,
            roll_osc=roll_osc,
            joint_by_name=JOINT_BY_NAME,
            clamp=_clamp,
        )

        # ── 大腿摆动 flourish ──
        if not getattr(self, "spot_turn_active", False):
            for leg, joint_name in (
                ("fl", "fl_hip_pitch"),
                ("fr", "fr_hip_pitch"),
                ("rl", "rl_thigh"),
                ("rr", "rr_thigh"),
            ):
                # 前脚启用三关节足朝向 IK 时，hip/calf/tarsus 已由 (x,z,foot_pitch)
                # 唯一确定。IK 后再单独叠加前腿 hip flourish 会直接破坏足段朝向。
                if leg.startswith("f") and self.front_foot_track_deg is not None:
                    continue
                delta_u = self._swing_flourish(leg, t) * ramp
                if abs(delta_u) < 1e-9:
                    continue
                j = JOINT_BY_NAME[joint_name]
                targets[j.motor_id] = targets.get(j.motor_id, 0.0) + delta_u

        # ── 层5: 前腿跗关节收放 ──
        if not getattr(self, "spot_turn_active", False):
            for leg, tarsus_name in (("fl", "fl_tarsus"), ("fr", "fr_tarsus")):
                # 同上：足朝向跟踪开启时禁止在 IK 后独立改 tarsus。
                if self.front_foot_track_deg is not None:
                    continue
                delta_tar = self._tarsus_swing_delta(leg, t) * ramp
                if abs(delta_tar) < 1e-9:
                    continue
                j = JOINT_BY_NAME[tarsus_name]
                targets[j.motor_id] = targets.get(j.motor_id, 0.0) + delta_tar

        return targets

    def get_expected_roll(self, t: float) -> float:
        """预期身体侧倾角 (度): 继承基类 lateral_sway 项 + 脊柱 roll 律动贡献。

        NaturalTrot 的 waist_roll 周期性摆动会导致身体轻微侧倾,
        若不告知 IMU 这是"预期内"的, IMU PID 会对抗脊柱律动。
        """
        from marsdog_control.motion.spine_overlay import expected_spine_roll_deg
        base = super().get_expected_roll(t)
        _, spine_roll_deg = self._gated_spine_deg()
        return base + expected_spine_roll_deg(
            t=t,
            period=float(self.period),
            ramp=self._spine_ramp(t),
            spine_roll_deg=spine_roll_deg,
            spine_roll_phase_deg=float(self.spine_roll_phase_deg),
        )

    def describe(self) -> str:
        base = super().describe().replace("STABLE_TROT", "NATURAL_TROT")
        return (base
                + f"  spine(yaw/roll)={self.spine_yaw_deg:.1f}/"
                f"{self.spine_roll_deg:.1f}°"
                + f"  thigh_swing(F/R)={self.thigh_swing_front_deg:.1f}/"
                f"{self.thigh_swing_rear_deg:.1f}°"
                + f"  retract(F/R)={self.retract_front*100:.1f}/"
                f"{self.retract_rear*100:.1f}cm")


class NaturalSoftTrot(NaturalTrot):
    """低冲击 NaturalTrot。

    目标不是先追求夸张视觉，而是降低触地瞬间的速度/加速度突变:
      1. 对角腿轻微错相，避免两条腿同时砸地。
      2. stance/swing 都使用 minimum-jerk 轨迹，端点速度/加速度为 0。
      3. 触地初期先微缩腿吸震，再温和承重，不用 anti_roll 硬压腿长。
      4. 前腿 tarsus 翻爪改成整段 C2 小幅收放，落地前回到 0。
      5. 位控层横向质心规划 (com_shift_m>0)：换腿前把 CoM 压到支撑对角侧。
    """

    _PHASE_OFFSET = {'fl': 0.00, 'rr': 0.06, 'fr': 0.50, 'rl': 0.56}
    # 支撑相两端门控：0.12 偏“顿”，0.06 偏“震”；0.10 折中
    _STANCE_TAPER = 0.10

    def __init__(self, *args,
                 touchdown_compress: float = 0.0035,
                 anti_roll_soft_scale: float = 0.35,
                 toeoff_lift: float = 0.003,
                 retract_peak: float = 0.38,
                 lift_peak: float = 0.45,
                 rear_clearance_m: float = 0.0,
                 com_shift_m: float = 0.0,
                 com_shift_blend: float = 0.15,
                 raibert_enabled: bool = True,
                 raibert_kx: float = 0.05,
                 raibert_dx_max: float = 0.03,
                 **kwargs):
        super().__init__(*args, **kwargs)
        self.touchdown_compress = touchdown_compress
        self.anti_roll_soft_scale = anti_roll_soft_scale
        self.toeoff_lift = toeoff_lift
        self.retract_peak = retract_peak
        self.lift_peak = lift_peak
        # Raise rear hip-frame Z so feet clear ground (rear hip ~16mm lower than front).
        self.rear_clearance_m = float(rear_clearance_m)
        # Lateral CoM / weight-shift (m). >0 replaces half-sine lateral_sway.
        self.com_shift_m = float(com_shift_m)
        self.com_shift_blend = float(com_shift_blend)
        from marsdog_control.motion.sagittal_raibert import attach_raibert
        attach_raibert(
            self,
            enabled=bool(raibert_enabled),
            kx=float(raibert_kx),
            dx_max=float(raibert_dx_max),
            recipe_amp_front=self.amp_front,
            recipe_amp_rear=self.amp_rear,
        )

    def _update_raibert_placement(self, imu_state=None) -> float:
        """Test / Soft hook: refresh sagittal Raibert from SI vel_cmd."""
        from marsdog_control.motion.sagittal_raibert import update_raibert_from_imu
        return update_raibert_from_imu(self, imu_state)

    @property
    def _raibert_dx(self) -> float:
        rb = getattr(self, "_raibert", None)
        return float(rb.dx) if rb is not None else 0.0

    @property
    def _raibert_use_amp(self) -> bool:
        rb = getattr(self, "_raibert", None)
        return bool(rb.use_amp) if rb is not None else False

    @property
    def _raibert_amp_front(self) -> float:
        rb = getattr(self, "_raibert", None)
        return float(rb.amp_front) if rb is not None else 0.0

    @property
    def _raibert_amp_rear(self) -> float:
        rb = getattr(self, "_raibert", None)
        return float(rb.amp_rear) if rb is not None else 0.0

    @classmethod
    def from_build(cls, build: "SoftTrotBuild", **overrides: Any) -> "NaturalSoftTrot":
        """Preferred SoftTrot construction path (one typed bundle)."""
        kw = build.as_kwargs()
        kw.update(overrides)
        return cls(**kw)

    @classmethod
    def from_params(
        cls,
        params: "GaitParams",
        extras: Optional["NaturalExtras"] = None,
        soft: Optional["SoftExtras"] = None,
        **overrides: Any,
    ) -> "NaturalSoftTrot":
        from marsdog_control.motion.gait_params import (
            NaturalExtras as _NE, SoftExtras as _SE, SoftTrotBuild,
        )
        return cls.from_build(
            SoftTrotBuild(params, extras or _NE(), soft or _SE()),
            **overrides,
        )

    def _lateral_offset(self, t: float) -> float:
        """SoftTrot 横向移重：按 LateralOwner 单选 com_shift 或 sway（禁止双回退）。"""
        if getattr(self, "spot_turn_active", False):
            return 0.0
        planner = getattr(self, "_lateral_planner", None)
        if planner is None:
            return 0.0
        planner.sync_from_gait(self)
        return planner.soft_kinematic(
            t, self.period, self.stance_ratio,
            com_shift_m=self.com_shift_m,
            com_shift_blend=self.com_shift_blend,
            lateral_sway=self.lateral_sway,
        )

    def get_com_y_shift(self, t: float) -> float:
        """Body +Y CoM shift for logging / future MPC reference."""
        return self._lateral_offset(t)

    @staticmethod
    def _mj(u: float) -> float:
        return _ft.minimum_jerk(u)

    @classmethod
    def _mj_bump(cls, u: float, peak: float = 0.5) -> float:
        return _ft.minimum_jerk_bump(u, peak)

    def _swing_z(self, swing_t: float, step_h: float) -> float:
        return _ft.minimum_jerk_swing_z(swing_t, step_h, self.lift_peak)

    def _leg_xz(self, leg: str, t: float, turn: float = 0.0) -> tuple:
        phase = (t / self.period + self._PHASE_OFFSET[leg]) % 1.0
        is_front = leg.startswith('f')
        is_left = leg.endswith('l')

        base_amp = self.amp_front if is_front else self.amp_rear
        if getattr(self, "_raibert_use_amp", False):
            # Schedule idle-amp + nonzero SI cmd → revive recipe amp for X.
            rb_amp = (
                self._raibert_amp_front if is_front else self._raibert_amp_rear
            )
            if abs(self.amp_front) + abs(self.amp_rear) > 1e-9:
                sign = 1.0 if base_amp >= 0.0 else -1.0
            else:
                vx = float(getattr(self, "vel_cmd", (0.0, 0.0, 0.0))[0])
                sign = 1.0 if vx >= 0.0 else -1.0
            base_amp = sign * abs(float(rb_amp))
        if getattr(self, "spot_turn_active", False):
            amp = base_amp
        else:
            turn_amp = turn * self.max_turn_amp_diff
            amp = base_amp + turn_amp if is_left else base_amp - turn_amp

        cx = self.x_offset_front if is_front else self.x_offset_rear
        sh = self.step_height_front if is_front else self.step_height
        retract = self.retract_front if is_front else self.retract_rear
        if getattr(self, "spot_turn_active", False):
            retract = 0.0  # pure abduction placement — no fore-aft shuffle

        x, is_swing, u = _ft.natural_soft_trot_x(
            phase, amp, cx, self.stance_ratio, retract, self.retract_peak)
        if getattr(self, "spot_turn_active", False):
            dx, _dy = self._spot_foot_xy(leg, t, turn)
            x = cx + dx
            is_swing = self._spot.in_swing(leg)
            u = self._spot.swing_progress(leg) if is_swing else 0.0
        if is_swing:
            lift = self._swing_z(u, sh)
            if getattr(self, "spot_turn_active", False):
                # Same clearance budget front/rear so diagonal catch looks paired.
                lift += 0.014 * self._mj_bump(u, self.lift_peak)
            elif not is_front and self.rear_clearance_m > 0.0:
                lift += self.rear_clearance_m * self._mj_bump(u, self.lift_peak)
        else:
            if getattr(self, "spot_turn_active", False):
                lift = 0.0  # plant at nominal height; roll reach-down in get_targets
            else:
                lift = _ft.natural_soft_trot_stance_lift(
                    u, self._gated_anti_roll(), self.anti_roll_soft_scale,
                    self._anti_roll_diag_scale(t),
                    self.touchdown_compress, self.toeoff_lift)
        return x, lift

    def _swing_flourish(self, leg: str, t: float) -> float:
        phase = (t / self.period + self._PHASE_OFFSET[leg]) % 1.0
        return _ft.swing_flourish_mj(
            leg, phase, self.stance_ratio,
            self.thigh_swing_front_deg, self.thigh_swing_rear_deg, peak=0.45)

    def _tarsus_swing_delta(self, leg: str, t: float) -> float:
        phase = (t / self.period + self._PHASE_OFFSET[leg]) % 1.0
        return _ft.tarsus_swing_delta_mj(
            leg, phase, self.stance_ratio, self.tarsus_swing_deg, peak=0.42)

    def describe(self) -> str:
        return super().describe().replace("NATURAL_TROT", "NATURAL_SOFT_TROT")


# Four-beat dog walk: lift order LH→LF→RH→RF (rl→fl→rr→fr), ~0.25 apart.
# Offsets for this codebase: swing when phase >= stance; with stance≈0.75,
# lift at (stance - offset) mod 1 → rl@0, fl@0.25, rr@0.50, fr@0.75.
WALK_PHASE_OFFSET = {"fl": 0.50, "fr": 0.00, "rl": 0.75, "rr": 0.25}


class NaturalWalk(NaturalSoftTrot):
    """真狗四拍慢走 — 与 SoftTrot/Spot 解耦的独立家族。

    - 相位四拍错开（LH→LF→RH→RF），高 duty → 多数时间三足支撑
    - 自有足端 X/Z（匀速支撑 + 猫步摆动），不用 SoftTrot MJ 支撑相
    - 事件型 lateral sway + spine；MPC CoM-Y 与抬腿窗口同步
    - 永不启用 spot_turn（原地转仍走 SoftTrot）
    """

    family = "walk"
    _PHASE_OFFSET = dict(WALK_PHASE_OFFSET)

    def __init__(self, *args, com_sway_m: float = None, **kwargs):
        super().__init__(*args, **kwargs)
        self.family = "walk"
        self.spot_turn_active = False
        self._PHASE_OFFSET = dict(WALK_PHASE_OFFSET)
        self._PHASE_OFFSET_CRUISE = dict(WALK_PHASE_OFFSET)
        self.com_sway_m = float(
            com_sway_m if com_sway_m is not None
            else max(float(self.lateral_sway) * 1.4, float(self.lateral_sway))
        )

    def _lateral_offset(self, t: float) -> float:
        planner = getattr(self, "_lateral_planner", None)
        if planner is None:
            return 0.0
        planner.sync_from_gait(self)
        raw = _ft.lateral_offset_walk(t, self.period, self.lateral_sway)
        return planner.gate_kinematic(raw)

    def get_com_y_shift(self, t: float) -> float:
        """Body-frame CoM Y for SRB-MPC (+Y = left), owned by WALK_COM."""
        planner = getattr(self, "_lateral_planner", None)
        if planner is None:
            return 0.0
        planner.sync_from_gait(self)
        raw = _ft.walk_com_y_shift(t, self.period, self.com_sway_m)
        return planner.gate_force_y(raw)

    def _expected_diagonal_roll(self, t: float) -> float:
        return 0.0

    def _swing_z(self, swing_t: float, step_h: float) -> float:
        return _ft.natural_walk_swing_z(swing_t, step_h, self.lift_peak)

    def _leg_xz(self, leg: str, t: float, turn: float = 0.0) -> tuple:
        """Walk foot XZ — never SoftTrot MJ stance; spot never active."""
        self.spot_turn_active = False
        phase = (t / self.period + self._PHASE_OFFSET[leg]) % 1.0
        is_front = leg.startswith("f")
        is_left = leg.endswith("l")

        base_amp = self.amp_front if is_front else self.amp_rear
        turn_amp = turn * self.max_turn_amp_diff
        amp = base_amp + turn_amp if is_left else base_amp - turn_amp

        cx = self.x_offset_front if is_front else self.x_offset_rear
        sh = self.step_height_front if is_front else self.step_height
        retract = self.retract_front if is_front else self.retract_rear

        x, is_swing, u = _ft.natural_walk_x(
            phase, amp, cx, self.stance_ratio, retract, self.retract_peak)
        if is_swing:
            lift = self._swing_z(u, sh)
            if not is_front and self.rear_clearance_m > 0.0:
                lift += self.rear_clearance_m * _ft.minimum_jerk_bump(
                    u, self.lift_peak)
        else:
            lift = _ft.natural_walk_stance_lift(
                u, self.touchdown_compress, self.toeoff_lift)
        return x, lift

    def get_targets(self, t: float, imu_dz: dict = None,
                    imu_state: dict = None) -> dict:
        self.spot_turn_active = False
        return super().get_targets(t, imu_dz, imu_state)

    def describe(self) -> str:
        return super().describe().replace("NATURAL_SOFT_TROT", "NATURAL_WALK")


