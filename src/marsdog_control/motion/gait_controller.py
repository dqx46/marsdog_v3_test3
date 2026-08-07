"""Marsdog 步态控制器 — 站立 + 动态对角 Trot

基于 URDF 质量分布的动态步态规划:
  1. 三次 Bezier 摆动轨迹 — 快起巡航柔降，首尾零速
  2. 横向 CoM 摆动 — 利用 thigh_roll/hip_roll 让重心偏向支撑对角线
  3. body pitch 前倾 — 前进时微前倾补偿惯性
  4. 重力补偿力矩前馈 — 基于 URDF 腿段质量
  5. 全周期余弦 X 轨迹 — C-inf 连续无冲击

URDF 关键参数 (marsdog.urdf):
  总质量: 10.07 kg
  轴距 (前后 hip): 31.4 cm
  后腿横向间距: 6.8 cm, 前腿: 8.0 cm
  前腿: L1=7.0cm, L2=17.4cm
  后腿: L1=18.2cm, L2=8.0cm
"""

# [解耦] 真实实现已从 mocap_to_real 下沉到此 src 模块; 保持与旧代码逐字一致的扁平
# import, 由 ensure_legacy_path() 保证 mocap_to_real 在 sys.path 上可解析(其 compat
# 别名回指本 src 包, 单一模块实体, 不产生第二份副本)。
from marsdog_control.compat import ensure_legacy_path as _ensure_legacy_path
_ensure_legacy_path()

import math
from enum import Enum
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

_FRONT_HIP_OFFSET = abs(WAIST_Z + FL_HIP_Z)   # 0.031 m
_REAR_HIP_OFFSET  = abs(RL_HIP_Z)              # 0.015 m

_FRONT_X0 = fk_front_2d(0.0, 0.0)[0]
_REAR_X0  = fk_rear_2d(0.0, 0.0)[0]

# [P1] 外展 A/B: True 时仅 fl_thigh_roll 回退错误方向 (见 kinematics.ABD_LEGACY)
ABD_LEGACY = False

# [P2] 摆动腿 IMU 预调平权重 0~1 (由 walk.py --swing-level 设置, 默认0=仅支撑腿)
SWING_LEVEL = 0.0

# [C] 平滑步态: 支撑相 X 匀速(身体匀速前进,消除"一冲一冲") + 摆动相 Hermite 端点速度匹配
# (全周期 C1 连续, 落地水平速度=支撑速度不打滑)。默认 False=余弦(旧行为), 由 walk.py --smooth-gait 开启。
SMOOTH_GAIT = False


def _smoothstep(a: float, b: float, x: float) -> float:
    """在 [a,b] 上从 0 平滑升到 1 (C1), 区间外取端值。"""
    if b <= a:
        return 1.0 if x >= b else 0.0
    t = (x - a) / (b - a)
    if t <= 0.0:
        return 0.0
    if t >= 1.0:
        return 1.0
    return t * t * (3.0 - 2.0 * t)


def kp_phase_scale(phase: float, stance_ratio: float,
                   td_scale: float = 0.4, swing_scale: float = 0.7,
                   td_window: float = 0.15, liftoff_window: float = 0.06) -> float:
    """[A] 相位可变阻抗: 返回该相位下腿部 kp 的缩放系数。

    相位定义(StableTrot): phase∈[0,sr] 支撑, [sr,1) 摆动; 触地发生在 phase=0。
    曲线(全程 C1 连续, 用 smoothstep 避免 kp 突跳产生新冲击):
      触地窗口(摆动末→支撑初, 跨 phase=0): 降到 td_scale 软着陆吸震
      支撑中后期: 1.0 全力撑体重
      离地过渡→摆动: 平滑降到 swing_scale (腾空中等刚度)
    """
    phase %= 1.0
    sr = stance_ratio
    w = td_window
    wo = liftoff_window
    if phase < w:
        # 支撑初: 触地软化恢复 td_scale → 1.0
        return td_scale + (1.0 - td_scale) * _smoothstep(0.0, w, phase)
    elif phase < sr - wo:
        return 1.0
    elif phase < sr:
        # 离地过渡: 1.0 → swing_scale
        return 1.0 + (swing_scale - 1.0) * _smoothstep(sr - wo, sr, phase)
    elif phase < 1.0 - w:
        return swing_scale
    else:
        # 触地预备: swing_scale → td_scale (预软化)
        return swing_scale + (td_scale - swing_scale) * _smoothstep(1.0 - w, 1.0, phase)

# URDF 精确几何/质量参数 (从 marsdog.urdf 解析)
_ROBOT_MASS = 10.07          # kg
_HALF_TRACK_REAR = 0.034     # 后腿半横向间距 (m)
_HALF_TRACK_FRONT = 0.040    # 前腿半横向间距 (m)
_WHEELBASE = 0.314           # 轴距 (m)

# CoM 相对于 base_link (站立姿态, 所有关节=0)
_COM_X = 0.102               # 10.2cm 前方
_COM_Y = 0.0004              # 基本在中线

# CoM 相对于四脚支撑中心: -1.29cm (偏后)
_COM_BEHIND_SUPPORT = 0.013  # CoM 在支撑中心后方的距离

# 后腿 hip X = -4.17cm, 前腿 hip X = +27.14cm (相对 base_link)
_REAR_HIP_X = -0.0417
_FRONT_HIP_X = 0.2714
# CoM 到后腿 hip 的力臂: 14.37cm, 到前腿 hip 的力臂: 16.94cm
_COM_TO_REAR = 0.1437
_COM_TO_FRONT = 0.1694


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _cmd(jname: str, urdf_angle: float) -> tuple:
    j = JOINT_BY_NAME[jname]
    return j.motor_id, urdf_angle



class GaitController:
    """步态控制器基类。"""

    _ZERO_JOINTS = [
        "waist_roll",
        "fl_thigh_roll", "fr_thigh_roll",
        "fl_tarsus", "fr_tarsus",
        "rl_hip", "rr_hip",
        "head_pitch", "head_yaw", "head_roll", "neck_pitch",
    ]

    waist_pitch_offset: float = 0.0
    waist_yaw_offset: float = 0.0

    def _zero_targets(self) -> dict:
        t = {}
        for name in self._ZERO_JOINTS:
            j = JOINT_BY_NAME[name]
            t[j.motor_id] = 0.0
        j = JOINT_BY_NAME["waist_pitch"]
        t[j.motor_id] = self.waist_pitch_offset
        j = JOINT_BY_NAME["waist_yaw"]
        t[j.motor_id] = self.waist_yaw_offset
        return t

    def get_targets(self, t: float) -> dict:
        raise NotImplementedError

    def describe(self) -> str:
        return self.__class__.__name__


class StandController(GaitController):
    """静止站立姿态。"""

    # 2026-07-11: 老两连杆站姿已删除, 只剩新三连杆带前腿主动 tarsus 站姿。
    # 站立脚段默认绝对朝向 -90° (竖直指地); use_tarsus 恒为 True, 仅保留形参兼容旧调用。
    _DEFAULT_FOOT_PITCH_DEG = -90.0

    def __init__(self, body_height: float = 0.24,
                 x_offset_front: float = None, x_offset_rear: float = None,
                 hip_abduction: float = 0.05,
                 use_tarsus: bool = True,
                 front_stand_tarsus_deg: float = 0.0,
                 front_stand_foot_pitch_deg: float = None,
                 y_offset: float = 0.0):
        self.body_height = body_height
        self.x_offset_front = x_offset_front if x_offset_front is not None else _FRONT_X0
        self.x_offset_rear = x_offset_rear if x_offset_rear is not None else _REAR_X0
        self.hip_abduction = hip_abduction
        # Body-frame foot Y (+left); same sign as walk --y-shift.
        self.y_offset = float(y_offset)
        # use_tarsus 恒 True: 唯一站姿就是三连杆带主动 tarsus, 不再退回老姿态。
        self.use_tarsus = True
        # 必须和 NaturalTrot/StableTrot 的 front_standing_foot_pitch() 用同一套输入,
        # 否则 stand 的 tarsus 和步态 ramp=0 起点的 tarsus 会不一致 (assert_stand_matches_gait_start 捕获)。
        self.front_stand_tarsus_deg = front_stand_tarsus_deg
        self.front_stand_foot_pitch_deg = (
            front_stand_foot_pitch_deg
            if front_stand_foot_pitch_deg is not None
            else self._DEFAULT_FOOT_PITCH_DEG)
        self._update_cache()

    def _update_cache(self):
        foot_pitch = front_standing_foot_pitch(
            self.body_height, self.x_offset_front, self.front_stand_tarsus_deg,
            foot_pitch=math.radians(self.front_stand_foot_pitch_deg))
        self._cached = compute_standing_pose_3link(
            self.body_height, self.x_offset_front, self.x_offset_rear,
            self.hip_abduction, foot_pitch=foot_pitch)
        # Apply constant lateral foot offset (abd delta), matching SoftTrot y_body.
        y = float(self.y_offset)
        if abs(y) > 1e-9:
            h = max(1e-3, float(self.body_height))
            for leg, jname in (
                ("fl", "fl_thigh_roll"),
                ("fr", "fr_thigh_roll"),
                ("rl", "rl_hip"),
                ("rr", "rr_hip"),
            ):
                j = JOINT_BY_NAME[jname]
                side = 1.0 if leg.endswith("l") else -1.0
                self._cached[j.motor_id] = (
                    float(self._cached.get(j.motor_id, 0.0)) + side * y / h
                )

    def set_height(self, h: float):
        self.body_height = h
        self._update_cache()

    def set_hip_abduction(self, hip_abduction: float):
        """更新站立外展角并刷新缓存(URDF: 正=四腿同时向外)。"""
        self.hip_abduction = float(hip_abduction)
        self._update_cache()

    def get_targets(self, t: float) -> dict:
        targets = dict(self._cached)
        j_wp = JOINT_BY_NAME["waist_pitch"]
        targets[j_wp.motor_id] = _clamp(self.waist_pitch_offset, j_wp.limit_lo, j_wp.limit_hi)
        j_wy = JOINT_BY_NAME["waist_yaw"]
        targets[j_wy.motor_id] = _clamp(self.waist_yaw_offset, j_wy.limit_lo, j_wy.limit_hi)
        return targets

    def describe(self) -> str:
        return f"STAND  height={self.body_height:.3f}m"


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
                 y_offset: float = 0.0):
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
        # Constant body-frame foot Y (+left); same as walk --y-shift / stand.
        self.y_offset         = float(y_offset)
        self.lateral_sway     = lateral_sway
        self.anti_roll        = anti_roll
        self.reactive_kp      = reactive_kp
        self.reactive_kd      = reactive_kd
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

        x, is_swing, u = _ft.stable_trot_x(phase, amp, cx, self.stance_ratio, SMOOTH_GAIT)
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
                lift = _ft.stance_anti_roll_lift(u, self.anti_roll, self._anti_roll_diag_scale(t))
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
        return self._spot.com_shift_xy(
            t, self.period, self.stance_ratio, self._PHASE_OFFSET)

    def _lateral_offset(self, t: float) -> float:
        """横向 CoM 偏移。

        Spot: Y of support-triangle CoM shift (unloads the swing leg).
        Cruise: diagonal trot sway (unchanged).
        """
        if getattr(self, "spot_turn_active", False):
            # CoM unload via executor MPC/base_acc only — abd sway on top of
            # world-hold twist was stacking and tipping in catch.
            return 0.0
        return _ft.lateral_offset_trot(t, self.period, self.stance_ratio, self.lateral_sway)

    def _expected_diagonal_roll(self, t: float) -> float:
        """对角 Trot 支撑引起的预期 roll (度), 与 sway 无关。

        FL+RR 支撑 (phase 0~sr): 负 roll; FR+RL 支撑 (phase 0.5~0.5+sr): 正 roll。
        半正弦包络, 支撑中期达峰 — 匹配无 IMU 时测得的 ~2×步频摆动。
        """
        return _ft.expected_diagonal_roll(
            t, self.period, self.stance_ratio,
            self.trot_roll_ff_neg_deg, self.trot_roll_ff_pos_deg)

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

        摆动腿腾空无法推动身体, 但给摆动腿落脚点一点预调平(SWING_LEVEL)可让它
        按纠正后的姿态落地, 消除"摆动→支撑"切换时纠正突然出现的顿挫(开源做法)。
        [P2] SWING_LEVEL=0 时=原行为(仅支撑腿); >0 时摆动腿获得该比例的纠正, 用 max 保持连续。
        """
        return _ft.stance_weight(phase, self.stance_ratio, SWING_LEVEL, self._STANCE_TAPER)

    def _foot_track_gate(self, phase: float) -> float:
        """足朝向跟踪门控 [floor,1]: 支撑相=1(脚尖指地), 摆动相=front_foot_swing_track。"""
        return _ft.foot_track_gate(
            phase, self.stance_ratio, self.front_foot_swing_track, self._STANCE_TAPER)

    def get_targets(self, t: float, imu_dz: dict = None,
                    imu_state: dict = None) -> dict:
        """生成步态目标。"""
        targets = self._zero_targets()

        # Spot needs yaw/base + unitree tick before any _leg_xz / foot_xy call.
        # Turn LPF first so tick sees the same filtered stick as the rest of gait.
        self._turn_filtered += self.turn_filter_alpha * (self._turn_cmd - self._turn_filtered)
        if getattr(self, "spot_turn_active", False):
            self._spot_update_pose(t, imu_state)

        # Spot: do not use the 1s forward-walk ramp — it starves swing/abduct
        # and looks like incomplete in-place marching. Brief 0.25s blend only.
        if getattr(self, "spot_turn_active", False):
            blend = 0.25
            if blend > 0 and t < blend:
                s = t / blend
                ramp = s * s * (3.0 - 2.0 * s)
            else:
                ramp = 1.0
        elif self.ramp_duration > 0 and t < self.ramp_duration:
            s = t / self.ramp_duration
            ramp = s * s * (3.0 - 2.0 * s)
        else:
            ramp = 1.0

        z_front_base = -(self.body_height - _FRONT_HIP_OFFSET)
        z_rear_base  = -(self.body_height - _REAR_HIP_OFFSET)

        if getattr(self, "spot_turn_active", False):
            lat_offset = 0.0  # sway fights tangential abduction
        else:
            lat_offset = self._lateral_offset(t) * ramp

            # Raibert 反应式校正 — 全程连续(不分stance/swing) + 低通滤波
        if imu_state and not getattr(self, "spot_turn_active", False):
            roll = imu_state.get('roll', 0.0)
            gyro_roll = imu_state.get('gyro_roll', 0.0)
            raw = self.reactive_kp * roll + self.reactive_kd * gyro_roll
            raw = _clamp(raw, -0.10, 0.10)
            self._reactive_filtered += 0.15 * (raw - self._reactive_filtered)
        elif getattr(self, "spot_turn_active", False):
            self._reactive_filtered *= 0.9
        reactive = self._reactive_filtered * ramp

        # Raibert Heuristic for Velocity (Dynamic Foot Placement)
        dx_raibert = 0.0
        if (
            imu_state
            and "vel_xyz" in imu_state
            and not getattr(self, "spot_turn_active", False)
        ):
            v_actual = imu_state["vel_xyz"][0]
            avg_amp = (self.amp_front + getattr(self, "amp_rear", self.amp_front)) / 2.0
            vx_cmd = (avg_amp * 2.0) / self.period
            # Raibert 落足点：轻度补偿速度误差，限幅防撕裂步态
            if vx_cmd > 0.05:
                # v_actual > vx_cmd → 前冲 → 落足更靠前
                dx_raibert = 0.03 * (v_actual - vx_cmd)
                dx_raibert = _clamp(dx_raibert, -0.03, 0.03)

        # 前腿标称姿态 (推力放大基准): hip 偏离此姿态被放大, calf 仅解算 Z
        hip0_f, _calf0_f = ik_front_leg_2d(self.x_offset_front, z_front_base)

        for leg in ('fl', 'fr'):
            phase = (t / self.period + self._PHASE_OFFSET[leg]) % 1.0
            x_u, lift = self._leg_xz(leg, t, self._turn_filtered)
            cx = self.x_offset_front
            x_u = cx + (x_u - cx) * ramp + dx_raibert * ramp
            lift *= ramp
            z_u = z_front_base + lift
            if imu_dz:
                if getattr(self, "spot_turn_active", False):
                    w_st = 0.0 if self.spot_leg_in_swing(leg) else 1.0
                    z_u += imu_dz.get(leg, 0.0) * w_st
                else:
                    z_u += imu_dz.get(leg, 0.0) * self._stance_weight(phase)

            in_swing = (
                self.spot_leg_in_swing(leg)
                if getattr(self, "spot_turn_active", False)
                else phase >= self.stance_ratio
            )
            is_left = leg.endswith("l")
            if in_swing and self.swing_clearance_per_rad > 0 and imu_state:
                roll = imu_state.get('roll', 0.0)
                # roll>0 左高; 本侧偏低时 low_side>0 → 抬高摆动足端
                low_side = (-roll if is_left else roll)
                if low_side > 0:
                    z_u += low_side * self.swing_clearance_per_rad * self.body_height * ramp

            if self.front_foot_track_deg is not None:
                # 足朝向跟踪 (RL 策略: 脚尖时刻指地) — 三关节协同 3-DOF IK。
                # ramp 起点 = 站立足朝向(tarsus≈0), 平滑过渡到目标朝向。
                target_fp = math.radians(self.front_foot_track_deg)
                # 支撑相额外前倾蹬地 — spot 必须关，否则原地转变慢速前进
                if (
                    not in_swing
                    and self.front_foot_stance_push_deg != 0.0
                    and not getattr(self, "spot_turn_active", False)
                ):
                    s = phase / self.stance_ratio
                    target_fp -= math.radians(
                        self.front_foot_stance_push_deg) * math.sin(math.pi * s)
                # 摆动相回站立朝向(gate<1): 减少 tarsus 甩动与反作用力矩
                gate = self._foot_track_gate(phase)
                track_fp = (
                    self._front_stand_foot_pitch
                    + gate * (target_fp - self._front_stand_foot_pitch)
                )
                foot_pitch = (
                    self._front_stand_foot_pitch
                    + ramp * (track_fp - self._front_stand_foot_pitch)
                )
                hip_u, calf_u, tarsus_u = ik_front_3link_foot_orient(
                    x_u, z_u, foot_pitch)
            else:
                # 旧法: tarsus 支撑相 sin 蹬地鼓包, 摆动相中性
                tarsus_u = front_tarsus_schedule(
                    phase, self.stance_ratio, self.front_tarsus_push) * ramp
                # 3-link IK: 给定足端 (x,z) 与脚踝角求 hip+calf。tarsus=0 时退化为旧 2-link。
                hip_ik, calf_ik = ik_front_3link(x_u, z_u, tarsus_u)
                thrust_g = (
                    self.front_thrust_swing_gain if in_swing
                    else self.front_thrust_gain
                )
                if thrust_g >= 0.999:
                    hip_u, calf_u = hip_ik, calf_ik
                else:
                    # 解耦(削弱前腿推进)保留为可调回退: 衰减大腿摆动, calf 解算保持足端 Z
                    hip_u = hip0_f + thrust_g * (hip_ik - hip0_f)
                    calf_u = solve_front_calf_for_z_3link(
                        hip_u, z_u, tarsus_u, calf_init=calf_ik)
            mid_hp, cmd_hp = _cmd(f'{leg}_hip_pitch', hip_u)
            mid_ca, cmd_ca = _cmd(f'{leg}_calf', calf_u)
            targets[mid_hp] = cmd_hp
            targets[mid_ca] = cmd_ca

            mid_ta, cmd_ta = _cmd(f'{leg}_tarsus', tarsus_u)
            targets[mid_ta] = cmd_ta

            # 1. Sway (lateral CoM shift)
            # lat_offset > 0 means body moves left -> feet must move right (-Y) relative to body
            y_sway = -lat_offset

            # 2. Raibert reactive step
            # roll < 0 (left) -> reactive < 0 -> feet must step left (+Y) to catch
            if getattr(self, "spot_turn_active", False):
                y_reactive = 0.0
            else:
                y_reactive = -reactive * self.body_height if phase >= self.stance_ratio else 0.0

            # 3. Turn step
            if getattr(self, "spot_turn_active", False):
                y_turn = self._spot.cached_xy(leg)[1]
            else:
                y_turn = self._leg_y_turn(leg, t, self._turn_filtered) * ramp

            # Total Y offset
            y_total = (
                y_sway + y_reactive + y_turn
                + float(getattr(self, "y_offset", 0.0))
            )

            # Convert to abduction joint angle
            abd_delta = self._y_body_to_abd_roll(leg, y_total)

            roll_angle = front_thigh_roll_abd_urdf(leg, self.hip_abduction) + abd_delta
            mid_tr, cmd_tr = _cmd(f'{leg}_thigh_roll', roll_angle)
            targets[mid_tr] = cmd_tr

        for leg in ('rl', 'rr'):
            phase = (t / self.period + self._PHASE_OFFSET[leg]) % 1.0
            x_u, lift = self._leg_xz(leg, t, self._turn_filtered)
            cx = self.x_offset_rear
            x_u = cx + (x_u - cx) * ramp + dx_raibert * ramp
            lift *= ramp
            z_u = z_rear_base + lift
            if imu_dz:
                if getattr(self, "spot_turn_active", False):
                    # FSM stance only — trot phase was fighting catch diagonals.
                    w_st = 0.0 if self.spot_leg_in_swing(leg) else 1.0
                    z_u += imu_dz.get(leg, 0.0) * w_st
                else:
                    z_u += imu_dz.get(leg, 0.0) * self._stance_weight(phase)

            thigh_u, calf_u = ik_rear_leg_2d(x_u, z_u)
            mid_th, cmd_th = _cmd(f'{leg}_thigh', thigh_u)
            mid_ca, cmd_ca = _cmd(f'{leg}_calf', calf_u)
            targets[mid_th] = cmd_th
            targets[mid_ca] = cmd_ca

            # 由于 URDF 已在 2026-07 修正为对称语义，所有横向关节正值均代表外展
            # 统一使用 _y_body_to_abd_roll 将 Y 轴偏移转换为外展角
            
            # 1. Sway (lateral CoM shift)
            y_sway = -lat_offset

            # 2. Raibert reactive step
            if getattr(self, "spot_turn_active", False):
                y_reactive = 0.0
            else:
                y_reactive = -reactive * self.body_height if phase >= self.stance_ratio else 0.0

            # 3. Turn step
            if getattr(self, "spot_turn_active", False):
                y_turn = self._spot.cached_xy(leg)[1]
            else:
                y_turn = self._leg_y_turn(leg, t, self._turn_filtered) * ramp

            # Total Y offset
            y_total = (
                y_sway + y_reactive + y_turn
                + float(getattr(self, "y_offset", 0.0))
            )

            # Convert to abduction joint angle
            abd_delta = self._y_body_to_abd_roll(leg, y_total)

            hip_roll = self.hip_abduction + abd_delta
            mid_hr, cmd_hr = _cmd(f'{leg}_hip', hip_roll)
            targets[mid_hr] = cmd_hr

        j_wp = JOINT_BY_NAME["waist_pitch"]
        targets[j_wp.motor_id] = _clamp(
            self.waist_pitch_offset, j_wp.limit_lo, j_wp.limit_hi)
        
        # waist_yaw: turn>0(左转) → 正角，前半身朝转向侧拧（与腿同向）。
        # spot 用中等偏置+对角脉冲；勿用旧 ±20° 硬锁。
        j_wy = JOINT_BY_NAME["waist_yaw"]
        if getattr(self, "spot_turn_active", False):
            turn = float(self._turn_filtered)
            bias = (
                turn
                * float(self.spot_waist_yaw_rad)
                * ramp
                * self.waist_yaw_turn_sign
            )
            bp = (t / max(1e-6, self.period)) % 1.0
            pulse = (
                turn
                * float(self.spot_waist_yaw_pulse_rad)
                * math.sin(2.0 * math.pi * bp)
                * ramp
                * self.waist_yaw_turn_sign
            )
            wy_turn = bias + pulse
        else:
            wy_turn = (
                self._turn_filtered
                * self.max_turn_waist_yaw
                * ramp
                * self.waist_yaw_turn_sign
            )
        targets[j_wy.motor_id] = _clamp(
            self.waist_yaw_offset + wy_turn, j_wy.limit_lo, j_wy.limit_hi)

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
    _PHASE_OFFSET = {'fl': 0.0, 'rl': 0.0, 'fr': 0.5, 'rr': 0.5}

    def _lateral_offset(self, t: float) -> float:
        """Pace 专用横向偏移 — 使用全周期余弦确保抬腿瞬间已有重心转移。

        FR+RR 单支撑中点: phase = sr/2
        FL+RL 单支撑中点: phase = (sr+1)/2
        在两对腿抬起瞬间，重心已转移 ≥71% 到支撑侧。
        """
        return _ft.lateral_offset_pace(t, self.period, self.stance_ratio, self.lateral_sway)


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

    def _spine_ramp(self, t: float) -> float:
        """脊柱律动软启动。"""
        if self.ramp_duration > 0 and t < self.ramp_duration:
            s = t / self.ramp_duration
            return s * s * (3.0 - 2.0 * s)
        return 1.0

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
                lift = _ft.stance_anti_roll_lift(u, self.anti_roll, self._anti_roll_diag_scale(t))
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

        ramp = self._spine_ramp(t)
        bp = (t / self.period) % 1.0

        # Spot turn: freeze gait-locked spine oscillation (it fights yaw).
        spine_scale = 0.0 if getattr(self, "spot_turn_active", False) else 1.0

        # ── 层3: 脊柱对角律动 ──
        yaw_osc = (
            math.radians(self.spine_yaw_deg)
            * math.sin(2.0 * math.pi * bp + math.radians(self.spine_phase_deg))
            * ramp
            * spine_scale
        )
        roll_osc = (
            math.radians(self.spine_roll_deg)
            * math.sin(2.0 * math.pi * bp
                       + math.radians(self.spine_roll_phase_deg))
            * ramp
            * spine_scale
        )

        j_wy = JOINT_BY_NAME["waist_yaw"]
        targets[j_wy.motor_id] = _clamp(
            targets.get(j_wy.motor_id, 0.0) + yaw_osc,
            j_wy.limit_lo, j_wy.limit_hi)

        j_wr = JOINT_BY_NAME["waist_roll"]
        targets[j_wr.motor_id] = _clamp(
            targets.get(j_wr.motor_id, 0.0) + roll_osc,
            j_wr.limit_lo, j_wr.limit_hi)

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
        base = super().get_expected_roll(t)
        if abs(self.spine_roll_deg) < 0.01:
            return base
        ramp = self._spine_ramp(t)
        bp = (t / self.period) % 1.0
        spine_roll_contrib = (
            self.spine_roll_deg
            * math.sin(2.0 * math.pi * bp
                       + math.radians(self.spine_roll_phase_deg))
            * ramp
        )
        return base + spine_roll_contrib

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

    def _lateral_offset(self, t: float) -> float:
        """SoftTrot 横向移重：优先事件型 com_shift，否则回退半正弦 lateral_sway。"""
        if getattr(self, "spot_turn_active", False):
            return 0.0
        if abs(self.com_shift_m) > 1e-6:
            return _ft.lateral_offset_soft_trot_com(
                t, self.period, self.com_shift_m, self.com_shift_blend)
        return _ft.lateral_offset_trot(
            t, self.period, self.stance_ratio, self.lateral_sway)

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
                    u, self.anti_roll, self.anti_roll_soft_scale,
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
        return _ft.lateral_offset_walk(t, self.period, self.lateral_sway)

    def get_com_y_shift(self, t: float) -> float:
        """Body-frame CoM Y for SRB-MPC (+Y = left), event-locked to walk."""
        return _ft.walk_com_y_shift(t, self.period, self.com_sway_m)

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


class JumpPhase(str, Enum):
    IDLE = "idle"
    CROUCH = "crouch"
    PUSH = "push"
    FLIGHT = "flight"
    LAND = "land"
    RECOVER = "recover"


class JumpController(StandController):
    """原地四足同步 hop — 与 SoftTrot/Walk/Spot 解耦。

    IDLE → CROUCH → PUSH → FLIGHT → LAND → RECOVER → IDLE
    family=jump；永不 spot_turn。
    """

    family = "jump"
    # All legs sync; ContactSchedule uses phase<=stance as "scheduled stance".
    _PHASE_OFFSET = {"fl": 0.0, "fr": 0.0, "rl": 0.0, "rr": 0.0}

    def __init__(
        self,
        body_height: float = 0.24,
        x_offset_front: float = None,
        x_offset_rear: float = None,
        hip_abduction: float = 0.05,
        front_stand_tarsus_deg: float = 0.0,
        front_stand_foot_pitch_deg: float = None,
        y_offset: float = 0.0,
        crouch_depth: float = 0.045,
        crouch_s: float = 0.28,
        push_s: float = 0.12,
        flight_s: float = 0.18,
        land_s: float = 0.22,
        recover_s: float = 0.25,
        flight_clearance: float = 0.025,
        land_compress: float = 0.012,
        push_vz: float = 0.55,
        push_extend: float = 0.020,
        # Jump-only base-Z PD (do NOT write into global DynamicsConfig / Soft args).
        kp_base_z: float = 80.0,
        kd_base_z: float = 10.0,
        **_ignored,
    ):
        super().__init__(
            body_height=body_height,
            x_offset_front=x_offset_front,
            x_offset_rear=x_offset_rear,
            hip_abduction=hip_abduction,
            front_stand_tarsus_deg=front_stand_tarsus_deg,
            front_stand_foot_pitch_deg=front_stand_foot_pitch_deg,
            y_offset=y_offset,
        )
        self.family = "jump"
        self.spot_turn_active = False
        self.stand_height = float(body_height)
        self.crouch_depth = float(crouch_depth)
        self.crouch_s = float(crouch_s)
        self.push_s = float(push_s)
        self.flight_s = float(flight_s)
        self.land_s = float(land_s)
        self.recover_s = float(recover_s)
        self.flight_clearance = float(flight_clearance)
        self.land_compress = float(land_compress)
        self.push_vz = float(push_vz)
        self.push_extend = float(push_extend)
        self.kp_base_z = float(kp_base_z)
        self.kd_base_z = float(kd_base_z)

        self.phase = JumpPhase.IDLE
        self._phase_t0 = 0.0
        self.trigger = False
        self.auto_rejump = False
        self.vel_cmd = (0.0, 0.0, 0.0)
        self.speed_frac = 0.0
        self.stance_ratio = 1.0
        self._meas_vz = 0.0
        self._meas_z = 0.0
        self.period = (
            self.crouch_s + self.push_s + self.flight_s
            + self.land_s + self.recover_s
        )
        self.amp_front = 0.0
        self.amp_rear = 0.0
        self.turn_cmd = 0.0
        self.turn_y_gain = 0.0
        self._PHASE_OFFSET = dict(type(self)._PHASE_OFFSET)
        self._height_cmd = self.stand_height
        self._reactive_filtered = 0.0

    def set_height(self, h: float):
        self.stand_height = float(h)
        if self.phase is JumpPhase.IDLE:
            self.body_height = self.stand_height
            self._height_cmd = self.stand_height
            self._update_cache()

    def set_period(self, p: float):
        # Jump timing is phase-duration based; ignore SoftTrot period broadcast.
        return

    def request_jump(self, enable: bool = True):
        self.trigger = bool(enable)

    def _dur(self, phase: JumpPhase) -> float:
        return {
            JumpPhase.IDLE: 0.0,
            JumpPhase.CROUCH: self.crouch_s,
            JumpPhase.PUSH: self.push_s,
            JumpPhase.FLIGHT: self.flight_s,
            JumpPhase.LAND: self.land_s,
            JumpPhase.RECOVER: self.recover_s,
        }[phase]

    def _enter(self, phase: JumpPhase, t: float):
        self.phase = phase
        self._phase_t0 = float(t)

    def _phase_u(self, t: float) -> float:
        dur = self._dur(self.phase)
        if dur <= 1e-9:
            return 1.0
        return max(0.0, min(1.0, (float(t) - self._phase_t0) / dur))

    def _advance(self, t: float):
        t = float(t)
        u = self._phase_u(t)
        if self.phase is JumpPhase.IDLE:
            if self.trigger or self.auto_rejump:
                self._enter(JumpPhase.CROUCH, t)
                self.trigger = False
        elif self.phase is JumpPhase.CROUCH:
            if u >= 1.0:
                self._enter(JumpPhase.PUSH, t)
        elif self.phase is JumpPhase.PUSH:
            # Leave at first vz peak (~0.85); don't keep PUSH into stilts dig.
            liftoff_vz = max(0.84, 0.38 * max(0.3, self.push_vz))
            past_peak = (
                u >= 0.34
                and self._meas_vz >= 0.60
                and self._meas_vz < getattr(self, "_prev_vz", self._meas_vz) - 0.010
            )
            if u >= 1.0 or (u >= 0.34 and self._meas_vz >= liftoff_vz) or past_peak:
                self._enter(JumpPhase.FLIGHT, t)
        elif self.phase is JumpPhase.FLIGHT:
            # Touch down when descending — don't wait out full flight_s with
            # folded legs near the ground (soft-contact second smash).
            if u >= 1.0 or (u >= 0.25 and self._meas_vz < 0.0):
                self._enter(JumpPhase.LAND, t)
        elif self.phase is JumpPhase.LAND:
            if u >= 1.0:
                self._enter(JumpPhase.RECOVER, t)
        elif self.phase is JumpPhase.RECOVER:
            if u >= 1.0:
                self._enter(JumpPhase.IDLE, t)
                if self.auto_rejump:
                    self.trigger = True

    def _smooth(self, u: float) -> float:
        u = max(0.0, min(1.0, u))
        return u * u * (3.0 - 2.0 * u)

    def _height_for_phase(self, t: float) -> float:
        u = self._smooth(self._phase_u(t))
        h0 = self.stand_height
        hc = max(0.14, h0 - self.crouch_depth)
        hp = min(h0 + self.push_extend, hc + self.crouch_depth + self.push_extend)
        if self.phase is JumpPhase.IDLE:
            return h0
        if self.phase is JumpPhase.CROUCH:
            return h0 + (hc - h0) * u
        if self.phase is JumpPhase.PUSH:
            return hc + (hp - hc) * u
        if self.phase is JumpPhase.FLIGHT:
            # Hold push-extend height while feet retract via clearance in IK frame.
            return hp
        if self.phase is JumpPhase.LAND:
            # Soft land crouch — never slam from push-extend / full stand.
            hl = max(0.14, h0 - self.land_compress)
            return hl
        if self.phase is JumpPhase.RECOVER:
            hl = max(0.14, h0 - self.land_compress)
            # Slow ease to stand; stay slightly short early to avoid dig-smash.
            return hl + (h0 - hl) * (u * u)
        return h0

    def in_flight(self) -> bool:
        return self.phase is JumpPhase.FLIGHT

    def note_base_vz(self, vz: float) -> None:
        self._prev_vz = float(getattr(self, "_meas_vz", 0.0))
        self._meas_vz = float(vz)

    def note_base_z(self, z: float) -> None:
        self._meas_z = float(z)

    def jump_force_scale_at(self, t: float) -> float:
        if self.phase is JumpPhase.FLIGHT:
            return 0.0
        if self.phase is JumpPhase.PUSH:
            return 1.0
        if self.phase is JumpPhase.LAND:
            u = self._phase_u(t)
            # Soft touchdown — high force here is the "second smash".
            return 0.12 + 0.28 * self._smooth(u)
        if self.phase is JumpPhase.RECOVER:
            u = self._phase_u(t)
            return 0.25 + 0.30 * self._smooth(u)
        return 1.0

    def predict_jump_force_scale(self, t_future: float) -> float:
        t = t_future
        phase = self.phase
        t0 = self._phase_t0
        
        while True:
            dur = self._dur(phase)
            if t < t0 + dur or dur <= 1e-9:
                break
            t0 += dur
            if phase is JumpPhase.IDLE:
                if self.trigger or self.auto_rejump:
                    phase = JumpPhase.CROUCH
                else:
                    break
            elif phase is JumpPhase.CROUCH: phase = JumpPhase.PUSH
            elif phase is JumpPhase.PUSH: phase = JumpPhase.FLIGHT
            elif phase is JumpPhase.FLIGHT: phase = JumpPhase.LAND
            elif phase is JumpPhase.LAND: phase = JumpPhase.RECOVER
            elif phase is JumpPhase.RECOVER: 
                phase = JumpPhase.IDLE
                break
                
        if phase is JumpPhase.FLIGHT:
            return 0.0
        if phase is JumpPhase.PUSH:
            return 1.0
        if phase is JumpPhase.LAND:
            dur = self._dur(phase)
            u = max(0.0, min(1.0, (t - t0) / dur)) if dur > 1e-9 else 1.0
            return 0.12 + 0.28 * self._smooth(u)
        if phase is JumpPhase.RECOVER:
            dur = self._dur(phase)
            u = max(0.0, min(1.0, (t - t0) / dur)) if dur > 1e-9 else 1.0
            return 0.25 + 0.30 * self._smooth(u)
        return 1.0

    def desired_vz(self, t: float = None) -> float:
        if self.phase is JumpPhase.PUSH:
            u = self._phase_u(t if t is not None else self._phase_t0)
            # Strong lead — lagging cmd brakes a rising hop.
            return self.push_vz * min(1.0, 0.45 + 0.55 * self._smooth(u))
        if self.phase is JumpPhase.FLIGHT:
            return 0.0
        if self.phase is JumpPhase.LAND:
            return -0.15
        return 0.0

    def get_target_z(self, t: float = None) -> float:
        return float(self._height_cmd)

    def get_targets(self, t: float, imu_dz: dict = None,
                    imu_state: dict = None) -> dict:
        self.spot_turn_active = False
        self._advance(t)
        # ContactSchedule: stance_ratio=0 → all swing in FLIGHT; else all stance.
        self.stance_ratio = 0.0 if self.phase is JumpPhase.FLIGHT else 1.0
        h = self._height_for_phase(t)
        self._height_cmd = h
        # body_height = hip-to-foot; lower IK height folds legs for flight clearance.
        if self.phase is JumpPhase.FLIGHT:
            u = self._phase_u(t)
            # Fast fold, but not so hard PD yanks the body back down.
            early = 0.85 + 0.15 * self._smooth(min(1.0, u / 0.12))
            late = self._smooth(max(0.0, (u - 0.12) / 0.88))
            retract = max(early, late)
            h_ik = max(0.14, h - self.flight_clearance * retract)
            if self._meas_z > 0.12:
                h_ik = min(h_ik, max(0.14, self._meas_z - 0.018))
        elif self.phase is JumpPhase.PUSH:
            # Extend for launch; flight phase does the retract (peel-in-push
            # cut impulse and killed air height).
            h_ik = h
            if self._meas_z > 0.12:
                h_ik = min(h_ik, max(0.14, self._meas_z + 0.024))
        elif self.phase is JumpPhase.CROUCH:
            h_ik = h
            if self._meas_z > 0.12:
                h_ik = min(h_ik, max(0.14, self._meas_z + 0.010))
        elif self.phase in (JumpPhase.LAND, JumpPhase.RECOVER):
            h_ik = h
            if self._meas_z > 0.12:
                # Track body height so falling doesn't bury feet (soft-contact bounce).
                h_ik = min(h_ik, max(0.14, self._meas_z - 0.002))
        else:
            h_ik = h
        if abs(h_ik - self.body_height) > 1e-5:
            self.body_height = h_ik
            self._update_cache()
        targets = super().get_targets(t)
        self.body_height = h
        return targets

    def describe(self) -> str:
        return (
            f"JUMP[{self.phase.value}]  h0={self.stand_height:.3f}m  "
            f"crouch={self.crouch_depth*1000:.0f}mm  "
            f"push_vz={self.push_vz:.2f}m/s"
        )
