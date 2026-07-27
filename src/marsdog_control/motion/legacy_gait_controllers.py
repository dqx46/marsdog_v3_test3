"""Retired gait controllers — kept for offline analysis / manual tests only.

``TrotController`` / ``TurnTrotController`` / ``SimpleTrot`` are **not** on the
live control path. ``gait_recipes.build_controller_set`` (the sole factory the
runtime uses) only ever constructs ``StandController``/``StableTrot``/
``StablePace``/``NaturalTrot``/``NaturalSoftTrot`` — those stayed in
``gait_controller.py``. These three predate ``StableTrot`` (which superseded
them with Raibert reactive foot placement + lateral CoM sway) and now exist
only for:

  - ``manual_tests/legacy/test_amp_smooth.py``
  - ``apps/tools/analysis/dump_traj.py``
  - the ``python -m marsdog_control.motion.legacy_gait_controllers`` CLI below

Extracted out of ``gait_controller.py`` (Phase N) purely to shrink the "巨石"
module readers actually have to hold in their head for the live path; zero
behavior change, zero live-path risk (nothing on ``RuntimePipeline.tick``
imports this module).
"""

from __future__ import annotations

from marsdog_control.compat import ensure_legacy_path as _ensure_legacy_path
_ensure_legacy_path()

import math

from marsdog_control.motion.gait_controller import (
    GaitController, _FRONT_HIP_OFFSET, _REAR_HIP_OFFSET, _FRONT_X0, _REAR_X0,
    _clamp, _cmd,
)
from marsdog_control.motion.kinematics import (
    ik_front_leg_2d, ik_rear_leg_2d, front_thigh_roll_abd_urdf,
)
from marsdog_control.config.joints import JOINT_BY_NAME


class TrotController(GaitController):
    """动态对角小跑 — 基于 URDF 的 3D 步态规划

    核心改进:
      1. X: 全周期余弦 cos(2π·phase) — C-inf 连续
      2. Z: 三次 Bezier — 快起(30%)、巡航(40%)、柔降(30%), 首尾零速
      3. Y: 横向 CoM 摆动 — 向支撑对角线偏移, 提高稳定性
      4. pitch: 前进时微前倾, 补偿动态惯性

    相位 (归一化 [0,1)):
      FL+RR = 0.0  (对角同相)
      FR+RL = 0.5  (对角同相)
    """

    _PHASE_OFFSET = {'fl': 0.0, 'rr': 0.0, 'fr': 0.5, 'rl': 0.5}

    def __init__(self,
                 body_height:    float = 0.24,
                 amp_front:      float = 0.04,
                 amp_rear:       float = 0.047,
                 step_height:    float = 0.025,
                 step_height_front: float = None,
                 period:         float = 1.2,
                 stance_ratio:   float = 0.6,
                 x_offset_front: float = None,
                 x_offset_rear:  float = None,
                 lateral_sway:   float = 0.008,
                 pitch_lean:     float = 0.03,
                 hip_abduction:  float = 0.04,
                 anti_roll:      float = 0.006,
                 soft_landing:   float = 0.005):
        self.body_height = body_height
        self.amp_front   = amp_front
        self.amp_rear    = amp_rear
        self.step_height = step_height
        self.step_height_front = step_height_front if step_height_front is not None else step_height * 0.7
        self.period      = period
        self.stance_ratio = stance_ratio
        self.x_offset_front = x_offset_front if x_offset_front is not None else _FRONT_X0
        self.x_offset_rear  = x_offset_rear if x_offset_rear is not None else _REAR_X0
        self.lateral_sway = lateral_sway    # 横向摆动幅度 (m), 8mm
        self.pitch_lean   = pitch_lean      # 前进前倾角 (rad), ~1.7°
        self.hip_abduction = hip_abduction  # 静态髋外展角 (rad), ~2.3°
        self.anti_roll    = anti_roll       # 支撑腿Z补偿 (m), 抗roll倾倒
        self.soft_landing = soft_landing    # 着地缓冲高度 (m), 减少冲击

    _PEAK_T = 0.4   # 峰值在摆动相 40% 处 (略快起, 柔落)

    def _swing_z(self, swing_t: float, step_h: float) -> float:
        """分段 sin²/cos² Z 轨迹: 快起慢落, 首尾严格零速。

        Rise (0→t_peak): sin²(π·t/(2·t_peak))  — 快速上升
        Fall (t_peak→1): cos²(π·(t-t_peak)/(2·(1-t_peak))) — 缓慢下降

        数学保证: z'(0)=0, z'(1)=0, z'(t_peak)=0 (C1连续)
        """
        tp = self._PEAK_T
        if swing_t <= tp:
            s = math.sin(math.pi * swing_t / (2.0 * tp))
            return step_h * s * s
        else:
            s = math.cos(math.pi * (swing_t - tp) / (2.0 * (1.0 - tp)))
            return step_h * s * s

    def _body_x_shift(self, is_front: bool) -> float:
        """前后腿分别施加的足端 X 中性点偏移。
        前进: 前腿略前移, 后腿略前移
        后退: 前腿后移(推body后退), 后腿前移(收到身体下方防蹬空)
        """
        speed_norm = self.amp_front / 0.06 if abs(self.amp_front) > 0.001 else 0.0
        speed_norm = _clamp(speed_norm, -1.0, 1.0)
        if is_front:
            if speed_norm >= 0:
                return 0.015 * speed_norm
            else:
                return 0.02 * speed_norm
        else:
            if speed_norm >= 0:
                return 0.01 * speed_norm
            else:
                return -0.025 * speed_norm

    def _leg_xz(self, leg: str, t: float) -> tuple:
        """计算单腿足端 (x, z_lift) 相对于 hip。

        z_lift 含义:
          - 支撑相: anti-roll 补偿 (负值=腿伸长=撑起body)
          - 摆动相: 抬腿高度 + soft landing 缓冲
        """
        phase = (t / self.period + self._PHASE_OFFSET[leg]) % 1.0
        is_front = leg.startswith('f')
        amp = self.amp_front if is_front else self.amp_rear
        cx = self.x_offset_front if is_front else self.x_offset_rear
        sh = self.step_height_front if is_front else self.step_height

        cx = cx + self._body_x_shift(is_front)

        if phase < self.stance_ratio:
            # 支撑相: anti-roll Z补偿
            # 支撑中期腿略伸长(z更负), 抵抗body向摆动侧倾倒
            # 用半正弦窗: 支撑相开头和结尾=0, 中间最大
            stance_t = phase / self.stance_ratio
            x = cx + amp * math.cos(math.pi * stance_t)
            anti_roll_z = -self.anti_roll * math.sin(math.pi * stance_t)
            lift = anti_roll_z
        else:
            swing_t = (phase - self.stance_ratio) / (1.0 - self.stance_ratio)
            x = cx - amp * math.cos(math.pi * swing_t)
            lift = self._swing_z(swing_t, sh)
            # soft landing: 摆动相末尾保留微小高度, 避免硬着地
            if swing_t > 0.8:
                landing_t = (swing_t - 0.8) / 0.2
                lift = max(lift, self.soft_landing * (1.0 - landing_t))
        return x, lift

    def _lateral_offset(self, t: float) -> float:
        """计算横向 CoM 偏移量 — 与支撑相严格同步。

        FL+RR 支撑 (phase 0~stance_ratio): body 偏左(+Y), 向 FL 侧
        FR+RL 支撑 (phase stance_ratio~1.0): body 偏右(-Y), 向 FR 侧

        用 cos 平滑, 在支撑相中间达峰, 切换时过零。
        """
        phase = (t / self.period) % 1.0
        sr = self.stance_ratio
        if phase < sr:
            # FL+RR 支撑: 用半余弦窗, 中间最大, 两端为零
            t_norm = phase / sr
            return self.lateral_sway * math.sin(math.pi * t_norm)
        else:
            # FR+RL 支撑: 反向
            t_norm = (phase - sr) / (1.0 - sr)
            return -self.lateral_sway * math.sin(math.pi * t_norm)

    def _pitch_compensation(self, t: float) -> float:
        """前进速度相关的前倾补偿。

        amp > 0 时前进, body 微前倾抵抗惯性后仰;
        amp < 0 时后退, body 微后仰 (幅度更大, 帮助后腿着地)。
        """
        speed_norm = self.amp_front / 0.06 if abs(self.amp_front) > 0.001 else 0.0
        speed_norm = _clamp(speed_norm, -1.0, 1.0)
        if speed_norm < 0:
            return self.pitch_lean * speed_norm * 1.5
        return self.pitch_lean * speed_norm

    def get_targets(self, t: float, imu_dz: dict = None) -> dict:
        """生成步态目标.

        Args:
            t: 当前时间 (s)
            imu_dz: IMU 姿态闭环 Z 修正量 {'fl': dz, 'fr': dz, 'rl': dz, 'rr': dz}
                    正值=腿缩短, 负值=腿伸长. None=不修正.
        """
        targets = self._zero_targets()

        # 动态补偿
        lat_offset = self._lateral_offset(t)
        pitch_comp = self._pitch_compensation(t)

        z_front_base = -(self.body_height - _FRONT_HIP_OFFSET)
        z_rear_base  = -(self.body_height - _REAR_HIP_OFFSET)

        # 前腿 (hip_pitch + thigh_roll + calf)
        for leg in ('fl', 'fr'):
            x_u, lift = self._leg_xz(leg, t)
            z_u = z_front_base + lift
            # IMU 修正只作用于支撑腿，摆动腿由轨迹规划决定高度
            if imu_dz:
                phase = (t / self.period + self._PHASE_OFFSET[leg]) % 1.0
                if phase < self.stance_ratio:
                    z_u += imu_dz.get(leg, 0.0)
            hip_u, calf_u = ik_front_leg_2d(x_u, z_u)
            mid_hp, cmd_hp = _cmd(f'{leg}_hip_pitch', hip_u)
            mid_ca, cmd_ca = _cmd(f'{leg}_calf',      calf_u)
            targets[mid_hp] = cmd_hp
            targets[mid_ca] = cmd_ca

            # 横向: 静态外展 + 动态 CoM 摆动 (fl +urdf / fr -urdf = 向外)
            sway_sign = -1.0 if leg == 'fl' else 1.0
            sway_angle = sway_sign * lat_offset / self.body_height
            roll_angle = (
                front_thigh_roll_abd_urdf(leg, self.hip_abduction) + sway_angle
            )
            mid_tr, cmd_tr = _cmd(f'{leg}_thigh_roll', roll_angle)
            targets[mid_tr] = cmd_tr

        # 后腿 (hip_roll + thigh + calf)
        for leg in ('rl', 'rr'):
            x_u, lift = self._leg_xz(leg, t)
            z_u = z_rear_base + lift
            if imu_dz:
                phase = (t / self.period + self._PHASE_OFFSET[leg]) % 1.0
                if phase < self.stance_ratio:
                    z_u += imu_dz.get(leg, 0.0)
            thigh_u, calf_u = ik_rear_leg_2d(x_u, z_u)
            mid_th, cmd_th = _cmd(f'{leg}_thigh', thigh_u)
            mid_ca, cmd_ca = _cmd(f'{leg}_calf',  calf_u)
            targets[mid_th] = cmd_th
            targets[mid_ca] = cmd_ca

            # 横向: 静态外展 + 动态 CoM 摆动
            # 实测: RL正值=向外, RR负值=向外 (RL/RR的j.sign都是+1)
            # 为了同向倾斜: 后腿都用 +lat_offset/h
            abd_sign = 1.0 if leg == 'rl' else -1.0
            abduction = self.hip_abduction * abd_sign
            sway_angle = lat_offset / self.body_height
            hip_roll = abduction + sway_angle
            mid_hr, cmd_hr = _cmd(f'{leg}_hip', hip_roll)
            targets[mid_hr] = cmd_hr

        # 动态 pitch 补偿 → 叠加到 waist_pitch
        j_wp = JOINT_BY_NAME["waist_pitch"]
        targets[j_wp.motor_id] = _clamp(
            self.waist_pitch_offset + pitch_comp,
            j_wp.limit_lo, j_wp.limit_hi)

        # waist_yaw 保持归零 (直行不扭腰)
        j_wy = JOINT_BY_NAME["waist_yaw"]
        targets[j_wy.motor_id] = _clamp(
            self.waist_yaw_offset, j_wy.limit_lo, j_wy.limit_hi)

        return targets

    def set_period(self, p: float):
        self.period = max(0.2, min(2.0, p))

    def set_height(self, h: float):
        self.body_height = h

    def describe(self) -> str:
        step_f = 2 * abs(self.amp_front) * 100
        step_r = 2 * abs(self.amp_rear) * 100
        import math as _m
        return (f"TROT  T={self.period:.2f}s  "
                f"H={self.body_height:.3f}m  "
                f"stride(F/R)={step_f:.1f}/{step_r:.1f}cm  "
                f"lift(F/R)={self.step_height_front*100:.1f}/{self.step_height*100:.1f}cm  "
                f"anti_roll={self.anti_roll*1000:.0f}mm  "
                f"soft_land={self.soft_landing*1000:.0f}mm  "
                f"abd={_m.degrees(self.hip_abduction):.1f}°  "
                f"stance={self.stance_ratio:.0%}")


# ─────────────────────────────────────────────────────────────────────────────
# 转向 Trot — 利用 waist_yaw 扭腰 + 差速步幅实现原地/行进转向
# ─────────────────────────────────────────────────────────────────────────────

class TurnTrotController(TrotController):
    """带转向的 Trot 控制器。

    转向策略 (模拟真实犬类):
      1. waist_yaw 向转向方向扭转, 让前半身先朝向目标方向
      2. 前腿差速: 外侧腿步幅增大, 内侧腿步幅减小
      3. 后腿差速: 与前腿相反 (外小内大), 形成绕竖轴旋转
      4. 周期稍慢, 确保转向时稳定

    turn_cmd: -1.0(右转) ~ +1.0(左转), 0=直行
    """

    def __init__(self, **kwargs):
        # 转向专用参数
        self.turn_period = kwargs.pop('turn_period', 1.4)  # 转向时周期放慢，让动作更优雅从容
        self.max_yaw_angle = kwargs.pop('max_yaw_angle', 0.35)  # rad, ~20° 减小最大扭腰幅度，避免动作过猛
        self.max_diff_ratio = kwargs.pop('max_diff_ratio', 0.3)  # 差速比例调小，让内外侧腿步幅差异更平缓
        self.yaw_alpha = kwargs.pop('yaw_alpha', 0.08)  # 低通系数调小，让扭腰的过渡更柔和
        self.max_waist_roll = kwargs.pop('max_waist_roll', 0.08) # 身体向内倾斜角减小
        self.max_bank_angle = kwargs.pop('max_bank_angle', 0.05) # 腿部向内倾斜角减小
        super().__init__(**kwargs)
        self._turn_cmd = 0.0
        self._forward_cmd = 0.0
        self._yaw_target = 0.0
        self._yaw_current = 0.0

    @property
    def turn_cmd(self):
        return self._turn_cmd

    @turn_cmd.setter
    def turn_cmd(self, v):
        self._turn_cmd = _clamp(v, -1.0, 1.0)

    @property
    def forward_cmd(self):
        return self._forward_cmd

    @forward_cmd.setter
    def forward_cmd(self, v):
        self._forward_cmd = _clamp(v, -1.0, 1.0)

    def get_targets(self, t: float, imu_dz: dict = None) -> dict:
        """生成带转向的步态目标."""
        turn = self._turn_cmd
        fwd = self._forward_cmd

        # 转向时用较慢周期
        if abs(turn) > 0.1:
            effective_period = self.period + (self.turn_period - self.period) * abs(turn)
        else:
            effective_period = self.period

        # 计算各腿步幅
        if abs(fwd) < 0.01:
            # 原地转向 (turn in place): 左转(turn>0)时，左侧腿后退，右侧腿前进
            # 原地转向时，步幅也放慢一点，显得更优雅
            amp_fl = -turn * self.amp_front * 0.6
            amp_rl = -turn * self.amp_rear * 0.6
            amp_fr =  turn * self.amp_front * 0.6
            amp_rr =  turn * self.amp_rear * 0.6
        else:
            # 行进间转向 (walking turn): 差速步幅 + 身体弯曲
            diff = turn * self.max_diff_ratio
            # 左转(turn>0)时，左侧(内侧)步幅小，右侧(外侧)步幅大。前后腿同向！
            amp_fl = fwd * self.amp_front * (1.0 - diff)
            amp_fr = fwd * self.amp_front * (1.0 + diff)
            amp_rl = fwd * self.amp_rear * (1.0 - diff)
            amp_rr = fwd * self.amp_rear * (1.0 + diff)

        # waist_yaw 目标角度 (平滑过渡)
        self._yaw_target = turn * self.max_yaw_angle
        alpha_yaw = self.yaw_alpha
        self._yaw_current += alpha_yaw * (self._yaw_target - self._yaw_current)

        # 仿生倾斜 (Leaning): 转向时身体向内侧倾斜
        # turn>0 (左转): 需要向左倾斜。
        # waist_roll 轴为 +X，正角使上半身向右倾斜，所以需要负角向左倾斜。
        waist_roll_target = -turn * self.max_waist_roll
        # bank_angle: 腿部整体倾斜。turn>0(左转)时，腿需要向右旋转以使身体向左倾斜。
        # thigh_roll/hip_roll 轴为 +X，正角向左旋转。所以需要负角向右旋转。
        bank_angle = -turn * self.max_bank_angle

        # 构建目标 (复用 TrotController 的足端轨迹逻辑)
        targets = self._zero_targets()

        lat_offset = self._lateral_offset(t)
        pitch_comp = self._pitch_compensation(t)

        z_front_base = -(self.body_height - _FRONT_HIP_OFFSET)
        z_rear_base = -(self.body_height - _REAR_HIP_OFFSET)

        # 前腿 — 使用各自独立步幅
        for leg, amp_leg in [('fl', amp_fl), ('fr', amp_fr)]:
            phase = (t / effective_period + self._PHASE_OFFSET[leg]) % 1.0
            is_front = True
            cx = self.x_offset_front + self._body_x_shift(is_front)
            sh = self.step_height_front

            if phase < self.stance_ratio:
                stance_t = phase / self.stance_ratio
                x = cx + amp_leg * math.cos(math.pi * stance_t)
                lift = -self.anti_roll * math.sin(math.pi * stance_t)
            else:
                swing_t = (phase - self.stance_ratio) / (1.0 - self.stance_ratio)
                x = cx - amp_leg * math.cos(math.pi * swing_t)
                lift = self._swing_z(swing_t, sh)
                if swing_t > 0.8:
                    landing_t = (swing_t - 0.8) / 0.2
                    lift = max(lift, self.soft_landing * (1.0 - landing_t))

            z_u = z_front_base + lift
            if imu_dz and phase < self.stance_ratio:
                z_u += imu_dz.get(leg, 0.0)

            hip_u, calf_u = ik_front_leg_2d(x, z_u)
            mid_hp, cmd_hp = _cmd(f'{leg}_hip_pitch', hip_u)
            mid_ca, cmd_ca = _cmd(f'{leg}_calf', calf_u)
            targets[mid_hp] = cmd_hp
            targets[mid_ca] = cmd_ca

            sway_sign = -1.0 if leg == 'fl' else 1.0
            sway_angle = sway_sign * lat_offset / self.body_height
            roll_angle = (
                front_thigh_roll_abd_urdf(leg, self.hip_abduction)
                + sway_angle + bank_angle
            )
            mid_tr, cmd_tr = _cmd(f'{leg}_thigh_roll', roll_angle)
            targets[mid_tr] = cmd_tr

        # 后腿 — 使用各自独立步幅
        for leg, amp_leg in [('rl', amp_rl), ('rr', amp_rr)]:
            phase = (t / effective_period + self._PHASE_OFFSET[leg]) % 1.0
            is_front = False
            cx = self.x_offset_rear + self._body_x_shift(is_front)
            sh = self.step_height

            if phase < self.stance_ratio:
                stance_t = phase / self.stance_ratio
                x = cx + amp_leg * math.cos(math.pi * stance_t)
                lift = -self.anti_roll * math.sin(math.pi * stance_t)
            else:
                swing_t = (phase - self.stance_ratio) / (1.0 - self.stance_ratio)
                x = cx - amp_leg * math.cos(math.pi * swing_t)
                lift = self._swing_z(swing_t, sh)
                if swing_t > 0.8:
                    landing_t = (swing_t - 0.8) / 0.2
                    lift = max(lift, self.soft_landing * (1.0 - landing_t))

            z_u = z_rear_base + lift
            if imu_dz and phase < self.stance_ratio:
                z_u += imu_dz.get(leg, 0.0)

            thigh_u, calf_u = ik_rear_leg_2d(x, z_u)
            mid_th, cmd_th = _cmd(f'{leg}_thigh', thigh_u)
            mid_ca, cmd_ca = _cmd(f'{leg}_calf', calf_u)
            targets[mid_th] = cmd_th
            targets[mid_ca] = cmd_ca

            abd_sign = 1.0 if leg == 'rl' else -1.0
            abduction = self.hip_abduction * abd_sign
            sway_angle = lat_offset / self.body_height
            hip_roll = abduction + sway_angle + bank_angle
            mid_hr, cmd_hr = _cmd(f'{leg}_hip', hip_roll)
            targets[mid_hr] = cmd_hr

        # waist_yaw 转向
        j_wy = JOINT_BY_NAME["waist_yaw"]
        targets[j_wy.motor_id] = _clamp(
            self.waist_yaw_offset + self._yaw_current,
            j_wy.limit_lo, j_wy.limit_hi)

        # waist_roll 身体侧倾
        j_wr = JOINT_BY_NAME["waist_roll"]
        targets[j_wr.motor_id] = _clamp(
            waist_roll_target,
            j_wr.limit_lo, j_wr.limit_hi)

        # waist_pitch 保持弓背
        j_wp = JOINT_BY_NAME["waist_pitch"]
        targets[j_wp.motor_id] = _clamp(
            self.waist_pitch_offset + pitch_comp,
            j_wp.limit_lo, j_wp.limit_hi)

        return targets

    def describe(self) -> str:
        base = super().describe()
        return base + f"  turn={self._turn_cmd:+.2f}  yaw={math.degrees(self._yaw_current):+.1f}°  fwd={self._forward_cmd:+.2f}"


class SimpleTrot(GaitController):
    """最简 Trot — 纯开环, 无任何补偿, 用于验证基础运动学。

    只有两个轨迹:
      X: 全周期余弦 cos(2π·phase) — 支撑推+摆动回
      Z: sin² 摆动抬腿, 支撑相 z=0

    所有高级特性 (anti-roll, lateral sway, pitch lean, soft landing,
    body_x_shift) 全部关闭, 确保出问题时排除范围最小。
    """

    _PHASE_OFFSET = {'fl': 0.0, 'rr': 0.0, 'fr': 0.5, 'rl': 0.5}

    def __init__(self,
                 body_height:   float = 0.24,
                 amp_front:     float = 0.02,
                 amp_rear:      float = 0.024,
                 step_height:   float = 0.02,
                 period:        float = 1.0,
                 stance_ratio:  float = 0.6,
                 x_offset_front: float = None,
                 x_offset_rear:  float = None,
                 hip_abduction: float = 0.04):
        self.body_height    = body_height
        self.amp_front      = amp_front
        self.amp_rear       = amp_rear
        self.step_height    = step_height
        self.period         = period
        self.stance_ratio   = stance_ratio
        self.x_offset_front = x_offset_front if x_offset_front is not None else _FRONT_X0
        self.x_offset_rear  = x_offset_rear if x_offset_rear is not None else _REAR_X0
        self.hip_abduction  = hip_abduction

    def _swing_z(self, swing_t: float) -> float:
        s = math.sin(math.pi * swing_t)
        return self.step_height * s * s

    def get_targets(self, t: float, imu_dz: dict = None) -> dict:
        targets = self._zero_targets()

        z_front_base = -(self.body_height - _FRONT_HIP_OFFSET)
        z_rear_base  = -(self.body_height - _REAR_HIP_OFFSET)

        for leg in ('fl', 'fr'):
            phase = (t / self.period + self._PHASE_OFFSET[leg]) % 1.0
            amp = self.amp_front
            cx = self.x_offset_front

            x = cx + amp * math.cos(2.0 * math.pi * phase)
            lift = 0.0
            if phase >= self.stance_ratio:
                swing_t = (phase - self.stance_ratio) / (1.0 - self.stance_ratio)
                lift = self._swing_z(swing_t)

            z_u = z_front_base + lift
            if imu_dz and phase < self.stance_ratio:
                z_u += imu_dz.get(leg, 0.0)

            hip_u, calf_u = ik_front_leg_2d(x, z_u)
            mid_hp, cmd_hp = _cmd(f'{leg}_hip_pitch', hip_u)
            mid_ca, cmd_ca = _cmd(f'{leg}_calf', calf_u)
            targets[mid_hp] = cmd_hp
            targets[mid_ca] = cmd_ca

            roll_angle = front_thigh_roll_abd_urdf(leg, self.hip_abduction)
            mid_tr, cmd_tr = _cmd(f'{leg}_thigh_roll', roll_angle)
            targets[mid_tr] = cmd_tr

        for leg in ('rl', 'rr'):
            phase = (t / self.period + self._PHASE_OFFSET[leg]) % 1.0
            amp = self.amp_rear
            cx = self.x_offset_rear

            x = cx + amp * math.cos(2.0 * math.pi * phase)
            lift = 0.0
            if phase >= self.stance_ratio:
                swing_t = (phase - self.stance_ratio) / (1.0 - self.stance_ratio)
                lift = self._swing_z(swing_t)

            z_u = z_rear_base + lift
            if imu_dz and phase < self.stance_ratio:
                z_u += imu_dz.get(leg, 0.0)

            thigh_u, calf_u = ik_rear_leg_2d(x, z_u)
            mid_th, cmd_th = _cmd(f'{leg}_thigh', thigh_u)
            mid_ca, cmd_ca = _cmd(f'{leg}_calf', calf_u)
            targets[mid_th] = cmd_th
            targets[mid_ca] = cmd_ca

            abd_sign = 1.0 if leg == 'rl' else -1.0
            hip_roll = self.hip_abduction * abd_sign
            mid_hr, cmd_hr = _cmd(f'{leg}_hip', hip_roll)
            targets[mid_hr] = cmd_hr

        j_wp = JOINT_BY_NAME["waist_pitch"]
        targets[j_wp.motor_id] = _clamp(
            self.waist_pitch_offset, j_wp.limit_lo, j_wp.limit_hi)
        j_wy = JOINT_BY_NAME["waist_yaw"]
        targets[j_wy.motor_id] = _clamp(
            self.waist_yaw_offset, j_wy.limit_lo, j_wy.limit_hi)

        return targets

    def set_period(self, p: float):
        self.period = max(0.2, min(2.0, p))

    def set_height(self, h: float):
        self.body_height = h

    def describe(self) -> str:
        return (f"SIMPLE_TROT  T={self.period:.2f}s  "
                f"H={self.body_height:.3f}m  "
                f"amp(F/R)={self.amp_front*100:.1f}/{self.amp_rear*100:.1f}cm  "
                f"lift={self.step_height*100:.1f}cm  "
                f"stance={self.stance_ratio:.0%}")


if __name__ == "__main__":
    import argparse

    from marsdog_control.motion.gait_controller import StandController

    p = argparse.ArgumentParser(description="步态控制器离线仿真输出(retired 控制器)")
    p.add_argument("--gait",   choices=["stand", "trot"], default="trot")
    p.add_argument("--height", type=float, default=0.24)
    p.add_argument("--period", type=float, default=0.5)
    p.add_argument("--steps",  type=int,   default=20, help="采样点数")
    args = p.parse_args()

    if args.gait == "stand":
        ctrl = StandController(args.height)
    else:
        ctrl = TrotController(body_height=args.height, period=args.period)

    print(f"\n{'='*68}")
    print(f"  步态: {ctrl.describe()}")
    print(f"{'='*68}")
    header = f"{'t':>6s}  {'leg':6s}  {'hip_urdf°':>10s}  {'calf_urdf°':>10s}  " \
             f"{'hip_motor°':>11s}  {'calf_motor°':>11s}"
    print(header)
    print("-" * 68)

    from marsdog_control.config.joints import JOINT_BY_NAME as JBN
    from marsdog_control.motion.kinematics import motor_to_urdf

    T_dur = args.period if args.gait == "trot" else 1.0
    for i in range(args.steps):
        t = T_dur * i / args.steps
        tgt = ctrl.get_targets(t)

        for leg, hip_name, calf_name in [
            ("FL", "fl_hip_pitch", "fl_calf"),
            ("FR", "fr_hip_pitch", "fr_calf"),
            ("RL", "rl_thigh",     "rl_calf"),
            ("RR", "rr_thigh",     "rr_calf"),
        ]:
            jh = JBN[hip_name]
            jc = JBN[calf_name]
            m_hp = tgt.get(jh.motor_id, 0.0)
            m_ca = tgt.get(jc.motor_id, 0.0)
            u_hp = motor_to_urdf(jh, m_hp)
            u_ca = motor_to_urdf(jc, m_ca)
            print(f"  {t:5.3f}  {leg:6s}  {math.degrees(u_hp):+10.2f}  "
                  f"{math.degrees(u_ca):+10.2f}  "
                  f"{math.degrees(m_hp):+11.2f}  {math.degrees(m_ca):+11.2f}")
        if i < args.steps - 1:
            print()


__all__ = ["TrotController", "TurnTrotController", "SimpleTrot"]
