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

"""Gait controllers — package-local imports (no legacy sys.path)."""
import math
from marsdog_control.motion.kinematics import (
    compute_standing_pose_3link,
    front_standing_foot_pitch,
    fk_front_2d, fk_rear_2d,
    WAIST_Z, FL_HIP_Z, RL_HIP_Z,
)
from marsdog_control.config.joints import JOINT_BY_NAME

_FRONT_HIP_OFFSET = abs(WAIST_Z + FL_HIP_Z)   # 0.031 m
_REAR_HIP_OFFSET  = abs(RL_HIP_Z)              # 0.015 m

_FRONT_X0 = fk_front_2d(0.0, 0.0)[0]
_REAR_X0  = fk_rear_2d(0.0, 0.0)[0]


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

    _lateral_planner = None
    _attitude_overlay_gate = None

    def __init__(self, lateral_planner=None, attitude_overlay_gate=None):
        self._lateral_planner = lateral_planner
        self._attitude_overlay_gate = attitude_overlay_gate

    def bind_ownership(self, *, lateral_planner, attitude_gate):
        self._lateral_planner = lateral_planner
        self._attitude_overlay_gate = attitude_gate

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
                 *,
                 lateral_planner=None,
                 attitude_overlay_gate=None):
        super().__init__(
            lateral_planner=lateral_planner,
            attitude_overlay_gate=attitude_overlay_gate,
        )
        self.body_height = body_height
        self.x_offset_front = x_offset_front if x_offset_front is not None else _FRONT_X0
        self.x_offset_rear = x_offset_rear if x_offset_rear is not None else _REAR_X0
        self.hip_abduction = hip_abduction
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


