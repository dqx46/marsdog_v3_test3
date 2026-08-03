"""统一数据结构 — 显式状态机 + 安全层 + 固定控制管线的层间契约。

本模块是纯逻辑, 不依赖硬件/CAN/MuJoCo, 可离线单测。所有实时层(输入/状态机/
规划/安全/执行)只通过这里定义的数据结构交流, 不互相改对象内部变量。

设计原则(对照重构目标):
  - 模块只通过数据结构交流 (IMU 不直接改腿长, 只产出 RobotState 里的姿态)。
  - ControlOutput 从第一天就带 MIT 五元组(位置+kp/kd+力矩前馈), 未来 VMC/WBC
    出力矩时只替换 controller, 不动这条契约。

迁移说明: 这些契约是分层解耦的真正归属地(core 层)。历史扁平模块
``mocap_to_real/robot_types.py`` 现在是指向本模块的同一性别名, 保证新旧代码共享
完全相同的类对象。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Set


class RobotMode(Enum):
    """机器人高层行为模式。方向(前进/后退)不再是独立模式, 收敛为 Direction 字段。"""
    BOOT = "boot"
    STAND = "stand"
    ZEROING = "zeroing"        # 进入 NATURAL 前的合法性守卫态(本轮不做在线站姿热切换)
    TROT = "trot"
    PACE = "pace"
    NATURAL = "natural"        # NaturalTrot 或 NaturalSoftTrot, 具体由启动参数选定的控制器承载
    WALK = "walk"              # NaturalWalk 四拍慢走（与 SoftTrot/Spot 解耦）
    JUMP = "jump"              # JumpController 原地 hop（与 SoftTrot/Walk/Spot 解耦）
    ESTOP = "estop"
    SHUTDOWN = "shutdown"


class Direction(Enum):
    FWD = "fwd"
    BWD = "bwd"


@dataclass
class RobotState:
    """一个控制周期开始时的传感器快照(只读事实, 不含任何决策)。"""
    t: float = 0.0                                   # time.monotonic() 快照
    joint_pos: Dict[int, float] = field(default_factory=dict)     # URDF 帧 rad (Backend 已 /sign)
    joint_enabled: Dict[int, bool] = field(default_factory=dict)
    online: Set[int] = field(default_factory=set)
    # ── IMU (base orientation) ──
    imu_connected: bool = False
    roll: float = 0.0            # rad
    pitch: float = 0.0           # rad
    yaw: float = 0.0             # rad
    gyro_roll: float = 0.0       # rad/s
    gyro_pitch: float = 0.0      # rad/s
    gyro_yaw: float = 0.0        # rad/s
    vel_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0) # (vx, vy, vz) world aligned
    joint_vel: Dict[int, float] = field(default_factory=dict)
    imu_age_s: float = float("inf")   # 上一帧 IMU 数据的年龄; inf=从未收到


@dataclass
class UserCommand:
    """本周期解析出的用户意图(手柄/键盘)。只表达"想干什么", 不直接改 mode。

    调参键(IMU kp / 重力补偿 / 配平 / 体高 / 摆幅 / 步频等)不进这里, 走开发调试旁路,
    因为它们不是上机安全关键路径。
    """
    vx: float = 0.0                              # 手柄前进轴 -1..1 (正=前进); SI 速度由 teleop 策略换算
    turn: float = 0.0                            # 手柄转向轴 -1..1
    request_mode: Optional[RobotMode] = None     # 本周期请求切到的模式(按钮/键沿触发)
    request_dir: Optional[Direction] = None
    pace: bool = False                           # 手柄 dpad 触发 pace 意图
    has_stick: bool = False                      # 手柄在线且本帧提供摇杆量(键盘-only 时 False)
    request_lie_down: bool = False               # 趴下请求(手柄 LT / 键盘 p)
    request_sit: bool = False                    # 坐下请求(键盘 z)
    request_bark: bool = False                   # 狗叫/狗头动作请求(手柄 RT 边沿)
    request_abd_flare_toggle: bool = False       # 站立外展方向验证(键盘 a 切换)
    estop: bool = False                          # 紧急停止
    quit: bool = False                           # 正常退出


@dataclass
class MotionTarget:
    """规划层产出的关节目标(电机帧)。这是"想让每个关节到哪", 尚未过安全层。"""
    q: Dict[int, float] = field(default_factory=dict)     # 电机帧目标 rad
    dq: Dict[int, float] = field(default_factory=dict)    # 速度前馈 rad/s
    source_mode: RobotMode = RobotMode.STAND


@dataclass
class ControlOutput:
    """执行层消费的最终指令 = 安全钳制后的 MotionTarget + 增益上下文。

    target/trq_ff 均为**纯 URDF(pre-sign)空间**; 由 Backend 层在下发前统一施加
    ``urdf * joint.sign`` 与电机物理限位, 因此上层管线不再感知电机方向。

    增益上下文承载达妙主动闸/相位可变阻抗/重力补偿前馈/整体 kp 斜坡, 由执行层
    (send_all)按现有通路解算成每个电机的 MIT 五元组(位置/速度/kp/kd/力矩)。
    """
    target: MotionTarget
    kp_phase: Optional[Dict[int, float]] = None   # 相位可变阻抗: mid->kp缩放
    trq_ff: Optional[Dict[int, float]] = None     # 重力补偿/VMC 前馈力矩(URDF 空间 Nm)
    kp_scale: float = 1.0                          # 整体 kp 斜坡(过渡/软启动)
    leg_kp_scale: float = 1.0                      # 腿部 kp 软化(VMC 开启时常为 0.15)
    dm_active: bool = False                        # 达妙 tarsus 是否主动跟踪(否则固定角)
    gait_active: bool = False                      # 是否处于活动步态(达妙参考前瞻仅步态期启用)
    control_period_s: float = 0.0                  # 本周期控制周期 (s)


@dataclass
class SafetyReport:
    """安全层一次 filter 的结果报告。"""
    ok: bool = True                               # 是否无任何限位/降级触发
    clamped_ids: List[int] = field(default_factory=list)   # 被限位/跳变钳制过的电机
    triggered_estop: bool = False                 # 是否要求上层进入 ESTOP
    imu_degraded: bool = False                    # IMU 时效过期, 上层应停用姿态反馈
    reason: str = ""                              # 触发原因(estop/降级时填)


@dataclass
class MotorSample:
    """Board 层统一电机反馈样本。

    上层只消费 motor-frame rad/rad/s/Nm 和 enabled/fault/timing，不再关心反馈来自
    LZ、EVO、DM、Incos，还是未来 STM32 board。
    """
    motor_id: int
    name: str = ""
    position: float = 0.0
    velocity: float = 0.0
    torque: float = 0.0
    enabled: bool = False
    fault: int = 0
    command_q: Optional[float] = None
    command_dq: Optional[float] = None
    command_kp: Optional[float] = None
    command_kd: Optional[float] = None
    command_tau: Optional[float] = None
    timing: Dict[str, float] = field(default_factory=dict)


@dataclass
class MotorFeedbackFrame:
    """Board.get_feedback() 返回的一帧统一反馈。"""
    t: float = 0.0
    samples: Dict[int, MotorSample] = field(default_factory=dict)

    def positions(self) -> Dict[int, float]:
        return {mid: sample.position for mid, sample in self.samples.items()}

    def enabled(self) -> Dict[int, bool]:
        return {mid: sample.enabled for mid, sample in self.samples.items()}


@dataclass
class MotorCommandFrame:
    """Recorder 用的统一下发快照。"""
    target_q: Dict[int, float] = field(default_factory=dict)
    target_dq: Dict[int, float] = field(default_factory=dict)
    kp: Dict[int, float] = field(default_factory=dict)
    kd: Dict[int, float] = field(default_factory=dict)
    torque_ff: Dict[int, float] = field(default_factory=dict)


# ── 后续接口预留(本轮不实现, 只定签名, 保证未来 VMC/WBC 不动上下游契约) ──

class Controller:
    """控制器抽象基类 — 位置控制/VMC/WBC/MPC 都实现这一个接口。

    位置控制阶段: MotionTarget 直接来自步态; VMC 阶段: state+motion_target->力矩;
    统一收敛到 ControlOutput(已含 MIT 五元组), 因此替换控制器不影响输入/状态机/
    安全/执行层。
    """

    def update(self, state: RobotState, command: UserCommand,
               motion_target: MotionTarget) -> ControlOutput:
        raise NotImplementedError


@dataclass
class BehaviorChannels:
    """行为层并行动作通道预留(本轮不接): 运动/头/尾/喇叭各自独立时间线。"""
    locomotion: Optional[str] = None
    head: Optional[str] = None
    tail: Optional[str] = None
    speaker: Optional[str] = None


__all__ = [
    "RobotMode",
    "Direction",
    "RobotState",
    "UserCommand",
    "MotionTarget",
    "ControlOutput",
    "SafetyReport",
    "MotorSample",
    "MotorFeedbackFrame",
    "MotorCommandFrame",
    "Controller",
    "BehaviorChannels",
]
