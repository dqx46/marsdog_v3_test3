"""Walk CLI: argparse defaults and preset application.

Migrated knobs take ``default=`` from ``marsdog_control.config.defaults.CLI``
(derived from ``RuntimeConfig`` schema — single source of truth).
Unmigrated gait-stack knobs still live here until absorbed into ``GaitStackConfig``.
"""

from __future__ import annotations

import argparse
import sys

from marsdog_control.config.defaults import CLI
from marsdog_control.config.gait_tuning import GAIT
from marsdog_control.config.real_patches import apply_sim_parity
from marsdog_control.motion.gait_recipes import (
    apply_trot_preview_real,
    apply_values,
)


def parse_args():
    p = argparse.ArgumentParser(
        description="Marsdog 稳定步态控制 (StableTrot)",
        allow_abbrev=False,
    )
    # [兼容] 历史主循环归属开关。真实应用已迁入 src/marsdog_control/apps/walk.py,
    # 该参数暂时保留以避免旧脚本/文档调用失败；后续 RuntimePipeline 完全接管后移除。
    p.add_argument("--legacy-loop", action=argparse.BooleanOptionalAction, default=True,
                   help="兼容保留参数: 当前真实 walk 应用已在 src 中运行")
    p.add_argument("--height",      type=float, default=CLI.height,
                   help="体高(m), 默认0.24=新站姿/NaturalSoftTrot 基准(唯一站姿), 方向测试同此")
    p.add_argument("--period",      type=float, default=CLI.period,
                   help="StableTrot 周期 (s), 默认 0.75; SoftTrot 请优先用 --gait-period/--gait-hz")
    p.add_argument(
        "--gait-period", type=float, default=None, metavar="SEC",
        help="[步频·推荐] SoftTrot/Natural 步态周期(秒); 同时写入 period 与 nat_period 并覆盖预设。"
             "例: --gait-period 1.2。不改 gains/recipe 文件，只覆盖本次运行",
    )
    p.add_argument(
        "--gait-hz", type=float, default=None, metavar="HZ",
        help="[步频·别名] SoftTrot/Natural 步频(Hz); 仅换算为 period=1/HZ，与 --gait-period 二选一。"
             "例: --gait-hz 0.833",
    )
    p.add_argument("--step-h",      type=float, default=CLI.step_h,
                   help="后腿抬腿高度 (m), 默认 2cm")
    p.add_argument("--step-h-front", type=float, default=None,
                   help="前腿抬腿高度 (m), 默认 = step_h")
    p.add_argument("--amp-front",   type=float, default=CLI.amp_front,
                   help="前腿半步长 (m), 默认 1.8cm")
    p.add_argument("--amp-rear",    type=float, default=CLI.amp_rear,
                   help="后腿半步长 (m), 默认 2.2cm")
    p.add_argument("--fwd-front-lift", type=float, default=GAIT.fwd_front_lift,
                   help="[前进] 前腿抬腿高度覆盖 (m), 0=用配方默认。抬高→前腿干净离地竖直落地, 不蹭地捣乱")
    p.add_argument("--fwd-front-amp-scale", type=float, default=GAIT.fwd_front_amp_scale,
                   help="[前进] 前腿摆幅缩放, <1 减少前腿推进→让后腿主导推进(前腿以支撑为主)")
    p.add_argument("--yaw-hold", action="store_true",
                   help="[航向保持] 不打方向时自动用yaw反馈纠正跑偏, 让狗走直线(治'走着走着掉头')")
    p.add_argument("--yaw-hold-kp", type=float, default=CLI.yaw_hold_kp,
                   help="[航向保持] 航向角误差增益 (turn_cmd/度), 默认0.03")
    p.add_argument("--yaw-hold-kd", type=float, default=CLI.yaw_hold_kd,
                   help="[航向保持] yaw角速度阻尼增益 (turn_cmd/(度/s)), 默认0.010")
    p.add_argument("--yaw-hold-sign", type=float, default=1.0,
                   help="[航向保持] 符号, 若纠反了(越纠越偏/转圈)改成 -1")
    p.add_argument("--yaw-hold-limit", type=float, default=CLI.yaw_hold_limit,
                   help="[航向保持] turn_cmd 自动修正上限, 默认0.4 (防止搞反时猛转)")
    # [转向独立层] 以下只影响转向, 完全不动前进步态参数(解耦: 调转向不用重调前进)
    p.add_argument("--turn-amp-diff", type=float, default=GAIT.turn_amp_diff,
                   help="[转向] 左右步幅差 (m), 越大转越急, 默认0.020")
    p.add_argument("--turn-y-amp", type=float, default=GAIT.turn_y_amp,
                   help="[转向] 转向跨步(髋外展)幅度 (m), 默认0.025")
    p.add_argument("--turn-smooth", type=float, default=GAIT.turn_smooth,
                   help="[转向] 转向指令低通系数, 越小越柔和不猛甩, 默认0.015")
    p.add_argument("--turn-waist-yaw", type=float, default=GAIT.turn_waist_yaw,
                   help="[转向] 转向时腰部扭曲最大角 (rad), 默认0.35")
    p.add_argument("--cruise-turn-scale", type=float, default=GAIT.cruise_turn_scale,
                   help="[走+转] 边走边转时的转向权限缩放(相对原地转), 越大转得越急, "
                        "默认0.6; 原地转(只右摇杆)不受影响")
    p.add_argument("--cruise-vx", type=float, default=0.100,
                   help="[前进] 巡航速度 m/s (SI)。摇杆只做走/停: 过阈值后固定用该速度 "
                        "(与仿真 --vx 对齐, 默认 0.10≈新菜谱中速; 满幅约 0.13); "
                        "推杆深浅不改步态。半速试: --cruise-vx 0.067")
    p.add_argument("--cruise-turn-yamp", type=float, default=GAIT.cruise_turn_yamp,
                   help="[走+转] 边走边转的横向跨步(蟹步)增益, hip改线修正后蟹步与差速同向"
                        "帮助转向, 默认1.0; 若仍斜行可降到0试纯差速")
    p.add_argument("--turn-sign", type=float, default=GAIT.turn_sign,
                   help="[转向] 全局左右方向符号(硬件改线后如左右反了改成-1), 默认1")
    p.add_argument("--waist-yaw-turn-sign", type=float, default=GAIT.waist_yaw_turn_sign,
                   help="[转向] 腰扭转方向符号, 若腰反向抵消转向改成-1, 默认1")
    p.add_argument("--stance",      type=float, default=CLI.stance,
                   help="支撑相占空比 duty∈(0,1), SoftTrot 默认随预设(~0.56);"
                        "显式传入则覆盖预设(例: --stance 0.36 飞跃相更长)。不改 recipe 文件")
    p.add_argument("--hip-abd",     type=float, default=CLI.hip_abd,
                   help="静态髋外展角 (rad), 默认 0.08 (~4.6°)")
    p.add_argument("--hip-abd-test", type=float, default=None, metavar="RAD",
                   help="仅测试四个髋外展轴的方向: 用主控 StandController 的 hip_abd=RAD "
                        "生成 ID 2/6/9/12 目标，其余关节冻结在启动读数；建议 0.08")
    p.add_argument("--leg-pitch-test", type=float, default=None, metavar="RAD",
                   help="仅测试大腿前后摆方向: 前腿大腿向前、后腿大腿向后；通过主控 "
                        "urdf_to_motor() 生成 ID 1/5/10/13 目标，其余关节冻结；建议 0.20")
    p.add_argument("--calf-pitch-test", type=float, default=None, metavar="RAD",
                   help="仅测试四个小腿前摆方向: 前/后腿小腿都向前；通过主控 "
                        "urdf_to_motor() 生成 ID 3/7/11/14 目标，其余关节冻结；建议 0.20")
    p.add_argument("--capture-lie-pose", action="store_true",
                   help="只读取当前电机位置并保存为 lie_down_pose.json(排除头部和脖子), 不起立不走路")
    p.add_argument("--waist-pitch", type=float, default=GAIT.waist_pitch,
                   help="腰部弓背角度 (rad)")
    p.add_argument("--waist-yaw-offset", type=float, default=GAIT.waist_yaw_offset)
    p.add_argument("--bwd-amp-scale", type=float, default=GAIT.bwd_amp_scale,
                   help="后退步幅缩放系数, 默认 0.7")
    p.add_argument("--bwd-step-h",  type=float, default=GAIT.bwd_step_h,
                   help="后退抬腿高度 (m), 默认 1.5cm")
    p.add_argument("--front-thrust-gain", type=float, default=GAIT.front_thrust_gain,
                   help="前腿推力增益: 放大大腿(hip_pitch)摆动, 小腿近似刚性只调高度 "
                        "(1.0=原全IK协调, >1=大腿出力更多; 默认 1.0 回退原版协调)")
    p.add_argument("--front-thrust-swing-gain", type=float, default=GAIT.front_thrust_swing_gain,
                   help="[摆动相] 前腿大腿增益, 默认1=全IK干净抬腿; 支撑相仍用 front-thrust-gain")
    p.add_argument("--front-tarsus-push", type=float, default=GAIT.front_tarsus_push,
                   help="[旧法] 前腿脚踝支撑相 sin 蹬地幅度(rad); 仅足朝向跟踪关闭时生效")
    p.add_argument("--front-foot-track-deg", type=float, default=GAIT.front_foot_track_deg,
                   help="前腿足朝向跟踪目标(deg, 足段绝对角), -78=新模型默认(与站姿配套); 设 0/None 关闭")
    p.add_argument("--front-foot-stance-push-deg", type=float, default=GAIT.front_foot_stance_push_deg,
                   help="足朝向模式下支撑相额外前倾蹬地度数(deg), 摆动相回中性")
    p.add_argument("--front-foot-swing-track", type=float, default=GAIT.front_foot_swing_track,
                   help="摆动相保留多少足朝向跟踪[0,1]; 0=摆动回站立朝向(减tarsus甩动), 1=全程跟踪")
    p.add_argument("--front-stand-tarsus-deg", type=float, default=GAIT.front_stand_tarsus_deg,
                   help="足朝向 ramp 起点对应的站立 tarsus 角度(deg); NaturalTrot 推荐 8")
    p.add_argument("--front-stand-foot-pitch-deg", type=float, default=GAIT.front_stand_foot_pitch_deg,
                   help="新站姿脚段绝对朝向(deg); -90=脚尖竖直指地(唯一站姿, 与足朝向跟踪配套)")
    p.add_argument("--swing-clearance-per-rad", type=float, default=GAIT.swing_clearance_per_rad,
                   help="摆动相 roll 低侧额外抬腿: z+=roll_low×系数×体高 (0=关, 默认0.35)")
    p.add_argument("--reactive-kp", type=float, default=GAIT.reactive_kp,
                   help="Raibert 反应式落脚点比例增益 (默认 0.0, 开启需确保仅作用于摆动腿)")
    p.add_argument("--reactive-kd", type=float, default=GAIT.reactive_kd,
                   help="Raibert 反应式落脚点微分增益 (默认 0.0, 避免gyro噪声导致抽搐)")
    p.add_argument("--roll-trim-mm", type=float, default=CLI.roll_trim_mm,
                   help="[T] roll 静态配平 (mm), 抵消步态直流姿态偏置; +=补偿右歪, 热键 k/l 调 (默认0)")
    p.add_argument("--pitch-trim-mm", type=float, default=CLI.pitch_trim_mm,
                   help="[T] pitch 静态配平 (mm), 抵消俯仰直流偏置 (默认0)")
    p.add_argument("--auto-trim", action=argparse.BooleanOptionalAction, default=False,
                   help="[AT·已移除] 整机调平学习已删除; 此开关保留兼容, 开启也会被忽略")
    p.add_argument("--auto-trim-rate", type=float, default=CLI.auto_trim_rate,
                   help="[AT·已移除] 兼容保留, 无效果")
    p.add_argument("--auto-trim-limit-mm", type=float, default=CLI.auto_trim_limit_mm,
                   help="[AT·已移除] 兼容保留, 无效果")
    p.add_argument("--trim-phases", type=int, default=CLI.trim_phases,
                   help="[AT·已移除] 兼容保留, 无效果")
    p.add_argument("--imu-predict-ms", type=float, default=CLI.imu_predict_ms,
                   help="[PRED] 执行器额外提前量(ms); SoftTrot 预设为 0; "
                        "总预测还会加当前 angle age")
    p.add_argument("--imu-predict-max-ms", type=float, default=CLI.imu_predict_max_ms,
                   help="[PRED] 数据年龄+执行提前量的总上限(ms), 默认80")
    p.add_argument("--imu-gyro-max-age-ms", type=float, default=CLI.imu_gyro_max_age_ms,
                   help="[PRED] gyro 超过该年龄时禁止外推, 默认30ms")
    p.add_argument("--dynamic-imu-predict", action=argparse.BooleanOptionalAction,
                   default=CLI.dynamic_imu_predict, help="[PRED] 总预测自动包含 IMU angle 数据年龄, 默认开")
    p.add_argument("--imu-angle-tau-ms", type=float, default=CLI.imu_angle_tau_ms,
                   help="[IMU] angle 一阶低通时间常数(ms), 默认25")
    p.add_argument("--imu-gyro-tau-ms", type=float, default=CLI.imu_gyro_tau_ms,
                   help="[IMU] gyro 一阶低通时间常数(ms), 默认15")
    p.add_argument("--imu-kp", type=float, default=CLI.imu_kp,
                   help="[IMU] 覆盖姿态 P 增益 kp(roll&pitch), 0=用内置默认0.03; 配合预测可抬到0.05-0.08")
    p.add_argument("--imu-softstart-s", type=float, default=CLI.imu_softstart_s,
                   help="[SS] IMU修正软启动: 步态启动/重触发后, 修正权限在该时间内0→1平滑拉起 "
                        "(SoftTrot/schema 默认 0=关; 非零时消除起步'起飞')")
    p.add_argument("--bwd-period",  type=float, default=GAIT.bwd_period,
                   help="后退步态周期 (s), 默认 0.85")
    p.add_argument("--fwd-use-bwd", action=argparse.BooleanOptionalAction, default=GAIT.fwd_use_bwd,
                   help="前进套用后退配方(周期/抬腿/前后幅度分布/+髋外展), 仅方向朝前, 更稳; "
                        "默认开, --no-fwd-use-bwd 关")
    p.add_argument("--lateral-sway", type=float, default=GAIT.lateral_sway,
                   help="横向重心摆动幅度 (m), 半正弦旧法; SoftTrot 有 --com-shift 时忽略")
    p.add_argument("--com-shift", type=float, default=GAIT.com_shift_m,
                   dest="com_shift_m", metavar="M",
                   help="[位控·质心] SoftTrot 横向移重 (m); NATURAL_SOFT_TROT 默认 0.012, 0=关; "
                        "正=FL+RR→右; --sim-parity 会关掉")
    p.add_argument("--com-shift-blend", type=float, default=GAIT.com_shift_blend,
                   dest="com_shift_blend", metavar="PHASE",
                   help="[位控·质心] 对角换腿 smoothstep 半宽 (相位 0~0.15), 默认随 GAIT")
    p.add_argument("--rear-clearance", type=float, default=GAIT.rear_clearance_m,
                   dest="rear_clearance_m", metavar="M",
                   help="[补丁] 后腿摆动额外净空 (m); 默认 0=关, 优先调 step_h")
    p.add_argument("--pace-period", type=float, default=GAIT.pace_period,
                   help="Pace 步态周期 (s), 默认 1.2 (慢速保稳)")
    p.add_argument("--pace-stance", type=float, default=GAIT.pace_stance,
                   help="Pace 步态站立比, 默认 0.75 (50%%双支撑)")
    p.add_argument("--pace-sway",   type=float, default=GAIT.pace_sway,
                   help="Pace 步态横向重心摆动幅度 (m), 默认 15mm")
    p.add_argument("--pace-amp",    type=float, default=GAIT.pace_amp,
                   help="Pace 步态前后步幅 (m), 默认 8mm (小步)")
    p.add_argument("--pace-step-h", type=float, default=GAIT.pace_step_h,
                   help="Pace 步态抬腿高度 (m), 默认 15mm")
    p.add_argument("--pace-hip-abd", type=float, default=GAIT.pace_hip_abd,
                   help="Pace 步态髋关节外展角 (rad), 默认 0.00")
    p.add_argument("--fade",        type=float, default=CLI.fade)
    p.add_argument("--ramp",        type=float, default=CLI.ramp,
                   help="步态启动振幅斜坡时间 (s), 默认 2.0")
    p.add_argument("--trot",        action="store_true")
    p.add_argument("--natural-trot", action="store_true",
                   help="启动自然 Trot (仿生步态: 膝折叠+水滴轨迹+脊柱律动); 运行中也可按 3 切换")
    p.add_argument("--natural-soft-trot", action=argparse.BooleanOptionalAction,
                   default=CLI.natural_soft_trot,
                   help="启动低冲击自然 Trot；正式默认开启，可用 --no-natural-soft-trot 临时关闭")
    p.add_argument("--natural-walk", action="store_true",
                   help="四拍真狗慢走(NaturalWalk)；与 SoftTrot/Spot 解耦，默认仍 SoftTrot")
    p.add_argument("--jump", action="store_true",
                   help="原地 hop(Jump)；与 SoftTrot/Walk/Spot 解耦；覆盖 --natural-walk")
    # 达妙无掉电零点记忆: 约定每次上电前手动掰到硬限位物理基准。不再用 CLI 开关确认。
    p.add_argument("--dm-kp-fl", type=float, default=CLI.dm_kp_fl,
                   help="[tarsus] 左前达妙 KP，默认220")
    p.add_argument("--dm-kp-fr", type=float, default=CLI.dm_kp_fr,
                   help="[tarsus] 右前达妙 KP，默认220")
    p.add_argument("--dm-kd-fl", type=float, default=CLI.dm_kd_fl,
                   help="[tarsus] 左前达妙 KD，默认10")
    p.add_argument("--dm-kd-fr", type=float, default=CLI.dm_kd_fr,
                   help="[tarsus] 右前达妙 KD，默认10")
    p.add_argument("--tarsus-lead-fl-ms", type=float, default=CLI.tarsus_lead_fl_ms,
                   help="[tarsus] 左前目标参考超前(ms)，正式 NaturalSoftTrot 默认40")
    p.add_argument("--tarsus-lead-fr-ms", type=float, default=CLI.tarsus_lead_fr_ms,
                   help="[tarsus] 右前目标参考超前(ms)，正式 NaturalSoftTrot 默认50")
    p.add_argument("--tarsus-lead-max-deg", type=float, default=CLI.tarsus_lead_max_deg,
                   help="[tarsus] 单周期参考超前最大角度，防止相位/IMU突变被线性外推放大")
    p.add_argument("--dm-dq-feedforward", action=argparse.BooleanOptionalAction,
                   default=CLI.dm_dq_feedforward, help="[tarsus] 向达妙发送轨迹速度前馈，默认开")
    p.add_argument("--dm-dq-max-rps", type=float, default=CLI.dm_dq_max_rps,
                   help="[tarsus] 达妙速度前馈绝对值上限(rad/s)，配合KD避免瞬态力矩饱和")
    p.add_argument("--bench-tarsus-side", choices=("fl", "fr"),
                   help="[地面测试] 站姿下仅对指定 tarsus 做±小角度正弦扫频")
    p.add_argument("--bench-tarsus-amp-deg", type=float, default=2.0)
    p.add_argument("--bench-tarsus-frequencies", default="0.25,0.5,1.0,2.0",
                   help="逗号分隔Hz；每档之间自动回中等待")
    p.add_argument("--bench-tarsus-cycles", type=float, default=3.0)
    p.add_argument("--bench-tarsus-settle-s", type=float, default=2.0)
    p.add_argument("--bench-max-error-deg", type=float, default=CLI.bench_max_error_deg)
    p.add_argument("--bench-max-tilt-deg", type=float, default=CLI.bench_max_tilt_deg)
    p.add_argument("--bench-max-torque-nm", type=float, default=CLI.bench_max_torque_nm)
    p.add_argument("--no-spine", action="store_true",
                   help="[NaturalTrot] 关闭脊柱律动(spine_yaw/roll=0), 首次上机安全验证用")
    p.add_argument("--trot-preview", action="store_true",
                   help="应用 MuJoCo sim-preview --trot 验证配方 (步态+IMU; 保留柔顺/重力补偿)")
    # ── NaturalTrot/NaturalSoftTrot 形状; SoftTrot 未显式给时灌 NATURAL_SOFT_TROT ──
    p.add_argument("--nat-period", type=float, default=GAIT.nat_period,
                   help="Natural/SoftTrot 周期 (s); SoftTrot 推荐改用 --gait-period/--gait-hz")
    p.add_argument("--nat-amp-front", type=float, default=GAIT.nat_amp_front)
    p.add_argument("--nat-amp-rear", type=float, default=GAIT.nat_amp_rear)
    p.add_argument("--nat-step-h", type=float, default=GAIT.nat_step_h)
    p.add_argument("--spine-yaw-deg", type=float, default=GAIT.spine_yaw_deg)
    p.add_argument("--spine-roll-deg", type=float, default=GAIT.spine_roll_deg)
    p.add_argument("--spine-phase-deg", type=float, default=GAIT.spine_phase_deg)
    p.add_argument("--thigh-swing-front-deg", type=float, default=GAIT.thigh_swing_front_deg)
    p.add_argument("--thigh-swing-rear-deg", type=float, default=GAIT.thigh_swing_rear_deg)
    p.add_argument("--retract-front", type=float, default=GAIT.retract_front)
    p.add_argument("--retract-rear", type=float, default=GAIT.retract_rear)
    p.add_argument("--tarsus-swing-deg", type=float, default=GAIT.tarsus_swing_deg)
    p.add_argument("--no-gamepad",  action="store_true")
    p.add_argument("--no-log",      action="store_true")
    p.add_argument("--scope", action="store_true",
                   help="启动日志软件示波器(独立进程读取CSV, 不进入控制环); 输出 log/scope_live.html")
    p.add_argument("--scope-window", type=float, default=6.0,
                   help="软件示波器显示最近多少秒, 默认6")
    p.add_argument("--scope-refresh", type=float, default=0.5,
                   help="软件示波器刷新周期(秒), 默认0.5")
    p.add_argument("--scope-motors", default="3,7",
                   help="软件示波器通道 motor id, 逗号分隔, 默认3,7")
    p.add_argument("--no-tail",     action="store_true",
                   help="禁用尾巴后台动作控制")
    p.add_argument("--imu",         action="store_true",
                   help="启用 IMU 闭环足高补偿(TEMP: 当前默认关闭, 需显式打开)")
    p.add_argument("--imu-test",    action="store_true",
                   help="IMU 补偿验证模式: 放大增益, stand 下倾斜可见腿部反应")
    # ── Phase1-3 单变量实验开关 (默认全关, 保持黄金基线; 每次只开一个做对照) ──
    p.add_argument("--no-imu",      action="store_true",
                   help="[P1] 强制关闭 IMU 闭环(仍记录IMU); TEMP 下默认已关, 此开关冗余")
    p.add_argument("--abd-legacy",  action="store_true",
                   help="[P1] 反转髋外展方向(fl_thigh_roll/rl_hip/rr_hip)回修正前, A/B 验证外展方向")
    p.add_argument("--swing-level", type=float, default=GAIT.swing_level,
                   help="[P2] 摆动腿 IMU 预调平权重 0~1 (默认0=仅支撑腿; >0 让摆动腿落脚点也随姿态纠正)")
    p.add_argument("--imu-ema",     type=float, default=GAIT.imu_ema,
                   help="[P2] IMU 角度额外 EMA 滤波系数 0~1 (默认0=关; D 项改用滤波后角速度去抖)")
    p.add_argument("--ff-decouple", action=argparse.BooleanOptionalAction,
                   default=CLI.ff_decouple,
                   help="[P2] expected_roll/pitch 前馈解耦; SoftTrot 预设常开, "
                        "--no-ff-decouple 关; --sim-parity 默认关")
    p.add_argument("--leg-kp-scale", type=float, default=CLI.leg_kp_scale,
                   help="[临时] kp 叠加缩放(默认1.0)。SoftTrot 请改 config/gains.py 分品牌表，"
                        "勿再用本参数做全局软化；跳步阶段仍可能临时改写")
    p.add_argument("--x-shift",     type=float, default=GAIT.x_shift,
                   help="四脚落脚点整体X偏移(m); 正值=脚向前, 等效重心后移 "
                        f"(默认 {GAIT.x_shift})")
    # ── A: 线性油门 → 步幅缩放 ──
    p.add_argument("--throttle-min-scale", type=float, default=GAIT.throttle_min_scale,
                   help="[A] 线性油门: 轻推时的最小步幅比例 (默认0.5; 设1.0=恒定满步幅=旧行为)")
    # ── B: 非线性阻尼 (压落脚振铃) ──
    p.add_argument("--damp-hard-mm", type=float, default=GAIT.damp_hard_mm,
                   help="[B] 大角速度时的硬阻尼限幅(mm), 默认3=关(=软限幅); 设10~14 开启压振铃")
    p.add_argument("--damp-gyro-lo", type=float, default=GAIT.damp_gyro_lo,
                   help="[B] 软区角速度上限(deg/s), 以下用软限幅不抽腿 (默认20)")
    p.add_argument("--damp-gyro-hi", type=float, default=GAIT.damp_gyro_hi,
                   help="[B] 硬区角速度下限(deg/s), 以上用硬限幅压振铃 (默认80)")
    # ── C: 平滑步态 (匀速支撑相 + C1 摆动, 消除"一冲一冲"顿挫) ──
    p.add_argument("--smooth-gait", action="store_true",
                   help="[C] 支撑相匀速+摆动Hermite速度匹配: 消除顿挫, 身体匀速前进不打滑")
    p.add_argument("--anti-roll",   type=float, default=GAIT.anti_roll,
                   help="[C] 支撑相中期主动伸腿量(m), 默认0.003; 设0可去掉上下顶地颠簸")
    p.add_argument("--trot-roll-ff-neg-deg", type=float, default=GAIT.trot_roll_ff_neg_deg,
                   help="对角 Trot 预期 roll 负峰 (度), ff_decouple 用; trot-preview 默认 3.0")
    p.add_argument("--trot-roll-ff-pos-deg", type=float, default=GAIT.trot_roll_ff_pos_deg,
                   help="对角 Trot 预期 roll 正峰 (度); 0=自动取 neg×0.55")
    p.add_argument("--anti-roll-asym-neg", type=float, default=GAIT.anti_roll_asym_neg,
                   help="FL+RR 支撑相 anti_roll 缩放 (trot-preview 默认 1.30)")
    p.add_argument("--anti-roll-asym-pos", type=float, default=GAIT.anti_roll_asym_pos,
                   help="FR+RL 支撑相 anti_roll 缩放 (trot-preview 默认 0.85)")
    p.add_argument("--imu-phase-gate", action=argparse.BooleanOptionalAction, default=CLI.imu_phase_gate,
                   help="[F] 步态相位门控 IMU: 触地/离地降增益, 支撑中期全力闭环 (默认开)")
    p.add_argument("--imu-phase-td-gain", type=float, default=CLI.imu_phase_td_gain,
                   help="[F] 触地/离地窗口 IMU 增益 (默认 0.35)")
    p.add_argument("--imu-phase-swing-gain", type=float, default=CLI.imu_phase_swing_gain,
                   help="[F] 摆动相 IMU 增益 (默认 0.70)")
    # ── D: roll P 增益调度 (救间歇发散) ──
    p.add_argument("--roll-p-boost", type=float, default=GAIT.roll_p_boost,
                   help="[D] 大倾角P增益放大倍数, 默认1.0=关; 设2~3 开启, 救间歇发散")
    p.add_argument("--roll-p-lo-deg", type=float, default=GAIT.roll_p_lo_deg,
                   help="[D] P增益开始放大的倾角(deg), 以下保持温和 (默认6)")
    p.add_argument("--roll-p-hi-deg", type=float, default=GAIT.roll_p_hi_deg,
                   help="[D] P增益达到最大放大的倾角(deg) (默认14)")
    p.add_argument("--max-corr-mm",  type=float, default=CLI.max_corr_mm,
                   help="[D] IMU 最大补偿限幅(mm), 默认20; 增益调度救发散时可提到30")
    # ── E: 落地冲击抑制 (IMU 机制保护) ──
    p.add_argument("--td-imu-freeze-i", action=argparse.BooleanOptionalAction,
                   default=False,
                   help="[E] 触地窗口冻结IMU积分; SoftTrot 预设默认关; "
                        "--td-imu-freeze-i 开; --sim-parity 亦关")
    p.add_argument("--imu-slew-mm-s", type=float, default=CLI.imu_slew_mm_s,
                   help="[E] IMU校正斜率限制(mm/s), 0=关闭; 建议 120~300")
    p.add_argument("--load-trim-cal", action=argparse.BooleanOptionalAction,
                   default=False,
                   help="[AT·已移除] 不再加载 trim_cal.json; 开关保留兼容")
    p.add_argument("--save-trim-cal", action=argparse.BooleanOptionalAction,
                   default=False,
                   help="[AT·已移除] 不再保存 trim_cal.json; 开关保留兼容")
    p.add_argument(
        "--sim-parity", action=argparse.BooleanOptionalAction, default=False,
        help="[研究] 关闭叠加补偿: IMU门控/预测/软启动、达妙 lead/dq_ff、"
             "anti_roll/sway/roll_ff、com_shift/spine/rear_clearance/"
             "flourish 等。保留 period/amp/step_h/touchdown_compress。"
             "单项可用显式 --flag 再打开做 A/B",
    )
    # ── A(柔顺): 相位可变阻抗 (落地降kp吸震, 支撑中期撑体重) ──
    p.add_argument("--var-impedance", action=argparse.BooleanOptionalAction, default=CLI.var_impedance,
                   help="[柔顺A] 相位可变阻抗: 触地窗口降腿部kp软着陆, 支撑中期恢复1.0, 摆动中等; 正式 NaturalSoftTrot 默认关, --var-impedance 开")
    p.add_argument("--td-kp-scale",   type=float, default=CLI.td_kp_scale,
                   help="[柔顺A] 触地窗口 kp 比例 (默认0.4; 越小越软, 塌腿则调高)")
    p.add_argument("--swing-kp-scale", type=float, default=CLI.swing_kp_scale,
                   help="[柔顺A] 摆动相 kp 比例 (默认0.7)")
    p.add_argument("--td-window",     type=float, default=CLI.td_window,
                   help="[柔顺A] 触地软化窗口相位宽度 (默认0.15)")
    # ── B(柔顺): 重力补偿前馈 (让整体降 kp 可持续) ──
    p.add_argument("--gravity-comp",  action=argparse.BooleanOptionalAction, default=CLI.gravity_comp,
                   help="[柔顺B] 腿部重力补偿前馈: 按关节角算 τ_g 写入 trq_ff, 替换静态值; 默认开, --no-gravity-comp 关")
    p.add_argument("--vmc", action=argparse.BooleanOptionalAction, default=CLI.vmc,
                   help="[实验] 解耦 VMC: 启用 Z轴与 Roll 轴解析雅可比虚拟模型控制，启用时覆盖 gravity-comp")
    p.add_argument("--wbc", action=argparse.BooleanOptionalAction, default=False,
                   help="[实验] WBC+SRB-MPC: Pinocchio reduced 模型全身力控，覆盖 VMC/gravity-comp")
    p.add_argument("--grav-scale",    type=float, default=CLI.grav_scale,
                   help="[柔顺B] 重力补偿整体缩放 (默认0.5保守; 静态验证正确后逼近1.0)")
    # ── Dynamics (MPC+WBC) ──
    from marsdog_control.config.schema import DynamicsConfig as _Dyn
    _DYN = _Dyn()
    p.add_argument("--wbc-mu", type=float, default=_DYN.mu,
                   help="[动力学] 摩擦锥 μ (WBC/MPC 统一)")
    p.add_argument("--wbc-f-min", type=float, default=_DYN.f_min,
                   help="[动力学] 支撑足最小法向力 N")
    p.add_argument("--wbc-f-max", type=float, default=_DYN.f_max,
                   help="[动力学] 单足最大法向力 N (~3*mg/4)")
    p.add_argument("--mpc-horizon", type=int, default=_DYN.mpc_horizon,
                   help="[动力学] SRB-MPC 预测步数")
    p.add_argument("--mpc-dt", type=float, default=_DYN.mpc_dt,
                   help="[动力学] SRB-MPC 单步时间 s")
    p.add_argument("--wbc-tau-limit", type=float, default=_DYN.tau_limit_nm,
                   help="[动力学] WBC 关节力矩限幅 Nm")
    p.add_argument("--wbc-tau-scale", type=float, default=_DYN.wbc_tau_scale,
                   help="[动力学] WBC 关节力矩输出增益 (默认0.5, QP限幅后乘)")
    p.add_argument("--kp-base-z", type=float, default=_DYN.kp_base_z)
    p.add_argument("--kd-base-z", type=float, default=_DYN.kd_base_z)
    p.add_argument("--kp-base-roll", type=float, default=_DYN.kp_base_roll)
    p.add_argument("--kd-base-roll", type=float, default=_DYN.kd_base_roll)
    p.add_argument("--kp-base-pitch", type=float, default=_DYN.kp_base_pitch)
    p.add_argument("--kd-base-pitch", type=float, default=_DYN.kd_base_pitch)
    p.add_argument(
        "--base-estimate-mode",
        choices=("truth", "estimator"),
        default=_DYN.base_estimate_mode,
        help="[动力学] 基座速度: estimator=支撑足+IMU(默认/实机), truth=仿真真值(调试)",
    )
    p.add_argument("--swing-foot-kp", type=float, default=_DYN.swing_foot_kp,
                   help="[动力学] WBC 摆动足笛卡尔 PD kp")
    p.add_argument("--swing-foot-kd", type=float, default=_DYN.swing_foot_kd,
                   help="[动力学] WBC 摆动足笛卡尔 PD kd")
    p.add_argument("--urdf-path", type=str, default=_DYN.urdf_path,
                   help="[动力学] Pinocchio URDF 路径")
    explicit_dests = set()
    for token in sys.argv[1:]:
        if token.startswith("--"):
            action = p._option_string_actions.get(token.split("=", 1)[0])
            if action is not None:
                explicit_dests.add(action.dest)
    args = p.parse_args()
    args._explicit_cli = explicit_dests
    # sim-parity before cadence/presets so SoftTrot recipe cannot revive patches.
    apply_sim_parity(args)
    apply_gait_cadence_cli(args)
    if args.trot_preview:
        explicit_values = {
            key: getattr(args, key)
            for key in getattr(args, "_explicit_cli", set())
            if hasattr(args, key)
        }
        apply_trot_preview_real(args)
        for key, value in explicit_values.items():
            setattr(args, key, value)
    if args.natural_soft_trot:
        args.natural_trot = True
    return args


def apply_gait_cadence_cli(args):
    """Resolve ``--gait-period`` / ``--gait-hz`` into ``period`` + ``nat_period``.

    Marks both as explicit so SoftTrot presets cannot overwrite them.
    Returns the resolved period in seconds, or ``None`` if unused.
    """
    explicit = set(getattr(args, "_explicit_cli", set()))
    has_period = "gait_period" in explicit and getattr(args, "gait_period", None) is not None
    has_hz = "gait_hz" in explicit and getattr(args, "gait_hz", None) is not None
    if has_period and has_hz:
        raise SystemExit("请只使用 --gait-period 或 --gait-hz 之一")

    period = None
    if has_hz:
        hz = float(args.gait_hz)
        if hz <= 0.0:
            raise SystemExit("--gait-hz 必须 > 0")
        period = 1.0 / hz
    elif has_period:
        period = float(args.gait_period)
        if period <= 0.0:
            raise SystemExit("--gait-period 必须 > 0")

    if period is None:
        return None

    args.period = float(period)
    args.nat_period = float(period)
    args.gait_period = float(period)
    args.gait_hz = 1.0 / float(period)
    explicit |= {"period", "nat_period", "gait_period", "gait_hz"}
    args._explicit_cli = explicit
    return float(period)


def apply_preset_preserving_cli(args, values):
    """应用预设，但显式 CLI 对应的 dest 永远拥有最终优先级。

    Also syncs ``values`` (recipe dict) back from final ``args``. SoftTrot
    builders prefer ``natural_params["period"]`` over ``cfg.nat_period``; without
    this sync, ``--gait-period`` would update args/banner but leave ``nat_fwd``
    stuck on the recipe period.
    """
    explicit = {
        key: getattr(args, key)
        for key in getattr(args, "_explicit_cli", set())
        if hasattr(args, key)
    }
    apply_values(args, values)
    for key, value in explicit.items():
        setattr(args, key, value)
    for key in list(values.keys()):
        if hasattr(args, key):
            values[key] = getattr(args, key)
    # SoftTrot/Natural builders read recipe["period"]; follow nat_period.
    if "period" in values and hasattr(args, "nat_period"):
        values["period"] = float(args.nat_period)
    if "nat_period" in values and hasattr(args, "nat_period"):
        values["nat_period"] = float(args.nat_period)
    return sorted(explicit)
