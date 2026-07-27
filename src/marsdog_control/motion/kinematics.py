# [解耦] 真实实现已从 mocap_to_real 下沉到此 src 模块; 保持与旧代码逐字一致的扁平
# import, 由 ensure_legacy_path() 保证 mocap_to_real 在 sys.path 上可解析(其 compat
# 别名回指本 src 包, 单一模块实体, 不产生第二份副本)。
from marsdog_control.compat import ensure_legacy_path as _ensure_legacy_path
_ensure_legacy_path()

import math
from marsdog_control.config.joints import JOINT_MAP, JOINT_BY_NAME as JBN

# ─── URDF 几何常量 ──────────────────────────────────────────────────────────
# 前腿 hip 相对 body 的 Z 偏移
WAIST_Z = 0.055
FL_HIP_Z = -0.086
# 后腿 hip 相对 body 的 Z 偏移
RL_HIP_Z = -0.015

# 前腿等效 2-link 参数 (base 偏移已合入 L1)
# 2026-07-08 结构变更: 大腿(thigh_roll→calf)沿轴 +20mm, 小腿(calf→tarsus)沿轴 -12mm, 脚掌不变。
# L1 = hip→thigh_roll(-0.0205, 0.000038) + thigh_roll→calf(0.029858, -0.088934)
_FL_L1_VEC = (0.009358, -0.088896)
_FL_L1 = math.hypot(*_FL_L1_VEC)               # 0.0894 m (旧0.0700, hip偏移几何投影→有效+19.4mm)
_FL_PHI1 = math.atan2(_FL_L1_VEC[1], _FL_L1_VEC[0])  # -84.0°
# L2 (combined) = calf→tarsus(0.003643, -0.109949) + tarsus→foot(0, -0.051948)
# tarsus 焊死(=0) 时的等效第二段, 保留给 2-link 兼容路径 fk_front_2d/ik_front_leg_2d。
_FL_L2_VEC = (0.003643, -0.161897)
_FL_L2 = math.hypot(*_FL_L2_VEC)               # 0.1619 m (旧0.1739, -12.0mm)
_FL_PHI2 = math.atan2(_FL_L2_VEC[1], _FL_L2_VEC[0])  # -88.7°

# 前腿 3-link 拆分 (2026-07 tarsus 解锁): calf→tarsus = L2s, tarsus→foot = L3。
# 几何取自 URDF: fl_tarsus_joint origin(0.003643,-0.109949), fl_foot_joint origin(0,-0.051948)。
_FL_L2S_VEC = (0.003643, -0.109949)
_FL_L2S = math.hypot(*_FL_L2S_VEC)             # 0.1100 m (calf→tarsus, 旧0.1220, -12.0mm)
_FL_PHI2S = math.atan2(_FL_L2S_VEC[1], _FL_L2S_VEC[0])  # -88.1°
_FL_L3_VEC = (0.0, -0.051948)
_FL_L3 = math.hypot(*_FL_L3_VEC)               # 0.0519 m (tarsus→foot, 不变)
_FL_PHI3 = math.atan2(_FL_L3_VEC[1], _FL_L3_VEC[0])  # -90.0°

# 后腿等效 2-link 参数
# thigh_joint 有初始旋转 rpy=(0, 0.34907, 0)
_RL_THETA0 = 0.34907  # 20° 初始 Y 旋转
# 固定偏移 (thigh_joint origin, 在 hip 坐标系, 不随 thigh 旋转)
_RL_BASE = (-0.046282, 0.00005)
# L1 = thigh(0.023309,-0.11679) + foot(0.01159,-0.061956) (pantograph 合并)
_RL_L1_VEC = (0.034899, -0.178746)
_RL_L1 = math.hypot(*_RL_L1_VEC)               # 0.1821 m
_RL_PHI1 = math.atan2(_RL_L1_VEC[1], _RL_L1_VEC[0])  # -79.0°
# L2 = calf(-0.01875, -0.077772)
_RL_L2_VEC = (-0.01875, -0.077772)
_RL_L2 = math.hypot(*_RL_L2_VEC)               # 0.0800 m
_RL_PHI2 = math.atan2(_RL_L2_VEC[1], _RL_L2_VEC[0])  # -103.6°

# 旧常量 (向后兼容 gait_controller 导入)
FL_THIGH_LEN = _FL_L1
FL_SHIN_LEN  = _FL_L2
RL_THIGH_LEN = _RL_L1
RL_SHIN_LEN  = _RL_L2
RL_FOOT_LEN  = 0.0  # pantograph 已合入 L1

def urdf_to_motor(joint, urdf_angle):
    return urdf_angle * joint.sign

def motor_to_urdf(joint, motor_val):
    return motor_val / joint.sign if joint.sign != 0 else 0.0


def urdf_limits(joint) -> tuple[float, float]:
    """joint_config limit_lo/hi 是电机空间；MuJoCo/URDF 用关节角空间。"""
    if joint.sign == 0:
        return joint.limit_lo, joint.limit_hi
    u_lo = joint.limit_lo / joint.sign
    u_hi = joint.limit_hi / joint.sign
    return min(u_lo, u_hi), max(u_lo, u_hi)


def clamp_urdf(joint, urdf_angle: float) -> float:
    lo, hi = urdf_limits(joint)
    return max(lo, min(hi, urdf_angle))


# ─── 前腿 2D IK / FK ────────────────────────────────────────────────────────
# 结构: hip_pitch → thigh_roll(=0) → calf
# 在 XZ 平面: 标准 2-link arm, L1 随 hip 旋转, L2 随 (hip+calf) 旋转
# 无任何 rpy 偏置

def fk_front_2d(hip, calf):
    """前腿 FK: 返回足端 (x,z) 相对于 hip_pitch 关节, body 坐标系"""
    a1 = _FL_PHI1 - hip
    a2 = _FL_PHI2 - hip - calf
    x = _FL_L1 * math.cos(a1) + _FL_L2 * math.cos(a2)
    z = _FL_L1 * math.sin(a1) + _FL_L2 * math.sin(a2)
    return x, z

def ik_front_leg_2d(target_x, target_z):
    """前腿解析 IK: 输入足端 (x,z) 相对于 hip 关节, 返回 (hip_pitch, calf)"""
    D = math.hypot(target_x, target_z)
    cos_g = (D * D - _FL_L1 * _FL_L1 - _FL_L2 * _FL_L2) / (2 * _FL_L1 * _FL_L2)
    cos_g = max(-1.0, min(1.0, cos_g))
    gamma = math.acos(cos_g)  # 膝盖向前 → gamma > 0
    beta = math.atan2(target_z, target_x)
    psi = math.atan2(_FL_L2 * math.sin(gamma), _FL_L1 + _FL_L2 * math.cos(gamma))
    a = beta - psi
    hip = _FL_PHI1 - a
    calf = _FL_PHI2 - _FL_PHI1 - gamma
    return hip, calf


def solve_front_calf_for_z(hip, target_z, calf_init=0.0, iters=16):
    """1 维求解: 给定前腿 hip_pitch, 求使足端 Z 命中 target_z 的 calf 角度。

    用于"半刚性膝盖"解耦控制 — hip 负责前后摆动(推力), calf 只调节高度。
    足端 z = L1·sin(PHI1-hip) + L2·sin(PHI2-hip-calf), 仅第二项随 calf 变化,
    在工作区间内对 calf 单调, Newton 迭代数步即收敛。
    """
    const = _FL_L1 * math.sin(_FL_PHI1 - hip)
    calf = calf_init
    for _ in range(iters):
        a2 = _FL_PHI2 - hip - calf
        z = const + _FL_L2 * math.sin(a2)
        err = target_z - z
        if abs(err) < 1e-7:
            break
        # dz/dcalf = -L2 · cos(a2)  (da2/dcalf = -1)
        dz_dcalf = -_FL_L2 * math.cos(a2)
        if abs(dz_dcalf) < 1e-9:
            break
        calf += err / dz_dcalf
    return calf


def front_nominal_pose(body_height=0.24, target_x=None):
    """返回前腿站立标称 (hip0, calf0), 作为推力放大的基准姿态。"""
    x = target_x if target_x is not None else fk_front_2d(0.0, 0.0)[0]
    z = -(body_height - abs(WAIST_Z + FL_HIP_Z))
    return ik_front_leg_2d(x, z)


# ─── 前腿 3-link (tarsus 解锁) ─────────────────────────────────────────────
# 结构: hip_pitch → calf → tarsus → foot(fixed)。tarsus 现为主动电机。
# 与 2-link 一致: 各关节角在 FK 中"相减"(a = PHI_base - Σ关节角)。
# tarsus=0 时 3-link 完全退化为 2-link (L2s+L3 合成 = 旧 _FL_L2)。

def fk_front_3link(hip, calf, tarsus):
    """前腿 3-link FK: 返回足端 (x,z) 相对 hip_pitch 关节, body 坐标系。"""
    a1 = _FL_PHI1 - hip
    a2 = _FL_PHI2S - hip - calf
    a3 = _FL_PHI3 - hip - calf - tarsus
    x = _FL_L1 * math.cos(a1) + _FL_L2S * math.cos(a2) + _FL_L3 * math.cos(a3)
    z = _FL_L1 * math.sin(a1) + _FL_L2S * math.sin(a2) + _FL_L3 * math.sin(a3)
    return x, z


def front_foot_pitch(hip, calf, tarsus):
    """前脚足段在机身XZ平面的绝对朝向；-pi/2 表示竖直向下。"""
    return _FL_PHI3 - hip - calf - tarsus


def front_foot_pitch_from_motor(leg, motor_values):
    """由同一周期三个电机角重建前脚绝对朝向；缺值返回 NaN。"""
    names = (f"{leg}_hip_pitch", f"{leg}_calf", f"{leg}_tarsus")
    try:
        urdf = [
            motor_to_urdf(JBN[name], motor_values[JBN[name].motor_id])
            for name in names
        ]
    except (KeyError, TypeError, ValueError):
        return float("nan")
    if any(value is None or not math.isfinite(value) for value in urdf):
        return float("nan")
    return math.degrees(front_foot_pitch(*urdf))


def ik_front_3link(target_x, target_z, tarsus=0.0):
    """前腿 3-link IK: 给定足端 (x,z) 与脚踝角 tarsus, 求 (hip_pitch, calf)。

    tarsus 作为已知输入(冗余自由度由步态调度决定)。固定 tarsus 后,
    L2s 与 L3 合成一个"等效第二段"V2(内部弯折=tarsus), 再套用标准 2-link 解析 IK。
    """
    # 等效第二段矢量 (在 hip+calf=0 参考系下), 内部弯折 = tarsus
    v2x = _FL_L2S * math.cos(_FL_PHI2S) + _FL_L3 * math.cos(_FL_PHI3 - tarsus)
    v2z = _FL_L2S * math.sin(_FL_PHI2S) + _FL_L3 * math.sin(_FL_PHI3 - tarsus)
    l2_eff = math.hypot(v2x, v2z)
    phi2_eff = math.atan2(v2z, v2x)

    D = math.hypot(target_x, target_z)
    cos_g = (D * D - _FL_L1 * _FL_L1 - l2_eff * l2_eff) / (2 * _FL_L1 * l2_eff)
    cos_g = max(-1.0, min(1.0, cos_g))
    gamma = math.acos(cos_g)  # 膝盖向前 → gamma > 0
    beta = math.atan2(target_z, target_x)
    psi = math.atan2(l2_eff * math.sin(gamma), _FL_L1 + l2_eff * math.cos(gamma))
    a = beta - psi
    hip = _FL_PHI1 - a
    calf = phi2_eff - _FL_PHI1 - gamma
    return hip, calf


def solve_front_calf_for_z_3link(hip, target_z, tarsus=0.0, calf_init=0.0, iters=16):
    """3-link 版半刚性膝: 给定 hip 与 tarsus, 解 calf 使足端 Z 命中 target_z。"""
    const = _FL_L1 * math.sin(_FL_PHI1 - hip)
    calf = calf_init
    for _ in range(iters):
        a2 = _FL_PHI2S - hip - calf
        a3 = _FL_PHI3 - hip - calf - tarsus
        z = const + _FL_L2S * math.sin(a2) + _FL_L3 * math.sin(a3)
        err = target_z - z
        if abs(err) < 1e-7:
            break
        dz_dcalf = -_FL_L2S * math.cos(a2) - _FL_L3 * math.cos(a3)
        if abs(dz_dcalf) < 1e-9:
            break
        calf += err / dz_dcalf
    return calf


def front_tarsus_schedule(phase, stance_ratio=0.6, push_amp=0.0):
    """前腿脚踝(tarsus)相位调度 — 支撑相蹬地发力, 摆动相中性。

    push_amp<=0 → 恒 0 (等效锁死, 行为不变)。默认 0 以便逐步启用。
    支撑相用 sin(π·s) 平滑鼓包(两端为 0, 无相位跳变), 中期最大 plantarflex。
    """
    if push_amp == 0.0 or stance_ratio <= 0.0:
        return 0.0
    if phase >= stance_ratio:
        return 0.0
    s = phase / stance_ratio
    return push_amp * math.sin(math.pi * s)


def ik_front_3link_foot_orient(target_x, target_z, foot_pitch):
    """前腿 3-DOF 足朝向 IK — 同时命中足端 (x,z) 与足段绝对朝向 foot_pitch。

    对应 RL 学到的策略: 脚尖(足段 tarsus→foot)时刻朝向地面。3 个关节(hip,calf,tarsus)
    恰好求解 3 个约束(x, z, 足朝向), 唯一确定, 天然协调三关节。

    foot_pitch: 足段在 body 矢状面的绝对角(rad), -π/2≈竖直朝下(脚尖指地)。
    返回 (hip, calf, tarsus) URDF 角。
    """
    # tarsus 关节位置 = 足端 - L3·方向(foot_pitch)
    px = target_x - _FL_L3 * math.cos(foot_pitch)
    pz = target_z - _FL_L3 * math.sin(foot_pitch)
    # 用 L1 + L2S 二连杆解析 IK 到达 tarsus 关节位置 (膝向前, gamma>0)
    D = math.hypot(px, pz)
    cos_g = (D * D - _FL_L1 * _FL_L1 - _FL_L2S * _FL_L2S) / (2 * _FL_L1 * _FL_L2S)
    cos_g = max(-1.0, min(1.0, cos_g))
    gamma = math.acos(cos_g)
    beta = math.atan2(pz, px)
    psi = math.atan2(_FL_L2S * math.sin(gamma), _FL_L1 + _FL_L2S * math.cos(gamma))
    a = beta - psi
    hip = _FL_PHI1 - a
    calf = _FL_PHI2S - _FL_PHI1 - gamma
    # 由足朝向定义 a3 = _FL_PHI3 - hip - calf - tarsus 反解 tarsus
    tarsus = _FL_PHI3 - hip - calf - foot_pitch
    return hip, calf, tarsus


def front_standing_foot_pitch(body_height=0.24, target_x=None, tarsus_deg=0.0,
                              foot_pitch=None):
    """前腿站立位足段的绝对朝向(rad)。用作足朝向跟踪的 ramp 起点。

    tarsus_deg 必须和 StandController 的站立 tarsus 一致；否则 NaturalTrot 的
    ramp=0 第一帧会从旧 tarsus=0 站姿重新解算，造成站立→步态突跳。
    """
    if foot_pitch is not None:
        return foot_pitch
    if target_x is None:
        target_x = 0.0
    z_front = -(body_height - abs(WAIST_Z + FL_HIP_Z))
    tarsus = math.radians(tarsus_deg)
    hip0, calf0 = ik_front_3link(target_x, z_front, tarsus)
    return _FL_PHI3 - hip0 - calf0 - tarsus


# ─── 后腿 2D IK / FK ────────────────────────────────────────────────────────
# 结构: hip(roll=0) → thigh(pitch, 带 0.349rad 初始旋转) → calf → tarsus(mimic=-calf)
# 等效 2-link: L1(thigh+foot) 随 (0.349+thigh), L2(calf) 随 (0.349+thigh+calf)
# 有固定偏移 BASE 在 thigh 旋转之前

def fk_rear_2d(thigh, calf):
    """后腿 FK: 返回足端 (x,z) 相对于 hip 关节, body 坐标系"""
    t_eff = _RL_THETA0 + thigh
    a1 = _RL_PHI1 - t_eff
    a2 = _RL_PHI2 - t_eff - calf
    x = _RL_BASE[0] + _RL_L1 * math.cos(a1) + _RL_L2 * math.cos(a2)
    z = _RL_BASE[1] + _RL_L1 * math.sin(a1) + _RL_L2 * math.sin(a2)
    return x, z

def ik_rear_leg_2d(target_x, target_z):
    """后腿解析 IK: 输入足端 (x,z) 相对于 hip 关节, 返回 (thigh, calf)"""
    tx = target_x - _RL_BASE[0]
    tz = target_z - _RL_BASE[1]
    D = math.hypot(tx, tz)
    cos_g = (D * D - _RL_L1 * _RL_L1 - _RL_L2 * _RL_L2) / (2 * _RL_L1 * _RL_L2)
    cos_g = max(-1.0, min(1.0, cos_g))
    gamma = -math.acos(cos_g)  # 膝盖向后 → gamma < 0
    beta = math.atan2(tz, tx)
    psi = math.atan2(_RL_L2 * math.sin(gamma), _RL_L1 + _RL_L2 * math.cos(gamma))
    a = beta - psi
    thigh = _RL_PHI1 - _RL_THETA0 - a
    calf = _RL_PHI2 - _RL_PHI1 - gamma
    return thigh, calf


# ─── 旧接口兼容 (Z-constraint) ──────────────────────────────────────────────

def fk_front_leg_z(hip_pitch, calf):
    _, z = fk_front_2d(hip_pitch, calf)
    return z

def fk_rear_leg_z(thigh, calf):
    _, z = fk_rear_2d(thigh, calf)
    return z

def ik_front_leg(body_height, hip_pitch=0.0):
    z_target = -(body_height - abs(WAIST_Z + FL_HIP_Z))
    _, calf = ik_front_leg_2d(0.0, z_target)
    return calf

def ik_rear_leg(body_height, thigh=0.0):
    z_target = -(body_height - abs(RL_HIP_Z))
    _, calf = ik_rear_leg_2d(0.0, z_target)
    return calf

# 保留旧名字
fk_front_exact = fk_front_2d
fk_rear_exact = fk_rear_2d

def front_thigh_roll_abd_urdf(leg: str, hip_abduction: float) -> float:
    """前腿 thigh_roll 静态外展的 URDF 角。
    随着 URDF 右侧轴反转修正，现在左右两侧的正角都统一代表向外展。
    """
    return hip_abduction


def compute_standing_pose_3link(body_height=0.24, target_x_front=0.0, target_x_rear=0.0,
                                hip_abduction=0.05, foot_pitch=-math.pi / 2):
    """站立姿态 — 前腿用 3-link IK, 使足段达到指定绝对朝向。

    默认 foot_pitch=-pi/2, 即脚段竖直向下/脚掌平贴地面。后腿无 tarsus,
    仍用 2-link IK。
    """
    z_front = -(body_height - abs(WAIST_Z + FL_HIP_Z))
    z_rear = -(body_height - abs(RL_HIP_Z))

    fl_hip_urdf, fl_calf_urdf, fl_tarsus_urdf = ik_front_3link_foot_orient(
        target_x_front, z_front, foot_pitch)
    rl_thigh_urdf, rl_calf_urdf = ik_rear_leg_2d(target_x_rear, z_rear)

    fl_calf_urdf = max(-1.82, min(1.93, fl_calf_urdf))
    rl_calf_urdf = max(-0.5, min(1.56, rl_calf_urdf))

    fl_tr = front_thigh_roll_abd_urdf("fl", hip_abduction)
    rl_h = hip_abduction
    rr_h = hip_abduction

    urdf_angles = {}
    for j in JOINT_MAP:
        if "hip_pitch" in j.name:
            urdf_angles[j.motor_id] = fl_hip_urdf
        elif j.name == "fl_thigh_roll":
            urdf_angles[j.motor_id] = fl_tr
        elif j.name == "fr_thigh_roll":
            urdf_angles[j.motor_id] = front_thigh_roll_abd_urdf("fr", hip_abduction)
        elif j.name == "fl_calf":
            urdf_angles[j.motor_id] = fl_calf_urdf
        elif j.name == "fr_calf":
            urdf_angles[j.motor_id] = fl_calf_urdf
        elif j.name in ("fl_tarsus", "fr_tarsus"):
            urdf_angles[j.motor_id] = fl_tarsus_urdf
        elif j.name == "rl_hip":
            urdf_angles[j.motor_id] = rl_h
        elif j.name == "rr_hip":
            urdf_angles[j.motor_id] = rr_h
        elif j.name in ("rl_thigh", "rr_thigh"):
            urdf_angles[j.motor_id] = rl_thigh_urdf
        elif j.name == "rl_calf":
            urdf_angles[j.motor_id] = rl_calf_urdf
        elif j.name == "rr_calf":
            urdf_angles[j.motor_id] = rl_calf_urdf
        else:
            urdf_angles[j.motor_id] = 0.0

    return urdf_angles


def compute_standing_pose(body_height=0.24, target_x_front=0.0, target_x_rear=0.0, hip_abduction=0.05):
    """[已作废的名字] 老两连杆站姿已删除。

    2026-07-11: 唯一站姿 = 新三连杆带前腿主动 tarsus (脚段绝对朝向 -90°/竖直指地)。
    保留此函数名仅为向后兼容旧调用点(replay/print_standing_info), 内部直接转发到
    compute_standing_pose_3link, 不再有第二套站姿。
    """
    return compute_standing_pose_3link(
        body_height, target_x_front, target_x_rear, hip_abduction,
        foot_pitch=-math.pi / 2)


def compute_joint_check_pose(
    pose_angle: float = 0.35,
    calf_scale: float = 1.8,
) -> dict:
    """悬空关节方向定格 — sim_plan --joint-check 验证用。

    前腿: 大腿向后 (+angle), 小腿向前 (-calf_angle), 脚踝 tarsus +angle (蹬地方向验证)
    后腿: 大腿向前 (rl/rr:-angle), 小腿向后 (+calf_angle)
    """
    a = pose_angle
    ca = pose_angle * calf_scale
    urdf_by_name = {
        "fl_hip_pitch": +a,
        "fr_hip_pitch": +a,
        "fl_calf": -ca,
        "fr_calf": -ca,
        "fl_tarsus": +a,
        "fr_tarsus": +a,
        "rl_thigh": -a,
        "rr_thigh": -a,
        "rl_calf": +ca,
        "rr_calf": +ca,
        "fl_thigh_roll": 0.0,
        "fr_thigh_roll": 0.0,
        "rl_hip": 0.0,
        "rr_hip": 0.0,
    }
    motor_angles = {}
    for j in JOINT_MAP:
        u = urdf_by_name.get(j.name, 0.0)
        motor_angles[j.motor_id] = u
    return motor_angles


def print_standing_info(body_height=0.24, target_x_front=0.0, target_x_rear=0.0):
    pose = compute_standing_pose(body_height, target_x_front, target_x_rear)
    z_front = -(body_height - abs(WAIST_Z + FL_HIP_Z))
    z_rear = -(body_height - abs(RL_HIP_Z))
    fl_hip, fl_calf = ik_front_leg_2d(target_x_front, z_front)
    rl_thigh, rl_calf = ik_rear_leg_2d(target_x_rear, z_rear)
    
    print(f"\n{'='*60}")
    print(f"  站立姿态 (body_height={body_height:.3f}m, X_front={target_x_front}, X_rear={target_x_rear})")
    print(f"  前腿计算: Hip={math.degrees(fl_hip):+.1f}°, Calf={math.degrees(fl_calf):+.1f}°")
    print(f"  后腿计算: Thigh={math.degrees(rl_thigh):+.1f}°, Calf={math.degrees(rl_calf):+.1f}°")
    for j in JOINT_MAP:
        mid = j.motor_id
        deg = math.degrees(pose[mid])
        print(f"    Motor {mid:2d} ({j.name:18s}): {deg:+8.2f}°")
    return pose

if __name__ == "__main__":
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument("--height", type=float, default=0.24)
    p.add_argument("--x-front", type=float, default=0.0)
    p.add_argument("--x-rear", type=float, default=0.0)
    args = p.parse_args()
    print_standing_info(args.height, args.x_front, args.x_rear)
