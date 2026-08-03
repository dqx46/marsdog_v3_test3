"""[柔顺B] 腿部重力补偿前馈 (sagittal 简化模型)。

目的: 按当前关节角计算每条腿"撑住自身连杆重量"所需的关节力矩 tau_g(q),
写入 MIT/PTM 的 trq_ff。重力由前馈扛住后, 腿部 kp 就能整体降低 → 全程更柔顺、
落足冲击更小, 且不塌腿。

模型说明
--------
- 矢状面(XZ)平面模型, 假设躯干水平(base≈world, 重力沿 -Z)。
- 只补偿 pitch(绕 +Y)关节, 它们直接对抗重力:
    前腿: hip_pitch, calf   (thigh_roll 是外展/roll, 矢状面重力力矩极小, 跳过)
    后腿: thigh,     calf   (hip 是外展/roll, 跳过)
- 连杆质量/质心/关节几何全部取自 marsdog.urdf (SolidWorks 导出)。
- 前腿 tarsus 为被动 4 连杆(无电机), 近似为 calf 的刚性延伸(tarsus 角=0)。
- 后腿 tarsus 为 mimic(角 = -calf), URDF 明确, 精确建模。

符号约定
--------
- 输入角为 URDF 关节角(rad), 与 kinematics.fk_* 一致。
- 绕 +Y 轴的重力力矩 tau_grav = g * Σ m_i * x_i (x_i 为质心相对关节的前向偏移)。
- "保持静止"所需关节力矩(前馈) tau_hold = -tau_grav。
- 电机端力矩 tau_motor = tau_hold * joint.sign (与位置同一变换, 功率守恒)。
  电机驱动不做减速比换算(位置=URDF角), 故力矩亦为输出轴(关节)Nm, 直接下发。

安全
----
- 力矩方向务必先上机静态验证(见 __main__ 自检 + walk.py --gravity-comp 缩放先取 0.5)。
"""

import math

# [解耦] 真实实现已下沉到此 src 模块; 函数内的扁平 import (import kinematics as K)
# 由 ensure_legacy_path() 保证可解析。
from marsdog_control.compat import ensure_legacy_path as _ensure_legacy_path
_ensure_legacy_path()

G = 9.81  # m/s^2

# ── 连杆质量 (kg, 取自 URDF) ────────────────────────────────────────────────
_M_F_HIP   = 0.266   # fl/fr_hip_pitch_link
_M_F_THIGH = 0.087   # fl/fr_thigh_roll_link
_M_F_CALF  = 0.462   # fl/fr_calf_link
_M_F_TAR   = 0.067   # fl/fr_tarsus_link

_M_R_HIP   = 0.374   # rl/rr_hip_link
_M_R_THIGH = 0.414   # rl/rr_thigh_link
_M_R_CALF  = 0.048   # rl/rr_calf_link
_M_R_TAR   = 0.032   # rl/rr_tarsus_link


def _ry(theta, x, z):
    """绕 +Y 轴旋转 theta 后的 (x,z)。 R_y: x'=x·c+z·s, z'=-x·s+z·c"""
    c = math.cos(theta)
    s = math.sin(theta)
    return x * c + z * s, -x * s + z * c


# ─────────────────────────────────────────────────────────────────────────────
# 前腿 sagittal FK: 输入 URDF (hip_pitch, calf); tarsus 视为刚性(角=0)
# 链: hip_pitch(Y) → thigh_roll(X,视0) → calf(Y) → tarsus(视0) → foot
# 各偏移/质心均取自 URDF(投影到 XZ)。
# ─────────────────────────────────────────────────────────────────────────────

def _front_chain(q_hp, q_calf):
    """返回 (links, calf_joint_x, foot_x, foot_z)。
    links = [(mass, com_x), ...] 相对 hip_pitch 关节(hip 在 x=0)。"""
    # hip_pitch 关节在原点, 绕 Y 旋 q_hp
    th_hp = q_hp
    cx, cz = _ry(th_hp, -0.036249, -0.000863)
    hp = (_M_F_HIP, cx)

    # thigh_roll 关节 (在 hip_pitch link 系)
    dx, dz = _ry(th_hp, -0.0205, 3.8219e-5)
    trx, trz = dx, dz
    th_tr = th_hp                    # roll 不改变绕 Y 旋转
    cx, cz = _ry(th_tr, 0.019147, -0.03082)
    tr = (_M_F_THIGH, trx + cx)

    # calf 关节 (在 thigh_roll link 系) — 大腿连杆 90mm
    dx, dz = _ry(th_tr, 0.02861305, -0.08522429)
    cjx, cjz = trx + dx, trz + dz
    th_calf = th_tr + q_calf
    cx, cz = _ry(th_calf, 0.001263, -0.03026)
    calf = (_M_F_CALF, cjx + cx)

    # tarsus 关节 (在 calf link 系, 被动 → 角=0) — 小腿连杆 110mm
    dx, dz = _ry(th_calf, 0.003642077, -0.1099213)
    tjx, tjz = cjx + dx, cjz + dz
    th_tar = th_calf
    cx, cz = _ry(th_tar, 0.0, -0.026597)
    tar = (_M_F_TAR, tjx + cx)

    # foot (在 tarsus link 系)
    fx, fz = _ry(th_tar, 0.0, -0.051948)
    foot_x, foot_z = tjx + fx, tjz + fz

    return [hp, tr, calf, tar], cjx, foot_x, foot_z


def front_leg_ff(q_hp, q_calf):
    """前腿重力补偿前馈力矩 (URDF Nm)。返回 dict{'hip_pitch','calf'}。
    tau_hold = -g·Σ m·x (相对各自关节)。"""
    links, cjx, _, _ = _front_chain(q_hp, q_calf)
    tau_hp = -G * sum(m * x for m, x in links)                       # hip 在 x=0
    tau_calf = -G * sum(m * (x - cjx) for m, x in links[2:])         # calf,tarsus
    return {'hip_pitch': tau_hp, 'calf': tau_calf}


# ─────────────────────────────────────────────────────────────────────────────
# 后腿 sagittal FK: 输入 URDF (thigh, calf); tarsus mimic = -calf
# 链: hip(X,视0, rpy pitch -0.34907) → thigh(Y, rpy pitch +0.34907)
#     → calf(Y) → tarsus(mimic -calf) → foot
# ─────────────────────────────────────────────────────────────────────────────

_R_THETA0 = 0.34907  # rl/rr_thigh_joint rpy pitch (与 hip 的 -0.34907 配对)

def _rear_chain(q_thigh, q_calf):
    """返回 (links, calf_joint_x, foot_x, foot_z), 相对 hip 关节(x=0)。"""
    # hip 关节在原点; 外展角=0, 但有固定 rpy pitch -0.34907
    th_hip = -_R_THETA0
    cx, cz = _ry(th_hip, -0.045138, -0.000017)
    hip = (_M_R_HIP, cx)

    # thigh 关节 (在 hip link 系), 固定 rpy +0.34907 + 可动 q_thigh
    dx, dz = _ry(th_hip, -0.046282, 5.0587e-5)
    tx, tz = dx, dz
    th_thigh = th_hip + _R_THETA0 + q_thigh
    cx, cz = _ry(th_thigh, 0.008049, -0.059444)
    thigh = (_M_R_THIGH, tx + cx)

    # calf 关节 (在 thigh link 系)
    dx, dz = _ry(th_thigh, 0.023309, -0.11679)
    cjx, cjz = tx + dx, tz + dz
    th_calf = th_thigh + q_calf
    cx, cz = _ry(th_calf, -0.018059, -0.027577)
    calf = (_M_R_CALF, cjx + cx)

    # tarsus 关节 (在 calf link 系), mimic = -calf
    dx, dz = _ry(th_calf, -0.01875, -0.077772)
    tjx, tjz = cjx + dx, cjz + dz
    th_tar = th_calf + (-q_calf)     # mimic mult=-1
    cx, cz = _ry(th_tar, 0.001447, -0.022124)
    tar = (_M_R_TAR, tjx + cx)

    # foot (在 tarsus link 系), rl_foot origin=[0.01159,0.010411,-0.061956]
    fx, fz = _ry(th_tar, 0.01159, -0.061956)
    foot_x, foot_z = tjx + fx, tjz + fz

    return [hip, thigh, calf, tar], cjx, foot_x, foot_z


def rear_leg_ff(q_thigh, q_calf):
    """后腿重力补偿前馈力矩 (URDF Nm)。返回 dict{'thigh','calf'}。"""
    links, cjx, _, _ = _rear_chain(q_thigh, q_calf)
    # thigh 关节在 x=0? 不: thigh 关节相对 hip 有偏移。thigh 力矩需相对 thigh 关节。
    # thigh 关节 x:
    th_hip = -_R_THETA0
    dx, _dz = _ry(th_hip, -0.046282, 5.0587e-5)
    tjx = dx
    tau_thigh = -G * sum(m * (x - tjx) for m, x in links[1:])   # thigh,calf,tarsus
    tau_calf = -G * sum(m * (x - cjx) for m, x in links[2:])    # calf,tarsus
    return {'thigh': tau_thigh, 'calf': tau_calf}


# ── walk.py 接口: 输入某腿 URDF 关节角, 返回该腿各关节前馈力矩(URDF Nm) ──────

def leg_gravity_ff(leg, angles):
    """leg ∈ {'fl','fr','rl','rr'}; angles = URDF 关节角 dict。

    前腿 angles 需含 'hip_pitch','calf'; 后腿需含 'thigh','calf'。
    返回 dict: 关节名(不含腿前缀) → 前馈力矩(URDF Nm, 已是保持力矩)。
    左右腿在矢状面镜像对称, 力矩量级相同(方向由 walk 侧 joint.sign 处理)。
    """
    if leg in ('fl', 'fr'):
        return front_leg_ff(angles['hip_pitch'], angles['calf'])
    else:
        return rear_leg_ff(angles['thigh'], angles['calf'])


# ─────────────────────────────────────────────────────────────────────────────
# 自检: 1) FK 足端 与 kinematics.py 对照(验证坐标系一致);
#        2) 打印站立标称姿态下各关节重力前馈力矩(供上机静态验证方向/量级)。
# ─────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    import marsdog_control.motion.kinematics as K

    print("=== 足端 FK 交叉验证 (gravity_comp vs kinematics) ===")
    for (hp, calf) in [(0.0, 0.0), (0.3, -0.5), (-0.2, 0.4)]:
        _, _, fx, fz = _front_chain(hp, calf)
        kx, kz = K.fk_front_2d(hp, calf)
        print(f"前腿 hip={hp:+.2f} calf={calf:+.2f}  "
              f"gc=({fx:+.4f},{fz:+.4f})  kin=({kx:+.4f},{kz:+.4f})  "
              f"d=({fx-kx:+.4f},{fz-kz:+.4f})")
    for (th, calf) in [(0.0, 0.0), (0.3, -0.5), (-0.2, 0.4)]:
        _, _, fx, fz = _rear_chain(th, calf)
        kx, kz = K.fk_rear_2d(th, calf)
        print(f"后腿 thigh={th:+.2f} calf={calf:+.2f}  "
              f"gc=({fx:+.4f},{fz:+.4f})  kin=({kx:+.4f},{kz:+.4f})  "
              f"d=({fx-kx:+.4f},{fz-kz:+.4f})")

    print("\n=== 站立标称姿态 重力前馈力矩 (URDF Nm) ===")
    h = 0.24
    zf = -(h - abs(K.WAIST_Z + K.FL_HIP_Z))
    hp0, calf0 = K.ik_front_leg_2d(0.0, zf)
    print(f"前腿 站立 hip_pitch={hp0:+.3f} calf={calf0:+.3f}  ->  {front_leg_ff(hp0, calf0)}")
    zr = -(h - abs(K.RL_HIP_Z))
    th0, calf0r = K.ik_rear_leg_2d(0.0, zr)
    print(f"后腿 站立 thigh={th0:+.3f} calf={calf0r:+.3f}  ->  {rear_leg_ff(th0, calf0r)}")
