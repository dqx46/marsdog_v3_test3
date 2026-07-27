"""Central joint mapping table: motor ID ↔ CSV column ↔ sign ↔ bus.

23 个关节, 4 类电机, 4 条 CAN 总线:

总线拓扑 (2026-07-22 因克斯独立总线):
  lz_can_a  (USB-CAN 灵足 A): ID 1,2,5,6,17           — 前腿髋/大腿 + head_roll
  lz_can_b  (USB-CAN 灵足 B): ID 10,11,13,14,15,16,21 — 后腿从关节 + 头部 + waist_roll
  evo_can   (USB-CAN 泉智博): ID 9,12,18,19,20        — 后腿 hip + 颈腰
  dm_can    (u2can 达妙):      ID 4,8                  — 前腿 tarsus (S2325)
  incos_can (USB-CAN 因克斯独立): ID 3,7              — 前腿小腿

关节布局:
  ID  1- 4: FL 前左腿 (hip_pitch RS02, thigh_roll EL05, calf EC-A2806, tarsus DM-S2325)
  ID  5- 8: FR 前右腿 (hip_pitch RS02, thigh_roll EL05, calf EC-A2806, tarsus DM-S2325)
  ID  9-11: RL 后左腿 (hip PA43, thigh RS00, calf RS00)
  ID 12-14: RR 后右腿 (hip PA43, thigh RS00, calf RS00)
  ID 15-17: 头部 (pitch EL05, yaw EL05, roll EL05)
  ID 18-20: 颈/腰 (neck PA43, waist_yaw PA43, waist_pitch PA43)
  ID 21:    腰 roll (RS02)
  ID 22-23: 后腿 tarsus (EL05, 预留未接线)

CSV column layout (from play_motion.py):
    cols  0- 2   root XYZ
    cols  3- 6   root quaternion
    cols  7-30   tail (unused)
    cols 31-34   rl: hip, thigh, calf, tarsus
    cols 35-38   rr: hip, thigh, calf, tarsus
    cols 39-41   waist: roll, pitch, yaw
    cols 42-45   head: neck_pitch, head_roll, head_yaw, head_pitch
    cols 46-48   fl: hip_pitch, thigh_roll, calf
    cols 49-51   fr: hip_pitch, thigh_roll, calf
"""

import math

class JointDesc:
    __slots__ = ("motor_id", "name", "csv_col", "sign",
                 "mtype", "bus", "limit_lo", "limit_hi",
                 "model", "gear_ratio")

    def __init__(self, motor_id, name, csv_col, sign, mtype, bus,
                 limit_lo, limit_hi, model="", gear_ratio=1.0):
        self.motor_id = motor_id
        self.name = name
        self.csv_col = csv_col
        self.sign = sign
        self.mtype = mtype          # "lz" / "evo" / "dm" / "incos"
        self.bus = bus              # "lz_can_a" / "lz_can_b" / "evo_can" / "dm_can" / "incos_can"
        self.limit_lo = limit_lo
        self.limit_hi = limit_hi
        self.model = model
        self.gear_ratio = gear_ratio

_PI = math.pi

JOINT_MAP = [
    # ── 前左腿 (Front Left) ──────────────────────────────────────
    JointDesc(1,  "fl_hip_pitch",   46, -1, "lz", "lz_can_a",  -_PI, 0.97,
              model="RS02", gear_ratio=7.75),
    JointDesc(2,  "fl_thigh_roll",  47, -1, "lz", "lz_can_a",  -_PI, 0.18,
              model="EL05", gear_ratio=9),
    # 2026-07-17 实测标定: 侧翻逐个驱动+肉眼确认, fl 正确站立小腿角为 +电机角
    # (原 sign=+1 会发 -80° 折反)。改 sign=-1; 限位随之做镜像对调保持 URDF 可达范围。
    JointDesc(3,  "fl_calf",        48, -1, "incos", "incos_can",  -1.93, 1.82,
              model="EC-A2806-P2-36", gear_ratio=36),
    # limit 对照 URDF fl_tarsus_joint (无 mimic, 独立自由度): lower=-0.03 upper=1.49
    # 实机方向验证：左前达妙安装方向与右前相反，URDF 正角须映射到负电机角(sign=-1)。
    # limit_lo/hi 是"电机空间"(见 kinematics.urdf_limits), sign=-1 时须把 URDF 范围翻过来:
    # URDF[-0.03,1.49] -> 电机[-1.49,0.03]; 否则新站姿的 URDF tarsus(+0.55) 会被错误钳到 -0.03。
    JointDesc(4,  "fl_tarsus",      -1, -1, "dm", "dm_can",   -1.49, 0.03,
              model="DM-S2325", gear_ratio=1.0),
    # ── 前右腿 (Front Right) ──────────────────────────────────────
    JointDesc(5,  "fr_hip_pitch",   49, +1, "lz", "lz_can_a",  -0.97, _PI,
              model="RS02", gear_ratio=7.75),
    JointDesc(6,  "fr_thigh_roll",  50, +1, "lz", "lz_can_a",  -0.18, _PI,
              model="EL05", gear_ratio=9),
    # 2026-07-17 实测标定: fr 正确站立小腿角为 -电机角 (原 sign=-1 会发 +80° 折反)。
    # 改 sign=+1; 限位随之做镜像对调保持 URDF 可达范围。
    JointDesc(7,  "fr_calf",        51, +1, "incos", "incos_can",  -1.82, 1.93,
              model="EC-A2806-P2-36", gear_ratio=36),
    # limit 对照 URDF fr_tarsus_joint (无 mimic, 独立自由度): lower=-0.03 upper=1.49
    JointDesc(8,  "fr_tarsus",      -1, +1, "dm", "dm_can",   -0.03, 1.49,
              model="DM-S2325", gear_ratio=1.0),
    # ── 后左腿 (Rear Left) ───────────────────────────────────────
    JointDesc(9,  "rl_hip",         31, +1, "evo", "evo_can",  -0.49, 0.77,
              model="PA43", gear_ratio=25),
    JointDesc(10, "rl_thigh",       32, -1, "lz", "lz_can_b",  -0.71, 1.77,
              model="RS00", gear_ratio=10),
    JointDesc(11, "rl_calf",        33, -1, "lz", "lz_can_b",  -1.56, 0.0,
              model="RS00", gear_ratio=10),
    # ── 后右腿 (Rear Right) ──────────────────────────────────────
    JointDesc(12, "rr_hip",         35, -1, "evo", "evo_can",  -0.49, 0.77,
              model="PA43", gear_ratio=25),
    JointDesc(13, "rr_thigh",       36, +1, "lz", "lz_can_b",  -1.77, 0.71,
              model="RS00", gear_ratio=10),
    JointDesc(14, "rr_calf",        37, +1, "lz", "lz_can_b",   0.0, 1.56,
              model="RS00", gear_ratio=10),
    # ── 头部 (Head) ──────────────────────────────────────────────
    JointDesc(15, "head_pitch",     45, +1, "lz", "lz_can_b",  -1.7, 0.5,
              model="EL05", gear_ratio=9),
    JointDesc(16, "head_yaw",       44, +1, "lz", "lz_can_b",  -_PI, _PI,
              model="EL05", gear_ratio=9),
    JointDesc(17, "head_roll",      43, +1, "lz", "lz_can_a",  -_PI, _PI,
              model="EL05", gear_ratio=9),
    # ── 颈/腰 (Neck/Waist) ──────────────────────────────────────
    JointDesc(18, "neck_pitch",     42, +1, "evo", "evo_can",  -0.58, 0.45,
              model="PA43", gear_ratio=25),
    JointDesc(19, "waist_yaw",      41, +1, "evo", "evo_can",  -1.2, 1.2,
              model="PA43", gear_ratio=25),
    JointDesc(20, "waist_pitch",    40, +1, "evo", "evo_can",   0.0, 0.40,
              model="PA43", gear_ratio=25),
    JointDesc(21, "waist_roll",     39, +1, "lz", "lz_can_b",  -1.0, 1.0,
              model="RS02", gear_ratio=7.75),
    # ── 后腿 tarsus (预留, 尚未接线) ────────────────────────────
    JointDesc(22, "rl_tarsus",      34, +1, "lz", "none",      -_PI, _PI,
              model="EL05", gear_ratio=9),
    JointDesc(23, "rr_tarsus",      38, +1, "lz", "none",      -_PI, _PI,
              model="EL05", gear_ratio=9),
]

JOINT_BY_ID = {j.motor_id: j for j in JOINT_MAP}
JOINT_BY_NAME = {j.name: j for j in JOINT_MAP}

# 按总线分组
LZ_CAN_A_IDS = [j.motor_id for j in JOINT_MAP if j.bus == "lz_can_a"]   # [1,2,5,6,17]
LZ_CAN_B_IDS = [j.motor_id for j in JOINT_MAP if j.bus == "lz_can_b"]   # [10,11,13,14,15,16,21]
EVO_CAN_IDS  = [j.motor_id for j in JOINT_MAP if j.bus == "evo_can"]     # [9,12,18,19,20]
DM_CAN_IDS   = [j.motor_id for j in JOINT_MAP if j.bus == "dm_can"]      # [4,8]
INCOS_CAN_IDS = [j.motor_id for j in JOINT_MAP if j.bus == "incos_can"]  # [3,7]

ALL_IDS = [j.motor_id for j in JOINT_MAP if j.bus != "none"]
ALL_IDS_INCLUDING_SPARE = [j.motor_id for j in JOINT_MAP]

# 向后兼容 (旧代码中引用的名字)
LZ_CAN_IDS = LZ_CAN_A_IDS
LZ_SERIAL_IDS = LZ_CAN_B_IDS

DEFAULT_LZ_KP = 45.0
DEFAULT_LZ_KD = 4.0
DEFAULT_EVO_KP = 30.0
DEFAULT_EVO_KD = 4.0
DEFAULT_DM_KP = 30.0
DEFAULT_DM_KD = 0.5

# 达妙 S2325 (前腿 tarsus) 电机主机ID (MasterID)
# 2026-07-10 实测发现同事口头确认的"两个电机都是0x63"不准确: 电机4确实是0x63,
# 电机8实测是0x14(20)。已用 change_motor_param(REG_MST_ID) 把电机8也改成0x63并
# save_motor_param 持久化, 现在两个电机 MasterID 统一为 0x63。
#
# 注意: 两个电机共用同一个 MasterID 意味着它们的反馈帧 canId 相同, 若"几乎同时"
# 对两个电机分别发起请求(如高频并发 MIT 控制), 单靠 canId 无法区分是哪个电机的
# 回复 (咱们代码目前是严格串行访问, 一次只对一个 slave_id 发指令并等它的回复,
# 不会有歧义; 但如果以后改成给两条腿并发发送控制帧再统一收包解析, 需要改用
# canId==0 广播帧格式那种"电机ID编码在数据里"的分支, 或者干脆把两个电机的
# MasterID 重新改回不同的值)。
DM_MASTER_ID_BY_SLAVE = {
    4: 0x63,
    8: 0x63,
}
# 向后兼容: 若外部代码只需要一个默认值 (例如只操作单个电机场景)
DM_MASTER_ID = 0x63

def csv_to_motor_rad(csv_row, joint, scale=1.0, frame0_row=None):
    if joint.csv_col < 0:
        return 0.0
    raw = csv_row[joint.csv_col]
    if frame0_row is not None:
        base = frame0_row[joint.csv_col]
        delta = raw - base
        motor_val = (base + delta * scale) * joint.sign
    else:
        motor_val = raw * joint.sign * scale
    return max(joint.limit_lo, min(joint.limit_hi, motor_val))

def clamp_to_limits(value_rad, joint):
    return max(joint.limit_lo, min(joint.limit_hi, value_rad))
