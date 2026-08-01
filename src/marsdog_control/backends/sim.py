"""MuJoCo 物理仿真 — 与 20260705_1520 walk.py 完全对齐。

模型: marsdog/urdf/marsdog.urdf
执行: MIT 力矩 (position 执行器等效) + 相位可变阻抗 + 重力补偿前馈
控制: 200 Hz
"""

from __future__ import annotations

import csv
import math
import os
import tempfile
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Dict, Optional, TextIO, Tuple, Set

try:
    import mujoco
except ImportError:
    mujoco = None

import numpy as np

from marsdog_control.config.joints import JOINT_MAP, JOINT_BY_ID, JOINT_BY_NAME
from marsdog_control.core.types import ControlOutput, RobotState
from marsdog_control.config.gains import JOINT_GAINS
from marsdog_control.backends.base import RobotBackend
from marsdog_control.motion.kinematics import clamp_urdf, urdf_limits

# 未在 JOINT_GAINS 显式列出的关节回退增益 (与旧 motor_gains.DEFAULT_GAIN 一致)。
DEFAULT_GAIN = {"kp": 30.0, "kd": 4.0, "trq_ff": 0.0}

_HERE = os.path.dirname(os.path.abspath(__file__))
URDF_PATH = os.path.normpath(
    os.path.join(_HERE, "..", "..", "..", "marsdog", "urdf", "marsdog.urdf")
)
# Prefer repo-local meshes (next to urdf/); then sibling checkout ../meshes.
_MESH_CANDIDATES = (
    os.path.normpath(os.path.join(_HERE, "..", "..", "..", "marsdog", "meshes")),
    os.path.normpath(os.path.join(_HERE, "..", "..", "..", "..", "meshes")),
)
MESH_DIR = next((p for p in _MESH_CANDIDATES if os.path.isdir(p)), _MESH_CANDIDATES[0])

MIMIC_JOINTS = (
    ("rl_tarsus_joint", "rl_calf_joint", -1.0),
    ("rr_tarsus_joint", "rr_calf_joint", -1.0),
)

# 前腿 tarsus 解锁 (2026-07): 新增电机后不再焊死。URDF effort 仍是旧被动弹簧值
# (0.4119Nm) 过小, 用真实电机峰值力矩覆盖。
# 硬件实测: 峰值 5Nm / 额定 1.5Nm / 额定转速 380rpm。MIT 饱和取峰值。
# 注意: 该值须同时写到 actuator forcerange 与 jnt_actfrcrange
# (见 _apply_joint_actuator_force_limits), 否则关节侧仍按 0.4119Nm 钳死 → 跪地。
_TARSUS_EFFORT_OVERRIDE = {
    "fl_tarsus": 5.0,
    "fr_tarsus": 5.0,
}
_SIM_EFFORT_OVERRIDES: Dict[str, float] = {}
_JOINT_MAP_NAMES = {j.name for j in JOINT_MAP}

CONTROL_HZ = 200.0
PHYSICS_DT = 0.001
N_SUBSTEPS = 5
CONTROL_DT = PHYSICS_DT * N_SUBSTEPS

JOINT_ARMATURE = 0.03
FOOT_CLEARANCE = 0.01
KINEMATIC_FLOAT_Z = 0.35
JOINT_DAMPING = 0.2
TARSUS_DAMPING = 1.0

_FOOT_BODIES = ("fl_foot_link", "fr_foot_link", "rl_foot_link", "rr_foot_link")

# None = 不覆盖, 沿用 MuJoCo geom 默认摩擦 (1, 0.005, 0.0001)
_DEFAULT_GROUND_FRICTION: Optional[Tuple[float, float, float]] = None


@dataclass(frozen=True)
class SimPhysicsOptions:
    """仅仿真侧物理参数 — 不影响 walk.py 实机默认。"""

    # None = MuJoCo 默认摩擦; 显式传入才覆盖地面 geom
    ground_friction: Optional[Tuple[float, float, float]] = _DEFAULT_GROUND_FRICTION
    foot_friction: Optional[Tuple[float, float, float]] = None
    # MuJoCo contact timeconst/dampratio — smaller timeconst = harder floor.
    foot_solref: Tuple[float, float] = (0.02, 1.0)


def _friction_str(f: Tuple[float, float, float]) -> str:
    return f"{f[0]} {f[1]} {f[2]}"


def _motor_to_urdf_vel(joint, motor_vel: float) -> float:
    return motor_vel / joint.sign if joint.sign != 0 else 0.0


def _load_urdf_joint_specs() -> Dict[str, Dict[str, float]]:
    tree = ET.parse(URDF_PATH)
    specs: Dict[str, Dict[str, float]] = {}
    for joint in tree.getroot().findall("joint"):
        jname = joint.get("name", "")
        if not jname.endswith("_joint"):
            continue
        base = jname[: -len("_joint")]
        lim = joint.find("limit")
        if lim is None:
            continue
        specs[base] = {
            "effort": float(lim.get("effort", "0")),
            "lower": float(lim.get("lower", "-3.14159")),
            "upper": float(lim.get("upper", "3.14159")),
        }
    return specs


URDF_JOINT_SPECS = _load_urdf_joint_specs()


def _is_tail_name(name: str) -> bool:
    return "tail" in name.lower()


def _prepare_urdf() -> str:
    tree = ET.parse(URDF_PATH)
    root = tree.getroot()

    for mesh in root.iter("mesh"):
        fn = mesh.get("filename", "")
        if fn.startswith("package://marsdog/meshes/"):
            mesh.set(
                "filename",
                fn.replace("package://marsdog/meshes/", MESH_DIR + "/"),
            )

    for elem in list(root):
        if elem.tag in ("link", "joint") and _is_tail_name(elem.get("name", "")):
            root.remove(elem)

    mj_ext = root.find("mujoco")
    if mj_ext is None:
        mj_ext = ET.SubElement(root, "mujoco")
    comp = mj_ext.find("compiler")
    if comp is None:
        comp = ET.SubElement(mj_ext, "compiler")
    comp.set("fusestatic", "false")

    default = mj_ext.find("default")
    if default is None:
        default = ET.SubElement(mj_ext, "default")
    joint_def = default.find("joint")
    if joint_def is None:
        joint_def = ET.SubElement(default, "joint")
    joint_def.set("damping", str(JOINT_DAMPING))

    for joint in root.findall("joint"):
        jname = joint.get("name", "")
        # 仅焊死没有电机的 tarsus。前腿 tarsus 加电机(进 JOINT_MAP)后保留为 revolute。
        if jname in ("fl_tarsus_joint", "fr_tarsus_joint"):
            base = jname[: -len("_joint")]
            if base in _JOINT_MAP_NAMES:
                continue
            joint.set("type", "fixed")
            for child in list(joint):
                if child.tag in ("axis", "limit", "mimic"):
                    joint.remove(child)

    fd, path = tempfile.mkstemp(suffix=".urdf", prefix="marsdog_sim_")
    os.close(fd)
    tree.write(path, xml_declaration=True, encoding="unicode")
    return path


def _urdf_effort(joint_name: str) -> float:
    if joint_name in _SIM_EFFORT_OVERRIDES:
        return _SIM_EFFORT_OVERRIDES[joint_name]
    if joint_name in _TARSUS_EFFORT_OVERRIDE:
        return _TARSUS_EFFORT_OVERRIDE[joint_name]
    spec = URDF_JOINT_SPECS.get(joint_name)
    if spec and spec["effort"] > 0:
        return spec["effort"]
    return 14.0


def set_sim_effort_overrides(overrides: Optional[Dict[str, float]] = None) -> None:
    """设置临时仿真力矩上限覆盖；必须在构造 MujocoRobot 前调用。"""
    _SIM_EFFORT_OVERRIDES.clear()
    if overrides:
        _SIM_EFFORT_OVERRIDES.update(overrides)


def _print_actuator_calibration() -> None:
    print("[sim] MIT 力矩执行层 (τ = kp·Δq + kd·Δqd + trq_ff, URDF effort 饱和):")
    for j in JOINT_MAP:
        effort = _urdf_effort(j.name)
        g = JOINT_GAINS.get(j.name, DEFAULT_GAIN)
        sat_deg = math.degrees(effort / g["kp"]) if g["kp"] > 0 else 0.0
        print(
            f"  {j.name:16s} effort={effort:5.1f}Nm  "
            f"kp={g['kp']:6.1f} kd={g['kd']:4.1f} trq_ff={g.get('trq_ff', 0):4.2f}  "
            f"P饱和≈{sat_deg:4.1f}°"
        )


def _build_physics_mjcf(physics: Optional[SimPhysicsOptions] = None) -> str:
    phys = physics or SimPhysicsOptions()
    urdf_path = _prepare_urdf()
    tmp_model = mujoco.MjModel.from_xml_path(urdf_path)

    fd, mjcf_path = tempfile.mkstemp(suffix=".xml", prefix="marsdog_phys_")
    os.close(fd)
    mujoco.mj_saveLastXML(mjcf_path, tmp_model)
    os.remove(urdf_path)

    tree = ET.parse(mjcf_path)
    root = tree.getroot()

    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.SubElement(root, "compiler")
    compiler.set("angle", "radian")
    compiler.set("fusestatic", "false")

    option = root.find("option")
    if option is None:
        option = ET.SubElement(root, "option")
    option.set("timestep", str(PHYSICS_DT))
    option.set("gravity", "0 0 -9.81")
    option.set("integrator", "implicitfast")
    option.set("cone", "pyramidal")
    option.set("solver", "Newton")
    option.set("iterations", "50")
    option.set("tolerance", "1e-8")

    # MuJoCo 经典默认蓝场景 (skybox + 深蓝棋盘地板), 不做自定义高摩擦
    visual = root.find("visual")
    if visual is None:
        visual = ET.SubElement(root, "visual")
    rgba = visual.find("rgba")
    if rgba is None:
        rgba = ET.SubElement(visual, "rgba")
    rgba.set("haze", "0.15 0.25 0.35 1")

    asset = root.find("asset")
    if asset is None:
        asset = ET.SubElement(root, "asset")
    sky = ET.SubElement(asset, "texture")
    sky.set("type", "skybox")
    sky.set("builtin", "gradient")
    sky.set("rgb1", "0.3 0.5 0.7")
    sky.set("rgb2", "0 0 0")
    sky.set("width", "512")
    sky.set("height", "3072")
    tex = ET.SubElement(asset, "texture")
    tex.set("name", "texplane")
    tex.set("type", "2d")
    tex.set("builtin", "checker")
    tex.set("width", "512")
    tex.set("height", "512")
    tex.set("rgb1", "0.2 0.3 0.4")
    tex.set("rgb2", "0.1 0.15 0.2")
    tex.set("mark", "cross")
    tex.set("markrgb", "0.8 0.8 0.8")
    mat = ET.SubElement(asset, "material")
    mat.set("name", "matplane")
    mat.set("texture", "texplane")
    mat.set("texrepeat", "1 1")
    mat.set("texuniform", "true")
    mat.set("reflectance", "0.3")

    worldbody = root.find("worldbody")
    gnd = ET.SubElement(worldbody, "geom")
    gnd.set("name", "ground")
    gnd.set("type", "plane")
    gnd.set("size", "0 0 0.125")
    gnd.set("material", "matplane")
    gnd.set("condim", "3")
    # 仅显式传入时才覆盖; 默认走 MuJoCo geom 摩擦 (1, 0.005, 0.0001)
    if phys.ground_friction is not None:
        gnd.set("friction", _friction_str(phys.ground_friction))

    lgt = ET.SubElement(worldbody, "light")
    lgt.set("directional", "true")
    lgt.set("diffuse", "0.8 0.8 0.8")
    lgt.set("specular", "0.3 0.3 0.3")
    lgt.set("pos", "0 0 4")
    lgt.set("dir", "0 0 -1")

    root_body = worldbody.find("body")
    if root_body is not None:
        if not any(j.get("type") == "free" for j in root_body.findall("joint")):
            fj = ET.Element("joint")
            fj.set("name", "root_free")
            fj.set("type", "free")
            root_body.insert(0, fj)

    act_elem = root.find("actuator")
    if act_elem is not None:
        for child in list(act_elem):
            act_elem.remove(child)
    else:
        act_elem = ET.SubElement(root, "actuator")

    for j in JOINT_MAP:
        fmax = _urdf_effort(j.name)
        g = JOINT_GAINS.get(j.name, DEFAULT_GAIN)
        a = ET.SubElement(act_elem, "position")
        a.set("name", f"act_{j.name}")
        a.set("joint", f"{j.name}_joint")
        a.set("kp", f"{g['kp']:.1f}")
        a.set("kv", f"{g['kd']:.1f}")
        a.set("forcelimited", "true")
        a.set("forcerange", f"-{fmax} {fmax}")
        a.set("ctrllimited", "false")
        # Removed ctrlrange to allow feedforward torque to push ctrl outside limits

    tree.write(mjcf_path, xml_declaration=True, encoding="unicode")
    return mjcf_path


class SimRobotBackend(RobotBackend):
    def __init__(
        self,
        stand_controller=None,
        kinematic: bool = False,
        physics_options: Optional[SimPhysicsOptions] = None,
    ):
        if mujoco is None:
            raise RuntimeError("MuJoCo is not installed")
        self._stand = stand_controller
        self._kinematic = kinematic
        self._physics = physics_options
        self._mjcf_path = _build_physics_mjcf(physics_options)
        self.model = mujoco.MjModel.from_xml_path(self._mjcf_path)
        self.data = mujoco.MjData(self.model)
        # Soft-contact scrub: viscous freejoint XY damp while standing/holding
        # (1/s). 0 = off. Walk loop enables this when vx_cmd≈0.
        self._xy_hold_damp = 0.0
        # URDF <limit effort> 会写成 jnt_actfrcrange；仅改 actuator forcerange
        # 不够——力矩仍被关节侧钳死(前腿 tarsus 旧值 0.4119Nm → 跪地)。
        self._apply_joint_actuator_force_limits()
        self._apply_joint_damping()
        self._apply_foot_friction()

        self._joint_qpos: Dict[str, int] = {}
        for j in JOINT_MAP:
            jid = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_JOINT, f"{j.name}_joint"
            )
            if jid >= 0:
                self._joint_qpos[j.name] = self.model.jnt_qposadr[jid]

        self._motor_ctrl: Dict[int, int] = {}
        for j in JOINT_MAP:
            aid = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"act_{j.name}"
            )
            if aid < 0:
                raise RuntimeError(f"缺少执行器 act_{j.name}")
            self._motor_ctrl[j.motor_id] = aid

        self._tarsus_qpos: Dict[str, int] = {}
        self._tarsus_dof: Dict[str, int] = {}
        for name in ("rl_tarsus_joint", "rr_tarsus_joint"):
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid >= 0:
                self._tarsus_qpos[name] = self.model.jnt_qposadr[jid]
                self._tarsus_dof[name] = self.model.jnt_dofadr[jid]

        self._joint_dof: Dict[str, int] = {}
        for j in JOINT_MAP:
            jid = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_JOINT, f"{j.name}_joint"
            )
            if jid >= 0:
                self._joint_dof[j.name] = self.model.jnt_dofadr[jid]

        self._base_qadr = -1
        self._base_dof = -1
        for i in range(self.model.njnt):
            if self.model.jnt_type[i] == mujoco.mjtJoint.mjJNT_FREE:
                self._base_qadr = self.model.jnt_qposadr[i]
                self._base_dof = self.model.jnt_dofadr[i]
                break

        if kinematic:
            self.model.opt.gravity[:] = (0.0, 0.0, 0.0)
            print(
                f"[sim] njnt={self.model.njnt} nu={self.model.nu} | "
                f"运动学悬空 z={KINEMATIC_FLOAT_Z}m"
            )
            self._init_kinematic()
        else:
            print(
                f"[sim] njnt={self.model.njnt} nu={self.model.nu} "
                f"phys={PHYSICS_DT}s ctrl={CONTROL_DT}s ({CONTROL_HZ:.0f}Hz) | "
                f"MIT 力矩 + URDF effort"
            )
            _print_actuator_calibration()
            self._init_standing()

    def _apply_joint_actuator_force_limits(self) -> None:
        """把关节侧 actuatorfrcrange 对齐到电机峰值力矩。

        MuJoCo 3.x: qfrc_actuator = clamp(actuator_force, jnt_actfrcrange)。
        仅写 actuator/@forcerange 时，URDF 旧 effort(如 tarsus 0.4119Nm)仍钳死出力。
        """
        for j in JOINT_MAP:
            jid = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_JOINT, f"{j.name}_joint"
            )
            if jid < 0:
                continue
            fmax = float(_urdf_effort(j.name))
            self.model.jnt_actfrclimited[jid] = 1
            self.model.jnt_actfrcrange[jid, 0] = -fmax
            self.model.jnt_actfrcrange[jid, 1] = fmax

    def _apply_joint_damping(self) -> None:
        for i in range(self.model.njnt):
            if self.model.jnt_type[i] == mujoco.mjtJoint.mjJNT_FREE:
                continue
            dof = self.model.jnt_dofadr[i]
            if dof < 0:
                continue
            jname = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i) or ""
            # 后腿 tarsus 为被动 mimic → 高阻尼; 前腿 tarsus 现为主动电机 → 常规阻尼
            passive_tarsus = "tarsus" in jname and jname.startswith(("rl_", "rr_"))
            damp = TARSUS_DAMPING if passive_tarsus else JOINT_DAMPING
            self.model.dof_damping[dof] = damp
            self.model.dof_armature[dof] = JOINT_ARMATURE

        for fn in _FOOT_BODIES:
            gid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, fn)
            if gid >= 0:
                solref = (
                    self._physics.foot_solref
                    if self._physics is not None
                    else (0.02, 1.0)
                )
                self.model.geom_solref[gid] = [float(solref[0]), float(solref[1])]
                self.model.geom_solimp[gid] = [0.9, 0.95, 0.001, 0.5, 2.0]

    def _apply_foot_friction(self) -> None:
        ff = self._physics.foot_friction if self._physics else None
        if ff is None:
            return
        for fn in _FOOT_BODIES:
            bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, fn)
            if bid < 0:
                continue
            for gid in range(self.model.ngeom):
                if self.model.geom_bodyid[gid] == bid:
                    self.model.geom_friction[gid] = ff

    def _sync_passive_tarsus(self) -> None:
        for m_name, s_name, mult in MIMIC_JOINTS:
            calf_name = s_name.replace("_joint", "")
            cq = self._joint_qpos.get(calf_name)
            tq = self._tarsus_qpos.get(m_name)
            cdof = self._joint_dof.get(calf_name)
            tdof = self._tarsus_dof.get(m_name)
            if cq is None or tq is None or cdof is None or tdof is None:
                continue
            calf_q = self.data.qpos[cq]
            self.data.qpos[tq] = mult * calf_q
            self.data.qvel[tdof] = mult * self.data.qvel[cdof]

    def set_stand_controller(self, stand_controller) -> None:
        self._stand = stand_controller

    def _stand_targets(self) -> Dict[int, float]:
        if self._stand is not None:
            return self._stand.get_targets(0.0)
        from marsdog_control.motion.gait_controller import StandController
        return StandController(0.22).get_targets(0.0)

    def _set_qpos_from_targets(self, targets: Dict[int, float]) -> None:
        """targets 为纯 URDF 关节角 (StandController/规划层输出)。"""
        calf_urdf: Dict[str, float] = {}
        for j in JOINT_MAP:
            qadr = self._joint_qpos.get(j.name)
            if qadr is None:
                continue
            urdf = clamp_urdf(j, targets.get(j.motor_id, 0.0))
            self.data.qpos[qadr] = urdf
            if j.name in ("rl_calf", "rr_calf"):
                calf_urdf[j.name] = urdf

        for m_name, s_name, mult in MIMIC_JOINTS:
            calf_key = s_name.replace("_joint", "")
            calf_val = calf_urdf.get(calf_key)
            if calf_val is None:
                continue
            qadr = self._tarsus_qpos.get(m_name)
            if qadr is not None:
                self.data.qpos[qadr] = mult * calf_val

    def _place_on_ground(self) -> None:
        mujoco.mj_forward(self.model, self.data)
        min_z = float("inf")
        for fn in _FOOT_BODIES:
            bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, fn)
            if bid >= 0:
                min_z = min(min_z, self.data.xpos[bid][2])
        if min_z < 1e5:
            self.data.qpos[self._base_qadr + 2] -= min_z - FOOT_CLEARANCE

    def _init_kinematic(self) -> None:
        targets = self._stand_targets()
        self.data.qpos[:] = 0.0
        self.data.qvel[:] = 0.0
        self.data.ctrl[:] = 0.0
        if self._base_qadr >= 0:
            self.data.qpos[self._base_qadr + 3] = 1.0
            self.data.qpos[self._base_qadr + 2] = KINEMATIC_FLOAT_Z
        self._set_qpos_from_targets(targets)
        self._sync_passive_tarsus()
        mujoco.mj_forward(self.model, self.data)

    def apply_kinematic_targets(self, targets: Dict[int, float]) -> None:
        self._set_qpos_from_targets(targets)
        if self._base_qadr >= 0:
            self.data.qpos[self._base_qadr + 2] = KINEMATIC_FLOAT_Z
        self._sync_passive_tarsus()
        mujoco.mj_forward(self.model, self.data)
        self.data.qvel[:] = 0.0

    def advance_time(self, dt: float) -> None:
        self.data.time += max(dt, 0.0)

    def _init_standing(self) -> None:
        targets = self._stand_targets()
        self.data.qpos[:] = 0.0
        self.data.qvel[:] = 0.0
        self.data.ctrl[:] = 0.0
        if self._base_qadr >= 0:
            self.data.qpos[self._base_qadr + 3] = 1.0

        self._set_qpos_from_targets(targets)
        self.apply_motor_targets(targets)
        self._place_on_ground()

        mujoco.mj_forward(self.model, self.data)
        self.data.qvel[:] = 0.0
        self._sync_passive_tarsus()
        mujoco.mj_forward(self.model, self.data)

    def read_state(self, online_ids: Set[int]) -> RobotState:
        # 从 MuJoCo 读取各关节位置 (URDF 角度)
        urdf_pos = {}
        urdf_vel = {}
        for j in JOINT_MAP:
            qadr = self._joint_qpos.get(j.name)
            vadr = self._joint_dof.get(j.name)
            if qadr is not None:
                urdf_pos[j.motor_id] = self.data.qpos[qadr]
                if vadr is not None:
                    urdf_vel[j.motor_id] = self.data.qvel[vadr]
            elif j.name in ("fl_tarsus", "fr_tarsus"):
                # 如果作为固定关节被融合，则角度为0
                urdf_pos[j.motor_id] = 0.0
                urdf_vel[j.motor_id] = 0.0

        # 从 base 获取姿态
        roll, pitch, wx, wy = self.base_imu()
        vx, vy, vz = self.data.qvel[0], self.data.qvel[1], self.data.qvel[2]
        
        # 这里简单假设所有在线电机都是 enabled 的
        joint_enabled = {mid: True for mid in online_ids}
        
        return RobotState(
            t=self.sim_time,
            joint_pos=urdf_pos,
            joint_enabled=joint_enabled,
            online=online_ids,
            imu_connected=True,
            roll=roll,
            pitch=pitch,
            yaw=self.base_rpy[2],
            gyro_roll=wx,
            gyro_pitch=wy,
            gyro_yaw=self.data.qvel[5], # assuming base wx, wy, wz is 3,4,5
            vel_xyz=(vx, vy, vz),
            joint_vel=urdf_vel,
            imu_age_s=0.0
        )
        
    def send(self, output: ControlOutput) -> None:
        """直接消费纯 URDF 空间的 ControlOutput，下发给 MuJoCo。"""
        self.apply_motor_targets(
            output.target.q,
            velocities=output.target.dq,
            kp_phase=output.kp_phase,
            trq_ff=output.trq_ff,
            kp_scale=output.kp_scale,
            leg_kp_scale=output.leg_kp_scale,
        )

    def apply_motor_targets(
        self,
        targets: Dict[int, float],
        velocities: Optional[Dict[int, float]] = None,
        kp_phase: Optional[Dict[int, float]] = None,
        trq_ff: Optional[Dict[int, float]] = None,
        kp_scale: float = 1.0,
        leg_kp_scale: float = 1.0,
    ) -> None:
        """MIT 力矩(position 执行器等效)。targets/velocities/trq_ff 均为纯 URDF 空间；
        由于管线传下来的已是 URDF 指令，仿真器直接消费，无需再镜像反转 joint.sign。"""
        velocities = velocities or {}
        kp_phase = kp_phase or {}
        trq_ff = trq_ff or {}

        for j in JOINT_MAP:
            mid = j.motor_id
            aid = self._motor_ctrl.get(mid)
            if aid is None:
                continue

            ps = kp_phase.get(mid, 1.0)
            to = trq_ff.get(mid)

            g = JOINT_GAINS.get(j.name, DEFAULT_GAIN)
            is_leg = j.name[:3] in ("fl_", "fr_", "rl_", "rr_")
            # 与实机 resolve_gains 一致: kp *= kp_scale * leg_kp_scale * phase
            leg_s = (leg_kp_scale if is_leg else 1.0) * ps

            kp = g["kp"] * kp_scale * leg_s
            kd = g["kd"]
            trq = to if to is not None else g.get("trq_ff", 0.0)

            self.model.actuator_gainprm[aid, 0] = kp
            if self.model.actuator_gainprm.shape[1] > 1:
                self.model.actuator_gainprm[aid, 1] = kd

            # Target 已是纯 URDF 角(经 SafetySupervisor 兜底限位)，直接下发
            q_des = clamp_urdf(j, targets.get(mid, 0.0))
            qd_des = velocities.get(mid, 0.0)

            ctrl = q_des
            if abs(kp) > 1e-6:
                ctrl += (kd / kp) * qd_des + trq / kp
            # Do not clamp ctrl here, allow it to exceed limits for feedforward torque

            self.data.ctrl[aid] = ctrl

    def shutdown(self, reason: str = "") -> None:
        pass

    def set_xy_hold_damp(self, damp_per_s: float) -> None:
        """Viscous freejoint XY damp (1/s). Use while standing to kill scrub."""
        self._xy_hold_damp = max(0.0, float(damp_per_s))

    def step(self, n: int = N_SUBSTEPS) -> None:
        damp = float(self._xy_hold_damp)
        decay = math.exp(-damp * PHYSICS_DT) if damp > 1e-9 else 1.0
        for _ in range(n):
            self._sync_passive_tarsus()
            mujoco.mj_step(self.model, self.data)
            self._sync_passive_tarsus()
            if decay < 1.0:
                # Freejoint linear XY only — leave yaw/vertical alone.
                self.data.qvel[0] *= decay
                self.data.qvel[1] *= decay

    @property
    def sim_time(self) -> float:
        return self.data.time

    @property
    def base_pos(self) -> np.ndarray:
        return self.data.qpos[self._base_qadr: self._base_qadr + 3].copy()

    @property
    def com_xy(self) -> np.ndarray:
        masses = self.model.body_mass[1:]
        total = float(np.sum(masses))
        if total <= 1e-9:
            return self.base_pos[:2]
        com = np.sum(self.data.xipos[1:] * masses[:, None], axis=0) / total
        return com[:2].copy()

    def foot_positions_world(self) -> Dict[str, np.ndarray]:
        result: Dict[str, np.ndarray] = {}
        for name in _FOOT_BODIES:
            bid = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_BODY, name
            )
            if bid >= 0:
                result[name[:2]] = self.data.xpos[bid].copy()
        return result

    def foot_speeds_world(self) -> Dict[str, float]:
        result: Dict[str, float] = {}
        vel = np.zeros(6, dtype=float)
        for name in _FOOT_BODIES:
            bid = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_BODY, name
            )
            if bid < 0:
                continue
            mujoco.mj_objectVelocity(
                self.model, self.data, mujoco.mjtObj.mjOBJ_BODY,
                bid, vel, 0,
            )
            result[name[:2]] = float(np.linalg.norm(vel[3:5]))
        return result

    def foot_contacts(self) -> set[str]:
        foot_bodies = {}
        for name in _FOOT_BODIES:
            bid = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_BODY, name
            )
            if bid >= 0:
                foot_bodies[bid] = name[:2]
        contacts: set[str] = set()
        for i in range(self.data.ncon):
            contact = self.data.contact[i]
            for gid in (contact.geom1, contact.geom2):
                bid = int(self.model.geom_bodyid[gid])
                leg = foot_bodies.get(bid)
                if leg is not None:
                    contacts.add(leg)
        return contacts

    def foot_contact_slip_speeds(self) -> Dict[str, float]:
        """返回各足实际接触点相对世界的水平速度，而非 link 原点速度。"""
        foot_bodies = {}
        for name in _FOOT_BODIES:
            bid = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_BODY, name
            )
            if bid >= 0:
                foot_bodies[bid] = name[:2]
        result: Dict[str, float] = {}
        spatial = np.zeros(6, dtype=float)
        for i in range(self.data.ncon):
            contact = self.data.contact[i]
            for gid in (contact.geom1, contact.geom2):
                bid = int(self.model.geom_bodyid[gid])
                leg = foot_bodies.get(bid)
                if leg is None:
                    continue
                mujoco.mj_objectVelocity(
                    self.model, self.data, mujoco.mjtObj.mjOBJ_BODY,
                    bid, spatial, 0,
                )
                omega = spatial[:3]
                linear = spatial[3:]
                radius = contact.pos - self.data.xpos[bid]
                point_vel = linear + np.cross(omega, radius)
                speed = float(np.linalg.norm(point_vel[:2]))
                result[leg] = max(result.get(leg, 0.0), speed)
        return result

    @property
    def base_quat(self) -> np.ndarray:
        return self.data.qpos[self._base_qadr + 3: self._base_qadr + 7].copy()

    @property
    def base_rpy(self) -> tuple:
        q = self.base_quat
        w, x, y, z = q[0], q[1], q[2], q[3]
        sinr = 2.0 * (w * x + y * z)
        cosr = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr, cosr)
        sinp = 2.0 * (w * y - z * x)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = math.asin(sinp)
        siny = 2.0 * (w * z + x * y)
        cosy = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny, cosy)
        return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

    def base_imu(self) -> tuple[float, float, float, float]:
        r, p, _ = self.base_rpy
        roll = math.radians(r)
        pitch = math.radians(p)
        if self._base_dof >= 0:
            q = self.base_quat
            w, x, y, z = q[0], q[1], q[2], q[3]
            omega_w = self.data.qvel[self._base_dof + 3: self._base_dof + 6]
            uv = np.cross([x, y, z], omega_w)
            uuv = np.cross([x, y, z], uv)
            omega_b = omega_w + 2.0 * (w * uv + uuv)
            wx, wy = float(omega_b[0]), float(omega_b[1])
        else:
            wx = wy = 0.0
        return roll, pitch, wx, wy

    def reset_standing(self) -> None:
        x, y = self.base_pos[0], self.base_pos[1]
        self._init_standing()
        self.data.qpos[self._base_qadr + 0] = x
        self.data.qpos[self._base_qadr + 1] = y
        mujoco.mj_forward(self.model, self.data)
        self.data.qvel[:] = 0.0

    def apply_roll_impulse(self, roll_rate_deg_s: float) -> None:
        if self._base_dof >= 0:
            self.data.qvel[self._base_dof + 3] += math.radians(roll_rate_deg_s)

    def close(self) -> None:
        if self._mjcf_path and os.path.isfile(self._mjcf_path):
            try:
                os.remove(self._mjcf_path)
            except OSError:
                pass
