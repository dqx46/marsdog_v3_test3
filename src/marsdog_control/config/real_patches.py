"""Real-robot / gait overlay patches — inventory + named profiles.

Core locomotion geometry (period / stance / amp / step_h / touchdown_compress)
is NOT zeroed by the ``parity`` profile. Overlay compensations are.

Profiles (preferred operator surface):
  * ``parity``  — all overlays off (``--sim-parity``)
  * ``real_soft`` — SoftTrot default: only ``com_shift`` ON
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List, Mapping, Optional, Tuple


@dataclass(frozen=True)
class PatchSpec:
    """One compensation with a CLI dest and ON predicate."""

    key: str
    label: str
    dest: str
    layer: str  # imu | actuator | gait_overlay | persistence
    parity_value: Any
    kind: str = "bool"  # bool | nonzero | abs_nonzero | not_one


PATCHES: Tuple[PatchSpec, ...] = (
    PatchSpec(
        "imu_enable", "IMU足高闭环", "imu", "imu",
        parity_value=False, kind="bool",
    ),
    PatchSpec(
        "imu_softstart", "IMU软启动", "imu_softstart_s", "imu",
        parity_value=0.0, kind="nonzero",
    ),
    PatchSpec(
        "imu_predict", "IMU预测超前", "imu_predict_ms", "imu",
        parity_value=0.0, kind="nonzero",
    ),
    PatchSpec(
        "dynamic_imu_predict", "动态angle年龄预测", "dynamic_imu_predict", "imu",
        parity_value=False, kind="bool",
    ),
    PatchSpec(
        "imu_phase_gate", "IMU相位门控", "imu_phase_gate", "imu",
        parity_value=False, kind="bool",
    ),
    PatchSpec(
        "td_imu_freeze_i", "触地冻积分", "td_imu_freeze_i", "imu",
        parity_value=False, kind="bool",
    ),
    PatchSpec(
        "imu_slew", "IMU斜率限制", "imu_slew_mm_s", "imu",
        parity_value=0.0, kind="nonzero",
    ),
    PatchSpec(
        "ff_decouple", "expected_roll前馈解耦", "ff_decouple", "imu",
        parity_value=False, kind="bool",
    ),
    PatchSpec(
        "swing_level", "摆动腿IMU预调平", "swing_level", "imu",
        parity_value=0.0, kind="nonzero",
    ),
    PatchSpec(
        "var_impedance", "相位可变阻抗", "var_impedance", "actuator",
        parity_value=False, kind="bool",
    ),
    PatchSpec(
        "tarsus_lead_fl", "达妙tarsus lead FL", "tarsus_lead_fl_ms", "actuator",
        parity_value=0.0, kind="nonzero",
    ),
    PatchSpec(
        "tarsus_lead_fr", "达妙tarsus lead FR", "tarsus_lead_fr_ms", "actuator",
        parity_value=0.0, kind="nonzero",
    ),
    PatchSpec(
        "dm_dq_ff", "达妙dq前馈", "dm_dq_feedforward", "actuator",
        parity_value=False, kind="bool",
    ),
    PatchSpec(
        "anti_roll", "支撑anti_roll伸腿", "anti_roll", "gait_overlay",
        parity_value=0.0, kind="abs_nonzero",
    ),
    PatchSpec(
        "lateral_sway", "半正弦lateral_sway", "lateral_sway", "gait_overlay",
        parity_value=0.0, kind="abs_nonzero",
    ),
    PatchSpec(
        "trot_roll_ff", "trot_roll_ff预期侧倾", "trot_roll_ff_neg_deg", "gait_overlay",
        parity_value=0.0, kind="abs_nonzero",
    ),
    PatchSpec(
        "com_shift", "事件型com_shift移重", "com_shift_m", "gait_overlay",
        parity_value=0.0, kind="abs_nonzero",
    ),
    PatchSpec(
        "rear_clearance", "后腿rear_clearance", "rear_clearance_m", "gait_overlay",
        parity_value=0.0, kind="abs_nonzero",
    ),
    PatchSpec(
        "spine_yaw", "脊柱spine_yaw", "spine_yaw_deg", "gait_overlay",
        parity_value=0.0, kind="abs_nonzero",
    ),
    PatchSpec(
        "spine_roll", "脊柱spine_roll", "spine_roll_deg", "gait_overlay",
        parity_value=0.0, kind="abs_nonzero",
    ),
    PatchSpec(
        "thigh_flourish", "后腿thigh flourish", "thigh_swing_rear_deg", "gait_overlay",
        parity_value=0.0, kind="abs_nonzero",
    ),
    PatchSpec(
        "swing_track_gate", "摆动足跟踪门控(<1)", "front_foot_swing_track",
        "gait_overlay",
        parity_value=1.0, kind="not_one",
    ),
    PatchSpec(
        "stance_push", "支撑蹬地角push", "front_foot_stance_push_deg",
        "gait_overlay",
        parity_value=0.0, kind="abs_nonzero",
    ),
)


def _parity_overrides() -> Dict[str, Any]:
    out = {p.dest: p.parity_value for p in PATCHES}
    out["trot_roll_ff_pos_deg"] = 0.0
    out["roll_trim_mm"] = 0.0
    out["pitch_trim_mm"] = 0.0
    out["thigh_swing_front_deg"] = 0.0
    return out


@dataclass(frozen=True)
class PatchProfile:
    """Named overlay bundle — prefer this over flipping 23 switches."""

    name: str
    overrides: Mapping[str, Any]
    # Keys expected ON after profile + Soft pour (for banners / tests).
    expect_on: frozenset = frozenset()


PROFILE_PARITY = PatchProfile(
    name="parity",
    overrides=_parity_overrides(),
    expect_on=frozenset(),
)

# SoftTrot real default: overlays off except lateral com_shift (recipe).
# Overrides here only force OFF extras that Soft pour might leave; com_shift
# stays from SoftTrotRecipe (not listed → not clobbered by profile apply).
PROFILE_REAL_SOFT = PatchProfile(
    name="real_soft",
    overrides={
        "imu": False,
        "imu_softstart_s": 0.0,
        "imu_predict_ms": 0.0,
        "dynamic_imu_predict": False,
        "imu_phase_gate": False,
        "td_imu_freeze_i": False,
        "imu_slew_mm_s": 0.0,
        "ff_decouple": False,
        "swing_level": 0.0,
        "var_impedance": False,
        "tarsus_lead_fl_ms": 0.0,
        "tarsus_lead_fr_ms": 0.0,
        "dm_dq_feedforward": False,
        "anti_roll": 0.0,
        "lateral_sway": 0.0,
        "trot_roll_ff_neg_deg": 0.0,
        "trot_roll_ff_pos_deg": 0.0,
        "rear_clearance_m": 0.0,
        "spine_yaw_deg": 0.0,
        "spine_roll_deg": 0.0,
        "thigh_swing_rear_deg": 0.0,
        "thigh_swing_front_deg": 0.0,
        "front_foot_swing_track": 1.0,
        "front_foot_stance_push_deg": 0.0,
    },
    expect_on=frozenset({"com_shift"}),
)

PROFILES: Dict[str, PatchProfile] = {
    PROFILE_PARITY.name: PROFILE_PARITY,
    PROFILE_REAL_SOFT.name: PROFILE_REAL_SOFT,
}

# Back-compat alias used by older imports / tests.
SIM_PARITY_OVERRIDES: Dict[str, Any] = dict(PROFILE_PARITY.overrides)


def _is_on(args: Any, spec: PatchSpec) -> bool:
    if not hasattr(args, spec.dest):
        return False
    v = getattr(args, spec.dest)
    if spec.kind == "bool":
        return bool(v)
    if spec.kind == "nonzero":
        return abs(float(v)) > 1e-12
    if spec.kind == "abs_nonzero":
        return abs(float(v)) > 1e-9
    if spec.kind == "not_one":
        return abs(float(v) - 1.0) > 0.05
    return bool(v)


def apply_patch_profile(
    args: Any,
    profile: PatchProfile,
    *,
    force: bool = False,
) -> List[str]:
    """Apply profile overrides for non-explicit CLI dests. Returns applied keys."""
    explicit = set(getattr(args, "_explicit_cli", set()))
    applied: List[str] = []
    for dest, value in profile.overrides.items():
        if not force and dest in explicit:
            continue
        if hasattr(args, dest):
            setattr(args, dest, value)
            applied.append(dest)
    explicit |= set(profile.overrides.keys())
    args._explicit_cli = explicit
    args.patch_profile = profile.name
    if profile.name == "parity":
        args.sim_parity = True
        explicit.add("sim_parity")
        args._explicit_cli = explicit
    return sorted(applied)


def apply_sim_parity(args: Any) -> List[str]:
    """Apply ``parity`` profile when ``--sim-parity`` is set."""
    if not bool(getattr(args, "sim_parity", False)):
        return []
    return apply_patch_profile(args, PROFILE_PARITY)


def resolve_patch_profile(args: Any) -> Optional[str]:
    """Return active profile name for banners (parity / real_soft / None)."""
    named = getattr(args, "patch_profile", None)
    if named:
        return str(named)
    if bool(getattr(args, "sim_parity", False)):
        return PROFILE_PARITY.name
    if bool(getattr(args, "natural_soft_trot", False)):
        on = {k for k, is_on, _ in patch_status(args) if is_on}
        if on <= PROFILE_REAL_SOFT.expect_on or on == PROFILE_REAL_SOFT.expect_on:
            return PROFILE_REAL_SOFT.name
    return None


def patch_status(args: Any) -> List[Tuple[str, bool, str]]:
    out: List[Tuple[str, bool, str]] = []
    for spec in PATCHES:
        out.append((spec.key, _is_on(args, spec), spec.label))
    return out


def format_patch_banner(args: Any) -> str:
    lines = ["[patches] 叠加补偿状态 (核心几何 period/amp/step_h 不在此列):"]
    profile = resolve_patch_profile(args)
    if profile == "parity":
        lines.append("  profile=parity  (未显式指定的补丁已关闭)")
    elif profile == "real_soft":
        lines.append("  profile=real_soft  (默认仅 com_shift)")
    on_n = 0
    for key, is_on, label in patch_status(args):
        mark = "ON " if is_on else "off"
        if is_on:
            on_n += 1
        lines.append(f"  [{mark}] {key:<22} {label}")
    lines.append(
        f"  [core] T={float(getattr(args, 'nat_period', getattr(args, 'period', 0))):.2f}s "
        f"amp={float(getattr(args, 'nat_amp_front', 0))*100:.1f}/"
        f"{float(getattr(args, 'nat_amp_rear', 0))*100:.1f}cm "
        f"step_h={float(getattr(args, 'nat_step_h', getattr(args, 'step_h', 0)))*100:.1f}cm "
        f"td_compress={float(getattr(args, 'touchdown_compress', 0))*1000:.1f}mm"
    )
    lines.append(f"  summary: {on_n}/{len(PATCHES)} overlays ON")
    return "\n".join(lines)


def print_patch_banner(args: Any) -> None:
    print(format_patch_banner(args))


__all__ = [
    "PATCHES",
    "PROFILES",
    "PROFILE_PARITY",
    "PROFILE_REAL_SOFT",
    "PatchProfile",
    "PatchSpec",
    "SIM_PARITY_OVERRIDES",
    "apply_patch_profile",
    "apply_sim_parity",
    "format_patch_banner",
    "patch_status",
    "print_patch_banner",
    "resolve_patch_profile",
]
