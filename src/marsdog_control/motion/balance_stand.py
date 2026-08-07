"""BalanceStandPlanner — fixed world feet, shift via walk-convention foot offsets.

Used by ``sim_com_balance`` to manually search horizontal foot shifts while
keeping contact points approximately planted.

**Same sign as** ``run_walk --x-shift`` / ``--y-shift``:
  positive x_shift = feet forward (body CoM aft relative to feet)
  positive y_shift = feet left / +Y (body CoM right relative to feet)

Copy the printed values straight into ``./run_walk.sh --x-shift … --y-shift …``.
"""

from __future__ import annotations

import math
from typing import Dict, Optional

from marsdog_control.config.joints import JOINT_BY_NAME
from marsdog_control.motion.gait_controller import StandController
from marsdog_control.motion.kinematics import (
    FL_HIP_Z,
    RL_HIP_Z,
    WAIST_Z,
    front_standing_foot_pitch,
    front_thigh_roll_abd_urdf,
    ik_front_3link_foot_orient,
    ik_rear_leg_2d,
)

_FRONT_HIP_OFFSET = abs(WAIST_Z + FL_HIP_Z)
_REAR_HIP_OFFSET = abs(RL_HIP_Z)

_LIFT_LEGS = ("fl", "rr")


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _y_body_to_abd_roll(leg: str, y_body: float, body_height: float) -> float:
    """Body-frame foot Y (+left) → abduction joint delta (+ = outward both sides)."""
    side = 1.0 if leg.endswith("l") else -1.0
    lever = max(1e-3, float(body_height))
    return side * float(y_body) / lever


class BalanceStandPlanner:
    """Nominal stand feet + walk-convention x/y foot shift IK + optional FL–RR lift."""

    def __init__(
        self,
        stand: StandController,
        *,
        lift_z_m: float = 0.025,
        x_shift: float = 0.0,
        y_shift: float = 0.0,
        # Back-compat aliases (body-CoM convention, opposite of walk foot shift).
        com_x_m: Optional[float] = None,
        com_y_m: Optional[float] = None,
    ) -> None:
        self.stand = stand
        if com_x_m is not None:
            x_shift = -float(com_x_m)
        if com_y_m is not None:
            y_shift = -float(com_y_m)
        self.x_shift = float(x_shift)
        self.y_shift = float(y_shift)
        self.diag_lifted = False
        self.lift_z_m = float(lift_z_m)
        # Stand should be built with x/y_shift=0; planner owns absolute walk flags.
        self._base_x_front = float(stand.x_offset_front)
        self._base_x_rear = float(stand.x_offset_rear)
        self._base_y = float(getattr(stand, "y_offset", 0.0) or 0.0)
        self._foot_pitch = front_standing_foot_pitch(
            stand.body_height,
            stand.x_offset_front,
            stand.front_stand_tarsus_deg,
            foot_pitch=math.radians(stand.front_stand_foot_pitch_deg),
        )

    # --- aliases matching old body-CoM naming (sign flipped) ---
    @property
    def com_x_m(self) -> float:
        return -self.x_shift

    @com_x_m.setter
    def com_x_m(self, v: float) -> None:
        self.x_shift = -float(v)

    @property
    def com_y_m(self) -> float:
        return -self.y_shift

    @com_y_m.setter
    def com_y_m(self, v: float) -> None:
        self.y_shift = -float(v)

    def reset_com(self) -> None:
        self.x_shift = 0.0
        self.y_shift = 0.0

    def reset_shift(self) -> None:
        self.reset_com()

    def nudge_com(self, dx: float = 0.0, dy: float = 0.0) -> None:
        """Nudge in walk foot-shift space (same as nudge_shift)."""
        self.nudge_shift(dx=dx, dy=dy)

    def nudge_shift(self, dx: float = 0.0, dy: float = 0.0) -> None:
        self.x_shift += float(dx)
        self.y_shift += float(dy)

    def lift_diag(self) -> None:
        self.diag_lifted = True

    def plant_diag(self) -> None:
        self.diag_lifted = False

    def get_targets(self) -> Dict[int, float]:
        stand = self.stand
        h = float(stand.body_height)
        z_front0 = -(h - _FRONT_HIP_OFFSET)
        z_rear0 = -(h - _REAR_HIP_OFFSET)
        abd0 = float(stand.hip_abduction)
        sx, sy = float(self.x_shift), float(self.y_shift)

        # Walk convention: feet = base + shift (same as --x-shift / --y-shift).
        x_f = self._base_x_front + sx
        x_r = self._base_x_rear + sx
        y = self._base_y + sy
        feet = {
            "fl": (x_f, y, z_front0),
            "fr": (x_f, y, z_front0),
            "rl": (x_r, y, z_rear0),
            "rr": (x_r, y, z_rear0),
        }
        if self.diag_lifted:
            for leg in _LIFT_LEGS:
                x, yy, z = feet[leg]
                feet[leg] = (x, yy, z + self.lift_z_m)

        targets: Dict[int, float] = {}

        for leg in ("fl", "fr"):
            x, yy, z = feet[leg]
            hip_u, calf_u, tarsus_u = ik_front_3link_foot_orient(
                x, z, self._foot_pitch)
            calf_u = _clamp(calf_u, -1.82, 1.93)
            abd = front_thigh_roll_abd_urdf(leg, abd0) + _y_body_to_abd_roll(
                leg, yy, h)
            for jname, ang in (
                (f"{leg}_hip_pitch", hip_u),
                (f"{leg}_calf", calf_u),
                (f"{leg}_tarsus", tarsus_u),
                (f"{leg}_thigh_roll", abd),
            ):
                j = JOINT_BY_NAME[jname]
                targets[j.motor_id] = ang

        for leg in ("rl", "rr"):
            x, yy, z = feet[leg]
            thigh_u, calf_u = ik_rear_leg_2d(x, z)
            calf_u = _clamp(calf_u, -0.5, 1.56)
            abd = abd0 + _y_body_to_abd_roll(leg, yy, h)
            for jname, ang in (
                (f"{leg}_thigh", thigh_u),
                (f"{leg}_calf", calf_u),
                (f"{leg}_hip", abd),
            ):
                j = JOINT_BY_NAME[jname]
                targets[j.motor_id] = ang

        # Waist / head / unused zeros — match stand pose offsets.
        for name in (
            "waist_roll",
            "head_pitch",
            "head_yaw",
            "head_roll",
            "neck_pitch",
        ):
            j = JOINT_BY_NAME[name]
            targets[j.motor_id] = 0.0
        j_wp = JOINT_BY_NAME["waist_pitch"]
        targets[j_wp.motor_id] = _clamp(
            stand.waist_pitch_offset, j_wp.limit_lo, j_wp.limit_hi)
        j_wy = JOINT_BY_NAME["waist_yaw"]
        targets[j_wy.motor_id] = _clamp(
            stand.waist_yaw_offset, j_wy.limit_lo, j_wy.limit_hi)

        return targets

    def describe(self) -> str:
        lift = "FL+RR_up" if self.diag_lifted else "quad"
        return (
            f"x_shift={self.x_shift:+.4f} y_shift={self.y_shift:+.4f} "
            f"support={lift} lift_z={self.lift_z_m:.3f}"
        )

    def walk_cli_hint(self) -> str:
        return (
            f"./run_walk.sh --x-shift {self.x_shift:.4f} "
            f"--y-shift {self.y_shift:.4f}"
        )


def pinocchio_com_in_base(stand: StandController) -> Optional[tuple]:
    """Optional: URDF CoM (x,y,z) in base/world at stand pose (base at origin)."""
    try:
        import os

        import pinocchio as pin
        from marsdog_control.control.nmpc_reduced_model import (
            QuadrupedReducedModel,
            default_urdf_path,
        )
    except Exception:
        return None
    try:
        urdf = default_urdf_path()
        if not os.path.isfile(urdf):
            # default_urdf_path may overshoot to sibling ../marsdog; prefer repo-local.
            here = os.path.dirname(os.path.abspath(__file__))
            alt = os.path.abspath(
                os.path.join(here, "../../../marsdog/urdf/marsdog.urdf")
            )
            urdf = alt if os.path.isfile(alt) else urdf
        if not os.path.isfile(urdf):
            return None
        rm = QuadrupedReducedModel(urdf_path=urdf)
        q = pin.neutral(rm.model)
        targets = stand.get_targets(0.0)
        for jdesc in JOINT_BY_NAME.values():
            mid = jdesc.motor_id
            if mid not in targets:
                continue
            jname = f"{jdesc.name}_joint"
            idx = rm._joint_idx_q.get(jname)
            if idx is None:
                continue
            q[idx] = float(targets[mid])
        rm.apply_rear_tarsus_mimic(q)
        pin.centerOfMass(rm.model, rm.data, q)
        com = rm.data.com[0]
        return (float(com[0]), float(com[1]), float(com[2]))
    except Exception:
        return None
