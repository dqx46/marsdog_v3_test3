"""Front/rear leg IK target writers for StableTrot.get_targets.

Keeps sagittal/foot-orient IK and abduction overlays out of stable_trot.py.
"""

from __future__ import annotations

import math
from typing import Any, Optional

from marsdog_control.motion.foot_overlays import compose_foot_body_y
from marsdog_control.motion.gait_base import _cmd
from marsdog_control.motion.kinematics import (
    front_tarsus_schedule,
    front_thigh_roll_abd_urdf,
    ik_front_3link,
    ik_front_3link_foot_orient,
    ik_rear_leg_2d,
    solve_front_calf_for_z_3link,
)


def solve_front_legs(
    gait: Any,
    targets: dict,
    *,
    t: float,
    ramp: float,
    lat_offset: float,
    reactive: float,
    dx_raibert: float,
    z_front_base: float,
    hip0_f: float,
    imu_dz: Optional[dict] = None,
    imu_state: Optional[dict] = None,
) -> None:
    for leg in ('fl', 'fr'):
        phase = (t / gait.period + gait._PHASE_OFFSET[leg]) % 1.0
        x_u, lift = gait._leg_xz(leg, t, gait._turn_filtered)
        cx = gait.x_offset_front
        x_u = cx + (x_u - cx) * ramp + dx_raibert * ramp
        lift *= ramp
        z_u = z_front_base + lift
        if imu_dz:
            if getattr(gait, "spot_turn_active", False):
                w_st = 0.0 if gait.spot_leg_in_swing(leg) else 1.0
                z_u += imu_dz.get(leg, 0.0) * w_st
            else:
                z_u += imu_dz.get(leg, 0.0) * gait._stance_weight(phase)

        in_swing = (
            gait.spot_leg_in_swing(leg)
            if getattr(gait, "spot_turn_active", False)
            else phase >= gait.stance_ratio
        )
        is_left = leg.endswith("l")
        if in_swing and gait.swing_clearance_per_rad > 0 and imu_state:
            roll = imu_state.get('roll', 0.0)
            # roll>0 左高; 本侧偏低时 low_side>0 → 抬高摆动足端
            low_side = (-roll if is_left else roll)
            if low_side > 0:
                z_u += low_side * gait.swing_clearance_per_rad * gait.body_height * ramp

        if gait.front_foot_track_deg is not None:
            # 足朝向跟踪 (RL 策略: 脚尖时刻指地) — 三关节协同 3-DOF IK。
            # ramp 起点 = 站立足朝向(tarsus≈0), 平滑过渡到目标朝向。
            target_fp = math.radians(gait.front_foot_track_deg)
            # 支撑相额外前倾蹬地 — spot 必须关，否则原地转变慢速前进
            if (
                not in_swing
                and gait.front_foot_stance_push_deg != 0.0
                and not getattr(gait, "spot_turn_active", False)
            ):
                s = phase / gait.stance_ratio
                target_fp -= math.radians(
                    gait.front_foot_stance_push_deg) * math.sin(math.pi * s)
            # 摆动相回站立朝向(gate<1): 减少 tarsus 甩动与反作用力矩
            gate = gait._foot_track_gate(phase)
            track_fp = (
                gait._front_stand_foot_pitch
                + gate * (target_fp - gait._front_stand_foot_pitch)
            )
            foot_pitch = (
                gait._front_stand_foot_pitch
                + ramp * (track_fp - gait._front_stand_foot_pitch)
            )
            hip_u, calf_u, tarsus_u = ik_front_3link_foot_orient(
                x_u, z_u, foot_pitch)
        else:
            # 旧法: tarsus 支撑相 sin 蹬地鼓包, 摆动相中性
            tarsus_u = front_tarsus_schedule(
                phase, gait.stance_ratio, gait.front_tarsus_push) * ramp
            # 3-link IK: 给定足端 (x,z) 与脚踝角求 hip+calf。tarsus=0 时退化为旧 2-link。
            hip_ik, calf_ik = ik_front_3link(x_u, z_u, tarsus_u)
            thrust_g = (
                gait.front_thrust_swing_gain if in_swing
                else gait.front_thrust_gain
            )
            if thrust_g >= 0.999:
                hip_u, calf_u = hip_ik, calf_ik
            else:
                # 解耦(削弱前腿推进)保留为可调回退: 衰减大腿摆动, calf 解算保持足端 Z
                hip_u = hip0_f + thrust_g * (hip_ik - hip0_f)
                calf_u = solve_front_calf_for_z_3link(
                    hip_u, z_u, tarsus_u, calf_init=calf_ik)
        mid_hp, cmd_hp = _cmd(f'{leg}_hip_pitch', hip_u)
        mid_ca, cmd_ca = _cmd(f'{leg}_calf', calf_u)
        targets[mid_hp] = cmd_hp
        targets[mid_ca] = cmd_ca

        mid_ta, cmd_ta = _cmd(f'{leg}_tarsus', tarsus_u)
        targets[mid_ta] = cmd_ta

        # Body-Y overlays → abd (sway / reactive / turn).
        if getattr(gait, "spot_turn_active", False):
            y_turn = gait._spot.cached_xy(leg)[1]
            reactive_active = False
        else:
            y_turn = gait._leg_y_turn(leg, t, gait._turn_filtered) * ramp
            reactive_active = phase >= gait.stance_ratio
        y_total = compose_foot_body_y(
            lat_offset=lat_offset,
            reactive=reactive,
            body_height=gait.body_height,
            reactive_active=reactive_active,
            y_turn=y_turn,
        )

        # Convert to abduction joint angle
        abd_delta = gait._y_body_to_abd_roll(leg, y_total)

        roll_angle = front_thigh_roll_abd_urdf(leg, gait.hip_abduction) + abd_delta
        mid_tr, cmd_tr = _cmd(f'{leg}_thigh_roll', roll_angle)
        targets[mid_tr] = cmd_tr


def solve_rear_legs(
    gait: Any,
    targets: dict,
    *,
    t: float,
    ramp: float,
    lat_offset: float,
    reactive: float,
    dx_raibert: float,
    z_rear_base: float,
    imu_dz: Optional[dict] = None,
) -> None:
    for leg in ('rl', 'rr'):
        phase = (t / gait.period + gait._PHASE_OFFSET[leg]) % 1.0
        x_u, lift = gait._leg_xz(leg, t, gait._turn_filtered)
        cx = gait.x_offset_rear
        x_u = cx + (x_u - cx) * ramp + dx_raibert * ramp
        lift *= ramp
        z_u = z_rear_base + lift
        if imu_dz:
            if getattr(gait, "spot_turn_active", False):
                # FSM stance only — trot phase was fighting catch diagonals.
                w_st = 0.0 if gait.spot_leg_in_swing(leg) else 1.0
                z_u += imu_dz.get(leg, 0.0) * w_st
            else:
                z_u += imu_dz.get(leg, 0.0) * gait._stance_weight(phase)

        thigh_u, calf_u = ik_rear_leg_2d(x_u, z_u)
        mid_th, cmd_th = _cmd(f'{leg}_thigh', thigh_u)
        mid_ca, cmd_ca = _cmd(f'{leg}_calf', calf_u)
        targets[mid_th] = cmd_th
        targets[mid_ca] = cmd_ca

        # 由于 URDF 已在 2026-07 修正为对称语义，所有横向关节正值均代表外展
        # 统一使用 _y_body_to_abd_roll 将 Y 轴偏移转换为外展角
        if getattr(gait, "spot_turn_active", False):
            y_turn = gait._spot.cached_xy(leg)[1]
            reactive_active = False
        else:
            y_turn = gait._leg_y_turn(leg, t, gait._turn_filtered) * ramp
            reactive_active = phase >= gait.stance_ratio
        y_total = compose_foot_body_y(
            lat_offset=lat_offset,
            reactive=reactive,
            body_height=gait.body_height,
            reactive_active=reactive_active,
            y_turn=y_turn,
        )

        # Convert to abduction joint angle
        abd_delta = gait._y_body_to_abd_roll(leg, y_total)

        hip_roll = gait.hip_abduction + abd_delta
        mid_hr, cmd_hr = _cmd(f'{leg}_hip', hip_roll)
        targets[mid_hr] = cmd_hr

__all__ = ["solve_front_legs", "solve_rear_legs"]
