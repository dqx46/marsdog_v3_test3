"""Terminal status and periodic health reporting for the runtime loop."""

from __future__ import annotations

import math
import sys
import time
from dataclasses import dataclass, field
from typing import Sequence


@dataclass
class RuntimeStatusDisplay:
    """Owns human-facing loop status cadence."""

    real_joints: Sequence
    dm_fixed_targets: dict
    print_interval_s: float = 0.4
    health_interval_s: float = 10.0
    clock: object = time
    next_print: float = field(init=False)
    next_health: float = field(init=False)

    def __post_init__(self) -> None:
        now = self.clock.time()
        self.next_print = now
        self.next_health = now + self.health_interval_s

    def update(self, *, mode: str, height: float, active_gait, cmd,
               imu, imu_dz, lie_down_hold: bool, joint_direction_test: bool,
               hip_abd_test: bool, leg_pitch_test: bool,
               direction_test_start: float, direction_test_duration_s: float,
               lz, evo, incos=None, board=None,
               pose_hold_name: str | None = None,
               abd_flare_active: bool = False) -> None:
        now = self.clock.time()
        if now >= self.next_print:
            self._print_loop_status(
                mode=mode,
                height=height,
                active_gait=active_gait,
                cmd=cmd,
                imu=imu,
                imu_dz=imu_dz,
                lie_down_hold=lie_down_hold,
                pose_hold_name=pose_hold_name,
                joint_direction_test=joint_direction_test,
                hip_abd_test=hip_abd_test,
                leg_pitch_test=leg_pitch_test,
                direction_test_start=direction_test_start,
                direction_test_duration_s=direction_test_duration_s,
                abd_flare_active=abd_flare_active,
            )
            self.next_print = now + self.print_interval_s

        if now >= self.next_health:
            self._print_disabled(lz=lz, evo=evo, incos=incos, board=board)
            self.next_health = now + self.health_interval_s

    def _print_loop_status(self, *, mode: str, height: float, active_gait, cmd,
                           imu, imu_dz, lie_down_hold: bool,
                           joint_direction_test: bool, hip_abd_test: bool,
                           leg_pitch_test: bool, direction_test_start: float,
                           direction_test_duration_s: float,
                           pose_hold_name: str | None = None,
                           abd_flare_active: bool = False) -> None:
        if lie_down_hold:
            if pose_hold_name == "sit":
                sys.stdout.write(
                    "\r  [SIT] 保持坐下姿势  z=起立  p=改趴下  q=退出      ")
            else:
                sys.stdout.write(
                    "\r  [LIE_DOWN] 保持趴下姿势  p=起立  z=改坐下  q=退出      ")
        elif active_gait:
            tag = mode.upper()
            vel = getattr(active_gait, "vel_cmd", None)
            v_str = ""
            if isinstance(vel, (tuple, list)) and len(vel) >= 1:
                v_str = f"  v={float(vel[0]):+.3f}m/s"
            st = float(getattr(active_gait, "stance_ratio", 0.0))
            sh_r = float(getattr(active_gait, "step_height", 0.0))
            sh_f = float(getattr(active_gait, "step_height_front", sh_r) or sh_r)
            # Stick is engage-only; don't display raw depth (misleading).
            drive = "FWD" if cmd.vx > 0.15 else ("BWD" if cmd.vx < -0.15 else "—")
            if abs(getattr(cmd, "turn", 0.0) or 0.0) > 0.12 and drive == "—":
                drive = "TURN"
            sys.stdout.write(
                f"\r  [{tag}]  h={height:.3f}m  T={active_gait.period:.2f}s  "
                f"st={st:.0%}  "
                f"amp=±{abs(active_gait.amp_front)*100:.1f}/"
                f"{abs(active_gait.amp_rear)*100:.1f}cm  "
                f"lift={sh_f*100:.1f}/{sh_r*100:.1f}cm"
                f"{v_str}  drive={drive}      "
            )
        elif joint_direction_test:
            elapsed = self.clock.monotonic() - direction_test_start
            pct = min(100.0, 100.0 * elapsed / max(direction_test_duration_s, 1e-6))
            if hip_abd_test:
                desc = "ID2/6/9/12=主控站姿外展目标"
            elif leg_pitch_test:
                desc = "ID1/5 前腿大腿向前；ID10/13 后腿大腿向后"
            else:
                desc = "ID3/7/11/14 四个小腿向前"
            sys.stdout.write(f"\r  [DIRECTION_TEST] {pct:5.1f}%  {desc}；q=回启动角并失能      ")
        else:
            roll_deg = math.degrees(imu.roll) if (imu and imu.connected) else 0.0
            dz_str = ""
            if imu_dz:
                dz_str = (
                    f" dZ: FL{imu_dz.get('fl',0)*1000:+.1f}"
                    f" FR{imu_dz.get('fr',0)*1000:+.1f}"
                    f" RL{imu_dz.get('rl',0)*1000:+.1f}"
                    f" RR{imu_dz.get('rr',0)*1000:+.1f}mm"
                )
            tag = "STAND+ABD" if abd_flare_active else "STAND"
            hint = " a=收回外展" if abd_flare_active else " a=外展验证"
            sys.stdout.write(
                f"\r  [{tag}]  h={height:.3f}m  roll={roll_deg:+.1f}° "
                f"input_vx={cmd.vx:+.2f}{dz_str}{hint}        "
            )
        sys.stdout.flush()

    def _print_disabled(self, *, lz, evo, incos=None, board=None) -> None:
        disabled = []
        if board is not None:
            feedback = board.get_feedback(j.motor_id for j in self.real_joints)
            for joint in self.real_joints:
                sample = feedback.samples.get(joint.motor_id)
                if sample is None or not sample.enabled:
                    disabled.append(joint)
        else:
            for joint in self.real_joints:
                mid = joint.motor_id
                idx = mid - 1
                if joint.mtype == "lz":
                    enabled = lz.is_enabled[idx]
                elif joint.mtype == "incos":
                    enabled = incos is not None and incos.is_enabled[idx]
                elif joint.mtype == "dm":
                    enabled = mid in self.dm_fixed_targets
                else:
                    enabled = evo.status[idx] == 0x02
                if not enabled:
                    disabled.append(joint)
        if disabled:
            print("\n  [!] disabled: "
                  + ", ".join(f"M{j.motor_id}({j.name})" for j in disabled))


__all__ = ["RuntimeStatusDisplay"]
