"""Jump gait controller."""
import math
from enum import Enum

from marsdog_control.config.joints import JOINT_BY_NAME
from marsdog_control.motion.kinematics import (
    compute_standing_pose_3link,
    front_standing_foot_pitch,
    ik_front_3link_foot_orient,
    ik_rear_leg_2d,
)
from marsdog_control.motion.gait_base import (
    StandController,
    _FRONT_X0,
    _REAR_X0,
    _clamp,
    _cmd,
)

class JumpPhase(str, Enum):
    IDLE = "idle"
    CROUCH = "crouch"
    PUSH = "push"
    FLIGHT = "flight"
    LAND = "land"
    RECOVER = "recover"


class JumpController(StandController):
    """原地四足同步 hop — 与 SoftTrot/Walk/Spot 解耦。

    IDLE → CROUCH → PUSH → FLIGHT → LAND → RECOVER → IDLE
    family=jump；永不 spot_turn。
    """

    family = "jump"
    # All legs sync; ContactSchedule uses phase<=stance as "scheduled stance".
    _PHASE_OFFSET = {"fl": 0.0, "fr": 0.0, "rl": 0.0, "rr": 0.0}

    def __init__(
        self,
        body_height: float = 0.24,
        x_offset_front: float = None,
        x_offset_rear: float = None,
        hip_abduction: float = 0.05,
        front_stand_tarsus_deg: float = 0.0,
        front_stand_foot_pitch_deg: float = None,
        crouch_depth: float = 0.045,
        crouch_s: float = 0.28,
        push_s: float = 0.12,
        flight_s: float = 0.18,
        land_s: float = 0.22,
        recover_s: float = 0.25,
        flight_clearance: float = 0.025,
        land_compress: float = 0.012,
        push_vz: float = 0.55,
        push_extend: float = 0.020,
        # Jump-only base-Z PD (do NOT write into global DynamicsConfig / Soft args).
        kp_base_z: float = 80.0,
        kd_base_z: float = 10.0,
        **_ignored,
    ):
        super().__init__(
            body_height=body_height,
            x_offset_front=x_offset_front,
            x_offset_rear=x_offset_rear,
            hip_abduction=hip_abduction,
            front_stand_tarsus_deg=front_stand_tarsus_deg,
            front_stand_foot_pitch_deg=front_stand_foot_pitch_deg,
        )
        self.family = "jump"
        self.spot_turn_active = False
        self.stand_height = float(body_height)
        self.crouch_depth = float(crouch_depth)
        self.crouch_s = float(crouch_s)
        self.push_s = float(push_s)
        self.flight_s = float(flight_s)
        self.land_s = float(land_s)
        self.recover_s = float(recover_s)
        self.flight_clearance = float(flight_clearance)
        self.land_compress = float(land_compress)
        self.push_vz = float(push_vz)
        self.push_extend = float(push_extend)
        self.kp_base_z = float(kp_base_z)
        self.kd_base_z = float(kd_base_z)

        self.phase = JumpPhase.IDLE
        self._phase_t0 = 0.0
        self.trigger = False
        self.auto_rejump = False
        self.vel_cmd = (0.0, 0.0, 0.0)
        self.speed_frac = 0.0
        self.stance_ratio = 1.0
        self._meas_vz = 0.0
        self._meas_z = 0.0
        self.period = (
            self.crouch_s + self.push_s + self.flight_s
            + self.land_s + self.recover_s
        )
        self.amp_front = 0.0
        self.amp_rear = 0.0
        self.turn_cmd = 0.0
        self.turn_y_gain = 0.0
        self._PHASE_OFFSET = dict(type(self)._PHASE_OFFSET)
        self._height_cmd = self.stand_height
        self._reactive_filtered = 0.0

    def set_height(self, h: float):
        self.stand_height = float(h)
        if self.phase is JumpPhase.IDLE:
            self.body_height = self.stand_height
            self._height_cmd = self.stand_height
            self._update_cache()

    def set_period(self, p: float):
        # Jump timing is phase-duration based; ignore SoftTrot period broadcast.
        return

    def request_jump(self, enable: bool = True):
        self.trigger = bool(enable)

    def _dur(self, phase: JumpPhase) -> float:
        return {
            JumpPhase.IDLE: 0.0,
            JumpPhase.CROUCH: self.crouch_s,
            JumpPhase.PUSH: self.push_s,
            JumpPhase.FLIGHT: self.flight_s,
            JumpPhase.LAND: self.land_s,
            JumpPhase.RECOVER: self.recover_s,
        }[phase]

    def _enter(self, phase: JumpPhase, t: float):
        self.phase = phase
        self._phase_t0 = float(t)

    def _phase_u(self, t: float) -> float:
        dur = self._dur(self.phase)
        if dur <= 1e-9:
            return 1.0
        return max(0.0, min(1.0, (float(t) - self._phase_t0) / dur))

    def _advance(self, t: float):
        t = float(t)
        u = self._phase_u(t)
        if self.phase is JumpPhase.IDLE:
            if self.trigger or self.auto_rejump:
                self._enter(JumpPhase.CROUCH, t)
                self.trigger = False
        elif self.phase is JumpPhase.CROUCH:
            if u >= 1.0:
                self._enter(JumpPhase.PUSH, t)
        elif self.phase is JumpPhase.PUSH:
            # Leave at first vz peak (~0.85); don't keep PUSH into stilts dig.
            liftoff_vz = max(0.84, 0.38 * max(0.3, self.push_vz))
            past_peak = (
                u >= 0.34
                and self._meas_vz >= 0.60
                and self._meas_vz < getattr(self, "_prev_vz", self._meas_vz) - 0.010
            )
            if u >= 1.0 or (u >= 0.34 and self._meas_vz >= liftoff_vz) or past_peak:
                self._enter(JumpPhase.FLIGHT, t)
        elif self.phase is JumpPhase.FLIGHT:
            # Touch down when descending — don't wait out full flight_s with
            # folded legs near the ground (soft-contact second smash).
            if u >= 1.0 or (u >= 0.25 and self._meas_vz < 0.0):
                self._enter(JumpPhase.LAND, t)
        elif self.phase is JumpPhase.LAND:
            if u >= 1.0:
                self._enter(JumpPhase.RECOVER, t)
        elif self.phase is JumpPhase.RECOVER:
            if u >= 1.0:
                self._enter(JumpPhase.IDLE, t)
                if self.auto_rejump:
                    self.trigger = True

    def _smooth(self, u: float) -> float:
        u = max(0.0, min(1.0, u))
        return u * u * (3.0 - 2.0 * u)

    def _height_for_phase(self, t: float) -> float:
        u = self._smooth(self._phase_u(t))
        h0 = self.stand_height
        hc = max(0.14, h0 - self.crouch_depth)
        hp = min(h0 + self.push_extend, hc + self.crouch_depth + self.push_extend)
        if self.phase is JumpPhase.IDLE:
            return h0
        if self.phase is JumpPhase.CROUCH:
            return h0 + (hc - h0) * u
        if self.phase is JumpPhase.PUSH:
            return hc + (hp - hc) * u
        if self.phase is JumpPhase.FLIGHT:
            # Hold push-extend height while feet retract via clearance in IK frame.
            return hp
        if self.phase is JumpPhase.LAND:
            # Soft land crouch — never slam from push-extend / full stand.
            hl = max(0.14, h0 - self.land_compress)
            return hl
        if self.phase is JumpPhase.RECOVER:
            hl = max(0.14, h0 - self.land_compress)
            # Slow ease to stand; stay slightly short early to avoid dig-smash.
            return hl + (h0 - hl) * (u * u)
        return h0

    def in_flight(self) -> bool:
        return self.phase is JumpPhase.FLIGHT

    def note_base_vz(self, vz: float) -> None:
        self._prev_vz = float(getattr(self, "_meas_vz", 0.0))
        self._meas_vz = float(vz)

    def note_base_z(self, z: float) -> None:
        self._meas_z = float(z)

    def jump_force_scale_at(self, t: float) -> float:
        if self.phase is JumpPhase.FLIGHT:
            return 0.0
        if self.phase is JumpPhase.PUSH:
            return 1.0
        if self.phase is JumpPhase.LAND:
            u = self._phase_u(t)
            # Soft touchdown — high force here is the "second smash".
            return 0.12 + 0.28 * self._smooth(u)
        if self.phase is JumpPhase.RECOVER:
            u = self._phase_u(t)
            return 0.25 + 0.30 * self._smooth(u)
        return 1.0

    def predict_jump_force_scale(self, t_future: float) -> float:
        t = t_future
        phase = self.phase
        t0 = self._phase_t0
        
        while True:
            dur = self._dur(phase)
            if t < t0 + dur or dur <= 1e-9:
                break
            t0 += dur
            if phase is JumpPhase.IDLE:
                if self.trigger or self.auto_rejump:
                    phase = JumpPhase.CROUCH
                else:
                    break
            elif phase is JumpPhase.CROUCH: phase = JumpPhase.PUSH
            elif phase is JumpPhase.PUSH: phase = JumpPhase.FLIGHT
            elif phase is JumpPhase.FLIGHT: phase = JumpPhase.LAND
            elif phase is JumpPhase.LAND: phase = JumpPhase.RECOVER
            elif phase is JumpPhase.RECOVER: 
                phase = JumpPhase.IDLE
                break
                
        if phase is JumpPhase.FLIGHT:
            return 0.0
        if phase is JumpPhase.PUSH:
            return 1.0
        if phase is JumpPhase.LAND:
            dur = self._dur(phase)
            u = max(0.0, min(1.0, (t - t0) / dur)) if dur > 1e-9 else 1.0
            return 0.12 + 0.28 * self._smooth(u)
        if phase is JumpPhase.RECOVER:
            dur = self._dur(phase)
            u = max(0.0, min(1.0, (t - t0) / dur)) if dur > 1e-9 else 1.0
            return 0.25 + 0.30 * self._smooth(u)
        return 1.0

    def desired_vz(self, t: float = None) -> float:
        if self.phase is JumpPhase.PUSH:
            u = self._phase_u(t if t is not None else self._phase_t0)
            # Strong lead — lagging cmd brakes a rising hop.
            return self.push_vz * min(1.0, 0.45 + 0.55 * self._smooth(u))
        if self.phase is JumpPhase.FLIGHT:
            return 0.0
        if self.phase is JumpPhase.LAND:
            return -0.15
        return 0.0

    def get_target_z(self, t: float = None) -> float:
        return float(self._height_cmd)

    def get_targets(self, t: float, imu_dz: dict = None,
                    imu_state: dict = None) -> dict:
        self.spot_turn_active = False
        self._advance(t)
        # ContactSchedule: stance_ratio=0 → all swing in FLIGHT; else all stance.
        self.stance_ratio = 0.0 if self.phase is JumpPhase.FLIGHT else 1.0
        h = self._height_for_phase(t)
        self._height_cmd = h
        # body_height = hip-to-foot; lower IK height folds legs for flight clearance.
        if self.phase is JumpPhase.FLIGHT:
            u = self._phase_u(t)
            # Fast fold, but not so hard PD yanks the body back down.
            early = 0.85 + 0.15 * self._smooth(min(1.0, u / 0.12))
            late = self._smooth(max(0.0, (u - 0.12) / 0.88))
            retract = max(early, late)
            h_ik = max(0.14, h - self.flight_clearance * retract)
            if self._meas_z > 0.12:
                h_ik = min(h_ik, max(0.14, self._meas_z - 0.018))
        elif self.phase is JumpPhase.PUSH:
            # Extend for launch; flight phase does the retract (peel-in-push
            # cut impulse and killed air height).
            h_ik = h
            if self._meas_z > 0.12:
                h_ik = min(h_ik, max(0.14, self._meas_z + 0.024))
        elif self.phase is JumpPhase.CROUCH:
            h_ik = h
            if self._meas_z > 0.12:
                h_ik = min(h_ik, max(0.14, self._meas_z + 0.010))
        elif self.phase in (JumpPhase.LAND, JumpPhase.RECOVER):
            h_ik = h
            if self._meas_z > 0.12:
                # Track body height so falling doesn't bury feet (soft-contact bounce).
                h_ik = min(h_ik, max(0.14, self._meas_z - 0.002))
        else:
            h_ik = h
        if abs(h_ik - self.body_height) > 1e-5:
            self.body_height = h_ik
            self._update_cache()
        targets = super().get_targets(t)
        self.body_height = h
        return targets

    def describe(self) -> str:
        return (
            f"JUMP[{self.phase.value}]  h0={self.stand_height:.3f}m  "
            f"crouch={self.crouch_depth*1000:.0f}mm  "
            f"push_vz={self.push_vz:.2f}m/s"
        )
