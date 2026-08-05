"""End-to-end ownership: Soft+WBC / Soft+COM_SHIFT / Spot — non-owners are 0."""

from __future__ import annotations

import os
import sys
import unittest
from types import SimpleNamespace

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (_ROOT, os.path.join(_ROOT, "src")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from marsdog_control.config.control_policies import (  # noqa: E402
    AttitudeOwner,
    ControlPolicies,
    ForceMode,
    LateralOwner,
)
from marsdog_control.config.schema import FeatureFlags, RuntimeConfig  # noqa: E402
from marsdog_control.motion.attitude_overlay import (  # noqa: E402
    AttitudeOverlayGate,
    bind_ownership,
)
from marsdog_control.motion.gait_controller import (  # noqa: E402
    NaturalSoftTrot,
    NaturalWalk,
    StablePace,
)
from marsdog_control.motion.lateral_planner import LateralPlanner  # noqa: E402


def _bind(gait, *, lateral: LateralOwner, attitude: AttitudeOwner):
    planner = LateralPlanner(session_owner=lateral)
    gate = AttitudeOverlayGate(attitude=attitude)
    bind_ownership(lateral_planner=planner, attitude_gate=gate, gaits=[gait])
    return planner, gate


class SoftComShiftOwnershipE2E(unittest.TestCase):
    def test_com_shift_on_sway_and_force_y_off(self):
        g = NaturalSoftTrot(
            period=1.0, stance_ratio=0.72,
            com_shift_m=0.004, com_shift_blend=0.15,
            lateral_sway=0.008, anti_roll=0.01,
            spine_yaw_deg=3.0, swing_level=0.4,
        )
        planner, gate = _bind(
            g, lateral=LateralOwner.COM_SHIFT, attitude=AttitudeOwner.NONE)
        t = 0.25
        # Owner path: Soft com_shift kinematic non-zero.
        self.assertNotEqual(g._lateral_offset(t), 0.0)
        # Non-owners zero: force_y (session COM_SHIFT), IMU prelevel, …
        self.assertEqual(planner.gate_force_y(0.02), 0.0)
        self.assertEqual(g._gated_swing_level(), 0.0)
        # NONE still allows Soft roll / spine open-loop.
        self.assertAlmostEqual(g._gated_anti_roll(), 0.01)
        self.assertEqual(g._gated_spine_deg()[0], 3.0)
        self.assertFalse(gate.allows_imu_prelevel())


class SoftWbcOwnershipE2E(unittest.TestCase):
    def test_wbc_zeros_soft_attitude_and_keeps_com_shift(self):
        rt = RuntimeConfig(features=FeatureFlags(wbc_enabled=True))
        pol = ControlPolicies.from_runtime(rt, lateral=LateralOwner.COM_SHIFT)
        self.assertIs(pol.force, ForceMode.WBC)
        self.assertIs(pol.attitude, AttitudeOwner.WBC)

        g = NaturalSoftTrot(
            period=1.0, stance_ratio=0.72,
            com_shift_m=0.004, com_shift_blend=0.15,
            lateral_sway=0.008, anti_roll=0.012,
            trot_roll_ff_neg_deg=2.0, spine_yaw_deg=5.0,
            spine_roll_deg=2.4, swing_level=0.5,
        )
        planner, _ = _bind(g, lateral=pol.lateral, attitude=pol.attitude)
        t = 0.30
        self.assertNotEqual(g._lateral_offset(t), 0.0)  # COM_SHIFT still on
        self.assertEqual(planner.gate_force_y(0.02), 0.0)
        self.assertEqual(g._gated_anti_roll(), 0.0)
        self.assertEqual(g._gated_swing_level(), 0.0)
        self.assertEqual(g._gated_spine_deg(), (0.0, 0.0))
        self.assertEqual(g._expected_diagonal_roll(t), 0.0)


class SpotOwnershipE2E(unittest.TestCase):
    def test_spot_zeros_soft_kinematic_allows_spot_com(self):
        g = NaturalSoftTrot(
            period=1.0, stance_ratio=0.72,
            com_shift_m=0.004, lateral_sway=0.008,
        )
        g.spot_turn_active = True
        planner, _ = _bind(
            g, lateral=LateralOwner.COM_SHIFT, attitude=AttitudeOwner.NONE)
        self.assertIs(planner.sync_from_gait(g), LateralOwner.SPOT)
        self.assertEqual(g._lateral_offset(0.25), 0.0)
        self.assertEqual(planner.gate_force_y(0.02), 0.0)
        self.assertEqual(planner.gate_spot_com((0.01, -0.02)), (0.01, -0.02))


class WalkPaceOwnershipE2E(unittest.TestCase):
    def test_walk_com_allows_both_channels_pace_blocks_force_y(self):
        walk = NaturalWalk(period=1.0, stance_ratio=0.74, lateral_sway=0.008)
        pace = StablePace(period=0.8, stance_ratio=0.7, lateral_sway=0.010)
        pw, _ = _bind(
            walk, lateral=LateralOwner.COM_SHIFT, attitude=AttitudeOwner.NONE)
        pp, _ = _bind(
            pace, lateral=LateralOwner.COM_SHIFT, attitude=AttitudeOwner.NONE)

        self.assertIs(pw.sync_from_gait(walk), LateralOwner.WALK_COM)
        self.assertNotEqual(walk._lateral_offset(0.2), 0.0)
        self.assertNotEqual(walk.get_com_y_shift(0.2), 0.0)

        self.assertIs(pp.sync_from_gait(pace), LateralOwner.SWAY)
        self.assertNotEqual(pace._lateral_offset(0.2), 0.0)
        self.assertEqual(pp.gate_force_y(0.02), 0.0)


class AssemblyPoliciesRequiredTest(unittest.TestCase):
    def test_assemble_rejects_missing_policies(self):
        from marsdog_control.config.control_policies import ControlPolicyError
        from marsdog_control.runtime.walk_assembly import assemble_walk_loop_context
        from types import SimpleNamespace

        startup = SimpleNamespace(session=None, runtime_config=None)
        with self.assertRaises(ControlPolicyError):
            assemble_walk_loop_context(
                startup=startup,
                runtime_state=SimpleNamespace(
                    impedance=SimpleNamespace(
                        enabled=False, td_kp_scale=0.4,
                        swing_kp_scale=0.7, td_window=0.15),
                    gravity_comp=False, gravity_scale=0.5,
                ),
                hw=SimpleNamespace(),
                fsm=SimpleNamespace(),
                input_hal=SimpleNamespace(),
                stand=NaturalSoftTrot(),
                safety=SimpleNamespace(),
                imu_ctrl=SimpleNamespace(),
                targets={},
                cur_pos={},
                smooth_tgt={},
                real_joints=[],
                joint_map={},
                direction_test_base={},
                direction_test_start=0.0,
                control_hz=200.0,
                clock=SimpleNamespace(),
                write_log=lambda *_: None,
                log_writer=None,
                bark_with_mouth=lambda: None,
                build_lie_down_target=lambda *_: {},
                read_positions=lambda *_: {},
                smooth_transition=lambda *_: None,
            )


class GaitModuleSplitTest(unittest.TestCase):
    def test_facade_and_split_modules_export_same_classes(self):
        from marsdog_control.motion import gait_base, stable_trot, natural_gait, jump_gait
        from marsdog_control.motion import gait_controller as facade
        self.assertIs(facade.StableTrot, stable_trot.StableTrot)
        self.assertIs(facade.NaturalSoftTrot, natural_gait.NaturalSoftTrot)
        self.assertIs(facade.JumpController, jump_gait.JumpController)
        self.assertIs(facade.StandController, gait_base.StandController)
        self.assertTrue(hasattr(facade.GaitController, "bind_ownership"))


class SessionOwnershipInjectionTest(unittest.TestCase):
    def test_controller_set_bind_ownership_method(self):
        from marsdog_control.motion.gait_recipes import ControllerSet
        g = NaturalSoftTrot(period=1.0, stance_ratio=0.72, com_shift_m=0.004)
        cs = ControllerSet(
            stand=g, fwd=g, bwd=g, pace_fwd=g, pace_bwd=g,
            nat_fwd=g, walk_fwd=g, jump_fwd=g,
        )
        planner = LateralPlanner(session_owner=LateralOwner.COM_SHIFT)
        gate = AttitudeOverlayGate(attitude=AttitudeOwner.NONE)
        cs.bind_ownership(lateral_planner=planner, attitude_gate=gate)
        self.assertIs(g._lateral_planner, planner)
        self.assertIs(g._attitude_overlay_gate, gate)

    def test_startup_context_has_no_natural_params_field(self):
        from marsdog_control.runtime.walk_startup import WalkStartupContext
        fields = getattr(WalkStartupContext, "__dataclass_fields__", {})
        self.assertNotIn("natural_params", fields)
        self.assertNotIn("walk_params", fields)
        self.assertNotIn("jump_params", fields)
        self.assertIn("lateral_planner", fields)
        self.assertIn("attitude_gate", fields)

    def test_trot_tick_module_exists(self):
        from marsdog_control.motion import trot_tick
        self.assertTrue(hasattr(trot_tick, "prepare_stable_trot_tick"))


if __name__ == "__main__":
    unittest.main()
