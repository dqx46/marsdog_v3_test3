"""Control ownership Phase 1 — Soft packages + ControlPolicies mutex."""

from __future__ import annotations

import os
import sys
import unittest
from dataclasses import fields, replace
from types import SimpleNamespace
from unittest import mock

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (_ROOT, os.path.join(_ROOT, "src")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from marsdog_control.config.control_policies import (  # noqa: E402
    AttitudeOwner,
    ControlPolicies,
    ControlPolicyError,
    ForceMode,
    LateralOwner,
    resolve_force_mode,
)
from marsdog_control.config.schema import (  # noqa: E402
    DynamicsConfig,
    FeatureFlags,
    RuntimeConfig,
)
from marsdog_control.config.soft_trot_recipe import (  # noqa: E402
    NON_GEOMETRY_SOFT_FIELDS,
    SOFT_FORBIDDEN_POUR_FIELDS,
    SOFT_TROT_RECIPE,
    SoftAttitudeOverlay,
    SoftBalanceOverlay,
    SoftFootShape,
    SoftGeometry,
    SoftTrotRecipe,
)
from marsdog_control.motion.gait_recipes import NATURAL_SOFT_TROT  # noqa: E402


class SoftPackageOwnershipTest(unittest.TestCase):
    def test_soft_recipe_has_no_forbidden_pour_fields(self):
        names = {f.name for f in fields(SoftTrotRecipe)}
        leaked = names & SOFT_FORBIDDEN_POUR_FIELDS
        self.assertFalse(leaked, msg=f"Soft still owns: {sorted(leaked)}")

    def test_soft_to_dict_excludes_imu_wbc_keys(self):
        d = SOFT_TROT_RECIPE.to_dict()
        for key in (
            "kp_base_roll", "imu_kp", "com_y_shift_m",
            "kd_base_roll", "lateral_vel_damp", "swing_foot_kp",
        ):
            self.assertNotIn(key, d)
        for key in SOFT_FORBIDDEN_POUR_FIELDS:
            self.assertNotIn(key, NATURAL_SOFT_TROT)

    def test_soft_packages_roundtrip(self):
        from marsdog_control.config.soft_trot_recipe import (
            SoftAttitudeOverlay, SoftImpedanceAssist,
        )
        r = SoftTrotRecipe.from_parts(
            SoftGeometry(), SoftFootShape(), SoftBalanceOverlay(),
            SoftAttitudeOverlay(), SoftImpedanceAssist())
        self.assertEqual(r.to_dict(), SOFT_TROT_RECIPE.to_dict())
        self.assertEqual(r.as_geometry(), SoftGeometry())
        self.assertEqual(r.as_balance().lateral_sway, 0.0)
        self.assertAlmostEqual(r.as_balance().com_shift_m, 0.004)
        self.assertNotIn("anti_roll", SoftBalanceOverlay.__dataclass_fields__)
        self.assertEqual(r.as_attitude().anti_roll, 0.0)
        self.assertNotIn("swing_level", SoftFootShape.__dataclass_fields__)
        self.assertNotIn("spine_yaw_deg", SoftFootShape.__dataclass_fields__)
        self.assertIn("swing_level", SoftAttitudeOverlay.__dataclass_fields__)
        self.assertIn("spine_yaw_deg", SoftAttitudeOverlay.__dataclass_fields__)

    def test_leg_kp_scale_marked_non_geometry(self):
        from marsdog_control.config.soft_trot_recipe import SoftImpedanceAssist
        self.assertIn("leg_kp_scale", NON_GEOMETRY_SOFT_FIELDS)
        self.assertNotIn("leg_kp_scale", SoftGeometry.__dataclass_fields__)
        self.assertIn("leg_kp_scale", SoftImpedanceAssist.__dataclass_fields__)


class ControlPoliciesTest(unittest.TestCase):
    def test_wbc_implies_attitude_wbc_not_imu(self):
        rt = RuntimeConfig(
            features=FeatureFlags(wbc_enabled=True, vmc_enabled=False),
        )
        pol = ControlPolicies.from_runtime(rt, lateral=LateralOwner.COM_SHIFT)
        self.assertIs(pol.attitude, AttitudeOwner.WBC)
        self.assertIs(pol.force, ForceMode.WBC)
        self.assertFalse(pol.apply_imu_foot_balance)

    def test_imu_features_without_wbc(self):
        rt = RuntimeConfig(
            features=FeatureFlags(
                wbc_enabled=False, vmc_enabled=False,
                imu_enabled=True, imu_feedback_enabled=True,
            ),
        )
        pol = ControlPolicies.from_runtime(rt, lateral=LateralOwner.NONE)
        self.assertIs(pol.attitude, AttitudeOwner.IMU)
        self.assertIs(pol.force, ForceMode.IMPEDANCE)
        self.assertTrue(pol.apply_imu_foot_balance)

    def test_wbc_plus_vmc_fatal(self):
        rt = RuntimeConfig(
            features=FeatureFlags(wbc_enabled=True, vmc_enabled=True),
        )
        with self.assertRaises(ControlPolicyError):
            ControlPolicies.from_runtime(rt)

    def test_force_mode_resolve_mutex(self):
        self.assertIs(resolve_force_mode(True, False), ForceMode.WBC)
        self.assertIs(resolve_force_mode(False, True), ForceMode.VMC)
        self.assertIs(resolve_force_mode(False, False), ForceMode.IMPEDANCE)
        with self.assertRaises(ControlPolicyError):
            resolve_force_mode(True, True)

    def test_soft_default_lateral_com_shift(self):
        lat = ControlPolicies.soft_default_lateral(
            com_shift_m=0.004, lateral_sway=0.0)
        self.assertIs(lat, LateralOwner.COM_SHIFT)
        with self.assertRaises(ControlPolicyError):
            ControlPolicies.soft_default_lateral(
                com_shift_m=0.004, lateral_sway=0.004)

    def test_soft_default_balance_matches_owner(self):
        bal = SOFT_TROT_RECIPE.as_balance()
        self.assertAlmostEqual(bal.com_shift_m, 0.004)
        self.assertEqual(bal.lateral_sway, 0.0)
        self.assertEqual(RuntimeConfig().dynamics.com_y_shift_m, 0.0)


class WalkLoopOwnerGateTest(unittest.TestCase):
    def test_wbc_session_zeros_imu_dz_via_policies(self):
        """Light mock: policies.attitude=WBC must wipe imu_dz path."""
        from marsdog_control.config.control_policies import ControlPolicies

        pol = ControlPolicies(
            attitude=AttitudeOwner.WBC,
            lateral=LateralOwner.COM_SHIFT,
            force=ForceMode.WBC,
        )
        self.assertFalse(pol.apply_imu_foot_balance)

        # Mirror walk_loop gate without full sim.
        imu_dz = {"fl": 0.01, "fr": -0.01, "rl": 0.02, "rr": -0.02}
        if not pol.apply_imu_foot_balance:
            imu_dz = {"fl": 0.0, "fr": 0.0, "rl": 0.0, "rr": 0.0}
        self.assertEqual(imu_dz, {"fl": 0.0, "fr": 0.0, "rl": 0.0, "rr": 0.0})

    def test_executor_force_mode_matches_flags(self):
        from marsdog_control.control.executor import ExecutorConfig

        cfg = ExecutorConfig(wbc_enabled=True, vmc_enabled=False)
        self.assertIs(cfg.force_mode, ForceMode.WBC)
        cfg2 = ExecutorConfig.from_runtime_config(
            RuntimeConfig(features=FeatureFlags(wbc_enabled=True)))
        self.assertIs(cfg2.force_mode, ForceMode.WBC)
        self.assertTrue(cfg2.wbc_enabled)
        with self.assertRaises(ValueError):
            ExecutorConfig(wbc_enabled=True, vmc_enabled=True)



class LateralPlannerGateTest(unittest.TestCase):
    def test_com_shift_owner_zeros_sway_and_force_y(self):
        from marsdog_control.motion.lateral_planner import LateralPlanner
        p = LateralPlanner(session_owner=LateralOwner.COM_SHIFT)
        self.assertAlmostEqual(
            p.soft_kinematic(
                0.25, 1.0, 0.72,
                com_shift_m=0.004, com_shift_blend=0.15, lateral_sway=0.01),
            -0.004)
        # COM_SHIFT with zero amp must NOT fall back to sway.
        self.assertEqual(
            p.soft_kinematic(
                0.25, 1.0, 0.72,
                com_shift_m=0.0, com_shift_blend=0.15, lateral_sway=0.01),
            0.0)
        self.assertEqual(p.trot_sway_kinematic(0.25, 1.0, 0.72, 0.01), 0.0)
        self.assertEqual(p.gate_force_y(0.012), 0.0)
        self.assertEqual(p.gate_spot_com((0.01, 0.02)), (0.0, 0.0))

    def test_spot_elevates_owner(self):
        from marsdog_control.motion.lateral_planner import LateralPlanner
        p = LateralPlanner(session_owner=LateralOwner.COM_SHIFT)
        gait = SimpleNamespace(spot_turn_active=True, family="trot")
        self.assertIs(p.sync_from_gait(gait), LateralOwner.SPOT)
        self.assertEqual(p.soft_kinematic(
            0.25, 1.0, 0.72,
            com_shift_m=0.004, com_shift_blend=0.15, lateral_sway=0.0), 0.0)
        self.assertEqual(p.gate_spot_com((0.01, -0.02)), (0.01, -0.02))
        gait.spot_turn_active = False
        self.assertIs(p.sync_from_gait(gait), LateralOwner.COM_SHIFT)

    def test_walk_family_overrides_to_walk_com(self):
        from marsdog_control.motion.lateral_planner import LateralPlanner
        p = LateralPlanner(session_owner=LateralOwner.COM_SHIFT)
        gait = SimpleNamespace(spot_turn_active=False, family="walk")
        self.assertIs(p.sync_from_gait(gait), LateralOwner.WALK_COM)
        self.assertTrue(p.allows_sway_kinematic())
        self.assertTrue(p.allows_force_y())
        self.assertAlmostEqual(p.gate_force_y(0.01), 0.01)

    def test_pace_sway_blocks_force_y(self):
        from marsdog_control.motion.lateral_planner import LateralPlanner
        p = LateralPlanner(session_owner=LateralOwner.COM_SHIFT)
        gait = SimpleNamespace(spot_turn_active=False, family="pace")
        self.assertIs(p.sync_from_gait(gait), LateralOwner.SWAY)
        self.assertTrue(p.allows_sway_kinematic())
        self.assertFalse(p.allows_force_y())
        self.assertEqual(p.gate_force_y(0.01), 0.0)

    def test_soft_controller_respects_planner(self):
        from marsdog_control.motion.gait_controller import NaturalSoftTrot
        from marsdog_control.motion.lateral_planner import LateralPlanner
        from marsdog_control.motion.foot_trajectory import (
            lateral_offset_soft_trot_com, lateral_offset_trot,
        )
        g = NaturalSoftTrot(
            period=1.0, stance_ratio=0.56,
            lateral_sway=0.008, com_shift_m=0.004, com_shift_blend=0.15,
        )
        p = LateralPlanner(session_owner=LateralOwner.COM_SHIFT)
        p.attach_to(g)
        t = 0.25
        self.assertAlmostEqual(
            g._lateral_offset(t),
            lateral_offset_soft_trot_com(t, 1.0, 0.004, 0.15))
        # Force SWAY owner → sway path only.
        p.session_owner = LateralOwner.SWAY
        self.assertAlmostEqual(
            g._lateral_offset(t),
            lateral_offset_trot(t, 1.0, 0.56, 0.008))


class AttitudeOverlayGateTest(unittest.TestCase):
    def test_wbc_zeros_anti_roll_roll_ff_and_spine(self):
        from marsdog_control.motion.attitude_overlay import AttitudeOverlayGate
        from marsdog_control.motion.gait_controller import NaturalSoftTrot
        gate = AttitudeOverlayGate(attitude=AttitudeOwner.WBC)
        self.assertFalse(gate.allows_kinematic_roll_patch())
        self.assertFalse(gate.allows_spine())
        self.assertFalse(gate.allows_imu_prelevel())
        g = NaturalSoftTrot(
            anti_roll=0.010, trot_roll_ff_neg_deg=2.0,
            trot_roll_ff_pos_deg=1.0, swing_level=0.5,
            spine_yaw_deg=5.0, spine_roll_deg=2.0,
        )
        gate.attach_to(g)
        self.assertEqual(g._gated_anti_roll(), 0.0)
        self.assertEqual(g._gated_swing_level(), 0.0)
        self.assertEqual(g._expected_diagonal_roll(0.25), 0.0)
        self.assertEqual(g._gated_spine_deg(), (0.0, 0.0))

    def test_none_allows_anti_roll_not_imu_prelevel(self):
        from marsdog_control.motion.attitude_overlay import AttitudeOverlayGate
        from marsdog_control.motion.gait_controller import StableTrot
        gate = AttitudeOverlayGate(attitude=AttitudeOwner.NONE)
        self.assertTrue(gate.allows_kinematic_roll_patch())
        self.assertFalse(gate.allows_imu_prelevel())
        g = StableTrot(anti_roll=0.008, swing_level=0.4)
        gate.attach_to(g)
        self.assertAlmostEqual(g._gated_anti_roll(), 0.008)
        self.assertEqual(g._gated_swing_level(), 0.0)

    def test_missing_gate_fail_closed(self):
        from marsdog_control.motion.gait_controller import NaturalSoftTrot
        g = NaturalSoftTrot(
            anti_roll=0.010, swing_level=0.5,
            spine_yaw_deg=3.0, lateral_sway=0.008, com_shift_m=0.004,
        )
        self.assertEqual(g._gated_anti_roll(), 0.0)
        self.assertEqual(g._gated_swing_level(), 0.0)
        self.assertEqual(g._gated_spine_deg(), (0.0, 0.0))
        self.assertEqual(g._lateral_offset(0.25), 0.0)

    def test_wbc_policies_forbid_soft_attitude_overlay(self):
        rt = RuntimeConfig(features=FeatureFlags(wbc_enabled=True))
        pol = ControlPolicies.from_runtime(rt, lateral=LateralOwner.COM_SHIFT)
        self.assertFalse(pol.apply_soft_attitude_overlay)
        self.assertFalse(pol.apply_imu_foot_balance)


class WbcNoGravityBlendTest(unittest.TestCase):
    def test_wbc_apply_source_has_no_gravity_trq_blend(self):
        import inspect
        from marsdog_control.control import executor_wbc_apply as mod
        src = inspect.getsource(mod.ExecutorWbcApplyMixin)
        self.assertNotIn("gravity_trq", src)


class ImpedanceAssistOwnershipTest(unittest.TestCase):
    def test_wbc_still_carries_impedance_assist_scale(self):
        rt = RuntimeConfig(features=FeatureFlags(wbc_enabled=True))
        pol = ControlPolicies.from_runtime(rt, lateral=LateralOwner.COM_SHIFT)
        self.assertIs(pol.force, ForceMode.WBC)
        self.assertTrue(pol.impedance.enabled)
        self.assertAlmostEqual(pol.impedance.effective_leg_kp_scale(), 0.90)

    def test_assist_disabled_forces_unity(self):
        from marsdog_control.config.control_policies import ImpedanceAssist
        a = ImpedanceAssist(enabled=False, leg_kp_scale=0.5)
        self.assertEqual(a.effective_leg_kp_scale(), 1.0)

if __name__ == "__main__":
    unittest.main()
