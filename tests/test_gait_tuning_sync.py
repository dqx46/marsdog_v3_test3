"""SoftTrot shape keys stay aligned between preset and GaitCliDefaults / schema."""

import os
import sys
import unittest

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (_ROOT, os.path.join(_ROOT, "src")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from marsdog_control.config.gait_tuning import GAIT, soft_trot_shape_keys  # noqa: E402
from marsdog_control.config.schema import RuntimeConfig  # noqa: E402
from marsdog_control.config.soft_trot_recipe import (  # noqa: E402
    SCHEMA_GEOMETRY_FROM_RECIPE,
    SOFT_TROT_RECIPE,
)
from marsdog_control.motion.gait_recipes import NATURAL_SOFT_TROT  # noqa: E402


class SoftTrotShapeSyncTest(unittest.TestCase):
    def test_gait_cli_defaults_match_soft_preset_for_shared_keys(self):
        keys = soft_trot_shape_keys()
        mismatches = []
        for key in sorted(keys):
            if key not in NATURAL_SOFT_TROT:
                mismatches.append(f"{key}: missing from NATURAL_SOFT_TROT")
                continue
            expected = NATURAL_SOFT_TROT[key]
            actual = getattr(GAIT, key)
            if actual != expected:
                mismatches.append(
                    f"{key}: GAIT={actual!r} vs NATURAL_SOFT_TROT={expected!r}")
        self.assertFalse(
            mismatches,
            msg="GaitCliDefaults 与 SoftTrot 预设漂移，请只改 SoftTrotRecipe "
                "后派生 GAIT：\n" + "\n".join(mismatches))

    def test_natural_soft_dict_matches_recipe(self):
        recipe_dict = SOFT_TROT_RECIPE.to_dict()
        self.assertEqual(NATURAL_SOFT_TROT, recipe_dict)

    def test_schema_geometry_matches_recipe(self):
        cfg = RuntimeConfig()
        mismatches = []
        for recipe_attr, section, field_name in SCHEMA_GEOMETRY_FROM_RECIPE:
            expected = getattr(SOFT_TROT_RECIPE, recipe_attr)
            actual = getattr(getattr(cfg, section), field_name)
            if actual != expected:
                mismatches.append(
                    f"{section}.{field_name}: schema={actual!r} "
                    f"vs SoftTrotRecipe.{recipe_attr}={expected!r}")
        self.assertFalse(
            mismatches,
            msg="schema 核心几何未从 SoftTrotRecipe 派生：\n"
                + "\n".join(mismatches))

    def test_soft_trot_aliases_are_same_object(self):
        from marsdog_control.motion.gait_recipes import (
            NATURAL_SOFT_TROT_REAL,
            NATURAL_SOFT_TROT_WBC,
        )
        self.assertIs(NATURAL_SOFT_TROT, NATURAL_SOFT_TROT_WBC)
        self.assertIs(NATURAL_SOFT_TROT, NATURAL_SOFT_TROT_REAL)

    def test_nat_cli_aliases_match_canonical(self):
        r = SOFT_TROT_RECIPE
        self.assertEqual(r.nat_amp_front, r.amp_front)
        self.assertEqual(r.nat_amp_rear, r.amp_rear)
        self.assertEqual(r.nat_period, r.period)
        self.assertEqual(r.nat_step_h, r.step_h)
        d = r.to_dict()
        self.assertEqual(d["nat_amp_front"], d["amp_front"])
        self.assertEqual(d["nat_period"], d["period"])

    def test_walk_jump_dicts_match_typed_recipes(self):
        from marsdog_control.config.jump_recipe import JUMP_RECIPE, JUMP_RECIPE_WBC
        from marsdog_control.config.walk_recipe import WALK_RECIPE, WALK_RECIPE_WBC
        from marsdog_control.motion.gait_recipes import (
            JUMP_REAL, JUMP_WBC, NATURAL_WALK_REAL, NATURAL_WALK_WBC,
        )
        self.assertEqual(NATURAL_WALK_REAL, WALK_RECIPE.to_dict())
        self.assertEqual(NATURAL_WALK_WBC, WALK_RECIPE_WBC.to_dict())
        self.assertEqual(JUMP_REAL, JUMP_RECIPE.to_dict())
        self.assertEqual(JUMP_WBC, JUMP_RECIPE_WBC.to_dict())

    def test_gait_params_defaults_match_soft_recipe(self):
        from marsdog_control.motion.gait_params import GaitParams, SoftExtras
        p = GaitParams()
        self.assertEqual(p.body_height, SOFT_TROT_RECIPE.height)
        self.assertEqual(p.amp_front, SOFT_TROT_RECIPE.amp_front)
        self.assertEqual(p.amp_rear, SOFT_TROT_RECIPE.amp_rear)
        self.assertEqual(p.period, SOFT_TROT_RECIPE.period)
        self.assertEqual(p.stance_ratio, SOFT_TROT_RECIPE.stance)
        self.assertEqual(p.step_height, SOFT_TROT_RECIPE.step_h)
        s = SoftExtras()
        self.assertEqual(s.com_shift_m, SOFT_TROT_RECIPE.com_shift_m)
        self.assertEqual(s.touchdown_compress, SOFT_TROT_RECIPE.touchdown_compress)

    def test_soft_trot_from_build_roundtrip(self):
        from marsdog_control.motion.gait_controller import NaturalSoftTrot
        from marsdog_control.motion.gait_params import (
            GaitParams, NaturalExtras, SoftExtras, SoftTrotBuild,
        )
        build = SoftTrotBuild(
            GaitParams(body_height=0.25, amp_front=0.022, amp_rear=0.030),
            NaturalExtras(spine_yaw_deg=0.0),
            SoftExtras(com_shift_m=0.004),
        )
        ctrl = NaturalSoftTrot.from_build(build)
        self.assertAlmostEqual(ctrl.com_shift_m, 0.004)
        self.assertAlmostEqual(ctrl.amp_front, 0.022)
        self.assertAlmostEqual(ctrl.body_height, 0.25)

    def test_soft_packages_cover_recipe_fields(self):
        from marsdog_control.config.soft_trot_recipe import (
            ATTITUDE_OVERLAY_FIELDS,
            BALANCE_OVERLAY_FIELDS,
            FOOT_SHAPE_FIELDS,
            GEOMETRY_FIELDS,
            IMPEDANCE_ASSIST_FIELDS,
            SoftTrotRecipe,
        )
        from dataclasses import fields
        recipe_fields = {f.name for f in fields(SoftTrotRecipe)}
        packaged = (
            GEOMETRY_FIELDS | FOOT_SHAPE_FIELDS
            | BALANCE_OVERLAY_FIELDS | ATTITUDE_OVERLAY_FIELDS
            | IMPEDANCE_ASSIST_FIELDS
        )
        self.assertEqual(recipe_fields, packaged)

    def test_soft_balance_is_lateral_only(self):
        from marsdog_control.config.soft_trot_recipe import (
            SoftAttitudeOverlay,
            SoftBalanceOverlay,
            SoftImpedanceAssist,
            SoftGeometry,
        )
        bal = SoftBalanceOverlay()
        self.assertEqual(
            set(bal.__dataclass_fields__),
            {"com_shift_m", "com_shift_blend", "lateral_sway"},
        )
        att = SoftAttitudeOverlay()
        self.assertIn("anti_roll", att.__dataclass_fields__)
        self.assertNotIn("com_shift_m", att.__dataclass_fields__)
        self.assertNotIn("leg_kp_scale", SoftGeometry.__dataclass_fields__)
        self.assertEqual(SoftImpedanceAssist().leg_kp_scale, 0.90)

    def test_soft_control_pour_empty(self):
        self.assertEqual(SOFT_TROT_RECIPE.control_pour_dict(), {})
        from marsdog_control.config.soft_trot_recipe import CONTROL_POUR_FIELDS
        self.assertEqual(CONTROL_POUR_FIELDS, frozenset())

    def test_instance_flags_not_module_globals(self):
        import marsdog_control.motion.gait_controller as gc
        from marsdog_control.motion.gait_controller import NaturalSoftTrot
        from marsdog_control.motion.gait_params import GaitParams, SoftExtras, SoftTrotBuild
        self.assertFalse(hasattr(gc, "ABD_LEGACY"))
        self.assertFalse(hasattr(gc, "SWING_LEVEL"))
        self.assertFalse(hasattr(gc, "SMOOTH_GAIT"))
        ctrl = NaturalSoftTrot.from_build(SoftTrotBuild(
            GaitParams(swing_level=0.3, smooth_gait=True),
            soft=SoftExtras(com_shift_m=0.0),
        ))
        self.assertAlmostEqual(ctrl.swing_level, 0.3)
        self.assertTrue(ctrl.smooth_gait)


if __name__ == "__main__":
    unittest.main()
