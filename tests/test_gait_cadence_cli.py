"""CLI cadence flags: --gait-period / --gait-hz must reach SoftTrot schedule."""

from __future__ import annotations

import sys
import unittest
from unittest import mock

from marsdog_control.apps.walk_cli import (
    apply_gait_cadence_cli,
    apply_preset_preserving_cli,
    parse_args,
)
from marsdog_control.config.stack_build import FsmDriveConfig, GaitStackConfig
from marsdog_control.core.types import RobotMode, RobotState
from marsdog_control.motion.gait_recipes import (
    NATURAL_SOFT_TROT_WBC,
    build_controller_set,
)
from marsdog_control.motion.gait_schedule import SoftTrotSchedule, VelocityCommand
from marsdog_control.runtime.fsm import RuntimeStateMachine


def _trace_soft_trot(argv_extra):
    """Parse → preset → stack → nat_fwd → SoftTrotSchedule (no hardware)."""
    argv = ["walk", "--natural-soft-trot", "--no-gamepad", "--no-tail", *argv_extra]
    with mock.patch.object(sys, "argv", argv):
        args = parse_args()
    recipe = dict(NATURAL_SOFT_TROT_WBC)
    apply_preset_preserving_cli(args, recipe)
    cfg = GaitStackConfig.from_args(args)
    controllers = build_controller_set(
        cfg,
        front_x0=0.0,
        rear_x0=0.0,
        natural_params=recipe,
    )
    drive = FsmDriveConfig.from_args(args)
    fsm = RuntimeStateMachine(
        controllers,
        drive,
        height=float(cfg.height),
        fwd_amp_front=float(controllers.nat_fwd.amp_front),
        fwd_amp_rear=float(controllers.nat_fwd.amp_rear),
        natural_configured=True,
        start_mode=RobotMode.STAND,
    )
    return args, recipe, cfg, controllers.nat_fwd, fsm


class TestGaitCadenceCli(unittest.TestCase):
    def test_gait_period_sets_both_and_beats_preset(self):
        with mock.patch.object(sys, "argv", ["walk", "--gait-period", "1.0"]):
            args = parse_args()
        self.assertAlmostEqual(args.period, 1.0)
        self.assertAlmostEqual(args.nat_period, 1.0)
        recipe = dict(NATURAL_SOFT_TROT_WBC)
        apply_preset_preserving_cli(args, recipe)
        self.assertAlmostEqual(args.period, 1.0)
        self.assertAlmostEqual(args.nat_period, 1.0)
        # Critical: recipe dict must sync, or nat_fwd stays on preset 0.87
        self.assertAlmostEqual(recipe["period"], 1.0)
        self.assertAlmostEqual(recipe["nat_period"], 1.0)

    def test_gait_hz_converts_to_period(self):
        with mock.patch.object(sys, "argv", ["walk", "--gait-hz", "2.0"]):
            args = parse_args()
        self.assertAlmostEqual(args.period, 0.5)
        self.assertAlmostEqual(args.nat_period, 0.5)

    def test_apply_rejects_both_flags(self):
        from argparse import Namespace

        args = Namespace(
            gait_period=1.0,
            gait_hz=1.0,
            period=0.75,
            nat_period=0.9,
            _explicit_cli={"gait_period", "gait_hz"},
        )
        with self.assertRaises(SystemExit):
            apply_gait_cadence_cli(args)

    def test_end_to_end_nat_fwd_and_schedule_use_cli_period(self):
        """Regression: recipe dict must not leave SoftTrot stuck at preset period."""
        preset_T = float(NATURAL_SOFT_TROT_WBC["period"])
        self.assertGreater(preset_T, 0.4)

        args, recipe, cfg, nat_fwd, fsm = _trace_soft_trot(["--gait-period", "1.20"])
        self.assertAlmostEqual(args.nat_period, 1.20)
        self.assertAlmostEqual(cfg.nat_period, 1.20)
        self.assertAlmostEqual(cfg.period, 1.20)
        self.assertAlmostEqual(recipe["period"], 1.20)
        self.assertAlmostEqual(float(nat_fwd.period), 1.20)
        self.assertAlmostEqual(fsm._nat_schedule.env.period_nom, 1.20)

        # SI cruise (m/s) must schedule from the CLI envelope, not the preset period.
        state = RobotState(imu_connected=False)
        cruise_mps = SoftTrotSchedule(fsm._nat_schedule.env).vx_at_legacy_norm(0.5)
        fsm._apply_natural_throttle(state, cruise_mps, 0.0)
        env = fsm._nat_schedule.env
        self.assertGreaterEqual(float(nat_fwd.period), env.period_min - 1e-9)
        self.assertLessEqual(float(nat_fwd.period), env.period_max + 1e-9)
        # Distinct from default-preset cruise period
        _, _, _, nat_default, fsm_default = _trace_soft_trot([])
        cruise_default = SoftTrotSchedule(
            fsm_default._nat_schedule.env
        ).vx_at_legacy_norm(0.5)
        fsm_default._apply_natural_throttle(state, cruise_default, 0.0)
        self.assertNotAlmostEqual(float(nat_fwd.period), float(nat_default.period), places=3)

    def test_default_keeps_soft_trot_preset_period(self):
        args, recipe, cfg, nat_fwd, fsm = _trace_soft_trot([])
        preset_T = float(NATURAL_SOFT_TROT_WBC["period"])
        self.assertAlmostEqual(float(nat_fwd.period), preset_T, places=2)
        self.assertAlmostEqual(fsm._nat_schedule.env.period_nom, preset_T, places=2)

    def test_cli_stance_and_period_reach_schedule(self):
        """--gait-period / --stance override SoftTrot without editing recipes."""
        args, recipe, cfg, nat_fwd, fsm = _trace_soft_trot(
            ["--gait-period", "1.2", "--stance", "0.36"]
        )
        self.assertAlmostEqual(args.stance, 0.36)
        self.assertAlmostEqual(recipe["stance"], 0.36)
        self.assertAlmostEqual(float(nat_fwd.stance_ratio), 0.36)
        self.assertAlmostEqual(fsm._nat_schedule.env.stance_nom, 0.36)
        self.assertLessEqual(fsm._nat_schedule.env.stance_min, 0.36)
        self.assertGreaterEqual(fsm._nat_schedule.env.stance_max, 0.36)
        self.assertAlmostEqual(float(nat_fwd.period), 1.2)

    def test_unsynced_recipe_would_ignore_cli(self):
        """Document the old bug: args updated but stale recipe dict wins in builder."""
        with mock.patch.object(sys, "argv", ["walk", "--gait-period", "1.20"]):
            args = parse_args()
        stale = dict(NATURAL_SOFT_TROT_WBC)  # intentionally NOT passed through sync
        # Mimic old apply_preset that only mutated args:
        from marsdog_control.motion.gait_recipes import apply_values
        apply_values(args, stale)  # would overwrite CLI...
        # restore CLI on args only (old behavior)
        args.period = 1.20
        args.nat_period = 1.20
        cfg = GaitStackConfig.from_args(args)
        controllers = build_controller_set(
            cfg, front_x0=0.0, rear_x0=0.0, natural_params=stale,
        )
        # Stale recipe keeps preset period → nat_fwd ignores CLI (the bug we fixed)
        self.assertAlmostEqual(
            float(controllers.nat_fwd.period),
            float(NATURAL_SOFT_TROT_WBC["period"]),
            places=2,
        )
        self.assertAlmostEqual(cfg.nat_period, 1.20)


if __name__ == "__main__":
    unittest.main()
