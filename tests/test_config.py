"""Tests for the typed configuration layer.

These are pure-Python tests that do not require any robot hardware, so they run
on Windows/Linux/CI alike.
"""

import dataclasses
import os
import sys
import unittest
from argparse import Namespace

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (_ROOT, os.path.join(_ROOT, "src")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from marsdog_control.config import (  # noqa: E402
    RuntimeConfig,
    apply_runtime_config_to_legacy_args,
    bootstrap_runtime_config,
    runtime_config_from_args,
    runtime_config_summary,
    validate_runtime_config,
)
from marsdog_control.core.units import deg_to_rad, mm_to_m, ms_to_s  # noqa: E402


class UnitConversionTest(unittest.TestCase):
    def test_loader_converts_user_units_to_si(self):
        args = Namespace(
            max_corr_mm=12.0,
            imu_predict_ms=25.0,
            bench_max_error_deg=6.0,
            roll_trim_mm=4.0,
        )
        cfg = runtime_config_from_args(args)
        self.assertAlmostEqual(cfg.control.max_correction_m, mm_to_m(12.0))
        self.assertAlmostEqual(cfg.imu.predict_s, ms_to_s(25.0))
        self.assertAlmostEqual(cfg.safety.bench_max_error_rad, deg_to_rad(6.0))
        self.assertAlmostEqual(cfg.imu.roll_trim_m, mm_to_m(4.0))

    def test_missing_args_fall_back_to_defaults(self):
        cfg = runtime_config_from_args(Namespace())
        self.assertIsInstance(cfg, RuntimeConfig)
        self.assertAlmostEqual(cfg.gait.body_height_m, 0.24)
        self.assertTrue(cfg.gait.natural_soft_trot_enabled)
        self.assertTrue(cfg.gait.natural_trot_enabled)


class FeatureFlagTest(unittest.TestCase):
    def test_negative_switches_map_to_flags(self):
        args = Namespace(no_imu=True, no_gamepad=True, no_tail=True, log=False,
                         gravity_comp=False, var_impedance=True)
        cfg = runtime_config_from_args(args)
        self.assertFalse(cfg.features.imu_enabled)
        self.assertFalse(cfg.features.gamepad_enabled)
        self.assertFalse(cfg.features.tail_enabled)
        self.assertFalse(cfg.features.logging_enabled)
        self.assertFalse(cfg.features.gravity_comp_enabled)
        self.assertTrue(cfg.features.variable_impedance_enabled)

    def test_log_flag_enables_logging(self):
        args = Namespace(log=True)
        cfg = runtime_config_from_args(args)
        self.assertTrue(cfg.features.logging_enabled)


class LoaderOneWayTest(unittest.TestCase):
    """CLI → RuntimeConfig is one-way; no write-back on the walk path."""

    def test_loader_preserves_cli_values_in_typed_config(self):
        args = Namespace(
            height=0.22,
            period=0.91,
            max_corr_mm=13.0,
            imu_predict_ms=18.0,
            tarsus_lead_max_deg=4.0,
            no_gamepad=True,
            gravity_comp=False,
            var_impedance=True,
            dm_kp_fl=70.0,
            dm_kp_fr=80.0,
        )
        cfg = runtime_config_from_args(args)
        self.assertAlmostEqual(cfg.gait.body_height_m, 0.22)
        self.assertAlmostEqual(cfg.gait.period_s, 0.91)
        self.assertAlmostEqual(cfg.control.max_correction_m, mm_to_m(13.0))
        self.assertAlmostEqual(cfg.imu.predict_s, ms_to_s(18.0))
        self.assertAlmostEqual(cfg.dm_tarsus.lead_max_rad, deg_to_rad(4.0))
        self.assertFalse(cfg.features.gamepad_enabled)
        self.assertFalse(cfg.features.gravity_comp_enabled)
        self.assertTrue(cfg.features.variable_impedance_enabled)
        self.assertAlmostEqual(cfg.dm_tarsus.kp_fl, 70.0)
        self.assertAlmostEqual(cfg.dm_tarsus.kp_fr, 80.0)

    def test_offline_bridge_still_fills_namespace_for_tools(self):
        cfg = runtime_config_from_args(Namespace())
        legacy = apply_runtime_config_to_legacy_args(Namespace(), cfg)
        self.assertAlmostEqual(legacy.height, 0.24)
        self.assertAlmostEqual(legacy.imu_angle_tau_ms, 25.0)
        self.assertAlmostEqual(legacy.bench_max_error_deg, 8.0)
        self.assertAlmostEqual(legacy.dm_kp_fl, 220.0)


class BootstrapTest(unittest.TestCase):
    def test_bootstrap_reports_valid_args_without_writeback(self):
        messages = []
        args = Namespace(max_corr_mm=11.0, no_gamepad=True, height=0.21)

        result = bootstrap_runtime_config(args, emit=messages.append)

        self.assertFalse(result.failed)
        self.assertFalse(result.fatal)
        self.assertIsNotNone(result.config)
        self.assertTrue(hasattr(args, "_runtime_config"))
        self.assertFalse(args._runtime_config_error)
        # One-way: CLI values unchanged; typed config carries SI truth.
        self.assertAlmostEqual(args.max_corr_mm, 11.0)
        self.assertAlmostEqual(args.height, 0.21)
        self.assertTrue(args.no_gamepad)
        self.assertAlmostEqual(result.config.control.max_correction_m, mm_to_m(11.0))
        self.assertAlmostEqual(result.config.gait.body_height_m, 0.21)
        self.assertFalse(result.config.features.gamepad_enabled)
        self.assertTrue(any("RuntimeConfig" in msg for msg in messages))

    def test_bootstrap_marks_fatal_on_validation_error(self):
        messages = []
        args = Namespace(period=-1.0, max_corr_mm=9.0)

        result = bootstrap_runtime_config(args, emit=messages.append)

        self.assertTrue(result.fatal)
        self.assertIsNotNone(result.config)
        self.assertTrue(args._runtime_config_error)
        self.assertAlmostEqual(args.period, -1.0)
        self.assertTrue(any("[CONFIG]" in msg for msg in messages))

    def test_bootstrap_fatal_on_build_exception(self):
        class _Boom:
            def __getattr__(self, name):
                raise RuntimeError("boom")

        messages = []
        result = bootstrap_runtime_config(_Boom(), emit=messages.append)
        self.assertTrue(result.fatal)
        self.assertTrue(result.failed)
        self.assertIsNone(result.config)
        self.assertTrue(any("构建失败" in msg for msg in messages))


class ValidationTest(unittest.TestCase):
    def test_default_config_is_valid(self):
        result = validate_runtime_config(RuntimeConfig())
        self.assertTrue(result.ok, msg=str(result.errors))

    def test_bad_period_is_error(self):
        args = Namespace(period=-1.0)
        cfg = runtime_config_from_args(args)
        result = validate_runtime_config(cfg)
        self.assertFalse(result.ok)
        self.assertTrue(any("period" in e for e in result.errors))

    def test_predict_max_below_predict_is_error(self):
        args = Namespace(imu_predict_ms=100.0, imu_predict_max_ms=50.0)
        cfg = runtime_config_from_args(args)
        result = validate_runtime_config(cfg)
        self.assertFalse(result.ok)

    def test_negative_gait_amplitude_is_error(self):
        args = Namespace(amp_front=-0.01)
        cfg = runtime_config_from_args(args)
        result = validate_runtime_config(cfg)
        self.assertFalse(result.ok)
        self.assertTrue(any("amp_front" in e for e in result.errors))

    def test_dm_lead_limit_above_safe_bound_is_error(self):
        args = Namespace(tarsus_lead_max_deg=30.0)
        cfg = runtime_config_from_args(args)
        result = validate_runtime_config(cfg)
        self.assertFalse(result.ok)
        self.assertTrue(any("lead_max" in e for e in result.errors))

    def test_dm_gains_in_range(self):
        base = RuntimeConfig()
        dm = dataclasses.replace(base.dm_tarsus, kd_fl=10.0, kd_fr=10.0)
        cfg = dataclasses.replace(base, dm_tarsus=dm)
        result = validate_runtime_config(cfg)
        self.assertTrue(result.ok)

    def test_summary_is_string(self):
        cfg = RuntimeConfig()
        text = runtime_config_summary(cfg, validate_runtime_config(cfg))
        self.assertIn("RuntimeConfig", text)


if __name__ == "__main__":
    unittest.main()
