"""Guard: CLI argparse defaults stay derived from ``RuntimeConfig`` schema.

``marsdog_control.config.defaults.CLI`` is built from ``RuntimeConfig()``.
``walk_cli.parse_args`` uses ``default=CLI.*`` for migrated knobs. This test
fails loudly if someone re-introduces hand-copied magic numbers that drift
from schema.
"""

import os
import sys
import unittest
from unittest import mock

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (_ROOT, os.path.join(_ROOT, "src")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from marsdog_control.apps.walk_cli import parse_args  # noqa: E402
from marsdog_control.config.defaults import CLI  # noqa: E402
from marsdog_control.config.loader import runtime_config_from_args  # noqa: E402
from marsdog_control.config.schema import RuntimeConfig  # noqa: E402
from marsdog_control.core.units import deg_to_rad, mm_to_m, ms_to_s  # noqa: E402

# Schema field expectations (authoritative numbers live in schema.py).
# SoftTrot-aligned defaults: lead/predict/dq_ff off; geometry matches NATURAL_SOFT_TROT.
_SCHEMA_DEFAULTS = [
    ("features", "imu_enabled", False),
    ("features", "imu_feedback_enabled", False),
    ("imu", "auto_trim_enabled", False),
    ("imu", "phase_gate_enabled", False),
    ("features", "gravity_comp_enabled", True),
    ("features", "variable_impedance_enabled", False),
    ("features", "tail_enabled", True),
    ("features", "gamepad_enabled", True),
    ("features", "logging_enabled", True),
    ("features", "dm_tarsus_active", True),
    ("features", "dm_dq_feedforward_enabled", False),
    ("features", "ff_decouple_enabled", False),
    ("features", "smooth_gait_enabled", False),
    ("gait", "body_height_m", 0.25),
    ("gait", "period_s", 1.20),
    ("gait", "step_height_m", 0.024),
    ("gait", "amp_front_m", 0.022),
    ("gait", "amp_rear_m", 0.030),
    ("gait", "stance_ratio", 0.74),
    ("gait", "hip_abduction_rad", 0.08),
    ("gait", "ramp_s", 3.5),
    ("gait", "fade_s", 3.0),
    ("gait", "natural_trot_enabled", True),
    ("gait", "natural_soft_trot_enabled", True),
    ("control", "leg_kp_scale", 1.0),
    ("control", "td_kp_scale", 0.4),
    ("control", "swing_kp_scale", 0.7),
    ("control", "td_window_s", 0.15),
    ("control", "gravity_scale", 1.0),
    ("control", "max_correction_m", mm_to_m(20.0)),
    ("control", "imu_slew_m_s", mm_to_m(0.0)),
    ("control", "yaw_hold_kp", 0.03),
    ("control", "yaw_hold_kd", 0.010),
    ("control", "yaw_hold_limit", 0.4),
    ("imu", "predict_s", 0.0),
    ("imu", "predict_max_s", ms_to_s(80.0)),
    ("imu", "gyro_max_age_s", ms_to_s(30.0)),
    ("imu", "angle_tau_s", ms_to_s(25.0)),
    ("imu", "gyro_tau_s", ms_to_s(15.0)),
    ("imu", "kp", 0.05),
    ("imu", "softstart_s", 0.0),
    ("imu", "auto_trim_rate_m_rad_s", 0.08),
    ("imu", "auto_trim_limit_m", mm_to_m(12.0)),
    ("imu", "trim_phases", 1),
    ("imu", "phase_td_gain", 0.35),
    ("imu", "phase_swing_gain", 0.70),
    ("safety", "bench_max_error_rad", deg_to_rad(8.0)),
    ("safety", "bench_max_tilt_rad", deg_to_rad(8.0)),
    ("safety", "bench_max_torque_nm", 5.0),
    ("dm_tarsus", "kp_fl", 220.0),
    ("dm_tarsus", "kp_fr", 220.0),
    ("dm_tarsus", "kd_fl", 10.0),
    ("dm_tarsus", "kd_fr", 10.0),
    ("dm_tarsus", "lead_fl_s", 0.0),
    ("dm_tarsus", "lead_fr_s", 0.0),
    ("dm_tarsus", "lead_max_rad", deg_to_rad(3.0)),
    ("dm_tarsus", "dq_max_rad_s", 3.0),
]


class SchemaDefaultsTest(unittest.TestCase):
    def test_schema_defaults_match_expected_table(self):
        config = RuntimeConfig()
        mismatches = []
        for section, field_name, expected in _SCHEMA_DEFAULTS:
            actual = getattr(getattr(config, section), field_name)
            if actual != expected:
                mismatches.append(
                    f"{section}.{field_name}: schema={actual!r} expected={expected!r}")
        self.assertFalse(mismatches, msg="\n".join(mismatches))

    def test_cli_helper_mirrors_schema(self):
        self.assertAlmostEqual(CLI.height, 0.25)
        self.assertAlmostEqual(CLI.dm_kp_fl, 220.0)
        self.assertAlmostEqual(CLI.max_corr_mm, 20.0)
        self.assertAlmostEqual(CLI.tarsus_lead_fl_ms, 0.0)
        self.assertTrue(CLI.natural_soft_trot)
        self.assertTrue(CLI.gravity_comp)

    def test_empty_cli_loads_to_schema_defaults(self):
        with mock.patch.object(sys, "argv", ["walk.py"]):
            args = parse_args()
        cfg = runtime_config_from_args(args)
        schema = RuntimeConfig()
        self.assertAlmostEqual(cfg.gait.body_height_m, schema.gait.body_height_m)
        self.assertAlmostEqual(cfg.gait.period_s, schema.gait.period_s)
        self.assertAlmostEqual(cfg.dm_tarsus.kp_fl, schema.dm_tarsus.kp_fl)
        self.assertAlmostEqual(cfg.control.leg_kp_scale, schema.control.leg_kp_scale)
        self.assertEqual(cfg.features.gravity_comp_enabled,
                         schema.features.gravity_comp_enabled)
        self.assertTrue(cfg.gait.natural_soft_trot_enabled)
        self.assertTrue(cfg.features.dm_tarsus_active)


if __name__ == "__main__":
    unittest.main()
