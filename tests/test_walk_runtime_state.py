"""Unit tests for WalkRuntimeState."""

import unittest

from marsdog_control.control.executor import CommandExecutor, ExecutorConfig
from marsdog_control.runtime.walk_state import WalkRuntimeState


class WalkRuntimeStateTest(unittest.TestCase):
    def test_sync_executor_and_actuation_snapshot(self):
        state = WalkRuntimeState(
            joint_gains={"fl_hip_pitch": {"kp": 1.0, "kd": 1.0, "trq_ff": 0.0}},
            leg_kp_scale=0.65,
            gravity_comp=True,
            gravity_scale=1.0,
        )
        state.impedance.enabled = True
        state.impedance.td_kp_scale = 0.35
        state.dm.active = True
        state.dm.fixed_targets = {4: 0.1}

        executor = CommandExecutor(config=ExecutorConfig())
        state.sync_executor_config(executor)
        self.assertTrue(executor.config.variable_impedance)
        self.assertAlmostEqual(executor.config.td_kp_scale, 0.35)
        self.assertTrue(executor.config.gravity_comp)

        act = state.to_actuation_runtime()
        self.assertTrue(act.dm_tarsus_active)
        self.assertEqual(act.dm_fixed_targets[4], 0.1)
        self.assertAlmostEqual(act.leg_kp_scale, 0.65)

    def test_dev_tuning_roundtrip(self):
        state = WalkRuntimeState()
        rt = state.as_dev_tuning()
        rt.td_kp_scale = 0.55
        rt.grav_scale = 0.8
        state.apply_dev_tuning_result(rt)
        self.assertAlmostEqual(state.impedance.td_kp_scale, 0.55)
        self.assertAlmostEqual(state.gravity_scale, 0.8)

    def test_apply_control_and_dm_from_cli(self):
        state = WalkRuntimeState()
        state.apply_control_features(
            leg_kp_scale=0.8,
            var_impedance=True,
            td_kp_scale=0.3,
            swing_kp_scale=0.6,
            td_window=0.12,
            gravity_comp=True,
            gravity_scale=0.7,
        )
        self.assertTrue(state.impedance.enabled)
        self.assertAlmostEqual(state.leg_kp_scale, 0.8)
        state.apply_dm_tarsus(
            active=True,
            kp_fl=70.0, kp_fr=75.0, kd_fl=2.5, kd_fr=2.6,
            lead_fl_s=0.02, lead_fr_s=0.03, lead_max_s=0.1,
            lead_max_rad=0.05, dq_feedforward=False, dq_max_rps=2.0,
        )
        self.assertTrue(state.dm.active)
        self.assertAlmostEqual(state.dm.kp_by_id[4], 70.0)
        self.assertAlmostEqual(state.dm.kp_by_id[8], 75.0)
        self.assertFalse(state.dm.dq_feedforward)


if __name__ == "__main__":
    unittest.main()
