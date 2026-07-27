"""离线单测: RuntimeStateMachine 转移合法性/守卫 + SafetySupervisor 各项钳制。

不碰硬件, 用 stub 控制器。运行: python3 test_runtime.py
"""

import math
import unittest
from types import SimpleNamespace

from marsdog_control.config.joints import JOINT_BY_ID
from marsdog_control.core.types import (Direction, MotionTarget, RobotMode, RobotState,
                         UserCommand)
from marsdog_control.runtime.fsm import RuntimeStateMachine
from marsdog_control.safety.supervisor import SafetySupervisor
from marsdog_control.hardware.diagnostics import find_lz_recoverable_faults
from marsdog_control.input.user_input import poll_user_command
import walk
import marsdog_control.motion.kinematics as kinematics


class _FakeGamepad:
    connected = True

    def __init__(self, state):
        self._state = state

    def get_state(self):
        return self._state


class _FakeController:
    def __init__(self, name):
        self.name = name
        self.body_height = 0.22
        self.period = 0.9
        self.amp_front = 0.02
        self.amp_rear = 0.02
        self.turn_cmd = 0.0
        self.turn_y_gain = 1.0
        self.stance_ratio = 0.6
        self._reactive_filtered = 0.0

    def set_height(self, h):
        self.body_height = h

    def set_period(self, p):
        self.period = p


class _FakeSet:
    def __init__(self):
        self.stand = _FakeController("stand")
        self.trot_fwd = _FakeController("trot_fwd")
        self.trot_bwd = _FakeController("trot_bwd")
        self.pace_fwd = _FakeController("pace_fwd")
        self.pace_bwd = _FakeController("pace_bwd")
        self.nat_fwd = _FakeController("nat_fwd")

    def as_tuple(self):
        return (self.stand, self.trot_fwd, self.trot_bwd,
                self.pace_fwd, self.pace_bwd, self.nat_fwd)


def _fake_args():
    return SimpleNamespace(
        throttle_min_scale=0.3, yaw_hold=False, yaw_hold_sign=1.0,
        yaw_hold_kp=0.01, yaw_hold_kd=0.001, yaw_hold_limit=0.3,
        cruise_turn_yamp=0.5, cruise_turn_scale=0.6,
        amp_front=0.02, amp_rear=0.02, bwd_amp_scale=1.0,
        gp_trot_threshold=0.15, gp_deadzone=0.12,
    )


def _make_fsm(natural_configured=False):
    return RuntimeStateMachine(_FakeSet(), _fake_args(),
                               height=0.22,
                               fwd_amp_front=0.02, fwd_amp_rear=0.02,
                               natural_configured=natural_configured)


class TestFSM(unittest.TestCase):
    def test_boot_to_stand(self):
        fsm = _make_fsm()
        self.assertEqual(fsm.mode, RobotMode.STAND)
        self.assertIsNone(fsm.active_gait)

    def test_stand_to_trot_legal(self):
        fsm = _make_fsm()
        ok = fsm.request_transition(RobotMode.TROT, Direction.FWD)
        self.assertTrue(ok)
        self.assertEqual(fsm.mode, RobotMode.TROT)
        self.assertIs(fsm.active_gait, fsm.trot_fwd)

    def test_trot_direction_switch(self):
        fsm = _make_fsm()
        fsm.request_transition(RobotMode.TROT, Direction.FWD)
        fsm.request_transition(RobotMode.TROT, Direction.BWD)
        self.assertEqual(fsm.direction, Direction.BWD)
        self.assertIs(fsm.active_gait, fsm.trot_bwd)

    def test_natural_allowed_when_configured(self):
        fsm = _make_fsm(natural_configured=True)
        ok = fsm.request_transition(RobotMode.NATURAL)
        self.assertTrue(ok)
        self.assertEqual(fsm.mode, RobotMode.NATURAL)
        self.assertIs(fsm.active_gait, fsm.nat_fwd)
        self.assertTrue(fsm.dm_active())   # 配了自然步态 -> 全程主动驱动 tarsus

    def test_dm_active_in_stand_when_natural_configured(self):
        # 带前腿主动 tarsus 的新站姿: STAND 下 tarsus 也必须主动驱动
        fsm = _make_fsm(natural_configured=True)
        self.assertEqual(fsm.mode, RobotMode.STAND)
        self.assertTrue(fsm.dm_active())

    def test_walk_mode_natural_when_configured(self):
        fsm = _make_fsm(natural_configured=True)
        self.assertEqual(fsm.walk_mode, RobotMode.NATURAL)

    def test_walk_mode_falls_back_to_trot(self):
        # 未配自然步态 -> 摇杆"走"退回 StableTrot
        fsm = _make_fsm(natural_configured=False)
        self.assertEqual(fsm.walk_mode, RobotMode.TROT)
        self.assertFalse(fsm.dm_active())

    def test_stick_drive_enters_natural(self):
        # 推杆前进 -> 进入 NATURAL(软 trot), 摆幅按满推缩放到满幅
        fsm = _make_fsm(natural_configured=True)
        cmd = UserCommand(vx=1.0, turn=0.0, has_stick=True)
        fsm.update(RobotState(), cmd, {})
        self.assertEqual(fsm.mode, RobotMode.NATURAL)
        self.assertAlmostEqual(fsm.nat_fwd.amp_front, fsm.nat_amp_front, places=6)
        # 松杆 -> 归站立
        fsm.update(RobotState(), UserCommand(vx=0.0, turn=0.0, has_stick=True), {})
        self.assertEqual(fsm.mode, RobotMode.STAND)

    def test_illegal_transition_rejected(self):
        fsm = _make_fsm()
        fsm.request_transition(RobotMode.NATURAL)
        # NATURAL 只能回 STAND, 不能直达 TROT
        ok = fsm.request_transition(RobotMode.TROT, Direction.FWD)
        self.assertFalse(ok)
        self.assertEqual(fsm.mode, RobotMode.NATURAL)

    def test_estop_from_any(self):
        for setup in (RobotMode.STAND, RobotMode.TROT, RobotMode.NATURAL):
            fsm = _make_fsm()
            if setup is RobotMode.TROT:
                fsm.request_transition(RobotMode.TROT, Direction.FWD)
            elif setup is RobotMode.NATURAL:
                fsm.request_transition(RobotMode.NATURAL)
            ok = fsm.request_transition(RobotMode.ESTOP)
            self.assertTrue(ok)
            self.assertEqual(fsm.mode, RobotMode.ESTOP)
            self.assertIsNone(fsm.active_gait)

    def test_stick_walk_starts_trot(self):
        fsm = _make_fsm()
        st = RobotState()
        cmd = UserCommand(vx=0.8, turn=0.0, has_stick=True)
        fsm.update(st, cmd, last_targets={1: 0.0})
        self.assertEqual(fsm.mode, RobotMode.TROT)
        self.assertEqual(fsm.direction, Direction.FWD)

    def test_stick_idle_returns_stand(self):
        fsm = _make_fsm()
        st = RobotState()
        fsm.update(st, UserCommand(vx=0.8, has_stick=True), {1: 0.0})
        self.assertEqual(fsm.mode, RobotMode.TROT)
        fsm.update(st, UserCommand(vx=0.0, has_stick=True), {1: 0.0})
        self.assertEqual(fsm.mode, RobotMode.STAND)

    def test_natural_stick_does_not_switch(self):
        fsm = _make_fsm()
        fsm.request_transition(RobotMode.NATURAL)
        fsm.update(RobotState(), UserCommand(vx=0.8, has_stick=True), {1: 0.0})
        self.assertEqual(fsm.mode, RobotMode.NATURAL)   # 推杆不把自然步态切走


class _StandTargetStub:
    def __init__(self, targets):
        self.targets = targets

    def get_targets(self, _t):
        return dict(self.targets)


class TestHipAbductionDirectionTest(unittest.TestCase):
    def test_direction_target_uses_only_main_stand_targets_for_four_hips(self):
        """方向测试只能替换四个髋的目标，且最终值必须逐字来自主控 StandController。"""
        self.assertTrue(
            hasattr(walk, "build_hip_abduction_test_target"),
            "walk.py 必须提供主控制链内置的髋外展方向测试目标生成器",
        )
        held = {1: 0.11, 2: -0.02, 3: 0.31, 5: -0.12, 6: 0.03, 9: 0.04, 12: -0.05}
        stand = _StandTargetStub({2: 0.20, 6: -0.21, 9: 0.22, 12: -0.23})

        motion = walk.build_hip_abduction_test_target(
            stand, held, set(held), elapsed_s=9.0, duration_s=3.0,
        )

        self.assertEqual(motion.source_mode, RobotMode.ZEROING)
        self.assertEqual(motion.q[1], held[1])      # 非测试关节必须保持当前位置
        self.assertEqual(motion.q[3], held[3])
        for mid in (2, 6, 9, 12):
            self.assertEqual(motion.q[mid], stand.targets[mid])


class TestStandingPoseIsNew(unittest.TestCase):
    """老两连杆站姿已作废: 唯一站姿=新三连杆带前腿主动 tarsus (脚段 -90°)。"""

    def test_compute_standing_pose_returns_new_3link_pose(self):
        old_alias = kinematics.compute_standing_pose(0.24)
        new_pose = kinematics.compute_standing_pose_3link(0.24, foot_pitch=-math.pi / 2)
        self.assertEqual(old_alias, new_pose)

    def test_stand_controller_defaults_to_new_tarsus_pose(self):
        from marsdog_control.motion.gait_controller import StandController
        stand = StandController(body_height=0.24)
        self.assertTrue(stand.use_tarsus)               # 默认即新站姿, 不再退回老姿态
        tgt = stand.get_targets(0)
        fl_tarsus_id = 4
        # 新站姿的前腿 tarsus 是被主动摆到非零角度的(脚段指地), 老站姿这里=0。
        self.assertNotAlmostEqual(tgt[fl_tarsus_id], 0.0, places=3)


class TestPitchDirectionTest(unittest.TestCase):
    def test_front_left_tarsus_uses_verified_reversed_motor_mapping(self):
        # 实机确认：左前达妙 tarsus 的安装方向与右前相反。
        self.assertEqual(JOINT_BY_ID[4].sign, -1)

    def test_front_left_abduction_urdf_axis_matches_verified_outward_direction(self):
        # 实机验证：fl 的旧正外展方向实际向内，URDF 正方向必须翻转。
        self.assertEqual(kinematics.front_thigh_roll_abd_urdf("fl", 0.16), -0.16)

    def test_pitch_direction_target_uses_main_urdf_to_motor_mapping(self):
        self.assertTrue(
            hasattr(walk, "build_leg_pitch_direction_test_target"),
            "walk.py 必须提供主控制链内置的大腿前后摆方向测试目标生成器",
        )
        held = {1: 0.11, 5: -0.12, 10: 0.13, 13: -0.14, 2: 0.02}
        motion = walk.build_leg_pitch_direction_test_target(
            held, set(held), amplitude_rad=0.20,
        )

        self.assertEqual(motion.source_mode, RobotMode.ZEROING)
        self.assertEqual(motion.q[2], held[2])      # 非测试轴冻结
        # 语义: 前腿向前(URDF 负角) / 后腿向后(URDF 正角)。motor sign 必须由同一
        # urdf_to_motor() 完成，不能在测试模式另写一套电机符号。
        # 前腿 sign(fl=-1,fr=+1): URDF -0.20 -> motor +0.20 / -0.20
        self.assertAlmostEqual(motion.q[1] - held[1], +0.20, places=8)
        self.assertAlmostEqual(motion.q[5] - held[5], -0.20, places=8)
        # 后腿 sign(rl=-1,rr=+1) + 后腿视觉放大 2.0×: URDF +0.40(向后) -> motor -0.40 / +0.40
        self.assertAlmostEqual(motion.q[10] - held[10], -0.40, places=8)
        self.assertAlmostEqual(motion.q[13] - held[13], +0.40, places=8)

    def test_calf_direction_target_uses_main_urdf_to_motor_mapping(self):
        self.assertTrue(
            hasattr(walk, "build_calf_pitch_direction_test_target"),
            "walk.py 必须提供主控制链内置的小腿前摆方向测试目标生成器",
        )
        held = {3: 0.11, 7: -0.12, 11: 0.13, 14: -0.14, 1: 0.02}
        motion = walk.build_calf_pitch_direction_test_target(
            held, set(held), amplitude_rad=0.20,
        )

        self.assertEqual(motion.source_mode, RobotMode.ZEROING)
        self.assertEqual(motion.q[1], held[1])      # 非测试轴冻结
        # 语义: 四个小腿都向前。按主控 FK, 固定大腿时 calf URDF 负角会让足端 +x。
        # 电机符号仍只由 urdf_to_motor()/joint.sign 决定。
        self.assertAlmostEqual(motion.q[3] - held[3], -0.20, places=8)
        self.assertAlmostEqual(motion.q[7] - held[7], +0.20, places=8)
        self.assertAlmostEqual(motion.q[11] - held[11], +0.20, places=8)
        self.assertAlmostEqual(motion.q[14] - held[14], -0.20, places=8)


class TestMotorRecoveryDetection(unittest.TestCase):
    def test_lz_large_error_with_zero_torque_is_recoverable_fault(self):
        fake_lz = SimpleNamespace(
            is_enabled=[True] * 64,
            fault=[0] * 64,
            torque=[0.0] * 64,
            get_position=lambda mid: math.radians(-22.0),
        )
        joint = JOINT_BY_ID[11]  # rl_calf
        faults = find_lz_recoverable_faults(
            fake_lz, [joint], {11: math.radians(-48.9)},
            max_error_rad=math.radians(15.0),
            low_torque_nm=0.10,
        )

        self.assertEqual([f.motor_id for f in faults], [11])

    def test_lz_normal_tracking_is_not_recoverable_fault(self):
        fake_lz = SimpleNamespace(
            is_enabled=[True] * 64,
            fault=[0] * 64,
            torque=[1.2] * 64,
            get_position=lambda mid: math.radians(-47.0),
        )
        joint = JOINT_BY_ID[11]
        faults = find_lz_recoverable_faults(
            fake_lz, [joint], {11: math.radians(-48.9)},
            max_error_rad=math.radians(15.0),
            low_torque_nm=0.10,
        )

        self.assertEqual(faults, [])


class TestLieDownCommand(unittest.TestCase):
    def test_latest_log_lie_down_pose_excludes_head_motors(self):
        pose = walk.load_lie_down_pose_from_log(
            "/home/cat/公共的/20260705_1520/mocap_to_real/log/walk_log_20260711_155837.csv"
        )

        self.assertNotIn(15, pose)
        self.assertNotIn(16, pose)
        self.assertNotIn(17, pose)
        self.assertNotIn(18, pose)
        self.assertIn(1, pose)
        self.assertIn(21, pose)
        self.assertAlmostEqual(pose[11], math.radians(-30.364), places=6)

    def test_lie_down_pose_can_be_saved_and_loaded_from_json(self):
        import os
        import tempfile

        pose = {1: 0.1, 15: 0.9, 18: 0.8, 21: -0.2}
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "lie_down_pose.json")
            walk.save_lie_down_pose(path, pose)
            loaded = walk.load_lie_down_pose(path)

        self.assertEqual(loaded, {1: 0.1, 21: -0.2})

    def test_build_lie_down_target_uses_saved_pose_when_available(self):
        import os
        import tempfile

        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "lie_down_pose.json")
            walk.save_lie_down_pose(path, {1: 0.123, 15: 0.9, 18: 0.8, 21: -0.456})
            target = walk.build_lie_down_target({1, 15, 18, 21}, pose_path=path)

        self.assertEqual(target, {1: 0.123, 21: -0.456})

    def test_left_trigger_edge_requests_lie_down(self):
        fsm = SimpleNamespace(
            args=SimpleNamespace(turn_sign=1.0),
            set_period=lambda p: None,
            trot_fwd=SimpleNamespace(period=0.9),
        )
        inp = walk._InputState()
        base = dict(select=False, b=False, start=False, lb=False, rb=False,
                    ly=0.0, rx=0.0, dpad_up=False, dpad_down=False)

        cmd, _ = poll_user_command(
            _FakeGamepad(SimpleNamespace(**base, lt=0.8)), None, fsm, inp
        )
        self.assertTrue(cmd.request_lie_down)

        cmd, _ = poll_user_command(
            _FakeGamepad(SimpleNamespace(**base, lt=0.9)), None, fsm, inp
        )
        self.assertFalse(cmd.request_lie_down)

        poll_user_command(
            _FakeGamepad(SimpleNamespace(**base, lt=0.0)), None, fsm, inp
        )
        cmd, _ = poll_user_command(
            _FakeGamepad(SimpleNamespace(**base, lt=0.8)), None, fsm, inp
        )
        self.assertTrue(cmd.request_lie_down)

class TestSafety(unittest.TestCase):
    def test_joint_limit_clamp(self):
        sup = SafetySupervisor()
        mid = 1
        j = JOINT_BY_ID[mid]
        st = RobotState(joint_pos={mid: j.limit_hi})   # 实测在上限, 避免跳变闸先触发
        tgt = MotionTarget(q={mid: j.limit_hi + 1.0})
        safe, rep = sup.filter(st, tgt)
        self.assertLessEqual(safe.q[mid], j.limit_hi + 1e-9)
        self.assertIn(mid, rep.clamped_ids)
        self.assertFalse(rep.ok)

    def test_delta_clamp(self):
        sup = SafetySupervisor(max_delta_rad=math.radians(20.0))
        mid = 1
        st = RobotState(joint_pos={mid: 0.0})
        # 首周期建立 prev(=0), 不限
        sup.filter(st, MotionTarget(q={mid: 0.0}))
        # 第二周期相对上次输出跳 90° -> 限到 20°
        safe, rep = sup.filter(st, MotionTarget(q={mid: math.radians(90.0)}))
        self.assertAlmostEqual(safe.q[mid], math.radians(20.0), places=6)
        self.assertIn(mid, rep.clamped_ids)

    def test_first_cycle_no_delta_clamp(self):
        sup = SafetySupervisor(max_delta_rad=math.radians(20.0))
        mid = 1
        j = JOINT_BY_ID[mid]
        q0 = max(j.limit_lo, min(j.limit_hi, math.radians(30.0)))
        safe, rep = sup.filter(RobotState(), MotionTarget(q={mid: q0}))
        self.assertAlmostEqual(safe.q[mid], q0, places=6)   # 首周期不被跳变闸限

    def test_no_clamp_when_within_limits(self):
        sup = SafetySupervisor()
        mid = 1
        st = RobotState(joint_pos={mid: 0.0})
        tgt = MotionTarget(q={mid: math.radians(5.0)})
        safe, rep = sup.filter(st, tgt)
        self.assertTrue(rep.ok)
        self.assertEqual(rep.clamped_ids, [])

    def test_fall_guard_estop(self):
        sup = SafetySupervisor(fall_guard_deg=45.0)
        st = RobotState(imu_connected=True, imu_age_s=0.01,
                        roll=math.radians(60.0))
        _, rep = sup.filter(st, MotionTarget())
        self.assertTrue(rep.triggered_estop)
        self.assertFalse(rep.ok)

    def test_imu_degraded(self):
        sup = SafetySupervisor(imu_max_age_s=0.3, require_imu=False)
        st = RobotState(imu_connected=True, imu_age_s=1.0,
                        roll=0.0)
        _, rep = sup.filter(st, MotionTarget())
        self.assertTrue(rep.imu_degraded)
        self.assertFalse(rep.triggered_estop)   # require_imu=False 只降级不 estop

    def test_imu_lost_estop_when_required(self):
        sup = SafetySupervisor(imu_max_age_s=0.3, require_imu=True)
        st = RobotState(imu_connected=True, imu_age_s=1.0)
        _, rep = sup.filter(st, MotionTarget())
        self.assertTrue(rep.triggered_estop)


if __name__ == "__main__":
    unittest.main(verbosity=2)
