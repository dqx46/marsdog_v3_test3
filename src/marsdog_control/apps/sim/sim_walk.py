import time
import sys

import os


def _prefer_x11_for_mujoco_viewer() -> None:
    """Avoid MuJoCo/GLFW Wayland HiDPI bug: scene stuck in a corner of a black window.

    pip's ``glfw`` ships separate Wayland/X11 ``libglfw.so``. On this machine the
    Wayland build reports monitor scale 2.0 while the framebuffer stays 1x, so
    Simulate only fills ~1/4 of the window. Force the bundled X11 backend via
    ``PYGLFW_LIBRARY`` (keeps ``WAYLAND_DISPLAY`` intact — clearing it breaks
    Wayland-only GLFW and yields ``Failed to connect to display``).

    Set ``MARDOG_ALLOW_WAYLAND=1`` to keep the default Wayland glfw.
    """
    if os.environ.get("MARDOG_ALLOW_WAYLAND") == "1":
        return
    if os.environ.get("PYGLFW_LIBRARY"):
        return
    try:
        import importlib.util
        spec = importlib.util.find_spec("glfw")
        if spec is None or not spec.submodule_search_locations:
            return
        root = spec.submodule_search_locations[0]
        x11_so = os.path.join(root, "x11", "libglfw.so")
        if os.path.isfile(x11_so):
            os.environ["PYGLFW_LIBRARY"] = x11_so
            os.environ.setdefault("GDK_BACKEND", "x11")
            os.environ.setdefault("QT_QPA_PLATFORM", "xcb")
    except Exception:
        return


# Must run before importing mujoco.viewer / glfw.
_prefer_x11_for_mujoco_viewer()

# 如果需要图形界面，需要确保在主线程
try:
    if os.environ.get("NO_VIEWER") == "1":
        HAS_VIEWER = False
    else:
        import mujoco.viewer
        HAS_VIEWER = True
except ImportError:
    HAS_VIEWER = False

from marsdog_control.backends.sim import SimRobotBackend, SimPhysicsOptions
from marsdog_control.backends.base import RobotBackend
from marsdog_control.core.protocols import ClockLike
from marsdog_control.core.types import RobotState, ControlOutput
from marsdog_control.config.joints import JOINT_MAP
from marsdog_control.runtime.walk_controllers import assemble_walk_control_stack
from marsdog_control.runtime.walk_assembly import assemble_walk_loop_context
from marsdog_control.runtime.walk_loop import LoopHardware, tick_walk_loop
from marsdog_control.runtime.fsm import RuntimeStateMachine
from marsdog_control.input.hal import WalkInputHAL
from marsdog_control.input.user_input import KeyReader, InputState
from marsdog_control.safety.supervisor import SafetySupervisor
from marsdog_control.control.imu_balance import ImuAttitudeController
from marsdog_control.runtime.walk_state import WalkRuntimeState
from marsdog_control.runtime.walk_startup import WalkStartupContext


class DummyMotor:
    def __init__(self):
        self.is_connected = [True] * 24
        self.is_enabled = [True] * 24
        self.status = [0x02] * 24
        self.limit_hit = [False] * 24
        self.fault = [0] * 24
        self.over_temp = [False] * 24
        self.vbus = [24.0] * 24

class SimImuFake:
    """桥接 SimRobotBackend 的 IMU 数据"""
    def __init__(self, backend: SimRobotBackend):
        self.backend = backend
        self.connected = True
        self.rx_count = 1
        
    def poll(self):
        # 实际读取由 backend 维护
        pass

    @property
    def roll(self):
        r, p, wx, wy = self.backend.base_imu()
        return r
        
    @property
    def pitch(self):
        r, p, wx, wy = self.backend.base_imu()
        return p
        
    @property
    def gyro_roll(self):
        r, p, wx, wy = self.backend.base_imu()
        return wx
        
    @property
    def gyro_pitch(self):
        r, p, wx, wy = self.backend.base_imu()
        return wy

    def frame_ages(self, now):
        return {"roll": 0.0, "pitch": 0.0, "gyro": 0.0}

class SimClock(ClockLike):
    """基于 MuJoCo 仿真时间的时钟"""
    def __init__(self, backend: SimRobotBackend):
        self.backend = backend
        
    def time(self) -> float:
        return float(self.backend.sim_time)
        
    def monotonic(self) -> float:
        return float(self.backend.sim_time)
        
    def sleep(self, seconds: float) -> None:
        # 在仿真中 sleep 通常不应该真的阻塞，或者我们直接推进时间
        # 为了与 tick_walk_loop 的 sleep 兼容，我们不使用真实睡眠，而是通过步进物理来消耗时间。
        # 但在 sim_walk 这里，物理 step 是外层维护的，所以我们这里可以作为空转。
        pass


def make_controllers(args, startup):
    from marsdog_control.runtime.walk_controllers import assemble_walk_control_stack
    stack = assemble_walk_control_stack(
        args,
        natural_active=startup.natural_active,
        natural_params=startup.natural_params,
        gp_trot_threshold=0.3,
        gp_deadzone=0.12,
        natural_soft=startup.natural_soft,
        natural_walk=getattr(startup, "natural_walk", False),
        walk_params=getattr(startup, "walk_params", None),
        natural_jump=getattr(startup, "natural_jump", False),
        jump_params=getattr(startup, "jump_params", None),
        trot_flag=startup.trot_flag,
        no_spine=startup.no_spine,
        load_trim_cal=lambda: None
    )
    return stack


def main():
    is_headless = "--headless" in sys.argv

    # 1. CLI / startup (先解析站高，再给 Sim 初始姿态)
    from marsdog_control.apps.walk_cli import parse_args
    from marsdog_control.runtime.walk_startup import prepare_walk_startup as _prepare_walk_startup
    from marsdog_control.config.gains import SIM_JOINT_GAINS

    old_argv = sys.argv
    # 保留用户透传 flag。默认: natural_soft_trot；若用户要 WBC 则不再强塞 VMC。
    # --headless / --duration 仅仿真入口识别，不进 walk_cli
    # 有窗口时默认一直跑到关窗；只有显式 --duration 才限时退出。
    # 无头默认 5s，避免挂死。
    duration_s = 5.0
    duration_explicit = False
    # 默认与真机 cruise_vx (SI m/s) 对齐；原地转: --vx 0 --turn 0.8
    # 0.10 m/s ≈ 新菜谱中速；半速 --vx 0.067；偏快满幅 --vx 0.13
    drive_vx = 0.10
    drive_turn = 0.0
    stand_only = False
    filtered = []
    i = 1
    while i < len(old_argv):
        a = old_argv[i]
        if a in ("--stand-only", "--stand"):
            stand_only = True
            i += 1
            continue
        if a == "--duration" and i + 1 < len(old_argv):
            try:
                duration_s = float(old_argv[i + 1])
                duration_explicit = True
            except ValueError:
                pass
            i += 2
            continue
        if a.startswith("--duration="):
            try:
                duration_s = float(a.split("=", 1)[1])
                duration_explicit = True
            except ValueError:
                pass
            i += 1
            continue
        if a == "--vx" and i + 1 < len(old_argv):
            try:
                drive_vx = float(old_argv[i + 1])
            except ValueError:
                pass
            i += 2
            continue
        if a.startswith("--vx="):
            try:
                drive_vx = float(a.split("=", 1)[1])
            except ValueError:
                pass
            i += 1
            continue
        if a == "--turn" and i + 1 < len(old_argv):
            try:
                drive_turn = float(old_argv[i + 1])
            except ValueError:
                pass
            i += 2
            continue
        if a.startswith("--turn="):
            try:
                drive_turn = float(a.split("=", 1)[1])
            except ValueError:
                pass
            i += 1
            continue
        if a in ("--headless", "--natural-soft-trot", "--natural-walk", "--jump",
                 "--vmc", "--no-vmc", "--wbc", "--no-wbc", "--stand-only", "--stand"):
            i += 1
            continue
        filtered.append(a)
        i += 1

    if stand_only:
        drive_vx = 0.0
        drive_turn = 0.0

    forced = ["--natural-soft-trot"]
    user_wants_jump = "--jump" in old_argv
    user_wants_walk = "--natural-walk" in old_argv
    if user_wants_jump:
        forced.append("--jump")
    elif user_wants_walk:
        forced.append("--natural-walk")
    # 控制器三态显式化: 不再"未开 --wbc 就静默塞 --vmc"。
    #   --wbc        → WBC+MPC (覆盖 VMC)
    #   --vmc        → 解耦 VMC (软腿 + Z/Roll 雅可比 trq_ff)
    #   两者都不给   → 干净基线: 裸关节阻抗 + 重力补偿 (--no-vmc --no-wbc)
    user_wants_wbc = "--wbc" in old_argv and "--no-wbc" not in old_argv
    user_wants_vmc = "--vmc" in old_argv and "--no-vmc" not in old_argv
    if user_wants_wbc:
        forced.extend(["--wbc", "--no-vmc"])
    elif user_wants_vmc:
        forced.extend(["--vmc", "--no-wbc"])
    else:
        forced.extend(["--no-vmc", "--no-wbc"])
    # Real-parity default: estimator path unless user explicitly asks for truth.
    if not any(
        a == "--base-estimate-mode" or a.startswith("--base-estimate-mode=")
        for a in old_argv
    ):
        forced.extend(["--base-estimate-mode", "estimator"])

    # Inject SI cruise so assemble/teleop banner matches sim --vx.
    cruise_flags = ["--cruise-vx", f"{abs(float(drive_vx)):.6g}"]
    sys.argv = [sys.argv[0]] + forced + cruise_flags + filtered
    args = parse_args()
    sys.argv = old_argv

    runtime_state = WalkRuntimeState()
    startup = _prepare_walk_startup(
        args, runtime_state=runtime_state, joint_gains=SIM_JOINT_GAINS)
    print(
        "[Sim] using SIM_JOINT_GAINS (SI impedance; real Incos load gains not applied)"
    )
    if stand_only:
        print("[Sim] --stand-only：保持 STAND，不自动切入行走")
    print(
        f"[Sim] --vx {drive_vx:.3f} → cruise_vx={abs(float(drive_vx)):.3f} m/s "
        f"(中速默认 0.10；满幅约 0.13；半速 0.067)"
    )

    # 仿真侧：达妙无真实反馈，用固定目标占位，避免 status 误报 disabled
    runtime_state.dm.fixed_targets[4] = 0.0
    runtime_state.dm.fixed_targets[8] = 0.0

    est_mode = startup.runtime_config.dynamics.base_estimate_mode
    if est_mode == "truth":
        print(
            "[Sim] WARN: --base-estimate-mode=truth (debug only); "
            "real robot uses estimator — do not treat this run as deploy parity"
        )

    spot_demo = abs(drive_vx) < 0.12 and abs(drive_turn) >= 0.12
    if spot_demo:
        print(
            f"[Sim] Spot-turn demo  vx={drive_vx:.2f} turn={drive_turn:.2f}  "
            "(abduction-led; amp_x=0)"
        )

    if runtime_state.wbc_enabled:
        print(
            f"[Sim] WBC+MPC ON  leg_kp_scale={runtime_state.leg_kp_scale:.2f}  "
            f"estimate={est_mode}  "
            f"duration={duration_s:.1f}s"
        )
    elif runtime_state.vmc_enabled:
        print(
            f"[Sim] VMC ON  leg_kp_scale={runtime_state.leg_kp_scale:.2f}  "
            f"(软腿 + Z/Roll 雅可比 trq_ff)"
        )
    else:
        print(
            f"[Sim] 基线 (VMC/WBC OFF)  leg_kp_scale={runtime_state.leg_kp_scale:.2f}  "
            f"(裸关节阻抗 + 重力补偿, gravity_comp={runtime_state.gravity_comp})"
        )

    # 2. Controllers → Stand 决定初始 qpos
    stack = make_controllers(args, startup)
    stand = stack.stand
    fsm = stack.fsm
    safety = stack.safety
    imu_ctrl = stack.imu_ctrl
    # Sim CLI --vx = teleop cruise speed [m/s]（与真机摇杆走/停语义一致）
    from dataclasses import replace
    fsm.drive = replace(fsm.drive, cruise_vx=max(0.0, abs(float(drive_vx))))
    print(
        f"[Sim] teleop cruise_vx={fsm.drive.cruise_vx:.3f} m/s "
        f"(SI; stick depth ignored)"
    )

    # 3. Backend（用 recipe 站高初始化，避免默认 0.22m 与 gait 不一致）
    # Jump: slightly harder foot contact so push force doesn't bury soft soles.
    if getattr(startup, "natural_jump", False):
        physics = SimPhysicsOptions(foot_solref=(0.004, 1.0))
    else:
        physics = SimPhysicsOptions()
    backend = SimRobotBackend(stand_controller=stand, physics_options=physics)

    # 4. Fakes / HW
    clock = SimClock(backend)
    fsm.clock = clock
    fsm.t_gait = clock.time()
    
    imu = SimImuFake(backend)
    online_ids = set(j.motor_id for j in JOINT_MAP)
    dummy_hw = DummyMotor()
    hw = LoopHardware(imu=imu, online=online_ids, lz=dummy_hw, evo=dummy_hw, dm=dummy_hw, incos=dummy_hw)

    # 5. Inputs
    keyboard = KeyReader()
    input_state = InputState()
    gamepad = None
    input_hal = WalkInputHAL(gamepad, keyboard, input_state, runtime_state)

    def fake_poll(fsm):
        from marsdog_control.core.types import UserCommand, RobotMode
        cmd = UserCommand()
        if stand_only:
            # 纯站立：永不切入 NATURAL/WALK，摇杆保持零
            fake_poll.tick += 1
            return cmd, None
        if fake_poll.tick == 200:
            if getattr(startup, "natural_jump", False):
                target = RobotMode.JUMP
            elif getattr(startup, "natural_walk", False):
                target = RobotMode.WALK
            else:
                target = RobotMode.NATURAL
            cmd.request_mode = target
            print(f"[Sim] Auto-triggering {target.value} via UserCommand...")
        if fake_poll.tick > 200:
            # 模拟真机摇杆 engage；实际速度由 teleop→cruise_vx(m/s)→schedule
            if abs(drive_vx) > 1e-9:
                cmd.vx = 1.0 if drive_vx > 0 else -1.0
            else:
                cmd.vx = 0.0
            cmd.turn = drive_turn
            cmd.has_stick = True
        fake_poll.tick += 1
        return cmd, None
    fake_poll.tick = 0

    input_hal.poll = fake_poll

    targets = dict(stand.get_targets(0.0))

    # 组装完整的 WalkLoopContext
    # 注意我们使用了一些 dummy 补丁。由于 v3 要求一些回调，我们使用基础 lambda
    ctx = assemble_walk_loop_context(
        startup=startup,
        runtime_state=runtime_state,
        hw=hw,
        fsm=fsm,
        input_hal=input_hal,
        stand=stand,
        safety=safety,
        imu_ctrl=imu_ctrl,
        targets=targets,
        cur_pos=targets.copy(),
        smooth_tgt={},
        real_joints=[j for j in JOINT_MAP],
        joint_map=JOINT_MAP,
        direction_test_base=targets.copy(),
        direction_test_start=0.0,
        control_hz=200.0,
        clock=clock,
        write_log=lambda x: None,
        log_writer=None,
        bark_with_mouth=lambda: None,
        build_lie_down_target=lambda a,b,c,d,e,f,g: targets.copy(),
        read_positions=lambda a,b,c,d: targets.copy(),
        smooth_transition=lambda a,b,c,d,e: targets.copy(),
        backend=backend,
    )
    
    keyboard.start()
    if HAS_VIEWER and not is_headless:
        gl = os.environ.get("PYGLFW_LIBRARY", "")
        print(
            "[Sim] viewer: using GLFW X11 backend"
            + (f" ({os.path.basename(os.path.dirname(gl))}/libglfw.so)" if gl else "")
            + " to avoid Wayland black-corner. Set MARDOG_ALLOW_WAYLAND=1 to override."
        )
    print("[Sim] Running loop...")
    tick = 0
    last_wall = time.time()
    wall0 = last_wall
    sim0 = float(backend.sim_time)
    last_rt_report = wall0
    max_ticks = int(duration_s * 200)
    try:
        if HAS_VIEWER and not is_headless:
            # 与旧版 sim_walk 一致：控制 + 物理 + viewer 全在主线程，避免段错误
            with mujoco.viewer.launch_passive(backend.model, backend.data) as viewer:
                while viewer.is_running() and ctx.running:
                    keep_going = tick_walk_loop(ctx)
                    if not keep_going:
                        break
                    backend.step()
                    tick += 1

                    # 降低渲染频率到 50Hz (每 4 个控制周期渲染一次)
                    # 否则 viewer.sync() 每次消耗太多时间，会导致仿真慢动作播放
                    if tick % 4 == 0:
                        pos = backend.base_pos
                        viewer.cam.lookat[0] = pos[0]
                        viewer.cam.lookat[1] = pos[1]
                        viewer.cam.lookat[2] = 0.15
                        viewer.sync()

                    # 使用绝对时间追踪，把 viewer.sync 欠下的时间在后续帧追回来
                    target_wall = wall0 + (float(backend.sim_time) - sim0)
                    now_wall = time.time()
                    sleep_t = target_wall - now_wall
                    if sleep_t > 0:
                        time.sleep(sleep_t)
                    last_wall = time.time()
                    if last_wall - last_rt_report >= 1.0:
                        sim_elapsed = float(backend.sim_time) - sim0
                        wall_elapsed = last_wall - wall0
                        rt = sim_elapsed / max(1e-9, wall_elapsed)
                        print(
                            f"[Sim][RT] wall={wall_elapsed:.2f}s sim={sim_elapsed:.2f}s "
                            f"realtime_factor={rt:.3f}x ticks={tick}"
                        )
                        last_rt_report = last_wall
                    if duration_explicit and tick >= max_ticks:
                        print(f"[Sim] duration reached ({duration_s:.1f}s sim), stopping viewer loop")
                        break
        else:
            import math
            import numpy as np

            trq_peak = 0.0
            fz_peak = 0.0
            report_ticks = {
                max(1, max_ticks // 5),
                max(1, 2 * max_ticks // 5),
                max(1, 3 * max_ticks // 5),
                max(1, 4 * max_ticks // 5),
                max_ticks,
            }
            while ctx.running:
                keep_going = tick_walk_loop(ctx)
                if not keep_going:
                    break
                backend.step()
                tick += 1
                if tick in report_ticks or tick in (50, 250, 600):
                    tel = ctx.executor.telemetry
                    if tel.get("fc_des"):
                        fc = tel["fc_des"][-1]
                        fz_peak = max(
                            fz_peak,
                            float(max(abs(fc[2]), abs(fc[5]), abs(fc[8]), abs(fc[11]))),
                        )
                    if tel.get("tau_opt"):
                        tau = tel["tau_opt"][-1]
                        trq_peak = max(trq_peak, float(np.max(np.abs(tau))))
                    mpc_ok = tel["mpc_ok"][-1] if tel.get("mpc_ok") else False
                    wbc_ok = tel["wbc_ok"][-1] if tel.get("wbc_ok") else False
                    print(
                        f"[Sim] t={tick}/{max_ticks} "
                        f"wbc={ctx.executor.config.wbc_enabled} "
                        f"vmc={ctx.executor.config.vmc_enabled} "
                        f"leg_kp={ctx.executor.config.leg_kp_scale:.2f} "
                        f"|tau|_max={trq_peak:.2f}Nm |Fz|_peak={fz_peak:.1f}N "
                        f"mpc_ok={mpc_ok} wbc_ok={wbc_ok} "
                        f"roll={imu.roll:.3f} pitch={imu.pitch:.3f} "
                        f"z={backend.base_pos[2]:.3f}"
                    )
                if tick >= max_ticks:
                    rolls = list(ctx.executor.telemetry.get("roll", []))
                    pitches = list(ctx.executor.telemetry.get("pitch", []))
                    r_peak = max((abs(x) for x in rolls), default=0.0)
                    p_peak = max((abs(x) for x in pitches), default=0.0)
                    wall_elapsed = time.time() - wall0
                    sim_elapsed = float(backend.sim_time) - sim0
                    rt = sim_elapsed / max(1e-9, wall_elapsed)
                    print(
                        f"[Sim] Test finished ({duration_s:.1f}s). "
                        f"wall={wall_elapsed:.2f}s sim={sim_elapsed:.2f}s "
                        f"realtime_factor={rt:.3f}x "
                        f"roll_peak={math.degrees(r_peak):.1f}deg "
                        f"pitch_peak={math.degrees(p_peak):.1f}deg "
                        f"z_end={backend.base_pos[2]:.3f} "
                        f"|tau|_max={trq_peak:.2f}Nm |Fz|_peak={fz_peak:.1f}N "
                        f"wbc={ctx.executor.config.wbc_enabled}"
                    )
                    if r_peak > math.radians(25) or p_peak > math.radians(25):
                        print("[Sim] WARN: attitude peak > 25deg (unstable)")
                    elif fz_peak > 150:
                        print("[Sim] WARN: Fz peak > 150N (force saturation risk)")
                    else:
                        print("[Sim] OK: attitude/force within soft limits")
                    break
    except KeyboardInterrupt:
        pass
    finally:
        print("[Sim] Shutting down...")
        if hasattr(ctx.executor, "telemetry"):
            import json
            from collections import deque
            import numpy as np

            class NumpyEncoder(json.JSONEncoder):
                def default(self, obj):
                    if isinstance(obj, np.ndarray):
                        return obj.tolist()
                    if isinstance(obj, deque):
                        return list(obj)
                    if isinstance(obj, (np.floating, np.integer)):
                        return obj.item()
                    return json.JSONEncoder.default(self, obj)

            tel = ctx.executor.telemetry
            # Prefer DynamicsTelemetry helpers when available
            if hasattr(tel, "as_lists"):
                payload = tel.as_lists()
            else:
                payload = {k: list(v) for k, v in tel.items()}
            with open("telemetry.json", "w") as f:
                json.dump(payload, f, cls=NumpyEncoder)
            print("[Sim] Telemetry saved to telemetry.json")

            csv_n = 0
            if hasattr(tel, "write_csv"):
                csv_n = tel.write_csv("telemetry.csv")
                print(f"[Sim] Telemetry CSV saved to telemetry.csv ({csv_n} rows)")
            if hasattr(tel, "write_summary_json"):
                tel.write_summary_json(
                    "telemetry_summary.json",
                    extra={
                        "source": "sim_walk",
                        "note": "Compare with real same-param run; prefer estimator mode",
                    },
                )
                print("[Sim] Telemetry summary saved to telemetry_summary.json")
            if hasattr(tel, "format_summary"):
                print(tel.format_summary(prefix="[Tel]"))

        keyboard.stop()


if __name__ == "__main__":
    main()
