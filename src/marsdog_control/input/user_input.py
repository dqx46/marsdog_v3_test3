"""User input helpers for the Marsdog runtime.

This module owns the keyboard reader and the one-cycle input parsing math.  It
does not mutate robot mode directly: `poll_user_command` returns a `UserCommand`
and a development hotkey, while `apply_dev_tuning` is an explicit dev bypass
that mutates only the passed-in controllers/runtime object.
"""

from __future__ import annotations

import select
import sys
from dataclasses import dataclass
from typing import Callable, Optional

from marsdog_control.core.types import Direction, RobotMode, UserCommand

# 手柄阈值默认值(单一真源)。walk 侧同名常量与此对齐; InputManager 直接复用,
# 从而无需反向 import walk。
GP_DEADZONE = 0.12
GP_PERIOD_STEP = 0.05
GP_LT_THRESHOLD = 0.60          # 左扳机 → 平滑回零
GP_LIE_DOWN_LT_THRESHOLD = GP_LT_THRESHOLD  # 兼容旧名
GP_BARK_RT_THRESHOLD = 0.60

try:
    import termios
    import tty
except ImportError:  # Windows/offline tests: keyboard reader simply stays disabled.
    class _TermiosShim:
        TCSADRAIN = 0
        TCIFLUSH = 0
        error = Exception

        @staticmethod
        def tcgetattr(_stdin):
            raise _TermiosShim.error()

        @staticmethod
        def tcsetattr(*_args, **_kwargs):
            return None

        @staticmethod
        def tcflush(*_args, **_kwargs):
            return None

    class _TtyShim:
        @staticmethod
        def setcbreak(_fd):
            return None

    termios = _TermiosShim()
    tty = _TtyShim()


class KeyReader:
    def __init__(self):
        self._old = None
        self._enabled = False

    def start(self):
        try:
            self._old = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
            self._enabled = True
        except termios.error:
            self._enabled = False

    def stop(self):
        if self._old is not None:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old)
            except termios.error:
                pass
            self._old = None
            self._enabled = False

    def flush(self):
        if not self._enabled:
            return
        try:
            import termios as _t
            _t.tcflush(sys.stdin, _t.TCIFLUSH)
        except Exception:
            pass

    def get(self):
        if not self._enabled:
            return None
        try:
            r, _, _ = select.select([sys.stdin], [], [], 0)
            if not r:
                return None
            ch = sys.stdin.read(1)
            if ch == '\x1b':
                for _ in range(10):
                    r2, _, _ = select.select([sys.stdin], [], [], 0.02)
                    if not r2:
                        break
                    b = sys.stdin.read(1)
                    if b.isalpha() or b == '~':
                        break
                return None
            return ch
        except Exception:
            return None


@dataclass
class InputState:
    """输入层的边沿检测状态(按钮上次电平)。手柄/键盘只产出意图, 不改 mode。"""
    gp_start_prev: bool = False
    gp_lb_prev: bool = False
    gp_rb_prev: bool = False
    gp_lt_prev: bool = False
    gp_rt_prev: bool = False
    gp_x_prev: bool = False
    gp_ly_offset: float = 0.0


@dataclass
class DevTuningRuntime:
    """Development hotkey state that was historically stored in walk globals."""
    td_kp_scale: float
    grav_scale: float


def poll_user_command(gp, kb, fsm, inp: InputState, *,
                      gp_period_step: float = GP_PERIOD_STEP,
                      gp_lie_down_lt_threshold: float = GP_LT_THRESHOLD,
                      gp_lt_threshold: Optional[float] = None,
                      gp_bark_rt_threshold: float = GP_BARK_RT_THRESHOLD,
                      gp_deadzone: float = GP_DEADZONE) -> tuple:
    """Input: 手柄+键盘 -> (UserCommand, dev_key)。

    只表达"想干什么"(vx/turn/请求模式/estop/quit), 绝不直接改 mode。
    调参键(dev_key)交给 apply_dev_tuning 走开发旁路, 不进安全关键管线。
    """
    # 兼容旧参数名 gp_lie_down_lt_threshold（现用于 LT→回零）
    lt_thr = (
        float(gp_lt_threshold) if gp_lt_threshold is not None
        else float(gp_lie_down_lt_threshold)
    )
    cmd = UserCommand()

    if gp is not None and gp.connected:
        st = gp.get_state()
        cmd.has_stick = True
        if st.select or st.b:
            cmd.estop = True
        # START 边沿 = 站立<->trot toggle 意图
        if st.start and not inp.gp_start_prev:
            cmd.request_mode = RobotMode.TROT   # FSM 内按当前模式做 toggle
        inp.gp_start_prev = st.start
        # LB/RB 步频步进(开发旁路, 直接调 FSM 持有的控制器组)
        if st.lb and not inp.gp_lb_prev:
            fsm.set_period(min(2.0, fsm.trot_fwd.period + gp_period_step))
        inp.gp_lb_prev = st.lb
        if st.rb and not inp.gp_rb_prev:
            fsm.set_period(max(0.25, fsm.trot_fwd.period - gp_period_step))
        inp.gp_rb_prev = st.rb
        lt_pressed = st.lt > lt_thr
        if lt_pressed and not inp.gp_lt_prev:
            cmd.request_go_zero = True          # LT = 平滑回零
        inp.gp_lt_prev = lt_pressed
        x_pressed = bool(getattr(st, "x", False))
        if x_pressed and not inp.gp_x_prev:
            cmd.request_lie_down = True         # X = 趴下
        inp.gp_x_prev = x_pressed
        rt_pressed = getattr(st, "rt", 0.0) > gp_bark_rt_threshold
        if rt_pressed and not inp.gp_rt_prev:
            cmd.request_bark = True
        inp.gp_rt_prev = rt_pressed
        cmd.vx = -(st.ly - inp.gp_ly_offset)
        cmd.turn = -st.rx * fsm.drive.turn_sign if abs(st.rx) > gp_deadzone else 0.0
        if st.dpad_up or st.dpad_down:
            cmd.pace = True
            cmd.request_dir = Direction.FWD if st.dpad_up else Direction.BWD

    dev_key = None
    key = kb.get() if kb is not None else None
    if key in ('q', 'Q', '\x03'):
        cmd.quit = True
    elif key in (' ', 's', 'S'):
        cmd.request_mode = RobotMode.TROT       # toggle(站立<->trot)
    elif key == '3':
        cmd.request_mode = RobotMode.NATURAL
    elif key in ('z', 'Z'):
        cmd.request_sit = True                  # 坐下(sit_pose.json)
    elif key in ('p', 'P'):
        cmd.request_lie_down = True             # 趴下(lie_down_pose.json)
    elif key == '0':
        cmd.request_go_zero = True              # 平滑回零(全轴→0)
    elif key in ('a', 'A'):
        cmd.request_abd_flare_toggle = True     # 站立四腿外展方向验证
    else:
        dev_key = key                           # 其余交给开发调参旁路
    return cmd, dev_key


def apply_dev_tuning(dev_key, fsm, imu_ctrl, lz, evo, dm, rt: DevTuningRuntime,
                     *, check_motors: Callable) -> bool:
    """开发调试旁路: 步频/体高/摆幅/IMU增益/配平/重力补偿等热键直改控制器/IMU。

    这些不是上机安全关键路径, 刻意不走 UserCommand->pipeline, 以保持安全路径干净。
    返回是否处理了该键(用于打印回显之类, 目前忽略)。
    """
    if not dev_key:
        return False
    k = dev_key
    if k in ('i', 'I'):
        # 原 p=状态；p 已留给趴下姿势
        check_motors(lz, evo, dm, label="dev")
    elif k in ('+', '='):
        fsm.set_period(max(0.25, fsm.trot_fwd.period - 0.05))
    elif k in ('-', '_'):
        fsm.set_period(min(2.0, fsm.trot_fwd.period + 0.05))
    elif k == 'u':
        fsm.set_height(min(0.30, fsm.height + 0.01))
    elif k == 'd':
        fsm.set_height(max(0.15, fsm.height - 0.01))
    elif k == 'f':
        fsm.adjust_fwd_amp(+0.005)
    elif k == 'v':
        fsm.adjust_fwd_amp(-0.005)
    elif k == ']':
        imu_ctrl.p_boost = min(5.0, imu_ctrl.p_boost + 0.5)
    elif k == '[':
        imu_ctrl.p_boost = max(1.0, imu_ctrl.p_boost - 0.5)
    elif k == "'":
        imu_ctrl.kp_roll = min(0.10, imu_ctrl.kp_roll + 0.005)
        imu_ctrl.kp_pitch = min(0.10, imu_ctrl.kp_pitch + 0.005)
    elif k == ';':
        imu_ctrl.kp_roll = max(0.005, imu_ctrl.kp_roll - 0.005)
        imu_ctrl.kp_pitch = max(0.005, imu_ctrl.kp_pitch - 0.005)
    elif k == '.':
        rt.td_kp_scale = min(1.0, rt.td_kp_scale + 0.05)
    elif k == ',':
        rt.td_kp_scale = max(0.05, rt.td_kp_scale - 0.05)
    elif k == 'l':
        imu_ctrl.roll_trim = min(0.015, imu_ctrl.roll_trim + 0.0005)
    elif k == 'k':
        imu_ctrl.roll_trim = max(-0.015, imu_ctrl.roll_trim - 0.0005)
    elif k == 'm':
        rt.grav_scale = min(1.5, rt.grav_scale + 0.1)
    elif k == 'n':
        rt.grav_scale = max(-1.5, rt.grav_scale - 0.1)
    else:
        return False
    return True


__all__ = [
    "DevTuningRuntime",
    "InputState",
    "KeyReader",
    "apply_dev_tuning",
    "poll_user_command",
]
