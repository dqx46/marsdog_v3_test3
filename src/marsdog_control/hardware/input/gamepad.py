"""
gamepad.py — Xbox 360 手柄读取模块 (基于 Linux joystick API /dev/input/jsX)

兼容：有线 Xbox 360 / 2.4G 无线 PS2 兼容手柄（USB 接收器自报 045e:028e）

设备映射（与 mydog_ref/GamepadRemoteInput.cpp 完全一致）
----------------------------------------------
轴索引  含义          范围
  0     左摇杆 X     -32767 ~ +32767 (左负 右正)
  1     左摇杆 Y     -32767 ~ +32767 (上正 下负, 驱动层已取反)
  2     LT           -32767 ~ +32767 (未按=-32767, 全按=+32767)
  3     右摇杆 X     -32767 ~ +32767
  4     右摇杆 Y     -32767 ~ +32767 (驱动层已取反)
  5     RT           -32767 ~ +32767
  6     D-pad X      -1/0/+1 (左=-1, 右=+1)
  7     D-pad Y      -1/0/+1 (上=-1, 下=+1)

按键索引  含义
  0   A        1   B        2   X        3   Y
  4   LB       5   RB       6   SELECT   7   START
  8   GUIDE    9   LS(左摇杆按下)  10  RS(右摇杆按下)
"""

import os
import struct
import threading
from dataclasses import dataclass, field
from typing import Optional

# joystick 事件格式: (timestamp_ms u32, value i16, type u8, number u8)
_JS_EVENT_FMT = "IhBB"
_JS_EVENT_SIZE = struct.calcsize(_JS_EVENT_FMT)
_JS_EVENT_BUTTON = 0x01
_JS_EVENT_AXIS   = 0x02
_JS_EVENT_INIT   = 0x80

# 轴归一化范围
_AXIS_MAX = 32767.0

# 死区（归一化后）
_DEADZONE = 0.08


def _normalize_axis(raw: int, deadzone: float = _DEADZONE) -> float:
    """将原始轴值 [-32767, 32767] 归一化为 [-1.0, 1.0]，并应用死区。"""
    v = raw / _AXIS_MAX
    v = max(-1.0, min(1.0, v))
    if abs(v) < deadzone:
        return 0.0
    sign = 1.0 if v > 0 else -1.0
    return sign * (abs(v) - deadzone) / (1.0 - deadzone)


def _normalize_trigger(raw: int) -> float:
    """将扳机轴值 [-32767, 32767] 归一化为 [0.0, 1.0]。"""
    return (raw + _AXIS_MAX) / (2.0 * _AXIS_MAX)


@dataclass
class GamepadState:
    """当前手柄状态快照（线程安全副本）。"""
    # 摇杆，归一化 [-1, 1]
    lx: float = 0.0   # 左 X (右为正)
    ly: float = 0.0   # 左 Y (上为正，驱动层已取反)
    rx: float = 0.0   # 右 X
    ry: float = 0.0   # 右 Y (上为正)
    # 扳机，归一化 [0, 1]
    lt: float = 0.0
    rt: float = 0.0
    # 按键 (bool)
    a: bool = False
    b: bool = False
    x: bool = False
    y: bool = False
    lb: bool = False
    rb: bool = False
    select: bool = False
    start: bool = False
    ls: bool = False    # 左摇杆按下
    rs: bool = False    # 右摇杆按下
    # 方向键 — 来自轴6/7 (hat axes)
    dpad_up: bool    = False
    dpad_down: bool  = False
    dpad_left: bool  = False
    dpad_right: bool = False
    # 原始轴值 (8个)
    raw_axes: list = field(default_factory=lambda: [0] * 8)
    connected: bool = True


class Gamepad:
    """
    非阻塞手柄读取器，后台线程持续消费 /dev/input/jsX。

    用法::

        gp = Gamepad()
        if gp.connected:
            state = gp.get_state()
            print(state.lx, state.ly)
        gp.close()
    """

    def __init__(self, device: str = "/dev/input/js0", deadzone: float = _DEADZONE):
        self._device   = device
        self._deadzone = deadzone
        self._fd: Optional[int] = None
        self._lock     = threading.Lock()
        self._state    = GamepadState(connected=False)
        self._running  = False
        self._thread: Optional[threading.Thread] = None
        self._open()

    # ------------------------------------------------------------------
    # public API
    # ------------------------------------------------------------------

    @property
    def connected(self) -> bool:
        return self._state.connected

    def get_state(self) -> GamepadState:
        """返回当前状态的线程安全快照。"""
        with self._lock:
            import copy
            return copy.copy(self._state)

    def close(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        if self._fd is not None:
            try:
                os.close(self._fd)
            except OSError:
                pass
            self._fd = None

    # ------------------------------------------------------------------
    # internal
    # ------------------------------------------------------------------

    def _open(self):
        if not os.path.exists(self._device):
            print(f"[Gamepad] 未找到设备 {self._device}，请检查 xpad 模块是否加载")
            return
        try:
            self._fd = os.open(self._device, os.O_RDONLY | os.O_NONBLOCK)
        except OSError as e:
            print(f"[Gamepad] 打开 {self._device} 失败: {e}")
            return
        with self._lock:
            self._state = GamepadState(connected=True)
        self._running = True
        self._thread = threading.Thread(target=self._read_loop,
                                        name="gamepad-read", daemon=True)
        self._thread.start()
        print(f"[Gamepad] 已连接 {self._device}")

    def _read_loop(self):
        import select
        while self._running and self._fd is not None:
            try:
                r, _, _ = select.select([self._fd], [], [], 0.02)
                if not r:
                    continue
                data = os.read(self._fd, _JS_EVENT_SIZE * 16)
            except OSError:
                with self._lock:
                    self._state.connected = False
                break

            for i in range(0, len(data), _JS_EVENT_SIZE):
                chunk = data[i:i + _JS_EVENT_SIZE]
                if len(chunk) < _JS_EVENT_SIZE:
                    break
                _, val, typ, num = struct.unpack(_JS_EVENT_FMT, chunk)
                typ_clean = typ & ~_JS_EVENT_INIT
                self._dispatch(typ_clean, num, val)

    def _dispatch(self, typ: int, num: int, val: int):
        with self._lock:
            s = self._state
            if typ == _JS_EVENT_AXIS:
                if num < len(s.raw_axes):
                    s.raw_axes[num] = val
                if num == 0:
                    s.lx = _normalize_axis(val, self._deadzone)
                elif num == 1:
                    s.ly = _normalize_axis(val, self._deadzone)  # 驱动已取反
                elif num == 2:
                    s.lt = _normalize_trigger(val)
                elif num == 3:
                    s.rx = _normalize_axis(val, self._deadzone)
                elif num == 4:
                    s.ry = _normalize_axis(val, self._deadzone)
                elif num == 5:
                    s.rt = _normalize_trigger(val)
                elif num == 6:
                    # D-pad X hat: -1=左, 0=中, +1=右
                    s.dpad_left  = (val < 0)
                    s.dpad_right = (val > 0)
                elif num == 7:
                    # D-pad Y hat: -1=上, 0=中, +1=下
                    s.dpad_up   = (val < 0)
                    s.dpad_down = (val > 0)
            elif typ == _JS_EVENT_BUTTON:
                pressed = bool(val)
                _map = {
                    0: 'a',  1: 'b',  2: 'x',  3: 'y',
                    4: 'lb', 5: 'rb',
                    6: 'select', 7: 'start',
                    9: 'ls',     10: 'rs',
                }
                if num in _map:
                    setattr(s, _map[num], pressed)


# ---------------------------------------------------------------------------
# 快速测试
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    import time

    gp = Gamepad()
    if not gp.connected:
        print("手柄未连接，退出")
        raise SystemExit(1)

    print("读取 10 秒数据，请拨动摇杆/按键…")
    try:
        t0 = time.monotonic()
        while time.monotonic() - t0 < 10.0:
            st = gp.get_state()
            print(
                f"\rLX={st.lx:+.2f} LY={st.ly:+.2f}  "
                f"RX={st.rx:+.2f} RY={st.ry:+.2f}  "
                f"LT={st.lt:.2f} RT={st.rt:.2f}  "
                f"A={int(st.a)} B={int(st.b)} X={int(st.x)} Y={int(st.y)}  "
                f"LB={int(st.lb)} RB={int(st.rb)} "
                f"SEL={int(st.select)} STA={int(st.start)}  "
                f"▲={int(st.dpad_up)} ▼={int(st.dpad_down)} "
                f"◀={int(st.dpad_left)} ▶={int(st.dpad_right)}",
                end="", flush=True
            )
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        print()
        gp.close()
