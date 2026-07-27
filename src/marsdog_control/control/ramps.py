"""Time-based control ramps (pure math, no hardware).

These are the smoothstep ramps the main loop uses to fade authority in without
"起飞": IMU-feedback soft-start, trim/feed-forward soft-start, and the
log-only gait ramp fraction. They are deterministic functions of
``elapsed_s / duration_s`` and are fully unit-testable offline.
"""

from __future__ import annotations


def smoothstep(x: float) -> float:
    """标准 smoothstep 3x²-2x³, 定义域外不做钳制(调用方负责钳制 x∈[0,1])。"""
    return 3.0 * x * x - 2.0 * x * x * x


def softstart_gain(elapsed_s: float, duration_s: float) -> float:
    """把 0→1 的权限在 ``duration_s`` 内按 smoothstep 平滑拉起。

    - ``duration_s <= 0``: 视为关闭软启动, 直接返回 1.0(全权限)。
    - ``elapsed_s >= duration_s``: 已完成, 返回 1.0。
    - 其余: 返回 smoothstep(clamp(elapsed/duration, 0, 1))。

    与 walk 主循环里 ss_gain/trim_gain 的内联公式逐字节等价, 单一真源。
    """
    if duration_s <= 1e-6:
        return 1.0
    x = elapsed_s / duration_s
    if x >= 1.0:
        return 1.0
    if x < 0.0:
        x = 0.0
    return smoothstep(x)


__all__ = ["smoothstep", "softstart_gain"]
