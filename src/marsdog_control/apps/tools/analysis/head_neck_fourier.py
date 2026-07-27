"""前进步态使用的头颈 Fourier 行为通道。

运行逻辑:
  1. 从同目录 ``fourier_fit.json`` 加载 ID 15/16/17/18 的 Fourier 系数。
  2. ``update`` 每个控制周期只修改 MotionTarget，不直接访问电机或 CAN。
  3. 前进时用五次平滑包络渐入周期轨迹，停止前进时非阻塞渐退到步态基准。

用法:
    channel = HeadNeckFourier()
    channel.update(motion, is_forward_walking(fsm))

所有输出均为电机帧 rad；最终关节限位仍由 walk.py 的 SafetySupervisor 统一处理。
"""

from __future__ import annotations

import json
import math
import os
import re
import time
from typing import Dict, Optional, Tuple

from marsdog_control.core.types import Direction, MotionTarget


HEAD_NECK_COLUMNS = {
    15: "head_pitch_joint",
    16: "head_yaw_joint",
    17: "head_roll_joint",
    18: "neck_pitch_joint",
}
DEFAULT_COEFFICIENTS = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "fourier_fit.json"
)


def _number(text: str) -> float:
    """兼容旧 tab 导出中形如 ``30.0JS:30`` 的数值。"""
    match = re.match(
        r"\s*([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)", text
    )
    if not match:
        raise ValueError(f"不是有效数字: {text!r}")
    return float(match.group(1))


def _normalize_json(data: dict) -> dict:
    columns = data.get("columns", {})
    normalized = {}
    for name in HEAD_NECK_COLUMNS.values():
        item = columns.get(name)
        if not isinstance(item, dict):
            raise ValueError(f"系数文件缺少 columns.{name}")
        coeffs = item.get("coefficients", item)
        normalized[name] = {
            key: float(value)
            for key, value in coeffs.items()
            if re.fullmatch(r"(?:a0|[ab]\d+)", key)
        }
    return {
        "period": float(data.get("duration_s", 0.0)),
        "order": int(data.get("order", 0)),
        "columns": normalized,
    }


def _parse_tab_export(text: str) -> dict:
    """解析旧版无括号、tab 缩进的 Fourier 系数文件。"""
    names = set(HEAD_NECK_COLUMNS.values())
    columns: Dict[str, Dict[str, float]] = {}
    metadata = {}
    current = None

    for raw_line in text.splitlines():
        fields = raw_line.strip().split(None, 1)
        if not fields:
            continue
        key = fields[0]
        value = fields[1] if len(fields) == 2 else ""
        if key in names:
            current = key
            columns[current] = {}
        elif current is not None and re.fullmatch(r"(?:a0|[ab]\d+)", key):
            columns[current][key] = _number(value)
        elif key in {"duration_s", "order"}:
            metadata[key] = _number(value)

    missing = names.difference(columns)
    if missing:
        raise ValueError(f"系数文件缺少关节: {sorted(missing)}")
    return {
        "period": float(metadata.get("duration_s", 0.0)),
        "order": int(metadata.get("order", 0)),
        "columns": columns,
    }


def load_fourier(path: str = DEFAULT_COEFFICIENTS) -> dict:
    """加载标准 JSON 或旧 tab 导出格式的 Fourier 系数。"""
    with open(path, "r", encoding="utf-8") as stream:
        text = stream.read()
    try:
        model = _normalize_json(json.loads(text))
    except json.JSONDecodeError:
        model = _parse_tab_export(text)

    if model["period"] <= 0.0:
        raise ValueError("Fourier 周期必须大于 0")
    if model["order"] <= 0:
        raise ValueError("Fourier 阶数必须大于 0")
    return model


def evaluate_fourier(
    coefficients: Dict[str, float],
    t: float,
    period: float,
    order: int,
    amplitude_scale: float = 1.0,
) -> Tuple[float, float]:
    """计算关节位置 q(rad) 与解析速度 dq(rad/s)。"""
    omega = 2.0 * math.pi / period
    q = coefficients["a0"]
    dq = 0.0
    for k in range(1, order + 1):
        ak = coefficients.get(f"a{k}", 0.0)
        bk = coefficients.get(f"b{k}", 0.0)
        phase = k * omega * t
        q += amplitude_scale * (
            ak * math.cos(phase) + bk * math.sin(phase)
        )
        dq += amplitude_scale * k * omega * (
            -ak * math.sin(phase) + bk * math.cos(phase)
        )
    return q, dq


def is_forward_walking(fsm, throttle_threshold: float = 0.15) -> bool:
    """仅识别有实际前进油门的步态，排除站立、后退和原地转向。"""
    return (
        fsm.active_gait is not None
        and fsm.direction is Direction.FWD
        and fsm.throttle > throttle_threshold
    )


def _smoothstep5(u: float) -> float:
    u = max(0.0, min(1.0, u))
    return 10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5


class HeadNeckFourier:
    """无硬件依赖、可在实时主循环逐周期调用的头颈行为通道。"""

    def __init__(
        self,
        coefficients_path: str = DEFAULT_COEFFICIENTS,
        *,
        amplitude_scale: float = 1.0,
        ramp_time: float = 2.0,
        period: Optional[float] = None,
    ):
        if amplitude_scale < 0.0:
            raise ValueError("amplitude_scale 不能为负数")
        if ramp_time < 0.0:
            raise ValueError("ramp_time 不能为负数")

        self.model = load_fourier(coefficients_path)
        self.period = self.model["period"] if period is None else float(period)
        if self.period <= 0.0:
            raise ValueError("period 必须大于 0")
        self.amplitude_scale = float(amplitude_scale)
        self.ramp_time = float(ramp_time)
        self._envelope = 0.0
        self._phase_time = 0.0
        self._last_time: Optional[float] = None

    def update(
        self,
        motion: MotionTarget,
        active: bool,
        now: Optional[float] = None,
    ) -> None:
        """原地叠加头颈目标；单次调用无阻塞，适用于 200 Hz 控制循环。

        q_cmd = q_base + alpha(t) * (q_fourier - q_base)，其中 alpha 使用五次
        smoothstep。退出前进后 alpha 渐退至零，此后不再覆盖规划器的头颈目标。
        """
        current_time = time.monotonic() if now is None else float(now)
        dt = (
            0.0
            if self._last_time is None
            else max(0.0, current_time - self._last_time)
        )
        self._last_time = current_time

        if self.ramp_time <= 0.0:
            self._envelope = 1.0 if active else 0.0
        else:
            step = dt / self.ramp_time
            if active:
                self._envelope = min(1.0, self._envelope + step)
            else:
                self._envelope = max(0.0, self._envelope - step)

        if self._envelope <= 0.0:
            self._phase_time = 0.0
            return

        self._phase_time = (self._phase_time + dt) % self.period
        blend = _smoothstep5(self._envelope)
        for motor_id, column in HEAD_NECK_COLUMNS.items():
            # 离线关节不会出现在 motion.q 中，避免行为通道重新注入无效电机。
            if motor_id not in motion.q:
                continue
            target, _ = evaluate_fourier(
                self.model["columns"][column],
                self._phase_time,
                self.period,
                self.model["order"],
                self.amplitude_scale,
            )
            base = motion.q[motor_id]
            motion.q[motor_id] = base + blend * (target - base)

