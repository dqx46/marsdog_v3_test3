"""Per-robot roll-trim / phase-feedforward calibration I/O.

量产: 一机一份 ``trim_cal.json``, 开机即调平, 零瞬态。读写都做成纯函数
(路径显式传入), 便于离线单测与 App 壳解耦。
"""

from __future__ import annotations

import datetime
import json
from typing import Optional, Sequence


def load_trim_cal(path: str) -> Optional[dict]:
    """加载 roll 配平/相位前馈标定。

    返回 dict{'roll_ff_mm':[...], 'roll_trim_mm':float} 或 None(缺失/损坏)。
    """
    try:
        with open(path) as fh:
            return json.load(fh)
    except Exception:
        return None


def save_trim_cal(path: str, roll_ff_mm: Sequence[float],
                  roll_trim_mm: float = 0.0) -> bool:
    """保存学到的相位前馈表 + 手动配平, 供下次开机加载。"""
    try:
        with open(path, "w") as fh:
            json.dump({
                "roll_ff_mm": [round(x, 3) for x in roll_ff_mm],
                "roll_trim_mm": round(roll_trim_mm, 3),
                "phases": len(roll_ff_mm),
                "saved": datetime.datetime.now().isoformat(timespec="seconds"),
            }, fh)
        return True
    except Exception as e:  # noqa: BLE001
        print(f"[AT] 配平保存失败: {e}")
        return False


__all__ = ["load_trim_cal", "save_trim_cal"]
