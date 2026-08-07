#!/usr/bin/env python3
"""Marsdog 日常工具菜单 — 统一入口。

整合:
  1) static_test       总线体检（不使能）
  2) go_zero           平滑回零（默认 3s，全轴含达妙/头）
  3) set_zero_selected 指定 ID 设零（默认写 Flash）
  4) sim_com_balance   CoM 站立 / 移重（默认真机 + 对角抬腿）
  5) capture_pose      捕获 sit / lie 姿态

用法:
  ./run_with_env.sh python -m marsdog_control.apps.tools.marsdog_menu
  ./marsdog_menu.sh
"""

from __future__ import annotations

import os
import shlex
import subprocess
import sys
from pathlib import Path

_REPO = Path(__file__).resolve().parents[4]
_SRC = _REPO / "src"


def _python() -> str:
    return sys.executable or "python3"


def _env() -> dict:
    env = os.environ.copy()
    src = str(_SRC)
    legacy = str(_REPO / "mocap_to_real")
    prev = env.get("PYTHONPATH", "")
    parts = [src, legacy] + ([prev] if prev else [])
    env["PYTHONPATH"] = os.pathsep.join(parts)
    env["PYTHONNOUSERSITE"] = "1"
    return env


def _run_module(module: str, argv: list[str] | None = None) -> int:
    cmd = [_python(), "-m", module, *(argv or [])]
    print("\n>>> " + " ".join(shlex.quote(c) for c in cmd) + "\n")
    try:
        return int(subprocess.call(cmd, cwd=str(_REPO), env=_env()))
    except KeyboardInterrupt:
        print("\n[menu] 已中断")
        return 130


def _ask(prompt: str, default: str = "") -> str:
    suffix = f" [{default}]" if default else ""
    try:
        raw = input(f"{prompt}{suffix}: ").strip()
    except (EOFError, KeyboardInterrupt):
        print()
        return ""
    return raw if raw else default


def _ask_yes(prompt: str, default: bool = False) -> bool:
    d = "Y/n" if default else "y/N"
    ans = _ask(f"{prompt} ({d})", "").lower()
    if not ans:
        return default
    return ans in ("y", "yes", "1")


def _menu_static_test() -> int:
    print("\n[总线体检] 不使能电机，只查通信。")
    return _run_module("marsdog_control.apps.tools.diagnostics.static_test")


def _menu_go_zero() -> int:
    print("\n[平滑回零] 渐变 3s → 全轴到 0（含达妙/头）。")
    return _run_module(
        "marsdog_control.apps.tools.calibration.go_zero",
        ["--fade", "3", "--include-head"],
    )


def _menu_set_zero_selected() -> int:
    print("\n[指定设零] 失能 → 手动摆位 → 写 Flash 零点（达妙无掉电记忆）。")
    print("常用: 前腿因克斯 2,3,6,7 | 达妙 4,8 | 单轴例如 11")
    ids = _ask("电机 ID（必填，逗号分隔）", "")
    if not ids:
        print("[menu] 未输入 ID，取消")
        return 0
    # 默认写 Flash（不演习）；工具内部仍有摆位后确认
    return _run_module(
        "marsdog_control.apps.tools.calibration.set_zero_selected",
        ["--ids", ids, "--yes"],
    )


def _menu_sim_com_balance() -> int:
    print("\n[CoM balance] 真机站立移重；kp/kd/trq_ff 与 SoftTrot(run_walk) 对齐。")
    print("  CSV → mocap_to_real/log/com_balance.csv")
    # --real --allow-lift；增益走 SoftTrot schema（leg_kp_scale + gravity）
    return _run_module(
        "marsdog_control.apps.sim.sim_com_balance",
        ["--real", "--allow-lift"],
    )


def _menu_capture_pose() -> int:
    print("\n[捕获姿态] 失能后拖动；z=sit  p=lie  r=打印  q=退出。")
    return _run_module("marsdog_control.apps.tools.calibration.capture_pose")


_ITEMS = [
    ("1", "总线体检 static_test", _menu_static_test),
    ("2", "平滑回零 go_zero", _menu_go_zero),
    ("3", "指定 ID 设零 set_zero_selected", _menu_set_zero_selected),
    ("4", "CoM 站立 sim_com_balance", _menu_sim_com_balance),
    ("5", "捕获 sit/lie capture_pose", _menu_capture_pose),
]


def main(argv: list[str] | None = None) -> int:
    _ = argv  # reserved for future non-interactive flags
    while True:
        print()
        print("=" * 56)
        print("  Marsdog 工具菜单")
        print(f"  仓库: {_REPO}")
        print("=" * 56)
        for key, title, _fn in _ITEMS:
            print(f"  [{key}] {title}")
        print("  [0] 退出")
        print("-" * 56)
        choice = _ask("选择", "").strip().lower()
        if choice in ("0", "q", "quit", "exit"):
            print("[menu] bye")
            return 0
        matched = False
        for key, _title, fn in _ITEMS:
            if choice == key:
                matched = True
                code = fn()
                print(f"\n[menu] 退出码 {code}")
                break
        if not matched:
            print("[menu] 无效选项")


if __name__ == "__main__":
    raise SystemExit(main())
