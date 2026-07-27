"""Software-scope subprocess helper (spawns ``scope_walk.py`` over the log)."""

from __future__ import annotations

import os
import subprocess
import sys
from typing import Optional


def start_scope(log_path: Optional[str], args, *, resource_dir: str):
    """Launch the live scope over ``log_path``; return the Popen or None.

    Returns None (with an explanatory print) when scope is disabled, logging is
    off, or the scope script is missing.
    """
    if not getattr(args, "scope", False):
        return None
    if not log_path:
        print("[scope] --scope 需要日志, 当前 --no-log 生效, 已跳过示波器")
        return None
    script = os.path.join(resource_dir, "scope_walk.py")
    if not os.path.exists(script):
        print(f"[scope] 找不到 {script}, 已跳过示波器")
        return None
    out_path = os.path.join(os.path.dirname(log_path), "scope_live.html")
    cmd = [
        sys.executable, script, log_path,
        "--window", str(getattr(args, "scope_window", 6.0)),
        "--refresh", str(getattr(args, "scope_refresh", 0.5)),
        "--motors", str(getattr(args, "scope_motors", "3,7")),
        "--output", out_path,
    ]
    try:
        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
    except OSError as exc:
        print(f"[scope] 启动失败: {exc}")
        return None
    print(f"[scope] 软件示波器已启动: {out_path}")
    return proc


def stop_scope(proc) -> None:
    """Terminate a running scope subprocess (best-effort)."""
    if proc is None or proc.poll() is not None:
        return
    try:
        proc.terminate()
        proc.wait(timeout=1.0)
    except Exception:
        try:
            proc.kill()
        except Exception:
            pass


__all__ = ["start_scope", "stop_scope"]
