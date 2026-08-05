#!/usr/bin/env python3
"""Marsdog CSV software oscilloscope.

This tool is deliberately out-of-band: it only reads walk_log_*.csv and renders
an auto-refreshing HTML/SVG page. It never touches motors or the realtime loop.
"""

from __future__ import annotations

import argparse
import csv
import html
import io
import math
import os
import time
from pathlib import Path

DEFAULT_LOG_DIR = Path(__file__).resolve().parent / "log"
DEFAULT_MOTORS = (3, 7)
DEFAULT_TAIL_MB = 32.0


def parse_motors(text: str) -> tuple[int, ...]:
    ids = tuple(int(x.strip(), 0) for x in text.split(",") if x.strip())
    if not ids:
        raise argparse.ArgumentTypeError("empty motor list")
    return ids


def finite(value: str | None) -> float:
    try:
        out = float(value) if value is not None else float("nan")
    except (TypeError, ValueError):
        return float("nan")
    return out if math.isfinite(out) else float("nan")


def latest_log(log_dir: Path) -> Path:
    from marsdog_control.io.logging import WALK_RECORDER_CSV

    fixed = log_dir / WALK_RECORDER_CSV
    if fixed.is_file():
        return fixed
    logs = sorted(log_dir.glob("walk_log_*.csv"), key=lambda p: p.stat().st_mtime)
    if not logs:
        raise FileNotFoundError(
            f"{log_dir} 无 {WALK_RECORDER_CSV} 也无 walk_log_*.csv")
    return logs[-1]


def iter_csv_tail(path: Path, tail_bytes: int):
    """Yield DictReader rows from a tail chunk plus the CSV header."""
    with path.open("rb") as fh:
        header = fh.readline().decode("utf-8", errors="replace")
        fh.seek(0, os.SEEK_END)
        size = fh.tell()
        if tail_bytes <= 0 or size <= tail_bytes:
            fh.seek(0)
            text = fh.read().decode("utf-8", errors="replace")
        else:
            fh.seek(max(0, size - tail_bytes))
            chunk = fh.read().decode("utf-8", errors="replace")
            chunk = chunk.split("\n", 1)[1] if "\n" in chunk else ""
            text = header + chunk
    return csv.DictReader(io.StringIO(text))


def read_rows(path: Path, motors: tuple[int, ...], gait_only: bool,
              window_s: float | None = None, tail_bytes: int = int(DEFAULT_TAIL_MB * 1024 * 1024)):
    data = {mid: {"name": f"ID{mid}", "rows": []} for mid in motors}
    latest_t = float("-inf")
    for row in iter_csv_tail(path, tail_bytes):
        if gait_only and row.get("gait_active") != "1":
            continue
        try:
            mid = int(row.get("motor_id", ""))
        except ValueError:
            continue
        if mid not in data:
            continue
        data[mid]["name"] = row.get("name") or data[mid]["name"]
        item = {
            "t": finite(row.get("run_t_s")),
            "target": finite(row.get("target_deg")),
            "actual": finite(row.get("actual_deg")),
            "error": finite(row.get("error_deg")),
            "torque": finite(row.get("torque_nm")),
        }
        if math.isfinite(item["t"]):
            latest_t = max(latest_t, item["t"])
            data[mid]["rows"].append(item)
            if window_s is not None and math.isfinite(latest_t):
                cutoff = latest_t - window_s
                for ch in data.values():
                    while ch["rows"] and ch["rows"][0]["t"] < cutoff:
                        ch["rows"].pop(0)
    return data


def crop_window(data, window_s: float):
    times = [r["t"] for ch in data.values() for r in ch["rows"]]
    if not times:
        return data, 0.0, window_s
    t1 = max(times)
    t0 = max(min(times), t1 - window_s)
    out = {}
    for mid, ch in data.items():
        out[mid] = {
            "name": ch["name"],
            "rows": [r for r in ch["rows"] if t0 <= r["t"] <= t1],
        }
    return out, t0, t1


def rng(vals, symmetric=False, minimum=1.0):
    vals = [v for v in vals if math.isfinite(v)]
    if not vals:
        return -minimum, minimum
    if symmetric:
        b = max(minimum, max(abs(v) for v in vals) * 1.08)
        return -b, b
    lo, hi = min(vals), max(vals)
    span = max(minimum, hi - lo)
    mid = (lo + hi) / 2
    return mid - span * 0.6, mid + span * 0.6


def stat(rows):
    errs = [r["error"] for r in rows if math.isfinite(r["error"])]
    torques = [abs(r["torque"]) for r in rows if math.isfinite(r["torque"])]
    if not errs:
        return "no data"
    abs_err = sorted(abs(x) for x in errs)
    rms = math.sqrt(sum(x * x for x in errs) / len(errs))
    p95 = abs_err[int(0.95 * (len(abs_err) - 1))]
    return (f"n={len(errs)} RMS={rms:.2f}deg p95={p95:.2f}deg "
            f"max={abs_err[-1]:.2f}deg |torque|max={max(torques, default=float('nan')):.2f}Nm")


def render(data, source: Path, output: Path, *, t0: float, t1: float,
           window_s: float, refresh_s: float, title: str) -> str:
    width, left, right = 1500, 78, 34
    plot_w = width - left - right
    row_h, angle_h, error_h, torque_h = 330, 205, 92, 70
    height = 76 + len(data) * row_h + 64
    colors = ["#1f77b4", "#d62728", "#2ca02c", "#9467bd"]
    err_lo, err_hi = rng(
        [r["error"] for ch in data.values() for r in ch["rows"]],
        symmetric=True, minimum=5.0)
    span = max(1e-9, t1 - t0)

    def sx(t): return left + (t - t0) / span * plot_w
    def sy(v, lo, hi, top, h): return top + (hi - v) / max(1e-9, hi - lo) * h

    def points(rows, key, lo, hi, top, h):
        if len(rows) > 3000:
            rows = rows[::max(1, len(rows) // 3000)]
        return " ".join(
            f"{sx(r['t']):.1f},{sy(r[key], lo, hi, top, h):.1f}"
            for r in rows if math.isfinite(r[key]))

    def grid(top, h, lo, hi, label, xlabels):
        lines = [f'<rect x="{left}" y="{top}" width="{plot_w}" height="{h}" fill="white" stroke="#c9c9c9"/>']
        for i in range(5):
            y = top + i * h / 4
            val = hi - i * (hi - lo) / 4
            lines.append(f'<line x1="{left}" y1="{y:.1f}" x2="{left + plot_w}" y2="{y:.1f}" stroke="#e8e8e8"/>')
            lines.append(f'<text x="{left - 8}" y="{y + 4:.1f}" text-anchor="end" font-size="12" fill="#555">{val:.1f}</text>')
        for i in range(7):
            x = left + i * plot_w / 6
            tv = t0 + i * (t1 - t0) / 6
            lines.append(f'<line x1="{x:.1f}" y1="{top}" x2="{x:.1f}" y2="{top + h}" stroke="#f0f0f0"/>')
            if xlabels:
                lines.append(f'<text x="{x:.1f}" y="{top + h + 20}" text-anchor="middle" font-size="12" fill="#555">{tv:.2f}</text>')
        lines.append(f'<text x="22" y="{top + h / 2:.1f}" transform="rotate(-90 22,{top + h / 2:.1f})" text-anchor="middle" font-size="12">{html.escape(label)}</text>')
        return lines

    svg = [
        f'<svg width="{width}" height="{height}" viewBox="0 0 {width} {height}" xmlns="http://www.w3.org/2000/svg">',
        '<rect width="100%" height="100%" fill="#fafafa"/>',
        '<style>text{font-family:Arial,Helvetica,sans-serif}.small{font-size:12px;fill:#555}.label{font-size:13px;fill:#222}</style>',
        f'<text x="{left}" y="30" font-size="22" font-weight="700">{html.escape(title)}</text>',
        f'<text x="{left}" y="52" class="small">source={html.escape(str(source))} | window={window_s:.1f}s | run_t_s={t0:.2f}-{t1:.2f}s | refresh={refresh_s:.1f}s</text>',
    ]
    for idx, (mid, ch) in enumerate(data.items()):
        rows = ch["rows"]
        color = colors[idx % len(colors)]
        top = 76 + idx * row_h
        e_top = top + angle_h + 18
        tq_top = e_top + error_h + 18
        a_lo, a_hi = rng([v for r in rows for v in (r["target"], r["actual"])], minimum=8.0)
        tq_lo, tq_hi = rng([r["torque"] for r in rows], symmetric=True, minimum=2.0)
        svg.append(f'<text x="{left}" y="{top - 10}" font-size="17" font-weight="700">ID{mid} {html.escape(ch["name"])}</text>')
        svg.append(f'<text x="{width - 600}" y="{top - 10}" class="label">{html.escape(stat(rows))}</text>')
        svg.extend(grid(top, angle_h, a_lo, a_hi, "angle (deg)", False))
        svg.append(f'<polyline fill="none" stroke="{color}" stroke-width="2" points="{points(rows, "target", a_lo, a_hi, top, angle_h)}"/>')
        svg.append(f'<polyline fill="none" stroke="{color}" stroke-width="1.6" stroke-dasharray="6 4" opacity="0.82" points="{points(rows, "actual", a_lo, a_hi, top, angle_h)}"/>')
        svg.extend(grid(e_top, error_h, err_lo, err_hi, "error (deg)", False))
        svg.append(f'<line x1="{left}" y1="{sy(0, err_lo, err_hi, e_top, error_h):.1f}" x2="{left + plot_w}" y2="{sy(0, err_lo, err_hi, e_top, error_h):.1f}" stroke="#222" opacity="0.55"/>')
        svg.append(f'<polyline fill="none" stroke="{color}" stroke-width="1.8" points="{points(rows, "error", err_lo, err_hi, e_top, error_h)}"/>')
        svg.extend(grid(tq_top, torque_h, tq_lo, tq_hi, "torque (Nm)", idx == len(data) - 1))
        svg.append(f'<polyline fill="none" stroke="{color}" stroke-width="1.3" points="{points(rows, "torque", tq_lo, tq_hi, tq_top, torque_h)}"/>')
    svg.append("</svg>")
    refresh = f'<meta http-equiv="refresh" content="{refresh_s:.1f}">' if refresh_s > 0 else ""
    return f'<!doctype html><html><head><meta charset="utf-8">{refresh}<title>{html.escape(title)}</title></head><body style="margin:0;background:#fafafa"><div style="padding:8px 12px;font:13px Arial">软件示波器输出: <code>{html.escape(str(output))}</code></div>{"".join(svg)}</body></html>'


def build(args):
    log_path = latest_log(args.log_dir) if args.latest else Path(args.log).resolve()
    data = read_rows(
        log_path, args.motors,
        gait_only=not args.include_stand,
        window_s=args.window,
        tail_bytes=int(args.tail_mb * 1024 * 1024),
    )
    data, t0, t1 = crop_window(data, args.window)
    out = Path(args.output).resolve() if args.output else log_path.with_name("scope_live.html")
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(render(data, log_path, out, t0=t0, t1=t1, window_s=args.window,
                          refresh_s=0.0 if args.once else args.refresh,
                          title=args.title), encoding="utf-8")
    return out


def main():
    ap = argparse.ArgumentParser(description="Marsdog CSV 软件示波器")
    ap.add_argument("log", nargs="?")
    ap.add_argument("--latest", action="store_true")
    ap.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    ap.add_argument("--motors", type=parse_motors, default=DEFAULT_MOTORS)
    ap.add_argument("--window", type=float, default=6.0)
    ap.add_argument("--refresh", type=float, default=0.5)
    ap.add_argument("--tail-mb", type=float, default=DEFAULT_TAIL_MB,
                    help="每次刷新只读取日志尾部多少MB, 默认32; 设0读取全文件")
    ap.add_argument("--output")
    ap.add_argument("--include-stand", action="store_true")
    ap.add_argument("--once", action="store_true")
    ap.add_argument("--title", default="Marsdog Software Oscilloscope")
    args = ap.parse_args()
    if not args.latest and not args.log:
        ap.error("需要日志路径或 --latest")
    while True:
        out = build(args)
        print(f"[scope] wrote {out}", flush=True)
        if args.once:
            break
        time.sleep(max(0.1, args.refresh))


if __name__ == "__main__":
    main()
