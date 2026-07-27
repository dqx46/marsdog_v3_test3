#!/usr/bin/env python3
"""Offline motor tracking plots for tests/Motor_test bench CSVs.

Examples (repo root):
    PYTHONPATH=src python3 tests/Motor_test/plot_tracking.py --latest --error
    PYTHONPATH=src python3 tests/Motor_test/plot_tracking.py log/bench_motor_sine_....csv \\
        --motors 3,7 --error
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

DEFAULT_LOG_DIR = Path(__file__).resolve().parent / "log"
DEFAULT_MOTORS = (3, 7, 4, 8)


def parse_motors(text: str) -> tuple[int, ...]:
    ids = tuple(int(x.strip(), 0) for x in text.split(",") if x.strip())
    if not ids:
        raise argparse.ArgumentTypeError("empty motor list")
    return ids


def finite(value) -> float:
    try:
        out = float(value) if value is not None else float("nan")
    except (TypeError, ValueError):
        return float("nan")
    return out if math.isfinite(out) else float("nan")


def latest_log(log_dir: Path) -> Path:
    logs = sorted(log_dir.glob("bench_motor_*.csv"), key=lambda p: p.stat().st_mtime)
    if not logs:
        raise FileNotFoundError(f"{log_dir} has no bench_motor_*.csv")
    return logs[-1]


def load_series(path: Path, motors: tuple[int, ...], *, gait_only: bool,
                t_min: float | None, t_max: float | None):
    """Return {mid: {name, t, target, actual, error, torque}}."""
    data = {
        mid: {
            "name": f"ID{mid}",
            "t": [],
            "target": [],
            "actual": [],
            "error": [],
            "torque": [],
        }
        for mid in motors
    }
    with path.open(newline="") as fh:
        for row in csv.DictReader(fh):
            if gait_only and row.get("gait_active") != "1":
                continue
            try:
                mid = int(row.get("motor_id", ""))
            except ValueError:
                continue
            if mid not in data:
                continue
            t = finite(row.get("run_t_s"))
            if not math.isfinite(t):
                t = finite(row.get("t_s"))
            if not math.isfinite(t):
                continue
            if t_min is not None and t < t_min:
                continue
            if t_max is not None and t > t_max:
                continue
            tgt = finite(row.get("target_deg"))
            act = finite(row.get("actual_deg"))
            if not (math.isfinite(tgt) and math.isfinite(act)):
                continue
            ch = data[mid]
            ch["name"] = row.get("name") or ch["name"]
            ch["t"].append(t)
            ch["target"].append(tgt)
            ch["actual"].append(act)
            ch["error"].append(finite(row.get("error_deg")))
            ch["torque"].append(finite(row.get("torque_nm")))
    return data


def _err_stats(errors: list[float]) -> str:
    errs = [e for e in errors if math.isfinite(e)]
    if not errs:
        return "no data"
    abs_err = sorted(abs(e) for e in errs)
    rms = math.sqrt(sum(e * e for e in errs) / len(errs))
    p95 = abs_err[int(0.95 * (len(abs_err) - 1))]
    return f"n={len(errs)}  RMS={rms:.2f}°  p95={p95:.2f}°  max={abs_err[-1]:.2f}°"


def infer_sine_label(data, motors: tuple[int, ...]) -> str:
    """Infer ±amp / period / freq from the first motor's target sine (bench CSVs
    do not store CLI period/amp). Returns a short Chinese label, or ''."""
    for mid in motors:
        t = data[mid]["t"]
        tgt = data[mid]["target"]
        if len(t) < 20:
            continue
        # Detend: target is q0 + amp*sin(...); amp ≈ half peak-to-peak.
        ymin, ymax = min(tgt), max(tgt)
        amp = 0.5 * (ymax - ymin)
        if amp < 0.05:
            continue
        mid_y = 0.5 * (ymin + ymax)
        # Rising zero-crossings of (target - mid) → period estimate.
        crosses = []
        for i in range(1, len(tgt)):
            a, b = tgt[i - 1] - mid_y, tgt[i] - mid_y
            if a < 0.0 <= b:
                # Linear interpolate crossing time.
                frac = (-a) / (b - a) if b != a else 0.0
                crosses.append(t[i - 1] + frac * (t[i] - t[i - 1]))
        if len(crosses) < 2:
            continue
        periods = [crosses[i] - crosses[i - 1] for i in range(1, len(crosses))]
        periods = [p for p in periods if 0.05 < p < 30.0]
        if not periods:
            continue
        period = sum(periods) / len(periods)
        freq = 1.0 / period
        return f"扫频 ±{amp:.1f}°  f≈{freq:.2f} Hz  (period≈{period:.2f}s)"
    return ""


def _has_matplotlib() -> bool:
    try:
        import matplotlib  # noqa: F401
        return True
    except ImportError:
        return False


def _configure_cjk_font() -> None:
    """Prefer a CJK-capable font so 扫频 labels render (Noto/AR PL on Ubuntu)."""
    from matplotlib import font_manager, rcParams

    preferred = (
        "Noto Sans CJK SC",
        "Noto Serif CJK SC",
        "AR PL UMing CN",
        "AR PL UKai CN",
        "WenQuanYi Micro Hei",
        "SimHei",
    )
    available = {f.name for f in font_manager.fontManager.ttflist}
    for name in preferred:
        if name in available:
            rcParams["font.sans-serif"] = [name, "DejaVu Sans"]
            rcParams["axes.unicode_minus"] = False
            return


def plot_tracking_mpl(data, *, motors: tuple[int, ...], show_error: bool,
                      show_torque: bool, title: str, output: Path,
                      show: bool) -> Path:
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    _configure_cjk_font()

    rows_per = 1 + int(show_error) + int(show_torque)
    n = len(motors)
    fig_h = max(3.0, 2.4 * n * (0.55 + 0.25 * rows_per))
    fig, axes = plt.subplots(n, rows_per, figsize=(12, fig_h), sharex="col",
                             squeeze=False)
    fig.suptitle(title, fontsize=13, fontweight="bold")

    for i, mid in enumerate(motors):
        ch = data[mid]
        name = ch["name"]
        t = ch["t"]
        ax0 = axes[i][0]
        if not t:
            ax0.text(0.5, 0.5, f"ID{mid} {name}: no data",
                     ha="center", va="center", transform=ax0.transAxes)
            ax0.set_ylabel(f"ID{mid}")
            continue
        ax0.plot(t, ch["target"], "--", color="#c0392b", lw=1.2, label="target")
        ax0.plot(t, ch["actual"], "-", color="#2980b9", lw=1.0, label="actual")
        ax0.set_ylabel(f"ID{mid}\n{name}\ndeg")
        ax0.grid(True, alpha=0.35)
        ax0.legend(loc="upper right", fontsize=8)
        ax0.set_title(_err_stats(ch["error"]), fontsize=9, loc="left")

        col = 1
        if show_error:
            ax_e = axes[i][col]
            ax_e.plot(t, ch["error"], color="#8e44ad", lw=0.9)
            ax_e.axhline(0.0, color="#999", lw=0.6)
            ax_e.set_ylabel("err°")
            ax_e.grid(True, alpha=0.35)
            col += 1
        if show_torque:
            ax_t = axes[i][col]
            ax_t.plot(t, ch["torque"], color="#16a085", lw=0.9)
            ax_t.set_ylabel("Nm")
            ax_t.grid(True, alpha=0.35)

    for ax in axes[-1]:
        ax.set_xlabel("run_t_s")
    # Leave room for multi-line frequency label in suptitle.
    fig.tight_layout(rect=(0, 0, 1, 0.96 if "\n" in title else 0.98))
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=130)
    print(f"[plot] matplotlib → {output.resolve()}")
    if show:
        plt.show()
    plt.close(fig)
    return output


def _polyline(xs, ys, x0, y0, w, h, t0, t1, ymin, ymax, color, dash=False):
    if not xs or t1 <= t0 or ymax <= ymin:
        return ""
    pts = []
    for x, y in zip(xs, ys):
        if not (math.isfinite(x) and math.isfinite(y)):
            continue
        px = x0 + (x - t0) / (t1 - t0) * w
        py = y0 + h - (y - ymin) / (ymax - ymin) * h
        pts.append(f"{px:.1f},{py:.1f}")
    if len(pts) < 2:
        return ""
    dash_attr = ' stroke-dasharray="6 4"' if dash else ""
    return (f'<polyline fill="none" stroke="{color}" stroke-width="1.4"'
            f'{dash_attr} points="{" ".join(pts)}"/>')


def plot_tracking_svg(data, *, motors: tuple[int, ...], show_error: bool,
                      show_torque: bool, title: str, output: Path) -> Path:
    """Stdlib SVG fallback when matplotlib is not installed."""
    width, left, right, top0 = 1200, 70, 24, 56
    panel_h = 160 + (70 if show_error else 0) + (60 if show_torque else 0)
    gap = 28
    height = top0 + 24 + len(motors) * (panel_h + gap)
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
        f'viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#fafafa"/>',
        f'<text x="{left}" y="28" font-size="18" font-weight="700" '
        f'font-family="Arial">{title}</text>',
        f'<text x="{left}" y="46" font-size="12" fill="#666" font-family="Arial">'
        f'target(red dashed) vs actual(blue) — stdlib SVG fallback '
        f'(install matplotlib for PNG)</text>',
    ]
    plot_w = width - left - right
    for i, mid in enumerate(motors):
        ch = data[mid]
        y_base = top0 + i * (panel_h + gap)
        parts.append(
            f'<text x="{left}" y="{y_base}" font-size="14" font-weight="700" '
            f'font-family="Arial">ID{mid} {ch["name"]}  {_err_stats(ch["error"])}</text>')
        t = ch["t"]
        if not t:
            parts.append(
                f'<text x="{left}" y="{y_base + 40}" font-size="12" fill="#999">'
                f'no data</text>')
            continue
        t0, t1 = min(t), max(t)
        if t1 <= t0:
            t1 = t0 + 1.0
        vals = ch["target"] + ch["actual"]
        ymin = min(vals)
        ymax = max(vals)
        pad = max(1.0, (ymax - ymin) * 0.08)
        ymin, ymax = ymin - pad, ymax + pad
        angle_h = 110
        ax_y = y_base + 10
        parts.append(
            f'<rect x="{left}" y="{ax_y}" width="{plot_w}" height="{angle_h}" '
            f'fill="#fff" stroke="#ddd"/>')
        parts.append(_polyline(
            t, ch["target"], left, ax_y, plot_w, angle_h, t0, t1, ymin, ymax,
            "#c0392b", dash=True))
        parts.append(_polyline(
            t, ch["actual"], left, ax_y, plot_w, angle_h, t0, t1, ymin, ymax,
            "#2980b9"))
        y_cursor = ax_y + angle_h + 8
        if show_error:
            eh = 55
            errs = [e for e in ch["error"] if math.isfinite(e)]
            emax = max(1.0, max(abs(e) for e in errs)) if errs else 1.0
            parts.append(
                f'<rect x="{left}" y="{y_cursor}" width="{plot_w}" height="{eh}" '
                f'fill="#fff" stroke="#ddd"/>')
            parts.append(_polyline(
                t, ch["error"], left, y_cursor, plot_w, eh, t0, t1, -emax, emax,
                "#8e44ad"))
            y_cursor += eh + 6
        if show_torque:
            th = 50
            tqs = [x for x in ch["torque"] if math.isfinite(x)]
            if tqs:
                tmin, tmax = min(tqs), max(tqs)
                if tmax <= tmin:
                    tmax = tmin + 1.0
            else:
                tmin, tmax = -1.0, 1.0
            parts.append(
                f'<rect x="{left}" y="{y_cursor}" width="{plot_w}" height="{th}" '
                f'fill="#fff" stroke="#ddd"/>')
            parts.append(_polyline(
                t, ch["torque"], left, y_cursor, plot_w, th, t0, t1, tmin, tmax,
                "#16a085"))
    parts.append("</svg>")
    if output.suffix.lower() != ".svg":
        output = output.with_suffix(".svg")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text("\n".join(parts), encoding="utf-8")
    print(f"[plot] SVG fallback → {output.resolve()}")
    print("[plot] 当前环境无 matplotlib; 有网后执行: pip3 install --user matplotlib")
    return output


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Motor_test bench 跟随曲线")
    ap.add_argument("log", nargs="?", help="bench_motor_*.csv 路径")
    ap.add_argument("--latest", action="store_true", help="使用 log/ 下最新 bench CSV")
    ap.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    ap.add_argument("--motors", type=parse_motors, default=DEFAULT_MOTORS,
                    help="逗号分隔电机 ID, 默认 3,7,4,8")
    ap.add_argument("--t-min", type=float, default=None, help="run_t_s 下限")
    ap.add_argument("--t-max", type=float, default=None, help="run_t_s 上限")
    ap.add_argument("--error", action="store_true", help="附带误差子图")
    ap.add_argument("--torque", action="store_true", help="附带力矩子图")
    ap.add_argument("--output", "-o", type=Path, default=None,
                    help="输出路径 (默认 log/<csv名>_tracking.png|.svg)")
    ap.add_argument("--show", action="store_true",
                    help="弹窗显示 (需 matplotlib + 图形界面)")
    ap.add_argument("--format", choices=("auto", "mpl", "svg"), default="auto",
                    help="auto=优先 matplotlib PNG, 否则 SVG; mpl=强制 matplotlib")
    ap.add_argument("--title", default=None,
                    help="图标题(默认: 文件名 + 从目标正弦推断的扫频标注)")
    args = ap.parse_args(argv)

    if args.latest:
        log_path = latest_log(args.log_dir)
    elif args.log:
        log_path = Path(args.log).resolve()
    else:
        ap.error("需要日志路径或 --latest")

    if not log_path.is_file():
        raise SystemExit(f"找不到日志: {log_path}")

    # Bench CSVs have no gait_active column — always plot all rows.
    data = load_series(
        log_path, args.motors,
        gait_only=False,
        t_min=args.t_min, t_max=args.t_max,
    )
    nonempty = sum(1 for mid in args.motors if data[mid]["t"])
    if nonempty == 0:
        raise SystemExit("所选电机在日志中无有效数据")

    use_mpl = args.format == "mpl" or (args.format == "auto" and _has_matplotlib())
    if args.format == "mpl" and not _has_matplotlib():
        raise SystemExit(
            "指定了 --format mpl 但未安装 matplotlib。\n"
            "请执行: pip3 install --user matplotlib\n"
            "或改用: --format svg"
        )

    out = args.output
    if out is None:
        ext = ".png" if use_mpl else ".svg"
        out = log_path.with_name(log_path.stem + "_tracking" + ext)

    if args.title:
        title = args.title
    else:
        sine_label = infer_sine_label(data, args.motors)
        title = f"Tracking: {log_path.name}"
        if sine_label:
            title = f"{sine_label}\n{title}"
    if use_mpl:
        plot_tracking_mpl(
            data, motors=args.motors, show_error=args.error,
            show_torque=args.torque, title=title, output=out, show=args.show)
    else:
        plot_tracking_svg(
            data, motors=args.motors, show_error=args.error,
            show_torque=args.torque, title=title, output=out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
