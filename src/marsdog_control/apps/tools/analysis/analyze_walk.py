#!/usr/bin/env python3
"""步态日志回归分析 — 统一指标口径 (Phase0)

用法:
    python3 analyze_walk.py <log.csv> [log2.csv ...]
    python3 analyze_walk.py                # 自动分析最新的 walk_log_*.csv

对每个日志的 trot_fwd / trot_bwd 段输出统一回归指标:
  - roll/pitch std、范围
  - gyro 峰值 (deg/s)
  - IMU dz 抖动 (2nd-diff, 蹦跳/抽腿代理量)
  - IMU dz 使用范围 (是否触顶 max_correction)
  - 每 0.1 相位分箱的 roll / dz (看步频结构)
  - loop dt / imu_age 健康度 (若日志有该列)

多个日志时并排对比 roll std / gyro / 抖动, 方便单变量前后对照。
"""
import csv
import glob
import math
import os
import statistics
import sys

REF_LEG = "fl_hip_pitch"   # 用这条腿的行作为"每控制周期一行"的锚


def _f(r, k, default=float("nan")):
    v = r.get(k, "")
    if v in ("", "nan", "None", None):
        return default
    try:
        return float(v)
    except ValueError:
        return default


def load_cycles(fn):
    """返回 {mode: [rows]}, 每行是一个控制周期(以 REF_LEG 锚定)。"""
    out = {}
    for r in csv.DictReader(open(fn)):
        if r.get("name") != REF_LEG:
            continue
        out.setdefault(r.get("mode", "?"), []).append(r)
    return out


def jitter_2nd_diff(seq):
    """|二阶差分| 均值/峰值: 高频抖动代理 (蹦跳/抽腿)。"""
    if len(seq) < 3:
        return 0.0, 0.0
    d2 = [abs(seq[i] - 2 * seq[i - 1] + seq[i - 2]) for i in range(2, len(seq))]
    return statistics.mean(d2), max(d2)


def phase_bins(rows, key, phase_key="phase_fl", nbins=10):
    bins = [[] for _ in range(nbins)]
    for r in rows:
        ph = _f(r, phase_key, 0.0) % 1.0
        b = min(nbins - 1, int(ph * nbins))
        v = _f(r, key)
        if not math.isnan(v):
            bins[b].append(v)
    return [statistics.mean(b) if b else float("nan") for b in bins]


def std(seq):
    seq = [x for x in seq if not math.isnan(x)]
    return statistics.stdev(seq) if len(seq) > 1 else 0.0


def percentile(seq, q):
    seq = sorted(x for x in seq if not math.isnan(x))
    if not seq:
        return float("nan")
    return seq[min(len(seq) - 1, max(0, math.ceil(q * len(seq)) - 1))]


def _xcorr_lag(a, b, max_lag=20):
    """返回使 corr(a[i], b[i+lag]) 绝对值最大的 lag 及该 corr。
    lag>0 表示 b 滞后 a: b 对 a 的响应延迟 lag 个采样。"""
    n = min(len(a), len(b))
    a = a[:n]; b = b[:n]
    ma = sum(a) / n; mb = sum(b) / n
    A = [x - ma for x in a]; B = [x - mb for x in b]
    da = math.sqrt(sum(x * x for x in A)) or 1e-9
    db = math.sqrt(sum(x * x for x in B)) or 1e-9
    best_lag, best = 0, 0.0
    for lag in range(-max_lag, max_lag + 1):
        s = c = 0.0
        for i in range(n):
            j = i + lag
            if 0 <= j < n:
                s += A[i] * B[j]; c += 1
        if c:
            corr = s / c / (da / math.sqrt(n)) / (db / math.sqrt(n))
            if abs(corr) > abs(best):
                best, best_lag = corr, lag
    return best_lag, best


def latency(rows):
    """估计姿态与控制输出的相关偏移；闭环内生信号不能据此推断IMU传输延迟。"""
    roll = [_f(r, "imu_roll_deg") for r in rows]
    rollc = [(_f(r, "imu_dz_fl_mm") - _f(r, "imu_dz_fr_mm")) / 2.0 for r in rows]
    pair = [(a, b) for a, b in zip(roll, rollc)
            if not math.isnan(a) and not math.isnan(b)]
    if len(pair) < 30:
        return None
    a = [p[0] for p in pair]; b = [p[1] for p in pair]
    if std(a) < 1e-6 or std(b) < 1e-6:
        return None
    lag, corr = _xcorr_lag(a, b)
    times = [_f(r, "run_t_s", _f(r, "t_s")) for r in rows]
    sample_dt = [
        (b - a) * 1000.0 for a, b in zip(times, times[1:])
        if not math.isnan(a) and not math.isnan(b) and b > a
    ]
    dt_median = statistics.median(sample_dt) if sample_dt else float("nan")
    return lag, corr, lag * dt_median


def analyze_mode(rows):
    roll = [_f(r, "imu_roll_deg") for r in rows]
    pitch = [_f(r, "imu_pitch_deg") for r in rows]
    gr = [_f(r, "imu_gyro_roll") for r in rows]
    gp = [_f(r, "imu_gyro_pitch") for r in rows]
    dz = {leg: [_f(r, f"imu_dz_{leg}_mm") for r in rows]
          for leg in ("fl", "fr", "rl", "rr")}
    jm, jx = jitter_2nd_diff([x for x in dz["fl"] if not math.isnan(x)])
    m = {
        "n": len(rows),
        "roll_std": std(roll),
        "roll_rng": (min(roll), max(roll)) if roll else (0, 0),
        "pitch_std": std(pitch),
        "pitch_mean": statistics.mean([x for x in pitch if not math.isnan(x)]) if pitch else 0,
        "gr_std": std(gr),
        "gr_peak": max((abs(x) for x in gr if not math.isnan(x)), default=0),
        "gp_peak": max((abs(x) for x in gp if not math.isnan(x)), default=0),
        "dz_jit_mean": jm,
        "dz_jit_max": jx,
        "dz_rng": {leg: (min(v), max(v)) for leg, v in dz.items()
                   if any(not math.isnan(x) for x in v)},
    }
    # loop / imu health (optional columns)
    for col, name in (
        ("dt_ms", "dt_ms"),
        ("control_period_ms", "control_period_ms"),
        ("imu_age_ms", "imu_age_ms"),
    ):
        vals = [_f(r, col) for r in rows]
        vals = [x for x in vals if not math.isnan(x)]
        if vals:
            m[name] = (statistics.mean(vals), percentile(vals, 0.95), max(vals))
    m["roll_bins"] = phase_bins(rows, "imu_roll_deg")
    m["dzfl_bins"] = phase_bins(rows, "imu_dz_fl_mm")
    for leg in ("fl", "fr"):
        target = [_f(row, f"foot_pitch_target_{leg}_deg") for row in rows]
        actual = [_f(row, f"foot_pitch_actual_{leg}_deg") for row in rows]
        pairs = [(tgt, act) for tgt, act in zip(target, actual)
                 if not math.isnan(tgt) and not math.isnan(act)]
        if pairs:
            errors = [abs(act - tgt) for tgt, act in pairs]
            m[f"foot_{leg}"] = {
                "target_min": min(tgt for tgt, _ in pairs),
                "actual_min": min(act for _, act in pairs),
                "error_p95": percentile(errors, 0.95),
                "rear_rate": 100.0 * sum(act < -100.0 for _, act in pairs) / len(pairs),
            }
    return m


def print_report(fn, data):
    print(f"\n{'=' * 64}\n{os.path.basename(fn)}\n{'=' * 64}")
    preferred = (
        "natural_fwd", "natural_bwd", "natural_trot",
        "trot_fwd", "trot_bwd", "pace_fwd", "pace_bwd", "stand",
    )
    modes = list(preferred) + sorted(set(data) - set(preferred))
    for mode in modes:
        rows = data.get(mode)
        if not rows or len(rows) < 10:
            continue
        m = analyze_mode(rows)
        print(f"\n[{mode}]  n={m['n']}")
        print(f"  roll : std={m['roll_std']:.2f}  rng=[{m['roll_rng'][0]:+.1f},{m['roll_rng'][1]:+.1f}]")
        print(f"  pitch: std={m['pitch_std']:.2f}  mean={m['pitch_mean']:+.1f}")
        print(f"  gyro : roll_std={m['gr_std']:.0f}  roll_peak={m['gr_peak']:.0f}  pitch_peak={m['gp_peak']:.0f} deg/s")
        print(f"  dz_fl: jitter(2nd-diff) mean={m['dz_jit_mean']:.2f} max={m['dz_jit_max']:.2f} mm")
        sat = []
        for leg, (lo, hi) in m["dz_rng"].items():
            sat.append(f"{leg}[{lo:+.0f},{hi:+.0f}]")
        print(f"  dz_rng(mm): {' '.join(sat)}")
        if "dt_ms" in m:
            print(f"  compute dt_ms: mean/P95/max="
                  f"{m['dt_ms'][0]:.2f}/{m['dt_ms'][1]:.2f}/{m['dt_ms'][2]:.2f}")
        if "control_period_ms" in m:
            print(f"  control period_ms: mean/P95/max="
                  f"{m['control_period_ms'][0]:.2f}/"
                  f"{m['control_period_ms'][1]:.2f}/"
                  f"{m['control_period_ms'][2]:.2f}")
        if "imu_age_ms" in m:
            print(f"  imu_age_ms: mean/P95/max="
                  f"{m['imu_age_ms'][0]:.1f}/{m['imu_age_ms'][1]:.1f}/{m['imu_age_ms'][2]:.1f}")
        for leg in ("fl", "fr"):
            foot = m.get(f"foot_{leg}")
            if foot:
                print(
                    f"  foot_{leg}: target/actual min="
                    f"{foot['target_min']:.1f}/{foot['actual_min']:.1f}° "
                    f"error P95={foot['error_p95']:.2f}° "
                    f"actual<-100°={foot['rear_rate']:.1f}%"
                )
        lat = latency(rows)
        if lat:
            lag, corr, ms = lat
            print(f"  姿态/控制互相关: roll_c 相对 roll lag={lag}样本 "
                  f"(~{ms:+.0f}ms) peak_corr={corr:+.2f}；"
                  "仅描述动态关系，不代表IMU传输延迟")
        rb = m["roll_bins"]
        print("  roll/相位: " + " ".join(f"{v:+4.1f}" if not math.isnan(v) else "  . " for v in rb))
        db = m["dzfl_bins"]
        print("  dzfl/相位: " + " ".join(f"{v:+4.1f}" if not math.isnan(v) else "  . " for v in db))


def compare(files, datas):
    print(f"\n{'=' * 64}\n对比 (trot_fwd / trot_bwd)\n{'=' * 64}")
    hdr = f"{'log':<26} {'mode':<10} {'roll_std':>8} {'gr_peak':>8} {'dz_jit':>7}"
    print(hdr)
    for fn, data in zip(files, datas):
        for mode in ("natural_fwd", "natural_bwd", "natural_trot",
                     "trot_fwd", "trot_bwd"):
            rows = data.get(mode)
            if not rows or len(rows) < 10:
                continue
            m = analyze_mode(rows)
            print(f"{os.path.basename(fn)[:26]:<26} {mode:<10} "
                  f"{m['roll_std']:>8.2f} {m['gr_peak']:>8.0f} {m['dz_jit_mean']:>7.2f}")


def main():
    files = sys.argv[1:]
    if not files:
        files = sorted(glob.glob("walk_log_*.csv"))[-1:]
        if not files:
            print("未找到 walk_log_*.csv"); return
    datas = [load_cycles(f) for f in files]
    for fn, data in zip(files, datas):
        print_report(fn, data)
    if len(files) > 1:
        compare(files, datas)


if __name__ == "__main__":
    main()
