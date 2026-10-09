#!/usr/bin/env python3
"""BSHSQ m1_scale 体数扫描（headless，固定 60 Hz）。
用法: python3 sweep_scale.py <bin> <threads> <ticks> <iters> <warmup> N1 N2 ...
输出: JSON（每个 N 一条），per-tick 墙钟取自 m1_scale 的 stderr 逐 tick 行。
"""
import json, math, re, statistics, subprocess, sys

TICK_RE = re.compile(r"^tick\s+(\d+):\s+([0-9.]+) ms")


def pct(xs, p):
    xs = sorted(xs)
    k = (len(xs) - 1) * p / 100.0
    lo, hi = math.floor(k), math.ceil(k)
    return xs[lo] if lo == hi else xs[lo] + (xs[hi] - xs[lo]) * (k - lo)


def run(binp, threads, n_dyn, ticks, iters, warmup):
    side = int(math.isqrt(n_dyn)) + 1
    n_static = (side + 2) ** 2  # 地面瓦片覆盖动态网格
    p = subprocess.run([binp, str(threads), str(n_static), str(n_dyn), str(ticks), str(iters)],
                       capture_output=True, text=True)
    per = {}
    for line in p.stderr.splitlines():
        m = TICK_RE.match(line)
        if m:
            per[int(m.group(1))] = float(m.group(2))
    meas = [per[t] for t in sorted(per) if t > warmup]
    summ = {}
    for line in p.stdout.splitlines():
        if line.startswith("平均"):
            summ["summary_line"] = line.strip()
        if line.startswith("活跃期"):
            summ["active_line"] = line.strip()
            m = re.search(r"(\d+) / (\d+) tick", line)
            if m:
                summ["active_ticks"] = int(m.group(1))
        if "wake_flips=" in line:
            summ["wake_flips"] = int(re.search(r"wake_flips=(\d+)", line).group(1))
        m = re.search(r"NaN (\d+) deep (\d+)$", line.strip())
        if line.startswith("平均") and m:
            summ["nan_final"], summ["deep_final"] = int(m.group(1)), int(m.group(2))
    mean = statistics.fmean(meas)
    act_n = summ.get("active_ticks", 0)
    act = [per[t] for t in sorted(per) if t <= act_n] or [0.0]
    act_mean = statistics.fmean(act)
    return {
        "threads": threads, "n_dynamic": n_dyn, "n_static": n_static, "ticks": ticks,
        "warmup_ticks": warmup, "measured_ticks": len(meas), "iters": iters, "rc": p.returncode,
        "ms_per_tick": {
            "mean": round(mean, 4), "p50": round(pct(meas, 50), 4), "p95": round(pct(meas, 95), 4),
            "p99": round(pct(meas, 99), 4), "max": round(max(meas), 4),
            "std": round(statistics.pstdev(meas), 4),
        },
        "active_window_ms": {
            "ticks": len(act), "mean": round(act_mean, 4), "p50": round(pct(act, 50), 4),
            "p95": round(pct(act, 95), 4), "p99": round(pct(act, 99), 4), "max": round(max(act), 4),
            "std": round(statistics.pstdev(act), 4),
        },
        "active_steps_per_s": round(1000.0 / act_mean, 1) if act_mean > 0 else None,
        "steps_per_s_mean": round(1000.0 / mean, 1) if mean > 0 else None,
        "p99_over_p50": round(pct(meas, 99) / pct(meas, 50), 2) if pct(meas, 50) > 0 else None,
        **summ,
    }


if __name__ == "__main__":
    binp, threads, ticks, iters, warmup = sys.argv[1], int(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4]), int(sys.argv[5])
    out = [run(binp, threads, int(n), ticks, iters, warmup) for n in sys.argv[6:]]
    print(json.dumps(out, ensure_ascii=False, indent=1))
