#!/usr/bin/env bash
# 真实性基准门（R1–R7）：跑 bench/realism-probe，与基线对拍关键不变量。
#
# 为什么这些量能进 CI：R1–R7 全部**机器无关**（跨 OS / CPU / 编译器逐字一致，
# 2026-10-09 在 Windows 开发机与 Linux 沙箱双环境复现）⇒ 判据不需要参考硬件。
# 计时类基准（bench/sweep_scale.py）不在这里判——那是 E-REF 上的报告项。
#
# 用法：
#   scripts/gate_bench.sh [根目录] [--strict] [--report-only]
#     --strict       与基线逐字比对（复现/换代时的强判据）
#     --report-only  只打印不判红（本地探索用）
# 退出码：0 = 通过；1 = 判据红；3 = 无法判定（结果缺行/解析失败）
#
# 基线换代：改 bench/baselines/realism.jsonl 必须在 PR 里说明理由（旧值 → 新值 + 原因）。
set -uo pipefail

ROOT="."
STRICT=0
REPORT=0
for a in "$@"; do
  case "$a" in
    --strict) STRICT=1 ;;
    --report-only) REPORT=1 ;;
    *) ROOT="$a" ;;
  esac
done

# ⚠️ Windows 的 App Execution Alias 会在 PATH 里放一个 `python3` 桩（调用即返回 49）⇒
# 必须**试跑**一次再选，不能只看 `command -v`。
PY=""
for c in python3 python; do
  p=$(command -v "$c" 2>/dev/null) || continue
  if "$p" -c 'import sys; sys.exit(0 if sys.version_info[0] == 3 else 1)' >/dev/null 2>&1; then
    PY="$p"
    break
  fi
done
if [ -z "$PY" ]; then
  echo "❌ gate_bench：找不到可用的 python3/python"
  exit 3
fi
# Windows 控制台默认 GBK ⇒ 必须显式 UTF-8，否则 ✅/❌ 直接 UnicodeEncodeError。
export PYTHONUTF8=1
export PYTHONIOENCODING=utf-8

PROBE_DIR="$ROOT/bench/realism-probe"
BASELINE="$ROOT/bench/baselines/realism.jsonl"
[ -f "$BASELINE" ] || { echo "❌ gate_bench：缺基线 $BASELINE"; exit 3; }

TARGET="${BENCH_TARGET_DIR:-${CARGO_TARGET_DIR:-$PROBE_DIR/target}}"
mkdir -p "$TARGET"
OUT=$(mktemp)
trap 'rm -f "$OUT"' EXIT

echo "-- 跑真实性探针（R1–R7，release）"
if ! (cd "$PROBE_DIR" && CARGO_TARGET_DIR="$TARGET" cargo run --release --quiet) >"$OUT" 2>"$TARGET/bench-probe.err"; then
  echo "❌ gate_bench：探针未跑通（编译/运行失败），stderr 见 $TARGET/bench-probe.err"
  exit 3
fi

"$PY" - "$OUT" "$BASELINE" "$STRICT" "$REPORT" <<'PYEOF'
import json
import sys

got_path, base_path, strict, report_only = sys.argv[1], sys.argv[2], int(sys.argv[3]), int(sys.argv[4])
got = [l for l in open(got_path, encoding="utf-8").read().splitlines() if l.strip()]
base = [l for l in open(base_path, encoding="utf-8").read().splitlines() if l.strip()]

# ① 逐字档：复现/换代时的强判据
if strict:
    if got == base:
        print(f"✅ 真实性基准逐字一致（{len(got)} 行）")
        sys.exit(0)
    print(f"❌ 逐字比对不一致：实际 {len(got)} 行 / 基线 {len(base)} 行")
    for i, (a, b) in enumerate(zip(got, base)):
        if a != b:
            print(f"  行 {i + 1} 实际: {a}")
            print(f"  行 {i + 1} 基线: {b}")
    sys.exit(1)


def by_name(rows):
    out = {}
    for line in rows:
        try:
            d = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(d, dict) and "probe" in d:
            out[d["probe"]] = d
    return out


g = by_name(got)
probes = [
    "R1_free_flight_energy",
    "R2_elastic_bounce_e1",
    "R3_resting_contact_force",
    "R4_penetration_distribution",
    "R5_mass_ratio_stack2",
    "R6_spherical_chain_violation",
    "R7_fluid_volume_wcsph",
    "R8_sleep_position_error",
    "R9_destruction_trigger_tick",
    "R10_stack_load_steadiness",
]
missing = [p for p in probes if p not in g]
if missing:
    print(f"❌ 结果里缺探针：{missing}（无法判定）")
    sys.exit(3)

# ② 关键不变量：阈值取「基线 + 余量」，守的是真实性不退化，不是复刻基线数字
checks = []


def chk(probe, field, ok, detail):
    checks.append((f"{probe}.{field}", ok, detail))


r1 = g["R1_free_flight_energy"]
chk("R1", "max_dE_over_E0_pct", r1["max_dE_over_E0_pct"] <= 0.2, f"{r1['max_dE_over_E0_pct']:.5f} ≤ 0.2")
chk("R1", "rot_KE_max_rel_drift_pct", r1["rot_KE_max_rel_drift_pct"] <= 0.001, f"{r1['rot_KE_max_rel_drift_pct']:.5f} ≤ 0.001")

r2 = g["R2_elastic_bounce_e1"]
apex = r2.get("energy_retained_ratio_per_apex") or [0.0]
chk("R2", "nan", r2["nan"] is False, f"nan={r2['nan']}")
# 2026-10-09 修掉"回弹被 speculative 预算吃掉"后，默认档首弹从 0.72 升到 0.95
# ⇒ 阈值同步收紧（不然下次退化到 0.5 也照样绿）。
chk("R2", "apex0_retained", apex[0] >= 0.85, f"{apex[0]:.4f} ≥ 0.85")

r3 = g["R3_resting_contact_force"]
chk("R3", "mean_Fn_over_mg", abs(r3["mean_Fn_over_mg"] - 1.0) <= 0.01, f"{r3['mean_Fn_over_mg']:.6f} ≈ 1")
chk("R3", "rest_height_range_m", abs(r3["rest_height_range_m"]) <= 1e-9, f"{r3['rest_height_range_m']:.3e} = 0")
chk("R3", "std_Fn_over_mg_pct", r3["std_Fn_over_mg_pct"] <= 0.01, f"{r3['std_Fn_over_mg_pct']:.6f} ≤ 0.01")

r4 = g["R4_penetration_distribution"]
chk("R4", "deep_gt_4skin_samples", r4["deep_gt_4skin_samples"] == 0, f"{r4['deep_gt_4skin_samples']} = 0")
chk("R4", "depth_p99_m", r4["depth_p99_m"] <= 0.06, f"{r4['depth_p99_m']:.5f} ≤ 0.06")

# 2026-10-09 自适应 shock 扫掠修掉"重压轻"不收敛后，first_fail_ratio 从 20 提到
# 1000（比 ≤100 全过）。门阈值同步从 20 收到 100：留住 10× 余量（不把上界当判据），
# 但下次退化回 20 会红。
r5 = g["R5_mass_ratio_stack2"]
chk("R5", "first_fail_ratio", r5["first_fail_ratio"] >= 100, f"{r5['first_fail_ratio']} ≥ 100")
nan_rows = [row["ratio"] for row in r5.get("rows", []) if row.get("nan")]
chk("R5", "rows_nan", not nan_rows, f"nan 行={nan_rows}")

r6 = g["R6_spherical_chain_violation"]
chk("R6", "sep_p99_m", r6["sep_p99_m"] <= 0.012, f"{r6['sep_p99_m']:.5f} ≤ 0.012")
chk("R6", "violation_rate_gt_5cm_pct", r6["violation_rate_gt_5cm_pct"] == 0.0, f"{r6['violation_rate_gt_5cm_pct']:.3f} = 0")

r7 = g["R7_fluid_volume_wcsph"]
chk("R7", "particles_conserved", r7["particles_end"] == r7["particles_t1"], f"{r7['particles_end']} = {r7['particles_t1']}")
chk("R7", "volume_max_abs_rel_err_pct", r7["volume_max_abs_rel_err_pct"] <= 80.0, f"{r7['volume_max_abs_rel_err_pct']:.3f} ≤ 80")

# 2026-10-09 P16：R7 溃坝全程 interior=0 ⇒ 68.7% 全部是「自由面核截断」的口径。
# 这条是**报告型判据**：把现状钉成事实，不是修。退化方向只有「自由面贡献升高
# 或内部贡献出现异常」，所以阈值取「不变量」而不是「越低越好」。
r7i = g.get("R7i_fluid_volume_split")
if r7i is not None:
    chk("R7i", "interior_frac_pct", r7i["interior_frac_pct"] <= 0.5, f"{r7i['interior_frac_pct']:.3f} ≤ 0.5")
    chk("R7i", "mean_rho_over_rho0_all", 0.6 <= r7i["mean_rho_over_rho0_all"] <= 1.05, f"{r7i['mean_rho_over_rho0_all']:.4f} ∈ [0.6, 1.05]")
    chk("R7i", "mean_rho_over_rho0_surface", 0.5 <= r7i["mean_rho_over_rho0_surface"] <= 0.95, f"{r7i['mean_rho_over_rho0_surface']:.4f} ∈ [0.5, 0.95]")

# 2026-10-10 R8：睡眠的**位置代价**（评审「方向 14」的真实性半边）。仓库此前只测入睡率/
# 唤醒率，不测"睡眠把体冻在哪儿"。判据两条：① **非空洞**——32 体必须**全部入睡**
# （否则本探针什么都没测，直接红）；② 睡着后的末态与"关睡眠参考"的逐体偏差有界。
r8 = g.get("R8_sleep_position_error")
if r8 is not None:
    chk("R8", "all_slept", r8["slept_a"] == r8["dynamic"], f"{r8['slept_a']}/{r8['dynamic']} 入睡")
    chk("R8", "pos_dev_max_m", r8["pos_dev_max_m"] <= 0.02, f"{r8['pos_dev_max_m']:.5f} ≤ 0.02")
    chk("R8", "ang_dev_max_deg", r8["ang_dev_max_deg"] <= 1.0, f"{r8['ang_dev_max_deg']:.5f} ≤ 1.0")

# 2026-10-10 R9：破坏触发的**步精度与子步不变性**。旧实现每子步 clear 重记 ⇒ tick 级
# 快照只剩最后一个子步（快撞在那儿已被解掉）⇒ 默认 2 子步下 12 m/s 撞击一块碎块都挖不出。
# 判据四条：解析预测对得上、两档子步**结果相同**（不变性，就是那个 bug）、都要挖得出、亚阈不触发。
r9 = g.get("R9_destruction_trigger_tick")
if r9 is not None:
    chk("R9", "trigger_matches_analytic", abs(r9["trigger_tick"] - r9["analytic_tick"]) <= 1,
        f"{r9['trigger_tick']} ≈ {r9['analytic_tick']}")
    chk("R9", "substep_invariant", r9["trigger_tick"] == r9["trigger_tick_substeps1"]
        and r9["debris_pieces"] == r9["debris_pieces_substeps1"],
        f"2 子步 {r9['trigger_tick']} tick/{r9['debris_pieces']} 块 = 1 子步 {r9['trigger_tick_substeps1']} tick/{r9['debris_pieces_substeps1']} 块")
    chk("R9", "carves", r9["debris_pieces"] > 0 and r9["debris_pieces_substeps1"] > 0,
        f"碎块 {r9['debris_pieces']} / {r9['debris_pieces_substeps1']} > 0")
    chk("R9", "sub_threshold_pieces", r9["sub_threshold_pieces"] == 0, f"{r9['sub_threshold_pieces']} = 0")

# 2026-10-10 R10：**多体堆叠的层间载荷方差**（评审 §1.5 标 MISSING）。逐点冲量不暴露，但
# 层间载荷 = 其上方各盒净接触力之和（望远镜求和），逐盒净力用速度差分测（R3 同招）。
# 判据：稳住后（预热 10 s）载荷均值对齐理论与波动有界；另要**非空洞**——预热只给 2 s 时
# 场景必须仍在明显波动（否则"稳"是假的，比如堆根本没受力）。
r10 = g.get("R10_stack_load_steadiness")
if r10 is not None:
    chk("R10", "mean_err_pct_settled", r10["mean_err_pct_settled"] <= 0.05,
        f"{r10['mean_err_pct_settled']:.4f}% ≤ 0.05%")
    chk("R10", "std_over_load_pct_settled", r10["std_over_load_pct_settled_10s"] <= 0.5,
        f"{r10['std_over_load_pct_settled_10s']:.4f}% ≤ 0.5%")
    chk("R10", "non_vacuous_early_window", r10["std_over_load_pct_early_2s"] > 1.0,
        f"2 s 窗口 {r10['std_over_load_pct_early_2s']:.3f}% > 1%（证明场景真在演化）")

fails = [(n, d) for n, ok, d in checks if not ok]
for name, ok, detail in checks:
    print(f"  {'✅' if ok else '❌'} {name}: {detail}")
if fails:
    print(f"❌ 真实性基准红：{len(fails)}/{len(checks)} 项越界")
    sys.exit(0 if report_only else 1)
print(f"✅ 真实性基准 PASS（{len(checks)} 项不变量）")
PYEOF
rc=$?
if [ "$REPORT" -eq 1 ] && [ "$rc" -eq 1 ]; then
  echo "（--report-only：越界不判红）"
  exit 0
fi
exit "$rc"
