#!/usr/bin/env bash
# 感官输出门（声音/震动 S1，`docs/ROUTE.md` §3.2）：三条契约的**工具链侧**判据。
#
# 与 crate 内 `crates/vxl-phys-sense/tests/sense_contracts.rs` 的分工：
#   · 那边判**库 API 的行为**（零干扰 / 确定性 / 预算）——随 workspace 测试在 CI 三工具链跑；
#   · 这边判**工具链产物**——`sense_dump` 两跑逐字一致 + 预算上界 + 与基线文件对拍。
# 只有库过而 CLI 漂移，音效师拿到的 cue 流就不可复现 ⇒ 两边都要过。
#
# 用法：
#   scripts/gate_sense.sh [根目录] [--report-only]
#     --report-only   只打印不判红（本地探索用）
#   写基线：WRITE_SENSE_BASELINE=1 scripts/gate_sense.sh .
# 退出码：0 = 通过；1 = 判据红；3 = 无法判定（跑不起来/解析失败）
#
# 基线换代：改 `bench/baselines/sense_dump_300x64.json` 必须在 PR 里说明理由（旧 → 新 + 原因）。
set -uo pipefail

ROOT="."
REPORT=0
for a in "$@"; do
  case "$a" in
    --report-only) REPORT=1 ;;
    *) ROOT="$a" ;;
  esac
done

# python 探测：Windows 的 App Execution Alias 会在 PATH 里放一个 `python3` 桩（调用即返回 49）
# ⇒ 必须**试跑**一次再选（口径同 `scripts/gate_bench.sh`）。
PY=""
for c in python3 python; do
  p=$(command -v "$c" 2>/dev/null) || continue
  if "$p" -c 'import sys; sys.exit(0 if sys.version_info[0] == 3 else 1)' >/dev/null 2>&1; then
    PY="$p"
    break
  fi
done
if [ -z "$PY" ]; then
  echo "❌ gate_sense：找不到可用的 python3/python"
  exit 3
fi

TMP="$(mktemp -d)" || exit 3
trap 'rm -rf "$TMP"' EXIT
cd "$ROOT" || exit 3
BASE="bench/baselines/sense_dump_300x64.json"

sense_run() {
  cargo run --release -q -p vxl-phys-sense --bin sense_dump -- "$@"
}

if ! sense_run 300 64 16 >"$TMP/a.jsonl" 2>"$TMP/a.json"; then
  echo "❌ gate_sense：sense_dump 跑失败"
  exit 1
fi
if ! sense_run 300 64 16 >"$TMP/b.jsonl" 2>"$TMP/b.json"; then
  echo "❌ gate_sense：sense_dump 第二跑失败"
  exit 1
fi

# ① 确定性（工具链侧）：两跑**逐字节**一致。
if ! cmp -s "$TMP/a.jsonl" "$TMP/b.jsonl"; then
  echo "❌ gate_sense：两跑 cue 流不一致（确定性红）——前 5 处差异："
  diff "$TMP/a.jsonl" "$TMP/b.jsonl" | head -n 5
  exit 1
fi

# ② 汇总行 + 流摘要 → 单行 JSON（与基线同形）。
if ! "$PY" - "$TMP/a.json" "$TMP/a.jsonl" "$TMP/summary.json" <<'PYEOF'
import hashlib, json, sys

stats_path, jsonl_path, out_path = sys.argv[1], sys.argv[2], sys.argv[3]
with open(stats_path, encoding="utf-8") as f:
    stats = json.loads(f.read().strip().splitlines()[-1])
h = hashlib.sha256()
lines = 0
with open(jsonl_path, "rb") as f:
    for chunk in iter(lambda: f.read(1 << 16), b""):
        h.update(chunk)
        lines += chunk.count(b"\n")
stats["jsonl_lines"] = lines
stats["sha256_jsonl"] = h.hexdigest()
with open(out_path, "w", encoding="utf-8", newline="\n") as f:
    f.write(json.dumps(stats, ensure_ascii=False, sort_keys=True) + "\n")
PYEOF
then
  echo "❌ gate_sense：统计行解析失败（CLI 的 stderr 格式变了？）"
  cat "$TMP/a.json"
  exit 3
fi

cat "$TMP/summary.json"

if [ "${WRITE_SENSE_BASELINE:-0}" = "1" ]; then
  mkdir -p "$(dirname "$BASE")"
  cp "$TMP/summary.json" "$BASE"
  echo "📝 已写入基线：$BASE"
  exit 0
fi

# ③ 判据：非空洞 + 预算上界 + 与基线对拍。
if ! "$PY" - "$TMP/summary.json" "$BASE" "$REPORT" <<'PYEOF'
import json, sys

summary_path, base_path, report = sys.argv[1], sys.argv[2], sys.argv[3] == "1"
with open(summary_path, encoding="utf-8") as f:
    got = json.loads(f.read())
try:
    with open(base_path, encoding="utf-8") as f:
        want = json.loads(f.read().strip().splitlines()[0])
except OSError:
    print(f"gate_sense: baseline missing -> {base_path} (regenerate with WRITE_SENSE_BASELINE=1)")
    sys.exit(0 if report else 1)

bad = []
# 非空洞：场景真的产生了冲击与 cue，且预算规则真的被触发过
if got["impacts"] <= 0 or got["cues"] <= 0:
    bad.append("empty scene: impacts/cues must be > 0 (vacuous gate)")
if got["dropped_budget"] <= 0:
    bad.append("budget rule never triggered (dropped_budget = 0) -> vacuous gate")
if got["dropped_weak"] <= 0:
    bad.append("weak filter never triggered (dropped_weak = 0) -> vacuous gate")
# 预算上界
if got["peak_cues_per_tick"] > got["budget"]:
    bad.append(f"peak cues/tick {got['peak_cues_per_tick']} > budget {got['budget']}")
# 与基线逐字段对拍（含流摘要 ⇒ 等价于"cue 流逐字节一致"）
for k in sorted(set(want) | set(got)):
    if want.get(k) != got.get(k):
        bad.append(f"{k}: baseline {want.get(k)} vs got {got.get(k)}")

if bad:
    for b in bad:
        print(f"gate_sense FAIL: {b}")
    sys.exit(0 if report else 1)
print("gate_sense PASS: determinism / budget / baseline")
PYEOF
then
  exit 1
fi
exit 0
