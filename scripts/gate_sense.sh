#!/usr/bin/env bash
# 感官输出门（声音/震动 S1 + S2 + S3，`docs/ROUTE.md` §3.2）：契约的**工具链侧**判据。
#
# 与 crate 内 `crates/vxl-phys-sense/tests/*.rs` 的分工：
#   · 那边判**库 API 的行为**（零干扰 / 确定性 / 预算 / 静止体不产循环音 / 缺项拒绝）——随 workspace 测试在 CI 三工具链跑；
#   · 这边判**工具链产物**——`sense_dump` 两跑逐字一致 + 预算上界 + 与基线文件对拍 + **表→音色组 id 的搬运**。
# 只有库过而 CLI 漂移，音效师拿到的 cue 流就不可复现 ⇒ 两边都要过。
#
# 三个模式各判一遍（同一套判据口径）：
#   · impact（S1 冲击）  → bench/baselines/sense_dump_300x64.json
#   · slide （S2 持续接触）→ bench/baselines/sense_dump_slide_300x64.json
#   · table （S3 材质表）  → 无需基线：判"表 → 音色组 id"**逐行**正确 + **缺项显式拒绝**
#                            （半张表必须退出码 2、点名缺的槽、且**不产出任何 cue 行**）
#
# 用法：
#   scripts/gate_sense.sh [根目录] [--report-only]
#     --report-only   只打印不判红（本地探索用）
#   写基线：WRITE_SENSE_BASELINE=1 scripts/gate_sense.sh .
# 退出码：0 = 通过；1 = 判据红；3 = 无法判定（跑不起来/解析失败）
#
# 基线换代：改 `bench/baselines/sense_dump*.json` 必须在 PR 里说明理由（旧 → 新 + 原因）。
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
# Windows 控制台默认 GBK ⇒ 必须显式 UTF-8，否则判据里的中文（含 `gate_sense[s3]` 摘要行）乱码。
export PYTHONUTF8=1
export PYTHONIOENCODING=utf-8

TMP="$(mktemp -d)" || exit 3
trap 'rm -rf "$TMP"' EXIT
cd "$ROOT" || exit 3

sense_run() {
  cargo run --release -q -p vxl-phys-sense --bin sense_dump -- "$@"
}

# 判一个模式：$1=impact|slide  $2=基线路径。返回 0 通过 / 1 判据红 / 3 无法判定。
check_mode() {
  local mode="$1" base="$2" count_key out_key peak_key
  case "$mode" in
    slide) count_key=contacts; out_key=slides;  peak_key=peak_slides_per_tick ;;
    *)     count_key=impacts;  out_key=cues;    peak_key=peak_cues_per_tick ;;
  esac

  if ! sense_run 300 64 16 "$mode" >"$TMP/$mode.a.jsonl" 2>"$TMP/$mode.a.json"; then
    echo "❌ gate_sense[$mode]：sense_dump 跑失败"
    return 1
  fi
  if ! sense_run 300 64 16 "$mode" >"$TMP/$mode.b.jsonl" 2>"$TMP/$mode.b.json"; then
    echo "❌ gate_sense[$mode]：sense_dump 第二跑失败"
    return 1
  fi

  # ① 确定性（工具链侧）：两跑**逐字节**一致。
  if ! cmp -s "$TMP/$mode.a.jsonl" "$TMP/$mode.b.jsonl"; then
    echo "❌ gate_sense[$mode]：两跑 cue 流不一致（确定性红）——前 5 处差异："
    diff "$TMP/$mode.a.jsonl" "$TMP/$mode.b.jsonl" | head -n 5
    return 1
  fi

  # ② 汇总行 + 流摘要 → 单行 JSON（与基线同形）。
  if ! "$PY" - "$TMP/$mode.a.json" "$TMP/$mode.a.jsonl" "$TMP/$mode.summary.json" <<'PYEOF'
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
    echo "❌ gate_sense[$mode]：统计行解析失败（CLI 的 stderr 格式变了？）"
    cat "$TMP/$mode.a.json"
    return 3
  fi

  cat "$TMP/$mode.summary.json"

  if [ "${WRITE_SENSE_BASELINE:-0}" = "1" ]; then
    mkdir -p "$(dirname "$base")"
    cp "$TMP/$mode.summary.json" "$base"
    echo "📝 已写入基线：$base"
    return 0
  fi

  # ③ 判据：非空洞 + 预算上界 + 与基线对拍。
  if ! "$PY" - "$TMP/$mode.summary.json" "$base" "$REPORT" \
      "$count_key" "$out_key" "$peak_key" "$mode" <<'PYEOF'
import json, sys

summary_path, base_path, report = sys.argv[1], sys.argv[2], sys.argv[3] == "1"
count_key, out_key, peak_key, mode = sys.argv[4], sys.argv[5], sys.argv[6], sys.argv[7]
with open(summary_path, encoding="utf-8") as f:
    got = json.loads(f.read())
try:
    with open(base_path, encoding="utf-8") as f:
        want = json.loads(f.read().strip().splitlines()[0])
except OSError:
    print(f"gate_sense: baseline missing -> {base_path} (regenerate with WRITE_SENSE_BASELINE=1)")
    sys.exit(0 if report else 1)

bad = []
# 非空洞：场景真的产生了接触与 cue，且预算/弱过滤规则真的被触发过
if got[count_key] <= 0 or got[out_key] <= 0:
    bad.append(f"empty scene[{mode}]: {count_key}/{out_key} must be > 0 (vacuous gate)")
if got["dropped_budget"] <= 0:
    bad.append("budget rule never triggered (dropped_budget = 0) -> vacuous gate")
if got["dropped_weak"] <= 0:
    bad.append("weak filter never triggered (dropped_weak = 0) -> vacuous gate")
# 预算上界
if got[peak_key] > got["budget"]:
    bad.append(f"peak {peak_key} {got[peak_key]} > budget {got['budget']}")
# 与基线逐字段对拍（含流摘要 ⇒ 等价于"cue 流逐字节一致"）
for k in sorted(set(want) | set(got)):
    if want.get(k) != got.get(k):
        bad.append(f"{k}: baseline {want.get(k)} vs got {got.get(k)}")

if bad:
    for b in bad:
        print(f"gate_sense FAIL[{mode}]: {b}")
    sys.exit(0 if report else 1)
print(f"gate_sense PASS[{mode}]: determinism / budget / baseline")
PYEOF
  then
    return 1
  fi
  return 0
}

# 判 S3（材质表 / 资产映射）：$1=仓库示例表路径。返回 0 通过 / 1 判据红 / 3 无法判定。
#
# S3 与前两个模式不同：它是作用在**已有** cue 流上的**纯映射**（`map_banks`），所以"零干扰"是
# 类型级的（不借 `&World`）；这里要判的是**搬运正确**与**缺项显式拒绝**两条。
#
# 搬运正确**必须用本门自己生成的参考表**（`slot -> 1000+slot`）在**多材质**场景（`bank`，槽
# 1/2/3）上判：拿仓库里的示例表去判是自指的——"永远返回第一条"那种实现照样能对上自己的表。
# 参考表与 `1000+mat` 逐行对拍 ⇒ 查错行 / 恒返回首条 / 槽位偏移都会被抓住；再要求**至少两个**
# 槽真的出现（否则"选择"这件事没发生，判据空洞）。仓库示例表另判一遍（`impact`，槽 0）：
# 防止示例资产腐烂（表改了格式而场景没跟）。
check_table() {
  local asset="$1"

  # 参考表：本门自造，不依赖仓库里任何资产/基线。
  "$PY" - "$TMP/s3.ref.txt" <<'PYEOF'
import sys

with open(sys.argv[1], "w", encoding="utf-8", newline=chr(10)) as f:
    for slot in range(0, 9):
        f.write(f"{slot} -> {1000 + slot}" + chr(10))
PYEOF

  if ! sense_run 300 64 16 bank "$TMP/s3.ref.txt" >"$TMP/s3.a.jsonl" 2>"$TMP/s3.a.json"; then
    echo "❌ gate_sense[s3]：多材质例场（bank）跑失败"
    cat "$TMP/s3.a.json"
    return 1
  fi
  if ! sense_run 300 64 16 bank "$TMP/s3.ref.txt" >"$TMP/s3.b.jsonl" 2>"$TMP/s3.b.json"; then
    echo "❌ gate_sense[s3]：多材质例场第二跑失败"
    return 1
  fi

  # ① 确定性（工具链侧）：带 bank 的流两跑**逐字节**一致。
  if ! cmp -s "$TMP/s3.a.jsonl" "$TMP/s3.b.jsonl"; then
    echo "❌ gate_sense[s3]：两跑带 bank 的 cue 流不一致（确定性红）"
    diff "$TMP/s3.a.jsonl" "$TMP/s3.b.jsonl" | head -n 5
    return 1
  fi

  # ② 语义（独立参考）：每行 `bank` 在行尾且 == `1000 + mat`；槽 ≥ 2 个（非空洞）；槽 ⊆ {1,2,3}。
  if ! "$PY" - "$TMP/s3.a.jsonl" <<'PYEOF'
import json, sys

lines = [l for l in open(sys.argv[1], encoding="utf-8").read().splitlines() if l]
if not lines:
    print("gate_sense FAIL[s3]: 空流（判据空洞）")
    sys.exit(1)
slots = set()
for i, l in enumerate(lines):
    rec = json.loads(l)
    if "bank" not in rec:
        print(f"gate_sense FAIL[s3]: 第 {i + 1} 行没有 `bank` 字段")
        sys.exit(1)
    if list(rec)[-1] != "bank":
        print(f"gate_sense FAIL[s3]: `bank` 必须追加在行尾（第 {i + 1} 行：{list(rec)}）")
        sys.exit(1)
    slots.add(rec["mat"])
    want = 1000 + rec["mat"]
    if rec["bank"] != want:
        print(f"gate_sense FAIL[s3]: 第 {i + 1} 行 mat={rec['mat']} 应映射到 {want}，实测 {rec['bank']}（查错行？）")
        sys.exit(1)
if len(slots) < 2:
    print(f"gate_sense FAIL[s3]: 流里只出现了 {sorted(slots)} 一个材质槽 ⇒ 判不出选表（判据空洞）")
    sys.exit(1)
if not slots <= {1, 2, 3}:
    print(f"gate_sense FAIL[s3]: 例场约定挂槽 1/2/3，实测 {sorted(slots)}")
    sys.exit(1)
print(f"gate_sense[s3]: 参考表 {len(lines)} 行 / {len(slots)} 个槽 {sorted(slots)} / bank == 1000 + mat")
PYEOF
  then
    return 1
  fi

  # ③ **判据：缺项显式拒绝**（不静默用默认音）。把流里用到的**第一个**槽从参考表里去掉
  # ⇒"半张表"：必须退出码 2、stderr **点名**缺的是哪个槽、且**一行 cue 都不许产出**（不部分映射）。
  local miss
  if ! miss=$("$PY" - "$TMP/s3.a.jsonl" "$TMP/s3.ref.txt" "$TMP/s3.partial.txt" <<'PYEOF'
import json, sys

jsonl_path, table_path, out_path = sys.argv[1], sys.argv[2], sys.argv[3]
lines = [l for l in open(jsonl_path, encoding="utf-8").read().splitlines() if l]
drop = json.loads(lines[0])["mat"]
kept = 0
with open(table_path, encoding="utf-8") as f, open(out_path, "w", encoding="utf-8", newline=chr(10)) as o:
    for raw in f:
        s = raw.strip()
        if not s or s.startswith("#"):
            continue
        if int(s.split("->")[0].strip()) == drop:
            continue
        o.write(s + chr(10))
        kept += 1
print(f"{drop} {kept}")
PYEOF
  ); then
    echo "❌ gate_sense[s3]：造半张表失败"
    return 3
  fi
  local slot="${miss%% *}"
  if [ "${miss##* }" -le 0 ]; then
    echo "❌ gate_sense[s3]：半张表把槽全删光了（参考表里应当还有别的槽）"
    return 1
  fi
  sense_run 300 64 16 bank "$TMP/s3.partial.txt" >"$TMP/s3.rej.jsonl" 2>"$TMP/s3.rej.err"
  local rrc=$?
  if [ "$rrc" -ne 2 ]; then
    echo "❌ gate_sense[s3]：缺项必须退出码 2（不回落默认音），实测 $rrc；stderr："
    cat "$TMP/s3.rej.err"
    return 1
  fi
  if ! grep -q "材质槽 $slot" "$TMP/s3.rej.err"; then
    echo "❌ gate_sense[s3]：缺项报错必须**点名**缺失的槽 $slot；实测 stderr："
    cat "$TMP/s3.rej.err"
    return 1
  fi
  if [ -s "$TMP/s3.rej.jsonl" ]; then
    echo "❌ gate_sense[s3]：缺项时不许产出任何 cue 行（半张表；实测 $(wc -l <"$TMP/s3.rej.jsonl") 行）"
    return 1
  fi

  # ④ 仓库**示例表**（`impact` 场景，槽 0）也要能用：防止示例资产腐烂。
  if ! sense_run 300 64 16 impact "$asset" >"$TMP/s3.asset.jsonl" 2>/dev/null; then
    echo "❌ gate_sense[s3]：仓库示例表 $asset 配 impact 场景必须能跑通"
    return 1
  fi
  if [ ! -s "$TMP/s3.asset.jsonl" ]; then
    echo "❌ gate_sense[s3]：示例表跑出来是空流"
    return 1
  fi
  if ! "$PY" - "$TMP/s3.asset.jsonl" "$asset" <<'PYEOF'
import json, sys

jsonl_path, asset_path = sys.argv[1], sys.argv[2]
want = {}
with open(asset_path, encoding="utf-8") as f:
    for raw in f:
        s = raw.strip()
        if not s or s.startswith("#"):
            continue
        m, b = s.split("->")
        want[int(m.strip())] = int(b.strip())
n = 0
for i, l in enumerate(open(jsonl_path, encoding="utf-8").read().splitlines()):
    if not l:
        continue
    rec = json.loads(l)
    n += 1
    if rec.get("bank") != want.get(rec["mat"]):
        print(f"gate_sense FAIL[s3]: 示例表第 {i + 1} 行 mat={rec['mat']} -> {rec.get('bank')}，表里是 {want.get(rec['mat'])}")
        sys.exit(1)
print(f"gate_sense[s3]: 示例表 {asset_path}（{len(want)} 条）映射 {n} 行命中")
PYEOF
  then
    return 1
  fi

  echo "gate_sense PASS[s3]: 多槽搬运（参考表逐行对拍）/ 缺项显式拒绝（退出码 2，点名槽 $slot）/ 示例表可用"
  return 0
}

rc=0
check_mode impact "bench/baselines/sense_dump_300x64.json" || rc=$?
check_mode slide "bench/baselines/sense_dump_slide_300x64.json" || rc=$?
check_table "crates/vxl-phys-sense/assets/sound_table.txt" || rc=$?
exit "$rc"
