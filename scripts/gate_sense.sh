#!/usr/bin/env bash
# 感官输出门（声音/震动 S1 + S2 + S3，`docs/ROUTE.md` §3.2）：契约的**工具链侧**判据。
#
# 与 crate 内 `crates/vxl-phys-sense/tests/*.rs` 的分工：
#   · 那边判**库 API 的行为**（零干扰 / 确定性 / 预算 / 静止体不产循环音 / 缺项拒绝）——随 workspace 测试在 CI 三工具链跑；
#   · 这边判**工具链产物**——`sense_dump` 两跑逐字一致 + 预算上界 + 与基线文件对拍 + **表→音色组 id 的搬运**。
# 只有库过而 CLI 漂移，音效师拿到的 cue 流就不可复现 ⇒ 两边都要过。
#
# 四个模式各判一遍（同一套判据口径）：
#   · impact（S1 冲击）  → bench/baselines/sense_dump_300x64.json
#   · slide （S2 持续接触）→ bench/baselines/sense_dump_slide_300x64.json
#   · table （S3 材质表）  → 无需基线：判"表 → 音色组 id"**逐行**正确 + **缺项显式拒绝**
#                            （半张表必须退出码 2、点名缺的槽、且**不产出任何 cue 行**）
#   · haptic（S4 震动包络）→ bench/baselines/sense_dump_haptic_300x64.json + 两条**结构性质**
#                            （上钳边界闭在 1.0 / 纯释放段逐帧非增且末帧恰 0.0 —— ROUTE §3.2 的 S4 判据）
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

# 判 S4（震动设备输出 / 包络）：$1=基线路径。返回 0 通过 / 1 判据红 / 3 无法判定。
#
# S4 与前几片的差别：它的输出是**带状态的包络**（设备帧流），所以除"确定性 + 基线对拍"外还要判
# 两条**结构性质**（`docs/ROUTE.md` §3.2 给 S4 定的判据就是这两条）：
#   · **上钳边界**：逐帧 `intensity ∈ [0,1]`，且场景真的顶到过 `1.0`（`clamped > 0` / `peak == 1.0`，
#     否则"边界"根本没被考到 ⇒ 空洞）；
#   · **包络单调**：`last_cue_tick` 之后的**纯释放段**必须逐帧非增、且末帧**恰好** `0.0`
#     —— 库侧判据 ② 首次运行就把 4e-45 次正规**不动点**（包络永远到不了 0，器件长期微振）抓红，
#     故这条不是装饰：它是"释放干净"唯一的守门人。
#
# 另判**更新率口径**：帧数 = `ticks × frames_per_tick`，且 `frames_per_tick = round(update_hz × tick_dt)`。
check_haptic() {
  local base="$1"

  if ! sense_run 300 64 16 haptic >"$TMP/hp.a.jsonl" 2>"$TMP/hp.a.json"; then
    echo "❌ gate_sense[haptic]：sense_dump 跑失败"
    cat "$TMP/hp.a.json"
    return 1
  fi
  if ! sense_run 300 64 16 haptic >"$TMP/hp.b.jsonl" 2>"$TMP/hp.b.json"; then
    echo "❌ gate_sense[haptic]：sense_dump 第二跑失败"
    return 1
  fi

  # ① 确定性（工具链侧）：两跑帧流**逐字节**一致。
  if ! cmp -s "$TMP/hp.a.jsonl" "$TMP/hp.b.jsonl"; then
    echo "❌ gate_sense[haptic]：两跑帧流不一致（确定性红）——前 5 处差异："
    diff "$TMP/hp.a.jsonl" "$TMP/hp.b.jsonl" | head -n 5
    return 1
  fi

  # ② 汇总行 + 流摘要 → 单行 JSON（与基线同形）。
  if ! "$PY" - "$TMP/hp.a.json" "$TMP/hp.a.jsonl" "$TMP/hp.summary.json" <<'PYEOF'
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
    echo "❌ gate_sense[haptic]：统计行解析失败（CLI 的 stderr 格式变了？）"
    cat "$TMP/hp.a.json"
    return 3
  fi

  cat "$TMP/hp.summary.json"

  if [ "${WRITE_SENSE_BASELINE:-0}" = "1" ]; then
    mkdir -p "$(dirname "$base")"
    cp "$TMP/hp.summary.json" "$base"
    echo "📝 已写入基线：$base"
    return 0
  fi

  # ③ 判据：非空洞 + 上钳边界 + 更新率口径 + 释放段单调 + 基线对拍。
  if ! "$PY" - "$TMP/hp.summary.json" "$TMP/hp.a.jsonl" "$base" "$REPORT" <<'PYEOF'
import json, sys

summary_path, jsonl_path, base_path, report = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4] == "1"
with open(summary_path, encoding="utf-8") as f:
    got = json.loads(f.read())
bad = []

# 非空洞：两个源都真的产过 cue（一路电平的两个来源都活），且上钳真的发生过。
if got["impact_cues"] <= 0 or got["slide_cues"] <= 0:
    bad.append(f"empty source: impact_cues={got['impact_cues']} slide_cues={got['slide_cues']} (vacuous gate)")
if got["clamped"] <= 0 or got["peak"] != 1.0:
    bad.append(f"clamp never happened: clamped={got['clamped']} peak={got['peak']} -> 上钳边界没被考到（空洞）")
# 上钳的**另一端**：释放必须真的落到 0。这条看**全精度**的 `silent`（Rust 侧逐帧计数），
# 不能只看帧流——`4e-45` 这种次正规残值按 6 位小数打印出来就是 `0.000000`（实测：注入
# 次正规不动点后，"末帧 == 0.0" 在流上照样通过，只有 `silent=0` 把它点出来）。
if got["silent"] <= 0:
    bad.append("silent=0：包络从没真正静默（次正规数不动点？释放必须真的收敛到 0）")

# 更新率口径：每 tick 恰好 frames_per_tick 帧；帧数 = ticks × 每 tick 帧数。
fpt = got["frames_per_tick"]
want_fpt = round(got["update_hz"] * got["tick_dt"])
if fpt < 1:
    bad.append(f"frames_per_tick={fpt} < 1（设备会静默）")
if abs(fpt - want_fpt) > 1e-9:
    bad.append(f"frames_per_tick {fpt} != round(update_hz × tick_dt) = {want_fpt}")
if got["frames"] != got["ticks"] * fpt or got["jsonl_lines"] != got["ticks"] * fpt:
    bad.append(f"frames {got['frames']} / jsonl {got['jsonl_lines']} != ticks × frames_per_tick {got['ticks'] * fpt}")

# 逐帧：帧号由 tick 唯一决定（帧号 = tick 的纯函数 ⇒ 无隐藏补帧/丢帧）；intensity 落在 [0,1]。
ints = []
cut = (got["last_cue_tick"] + 1) * fpt
with open(jsonl_path, encoding="utf-8") as f:
    for i, line in enumerate(f):
        rec = json.loads(line)
        if rec["frame"] != i or not (rec["tick"] * fpt <= i < (rec["tick"] + 1) * fpt):
            bad.append(f"第 {i + 1} 行：frame={rec['frame']} / tick={rec['tick']} 不自洽（帧号必须是 tick 的纯函数）")
        x = rec["intensity"]
        if not (0.0 <= x <= 1.0):
            bad.append(f"第 {i + 1} 行：intensity={x} 越界（上钳失效）")
        ints.append(x)
        if len(bad) >= 8:
            break

# 包络单调（真实流）：纯释放段逐帧非增，且末帧**恰好** 0.0。
tail = ints[cut:]
if not tail:
    bad.append(f"释放段为空（last_cue_tick={got['last_cue_tick']}：段的划分或帧数口径变了？）")
else:
    for i, (a, b) in enumerate(zip(tail, tail[1:])):
        if b > a:
            bad.append(f"释放段必须逐帧非增：第 {cut + i + 2} 帧 {a} -> {b}（上升）")
            break
    if tail[-1] != 0.0:
        bad.append(f"释放段末帧必须恰好 0.0，实测 {tail[-1]}（否则器件长期微振）")

# 与基线逐字段对拍（含流摘要 ⇒ 等价于"帧流逐字节一致"）。
want = None
try:
    with open(base_path, encoding="utf-8") as f:
        want = json.loads(f.read().strip().splitlines()[0])
except OSError:
    bad.append(f"基线缺失：{base_path}（用 WRITE_SENSE_BASELINE=1 生成）")
if want is not None:
    for k in sorted(set(want) | set(got)):
        if want.get(k) != got.get(k):
            bad.append(f"{k}: baseline {want.get(k)} vs got {got.get(k)}")

if bad:
    for b in bad[:8]:
        print(f"gate_sense FAIL[haptic]: {b}")
    sys.exit(0 if report else 1)
print(f"gate_sense PASS[haptic]: 上钳边界（peak=1.0 / clamped={got['clamped']} 帧）/ "
      f"释放段单调（{got['ticks'] * fpt - cut} 帧 → 恰好 0.0）/ 更新率 {fpt} 帧/tick / 基线对拍")
PYEOF
  then
    return 1
  fi
  return 0
}

rc=0
check_mode impact "bench/baselines/sense_dump_300x64.json" || rc=$?
check_mode slide "bench/baselines/sense_dump_slide_300x64.json" || rc=$?
check_table "crates/vxl-phys-sense/assets/sound_table.txt" || rc=$?
check_haptic "bench/baselines/sense_dump_haptic_300x64.json" || rc=$?
exit "$rc"
