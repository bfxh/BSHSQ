#!/usr/bin/env bash
# 词汇禁令扫描（裁决 61；施工令 T4/T6）——引擎仓源码/文档不得出现宿主业务词汇。
#
# 用法：scripts/vocab_scan.sh [根目录，默认 .]
# 退出码：0 = 通过；1 = 命中违规（CI 阻断）。
#
# 词表与白名单（裁决 61；crate 名豁免已按用户裁决 2026-09-12 删除）：
#   - vehicle / module / cell / 404：独立单词（词边界）形式禁止；
#     404 的例外只到「源码行号引用」形状，登记见下方 scan 处（不放宽其余用法）；
#   - vxl / vxl- 前缀：引擎自身代号，**不在扫描面**（施工令 §3 推荐解释）；
#   - `std::cell`：Rust 标准库路径（非宿主词汇）；
#   - `module: <shader 形参>`：**wgpu 计算管线描述符的字段名**（第三方 API，非宿主词汇；
#     2026-09-23 首片 GPU 管线落地时登记，见 crates/vxl-phys-gpu/src/probe.rs）。豁免只放
#     `module: ?&?shader` 这一种写法（形参名必须叫 shader）——别无差别地放过整个字段名。

set -u
ROOT="${1:-.}"
FAIL=0

scan() { # $1 = 正则；$2 = 白名单过滤正则（-v）
  local hits
  hits=$(grep -rniE "$1" "$ROOT/crates" "$ROOT/docs" \
      --include='*.rs' --include='*.md' --include='*.toml' 2>/dev/null \
    | grep -vE "$2" || true)
  if [ -n "$hits" ]; then
    echo "❌ 词汇禁令命中：$1"
    echo "$hits"
    FAIL=1
  fi
}

# vehicle：无豁免（原 crate 名已改名 vxl-phys-wheeled / vxl-phys-aero）
scan '\bvehicle\b' '^\s*$'
# module：豁免 wgpu 计算管线描述符的字段名（第三方 API；见头部说明）
scan '\bmodule\b' 'module: ?&?shader'
# cell：豁免 Rust 标准库路径 std::cell
scan '\bcell\b' 'std::cell'
# 404：仅豁免「404 本身就是源码行号引用」的那一行（`文件.rs:404`，且其后不接数字）。
#   登记理由（规范：行号本身是观测数据的一部分时登记例外，不得为过门改文档措辞）：
#   docs/OPEN-PROBLEMS.md 里 `见 \`sweep.rs:404\`` 是指向 eval_hit 命中判据的行号读数，
#   属观测数据而非宿主词汇用法。散文里裸写 404、或代码里 `status = 404` 仍判红。
#   已知代价：本门的白名单是逐行过滤（grep -vE），若同一行既有行号引用又有真正的 404 用法，
#   该行会被放过——现网无此形状，命中即另案收紧。
scan '\b404\b' '\.rs:404([^0-9]|$)'

if [ "$FAIL" -eq 0 ]; then
  echo "✅ 词汇禁令扫描通过（vehicle/module/cell/404；vxl- 前缀与 std::cell 豁免）"
else
  echo "—— 禁令词表与白名单说明见本脚本头部（施工令 §3）"
fi
exit "$FAIL"
