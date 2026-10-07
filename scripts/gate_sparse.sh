#!/usr/bin/env bash
# **M1「简单碰撞」档的标准出口**（`SPEC.md` §3 的 8B/30FPS 宣称载体 = `m1_sparse`）。
#
# 为什么有它（同一纪律见 `gate_scale.sh` 头注）：`OPEN-PROBLEMS.md` #3 定案"密堆（`m1_scale`）
# ≠ 简单碰撞"——SPEC §3 那档由 `m1_sparse` 承载，但它此前**只有 example、没有标准入口与判定
# 口径**，#3 当年记的 100k 数字还是 4096 体 / 60 tick 小样**外推**出来的（见 #3 的 2026-10-07
# 更正：直接跑 100k 后，那条外推不成立）。
#
# 用法：
#   bash scripts/gate_sparse.sh                                  # 默认 8 / 100000 / 400 / 16；**只报不判**
#   SPARSE_STRICT=1 bash scripts/gate_sparse.sh                  # 参考机上按 SPEC §3（30 FPS ⇒ 尾窗均 ≤33.3 ms）判
#   SPARSE_ARGS="8 100000 400 16" bash scripts/gate_sparse.sh    # 自定义档
#   SPARSE_LOG=<path> bash scripts/gate_sparse.sh                # 日志落盘位置
# 退出码：0 = 跑完（或 STRICT 判过）；1 = STRICT 不达标；2 = 找不到仓库根；3 = 解析不到读数行。
#
# ⚠️ **不进 CI**：CI runner 是共享 4 vCPU（`SPEC` §5：性能门**只在参考硬件**判定），且本档默认
#    要建 10 万 + 10 万体。判据归参考硬件；本机读数一律作**旁证**，且须标注机器状态（本仓有并发
#    开发者 —— 读 `gate_scale.sh` 头注的测量协议）。

set -u

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root="$(dirname "$here")"
cd "$root" || {
    echo "找不到仓库根（${root}）" >&2
    exit 2
}
export CARGO_TARGET_DIR="${CARGO_TARGET_DIR:-C:/vxl-wl-target}"
log="${SPARSE_LOG:-C:/vxl-wl-target/gate_sparse.log}"

# shellcheck disable=SC2206  # 就是要按空格切参数（与 example 的 argv 口径一致）
read -r -a args <<<"${SPARSE_ARGS:-8 100000 400 16}"
echo "== 简单碰撞档（m1_sparse）：${args[*]}（日志 ${log}）"
mkdir -p "$(dirname "$log")" 2>/dev/null || true

cargo run --release -q -p vxl-phys --example m1_sparse -- "${args[@]}" >"$log" 2>&1
rc=$?
if [ "$rc" -ne 0 ]; then
    echo "❌ m1_sparse 运行失败（exit ${rc}）——日志尾部：" >&2
    tail -n 12 "$log" >&2 || true
    exit "$rc"
fi

# 解析（缺行 = 没有真正读数 ⇒ exit 3，宁可红也不要"静默通过"）。
line="$(grep -m1 -E '全窗 均 [0-9.]+ ms \| 尾窗 均 [0-9.]+ / p50 [0-9.]+ / max [0-9.]+ ms' "$log" || true)"
[ -n "$line" ] || {
    echo "❌ 解析不到读数行 —— 没有真正做读数（输出格式变了？）" >&2
    tail -n 8 "$log" >&2 || true
    exit 3
}
all="$(echo "$line" | sed -E 's/.*全窗 均 ([0-9.]+) ms.*/\1/')"
tail_avg="$(echo "$line" | sed -E 's/.*尾窗 均 ([0-9.]+) .*/\1/')"
p50="$(echo "$line" | sed -E 's/.*p50 ([0-9.]+) .*/\1/')"
tail_max="$(echo "$line" | sed -E 's/.*max ([0-9.]+) ms.*/\1/')"

echo "  全窗均 ${all} ms ｜ 尾窗 均 ${tail_avg} ／ p50 ${p50} ／ max ${tail_max} ms"
echo "  口径：SPEC §5 —— 判据归**参考硬件**；本机读数只作旁证（30 FPS ⇒ 尾窗均 ≤33.3 ms）"

if [ "${SPARSE_STRICT:-0}" = "1" ]; then
    ok="$(awk -v v="$tail_avg" 'BEGIN{ print (v+0 <= 33.3) ? "yes" : "no" }')"
    if [ "$ok" = "yes" ]; then
        echo "✅ 尾窗均 ${tail_avg} ms ≤ 33.3 ms（SPEC §3 的 30 FPS 档）"
        exit 0
    fi
    echo "❌ 尾窗均 ${tail_avg} ms > 33.3 ms（SPEC §3 的 30 FPS 档）" >&2
    exit 1
fi
echo 'ℹ️ 只报不判（SPARSE_STRICT=1 用于参考硬件上的判定）'
exit 0
