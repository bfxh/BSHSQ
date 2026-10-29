#!/usr/bin/env bash
# **m1_pile 配置交错 A/B**（M1 性能档专用）：两套配方**交替**跑 N 轮，取**成对**差值。
#
# 为什么必须交错：本机单跑同码漂移 ±5%、并发时 ±25%（`scripts/ab_perf.sh` 的同一结论）。
# "改前跑一次、改后跑一次"分辨不出 5–10% 的差异 ⇒ 只认交替后的成对读数。
#
# 用法：bash scripts/m1_ab.sh "16 1 1 600" "16 4 1 600" [轮数=3]
# 输出：每轮两臂的 ms/tick + 每轮的比值，最后给**中位比值**。
set -u
A="${1:?用法: m1_ab.sh \"<iters substeps threads ticks>\" \"<...>\" [轮数]}"
B="${2:?}"
N="${3:-3}"
export CARGO_TARGET_DIR="${CARGO_TARGET_DIR:-C:/vxl-wl-target}"

ms_of() { # 跑一套配方，取最后一行的"累计均值"
    cargo run --release -q -p vxl-phys --example m1_pile -- $1 2>/dev/null \
        | grep -oE "累计均值 *[0-9.]+ ms" | tail -1 | grep -oE "[0-9.]+" | head -1
}

ratios=()
for r in $(seq 1 "$N"); do
    a=$(ms_of "$A"); b=$(ms_of "$B")
    if [ -z "${a:-}" ] || [ -z "${b:-}" ]; then echo "❌ 第 $r 轮取数失败（a='$a' b='$b'）"; exit 1; fi
    ratio=$(awk -v a="$a" -v b="$b" 'BEGIN{printf "%.3f", b/a}')
    echo "  轮 $r: A=$a ms | B=$b ms | B/A=$ratio"
    ratios+=("$ratio")
done
med=$(printf '%s\n' "${ratios[@]}" | sort -n | awk '{v[NR]=$1} END{print v[int((NR+1)/2)]}')
echo "⇒ 中位 B/A = $med（>1 表示 B 更慢）；成对读数，不是两次单跑"
