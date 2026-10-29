#!/usr/bin/env bash
# **同二进制、两套配置的交错 A/B**（`m1_pile` 这类"参数就是判据"的场景）。
#
# 为什么需要它：`scripts/ab_perf.sh` 比的是**两个 ref**（各编一份）；而调参时二进制是同一个，
# 只是命令行参数不同。本机单跑漂移 ±5%、并发时 ±25% ⇒ 必须**交替**跑并按轮配对比较。
#
# 用法：
#   bash scripts/ab_config.sh m1_pile "16 1 1 600" "16 4 1 600" [轮数=3]
# 输出：每轮 A/B 的读数 + 成对差的中位数（比"各跑一次再相减"可信得多）。
set -u
export PYTHONUTF8=1
ex="${1:?用法: ab_config.sh <example> \"<argsA>\" \"<argsB>\" [轮数]}"
a="${2:?缺 A 组参数}"
b="${3:?缺 B 组参数}"
rounds="${4:-3}"
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$root" || exit 2
export CARGO_TARGET_DIR="${CARGO_TARGET_DIR:-C:/vxl-wl-target}"

# 预热一次（编译 + 页表/时钟爬坡）：把首个被测样本从计时里剔出去
cargo run --release -q -p vxl-phys --example "$ex" -- $a >/dev/null 2>&1 || true

pair() { # pair <args> → 打印 "均值ms 健康字段"
    cargo run --release -q -p vxl-phys --example "$ex" -- $1 2>/dev/null \
        | grep -E "累计均值" | tail -1
}

echo "== 交错 A/B：$ex / A=[$a] vs B=[$b] / ${rounds} 轮"
for r in $(seq 1 "$rounds"); do
    la="$(pair "$a")"; lb="$(pair "$b")"
    echo "  轮 $r  A: $la"
    echo "  轮 $r  B: $lb"
done
echo
echo "⚠️ 读数按上面的行**成对**比较；不要只看最后一轮的绝对值（负载漂移 ±25%）。"
