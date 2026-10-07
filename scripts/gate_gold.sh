#!/usr/bin/env bash
# **金样门**（保真/睡眠轴的自动门；`RECIPES.md` §「金样门」）。
#
# 为什么有这个脚本：金样读数此前**只写在 `OPEN-PROBLEMS.md` 的 T5 行里、没有任何门看着**
# ⇒ 那行陈旧到与实际差一倍（入睡写 35/45、实测 45/45）都没人发现。harness 现在**自带冻结
# 基线自检**（不符即 `❌ … ≠ 基线 …` + exit 1），本脚本把"跑哪些场景、怎么判绿"固定下来：
# **三个 Rapier 对照场景**（col45/pile5/tower25）＋**一个 vxl-only M3 坍塌金样**（破坏路径没有
# Rapier 对照可做），免得每轮手敲命令还各写各的口径。
#
# 用法（从仓库根或任意目录均可；脚本自己找路径）：
#   bash scripts/gate_gold.sh
# 退出码：0 = 全绿；非 0 = 第一处失败项的退出码（并已打印诊断）。
#
# ⚠️ 跑它时**别并发其它构建**：Windows 会锁 `gold-sample.exe`，`cargo run` 报
#    `failed to remove file … os error 5`（本会话踩过一次）。
#
# ⚠️ 换代：有意改引擎而动了基线 ⇒ 在 `OPEN-PROBLEMS.md` T5 记旧值/新值/理由，
#    并同步改 `gold-sample/src/main.rs` 的 `frozen_baseline`（同 ADR-0004）。

set -u

# 脚本所在目录的上一级 = 仓库根（不依赖调用者的 cwd）。
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root="$(dirname "$here")"
cd "$root/gold-sample" || {
    echo "找不到 gold-sample 目录（仓库根=${root}）" >&2
    exit 2
}
export CARGO_TARGET_DIR="${CARGO_TARGET_DIR_GOLD:-C:/vxl-wl-target-gold}"
out="${GOLD_LOG_DIR:-/tmp}"

# 逐项跑、逐项贴退出码（管道会吞退出码 —— fb51911 事故的纪律）。
# `set +e`：任一场景失败要**先打诊断再退**，不要被 `-e` 吃掉输出（RELEASE.md 坑 1）。
fail() { # fail <名称> <码> <日志>
    echo "❌ ${1} 失败（退出码 ${2}）——日志尾部：" >&2
    tail -n 20 "$3" >&2 || true
    exit "$2"
}

echo "== 金样门 @ ${root}（CARGO_TARGET_DIR=${CARGO_TARGET_DIR}）"

cargo fmt --all -- --check >"${out}/g_fmt.log" 2>&1
rc=$?
echo "gold_fmt=$rc"
[ $rc -eq 0 ] || fail "gold fmt" $rc "${out}/g_fmt.log"

set +e
cargo clippy --release -q -- -D warnings >"${out}/g_clippy.log" 2>&1
rc=$?
echo "gold_clippy=$rc"
[ $rc -eq 0 ] || fail "gold clippy" $rc "${out}/g_clippy.log"

# 三场景 = 冻结基线（配方见 RECIPES.md）。任一非 0（基线不符/编译失败）即退出。
# **必须判"真的做了基线判定"**：harness 对非基线配方会打印"跳过基线判定"并 exit 0
# （那是给实验用的正当行为），但**门里出现"跳过"就是失败**——否则配方写错会静默全绿
# （本脚本第一版就踩了：把场景名 `shift` 掉只当日志名用 ⇒ harness 收到的 argv 从 `600`
#   开始、场景名变成 "600"、三个场景全跑错，而退出码全是 0）。
run_scene() { # run_scene <场景名> <后随参数…>
    local name="$1"; shift
    local log="${out}/g_${name}.log"
    cargo run --release -q -- "$name" "$@" >"$log" 2>&1
    local rc=$?
    local line
    line="$(grep -m1 -E '金样基线 (PASS|FAIL)|跳过基线判定' "$log" || true)"
    echo "gold_${name}=${rc}  ${line}"
    [ $rc -eq 0 ] || fail "gold ${name}" $rc "$log"
    case "$line" in
        *"金样基线 PASS"*) ;;
        *) echo "❌ gold ${name}：**没有真正做基线判定**（配方与 frozen_baseline 不匹配？）" >&2
           tail -n 8 "$log" >&2 || true
           exit 3 ;;
    esac
}

run_scene col45  600 16 0.01 4 3.0 30 4
run_scene pile5  600 16 0.01 4 3.0 30 4
run_scene tower25 2400 16 0.01 1 3.0 30 16

# **M3 坍塌金样**（vxl-only；`gold-sample` 是 Rapier 对照通道，装不了体素破坏）。
# 配方 = `1200 64`（32 个多格碎块、**全睡**）；自检在 example 里（PASS/FAIL），换基线按 ADR-0004。
# ⚠️ 必须在**主 workspace** 跑（`gold-sample` 自带 [workspace] ⇒ 在那里 `-p vxl-phys` 找不到包）。
run_m3() {
    local log="${out}/g_m3_collapse.log"
    ( cd "$root" && CARGO_TARGET_DIR="${CARGO_TARGET_DIR_M3:-C:/vxl-wl-target}" \
        cargo run --release -q -p vxl-phys --example m3_collapse -- 1200 64 ) >"$log" 2>&1
    local rc=$?
    local line
    line="$(grep -m1 -E '金样基线 (PASS|FAIL)|跳过基线判定' "$log" || true)"
    echo "gold_m3_collapse=${rc}  ${line}"
    [ $rc -eq 0 ] || fail "gold m3_collapse" $rc "$log"
    case "$line" in
        *"金样基线 PASS"*) ;;
        *) echo "❌ gold m3_collapse：**没有真正做基线判定**（配方与 frozen 不匹配？）" >&2
           tail -n 8 "$log" >&2 || true
           exit 3 ;;
    esac
}
run_m3

# **M4 悬臂金样**（vxl-only：`gold-sample` 是 Rapier 通道，装不了本仓的布料）。
# 配方 = `600` tick；冻结基线存的是 **`f32` 位模式**（十进制往返对 7 位有效数字会失真，
# 本轮踩过 —— 见 example 头注）。判据 = 自由端坐标 / 垂度 / 最大应变**逐位**相等。
run_cantilever() {
    local log="${out}/g_m4_cantilever.log"
    ( cd "$root" && CARGO_TARGET_DIR="${CARGO_TARGET_DIR_M3:-C:/vxl-wl-target}" \
        cargo run --release -q -p vxl-phys --example m4_cantilever -- 600 ) >"$log" 2>&1
    local rc=$?
    local line
    line="$(grep -m1 -E '金样基线 (PASS|FAIL)|跳过基线判定' "$log" || true)"
    echo "gold_m4_cantilever=${rc}  ${line}"
    [ $rc -eq 0 ] || fail "gold m4_cantilever" $rc "$log"
    case "$line" in
        *"金样基线 PASS"*) ;;
        *) echo "❌ gold m4_cantilever：**没有真正做基线判定**（配方与 frozen 不匹配？）" >&2
           tail -n 8 "$log" >&2 || true
           exit 3 ;;
    esac
}
run_cantilever

echo "✅ 金样门全绿（fmt / clippy / col45 / pile5 / tower25 / m3_collapse / m4_cantilever 全 0）"
exit 0
