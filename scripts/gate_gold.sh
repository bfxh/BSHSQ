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
    # ⚠️ 必须**压成单行**：GitHub Actions 会把多行 echo 合并成一行，早先金样失败时
    # "期望 / 实测"两个关键行就是这样被吃掉的，只能回本地复现才拿到读数。
    # 用 ` ⏎ ` 当行分隔符，读起来仍然是分段的。
    tail -n 20 "$3" | sed -e :a -e 'N;s/\n/ ⏎ /;ta' >&2 || true
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

# **vxl-only 金样**（`gold-sample` 是 Rapier 对照通道，装不了本仓的体素/布料）—— 三者**同款**：
# 跑**主 workspace** 的 example（`gold-sample` 自带 [workspace] ⇒ 在那里 `-p vxl-phys` 找不到包）、
# 要求真出现 `金样基线 PASS`（只打"跳过基线判定"即判失败）。判据与冻结值都在各 example 里，
# 换代按 ADR-0004 记新旧值与理由。
run_vxl_gold() { # run_vxl_gold <example> [--pkg <crate>] <配方参数…>
    local ex="$1"; shift
    local pkg="vxl-phys"
    if [ "${1:-}" = "--pkg" ]; then pkg="$2"; shift 2; fi
    local log="${out}/g_${ex}.log"
    ( cd "$root" && CARGO_TARGET_DIR="${CARGO_TARGET_DIR_M3:-C:/vxl-wl-target}" \
        cargo run --release -q -p "$pkg" --example "$ex" -- "$@" ) >"$log" 2>&1
    local rc=$?
    local line
    line="$(grep -m1 -E '金样基线 (PASS|FAIL)|跳过基线判定' "$log" || true)"
    echo "gold_${ex}=${rc}  ${line}"
    [ $rc -eq 0 ] || fail "gold ${ex}" $rc "$log"
    case "$line" in
        *"金样基线 PASS"*) ;;
        *) echo "❌ gold ${ex}：**没有真正做基线判定**（配方与 frozen 不匹配？）" >&2
           tail -n 8 "$log" >&2 || true
           exit 3 ;;
    esac
}

# M3 坍塌（体素破坏；配方 `1200 64`）＝ 悬空体素块预断裂成 32 个多格碎块、自由下落**全睡**。
run_vxl_gold m3_collapse 1200 64

# M4 悬臂（布料；配方 `600`）：0.4×0.08 m 布带一端钉住、重力下垂到稳态。冻结值存 **`f32` 位模式**
# （十进制往返对 7 位有效数字会失真 —— 本轮踩过，见 example 头注）。
run_vxl_gold m4_cantilever 600
# M4 旗飘（布料 + 气动；配方 `600`）：旗面左列钉旗杆、风沿 +z 垂直吹。⚠️ 旗是**动**的 ⇒ 冻结
# 读数只在同配方下逐位可复现，**不是**稳态解。
run_vxl_gold m4_flag 600

# M4 绳索（vxl-only；配方 `600`）：40 节点、两端钉死在 `y = 1.6`、初始水平 ⇒ 下垂 U 形；
# 动态球（r=0.12、密度 800）从 `y = 2.2` 落到绳中段被托住。四项判据：绳末态哈希（FNV-1a
# over f32 位模式）/ 绳最低点 / 球末态 y / 结构断言（绳真的垂下来了且球在绳低点之上）。
# 绳是收敛型系统 ⇒ 要位模式，不像车那样必须容差（判据口径见 docs/PHYSICS-INVARIANTS.md §2.4）。
run_vxl_gold m4_rope 600

# 声学（A1；配方 `40`）：体素地面 + 0.5 m 立方砸地（冲击 = 声源）+ 听者在 `z = 2`。
# 两档：**开阔**（直达 + 地板反射，共 2 条）与**遮挡**（`z = 1` 立强吸声墙，直达与反射全挡，0 条）。
# 判据：两档 `jsonl` 全量 FNV-1a 哈希 + 条数 + 结构断言（开阔 `direct≥1 且 total≥2`、
# 遮挡 `direct==0 且 total==0`）。声学是纯函数 ⇒ 要位模式。
run_vxl_gold a1_acoustic_gold --pkg vxl-phys-acoustics 40

# 高斯泼溅（M7；配方 `720`）：y=0 面铺 7x7 各向同性核（sigma=0.5、间距 0.25），0.7 m 立方
# 从等值面上方 0.5 m 落下静置。判据：末态位姿位模式哈希 + 盒底贴等值面（|差|<2 cm，
# 守 2026-09-21 那次 `contacts_point: depth = skin - f -> -f` 的修复）。
run_vxl_gold m7_splat_gold 720

# M5 溃坝（液体；配方 `400`）：体素盆（地板 + 围堰）+ 自由水柱 `0.25×0.25×0.65`（504 粒）
# 失支撑坍塌 ⇒ 质心沿 +x 前移、波前沿推进。判据 = 粒子数 / 末态质心 / 末态最大 x 的 `f32`
# 位模式逐位（与 M4 同口径；**不是**驻留瞬态断言 —— 自由柱坍塌本就该飞溅）。
run_vxl_gold m5_dam_break 400

# M5 浮箱（液体 × 刚体双向；配方 `720`）：水槽 + 铸装水块 + 轻盒（密度 300）从深潜位上浮。
# **实测选窗**：180/360 tick 时盒子还在动（v_y ≠ 0），**720 起完全入睡**（v_y 精确 0）且与
# 1200 的 y 逐位相同 ⇒ 取 720 才钉的是"浮到平衡"。判据 = 粒子数 / 末态盒心 y / v_y 的位模式逐位。
run_vxl_gold m5_float_box 720

# M6 浮体系泊（液体 × 刚体 + **可断裂约束**；配方 `720`）：水槽 + 铸装水块 + 系泊轻盒
# （绳 = `JointKind::Distance`，锚点在水面上方、绳长 0.30 m）。两个子场景：
# ① **被拉住**（`break_force = 0` ⇒ 绳永不断）⇒ 浮体停在绳长处 `y = 1.30`、`v_y ≈ 0`；
# ② **被扯断**（`break_force = 2 N` < 浮体重 ≈ 5.1 N）⇒ 绳断、浮体入水浮到水面 `y ≈ 1.21`。
# 判据 = 粒子数 / 两个子场景的末态 `y` 与 `v_y` 的 `f32` 位模式逐位 / 各自的断裂标志。
run_vxl_gold m6_float_mooring 720
# M6 帆：船体 + 帆 + 顺风（面元气动）——判据是「顺风档 vs 零风档」的差分，两档末态都冻成位模式。
run_vxl_gold m6_float_sail 720
# M6 车：厚板斜坡 + 车身 + 两个球轮（转动关节）—— 球是解析接触，纯滚动有闭式解。
run_vxl_gold m6_wheeled_car 180

echo "✅ 金样门全绿（fmt / clippy / col45 / pile5 / tower25 / m3_collapse / m4_cantilever / m4_flag / m5_dam_break / m5_float_box / m6_float_mooring 全 0）"
exit 0
