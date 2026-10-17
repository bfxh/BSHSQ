# BSHSQ —— vxl_phys 自研 Rust 物理引擎

> 依据：[`docs/SPEC.md`](docs/SPEC.md)（引擎权威规格，V1.3）+ 上游总架构
> `VoxelForge-V3/docs/PHYS-ENGINE-VXLPHYS-V2.md`（v2 权威，Rust 路线；施工分解见
> `VoxelForge-V3/docs/SPEC-VXLPHYS-M0-V1.md`）；M0 实测记录见
> [`docs/M0-GATES.md`](docs/M0-GATES.md)。
> 定位：**独立商业级 Rust 物理引擎**，纯 Rust、确定性优先、模块化 crate DAG，
> 与任何消费工程零关联；仓库目录名「BSHSQ」= **Rust 物理（WuLi）**。

## 状态（2026-10-17 实测刷新）

| 里程碑 | 内容 | 状态 |
|---|---|---|
| **M0 MVE** | 盒/圆柱/高度场 + 顺序冲量 + 岛 + 确定性哈希；1 万静态+1 千动态 ≥60Hz | ✅ 竖切可跑 |
| **M0 骨架续建（V2 §11）** | 相位 arena + hot/cold 32B（`core/mem.rs`）、xxh3-128 规范化哈希（10 轮断言）、m0_gates 门槛（p50 1.09ms）、CI 四门 + 三编译器矩阵 + 词汇禁令扫描 | ✅ 交付（记录：`docs/M0-GATES.md`） |
| **M1 刚体核心** | 稳定/金样/性能收口 | 🟡 **主体**：金样 3 项（`col45`/`pile5`/`tower25`）+ 默认档长跑稳定门；性能档缺口与架构诊断见 `OPEN-PROBLEMS.md` P3、`DESIGN-staged-solver.md` |
| **M2 互操作核心** | 四接口（Provider/Medium/Bridge/Element）+ 迁移窄相/地形为 provider + 贯通示例 | 🟡 **部分**：四 trait + `Shape::Provider` 通道 + 刚体↔体素贯通已跑通；`BridgeKind` 三个真实消费方（splat/fluid/cloth），`ElementKind` 仍零消费（见 `UNIFIED-SOLVER-REVIEW.md`） |
| **M3 破坏/地形（体素）** | Voronoi 预断裂 + 运行时切割 + 体素块→刚体/粒子 | 🟡 **主体**：预断裂 + 运行时切割（体素 & 凸体外壳）+ 体素块→刚体 + 演示 `m3_impact`/`m3_voronoi` + 坍塌金样 `m3_collapse`（hash 位模式判定） |
| **M4 软体/布** | XPBD + 与刚体共求解器 | 🟡 **主体**：XPBD 绳索/布（三组约束 + 自碰撞·点-边·自摩擦 + 撕裂 + 塑性）+ 面元气动 + 刚体反作用两腿 + 金样 `m4_cantilever`/`m4_flag`；**共求解器未做且已评审为不建议做**（`UNIFIED-SOLVER-REVIEW.md`） |
| **M5 液体** | PBF/FLIP 分层 + 刚体双向耦合 | 🟡 **主体**：PBF + FLIP（CPU 相位并行 + GPU 档）+ 刚体双向耦合 + 金样 `m5_dam_break`/`m5_float_box`（浮体平衡高度 `y=1.213774323`、`v_y→0`） |
| **M6 风/海洋/车辆/机械** | 面元气动 + 浮力 + 约束族 | ✅ **出口判据达标**：**三个金样全绿**（帆 `m6_float_sail` / 系泊 `m6_float_mooring` / 车 `m6_wheeled_car`），本地 + Linux CI 双通过；关节**可断裂**（`break_force` 阈值 → 永久断开）+ `Distance` 关节符号缺陷已修 |
| **M7 喷溅 + 收口** | 3DGS 三层用法（渲染桥/物理代理/隐式场）+ 全域收敛 | 🟡 **部分**：splat 隐式场 provider + 渲染桥 `export_splats`；三层用法余项与全域收口未做 |

> **「主体」= 能力已在树且带判据，≠ 出口达标**。每行的出口判据见
> [`docs/ROUTE.md`](docs/ROUTE.md) §7。
>
> **M6 的完成标准**（2026-10-17 达成）是「三个确定性金样陷阱存在、且全部在 Linux CI 上通过」。
> 车的金样的额外目的：**验证金样的跨平台可移植性** —— 它逼出了一个真结论：接触驱动的多体系统
> 无法做到逐位跨平台，只能要求**有界容差**（实测最大差 `1.2e-4`）。这条经验写进了
> [`docs/PHYSICS-INVARIANTS.md`](docs/PHYSICS-INVARIANTS.md) §2.4。
>
> **金样清单（13 项，全部在 CI 阻断）**：`col45` · `pile5` · `tower25`（Rapier 对照）·
> `m3_collapse` · `m4_cantilever` · `m4_flag` · **`m4_rope`** · `m5_dam_break` ·
> `m5_float_box` · `m6_float_mooring` · `m6_float_sail` · `m6_wheeled_car` · **`a1_acoustic_gold`**。
>
> **接下来做什么**：见 [`docs/PHYSICS-INVARIANTS.md`](docs/PHYSICS-INVARIANTS.md) §4.4 的
> 方向排序（VBD 块坐标下降 → 确定性产品化 → GPU 纯函数下沉），以及 `docs/CAPABILITIES.yaml`
> 的能力缺口登记表。

> 三轴标准（**兼容 · 性能 · 真实化**）、域矩阵与「怎么搞」以 `docs/ROUTE.md` 为准。
> **crate/模块/库的分类编目（含热路径标注）见 [`docs/CATALOG.md`](docs/CATALOG.md)**。

## 必读文档

| 你想知道 | 去哪 |
|---|---|
| **什么算物理 bug、怎么判、改代码必须遵守什么** | [`docs/PHYSICS-INVARIANTS.md`](docs/PHYSICS-INVARIANTS.md)（物理不变量宪法） |
| 系统要什么（需求/档位/验收） | [`docs/SPEC.md`](docs/SPEC.md)（V1.3，真源） |
| 某个能力在不在范围内、做到什么程度 | [`docs/CAPABILITIES.yaml`](docs/CAPABILITIES.yaml)（单一事实源） |
| 某条路线为什么选/不选 | [`docs/adr/`](docs/adr/)（0001–0007+） |
| 试过但没用的路线（别重做） | [`docs/EXPERIMENTS.md`](docs/EXPERIMENTS.md) |
| 当前还开着的问题 | [`docs/OPEN-PROBLEMS.md`](docs/OPEN-PROBLEMS.md) |
| 统一求解器该不该做 | [`docs/UNIFIED-SOLVER-REVIEW.md`](docs/UNIFIED-SOLVER-REVIEW.md)（评审结论：不建议） |
| 文档地图 | [`docs/INDEX.md`](docs/INDEX.md) |


## 结构（§1 crate DAG，禁止环）

```
crates/
├─ vxl-phys-core        数学(严格f32)/质量属性/配置/SoA体数据/调度抽象/互操作 trait   [零外部依赖]
├─ vxl-phys-broad       宽相：均匀空间哈希网格（M1 增量 BVH bvh2 风格）
├─ vxl-phys-narrow      窄相：SAT 凸-凸 + 参考面裁剪流形 + 球解析 + 高度场特化 + GJK/EPA 外壳 + 三角网
├─ vxl-phys-solver      顺序冲量(TGS-Soft 同族) + 并查集分岛 + 休眠
├─ vxl-phys-integrate   半隐式欧拉 + 固定步/子步                      [骨架]
├─ vxl-phys-field       力场注册表（重力/风/吸引）                    [骨架]
├─ vxl-phys-terrain     高度场账本（挖掘/包围盒）+ 稀疏体素 + 三角网
├─ vxl-phys-replay      状态哈希（FNV-1a→xxh3 可换）+ 回放记录器
├─ vxl-phys             门面 World（默认管线组装 + 体素/网格/喷涂/软体/流体的接入与判据）
├─ vxl-phys-soft        **XPBD 绳索/布**（结构·剪切·弯曲 + 自碰撞/点-边/自摩擦 + 撕裂 + 塑性（含弯曲对）+ 面元气动 + 刚体反作用两腿）
├─ vxl-phys-fluid       **PBF / FLIP 分层**（密度约束 + 网格投影 + 粒子回投）+ 边界双向 + 相位并行 + GPU 档
├─ vxl-phys-splat       3DGS 隐式场（σ 场 → SDF）+ `ProviderColliders` 三点查 + 渲染桥 `export_splats`
├─ vxl-phys-destruction Voronoi 预断裂 + 运行时切割 + 体素块→刚体（`extract_boxes`/`spawn_box_debris`）
├─ vxl-phys-gpu         wgpu 后端：流体全程 GPU 档（千万粒）+ 宽相/窄相上卡 + 默认**格序副本档**（2b 亦开）
├─ vxl-phys-aero        面元气动力（逐面力/力矩 + 线性升力）
├─ vxl-phys-wheeled     地面行驶域骨架（悬挂/轮胎/传动）                [骨架·M2]
├─ vxl-phys-marine      浮力采样/波浪骨架                             [骨架·M2+]
├─ vxl-phys-mech        齿轮/皮带/活塞/马达骨架                       [骨架·M2+]
├─ vxl-phys-memfind     内存子串（标量基线）
└─ vxl-phys-ffi         C ABI 批量接口约定
```

> **状态标注口径（2026-10-17 实测）**：无标记 = 有实现且在树带判据；**[骨架]** = 只有参数/接口壳
> （`mech` 34 行 / `marine` 29 / `integrate` 246 / `field` 81 / `wheeled` 92）；`[Mx]` = 计划里程碑。
>
> **规模**：23 个 crate，`crates/**/src` 合计 **96,404 行** Rust。最大三块是
> `narrow`（8,162）/ `gpu`（6,375）/ 门面 `vxl-phys`（6,260）。

依赖注入：`BroadPhase` / `NarrowPhase` / `ForceField` / `StateHash` / `PhysGpuBackend`
均为 trait，自定义实现不改核心。默认最小管线 = core+broad+narrow+solver+integrate+island。

## 快速开始

```bash
# 全量测试（含确定性/稳定性验收测试）
cargo test --workspace --release

# 13 项金样（本地等价物；CI 用同一个脚本当唯一配方来源）
bash scripts/gate_gold.sh

# 28 道本地门禁（打印 LOCAL_RC=0 才算过）
python -X utf8 scripts/local_gate.py

# 单个金样：跑 180 tick 的车，打印位模式读数
cargo run --release -p vxl-phys --example m6_wheeled_car -- 180

# M0 出口门槛：1 万静态+1 千动态 ≥60Hz + 健康 + 双跑哈希 + arena 平稳（JSON 报告）
cargo run --release -p vxl-phys --example m0_gates

# 确定性验证：10 轮 × 600 tick 全等（不一致 = 退出码 1）
cargo run --release -p vxl-phys --example determinism

# 词汇禁令扫描（裁决 61）
bash scripts/vocab_scan.sh .

# 并发原语 loom 模型（可选本地跑；CI 门之一）
RUSTFLAGS="--cfg loom" LOOM_MAX_PREEMPTIONS=3 cargo test -p vxl-phys-core --test loom_primitives
```


## CI 现在守什么（21 个 job，2026-10-17）

一次 push 跑 **21 个 job**，任一失败即阻断合并（「汇总（提交门）」把它们的结论聚合）：

| 组 | job | 守的东西 |
|---|---|---|
| **编译矩阵** | `MSVC` / `GCC` / `Clang+LLD` / `GCC-aarch64(qemu)` | 三编译器 × 两架构全部能编过；`clippy -D warnings` 在每份上单跑 |
| **确定性** | 「确定性」×4 + 「跨平台哈希比对」 | 同配置逐位可复现；跨编译器/跨架构哈希一致 |
| **内存/并发** | `miri`（core 子集）/ `loom` / `TSan` / `ASan` | UB、数据竞争、越界——静态分析抓不到的用这些兜 |
| **物理正确** | `金样 (Rapier 对照)` / `测试 (debug 档)` / `真实性基准` / `感官输出` | 13 项金样 + `debug_assert` 档全量测试 + 真实度基准 |
| **静态纪律** | 「静态」×3（依赖/文档/文本）+ `棘轮门禁` | `cargo deny` + 强化 lint + 词汇禁令 + god 门棘轮（只准降） |
| **性能** | `规模档 (10 万+10 万)` / `覆盖率` | 大场景不退化；覆盖率报告 |

**本地全量等价物**：`python -X utf8 scripts/local_gate.py`（28 道门，打印 `LOCAL_RC=0`）。
门禁脚本一律要 `python -X utf8` + `PYTHONUTF8=1`。


## 确定性承诺（§5）

- 固定 60 Hz + 可配子步；渲染帧率与物理解耦；
- 严格 f32：`#![forbid(unsafe_code)]`、无 fast-math、无平台 intrinsic、归约有序
  （岛内约束按 (a,b,点序) 固定排序；跨岛顺序无关）；
- 状态哈希按 `f32::to_bits` 有序归约（M0 FNV-1a 64，接口可换 xxh3）；
- 跨平台矩阵（x86_64 × {MSVC,GCC,Clang} × {O0,O2,O3,LTO} + aarch64）在 CI 中逐步开启，
  哈希不一致 = 阻断提交。

## 提交门（§11）

- **单元 + 集成 + 属性测试**：`cargo test --workspace --release`；
- **13 项金样**：3 项与 Rapier 同场景逐帧位姿对照（`col45`/`pile5`/`tower25`）+ 10 项
  vxl-only 场景（`m3`–`m6`），全部在 CI 阻断。金样分两类判据：
  - **位模式**（收敛型：浮体、布、绳索、坍塌）—— 要求 `f32::to_bits` 完全相等；
  - **有界容差**（接触驱动的多体系统：车）—— 实测跨平台最大差 `1.2e-4`，取 `1e-3` 留 8 倍余量。
    为什么不要求位模式，见 [`docs/PHYSICS-INVARIANTS.md`](docs/PHYSICS-INVARIANTS.md) §2.4。
- **性能报告字段** = 每帧物理 ms / 实体数 / 并行扩展比 / GPU 加速比 / 汇编收益
  （`docs/RECIPES.md` 有全部基准的复现命令）；
- **Criterion 基准** 随 `cargo bench`（需 crates.io 网络）。

## 许可

MIT OR Apache-2.0（依赖审计禁 GPL 传染）。

## 本机注意事项

- **仓库路径**：`D:\KF\BSHSQ`（ASCII）。历史上仓库在 `D:\开发`，非 ASCII 路径会让
  MinGW（`x86_64-pc-windows-gnu`）链接器无法解析目标文件路径，因此 `.cargo/config.toml`
  把 `target-dir` 固定到 `C:/vxl-wl-target`。
  **该规避的理由已随迁移消失**（2026-09-30 实测：`CARGO_TARGET_DIR=<仓库>/target`
  `cargo test -p vxl-phys-integrate` 在本机 GNU 工具链下编译+链接全过）；现在留着这一行
  只是因为 ① C 盘已有 7.0 GB 热缓存 ② 三个门脚本硬编码该路径。删它 = 一次全量重建，
  属可选项，不再是必需项。
- M0 零外部依赖（离线可构建）；xxh3 / Criterion / proptest / Rapier 金样 /
  wgpu 随 M1+ 引入（需要 crates.io 网络）。

