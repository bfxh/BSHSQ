# ROUTE —— 多域统一物理引擎路线（三轴标准：兼容 · 性能 · 真实化）

> 定位：本文件是**路线的顶层口径**，回答「要什么、标准是什么、怎么搞」。
> 与既有文档的关系：`SPEC.md` 的 §3（性能档）/§4（真实度档）/§6（并发）**继续有效**
> 且是本文件三轴标准的具体数字来源；本文件新增的是**互操作层（兼容轴）**、
> **域清单与耦合矩阵**、**修订后的里程碑顺序**；与外部 `V2 §11` 的差异处已标注。
> 状态：路线已落定，M1（刚体核心）在研；接口层未建（见 §6 差距）。

## 1. 一句话

**把所有物理域收进一套共享基座**（状态 / 表示 / 接触 / 求解 / 步进 / 场景 六层），
每个域既能独立「搞到极限」，又能与**任意**域双向耦合；三轴标准作为可判定验收。

## 2. 三轴标准（每轴都有判据，不许形容词）

### 2.1 兼容（Interop）

- **判据**：任意两域耦合**不得有域内特例**——只允许通过四个共享接口完成：
  | 接口 | 职责 | 方向 |
  |---|---|---|
  | `CollisionProvider` | `closest_point` / `penetration` / 流形生成 | 域 → 接触层 |
  | `MediumField` | `sample(x) → {ρ, v, μ, T}` / `deposit(x, p, Δm)` | 域 ↔ 域（介质） |
  | `ConstraintElement` | 统一的约束元素（求解器唯一消费形态） | 域 → 求解层 |
  | `StateBridge` | 表示转换与导入导出（渲染/持久化/回放） | 域 ↔ 外界 |
- **交付物**：§4 兼容矩阵**每个格子**都有 ① 可运行示例 ② 确定性测试（哈希门）。
- **反模式**（明确禁止）：`if domain == Fluid { ... }` 式的域内特判；跨域直接改对方
  内部数组；每域自带一套求解循环。

### 2.2 性能

- **判据**：`SPEC §3` 档位表（按域扩表）+ 身份场景集 + **劣化 >10% 阻断**（SPEC §12.1）。
- 每域必须给：① 单线程确定（哈希逐位）② 多线程扩展 ≥3×/8 线程（同构指标：
  小岛域看岛级、粒子域看粒级/分块）。
- **记账口径**（本仓既有方法，继续沿用）：相位级（宽相/窄相/求解/耦合/导出）+
  「固定成本 vs 规模成本」拆解（见 M1-PLAN §12 的做法）；验收数字必须同机同状态
  同窗口复测（环境波动可达 ±28%，M1-PLAN §9②）。
- **性能红线**：任何新域不得让已通过档位劣化 >10%（持续回归，SPEC §12.1）。

### 2.3 真实化（Realism）

- **判据**：每域 = 技术选型（定案）+ 客观参数档（`SPEC §4` 已建）+ **金样对拍**
  （本仓 `gold-sample` 机制）+ 守恒/解析校验（能量/动量、解析解、基准实验）。
- 「真实度」一律落成**三元组（技术 + 参数 + 代价）**，延续 SPEC §4 的写法；
  模糊词（"低/中/高"）只允许作为档位名，参数必须写全。
- 每新增能力，先写**对拍场景与容差**，再写实现（否则无法判定"真实"）。

## 3. 域清单（谁要进来、什么表示、什么技术）

| 域 | 表示 | 求解技术（定案/待定） | 现有骨架 |
|---|---|---|---|
| 刚体（凸体/多边形/复合） | 凸体 + trimesh + 高度场 | 顺序冲量 → TGS 族（SPEC §2.5）；GJK/EPA + SAT | ✅ M1 在研 |
| 体素（可破坏地形/建筑） | 稀疏体素 + SDF | 体素块→刚体/粒子（Voronoi 预断裂 + 运行时切割） | `terrain` 账本 + **`voxel::VoxelVolume`（占据位图 + 局域 SDF + `CollisionProvider`，已落地）** |
| 软体 | 粒子 + XPBD 距离/体积 | XPBD（SPEC §4.6） | `vxl-phys-soft` 参数骨架 |
| 布料 | 三角网 + XPBD 三组约束 | XPBD + 面元气动（SPEC §4.7） | 同上 |
| 液体 | 粒子（SPH/PBF）/ 网格（FLIP） | WCSPH → PBF/FLIP 分层（SPEC §4.8） | **`vxl-phys-fluid` WCSPH 已落地（0.3 切片 1：CPU 档驻留/体素边界/确定性；PLAN-0.3 §4）** |
| **高斯喷溅（3DGS）** | 各向异性高斯集合 | **三层用法（本文新增，§3.1）** | ❌ 未建 |
| 风/气动 | 面元 + 速度场 | 面元气动力 + 可选尾流（SPEC §4.7/§4.10 同族） | `vxl-phys-aero` 骨架 |
| 车辆 | 射线悬挂 + 刷子轮胎 | SPEC §4.10 | `vxl-phys-wheeled` 骨架 |
| 机械/关节 | 约束族（铰链/齿轮/绳…） | SPEC §2.5 | `vxl-phys-mech` 骨架 |
| 海洋 | 高度场/波浪 + 浮力采样 | SPEC §4.10 同族 | `vxl-phys-marine` 骨架 |

### 3.1 高斯喷溅进物理的三层用法（新技术选型）

3DGS 的本质是「一堆各向异性高斯 + 不透明度 + SH 颜色」。进物理有三条正交用法，
**按依赖顺序落地**：

1. **渲染桥（`StateBridge`）**：物理状态（粒子/软体节点/顶点）→ splat 集导出，
   供渲染消费；反向导入用于"从扫描场景起步"（splat → 初始粒子/碰撞代理）。
   代价低、价值高（消费方最常用），**先做**。
2. **物理代理（粒子域）**：splat = 各向异性粒子（质量 + 半径 + 取向）⇒
   可当**颗粒/流体/软体**粒子求解（"高斯粒子"）；碰撞用各向异性椭球近似
   （GJK 椭球支集或 SDF 求和）。
3. **隐式场（`CollisionProvider` / `MediumField`）**：高斯和 ⇒ 密度场/SDF
   （解析、可求导）⇒ 作为刚体/布料的接触提供者，或直接作为 SPH 的核。
   **数学桥**：SPH 核（poly6/Gaussian）与 splat 同构 ⇒ splat 场可直接充当
   流体的密度/压力源（双向耦合的天然接口）。

## 4. 兼容矩阵（必须成立的耦合，每格 = 示例 + 测试）

| ↓作用体 / →受体 | 刚体 | 体素/地形 | 软体/布 | 液体 | 喷溅 | 风/气动 |
|---|---|---|---|---|---|---|
| **刚体** | 接触（已建） | 接触（Provider）| 共享求解器接触 | 浮力/阻力（介质场）**← 液体→刚体这半 2026-09-22 已落地（示例 `fluid_buoyancy` + 测试 `fluid_coupling`）；2b 双向那半同年落地（`fluid_boundary`）；附加质量待补** | 代理/场接触 | 气动面元（双向） |
| **体素** | 块→刚体（破坏） | — | 支撑/穿刺 | 体积/水床 | 场共享 | — |
| **软体/布** | 共享求解器 | 支撑 | 自碰撞/自身 | 湿布（质量+阻力，双向） | 代理 | **面元气动**（SPEC §4.7） |
| **液体** | **双向 ✅（2b 落地 2026-09-22：Akinci 两层边界粒子；示例 `fluid_buoyancy 2b` + 测试 `fluid_boundary`）——精度 **0.87–1.52×ρVg**（Akinci 自洽体积标定后；残余=细分辨率下的波动，见表），记账见 `M1-EXIT.md` §4 + `OPEN-PROBLEMS.md` P7** | 地形/侵蚀（后续） | 双向（同通道） | 自碰撞 | 核同构（§3.1-3） | 风驱动表面 |
| **喷溅** | 场/代理 | 场 | 代理 | 核同构 | 聚类 | — |
| **风** | 气动 | — | 主耦合 | 表面驱动 | — | — |

**必须有的贯通示例（每个都是一条 CI 场景）**：浮箱（刚-液）、旗飘（布-风）、
落水布（布-液）、挖洞坍塌（体素-刚）、锥堆（刚-刚 已建）、扫描场景起步
（喷溅-刚/布）、溅水（刚-液-风）。

> **⚠️ 2026-10-05 复核：矩阵里"还没落地"的格子（别被上表的 ✅ 稀释）**
> - **布 × 液（湿布：质量+阻力、双向）** —— 🔶 **阻力 + 双向两半都已落地（2026-10-05）**：
>   **介质 → 布**：门面在水推进后按**面心**采样 `FluidSystem` 的 `MediumField` 填进 `cloth.medium`，
>   `predict` 里 `cloth_medium::inject` 施加 **Bridson 线化阻力** `F = ½·ρ·Cd·A·u·|u|`（与
>   `cloth_aero` 同式；**必须在 `predict` 内**，见下面的试刀记录）。
>   **布 → 介质**：`predict` 的每个子步把**反作用冲量** `−F(s)·h`（与受力同一份公式）累加进
>   `cloth.medium_reaction`；门面每 tick 末 `world_soft_reaction::cloth_medium_reaction` 读走并
>   沉积回流体（`MediumField::deposit`：按同一 poly6 权重分摊，**`Σ m·Δv` 严格守恒**）。
>   ⇒ 反作用用的是**整 tick 的精确冲量**，不是"按 tick 末速度重算一次"的近似；冲量是**逐 tick
>   累加器**，门面读完**原地清零**（长度不变 ⇒ 无重分配）。
>   判据：`crates/vxl-phys/tests/wet_cloth_gap.rs`（干对照自由落体 vs 湿布被拖住、落地明显更晚
>   且不穿透槽底）+ `tests/wet_cloth_two_way.rs`（布沿 +x 拖水；**定量**：`Δp布 + Δp水 ≈ 0`，
>   实测残差 1.2e-5 / Δp 0.806，**干对照恒 0**）+
>   `vxl-phys-soft/tests/cloth_medium.rs`（介质流带起布 / 空与 `density=0` 逐位不变）+
>   `vxl-phys-fluid/tests/medium_deposit.rs`（沉积动量守恒 / 真空逐位不变）。
>   **湿质量**（吸水后有效质量，2026-10-05 落地，**显式开关默认关**）：`cloth.medium.wet_mass`
>   开启后每个子步从逐面 `occupied` 重算逐顶点质量倍率 `m_eff = m·(1 + κ·wet)`（κ = 0.5）
>   并**从干质量 `mass` 重算 `inv_mass`** ⇒ 离水可逆、`occupied = 0` 时逐位还原 `1/m`；
>   判据 `vxl-phys-soft/tests/cloth_wet_mass.rs`（干对照 `inv_mass` 逐位相同 / 全浸没每个顶点
>   `inv_mass` 下降 / 同样阻力下更慢）。**为什么默认关**：吸水是**质量转移**，本实现不把被吸走的
>   水的动量记回流场 ⇒ 布-液**动量账会漏**（实测残差从 `-1.2e-5` 变 `+0.50`）；要默认开得先补
>   "吸收动量"那条通量。**仍缺**：per-substep 采样（现为每 tick 一次，子步内复用）；
> - **溅水（刚-液-风）** —— 🔶 **三域编排已落地（2026-10-05）**：判据
>   `crates/vxl-phys/tests/splash_scene.rs`（水槽 + 2b 流体 + **落球推水** + **静态风帆**：
>   ①柱区峰值 `|v| > 0.3`（刚×液）②风帆读数 = `½ρCdA|w|²` 解析值 ±1e-3（刚×风）
>   ③三域同帧两遍 `state_hash` 相同 ④开气动域不改落球轨迹 —— **域间正交**）。
>   ⚠️ 球体（`Shape::Sphere`）**拿不到气动力** —— `aero_pass` 只对 `TriMesh` 施力 ⇒ 场景里
>   刚×风那半由**独立风帆**承载，不是"球被风吹"；
> - **风 × 液（表面驱动）** —— ❌ 未落地（要气动-液面耦合；本片只做到"风与液同帧共存、互不干扰"）；
> - **风 × 布（旗飘）** —— ✅ **门面已接通（2026-10-05）**：`World::set_aero` 把同一份风配置
>   **下发**到每张布（`cloth.aero`），布的 `predict` 逐子步按面心施加面元力。判据
>   `crates/vxl-phys/tests/cloth_wind_scene.rs`（竖直旗面顶边钉住：**无风对照**自由节点
>   `mean_x ≡ 0` 且 `aero.enabled == false`；**有风**吹向 `+x`）。此前 `cloth.aero` 只有软体
>   crate 内部测试、**门面不接** ⇒ "旗飘"进不了 CI 场景；
> - **喷溅 × {刚/布}（代理/场）** —— 部分：splat 作为 `CollisionProvider`/`MediumField` 已落地；
>   **"扫描场景起步"（splat→初始粒子）已跑通（2026-10-05）**：`export_splats` 出核中心 →
>   `vxl_phys_fluid::fluid_access::from_positions` 按给定位置建流体（质量仍按静止晶格
>   `ρ0/Σ_lattice W` 标定 ⇒ 与晶格块同口径），判据 `crates/vxl-phys/tests/scan_startup.rs`
>   （粒子数 = 扫描点数、逐值落在扫描点上、装进 `World` 推进健康）；
> - **② 表示层 `StateBridge`** —— ✅ **两个真实现（流体 2026-10-05 / 布片 2026-10-06）**：此前该 trait
>   全仓**零实现**（`import_positions` 的默认实现就是 `false`）。**流体**：
>   `FluidSystem` 实现它：
>   `export_positions` 按索引序只写**流体段**（2b 边界粒子不算流体状态）、
>   `import_positions` 长度必须**恰好**相符（否则拒绝且一字不动），成功后**同时清速度**。
>   判据 `crates/vxl-phys-fluid/tests/state_bridge_roundtrip.rs`（长度不符拒绝且逐位不动 /
>   导入后位置逐值相等且速度归零 / **往返逐位幂等**）。**布片**：`ClothSheet` 同口径实现
>   （`kind()` = `Mesh`；导入时**连 XPBD 的 `prev` 一起对齐**——只清速度不齐 `prev` 会让下一
>   `step` 读出巨大隐式速度），判据 `crates/vxl-phys-soft/tests/cloth_state_bridge.rs`（同一套 5 条
>   + 一条 `prev` 对齐）。⇒ "扫描起步"只差**splat→粒子的转换**那一步；
> - **提供者对偶（provider×provider）** 与 **高度场 × 提供者** —— 已登记为**不受理**，
>   现状由 `crates/vxl-phys/tests/provider_pair_gaps.rs` **钉住**（做对偶解法那天那两条会红 ⇒ 翻面）；
>   影响目前为零（都是静态 Marker ⇒ 求解器本就不产约束）。
>
> **⚠️ 2026-10-05 落水布（布×液）试刀记录（负结果 + 正确的接线形状）—— 已按第 3/4 条落地，
> 见上一条；本条留作"别再重复踩"的记录**：
> 1. **介质通道是通的**（实测）：门面每 tick 在**面心**调 `FluidSystem::sample`，180 次调用里
>    **165 次 8 个面全部命中**（ρ≈970、流速可读），只有入水前 15 tick 为 0；
> 2. **但"门面按 tick 直接给 `cloth.vel` 加阻力冲量"无效**（实测：穿液面 `vy = −2.124`
>    —— 与自由落体 `√(2g·0.22) ≈ 2.08` 同值；加上阻力后**一位没变**）。机制是 XPBD 的
>    **位置式**速度回写：`predict` 里 `prev = pos; pos += vel·h`，`write_back` 再用
>    `(pos − prev)/h` **重算 `vel`** ⇒ **步前/步后的速度级注入都会被吞**；
> 3. ⇒ 正确形状（下一刀）：力必须进 **`predict` 内部**（与 `apply_aero` 同段位）。建议
>    `ClothSheet` 加**一个** scratch 字段 `medium: Vec<MediumSample>`（长度 = `tris.len()`；
>    **空 = 关 ⇒ 默认档逐位不变**），门面每 tick 先按面心采样填它，`predict` 里调
>    **自由函数**（不是方法——类型账按方法数只准减，`ClothSheet` 36 已顶）把 `F/m·h`
>    在 `prev = pos` **之前**加进 `vel`；
> 4. 棘轮账（先算好再动手）：`cloth.rs` 加 1 字段 = 23→24 顶格 ⇒ 同文件**最长函数须降 ≥1 行**
>    （≤10% 窗口 507 → 557 够用）；`soft/lib.rs` **零余量**（25 行、`max_fn=0`）⇒ 新模块走
>    `#[path]` 子模块，或先在 `lib.rs` 腾出 1 行。

## 5. 架构（共享基座六层 + 每域一插件）

```
⑥ 场景层   多域场景图 / 持久化 / 回放 + 状态哈希（已建 replay/哈希）
⑤ 步进层   子步 + 域顺序 + 双向耦合轮次 + 确定性规则（一个调度器，全域共用）
④ 求解层   ★统一约束元素（刚体接触/关节、XPBD 软体、粒子域、耦合约束）★
③ 接触层   统一流形（窄相）+ CollisionProvider 插件（体素 SDF/trimesh/喷溅场）
② 表示层   MediumField（介质采样/沉积）+ StateBridge（表示转换/导出）
① 状态层   SoA + 稳定 ID + 相位 arena + hot/cold（core/mem.rs 已建）
```

**关键设计决定（把「兼容」做成结构而非约定）**：

1. **一个求解器核心，多种元素**——④ 是唯一求解循环：刚体接触、关节、XPBD 约束、
   粒子域、耦合约束都是「元素」（统一索引空间 + 统一并行/确定性/成本模型）。
   好处：性能与确定性只需证明一次（三轴里的两轴都靠它）；新增域 = 新增元素类型。
2. **介质与接触是接口，不是调用**——②③ 的 trait 是跨域唯一通道（§2.1 反模式）。
3. **每域一个 crate**（`vxl-phys-<domain>`），实现四个接口 + 自带 gold-sample；
   依赖方向单向：域 → （core/interface），**域之间不得直接依赖**（耦合只走 ②④）。
4. **确定性规则统一**：跨域读写顺序、归约顺序、并行分块规则全域一致（SPEC §5/§6）。

## 6. 与现状的差距（诚实）

> **⚠️ 2026-10-05 复核：本表是 2026-09-14 的快照，第三列"差距"多数已不成立**（逐条对代码）：
> - "接口层（②③④ 的 trait）❌ 未定义" → **已定义**：`core::interop` 四 trait + ADR 0008/0009；
> - "soft/fluid/wheeled/aero/marine/mech/destruction/gpu 仅参数骨架" → **soft/fluid/aero/destruction
>   都已实现**（判据见各 crate 的 `tests/`，`CATALOG.md` 域层表已按此更正）；
>   **仍是参数骨架**的只有 `wheeled` / `marine` / `mech`（且**无消费方**）；
> - "高斯喷溅 ❌ 完全未建" → **已建**（`vxl-phys-splat`：隐式场 provider + `MediumField` + 渲染桥）；
> - "域骨架 无求解器实现" → 液体/软体/布/气动/破坏都已有求解器；真实差距是**性能档**与
>   **上表列出的几条耦合格子**（见 §4 的复核块）+ `OPEN-PROBLEMS.md` 的待裁决 6 条；
> - "多域场景/哈希/回放 需扩到多域同一哈希域" → 已扩（`replay`/状态哈希 + 多域示例与 CI 门禁）。
> ⇒ 本表**保留为历史**，判断现状请以 `CATALOG.md` + 各 crate 的测试为准。

| 项 | 现状 | 差距 |
|---|---|---|
| 刚体核心（M1） | 在研：稳定/金样多数达标 | **性能档（SPEC §3 30 FPS）缺口 ~9-10×**；架构三缺陷（DESIGN-staged-solver §7）；子步/廉价子步已证伪（§10）；下一步 warm 缓存槽位化 |
| 接口层（②③④ 的 trait） | ❌ 未定义 | 「兼容」的最小前置：先写 trait 草案 + 迁移现有窄相/地形为 provider |
| 域骨架 | soft/fluid/wheeled/aero/marine/mech/destruction/gpu 仅参数骨架 | 无求解器实现 |
| 高斯喷溅 | ❌ 完全未建 | 需新增 crate + §3.1 三层用法 |
| 多域场景/哈希/回放 | 单域已建（replay/状态哈希/CI 门禁） | 需扩到「多域同一哈希域」 |

## 7. 修订路线（先接口 → 后域 → 每域对拍）

| 里程碑 | 内容 | 出口判定（三轴同时） |
|---|---|---|
| **M1 刚体核心（在研）** | 稳定/金样/性能收口；求解器成本结构（见 DESIGN/M1-PLAN） | 稳定判据 ✓ + 金样容差 + 性能档（§3 数字）或核口径裁决 |
| **M2 互操作核心** | 四接口 trait 定案 + **外部碰撞提供者通道（`Shape::Provider` + `ProviderColliders`，ADR 0009）** + **贯通示例（刚体↔体素）✅ 已跑通**；余项**逐条定性**（2026-10-06 复核）：① ~~高度场迁到 provider 通道~~ **已结（2026-10-06）**——**不搬所有权**，把 provider 的**速度自适应接触带**直接做进**直连高度场路径**（`OPEN-PROBLEMS.md` #6 已落地；"整体搬进 `Providers`"那条形状**被否证**：`World::terrain` 是公开字段、`dig` 在原地改它 ⇒ 搬走会让**挖洞对窄相不可见**）② 球/凸体 provider 专用查询 —— 球已解析、凸体走**逐顶点采样**（既定口径，无实测需求）③ provider 对偶 —— 提供者都是**静态 Marker** ⇒ 静态×静态零收益（已登记缺口）④ 「键图 + 双 ABI」 —— 键图 = `EffectKey` + 转换窗口**已落地**（`PLAN-COUPLING.md` §3.4）；双 ABI = `vxl-phys-ffi` 的 C ABI 骨架（版本化头已就位）+ GPU 档（范围见 `PLAN-gpu.md`） | 接口无域内特例 + 示例确定性测试 + 无档位劣化 |
| **M3 破坏/地形（体素）** | Voronoi 预断裂 + 运行时切割 + **体素块→刚体（✅ 已落地：`extract_boxes` + `spawn_box_debris` + `apply_impact_destruction` 冲击破坏 + 演示 `m3_impact`）** | ~~坍塌金样~~（✅ **2026-10-07 落地**：`m3_collapse` 金样 = 预断裂整块 → 碎块下落全睡，进 `gate_gold.sh` 与本机/CI 阻断门，顺带三编译器哈希一致；**10 万碎片档**仍是参考机上的规模实验）；~~沙堆模型~~（❌ **不做（用户口径）**：体素→粒子沙/尘 —— 见本文件 §8「⚠️ 残留」段与 §2026-09-14 表、`PLAN-0.2.md` §L —— **别再据此立项**） |
| **M4 软体/布** | XPBD + 与刚体**共求解器** —— **2026-10-05 状态**：XPBD（布/绳）+ 提供者/刚体接触、自碰撞/自摩擦、撕裂/塑性、气动升力**均已落地**（各自默认关，判据 18 个测试文件）；"共求解器"按字面**仍未做**（现状 = XPBD 子步 + 反作用耦合）；✅ **体积/气压约束已落地（2026-10-07，`cloth/volume.rs`：Müller 2007 气压 + XPBD 投影，默认关）**；✅ **点-边对自摩擦已落地（2026-10-07，`cloth_edge_friction.rs`：与点-点自摩擦同一条库仑锥、同一个 `μ`）**；**面元力矩 / GPU 档仍缺** | 悬臂/旗飘金样 + 刚度档表（SPEC §4.6/4.7）**仍缺** |
| **M5 液体** | WCSPH → PBF/FLIP 分层 + 刚体双向（Akinci） | 溃坝/浮箱金样 + 不可压误差档 + 30 万粒档 |
| **M6 风/海洋/车辆/机械** | 面元气动 + 浮力采样 + 约束族（可断裂/限位） | 帆/浮体/车 金样 + SPEC §4.10 参数档 |
| **M7 喷溅 + 收口** | §3.1 三层用法 + 全域性能与真实化收敛 | 兼容矩阵全格绿 + 档位表全绿 + 持续回归建立 |

> 与 `V2 §11` 的关系：本表**替代**其里程碑顺序（新增「M2 互操作核心」为先，
> 域顺序改为 体素 → 软体/布 → 液体 → 风/车/机 → 喷溅）；SPEC §3/§4 的**数字不动**。
> 外部 V2 文档需按本表同步（待用户侧更新）。

**破坏路径的修复与残留（2026-09-14 两轮实测，如实）**：
- ✅ **冲击判据 = 沿接触法向的接近速度**（不是体速！）——按体速判会让「贴地滑行」
  触发破坏、把体脚下的地板一路挖穿（实测踩中）。
- ✅ **provider 接触带按相对速度自适应**（`band = max(skin, |v_rel|·dt·1.5)`）——
  固定 0.02 m 皮肤带小于每 tick 位移时，体**跨过皮肤带**（8 m/s ⇒ 0.13 m/tick），
  等到进入体内才建接触、接近速度已≈0 ⇒ 冲击不触发（实测：弹体无声停在墙前 0.04 m、
  零破坏）。
- ✅ **盒接触自适应查询范围**（先收集盒 AABB ±1 格内的占据格再求最近距离）——
  修前「采样点周围 ±1 格」让大碎块找不到最近格。
- ✅ **`fill_box` max 开区间**（闭区间会多填一格 ⇒ 场景「地板比预期厚一格」）。
- 修复后实测：8 m/s 弹体 → 首次命中 tick 12、挖 16 格、1 碎块、**逃逸 0**、场景干净；
  20 m/s → 同样干净但打出的碎块可能高速飞出（逃逸 1/1，属边界外飞行）。
- ✅ **CCD × 静止接触互锁（已修）**：命中判据细化为「**只有沿法向正在接近的
  采样才算命中**」——贴地滑行/静置的体在每个采样都天生有接触，按「有接触即命中」
  会被钳回起点、原地锁死（实测：弹体滑到墙前 0.1 m 停住）。回归测试
  `ccd_does_not_lock_sliding_body`（开 CCD + 8 m/s 贴地滑行 ⇒ 60 tick 位移 > 2 m）。
  现有三条 CCD 测试（薄墙拦停 / 默认关 / 不锁滑行）全绿。
- ✅ **任意形状切割第一步（球域）**：`extract_where(谓词)` 通用化 + `extract_sphere`
  （球心判据的弹坑）+ 门面 `carve_sphere`；冲击破坏的挖域由「盒板」改为**球形弹坑**
  （球心 = 接触点 + 冲击方向 × 1.05r ⇒ 坑近缘正好落在接触点、整体在材料里）。
  测试：球域挖洞（移除格数 ≈ 球体积格数、球心空、角点留）+ 门面破坏/确定性全绿。
- ✅ **Voronoi 预断裂（体素版）**：`fracture_voronoi`（格归属最近种子 ⇒ 逐种子提取
  + **守恒**：分区恰好覆盖域内全部占据格，测试守门）+ `seeds_jittered`（确定性抖动
  种子，无外部 RNG）+ 门面 `World::fracture_voronoi` + 演示 `m3_voronoi`
  （墙 64 格 → 27 碎块、逃逸 0、干净）。脆性**凸体外壳**（非体素）的 Voronoi 需
  `Shape::ConvexHull` + GJK/EPA（SPEC §2.4），列为后续。
- ✅ **凸体外壳切割**（2026-09-15）：窄相 `gjk.rs` 增 `clip_halfspace`（半空间裁剪，交点
  收敛到切割面上的 2D 凸包，防 O(n²) 膨胀）与 `fracture_voronoi_hull`（最近种子分区，
  平铺性同体素版）。
- ⚠️ **残留**：体素块→**粒子**（沙/尘）——按用户口径**不做**。



## 8. 立即动作（怎么搞，按顺序）

1. **写接口**（`vxl-phys-core` 新增 `interop` 模块）：`CollisionProvider` /
   `MediumField` / `StateBridge` / `ConstraintElement` 的 trait 草案 + 文档级示例；
   现有窄相（SAT/高度场）与地形账本**先包一层 provider 实现**（不改行为，哈希守门）。
   - **状态（2026-09-14）**：✅ 四 trait 草案落位 `vxl-phys-core::interop`；`Aabb`
     下移 core（broad 再导出，调用点零改动）；首个真实实现
     `impl CollisionProvider for HeightField` 已落地（含 6 项测试）；
     哈希逐位不变（纯新增）。见 ADR 0008。
   - **未做**：现役窄相/地形「经 provider 路径」的迁移（M2 工作项，要求逐位不变）；
     `ConstraintElement` 的状态视图待 M2 与求解器接口一并定案。
2. **刚体侧继续**：warm 缓存槽位化（DESIGN-staged-solver §10）→ 复测 →
   再谈结构换挡/P4 岛内并行染色。
3. **体素域先起**（它同时是 provider 与破坏载体）：稀疏体素 + SDF provider +
   与刚体的贯通示例（一条 CI 场景）。
   - **状态（2026-09-14）**：✅ 表示层与 provider 已落地——`vxl-phys-terrain::voxel`
     （占据位图 1 bit/格 + 占据包围盒增量维护 + 局域 SDF + 有限差分法线；
     `impl CollisionProvider`；盒形 6 面×4 角专用查询 `contacts_box_voxel`；
     6 项测试：SDF 符号/表面/挖洞读空/盒落地 4 底面接触/包围盒跟踪）。
     哈希逐位不变（纯新增）。
   - **贯通示例 ✅ 已跑通（2026-09-14）**：`Shape::Provider(id)` +
     `interop::ProviderColliders`（ADR 0009）——盒经完整管线（宽相 AABB ← provider
     bounds；窄相 provider 分支 ← `contacts_box_voxel`；解算；入睡）静置于体素顶面
     并入睡（门面测试 `box_falls_and_rests_on_voxel_provider`）；既有形状路径一字未动
     ⇒ `m0_gates`/`determinism` 哈希逐位不变。
   - **球 ✅ 已扩（2026-09-14）**：`ProviderColliders::contacts_sphere` +
     体素解析解（`depth = r − sdf(center)`、法线取 SDF 梯度）——球落体素地面
     停驻并入睡（门面测试 `sphere_rests_on_voxel_provider`）；体素侧单测覆盖
     深度/法线/缝超 skin 无接触。
    - **余（M2 后续）**：~~高度场迁到 provider 通道~~ **已结 2026-10-06（见下条）**；
      凸体 vs provider 的专用查询（无实测需求）；provider 对偶解（静态×静态，零收益）；键图 + 双 ABI（键图已落地）。
    - **2026-10-06 结案（`OPEN-PROBLEMS.md` #6）——不搬所有权，搬"那条带"**：原题「高度场迁
      provider 通道」的**收益本质**是把 provider 那条**速度自适应接触带**带给地形；而"整体搬进
      `Providers`"这条形状**被否证**：`World::terrain` 是**公开字段**、`dig` 在**原地**改它 ⇒
      搬走会让**挖洞对窄相不可见**（`tests::solver_misc::digging_removes_support` 立刻红）。
      ⇒ 落地改做在**直连高度场路径**里（`heightfield_pair` 算 `velocity_band` 写 `ws.inflate`，
      `hf.rs`/`mesh_pair.rs` 统一读 `skin + inflate`），**所有权与公开 API 不动**；对拍
      `tests/hf_provider_dispatch_parity.rs` 三档现在**两条路逐位一致**（原"旁路 0 点"分歧闭合）。
      同批（issue #4）宽相 fat 边距改**各轴独立** ⇒ 本机 8B 宽相均 **9.66 → 7.09 ms（−27%）**。
     - **2026-10-04 第一刀（已落地，逐位不变）**：`impl CollisionProvider for HeightField` 补上
       **`contacts_box`**——**委托窄相同一份数学**（`poly_heightfield`，零公式复制），并新增集成测试
       `crates/vxl-phys-narrow/tests/heightfield_interop.rs` 证明 provider 面与窄相**实际路由**
       （`DefaultNarrowPhase::collide` 产出的流形）**逐位相同**（点/深度/特征/法线）。四哈希与
       `m0_gates` 复跑不变（新方法不在生产路径上，**派发尚未切换**）。
    - ⚠️ **迁移的真正卡点（本刀查明的接口缺口）**：per-domain 的 `CollisionProvider` **只有
      `bounds / closest_point / contacts_box`** —— **没有**「球 / 点」查询（那些在窄相侧的
      `ProviderColliders` 上，由门面的 provider 注册表实现）。所以"**高度场整体迁过去**"不只是
      "包一层"：要么给域 trait 补 `contacts_sphere` / `contacts_point`，要么让注册表能按域派发。
      ⇒ 下一步二选一，**先定接口再搬**（否则搬一半会得到两套形状面）。
    - **2026-10-04 第二刀（已落地，逐位不变）——接口缺口按「补域 trait」定案（上面的 A 案）**：
      `CollisionProvider` 补 **`contacts_sphere` / `contacts_point`**，两个默认实现都返回
      `false`（**不支持**）——口径与窄相侧 `ProviderColliders` 的同名方法**一字对齐**
      （`depth = −sdf(p)` 内点为正、`feature = 0`、返回值 = 是否支持）。这样"默认不覆写"
      在两套接口里含义相同，**不会出现隐式解**。`HeightField` 随即覆写两者：
      `contacts_sphere` 委托 `sphere_heightfield`、`contacts_point` 与 `poly_heightfield`
      及外壳的地形腿（`support.rs::hull_pair` 的 L1 分支）的逐顶点采样**同一条式子**；
      集成测试 `heightfield_interop.rs` 再加两条**逐位等价**判据——球（vs `Sphere` 体走完整路由）
      与点（vs **单顶点外壳**走那条 L1 分支；四项全比——live 路径的 `feature` 从 0 起，
      与 provider 点查询的 0 相同。⚠️ 2026-10-05 更正：本行原写"走 `hull_heightfield`、
      `feature` 有意不同"——`hull_heightfield` 是**死代码**（已删），实际走的是 `hull_pair`）。
      ⇒ 域 trait 与窄相接口**同口径**了，「注册表按域派发」这一步现在只是机械搬运。
      ⚠️ 体素侧 `VoxelVolume` 当时的域 trait 只有 `bounds / closest_point`（球/点查询与
      盒的专用解都只在门面那条路上）——**已由下一刀补齐**，见下条。
      god 门棘轮处理：`interop.rs` / `heightfield.rs` 的 `#[cfg(test)] mod tests`
      **原样外迁**到 `tests/`（`interop_default_contacts.rs` / `heightfield_prims.rs`，
      改走公开 API）⇒ 两个 src 文件行数**净减**，基线一字未动。
    - **2026-10-05 第三刀（已落地，逐位不变）——体素侧补齐域 trait + 收掉两处历史遗骸**：
      `impl CollisionProvider for VoxelVolume` 补 **`contacts_box` / `contacts_sphere` /
      `contacts_point`**，各**委托 `voxel_contacts` 的同一份数学**（零公式复制）。
      ⚠️ 不覆写 `contacts_box` 会退回 trait 的「8 角点采样」默认实现，与体素专用
      「6 面 × 5 采样」**不是一回事**——这正是"两套形状面"的具体形态。新集成测试
      `crates/vxl-phys/tests/voxel_provider_domain_equivalence.rs` 钉住
      **域 trait ≡ 门面注册表**（盒/球/点 × 命中/落空/预期接触；点·深度·法线·特征与
      返回值逐位比）。顺手清掉两处遗骸：
      ① SDF 有限差分梯度原在 `closest_point` / `contacts_sphere_voxel` /
      `contacts_point_voxel` 里**各抄一份** ⇒ 收成 `sdf_gradient_normal` 一处；
      ② `contacts_box_voxel` 的「主导面 `best` / `deepest`」是**死代码**（算完
      `let _ = best;` 直接丢掉——主导面选择早已移到窄相按**闭合速度**挑，见该函数注释）
      ⇒ 只留「6 张面全不在带内 ⇒ 早退」，该函数 **99 → 83 行**。
      行为逐位不变：`determinism` / `m0_gates` 哈希与 `m1_islands` 串行/并行末态哈希一致。
    - **2026-10-05 第四刀（已落地，逐位不变）——门面改按域 trait 转发**：
      `Providers` 的体素三臂（`contacts_box / contacts_sphere / contacts_point`）由
      「直接调 `vxl_phys_terrain::voxel::contacts_*_voxel`」改成 **`v.contacts_*`（域 trait 方法）**
      ⇒ 门面只剩「id → 实体」的派发，**同一个域只有一份接触数学**；新增域时门面不必再加臂
      （这正是 §5「跨域只走四接口」要的形状）。`providers.rs` 173 → 171 行。
     **`contacts_point_boundary` 有意不并**：它是**流体专属的第二张面**（刚体要"外点梯度"口径、
      流体投影要"内点最近真表面"口径），域 trait 上没有对应方法 ⇒ 留在门面直派并加注说明，
      免得被"看起来统一"塞进 `contacts_point`（那是两套语义）。
    - **2026-10-05 前置对拍（已落地，只新增测试）——「把高度场派发切过去」是不是机械搬运**：
      `crates/vxl-phys-narrow/tests/hf_provider_dispatch_parity.rs` 用**测试替身**
      （把 provider 面三种查询原样转给域 trait ＝ 迁移后门面要做的事）对拍两条装配路：
      ① **只要接触集合由 `skin` 决定，两条路逐位一致**——盒 × flat 高度场，静止 4 点 /
      12 m/s 下落 4 点，点·深度·特征·法线全 `to_bits` 相等；
      ② **差异只有一处**：provider 分支的接触带 = `max(skin, |v_rel|·dt·1.5 + skin)`
      （2026-09-15 修的"跨过皮肤带"），高度场旁路**恒用 `skin`**
      ⇒ "运动中、缝 0.2 m"这档：旁路 **0 点**、provider **4 点**（预期接触，深度 −0.2）。
      ⇒ **切换派发不是逐位中性**：等于把那条自适应带也带给地形；而 `determinism` / `m0_gates`
      的场景地面**就是 flat 高度场** ⇒ 切了必改哈希（属**换代**）。该决策已列进
      `OPEN-PROBLEMS.md` 待裁决表第 6 条，**不自行拍板**。
4. **每加一域的顺序铁律**：先写档位表行与金样（含容差）→ 再写求解器 →
   最后接耦合矩阵格子。**不许先写实现后补验收**。

## 2026-09-14 · 多边形 + 高斯喷溅落地（M3 扩展）

| 域 | 状态 | 入口 / 实现 |
|---|---|---|
| 多边形（凸体外壳） | ✅ | `add_hull` / `spawn_hull_body` / `spawn_hull_pieces`；窄相 `gjk.rs`（GJK/EPA + 半空间裁剪 + Voronoi 预断裂）；**外壳×提供者 = 顶点采样多点流形**，外壳×{盒/球/外壳} = EPA 单法线 + 外壳近面顶点细化 |
| 高斯喷溅 | ✅ | 新 crate `vxl-phys-splat`：隐式场 σ(p)=Σw·exp(−½α(p))、SDF `(τ−σ)/|∇σ|`、`ProviderColliders` 三点查；`add_splat_field` 接入；渲染桥 `export_splats` |
| 演示与对照 | ✅ | `examples/showcase`（**六域同场**：体素+多边形+喷溅+三角网+刚体+流体）→ 逐帧转储（VXLD v3，帧尾流体粒子节）→ `scripts/render_demo.py`（`--src/--dst/--dist/--ty/--label`）→ `docs/demo/showcase_full.gif`；`examples/dam_break`（塌坝专项）→ `docs/demo/dam_break.gif`；`gold-sample` 增「活跃 tick 计时」双引擎对照 |
| 网格（任意三角网） | ✅ | `vxl-phys-terrain::mesh::TriMesh`：薄壳语义（`depth = skin − 最近三角形距离`，法线 = 面法线），均匀网格邻域加速（格边 = 平均边长，下限 0.5）；门面 `add_mesh`；测试：盒静置 / 球沿坡面法线接触 |
| 液体 | ✅（0.3 切片 1） | `vxl-phys-fluid` WCSPH：poly6 密度（含自身项）/ spiky 对称压力梯度 / Tait γ=7 / Monaghan 人工黏度 + XSPH / 镜像鬼影边界密度；确定性均匀网格 27 邻域；门面 `add_fluid`/`fluid_pass`（单向耦合）；体素边界终版 `contacts_point_voxel_solid`；测试 7/7 + 四哈希不变；驻留瞬态负面结论见 EXPERIMENTS / PLAN-0.3 §4.2 |
| 软体 / 布 | ✅（**2026-10-05 更正**：本行原写 ⏳/"求解器待接"，已过时） | `vxl-phys-soft`：布/绳 XPBD、自碰撞（点-点 + **点-边** + **点-边对自摩擦**）、自摩擦、撕裂、塑性、**体积/气压（2026-10-07）**、气动（含**升力**）、粒子↔刚体耦合；判据 = 该 crate 的测试文件 + `vxl-phys/tests/{cloth_*,rope_*,aero_face_*}.rs`；**仍缺**：面元力矩、GPU 档、M4 出口金样（悬臂/旗飘）。详见 `SURVEY-SOFT-CLOTH-AND-CONVERSION.md` 顶部的状态更正块 |
| 体素→粒子（沙/尘） | ❌ **不做（用户口径）**（**2026-10-05 更正**：本行原写 ⏳/"下一个自然切片"，与明文口径冲突） | 见同文件 §8 的"⚠️ 残留：…按用户口径**不做**"与 `PLAN-0.2.md` §L 表前的同一句 —— 别再据此立项 |

**可视化对照**（2026-09-14 追加）：`gold-sample` 支持逐帧转储（第 9 个参数 = dump 路径），
`scripts/render_compare.py` 渲染左右同屏（同场景/同相机/同 tick，底栏各自 ms/FPS）⇒
`docs/demo/compare_full.gif`；速度口径与结论见 EXPERIMENTS「双引擎对照」节。
