# PLAN：体素↔多边形转换的物理四件（含 `StateBridge` 交接策略）

> 立项依据：`SURVEY-SOFT-CLOTH-AND-CONVERSION.md` §1.2（"四个子需求里，三个半是空白"）与 §2-T2 的四件拆分；
> `PLAN-COUPLING.md` §4.4（"转换物理四件……应在这里定稿"）与 §9 P2（"体素↔多边形转换四件"）；
> `PLAN-triangle-first-class.md:83`（"没有几何缺口挡路 ⇒ 想先出可见成果可以走它"）。
> **一句话**：把"体素 → 多边形"的表示转换做成一条**有物理交接**的通道——① 转换本身、
> ② 转换途中的物理、③ 交接策略（三档）、④ 预算与确定性降级——逐片默认关、判据先行、零代际。
>
> 状态：**立项（未动工）**。拍板项 P1–P5（§7）待回执后按 §5 顺序开工；本文件不含行为改动。

## 0. 结论先行（现状 → 本计划做什么）

- **现状**（§1 逐条取证）：四个子需求里 3.5 个空白。唯一现成的"跨表示"是**体素 → 贪心盒碎块**
  （`spawn_box_debris` 一族）与**网格 → 静态 provider**（`TriMesh`，无体素化）；`StateBridge` 声明零实现；
  "转换途中的物理"没有任何接口位置。
- **本计划**把它们收成一条通道：`world_step/conversion.rs`（新域文件）+ 契约卡（挂 `PLAN-COUPLING.md` §3
  同一张卡）+ 四片迁移（V1–V4），**V1（离线提取工具）零运行时面**。
- **与 M3 破坏域的关系**：这是碎块形态的升级路径——贪心盒 → 真网格；同时**盒提取充任"超预算降级档"**
  （§3.4），"降级 = 退化到现行口径"，保真度下界有现成对照。
- **本计划只管容量侧的"挤爆"**（求解器超载降级）；"物理被压塌/碎裂"属 M3（survey §2-T2 已把两义分开命名）。

## 1. 取证：四件逐条现状（全部 `文件:行`；复核日 2026-10-01）

### 1.1 转换本身（体素 → 网格）——零

- `marching / surface nets / isosurface / dual contouring / remesh / voxelize / polygonize` **全仓零命中**
  （本次复核：`rg -i` 于 `crates/`，输出为空）。`morph / prev_shape / handover / materialize` 同样零命中；
  `transition|blend` 全仓唯一命中是窄相高度场法线测试（`narrow/src/tests.rs:701`），与本维度无关。
- 现有跨表示路径**仅两条**：
  ① **体素 → 轴对齐盒碎块**（贪心合并 + 移格）：`voxel_volume.rs:204`（`extract_boxes`）、`:211`
  （`extract_sphere`）、`:232`（`extract_where`，通用谓词）、`:314`（`fracture_voronoi`）；门面
  `world_body.rs:28/:33`（`spawn_box_debris[_vel]`）、`:59`（`carve_sphere`）、`:78`（`fracture_voronoi`）、
  `:111`（`apply_impact_destruction`）。
  ② **网格 → 静态 provider**：`terrain/src/mesh.rs:83`（`TriMesh`：顶点 + 三角 + 均匀网格加速），**无体素化**。
- 体素侧表示（源形态）：`VoxelVolume`（占据位图 + 局域 SDF）——字段 `voxel_volume.rs:12-22`、
  SDF `:153`、格中心 `:128`、维度 `:133`；`CollisionProvider` 实现在 `voxel_provider.rs`（`bounds:5`、
  `closest_point:15`，SDF + 有限差分法线）。
- 网格侧资产（**目标形态已就绪**——T1 已落地，见 `PLAN-triangle-first-class.md` §分步表）：
  `Shape::TriMesh { mesh, half }`（`shape.rs:64-71`；非凸、逐顶点采样；逐域支持矩阵见
  `SURVEY-SHAPE-SUPPORT-MATRIX.md`）＋ `MeshStore`（`mesh_store.rs:22`；`add:31`、`shell_props:92`）＋
  门面 `world_mesh.rs:14`（`add_trimesh`）、`:72`（`spawn_trimesh_body`，薄壳质量 `m = ρ·t·ΣA`，口径注在 `:66`）。

### 1.2 转换途中的物理——零

- 两个表示**并存**的接口位置：没有。今天"挖下碎块"是两段式（先 `extract_*` 移格、后 `push_dynamic` 建体），
  中间没有窗口（`world_body.rs:44-53` 逐行可见）。
- 既有"交接"全部是**域间让位（二选一、不叠加）**：流体 2a/2b 让位集（`world_step/medium.rs`、
  `fluid_covered_multi` 的 C4 修复）、GPU 档整趟回退（`world_step/narrow_tier.rs`）——**不是**同一物质换表示的过渡。
- 影子/双表示对拍：无先例可抄（本仓"双查询对拍"最近的是金样门与 A/B 对拍，工具层有，引擎层无）。

### 1.3 交接策略（"要不要附上前几何形态的效果"）——零

- `StateBridge`（`interop.rs:256-268`）＝ `kind / export_positions / import_positions`，是**导出/导入面**
  （渲染、持久化、回放、扫描起步），零实现、只在注释里被提到（`vxl-phys-splat/src/lib.rs:6`）；
  `BridgeKind { Rigid, Voxel, Particle, Splat, Mesh }`（`interop.rs:76-89`）。
- 动量连续性现状 = **档 (i) 不继承**：`apply_impact_destruction` 的碎块**静止生成**，
  明写"引擎不凭空造动量——『继承半速』实测是能量源，已否"（`world_body.rs:154-155`）；
  `spawn_box_debris_vel` 的初速由调用方显式给（`:33`）。
- warm 残量继承：无。求解器 warm 槽按**接触特征号**跨帧匹配（`solver` 侧），换表示后特征全变
  ⇒ 需要"特征再映射"口径，本仓无（属未建）。

### 1.4 预算与确定性降级——局部有、全局无

- 局部上限先例（可参照的形态）：流体/网格加速结构的格预算 `GRID_MAX_BINS`（`core/grid.rs:20`，
  消费者 `terrain/src/mesh.rs:105/:218`、`splat/src/lib.rs:129`）；求解器**参与式降点**
  （`island.rs:52` `point_reduce_after()`，默认 3：第 3 轮起跳过"至今零冲量"的浅缝点，**深穿透绝不跳**）。
- `FragmentBudget { B1K, B10K, B100K, B1M }`（`destruction/src/lib.rs:14`）＋ 分档映射
  `budget_cap:86` / `sites_within_budget:97`（`impact_tiers.rs`）：**有分档函数，无引擎侧消费者**
  （调用点只在 destruction 自身测试）。
- 拥挤/压力度量：只有唤醒门 `wake_gate_k`（`core/config.rs:166`，默认 0，语义是唤醒规则、**不是**容量保护）。
- 「挤爆」两义（survey §2-T2 ⚠️，本计划沿用）：**物理被压塌/碎裂**（M3）vs **求解器超载降级**（容量机制）——
  本计划只做后者。

## 2. 设计总纲：转换 = 一次"跨表示作用"，挂同一张契约卡

照 `PLAN-COUPLING.md` §3 的契约四必填字段，转换通道的取值：

| 字段 | 取值 |
|---|---|
| 通道 | **Position（表示切换）**——位姿不跳是默认契约（新旧表示同位姿）；速度侧按交接档（§3.3） |
| 时间戳 | **tick 末**（各域轮次之后、下一 tick 宽相之前）；并记录"源数据快照 tick"（体素域可被挖改，读格必须钉一个时刻） |
| 频次 | **事件级**（非每 tick；同一体素域同一 tick 至多一次转换调用——确定性前提） |
| 对称对 | **旧表示 ↔ 新表示**：对该体的线动量/角动量两侧对账（`Δp`、`ΔL` 有界判据，§4-I5） |

- **效应键**：新增 `EffectKey::Conversion` 与破坏域的提取（`extract_*` 一族）**互斥**——两者都消耗体素格，
  同一区域同一 tick 不得双消费（冲突 fail-loud，口径同 §3 冲突清单）。
- **域轮次登记**：新增 `conversion_pass` 须登记进 `crates/vxl-phys/tests/coupling_rounds.rs` 的 `ROUNDS`
  表（"新域不登记即红"的门已有齿，见该文件头注与注入实测）。
- **边界（不做什么）**：不做 CSG（`PLAN-boolean.md`）；不做软体/布自身的表示转换（软体线已定 XPBD 三角网）；
  不做 GPU 档（先 CPU）；不做跨块缝合/LOD 的完整形态（留 V5+ 入口）；不做 warm 残量继承（等 C5）。

## 3. 四件设计

### 3.1 转换本身：从 `VoxelVolume` 表面提取网格（**离线工具起步**）

- **算法三选**（拍板项 P1）：**surface nets**（每格一顶点、经格边零交点定位，格子面试连三角；无 256 表、
  无 MC 歧义面）／**marching cubes**（256 表 + 边表；跨块缝合有 Transvoxel 表，Lengyel 2010）／
  **dual contouring**（保锐利特征、需解 QEF，最重）。
  **建议：surface nets 起步**——代码面最小、无歧义面、顶点天然落在零集上（判据好写）；
  跨块缝合在"多块/LOD"成为真实需求时再选型（Transvoxel 表是公开数据，见 §6）。
- **落位**：`vxl-phys-terrain` 的 `voxel/` 子模块新增 `surface.rs`（只依赖 `VoxelVolume`，无新依赖）；
  示例工具 `crates/vxl-phys/examples/voxel_mesh_extract.rs`（与 `voxel_rest_probe.rs` 同族的探针形态，
  命名随施工定）。**输出 = `(points, tris)`**，直接喂 `MeshStore::add` / `add_trimesh` /
  `spawn_trimesh_body`。
- **判据**（§4-I1/I2/I3/I4）：零等值面一致（定义性）、体积/面积守恒（对格数口径，先量后写）、
  与体素路径的静置对拍（先量后写）、确定性（两跑逐位）。
- **如实登记**：`MeshStore::add` 会在注册期**静默丢弃**非法三角（越界/重复顶点/零面积，`mesh_store.rs:31` 区注释）
  ——提取器的判据必须断言"**零丢弃**"，把静默变显式；丢弃即红。

### 3.2 转换途中的物理："双重表示"窗口 = **影子对拍 + 提交式切换**

- **形态**：`world_step/conversion.rs`（新域文件，先例 `medium.rs`/`aero.rs`）+ 开关（默认关；
  `PhysConfig` 加字段，0 = 关，先例齐）。窗口语义（建议形态，拍板项 P2）：
  1. **窗口期物理照走源表示**：体素域仍是 provider，接触照旧——**不挖格**；
  2. **目标表示以"影子"并行**：把待转换区域提取成网格（§3.1），建**影子体**（不参与求解、不写速度/
     位置——防双计账，这正是契约要防的效应重复）；影子由源表示的**同一位姿**驱动（静态地形区 = 无位姿项；
     未来动态体素体 = 该体位姿，属 V5+ 差异）；
  3. **对拍**：对受影响的对，用窄相分别查询源 provider 与影子网格，比较**流形摘要**
     （点数 / 最大穿透 / 法向分布）——两侧生成器不同（SDF 采样 vs 逐顶点采样），阈值按"几何离散误差"
     口径论证，**先量后写**，不是逐点相等；
  4. **提交式切换**：对拍通过 ⇒ 切走目标表示（影子升为正式动态体、`extract_*` 移格、
     `refresh_provider_bounds()`——同 `spawn_box_debris` 先例 `world_body.rs:53`）；对拍不过 ⇒
     **默认拒绝切换**（保持源表示；可配降级盒档，见 P4）。fail-loud、无静默。
- **最小形态 = 切换前 1 拍对拍**；N-tick 窗口（"转换中持续运动"）为加强档，窗口长度进预算（V4）。
- **判据**（§4-I5/I6）：接触集合连续（对拍阈值内）；切换瞬间无穿透突变（前后一 tick 最大穿透深度差 ≤ 阈值）；
  同位姿不变式（影子与源位姿逐位相同）；**金丝雀**：人为错位影子 ⇒ 对拍判据必红（防"空过"）。

### 3.3 交接策略三档（拍板项 P3）

| 档 | 语义 | 实现 | 判据 |
|---|---|---|---|
| (i) 不继承 | "挖掘/新物种"语义（现状口径：引擎不凭空造动量，`world_body.rs:154-155`） | 新体静止生成 | Δp、ΔL = 0（逐位） |
| (ii) **继承速度/角速度**（建议默认） | "同一物质换表示"语义 | 新体 `linvel/angvel` 取自源；**带质心修正**：`L_new = L_old + (c_old − c_new) × p` | `|Δp| ≤ ε_m·|p|`、`|ΔL| ≤ ε_L·|L|`、能量不增（`E_new ≤ E_old + ε`），阈值先量后写 |
| (iii) 继承约束/warm 残量 | 最重：warm 槽按接触特征号匹配，换表示后特征全变 ⇒ 需"特征再映射" | **本计划不做**（等 C5），只留接口位登记 | —（登记为不做的边界） |

- 判据补充：**(ii) 档退化对拍**——同场景"手工建体带初速"与"转换交接"两条路逐位相同（口径钉死才接下一片）。

### 3.4 预算与确定性降级（拍板项 P4）

- **旋钮**（`PhysConfig`，0 = 不限）：单次转换网格顶点上限、每 tick 转换事件数、窗口长度上限。
- **降级两级（建议）**：超网格预算 ⇒ **凸包代理**（`add_hull` 现成、GJK/EPA 成熟；survey T3-② 的外部
  经验"凸包取多边形平面当接触提供者"同向）⇒ 仍超 ⇒ **贪心盒**（`extract_boxes` 现成，**即现行口径**
  ⇒ 降级下界有现成实测对照）。
- **确定性**：档位选择 = 格数/顶点数的纯函数 + 固定扫描序 ⇒ 同输入同结果（两跑逐位）；
  排队（超瞬时预算）按调用序，不引入随机。
- 判据（§4-I7）：超预算场景档位选择确定；降级产物保真度 ≥ 盒档（对照实测）；旋钮全 0 时逐位 = 不降级路径。

## 4. 判据体系（不变量 → 测试；阈值一律"先量后写"）

| # | 不变量 | 断言 | 建议位置 |
|---|---|---|---|
| **I0** | **零代际门** | 默认关 ⇒ 既有判据/金样/冻结哈希一字不动（不创建转换 ⇒ 恒等） | 全量门禁 |
| I1 | 零等值面一致 | 每个抽取顶点的 `|sdf(v)|` ≤ 格边端点最大 `|sdf|`（surface nets 定义性断言；每条被穿越格边零交点唯一） | `terrain` in-crate（`voxel/surface.rs` 测试） |
| I2 | 守恒 | `V_mesh`（散度定理）与 `N_occ·h³` 偏差 ≤ 表面带容差；`ΣA` 与解析界可比。**预期先验 c ≲ 0.5**（每表面格最多半格体积误差，待实测写死） | 同上 |
| I3 | 静置对拍 | 同场景（同 `voxel_rest_probe` 地板）：网格体与体素体静置高度差 ≤ 阈值；两路口径不同（SDF 深度 vs 顶点采样+skin），差值先量后写（现成锚见下注） | `crates/vxl-phys/tests/`（门面级） |
| I4 | 确定性 | 同输入两次提取逐位相同（含顶点/三角序）；**零丢弃**（`MeshStore` 过滤不丢任何三角） | in-crate |
| I5 | 接触集合连续 | 影子对拍：流形摘要（点数/最大穿透/法向分布）差 ≤ 阈值；**金丝雀：错位影子必红** | `crates/vxl-phys/tests/conversion_window.rs`（新） |
| I6 | 无穿透突变 | 切换前后一 tick 的最大穿透深度差 ≤ 阈值 | 同上 |
| I7 | 交接有界 | `Δp/ΔL/ΔE` 有界；(ii) 档与"手工建体"逐位退化 | `conversion_handoff.rs`（新） |
| I8 | 降级确定 | 超预算两跑逐位；降级保真度 ≥ 盒档 | `conversion_budget.rs`（新） |

> I3 的现成锚（先量后写时作对照用）：体素路径静置 −0.0004/0.0000/+0.0220（`voxel_contacts.rs:237` 注）；
> 软体侧世界级 +0.0050（P11，2026-10-01）。
>
> 判据卫生（沿用本仓教训）：金丝雀语料的期望值必须锚到规则之外的观测；阈值不许抄先例数字糊上去；
> "绿 = SKIP 不算绿"。

## 5. 分片迁移计划（每片独立、可单撤）

| 片 | 内容 | 判据 | 代际/回退 |
|---|---|---|---|
| **V1 ✅** | **离线提取**：`voxel/surface.rs`（surface nets）+ `examples/voxel_mesh_extract.rs` + I1–I4。**无运行时面**（不接线引擎相位）。落地注记：整体积提取（区域裁剪随 V2 窗口设计）；I1 带宽钉半格（首测 max\|sdf(v)\| = h/3）；I2 首测盒缺口 2.016 m³（PLAN 先验上界 8 内）、面积 40.177 | I1/I2/I3/I4 全绿（in-crate 5 + 门面 1） | 零代际（不接线）；单提交撤销 |
| **V2** | **双重表示窗口**：`world_step/conversion.rs` + 开关 + `conversion_pass` 登记轮次表 + 影子对拍 + I5/I6 | I5/I6 + I0 | 默认关；关档逐位不变 |
| **V3** | **交接策略**：(i)/(ii) 两档 + I7；门面 `convert_region_to_mesh(...)`（挖掘→建网体一条龙，替代盒路径的可选档） | I7 | 默认 (ii)；关 ⇒ 走现行盒路径 |
| **V4** | **预算与降级**：旋钮 + 凸包/盒两级 + I8；`apply_impact_destruction` 按预算接线（盒 → 网格，超预算回盒） | I8 | 旋钮 0 ⇒ 逐位 = V3 路径 |
| V5+ | **后续（本计划不做，登记入口）**：跨块缝合/LOD（Transvoxel 表现成）；**动态体素体**（需要体素域带位姿——现状 provider 是静态标记体，`shape.rs:46-48`，差异登记）；GPU 档；warm 残量继承（等 C5） | — | — |

## 6. 外部对标与护栏（先算账，别照抄；证据分级）

- **提取算法**（公开资料，均无专利主张，按证据分级）：
  - Surface nets——Gibson 1998（SIGGRAPH course）；Lysenko, *Smooth Voxel Terrain*（0fps.net, 2012）。
    **本计划第一选**（源码级参考多、形态小）。
  - Marching cubes——Lorensen & Cline, SIGGRAPH 1987（256 表）；歧义面修补见 MC33 一系。
  - Transvoxel——Lengyel, *Journal of Graphics, GPU, and Game Tools* 15(2), 2010；
    表数据与说明：[transvoxel.org](https://transvoxel.org/)（"transition cells" 拆 512 例 → 73 等价类，
    专治**跨块 LOD 缝合**；本仓 V5+ 选型时再评估）。
  - Dual contouring——Ju et al., SIGGRAPH 2002（保锐利特征，需 QEF；最重）。
- **工程对照（有源码）**：godot_voxel（[Zylann/godot_voxel](https://github.com/Zylann/godot_voxel)）：
  marching cubes / Transvoxel 生成**独立于渲染面的 `collision_surface`**，产出凹多边形形状当**静态**碰撞；
  其社区讨论明确"凹网格对动态体极不高效、需凸分解或退回块状 AABB 碰撞"
  （[issue #147](https://github.com/Zylann/godot_voxel/issues/147)）。⇒ 与本仓取向同向：
  `TriMesh` 非凸、逐顶点采样（`shape.rs:64-71`）适合做**碎片/薄壳**；大块动态体走凸包/盒降级（§3.4）。
- **本仓已否证的相邻路线（护栏，勿重做）**：
  ① 碎块"继承半速"实测是能量源（`world_body.rs:154-155`）⇒ 交接档 (ii) 必须走显式对账判据，
  不是随手折算一个系数；
  ② 贪心盒碎块曾把质量当密度传（`ρ·V²` 缺陷，2026-09-27 修，`world_body.rs:27` 注）⇒ 网格碎片质量必须
  走薄壳口径 `m = ρ·t·ΣA`（`world_mesh.rs:66`/`mesh_store.rs:92`），不得复用盒的"反解密度"公式；
  ③ survey T3-④：外部仓库"稀疏体素 raymarch、永不转多边形"对**烟/雾**可当渲染档，但本仓载体是
  **要参与接触的碎片** ⇒ 必须多边形化（"不吸收"档已记档）。
- **口径差异登记（不照抄的）**：外部引擎的 voxel→mesh 多用于**静态地形分块 + LOD**（供渲染/玩家碰撞），
  本仓的诉求是**可参与接触、可被挖/被撞的碎片与换表示通道** ⇒ 算法可借，接触口径必须按本仓判据重验。

## 7. 拍板项（**已回执：2026-10-02 用户「都一起搞」⇒ P1–P5 全按建议执行**）

- **P1 提取算法**：✅ surface nets 起步（无 256 表/歧义面、零集判据最直接）；
  跨块缝合在"多块/LOD"成为真实需求时再选型（Transvoxel 表现成）。
- **P2 窗口语义**：✅ "复制式 + 影子对拍 + 提交式切换"（窗口内体素不动、物理照走源表示；
  网格影子只算接触摘要供对拍）；最小形态 = 切换前 1 拍对拍。
- **P3 交接默认档**：✅ (ii) 继承速度/角速度（带质心修正）；(i) 保留给"挖掘/新物种"语义；
  (iii) 留到 C5 之后。
- **P4 降级档**：✅ 凸包 → 贪心盒 两级（盒档 = 现行口径，下界有实测对照）；
  对拍不过时的默认动作 = 拒绝切换（可配降级盒）。
- **P5 `StateBridge` 定位**：✅ 不扩 trait——它是导出/导入面（渲染/持久化/回放）；
  转换走 `world_step/conversion.rs` + 契约卡，`BridgeKind` 仅作工具链标签复用
  （两个面的消费者、频次、判据都不同；混进同一 trait 会把"零实现面"变成"两个零实现面"）。

## 8. 队列重排（写清，别只堆新计划）

- 对 `PLAN-triangle-first-class.md` 优先级表的更新：其 #2（T1 三角网进 `Shape`）**已落地**（2026-09-28 起七片）；
  其 #3 = 本计划，**几何缺口已消**。本计划 V1 是离线工具（可先出可见成果、零运行时面）。
- 与 `PLAN-COUPLING.md` §9 的关系：本计划是该表 **P2 行**的施工单；它用耦合契约的卡与轮次表，
  不动 C0–C4 的既有口径。
- 「挤爆」的**物理侧**（压塌/碎裂）不属本计划（M3/`PLAN-0.2.md` 线）；本计划只做**容量侧**（超载降级）。
- V2 之后与 GPU 档、求解器线并行无冲突（新文件 + 默认关；不碰热路径既有段位）。

## 9. 证据索引（本文件引用的关键落点）

- 四件现状：`SURVEY-SOFT-CLOTH-AND-CONVERSION.md` §1.2/§2-T2；`PLAN-COUPLING.md` §4.4/:338-344、§9/:706。
- 体素侧：`voxel_volume.rs:12-22,128,133,153,204,211,232,314`；`voxel_provider.rs:5,15`；
  `world_body.rs:27,28,33,44-53,59,78,111,154-155`；`voxel_contacts.rs:237`。
- 网格侧：`shape.rs:64-71`；`mesh_store.rs:22,31,92`；`world_mesh.rs:14,66,72`；
  `terrain/src/mesh.rs:83,105,218`；`SURVEY-SHAPE-SUPPORT-MATRIX.md`。
- 桥与契约：`interop.rs:76-89,256-268`、`:247-254`（`MediumField`）；`coupling_rounds.rs`（`ROUNDS` 表）；
  `world_step/medium.rs`、`world_step/narrow_tier.rs`（让位先例）。
- 预算先例：`core/grid.rs:20`；`island.rs:52`；`destruction/src/lib.rs:14`；`impact_tiers.rs:86,97`；
  `core/config.rs:166`。
- 外部：transvoxel.org；godot_voxel 仓库与 issue #147（URL 见 §6）。
