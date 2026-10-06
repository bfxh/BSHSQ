# PLAN-COUPLING —— 跨域耦合契约（统一口径 · 施工方案）

> 立项动因（2026-09-30，用户观察）：**四条耦合路径各讲各的口径**——`MediumField::deposit` 留了但是空的
> 且无调用点；流体反作用走 `fluid_reaction_pass`（第三套）；布料/绳索自带 `body_dv`/`body_dx` 两腿。
> 刚体、流体、布料、Gaussian 混在一起以后，**力/冲量/位置修正分别什么时候生效没有统一口径**。
>
> 本文档 = ① 现状取证（全部带 `文件:行`）② 问题清单（按"会不会真出事"排序）③ **统一口径设计**
> ④ 接口落位（遵守 god 门棘轮）⑤ 分片迁移计划（每片带判据与换代标记）⑥ 外部对标 ⑦ 队列重排。
>
> 状态：**方案定稿待施工**（本文档不落任何行为改动）；执行顺序见 §5，第 0 片是纯观测层（不动读数）。
> 关联：`ROUTE.md` §2.1（四接口）/§4（兼容矩阵）/§5（步进层：子步 + 域顺序 + **双向耦合轮次** + 一个调度器）、
> `M1-EXIT.md` §4（2b 记账）、`OPEN-PROBLEMS.md` P7（2b 精度残余）、`DESIGN-staged-solver.md` §7（求解器三缺陷）。

---

## 0. 一句话结论与要拍板的三件事

**一句话**：现在缺的不是某一条耦合的实现，而是**一条跨域作用的"契约"**——作用的量纲/时间戳/频次/对称对
没有声明，施加点散落在 10 处、睡眠门有三种写法、被丢弃的作用没有账。⇒ 先把**契约**写出来并用**观测层**
量出偏差，再按片把四条路径收进同一套通道（力 / 冲量 / 位置），最后才是"介质真双向"与"统一求解器元素"。

**要拍板的三件事**（都影响冻结读数，属换代级）：

| # | 决策 | 选项 | 影响面 |
|---|---|---|---|
| D1 | **反作用采样口径**：流体反作用现在是"**末子步快照**当 tick 常量力"（有效冲量 = `F_末 · dt_tick`，而 `F_末` 的时间尺度是 `dt_sub`） | (a) 保持现状但显式登记偏差；(b) 改成 `dt_sub` 时间加权的 **tick 平均力**（推荐，账更清、与 `窗口均值` 判据同口径）；(c) 改成冲量通道。**✅ 已拍板 (b) 并落地（2026-10-01，见 §5 C2）** | 2b 的所有读数（含 P7 精度表、`fluid_boundary` 门）——重冻登记见 §5 |
| D2 | **软体两腿是否并进统一通道**（速度腿 → 冲量通道；位置腿 → 位置通道 + 记账） | (a) 只登记不改（读数不动）；(b) 收进统一施加入口（**行为应逐位不变**，只是接线收口）；(c) 连"位置腿进虚拟位姿"那笔旧账一起修（**换代**，动 rope 冻结哈希与 14 格表） | (b) 风险低；(c) 已在 `SESSION-2026-09-28-TRIANGLE-CLOTH.md` 列为待拍板 |
| D3 | **介质（喷溅场）是否做真双向** | (a) 不做（保持单向 + 显式登记）；(b) 做"吸收动量"的最小版（核位置/速度演化 + `deposit` 接线，需新增场状态与推进器）；(c) 等 §3.1 的"物理代理（粒子域）"路线（把 splat 当粒子解，双向自然出现） | (b) 新增场状态 = 新接口 + 新判据；(c) 是 ROUTE §3.1 第二层，成本更大但架构更顺 |

---

## 1. 现状全景（取证）

### 1.1 一个 tick 的真实相位序（`crates/vxl-phys/src/world_step.rs`）

```
World::step                                   (:10)
├─ 复用判据（准静态门）                        (:17-25)   ※ detect_once_per_tick() 硬编码 false ⇒ 恒不复用
├─ for k in 0..substeps { Substep(dt, k==0) }  (:26-28)
│   ├─ A 力场（重力/风/吸引）                   :276   fields.apply → force += g·mass
│   ├─ B 介质 2a + 喷溅作介质                   :277   medium_pass（② 段 :196 有 awake 门；① 段 :121 只查 is_dynamic）
│   ├─ C 面元气动                               :280   aero_pass → force/torque（无 awake 门）
│   ├─ D 2b 流体反作用                          :283   fluid_reaction_pass（有 awake 门 :262）
│   ├─ E 速度积分 + **清力累加器**              :289   Integrator::integrate_velocities（睡眠体：写入被静默置零 :26-30）
│   ├─ F 宽相 / G 窄相                          :297-325
│   ├─ H 冲击快照 / I 关节唤醒                   :332 / :337
│   ├─ J 接触求解（顺序冲量 + 岛级睡眠）         :338-344  solver.solve（睡眠体在岛内被"按静态处理" ⇒ 速度被归零 :271-280/462-469）
│   ├─ K 关节求解（直写 linvel/angvel）         :345
│   ├─ L 位置积分                               :349
│   ├─ M 无偏置趟（默认关，stabilization=0）     :355-365
│   └─ N 选择性 CCD（钳位 + 清法向速度）        :368
└─ domain_pass（**全部子步之后**）              :29 → world_soft.rs:84-91
    ├─ ① fluid_pass：先重建边界粒子、再推进流体   world_step.rs:41-55
    ├─ ② 软体代理快照重建（绳/布共用）           world_soft.rs:86-88
    ├─ ③ rope_pass + **两腿回填**                world_soft.rs:89 → :150-154 → :224-244
    └─ ④ cloth_pass + **两腿回填**               world_soft.rs:90 → :208-213
```

**关键事实**：`force`/`torque` 是**每子步消费并清零**的累加器（`integrate/lib.rs:26-51`）⇒
**子步外写入只被一个子步消费**（这正是 `angular_impulse_contract.rs` 钉住的坑：tick 末注入要交付"整 tick
冲量"必须 `÷ dt_sub`，不是 `÷ dt`）。

### 1.2 十条"外部域 → 刚体"写入点总表

| # | 机制 | 位置 | 写什么 | 相位 | awake 门 | 对称性 |
|---|---|---|---|---|---|---|
| 1 | 力场（重力/风/吸引） | `field/lib.rs:29` | 力 | 子步 A | **无**（只查 `is_dynamic`） | 无（场是常量源） |
| 2 | 喷溅作介质（二次阻力） | `world_step.rs:148` | 力 | 子步 B① | **无**（只查 `is_dynamic` :121） | **无**（`deposit` 空） |
| 3 | 流体 2a（浮力+阻力） | `world_step.rs:246` | 力 | 子步 B② | **有**（:196） | **无**（`deposit` 空） |
| 4 | 面元气动（面元力/力矩） | `aero.rs:60-61` | 力+力矩 | 子步 C | **无** | 无（单向） |
| 5 | 流体 2b 反作用 | `world_step.rs:265-266` | 力+力矩 | 子步 D | **有**（:262） | **成对**（逐对 `−F_i` 进 `bforce`，`fluid_force.rs:204-207`） |
| 6 | 绳速度腿 | `world_soft.rs:238` | **Δv** | tick 末③ | 间接（代理 `inv_mass=0` :125-129） | 冲量取反（`rope.rs:587-594`） |
| 7 | 绳位置腿 | `world_soft.rs:241` | **Δx** | tick 末③ | 同上 | 位移口径（`rope.rs:553-555`） |
| 8 | 绳角反作用（默认关） | `world_soft.rs:185` | 力矩（`×substeps/dt`） | tick 末③ | **无** | 角冲量（`rope.rs:589-593`） |
| 9 | 布两腿 | `world_soft.rs:213` → :238/:241 | Δv + Δx | tick 末④ | 间接 | 同绳（`cloth_coupling.rs:272-275`） |
| 10 | 接触/关节/CCD（引擎自用） | `island.rs:88-136` / `joints/linalg.rs:118-121` / `world_ccd.rs:80,83` | Δv+Δω / **Δx** | 子步 J/K/N | 求解器自管 | 成对（冲量） |

另有旁路：`World.bodies` 是 `pub` 字段（`world_struct.rs:6`）⇒ 任意代码可裸写；`BodySet::apply_impulse`/
`set_linvel` 会 `wake`（`body.rs:159-195`），而 `set_angvel_raw` **不唤醒**（`:147-149`）。

### 1.3 四条耦合路径逐条解剖

**(a) 流体 ↔ 刚体（2b，Akinci 边界粒子）**：产生在流体力相位（逐对 `bforce[j] -= d·(mi·(coef+cv)/denom)`，
`fluid_force.rs:204-207`）→ 每子步清零重累（`:171-174`）→ 段序聚合 `aggregate_reactions`（`fluid_boundary.rs:5-19`，
量纲**力**，`fluid_access.rs:109-113`）→ 下一 tick 的**每个体子步**施加到 `force/torque`（`world_step.rs:254-269`）。
CPU 并行档的 `bforce` 走**串行补趟**保逐位（`fluid_force.rs:126-128`，判据 `parallel_equals_serial_bitwise`）。
GPU 档同口径走 `reduce.wgsl`（一个体一个 workgroup、段内升序、禁用原子加），但**跨端只到口径 B（相对 1e-5）**
且**没有 `#[test]`**，只有 example（`gpu_tick_probe`/`gpu_coupling_probe`）。
**软体/布真点式受体尚未落地**（`fluid_medium.rs:77-79` 原文）。

**(b) 流体 → 刚体（2a，介质采样）**：`medium_pass` 每子步 `sys.sample(p)`（体心 + 4 个水平表面点，
`world_step.rs:218-228`）→ 浮力 `−g·(ρ·V·frac_sub)` + 二次阻力 → `force`（`:241-246`）。**单向**
（`deposit` 空、无调用点）。与 2b **显式让位**（`covered` 集，`:199-201`）。

**(c) 软体 ↔ 刚体（两腿）**：XPBD 子步内累积（`rope.rs:594`/`cloth_coupling.rs:274`）；`body_dv` = **速度增量**
（已除体质量）、`body_dx` = **位移**（`λ_geom − λ` 被速度钳位压掉的那一份，`rope.rs:125-136`）；tick 末
`apply_two_leg_reactions` 直接写 `linvel`/`position`（`world_soft.rs:238/241`）。施加顺序 = **几何序**
（按粒子世界 `x` 升序，`rope.rs:490-496`；布的数组序是行主序故不等价，`cloth_coupling.rs:157-160`）。
角反作用默认关（`rope.rs:188`），理由是"接触模型看不见转动"（`world_soft.rs:155-162`）。

**(d) 介质 → 刚体（喷溅场）**：`GaussianSplatField` 实现 `MediumField`（`splat/lib.rs:481-510`），
`sample` 给 `density = σ·medium_density`、`velocity = 常值参数`、`viscosity = σ·medium_viscosity`、
`temperature = 0`、`occupied = clamp(σ/iso)`；`deposit` 空且注释写明"当前为单向耦合…双向见 ROUTE §3.1 待办"（`:507-509`）。
门面只读 `density` 一项，做**体心单点**二次阻力（`world_step.rs:121-148`）。**软体/布完全不采样介质**
（`vxl-phys-soft` 与 `world_soft.rs` 对 `Medium` 零命中）。

### 1.4 已经有、但只在局部成立的"隐性口径"（四条）

1. **力累加器口径**：每子步消费并清零 ⇒ 子步外注入只被消费一次 ⇒ "整 tick 冲量"要 `÷ dt_sub`
   （`tests/angular_impulse_contract.rs:1-6` 明文 + 金丝雀 `:53-69` 实测老口径 `0.500000`）。
2. **睡眠/静态口径**：软体侧"睡眠体对软体域呈现为静态"（代理 `inv_mass=0`，`world_soft.rs:122-129`，
   判据 `rope_scene::rope_does_not_disturb_a_sleeping_body`）；流体 2b 侧"消费点判 awake"（`world_step.rs:262`）；
   力场/喷溅/气动**无门**（写入被积分器静默丢弃，`integrate/lib.rs:26-30`）。
3. **确定性口径**：遍历按体索引升序（`body.rs:10`）、岛内约束按 `(a,b,点序)`（`solver/lib.rs:7`）、
   软体施加序 = 几何序（`rope.rs:490-496`）、流体求和序 = 段序 × 段内索引序（`fluid_boundary.rs:6`）。
4. **让位口径**：2a 让位给 2b（`covered`，`world_step.rs:199-201`）；卡上步进档让位是**流体粒度**
   （`world_step.rs:181-182`）；而 `covered` 是**跨流体单数组**（每流体 refresh 都 clear ⇒ 只有最后一个 2b 流体的覆盖集有效）。

---

## 2. 问题清单（按"会不会真出事"排序；每条给证据、触发条件、后果）

### A 类 —— **静默丢**（现在就在错，且没有账）

| # | 问题 | 证据 | 触发条件 | 后果 |
|---|---|---|---|---|
| A1 | 睡眠体收到的作用被**静默丢弃**，且三条路径口径不同：力场/喷溅/气动**无门**（写入后被积分器丢弃）；2a/2b 显式 `continue`；软体在代理侧 `inv_mass=0` | `integrate/lib.rs:26-30`；`world_step.rs:121/196/262`；`aero.rs:23-62`；`world_soft.rs:125-129` | 任何外力作用于睡眠体 | 物理上"说不清"（该唤醒还是该记一笔？），**无计数器** ⇒ 判据写不出来 |
| A2 | `torque` 的"每子步消费"契约只有两个测试守着，其余写入点靠注释；`world_soft.rs:136` 的注释与实现不符（实际 `×substeps/dt`） | `angular_impulse_contract.rs`；`world_soft.rs:136` vs `:185` | 新增任何 tick 末力矩注入 | `÷dt` vs `÷dt_sub` 差 `substeps` 倍的复发面 |
| A3 | 绳角反作用注入**无 awake 检查**（同函数里两腿有门、角腿没有） | `world_soft.rs:182-187` vs `:231` | 打开 `angular_reaction` | 睡眠体多一条偷吃角冲量的路（现在默认关 ⇒ 无害，但是定时炸弹） |
| A4 | `MediumField::deposit` 两条实现都空、**零调用点**，而 trait 文档自称"域之间的**唯一**双向通道" | `interop.rs:246-253`；`fluid_medium.rs:76-80`；`splat/lib.rs:509` | 任何人以为双向已通 | ROUTE §2.1"无域内特例"的判据**实际已被 2b 绕过**（2b 走 `set_boundary_particles`+`bforce`，不进 `MediumField`） |

### B 类 —— **口径不一致**（同一件事两种说法/做法）

| # | 问题 | 证据 |
|---|---|---|
| B1 | **三种 awake 门**并存：无门（写入点 1/2/4）、`is_dynamic` 门、`is_dynamic && awake` 门 | `field/lib.rs:25`；`world_step.rs:121` vs `:196` vs `:262`；`aero.rs:23-62` |
| B2 | 软体侧声称"与 2b 同段位、同量纲账"，实际**四不同**：量纲（力 vs Δv+Δx）、段位（子步内 vs tick 末）、频次（`substeps` 次 vs 1 次）、是否过积分器/限速/求解器 | 声明：`rigid.rs:9-10`、`rope.rs:109-110`、`world_soft.rs:136`；实现：`world_step.rs:265`、`world_soft.rs:238/241` |
| B3 | 文档指向**不存在的代码位置**：两处注释说 2b 反作用在 `medium_pass`"段③"施加，而该函数只有 ①② 两段，真正的施加在 `fluid_reaction_pass` | `world_build.rs:159`、`world_step.rs:172` vs `:254-269` |
| B4 | `RigidReaction.impulse`（冲量口径字段）**只写不读**；注释说"门面 ÷dt 后按力施加"，门面实际从不 `÷dt` | `rigid.rs:32-40` vs `world_soft.rs:224-244` |
| B5 | `MediumSample` 两个字段无消费者且两实现口径相反：`viscosity`（流体**拒绝**编造换算常数；喷溅填 `σ·medium_viscosity`）、`temperature`（恒 0） | `interop.rs:56-60`；`fluid_medium.rs:14-15`；`splat/lib.rs:497-498` |
| B6 | 喷溅段不读 `occupied`（自由表面语义白给）；流体段用 `frac_sub` 当门 | `world_step.rs:136` vs `:234-237` |
| B7 | 重力**双通道**：走 `GravityField` 当力（用 `1/inv_mass` 反推质量），积分器自带的 `gravity` 形参恒传 `ZERO` | `world_build.rs:13`；`field/lib.rs:28-29`；`world_step.rs:289` |

### C 类 —— **采样与时效**（值取错时刻/取错对象）

| # | 问题 | 证据 | 后果 |
|---|---|---|---|
| C1 | **末子步快照当 tick 常量**：`bforce` 每子步清零重累 ⇒ `breact` = 末子步的力，却被当整 tick 常量施加（有效冲量 `F_末·dt_tick`，而 `F_末` 的时间尺度是 `dt_sub`） | `fluid_force.rs:171-174`；`world_step.rs:251-253`；GPU 探针自认此项 `gpu_coupling_probe.rs:13-15` | 采样偏置 `O(dt_tick·dF/dt)`；流体子步数（4）与刚体子步数（2）互不相干，两边一变偏置就变 ⇒ **P7"波动"的一半来源** |
| C2 | 2a 与 2b 的**位姿时效不对称**：2b 用本 tick 重建的边界粒子（新位姿），2a 用**上一 tick 末**的流体状态在**本 tick 实时体位置**上采样 | `world_step.rs:202/228` vs `fluid_step.rs:34-37` | 被覆盖与未被覆盖的体，回路相位差不同（文档却把二者描述成同口径） |
| C3 | **1 tick 滞后无登记表**：反作用"产出于 tick N 末的域轮次、施加于 tick N+1 的子步"，只在注释里写过一次 | `world_step.rs:37-40` | 新增域时无法一眼看出谁滞后几拍 |
| C4 | ✅ **已修（2026-10-01）**：`covered` 原是**跨流体单数组** ⇒ 多流体时只有最后一个 2b 流体的覆盖集有效；修法 = 覆盖集**按流体各一份**（`covered[fi]`，`world_soft.rs` / `world_step.rs` / `world_body.rs` 访问器带 `fi`） | 取证与判据 = `crates/vxl-phys/tests/fluid_covered_multi.rs`：修前浸在流体 0 的轻盒被 2a 浮力从 y **1.274 弹到 5.852**（远处一个 2b 流体的存在即触发）；修后两场景 120 tick **逐位一致** |
| C5 | 卡上流体档的 `pmass` 只做**长度核对**（同形状不同间距/层数而粒子数相同时静默错） | `stepper.rs:10`；`coupling.rs:16-18` | 静默算错反作用 |
| C6 | 流体与刚体**各有一套 `gravity` 配置**且无一致性核对；2a 浮力用刚体侧 g | `world_step.rs:177/241`；`core/config.rs:76` | 配置不同值时"浮力与流体不符"，现有门发现不了 |

### D 类 —— **结构性缺失**（不是错，是"以后一定难收"）

| # | 缺失 | 现状 |
|---|---|---|
| D1 | **无统一施加入口**：10 个写入点各自直接写 `force/torque/linvel/position` | §1.2 总表 |
| D2 | **无受体门**（一个函数同时保证 `is_dynamic && awake && inv_mass>0`） | 三种写法（B1） |
| D3 | **无通道语义与时戳声明**（力/冲量/位置三选一 + 数据取自哪一拍） | 全靠注释 |
| D4 | **无效应去重**：2a/2b 靠一个手写 `covered` 数组做"二选一" | `world_step.rs:199-201` |
| D5 | **无审计**：动量账/能量账不成体系（`pressure_work` 定义了但零消费者） | `interop.rs:252` |
| D6 | **无滞后登记**（每条路径的往返拍数） | C3 |
| D7 | **介质无状态**：喷溅场无速度/质量槽、无 `advance`/`dt` ⇒ 结构上不可能双向（不只是"没接线"） | `splat/lib.rs:42-53`、全目录零 `advance` |
| D8 | ~~**`StateBridge` 零实现**~~ **——已有两个真实现（2026-10-06 更新）**：流体 `FluidSystem`（2026-10-05）与布片 `ClothSheet`（2026-10-06，`cloth_access.rs`），都只搬位置（`Vec<Vec3>`）⇒ **"转换中的物理/交接策略"仍然没有落点**（本表要的是这个，不是"有没有实现"） | `interop.rs:256-275`；`PLAN-triangle-first-class.md:83-84` |

---

## 3. 统一口径（设计主体）

### 3.0 契约的四个必填字段（这就是"什么时候生效"的答案）

**任何跨域作用都必须声明这四个字段**（写进代码类型 + 文档卡 + 判据）：

| 字段 | 取值 | 它回答的问题 |
|---|---|---|
| **通道** | `Force` / `Impulse` / `Position` | 这个作用以什么形式进入受体 |
| **时间戳** | `TickBoundary`（数据取自哪一拍末的状态） | 作用的数值来自哪一刻的域状态 |
| **频次** | `PerSubstep` / `PerTick` | 一个 tick 施加几次 |
| **对称对** | `None`（外部常量源）或 `Some(域)` | 谁收反作用、怎么对账 |

**判定表（用哪条通道）**：

| 物理形态 | 通道 | 为什么 |
|---|---|---|
| 连续场作用：重力、风、气动、浮力、阻力、介质阻力、面元力/力矩 | **Force** | 整 tick 内近似恒定；由积分器乘 `dt_sub` 消费 ⇒ 与子步数无关地给出"tick 平均力 × tick 时长"的冲量 |
| 瞬时/离散事件：接触、关节、**反作用的瞬时分量** | **Impulse** | 只能表达为"消掉本子步接近速度"；与求解器同段位（速度积分后、位置积分前） |
| 几何/收敛补足：CCD 钳位、**XPBD 位置腿**（被速度钳位压掉的那一份） | **Position** | 现有速度级无法表达（见"位置通道准入门槛"） |

**位置通道的准入门槛（三条同时满足）**：
1. **声明"为什么不能进速度级"**（例：XPBD 的 `λ_geom − λ_vel` 是"收敛补足"，改写成速度会改收敛语义）；
2. **提供等效动量记账**：`J_equiv = m_eff·Δx/dt`（**只用于审计，不用来施加**）；
3. **在固定几何序下施加**（`rope.rs:490-496` 已做；`cloth_coupling.rs:157-160` 如实登记了布与绳的序不等价）。

**禁止**：同一效应同时出现在两条通道（§3.4 效应键）；把位置修正用于"稳定深穿透"（那要修正速度上限，见 §7）。

### 3.1 时间轴与时间戳约定（一图钉死）

```
tick N                                          tick N+1
├─ 子步 0..S-1（刚体）
│   A..D 写 force/torque（Force 通道）
│   E  消费 + 清零（force → Δv）
│   J/K 冲量（Impulse 通道，求解器/关节）
│   L  位置积分
│   N  CCD（Position 通道，钳位）
└─ domain_pass = **域轮次**（全部子步之后）
    ① fluid_pass：重建边界粒子（时戳 P_N）→ 推进流体 → 产出反作用
    ② 软体代理快照（时戳 P_N）
    ③ rope / cloth 内部子步 → 产出 (Δv, Δx, τ)
    ④ 介质/气动采样与产出
    ⑤ **统一施加**（§3.6）：Force → 累加器；Impulse/Position → 直接写
       数值时戳 = P_N；生效拍 = N+1（Force 被 N+1 的子步消费；Δv/Δx 写在 N 末，
       N+1 的首个子步立刻看见）
```

**统一口径三条（U1–U3）**：
- **U1 时间戳**：跨域数据一律以**产出时刻的受体位姿**为时戳，卡里登记"受体何时看见它"（滞后 0 或 1 拍）。
- **U2 同拍不动**：没有任何跨域作用能在同一拍改变它自己的输入（禁止"边算边回灌"）⇒ 域轮次在子步循环**之外**。
- **U3 一拍一次兑换**：同一域对同一受体的作用，一个 tick **只兑换一次**（禁止"子步内 + tick 末"两种口径混用）。

### 3.2 受体门（receptor gate）——一处实现、十处调用

```
fn gate(bodies: &mut BodySet, i: usize) -> Option<Receptor>   // is_dynamic && awake && inv_mass > 0
```

- §1.2 的 10 个写入点**必须**经门取受体；取不到 ⇒ **不静默跳过**，记一笔 `dropped[channel] += 1`，
  进审计；门面测试断言"睡眠体：写入 0 **且** 计数 N"（把 A1 从"没账"变成"有账"）。
- **门与唤醒分开**：门只决定"是否施加"；唤醒由显式规则决定（默认建议：有效作用超过登记阈值就 `wake`，
  阈值进卡；现状是"孤立睡眠体永远收不到作用"）。

### 3.3 对称性与动量账（不变量 I1）

- **成对来源**（流体 2b、软体两腿）：给出 `Δp_body`，并声明 `Δp_domain` 落在哪里
  （流体：逐对 `bforce` 已对称，`fluid_force.rs:204-207`；软体：粒子侧速度/位置变化即 `Δp_domain`）。
  审计量 = `Δp_body + Δp_domain`（相对残差登记，判据给容差）。
- **单向来源**（力场/气动/介质/2a）：卡里声明 `对称对 = None`，审计里记为"外部源"（不是"账不平"）。
- **位置腿也进账**（用 `J_equiv`）⇒ 一条判据即可覆盖力/冲量/位置三种形态的动量守恒。

### 3.4 效应键（effect key）与让位/叠加

- 每个**物理效应**一个键：`buoyancy` / `drag_quadratic` / `akinci_reaction` / `aero_panel` / `medium_drag` / `gravity` …
- **一个键在一个 tick 只允许一条路径施加**（I3）；冲突 ⇒ **fail-loud**（断言 + 判据断言"冲突清单为空"），
  不再靠手写 `covered` 数组静默二选一。
- 让位表显式化：`(效应键, 受体类, 让位给谁, 生效条件)`。
- **修 C4**：`covered` 改 per-(流体, 受体) 位图（或让位表按流体分别求交）+ 新增"多流体"判据。

### 3.5 三条通道的施加落点

| 通道 | 施加段位 | 写入 | 消费 |
|---|---|---|---|
| Force | 子步开头（现 A..D 段位不变） | `force/torque` | `integrate_velocities` |
| Impulse | 速度积分之后、位置积分之前（**与求解器同段位**） | `linvel/angvel`（经门） | 立即（本子步位置积分看得见） |
| Position | 位置积分之后（几何序） | `position/rot`（经门） | 立即 |

> 三条通道的**段位差别**就是"什么时候生效"的完整答案：Force 影响**下一个**子步的速度；Impulse 影响**本**子步
> 位置积分出的位置；Position 直接改位置（不改速度）。

### 3.6 域轮次（domain round）与滞后登记表

`domain_pass` 从"四行调用"升级为**声明式轮次表**（新增域必须登记；门禁断言"表与代码一致"）：

| 序 | 域 | 读（时戳） | 产出通道 | 滞后 | 施加者 |
|---|---|---|---|---|---|
| 1 | 流体 | 受体 = 本 tick 末位姿 | Force（2b 反作用；口径见 D1） | 受体下一 tick 看见 | 统一施加器 |
| 2 | 软体（绳/布） | 同上 | Impulse（Δv）+ Position（Δx） | 同上 | 统一施加器 |
| 3 | 介质（喷溅） | 同上 | Force（阻力）——当前在子步内（D3 迁移项） | — | 统一施加器 |
| 4 | 气动 | 同上 | Force + Torque | — | 统一施加器 |

**统一施加器**（语义名 `ReactionSink`）：

```
coupling::apply_round(&mut bodies, &rounds, &mut audit)
    for r in rounds（固定序）{ for (body, channel, payload) in r { gate → 写 → 记账 } }
```

它是**唯一**允许写"耦合量"的地方（引擎自用的求解器/关节/CCD 除外）⇒ 新增域只实现"产出"侧，**不碰**施加侧。

---

## 4. 接口与落位（遵守仓内棘轮）

### 4.1 复用 ROUTE §2.1 的四接口（谁负责什么）

| 接口 | 在耦合里的职责 | 现状 → 目标 |
|---|---|---|
| `MediumField`（`interop.rs:247`） | **双向介质**：`sample` 读、`deposit` 写 | 只读单向 → 真双向（D3 决策后） |
| `ProviderColliders`（`:166`） | 接触/流形（已在用，不动） | ✅ |
| `ConstraintElement`（`:277`） | 长期：耦合约束进统一求解器（§5 C5） | 零实现（M2 草案） |
| `StateBridge`（`:257`） | 表示转换 + **物理交接**（转换中的动量/约束/warm 残量） | 零实现、只管位置 ⇒ 交接策略待定（§4.3） |

### 4.2 新增的最小接口（放哪：棘轮约束下的落位）

**硬约束**（实测）：`World` 字段 23 / 上限 24（**只剩 1 个位**）、方法 76（登记债务，**只准减**）；
`ClothSheet` 字段 23/24；`Packet` 32 字段；`god.gate.json` 的 `max_type_*`。⇒ 新增一律走
**新文件 + 新类型 + 自由函数 / 扩展 trait**，不往既有 god 类型上加方法。

| 新增 | 形态 | 落位建议 |
|---|---|---|
| 通道枚举与产出类型 | `enum Channel { Force, Impulse, Position }` + `struct Reaction { body: u32, channel: Channel, force: Vec3, torque: Vec3, disp: Vec3, src: EffectKey }` | 新文件 `core/src/coupling.rs`（或 `vxl-phys/src/world_step/coupling.rs`） |
| 受体门 | 自由函数 `gate(&mut BodySet, i) -> Option<Receptor>` | 同上（`BodySet` 的 impl 也可，但注意 `body.rs` 的棘轮） |
| 统一施加器 | 自由函数 `apply_round(&mut BodySet, rounds: &[Round], audit: &mut Audit)` | 同上 |
| 域轮次表 | `const fn`/静态表 + 门禁脚本断言（域清单 vs 表） | 新文件 + `scripts/` 加一条轻量断言 |
| 效应键与让位表 | `enum EffectKey` + `const ALLOWANCES: &[(EffectKey, …)]` | 同上 |
| 审计结构 | `struct CouplingAudit { dropped: [u32; 3], momentum_residual: f32, … }` | 同上；**World 若需持有 ⇒ 用掉那 1 个字段位**，或挂到已有诊断结构（`stats`/`probe`）里 |
| 每域一张"卡" | Rust 侧：`fn card(&self) -> CouplingCard`（各域 crate 内实现）；文档侧：本文 §3.0 模板 | 各域 crate（新增文件），**不改既有 impl 的方法数超限问题**：卡做成自由函数 `fn card_of(domain: &X) -> CouplingCard` |

### 4.3 `MediumField` 真双向要补的签名（D3 若选 (b)）

**✅ 切片 1 已落地（2026-10-01）**：下面 6 条缺口里第 1/4/5/6 条已按**最小口径**覆盖（第 4 条里的
"网格一致性"以"本片不移动核"规避），法向/接触点集与来源 id（第 2/3 条）仍缺——留切片 2 做
法向-切向分离与对称记账时再扩签名。细节、判据与读数见 §5「D3 切片 1」。

现状 `deposit(&mut self, x: Vec3, momentum: Vec3, mass: f32, pressure_work: f32)`（`interop.rs:253`）**缺**：

1. **`dt`（时间窗）**：否则无法区分"力"与"冲量"（对比 2b 明确定义为力，`world_step.rs:252-253`）；
2. **接触法向/接触点集**：只有标量 `x` ⇒ 无法做"法向/切向分离"（阻力-压力分离的前提，对比 `InteropContact` 有 `normal/depth/feature`）；
3. **来源体 id / 特征 id**：对称记账需要"同一对上的反对称"（流体侧正是靠逐对 `−F_i` + 段序，`fluid_force.rs:38-40`）；
4. **场侧状态**：`Splat` 结构没有速度/质量槽（`splat/lib.rs:42-53`），且场**无 `advance`/`dt`**（全目录零命中）
   ⇒ 结构上不可能"吸收动量"；必须补 ① `Splat`（或新增 `medium_state`）的速度/质量表示 ② **谁推进它**（域轮次里加一格）
   ③ 加速结构一致性（核一动，`rebuild_grid` 的网格就脏，`lib.rs:136`）。
5. **`&mut` 通道**：`providers.splat(id)` 只给 `&GaussianSplatField`（`providers.rs:79`），要给 `&mut` 得加 `splat_mut`；
6. **让位规则**：喷溅的"冻结行为阻力段"（`world_step.rs:108-109` 明文"不动"）与新反作用如何共存 ⇒ 效应键解决。

**切片 1–4 覆盖状态（逐条对照）**：第 1 条 `dt`——不扩签名，**由调用侧折进冲量**（`deposit` 收到的
就是 `−F·dt`，注释点名）；第 2 条法向/接触点集——**已做**（切片 4：`drag_split` 开时按**等值面法线**
`n̂ = −∇σ/|∇σ|` 分离为压力/摩擦两项，默认关、零梯度回落，见 §5「D3 切片 4」）；第 3 条来源 id/
对称记账——**以端到端动量账判据形态落地**（切片 4a：逐 tick 对账 `Δabs == m·g·dt − m·Δv`；
`src` id 未扩——账两侧同源、代码里未找到已知丢点，(ii) per-体归属留待取证需求）；第 4 条场侧状态
——**已做**（`kern_vel` + `advance` + `damping`，落在**场**上、不动 `Splat` 记录、渲染桥零变化；
**切片 2 起 `advance` 逐核平流** `center += v·dt` + 网格置脏，`rebuild_grid()` 后与全扫**逐位一致**；
**切片 3：世界 `provider_bounds` 随核漂移刷新**（每子步首段、双向场专属，见 §5「D3 切片 2/3」））；
第 5 条 `&mut` 通道——**已做**（`splat_mut`）；第 6 条让位——**已做**（`two_way` 默认关 ⇒ 冻结段
走原路径，开档才叠加反作用）。

### 4.4 `StateBridge`：表示转换 vs **物理交接**

现状只搬 `Vec<Vec3>`（位置）⇒ 连动量连续都不具备。**转换物理四件**（`PLAN-triangle-first-class.md:83-84`
已点名"`StateBridge` 该落的位置"）应在这里定稿：
`① 转换（表示 ↔ 表示）② 转换中的物理（质量/动量/约束/warm 残量怎么带过去）`
`③ 交接策略（不继承 / 继承速度角速度 / 继承约束 warm 残量 —— 三档，`SURVEY-SOFT-CLOTH-AND-CONVERSION.md:74`）`
`④ 预算降级`。**本设计不解决它**，但把它挂到同一张"契约卡"上（转换也是一种跨域作用，同样要声明时间戳与对称对）。

**⇒ 四件已定稿（2026-10-01）：`PLAN-CONVERSION.md`**（取证 / 设计 / 判据体系 / 四片迁移 V1–V4 / 拍板项 P1–P5）。

---

## 5. 迁移计划（分片 · 每片带判据与换代标记）

> 总原则：**先有账，再改账**。C0/C1 只加观测与收口（**零行为变更**⇒ 冻结读数一字不动）；
> C2 起才动口径，动口径 = 换代级 ⇒ 每片都要"重冻 + 登记 + 拍板"。

### ✅ C0 观测层（**已落地 2026-09-30**，`world_step/coupling.rs`；零行为变更）

**落地形态**（与本文档原设计的一处偏差，如实记）：审计**不持状态**——`World` 的成员位顶在 god 门棘轮上
（`world_struct.rs` 基线 23/24 且该文件 `max_fn_lines = 0`，没有"函数变短"可交换）⇒ 账一律**现算**：
`dropped_force_writes(&BodySet)`（静默丢弃的暴露面）与 `fluid_reaction_ledger(&World)`（2b 反作用
"交给不动的体"多少力）都是纯查询。真消费者 = **`World::health()`**（`HealthReport` 加两字段
`coupling_dropped_bodies/coupling_dropped_force`，**只记账、不进 `is_clean`**）⇒ 无死代码、无新方法。
**5 条判据**（in-crate，`coupling.rs::tests`）：门真值表 · 经门写 + 丢弃可见 · 分账划分
（含越界兜底）· **静态地板**端到端 · **睡眠地板**端到端（后两条断言"反作用非零 + 体不动 + 有账"）。
⚠️ 判据卫生：分账断言全部取**可精确表示的二进制值**（`0.1+0.2+0.4+0.8 ≠ 1.5` 在 f32 下成立）——
十进制小数值会让"账必须平"红在浮点上而不是账上。

### C0 观测层（原设计；已被上面的落地形态取代，保留备查）

| 项 | 内容 |
|---|---|
| 改动 | 新增 `CouplingAudit`（`dropped[3]`、`momentum_residual`、`per_effect[EffectKey]`）；在 §1.2 的 10 个写入点旁**只记账不改逻辑**；新增 `scripts/` 一条轻量断言（域清单 vs 轮次表） |
| 判据 | ① `coupling_audit_counts_dropped_writes`（睡眠体 + 外力 ⇒ dropped 计数 = 预期、体确实没动）② `coupling_momentum_residual_is_recorded`（2b 场景：`Δp_body + Δp_fluid` 相对残差 ≤ 登记阈值——**首次把这笔账量出来**）③ **四哈希 + 全部既有耦合判据逐位不变**（观测层不动行为的金丝雀） |
| 风险 | 无（纯加法）；唯一注意：审计字段若挂 `World` ⇒ 用掉那 1 个字段位（或挂进既有诊断结构） |

### ✅ C1 受体门统一（**已落地 2026-09-30**，与 C0 同片；行为等价已验）

**落地**：`world_step/coupling.rs::is_receptor` 是**唯一定义处**（`is_dynamic && awake && inv_mass > 0`），
`add_force`/`add_tick_torque` 是**力的通道与 tick 末力矩通道的唯一收口**。门面侧五处写入点全部改走它：
`splat_medium_pass`（原先只查 `is_dynamic`）· `medium_pass` 2a · `fluid_reaction_pass` 2b ·
`aero_pass`（快照不受影响，仍逐体留读数）· `world_soft` 绳的**角反作用**（原先**无任何检查**，§2 A3；
默认关 ⇒ 默认档逐位不变）。
**行为等价证据**：`gate_all.sh` 全绿且**金样三条读数与改动前逐字相同**（col45 0.0034/0.0016、
pile5 0.0041/0.0020、tower25 0.0950/0.0572）+ determinism 哈希一致；既有耦合判据 14 例全绿
（`fluid_boundary` 4 · `fluid_coupling` 3 · `rope_scene` 4 · `cloth_reaction_scene` 1 ·
`angular_impulse_contract` 2）。
**未做（如实）**：`vxl-phys-field` 的力场写入仍不过门（另一个 crate，语义是"对全体动体累加"）
⇒ 那部分暴露面由 `dropped_force_writes` 记账；软体两腿的门仍在**代理快照侧**（`inv_mass = 0`）不变。

### C1 受体门统一（原设计；已被上面的落地形态取代，保留备查）

| 项 | 内容 |
|---|---|
| 改动 | 三种 awake 门（无门/`is_dynamic`/`is_dynamic && awake`）统一为 `gate()`；**"写后被积分器丢弃"改为"提前不写"**并把两者都计入 `dropped`；软体代理侧 `inv_mass=0` 的口径保留（它是"呈现为静态"的语义，等价于门） |
| 为什么等价 | 三种写法在物理上都是"睡眠体不生效"（写入被丢 vs 提前跳），差别只在**账**；统一后读数应逐位不变 |
| 判据 | ① 既有全部门（`rope_scene` 三例、`cloth_reaction*`、`fluid_boundary` 四例、`fluid_coupling` 三例）**逐位不变** ② 四哈希不变 ③ 新判据 `sleeping_body_gets_no_coupling_and_is_counted`（断言"写入 0 **且** dropped > 0"——把 A1 变成可判定） |
| 风险 | 门的**求值时刻**必须固定在施加点（求解器可能在同一子步里唤醒体）⇒ 卡里登记求值时刻；唤醒策略本片不改（另立） |

### ✅ C2 的**前置测量已做**（2026-09-30，`crates/vxl-phys-fluid/tests/reaction_sampling_probe.rs`）

**怎么隔离它**（零引擎改动、只用公开 API）：把物理 tick 拆成 `SUB` 次**细 tick** 调用
（`FluidConfig{substeps:1}` + `step(dt/SUB)`）⇒ 每次调用后读 `boundary_reactions()`，即得**一个 tick 内
`SUB` 个逐子步样本** ⇒ 直接算 `F_last`（现口径）与 `F̄ = Σ F_s/SUB`（C2 口径）。场景与
`boundary_accuracy_probe.rs` 同款（水块摊到 0.8² 腔 ⇒ 水深 0.675、盒半长 0.06 摆柱中、`Basin` 提供者围水）。

**读数**（`SUB = 4`，静置 60 + 窗口 120 物理 tick，release）：

| 量 | 末子步快照（现行） | tick 时间平均（C2） | 差 |
|---|---|---|---|
| 窗口均值 `F_y` | 17.1868 N | 17.2836 N | **−0.56%** |
| **P7 读数 `F_y/ρVg`** | **1.014** | **1.020** | +0.6% |
| 侧向窗口均值 (x, z) | (−0.034, −0.496) N | (−0.008, −0.466) N | 同量级 |

**逐 tick 相对偏置 `|F_末 − F̄|/|F̄|`**（窗口内）：
`F_y` 中位 **0.111** / 最大 **0.397** / 均值 0.134；侧向（归一到 `|F̄_y|`）中位 0.179 / **最大 1.036**；
`|τ|` 中位 **0.382** / 最大 **1.915**。**tick 内逐子步峰峰离散度**最大 **0.745**。

**⇒ D1 的判据（本轮实测给出）**：
1. **窗口均值口径（P7 表）几乎不动**（−0.56%）⇒ C2 **不会**把精度表推翻，重冻成本小；
2. 但**逐 tick 的瞬时交付**偏 **11%（力）/ 38%（力矩）**，最坏 40%/190% ⇒ 对**瞬态**场景
   （溅水、溃坝、抛射、单 tick 冲击）这是**实质性误差**，且正是 P7 里"波动透传到体上"的一半来源
   （facade 把 11% 的抖动当常量力灌一整 tick）；
3. ⇒ 按"账要清 + 瞬态要对"的标准：**建议 (b) 时间加权**；若只关心稳态浮力，可"不做 (a) 但登记"。

### ✅ C2 已落地（**2026-10-01**，D1 拍板 (b)：dt_sub 时间加权 tick 平均）

**落地形态**（与施工单的一处偏差，如实记）：通道声明（`Channel`/`Reaction` 类型 + 四域"卡"）
**本片未做**——它是 C3 域轮次收口的载体；本片只改**采样口径**本身：

- **CPU**（`vxl-phys-fluid`）：`breact` 改 **tick 级累加器**——`step` 开头清零、每子步力相末尾
  `aggregate_reactions` 按段位置就地累加（求和序不变）、末尾 ÷ 子步数；**子步 = 1 不做除法**
  ⇒ 单子步档逐位不变。无新字段（`FluidSystem` 在 god 门登记债务中，成员只准减）。
- **GPU**（`vxl-phys-gpu`）：`reduce.wgsl` 加孪生入口 `reduce_add`（就地累加；覆写版 `reduce`
  保留给 report 自洽腿）；`ReactionStage::tick_average` = `begin_tick` 清账（`react_b` 补
  `COPY_DST`）→ 每子步 `encode_substep` + `encode_accumulate`（同一条命令链、逐子步提交）
  → 末子步回拷 → **一次**回读 → 主机 ÷ 子步数。步进器与两个探针全走它；缓冲零新增；
  sorted 副本档零接触（reduce 读的是 scatter 后的规范序 `out`）。
- **判据兑现**：① 新判据 `reaction_sampling_is_time_weighted`（合成线性力 1..4 ⇒ `F̄` 逐位
  = 2.5·nb、≠ 末值，in-crate）② 既有判据全绿：fluid 4 例 + **rope 三冻结哈希一字未动** +
  `box_on_rope` 托住 + 14 格稳健性表 + `parallel_equals_serial_bitwise`；**一处阈值重冻**：
  `fluid_boundary::body_pushes_water_only_with_boundary_coupling`（体推水）的差值阈
  0.15 → **0.10**（实测 +0.130 vs 对照 +0.000——入水冲击瞬态被 tick 平均抹平的直接换代效应，
  判别力"非零 vs 零"不变）③ P7 表重测（下）④ `gpu_coupling_probe`/`--tank` 口径 B 读数不劣化（下）。
- **换代登记（动了什么 / 没动什么）**：动 = 多子步（默认 4）2b 的逐 tick 反作用绝对读数
  （瞬时交付从"末子步快照"变"tick 平均"；窗口均值只差 −0.56%，前置测量）；**没动** = 流体自身
  演化（反作用不回灌流体 ⇒ `gpu_coupling_probe` 漂移表 0→0.00216 m 与旧表逐字相同）、
  无流体四哈希、金样三门、rope 三冻结哈希。
- **同批换代：`angular_reaction` 翻默认**（§9 P1 第三行提前与本片同批）：
  `Rope::angular_reaction` 默认 `false` → `true`。依据 = §8.4.31/§8.4.32 前瞻测量 + 判据守门
  （`angular_reaction_holds` 带转动自扮引擎 + `rope_scene` 偏置判据）+ 本批复核（
  `box_on_rope_in_a_world_is_held` 默认档托住、14 格全托、三哈希未动——角腿只在
  绳-动态体接触时介入，静态体/无体场景逐位不变）。残留局限不变：接触模型看不见转动
  （`world_soft.rs` 头注），转动感知代理仍属 §9 P2。

**读数（重冻登记，2026-10-01，本机 release）**：
- **P7 表**（`OPEN-PROBLEMS.md` §P7）：窗口均值不劣化（轴① 1.53/0.98/1.01/1.01/0.90 vs 旧
  1.52/0.98/1.02/1.00/0.91）；**细分辨率波动大降**（轴② ±4.80→**1.84**、±3.18→**1.31**——
  末子步采样偏置被消的直接证据）；h=0.1 档 ±1.64→1.65 持平 = 自由面晃动的物理项，不随采样口径消失。
- **`gpu_coupling_probe 20 240`**（口径 B）：第 1 tick relF **1.53e-6**；逐 tick 最坏 relF
  **8.20e-5** / relT **2.81e-4**（旧 1.05e-4/4.41e-4）；动量账两侧之差 **2.01e-6**（旧 4.91e-6）；
  非有限 0；漂移 0→0.00216 m 与旧表一致。
- **`gpu_tick_probe --tank`**（20³ + 16000 边界，锁步 1 tick）：逐粒 max|ΔF| **7.4e-5**
  （rel 5.08e-7）；每体 tick 平均 max|ΔF| **1.04e-3**（rel **6.55e-6**）/ |Δτ| rel **1.48e-5**
  （旧 9.13e-6/2.23e-5）；自洽 1.21e-5。

### C2 通道声明 + 反作用采样口径（**D1 决策后；换代**）——原施工单（保留备查）

| 项 | 内容 |
|---|---|
| 改动 | 引入 `Channel`/`Reaction` 类型并为四条路径写"卡"（先不改行为）；若 D1 选 (b)：流体反作用由"末子步快照"改为 **`dt_sub` 时间加权的 tick 平均力**（`F̄ = Σ_s F_s·dt_sub / dt_tick`） |
| 判据 | ① P7 的窗口均值表**不劣化**（预期改善：末子步采样偏置被消掉）② `fluid_boundary` / `fluid_coupling` 门通过（阈值按**新口径**重冻并登记）③ `gpu_coupling_probe` 逐粒/每体读数重测（两侧同口径）④ 新增 `reaction_sampling_is_time_weighted`（构造"力线性增长"的合成场景，断言 `F̄` 等于解析平均、且 ≠ 末值——**有分辨力**） |
| 换代标记 | ⚠️ 2b 的所有绝对读数会变 ⇒ 重冻 P7 表 + `fluid_boundary` 门阈值 + 登记 |

### ✅ C3 片 1 已落地（2026-10-01）：声明式轮次表 + 一致性断言（**零行为**）

**落地形态**（与施工单的偏差，如实记）：轮次表**不新增运行时面**——表与断言都落在
`crates/vxl-phys/tests/coupling_rounds.rs`：对 `world_step.rs` / `world_soft.rs` 两个编排文件
做 `include_str!` 源扫描，收 `_pass` 形状的调用点（实测扫描集 = 跨域 7 + 豁免 4），两向断言
「扫描集 − 豁免集 == 登记表」与「登记表 ∪ 豁免集 ⊆ 扫描集」⇒ **新域不登记即红、改名/删域
不同步即红**；扫描器自带金丝雀（合成源必命中）并做了注入实测（末尾加一行注释
`probe_stray_pass` ⇒ 红 ⇒ 撤回，证据记在判据头注）。**没做**：`apply_round` 统一施加器与
四域改「产出 Reaction → 统一施加」= **C3 片 2**（动软体施加路径与热路径零分配纪律
⇒ 独立片、三条冻结哈希 + 四哈希守门）。

### ✅ C3 片 2 已落地（2026-10-01）：统一施加器 `apply_round` + 三通道类型（**行为逐位不变**）

**落地形态**：`world_step/coupling.rs` 新增 `Channel` / `Reaction` / `apply_round`（§3.5/§3.6 的
`ReactionSink`）。产出一侧——**流体 2b** 与**软体两腿**改为「产出 `Reaction` 迭代器 → 统一施加」
（迭代器零分配 ⇒ **不占 `World` 字段位**；序 = 产出序 ⇒ 与改前**同一加法序**）。
**如实登记的偏差**：介质（2a/喷溅）与气动是**单通道 Force** 域、段位在子步内逐体即时施加
——它们留在 `add_force`（唯一力漏斗）上不经列表（转经只增间接、不改账）；介质 2a 原先是**就地写
累加器**，本片一并收口。"唯一落点"由新判据 `tests/coupling_sole_writer.rs`（源扫描 + 双金丝雀）
机械断言——**修前实测命中 1 处**（world_step.rs 的 2a 就地写），收口后为 0。
**判据兑现**：`coupling_sole_writer` / `coupling_rounds` / `rope_scene`（**三冻结哈希**）/
`fluid_boundary` / `fluid_coupling` / `fluid_covered_multi` / `angular_impulse_contract` 全绿 +
四哈希 + 金样门逐位不变（gate_all 全量通过）；in-crate 新增判据 ⑥（三通道落点 + Force 过门）。
（片 2 第 1 步的软体两腿"纯搬移"已被本体覆盖/吸收，历史见提交 `15d25bf`。）

| 项 | 内容 |
|---|---|
| 改动 | `domain_pass` 的四行调用 → 声明式轮次表 + `apply_round` 统一施加器；软体两腿/流体反作用/介质/气动都改为"产出 `Reaction` → 统一施加"；**段位保持不变**（软体仍在 tick 末、流体反作用仍在子步 D）⇒ 行为应逐位不变 |
| 判据 | ① 软体三条冻结哈希（`0x5a24_4091_4067_fe15` / `0xfaf4_d9e7_443d_dbef` / `0x36ea_adb9_02cc_8481`）**逐位不变** ② 四哈希 + 金样门不变 ③ 新门禁断言"域清单 == 轮次表"（新增域忘登记即红） |
| 备注 | 本片**是**"以后好收"的关键：之后新增域只需声明"卡"，施加侧不再复制粘贴 |
| C3'（换代，另拍板） | 若要把 Impulse 通道真正搬到"求解器同段位"（速度积分后、位置积分前），会改软体读数 ⇒ 单独一批、单独冻结 |

### ✅ D3 切片 1 已落地（2026-10-01）：喷溅场作介质真双向（**默认关**，零代际）

**落地形态**（相对下面施工单的收窄，如实记）：
- 新文件 `crates/vxl-phys-splat/src/flow.rs`：`GaussianSplatField` 加 `two_way`（默认 false）/
  `kern_vel`（逐核速度，与 `splats` 同序同长；`set_two_way(true)` 惰性补齐）/ `damping`（0.98）/
  `absorbed`（审计）+ `advance(dt)`/`deposit`/`sample` 速度项改造。
- **介质→体**：`sample` 速度项 = 核速度 **e 加权插值**（`exp(−α/2)·opacity`，与 `density_grad`
  同式同 `cut`；候选走同一条 `candidate_ids` ⇒ 序列逐条相同、逐位确定）；无近核 ⇒ 回落常值
  `medium_velocity`。
- **体→介质**：`deposit` 按同一核权重分摊 `Δv_k = (−F·dt)·(e_k/Σe)/m_k`，核质量
  `m_k = ρ·(4/3)π·σx σy σz`（1σ 椭球口径）。
- **轮次**：`domain_pass` 加一格 `Providers::advance_medium`（逐核 ×`damping`）；**本片不移动核位置**
  （位置演化 + `rebuild_grid` 一致性 = 切片 2）。
- **接线**：段①整体拆到 `world_step/medium.rs`（含 `splat_mut`/`advance_medium` 的 `impl Providers`；
  `world_step.rs` 文件行数净减、`providers.rs` 零改动）——god 门下的净账拆法（沿 `aero.rs` 先例）。
- **默认关 ⇒ 零代际**：`sample` 速度恒为 `medium_velocity`、`deposit`/`advance` 空操作。

**判据**（全绿）：in-crate 4 条（关档金丝雀 · `Σ m_k·Δv_k == J` rel<1e-4 · 单核场插值逐位退化 ·
`advance` 阻尼 ×0.125 逐位）+ 端到端 1 条（下节「D3 切片 1」读数表）。守门证据见 EXPERIMENTS 同节。

### ✅ D3 切片 2 已落地（2026-10-01）：核位置平流 + 网格一致性（默认关延续，零代际）

- `advance(dt)`：逐核 **`center += v·dt`**（零速核跳过）后按 `damping` 衰减；核一动 `grid = None`
  （查询退回全扫，与网格**逐位一致**；加速由消费者按需 `rebuild_grid()`）。`two_way` 关 ⇒ 首行短路。
- 判据：in-crate +2——**平流精确**（一步 `center += v₀·dt`、同一步 `v ← v₀·damping`，均逐位）·
  **平流后网格一致性**（移动 10 步 + `rebuild_grid()` ⇒ σ/∇σ 与全扫逐位）；端到端 +1（关档核中心
  逐位不动；开档 `moved=7/16、ΣΔ=(0, −0.543, 0)`，方向 = 体失去的动量）。
- 读数换代效应（on 档）：`splat_medium_two_way_reduces_drag` on 6.987/−1.320（切片 1）→
  **7.023/−1.261**（尾流被带走 ⇒ 反馈略弱、同向）；关档逐位不变。边界（供者包围盒不随漂移刷新）
  与读数见 EXPERIMENTS 同节。

### ✅ D3 切片 3 已落地（2026-10-01）：世界 AABB 随核漂移刷新（默认关延续，零代际）

- 切片 2 留档的边界（`provider_bounds` 是注册时快照）在此解掉：`splat_medium_pass` 每场先查
  `two_way` ⇒ **双向场用当前核位置重算 AABB**（`ProviderColliders::bounds`）写回
  `provider_bounds`——介质采样门与宽相 AABB 都读它。位置在本 pass（每子步首段）⇒ 恰在本子步
  宽相之前拿最新几何。单向场（默认）不进 ⇒ 零额外成本、逐位不变。
- 判据（端到端 1 条）：单核场注入 −y 动量平流 1 s ⇒ 关档 AABB/核**逐位不动**；开档
  `min.y −2.100 → −3.577`（核 y=−1.502）；探点「旧 AABB 外 ∧ 新 AABB 内」采样非真空
  （3.38e-2 vs 关档 0）——修前会把它当真空跳过。

### ✅ D3 切片 4 施工单（法向/切向分离 + 对称账）——**已按建议落地（见 §5「D3 切片 4」）**

**拍板记录（按施工单建议执行）**：4a 取 (i) 总线账/判据形态（`src` id **未扩**——账两侧同源、
代码里未找到已知丢点；(ii) per-体归属留待取证需求）；4b 取 **−∇σ 等值面法线** + 新旋钮
`cd_normal/cd_tangent` 默认 = 现行 `Cd`。以下为原施工单文本（保留备查）：

§4.3 六条缺口剩第 2（法向/接触点集）与第 3（来源 id/对称记账）。两件的落位与判据先钉死：

**4a 对称记账（`src` id）——先拍板"账的粒度"**：`deposit` 加 `src: u32`（来源体索引）是机械的
（`MediumField` 实现只有喷溅场一处）；障碍在**账存哪**：per-(体, 场) 的冲量无法从当前状态现算
（`bodies.force` 是混合累加器），而 `World` 成员位顶格（23/24）⇒ 若存历史必须拆域侧结构
（`Providers`/`soft.audit` 之类）。两条路：
- (i) **每 tick 总线账**（零新字段）：`splat_medium_pass` 末尾把本 tick 的 `ΣJ` 与该场
  `absorbed` 增量对拍（构造性恒等 + "无近核"分支的合成用例）——回归守门；
- (ii) **per-体归属**（要新结构）：账按 `(field, body)` 分组，为"谁在贡献"提供取证。
建议先 (i)（零结构变更、够守门），(ii) 等真有取证需求再立项。

**4b 法向/切向分离（物理，默认关）——先拍板"法线来源与系数"**：
- **法线来源**：载体介质没有接触面 ⇒ 取**等值面法线** `n̂ = −∇σ/|∇σ|`（采样点处；梯度本就在
  `sample` 里算过，复用零新成本）；退化为零梯度时回落到"与 `v_rel` 反向"（即现状）。
- **分离式**：`F = −k‖·(v·n̂)n̂ − k⊥·(v_rel − (v·n̂)n̂)`（压力项沿法线、摩擦项沿切向）。
- **系数选项**：(i) 同 `Cd`（纯几何分离、零新参数，但物理上压力/摩擦本不同）；(ii) 新旋钮
  `medium_cd_normal/medium_cd_tangent`（默认 = 现有 `Cd`，关档逐位不变）。建议 (ii) 但**默认值取
  同 Cd** ⇒ 开分离开关时行为连续可对拍。
- **开关落位**：`GaussianSplatField` 加 `drag_split: bool`（默认 false；`two_way` 之外独立子开关，
  沿 `Plastic.bend` 先例）；`deposit` 收**总 J**（不分离）——分离只影响力侧。
- **判据草案**：① 关档逐位不变（默认档金丝雀）；② 斜置板解析对拍：静止斜板在常 `v` 流中的
  法向/切向分量各自与闭式 ≤1e-5；③ 分离→同系数退化为现状（`k‖ = k⊥ = Cd` 时与未分离**逐位**）；
  ④ 零梯度回落用例（远场/纯真空）。

**两件的共同纪律**：`MediumField::deposit` 签名一变 ⇒ `interop.rs` trait + 喷溅实现 + 调用点
（`world_step/medium.rs`）三处同步；**默认关、零代际**；PLAN §4.3 覆盖状态表随片更新。

### ✅ D3 切片 4 已落地（2026-10-01）：法向/切向分离（`drag_split`，默认关）+ 端到端动量账

- **4b 分离式**：`GaussianSplatField` 加 `drag_split`（默认 false）/`cd_normal`/`cd_tangent`（默认
  1.0 = 现行 `DRAG_CD`）。开且 `cd_n ≠ cd_t` ⇒ `F = −½ρA|v|·(cd_n·v_n + cd_t·v_t)`，法线取**等值面
  `n̂ = −∇σ/|∇σ|`**（复用 `density_grad`；`|∇σ| ≤ 1e-6` 回落未分离式）。**两条退化路径都逐位**：
  关档 = 原式原样；`cd_n == cd_t` = 同式换系数（判据②钉死）。`deposit` 只收总 J（不分离）。
- **4a 端到端账**：逐 tick 对账 `Δ(场侧 absorbed) == m·g·dt − m·Δv(体侧)`（两侧来源独立：
  场累计 vs 体状态差）——把"体收到的作用 == 场吸收的动量"的反对称钉在端到端判据上。
- 判据 5 条（全绿）：in-crate 4（① 默认关逐位 · ② 同系数逐位退化 · ③ 轴上解析对拍 ≤1e-5 ·
  ④ 零梯度回落逐位）+ 端到端 1（动量账，90 tick 最坏相对误差 **5.59e-6** ≪ 1e-4 预算）。
- `src` id 未扩：账两侧同源（`deposit` 收到的就是 `−F·dt`，代码里未找到已知丢点）；per-体归属
  （施工单 (ii)）留待真实取证需求。**默认关 ⇒ 零代际**（关档走原式、逐位）。

### C4 介质真双向（**D3 决策后**）

| 项 | 内容 |
|---|---|
| 改动 | 给喷溅场补"能吸收动量的状态"（核速度/或 `medium_state` 粒子）+ 谁推进它（轮次表加一格）+ `deposit` 签名扩到 `(x, normal, momentum, mass, pressure_work, dt, src_id)` + `splat_mut` 访问器 + 加速结构一致性（核动则 `rebuild_grid`） |
| 判据 | ① `medium_two_way_momentum_balance`（吸力/推力的动量账 ≤ 阈值）② `medium_buoyancy_matches_fluid_2a`（同一场景：介质浮力与流体 2a 的窗口均值同量级——两条路径的**口径对齐**判据）③ A/B：单向 vs 双向的轨迹差**可测**（有分辨力）④ 喷溅既有 7 条单测不变 |
| 风险 | 场状态 = 新的时间步与稳定性问题（`dt` 与松弛）⇒ 参考 §7 的"软约束/子步"结论；**别做成隐式大步** |

### C5 耦合进统一求解器（**长期**，接 ROUTE §5 ④）

把"软体↔刚体接触"与"流体↔刚体"从"事后回填"改成 **`ConstraintElement`**（统一约束元素）⇒ 从根上消掉
1 拍滞后与位置级注能（外部对标：FleX 统一粒子 / Avian 同子步循环 / PhysX `PxDeformableAttachment` 的接口化）。
判据：滞后测量归零、动量残差 ≤ 阈值、性能劣化 ≤ 10%（ROUTE §2.2 红线）。

---

## 6. 判据体系（不变量 → 测试）

| 不变量 | 断言 | 建议位置 |
|---|---|---|
| **I1 对称性** | 成对来源的 `Δp_body + Δp_domain` 相对残差 ≤ 阈值；单向来源在卡里声明 `None` | `crates/vxl-phys/tests/coupling_ledger.rs`（新） |
| **I2 无静默** | 任一被门丢弃的写入必须计数：睡眠体场景 `dropped > 0` 且体未动 | `coupling_gate.rs`（新） |
| **I3 单一来源** | 一个 `EffectKey` 在一个 tick 的施加路径数 == 1；冲突清单为空 | `effect_keys.rs`（新） |
| **I4 时间戳一致** | 每条路径断言其登记的时间戳与滞后拍数（卡 vs 实测：同一场景跑两跑、比对"受体何时看见"） | `coupling_cards.rs`（新） |
| **I5 确定性** | 所有施加序 = 索引序或几何序（纯状态函数）：同场景两跑逐位一致（既有判据已覆盖，新增**多域同场**场景） | `multi_domain_determinism.rs`（新） |
| **I6 通道纯净** | 位置通道必须给出 `J_equiv` 与"为什么不进速度级"；位置通道不得写速度 | `coupling_cards.rs` |

**反空跑要求（本仓惯例，三连栽的教训）**：每条新判据都要带"撤掉机制必红"的金丝雀
（例：I1 的残差判据要配一个"人为破坏对称（只给体施力不给流体）⇒ 残差爆表"的对照）。

**冻结读数清单（任何片都不许动，除非该片明确是换代）**：
`determinism FINAL_HASH=0x711be572…`、m0 压力哈希、金样门 col45/pile5/tower25、
rope 三哈希（`0x5a24…`/`0xfaf4…`/`0x36ea…`）、`grid_table`/`packet_grid_canary` 等 GPU 冻结表。

**性能门**：新片不得让已通过档位劣化 >10%（ROUTE §2.2）；审计层自身开销 ≤ 0.5% tick（C0 需实测）。

---

## 7. 外部对标（结论 + 出处 + 可借用机制）

### 7.1 per-engine：力/冲量/位置分别在哪生效

| 引擎 | 力 | 冲量 | 位置修正 | 统一程度 |
|---|---|---|---|---|
| **Rapier** 0.36 | `add_force` 累加，**不自动清零**（需 `reset_forces`） | ①用户 `apply_impulse` 立即改速度；②求解器每**子步** PGS 冲量；③joint warm start 可缩放 | **无独立 position solver**：几何误差走**有上限的修正速度**（`normalized_max_corrective_velocity` 默认 3.0）+ 每子步两遍（biased / unbiased-relax，摩擦只在 relax 遍） | 高：刚体与软体粒子同处一个子步循环与 solver 槽区；软体在 island 里用**隐藏刚体代理**（`SoftFrame`），刚-软走 `SoftParticleAttachment`（带 warm-start 冲量） |
| **PhysX 5** | `addForce`，**下一步自动清**（`eRETAIN_ACCELERATIONS` 可保留） | TGS：子步数 = position iterations；每子步解完全部约束后**立即积分速度**；velocity iterations 只解最后子步的无偏方程 | "position iterations 解几何+速度误差"；跨帧携带速度迭代的后解速度 ⇒ 文档自述 **split impulse strategy** | 框架级统一（同一 `PxScene`/同一 step），**算法级不统一**：可变形体是独立 **XPBD** 求解器；跨域只走 `PxDeformableAttachment`（且明文"刚-刚/刚-世界必须用 joints"） |
| **Jolt** | `AddForce` 累加，`ResetForce()` 在 **post-integrate** 每 collision step 末清 | 顺序冲量 + warm starting（Catto GDC 2009）；warm 冲量按 Δt 缩放**封顶 4.0** | 独立阶段 `SolvePositionConstraints`（`numPositionSteps`），修正施加到 transform | 中：软体是同一 `Body` 体系但走**独立 stage**、也是 XPBD；软-软碰撞未实现 |
| **Bullet** | 刚体常规通道 | 刚-软在 cluster 用**冲量分配系数**（`kSR_SPLT_CL`），`btSoftBody::Body::applyImpulse` 把反向冲量加到刚体（动量对称） | 结构上"两套循环串联"：刚体步 → 软体约束 → 自碰撞 → 更新软体 ⇒ 结构性 1 步滞后；刚体侧 split impulse 默认开（`m_splitImpulse`） | 低-中 |
| **MuJoCo** | 阶段 20–23 力/加速度/约束力 | 约束力 = 软约束的乘子（**离散积分下约束力与执行更新的有效惯量同口径**） | **无位置投影阶段**；明文"硬约束极限**不允许**"（软接触互补性被有意违反） | 单求解器单口径，但**不混域** |
| **Chrono** | 固体→流体传**位置+速度**；流体→固体传**力+力矩** | 无共享冲量层 | 无 | **接口级统一**：`ChFsiSystem` + `ChFsiInterface`，"explicit force-displacement co-simulation"，两相**同时并行推进** ⇒ 结构性滞后 |
| **Avian / bevy_xpbd**（Rust 参考路线） | 每子步 `v += h·f/m` | 位置层约束的 λ；速度层（摩擦/恢复）单独一遍 | 每子步固定序：**积分速度 → 积分位置 → 解位置层约束 → 速度投影 `v=(x−x_prev)/h` → 解速度层约束**；默认 6–12 子步 | 高（同一子步循环里刚体+软体统一） |

### 7.2 文献结论（每条一句 + 出处）

1. **PBD（Müller 2007）**：位置层投影 + 速度事后反推 ⇒ "位置修正必然改写速度"，这是位置级耦合改能量的机制根源。`DOI 10.1145/1276377.1276390`
2. **XPBD（Macklin 2016）**：柔度 `α̃ = α/Δt²` 使刚度/阻尼**与 dt、迭代次数无关** ⇒ 子步数成为可自由加的自由度。`DOI 10.1145/2994258.2994272`
3. **Small Steps（Macklin 2019）**：**n 子步 × 每步 1 次迭代优于 1 步 × n 次迭代**（约束误差与阻尼显著更小）。`DOI 10.1145/3309486.3340247`
4. **统一粒子物理（Macklin 2014 / FleX）**：所有域 = 粒子 + 约束，**共享同一 PBD/SOR 求解器**，双向交互是"同求解器"的自然结果而非两求解器之间的桥。`DOI 10.1145/2601097.2601152`
5. **XPBD 刚体（Müller 2020）**：刚体 = 粒子 + 朝向；用"最新约束方向"（不冻结 Jacobian）；子步 = 最大子步数 + 每子步 1 次迭代，**改善能量守恒、减少穿隧**。`DOI 10.1111/cgf.14105`
6. **Akinci 2012（刚-流双向）**：边界粒子体积校正 `Ψ_b = ρ0·V_b`；"fluid 与 boundary 之间的压力/黏性力**成对对称，守恒线动量与角动量**"。`DOI 10.1145/2185520.2185558`（**本仓 2b 正是这条**：逐对 `−F_i` + 段序聚合）
7. **added-mass 不稳定（Causin 2005）**：分区松耦合在固/流密度接近时数值不稳定，**且基本与时间步长无关**（细化 Δt 不能解决）。`DOI 10.1016/j.cma.2004.12.005`
8. **滞后 = 负阻尼（Chrono×preCICE 文档）**：显式耦合读到的伙伴数据滞后一拍，"对回复力而言**表现为负阻尼，按窗口大小成比例注入能量**，且细化窗口只是减慢增长"。`api.projectchrono.org/precice_coupling.html`
9. **定点迭代本身仍不稳定**（隐式 ≠ 稳定）：需要 Aitken / IQN-ILS / IQN-IMVJ 等加速器。`precice.org/configuration-acceleration.html`
10. **子循环（subcycling）**：子循环只细化参与者自身积分，**不改耦合交换率**；粗细子步比过小有不稳定带、两侧离散混用会引入假振荡。`DOI 10.1002/fld.4688` + `precice` 文档

### 7.3 可直接借用的机制（本仓适配版）

| 机制 | 解决本仓的哪一条 |
|---|---|
| **每步清零的力累加器 + 显式保留开关**（PhysX/Jolt/Box2D；反例 Rapier/MuJoCo 是持久） | 明确"Force 通道 = 一拍一次"的语义（我们的累加器已是每子步清零 ⇒ 比它们更细一档，需在卡里写清） |
| **split impulse / 独立 position 阶段**（Bullet/PhysX/Jolt） | 位置通道的**隔离**：几何误差不跨帧携带 ⇒ 本仓 CCD 已如此，XPBD 位置腿应照此登记（`J_equiv` 只审计） |
| **`max_corrective_velocity`（修正速度上限）** | 位置级/偏置速度的注入速率闸（本仓已有 `max_corr`，`constraint.rs:279`）⇒ 位置通道应带同类闸 |
| **`SoftParticleAttachment` + 隐藏刚体代理**（Rapier） | 长期 C5：软体进 island/求解器的接口形态 |
| **`PxDeformableAttachment` + "刚-刚必须用 joints"的路由规则** | 效应键/路由规则的先例（明文禁止某类组合） |
| **统一粒子 + 共享求解器**（FleX） | C5 的目标形态（本仓有确定性/CPU 顺序冲量的约束 ⇒ 只能部分借鉴，见 §7.4） |
| **window + 子迭代 + 加速器**（preCICE） | 若将来做"强耦合/紧耦合"（D1 选 (c)），这是标准做法与陷阱清单 |
| **接触修改门**（Rapier `PhysicsHooks` / PhysX `PxContactModifyCallback`） | `EffectKey` 让位表的先例（"谁能改接触"的单一收口） |

### 7.4 不适用 / 存疑（依赖本仓没有的前提）

1. **FleX/统一粒子的前提是 GPU + 稀疏粒子-约束并行 SOR**；本仓是 CPU 顺序冲量 + 严格 f32 + 位级确定性（`SPEC §5`）⇒ **不可直接平移**；C5 只能是"约束元素进同一求解器"，不是"所有域都变粒子"。
2. **PhysX"统一时间步"只有框架级出处**；其可变形体是独立 XPBD 求解器、`minVelocityIters` 被忽略 ⇒ 引用时应限定为"框架级统一 + 接口级耦合"（与本设计 C3/C4 的定位相同）。
3. **"位置投影不注入能量"没有单一权威证明**；业界处理是工程性的（split impulse / 修正速度上限 / 柔度 / 子步），互不同量纲、不能互相替代论证 ⇒ 本仓只做"隔离 + 审计 + 闸"，不宣称守恒。
4. **强耦合分区 + subcycling 的定量稳定性结论**（不稳定带、BE-HHT 等）多来自二手摘要 ⇒ 引用时只用一手文献（Causin 2005 / De Moerloose 2019 / preCICE 文档）。

---

## 8. 风险与"不做的事"

**风险**
1. C2 会动 2b 全部绝对读数（换代）⇒ 必须与 P7 表、`fluid_boundary` 阈值**同批重冻**，并保留"旧口径"的可复现路径（一个开关或一份旧读数清单）。
2. C3 若只收口不动段位，风险低；一旦顺手搬段位 ⇒ 动软体冻结哈希（**C3' 必须独立拍板**）。
3. C1 的"写后被丢"→"提前不写"：需审计"是否有任何逻辑依赖写入的副作用"（力累加器只在积分器被读、不影响唤醒 ⇒ 已核；但仍要在判据里用四哈希证明）。
4. 多流体 `covered` 修复（C1 或 C3 顺带）：现有端到端判据全是单流体 ⇒ 必须**新增多流体判据**才能证明修对了。
5. 审计层自身的性能开销（C0）与分配（**热路径零分配**是本仓既有纪律）⇒ 预算固定、预分配。

**明确不做（本设计边界）**
- 不把 CPU 顺序冲量求解器换成 XPBD/统一粒子（丢确定性资产与金样；`PLAN-solver-limits.md:328` 已记"外部引擎的整组语义不可单件搬"）。
- 不在这一批里做 GPU 侧的耦合统一（先把 CPU 契约定下来；GPU 档的 `reduce.wgsl` 已同口径，接口不变）。
- 不动求解器内部的接触/关节通道（它们是"引擎自用"，已在同一套冲量语义里）。
- 不用"位置投影"去稳定深穿透（那是修正速度上限的活）。
- 不在 C0/C1 里改任何读数（这是"先有账"的全部意义）。

---

## 9. 队列重排（既有待办 × 本设计）

| 优先级 | 项 | 来源 | 为什么排这里 |
|---|---|---|---|
| **P0** | **C0 观测层**（审计 + 记账 + 域清单断言） | 本文档 | 零行为变更、独立可交付；给后面所有耦合工作提供账 |
| **P0** | **C1 受体门统一** | 本文档 | 消掉三种 awake 门与静默丢（A1）；行为等价 ⇒ 风险最低 |
| **P1** | ✅ **C2 反作用采样口径**（D1 拍板 (b)，2026-10-01 落地）+ P7 波动残余重测（已重测，残余收窄未关） | 本文档 × `OPEN-PROBLEMS.md` P7 | 同源：末子步采样偏置正是 P7"波动"的一半来源 |
| **P1** | ✅ **C3 域轮次收口全部落地**（2026-10-01）：片 1 轮次表 + 一致性断言（#28）· 片 2 第 1 步软体搬移（#29）· **片 2 本体：统一施加器 `apply_round` + 三通道 + 唯一落点断言** | 本文档 | 让"新增域"变成声明式；~~顺带修多流体 `covered`~~（已拆独立片落地，见 §2 C4） |
| **P1** | ✅ `angular_reaction` 翻默认（2026-10-01 与 C2 同批落地，三哈希未动） | `SESSION-2026-09-28-TRIANGLE-CLOTH.md:54` | 它的 `÷dt_sub` 契约正是统一口径的一部分 ⇒ 与 C2 同批最省 |
| **P2** | **C4 介质真双向**（D3 拍板后） | 本文档 × `PLAN-0.2.md:145` × ROUTE §3.1 | 需要新场状态与推进器；先有 C0–C3 的账与收口 |
| **P2** | ✅ **rope 位置腿进虚拟位姿已落地**（2026-10-01）：同款一行（`body_dx`/`body_disp` 同减）；14 格末速收敛 −0.6~−1.4→≈−0.15、大漂移 0.122→0.029；**三冻结哈希未动** ⇒ 有界换代 | `SESSION-2026-09-28-TRIANGLE-CLOTH.md:64` | 留档 §8.4.44 |
| **P2** | 体素↔多边形转换四件（含 `StateBridge` 交接策略）——施工单 **`PLAN-CONVERSION.md`**（2026-10-01） | `PLAN-triangle-first-class.md:83` | 与本文档同属"跨域契约"；几何缺口已消（T1 落地）；"转换也是一种跨域作用"⇒ 卡模板可直接复用 |
| **P3** | C5 耦合进统一求解器 | 本文档 × ROUTE §5 ④ | 长期；须在 C0–C4 之后（否则没有账判断"紧耦合是否更好"） |
| **P3** | 3DGS 物理代理（ROUTE §3.1 第二层）、破坏/地形、性能缺口（P3 30 FPS ~10×）、GPU 窄相/接触 | 既有队列 | 与本契约无冲突，按原优先级 |

---

## 10. 附录：证据索引（本方案引用的关键落点）

**相位与施加**
`world_step.rs:10-31`（tick/子步/域轮次）· `:271-370`（子步 A..N）· `:254-269`（2b 施加）· `:116-250`（介质 2a + 喷溅）·
`world_soft.rs:84-91`（域序）· `:150-154/:208-213`（绳/布施加）· `:224-244`（两腿回填）· `:177-188`（角反作用 ÷dt_sub）·
`integrate/lib.rs:26-51`（力累加器消费+清零+睡眠丢弃）

**契约与接口**
`core/interop.rs:113`（`CollisionProvider`）· `:166`（`ProviderColliders`）· `:247-254`（`MediumField` sample/deposit）·
`:256-275`（`StateBridge`）· `:277-292`（`ConstraintElement`）· `:293-315`（`FluidStepper`）· `:39-60`（`InteropContact`/`MediumSample`）·
`tests/angular_impulse_contract.rs:1-6,53-69`（tick 末注入契约 + 金丝雀）

**流体链**
`fluid_force.rs:31-40,50-60,126-128,171-174,204-207`（成对反作用/串行补趟/末子步快照）·
`fluid_boundary.rs:5-19,121-126`（聚合与段表）· `fluid_access.rs:109-113`（量纲=力）· `fluid_step.rs:13,34-37,70-77,84-114`（段序/体面速度/子步）·
`fluid_medium.rs:7,12-15,76-80`（只读采样 + deposit 空 + 理由）· `gpu/pipeline/reduce.wgsl:1-13,39`、`pipeline/reaction.rs:107-175`（GPU 同口径聚合、单线程/段序）

**软体链**
`soft/rope.rs:109-136,312-337,440-504,511-618,490-496`（两腿定义/清零点/虚拟位姿/施加/几何序）·
`soft/cloth_coupling.rs:43-64,68-87,157-175,201-211,222-276`（布同构 + 与绳的差异登记）·
`soft/rigid.rs:5-40`（代理口径/反作用口径声明）· `world_soft.rs:94-132`（代理快照 + 睡眠=静态）·
`tests/cloth_reaction.rs`、`soft/tests/rope_coupling_robustness.rs:75-120`（14 格）、`tests/rope_scene.rs:217,284`、`tests/angular_impulse_contract.rs`

**介质/场**
`splat/lib.rs:42-53,90,134-198,268-306,372-510`（`Splat` 无速度槽 / `sample` / `deposit` 空 / provider 三点查 / 加速结构）·
`field/lib.rs:25-29,74-94`（力场注册表）· `world_step.rs:108-148`（喷溅段：单点采样、冻结行为）·
`ROUTE.md:22`（四接口表）· `ROUTE.md:73-76`（splat 三层用法）· `PLAN-0.2.md:145`（deposit 单向待做）· `M1-EXIT.md:151,154,167-168`（2b 起点规格与二选一）

**求解器（约束边界）**
`DESIGN-staged-solver.md:161-195`（三结构缺陷：稳定靠重复扫掠 / 子步重跑碰撞 / 几何一次烘焙 ⇒ 注能）·
`solver/island.rs:88-136,190,311`（冲量施加点）· `solver_impl.rs:71-144,271-280,462-469,545-700`（相位编排/睡眠体按静态处理/两分支睡眠）·
`solver/helpers.rs:106-135`（`group_apply`）· `joints/linalg.rs:101-121`（关节直写）· `world_ccd.rs:80-83`（位置级钳位）

**外部对标**：见 §7 各条出处（Rapier / PhysX 5 / Jolt / Bullet / MuJoCo / Chrono / Avian；FleX 2014、PBD 2007、XPBD 2016/2020、Small Steps 2019、Akinci 2012、Causin 2005、preCICE 文档、De Moerloose 2019）。
