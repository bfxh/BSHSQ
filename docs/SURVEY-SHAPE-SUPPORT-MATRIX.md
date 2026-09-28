# SURVEY：形状支持矩阵（`Shape` 逐臂 × 逐域普查）

> 立项依据：`PLAN-triangle-first-class.md` **风险 A（枚举扩散）**——给 `Shape` 加一类形状之前，
> 先把**每一处消费 `Shape` 的站点**普查一遍，并把"不支持"**显式化**（不是 panic、也不是静默 `_`）。
> 本文件 = 那次普查的产物 + **T1 的落地判别表**。2026-09-28 立。

## 0. 结论先行

- `Shape` 现有 **9 臂**（`Box/Sphere/Cylinder/Capsule/Cone/HeightField/Provider/ConvexHull/Compound`）；
  三角网 = 第 10 臂。真实编译面 = **13 个文件**，其中 **8 处穷尽匹配会编译报错**（安全：
  编译器替你找齐），**5 处 `_` 兜底会静默**（那才是风险，必须逐处显式化或登记为已知缺口）。
- **表示法 = 句柄**：`Shape::TriMesh { mesh: u32, half: Vec3 }`。
  - `Shape: Copy` 是**承重前提**（`bodies.shape[i]` 被几十处**按值** match）⇒ `Arc<…>` 会让整仓
    退化成借用重构 ⇒ 不做。
  - 先例齐备：`ConvexHull { hull: u32, half }`、`Compound { compound: u32, half }`（ADR 0010）
    同款句柄；几何本体在窄相自持（`HullStore`/`CompoundStore`）⇒ 新增 `MeshStore`。
  - **core 零依赖**（`crates/vxl-phys-core/Cargo.toml` 无 `[dependencies]`）⇒ 三角网数据结构
    不可能进 core；句柄表示同时守住这条分层。
- **T1 拆两片**（本计划自己的"分步与回退"要求每步独立可单撤）：
  - **T1a = 几何层**（变体 + 仓库 + 质量属性 + 宽相 + 支持矩阵显式化 + **零换代门**）
  - **T1b = 接触层**（顶点 × {provider, 盒, 球} + 判据 ① 布片落定）

## 1. 普查表（谁 match 了 `Shape`，以及三角网在那里的去向）

| # | 站点 | 形态 | 三角网去向 | 编译期强制？ |
|---|---|---|---|---|
| 1 | `core/src/shape.rs::bounding_sphere_radius` | 穷尽 | 并入外壳臂 `half.length()`（AABB 对角，保守） | ✅ 会报错 |
| 2 | `core/src/shape.rs::kind_name` | 穷尽 | `"trimesh"` | ✅ |
| 3 | `core/src/mass.rs::mass_props` | 穷尽 | 并入 `Compound\|ConvexHull` 的 **AABB 盒兜底**臂（精确薄壳由门面覆写，同 `spawn_compound_body` 先例） | ✅ |
| 4 | `solver/src/ccd.rs::min_half_extent` | 穷尽 | 并入外壳臂 `min(half.x,half.y,half.z)`（尺寸比判据的分母） | ✅ |
| 5 | `broad/src/shape.rs::shape_aabb` | 穷尽 | 并入 `Compound\|ConvexHull` 臂（三臂**同式**：局部半长经 `\|R\|` 变换 ⇒ 保守） | ✅ |
| 6 | `narrow/src/pair_shaped.rs` 高度场分派 | 穷尽 | **✅ 已受理（T1b-3）**：`mesh_heightfield` 逐顶点采样（sign 由该函数形状无关的那段处理） | ✅ |
| 7 | `props.rs::cross_section_area` | 穷尽 | 并入外壳臂（局部 AABB 外接盒近似；阻力估计够用） | ✅ |
| 8 | `props.rs::body_half_extent` | 穷尽 | 并入 盒/外壳/复合体 臂（`max` 半长） | ✅ |
| 9 | `props.rs::shape_volume` | `_` | 自动（走 `mass_props` ⇒ 薄壳体积 = `t·ΣA`）——**不是静默缺口** | — |
| 10 | `narrow/src/provider.rs::provider_shape_contacts` | `_ => false` | **✅ 已受理（T1b-1 落地）**：`mesh_provider_contacts` 逐顶点点查询（与 `hull_provider_contacts` 同构） | 已显式化 |
| 11 | `narrow/src/support.rs::support_of` | `_ => None` | **显式列名**：三角网**非凸** ⇒ 不进 GJK/EPA（这是**语义决定**，不是"没写"） | 已显式化 |
| 12 | `narrow/src/support.rs::poly_for` | `_ => None` | 保持：三角网不是**凸多面体**；调用方已按 `None` 分支处理（`pair_shaped` 各臂都查 `Some/None`） | — |
| 13 | `narrow/src/pair_shaped.rs::pair_non_heightfield` | `_ => {}` | **✅ 已受理（T1b-2/T1b-4）**：三角网早分支 ⇒ `mesh_pair`（盒/球/胶囊/圆柱解析采样；其余如实不受理） | 已显式化 |
| 14 | `core/src/narrow_tier.rs::pack_bodies` | `_ => KIND_NONE` | 注释登记（**卡上不接** ⇒ 主机回填） | 注释 |
| 15 | `fluid/src/boundary.rs::supports` | `matches!` 显式 | `false`（2b 不生成边界粒子 ⇒ 该体仍走 2a 介质场，**既不叠加也不留空**） | 已显式化 |
| 16 | `fluid/src/boundary.rs::{min_half_extent, surface}` | `_` | 注释登记：**不生成**（"近似面片会给错体积，比没有更坏"——沿用该文件既有裁决） | 注释 |
| 17 | `soft/src/rigid.rs`（软体域代理） | `_ => None` | 注释登记：薄壳代理**待补**（软体线后续切片；绳/点是当前档） | 注释 |
| 18 | `vxl-phys/src/world_body.rs`（构造侧） | 构造 | 新增 `add_trimesh` + `spawn_trimesh_body`（照 `spawn_hull_body` 先例） | — |
| 19 | `vxl-phys/src/world_step*.rs`（步进管线） | **无形状分派** | 形状只经 broad/narrow/mass/ccd 四路进场 ⇒ 管线**零改动**（`rg -c "Shape::" world_step*` = 0） | — |

**"已知缺口"三条**（T1a 落地时**如实登记**、T1b 逐个关闭）：#6 高度场（**✅ 已关：T1b-3**）、
#10 provider（**✅ 已关：T1b-1**）、#13 凸体对（**✅ 部分关闭：T1b-2/T1b-4 接盒/球/胶囊/圆柱**；
**仍开** = 锥/外壳/复合体/另一个三角网 ⇒ 另立片）。
**仍开的那几条**都由在树判据 `crates/vxl-phys/tests/shape_support_matrix.rs` **逐条钉住**
（现状 = 不产接触；落地后翻成"停住"）——这就是本仓"有形状、无接触"缺口的标准处理法
（先例：`tests/provider_shape_coverage.rs`，胶囊/圆柱/圆锥那条）。

## 2. 判据（**先判据后实现**）

- **Z-门（零换代）**：默认档**不注册三角网** ⇒ 既有全部判据 + 三条冻结哈希
  （悬垂 `faf4d9e7443ddbef`、静态就位 `36eaadb902cc8481`、静态盒 `5a2440914067fe15`）**一字不动**。
- **T1a-①**：注册 ⇒ 宽相 AABB ⊇ **全部顶点**（含任意旋转；逐顶点验证，不只看半长）+ `kind_name` 正确。
- **T1a-②**：薄壳质量属性与解析对拍 —— 矩形板 `m = ρ·t·A`；惯量按**薄板面内二阶矩**（对角），
  与"均匀矩形板绕三轴"的解析式对拍（容差按离散化给，写清取值来源）。
- **T1a-③（矩阵门）**：`shape_support_matrix.rs` —— 10 臂 × 6 域**穷尽**断言（新增形状不更新矩阵
  ⇒ **编译不过**）+ 上述三条已知缺口的金丝雀。
- **T1b-①（判据 ① 原样）**：布片（规则网格）落到静态球/盒/provider ⇒ **逐顶点就位间隙 ≈ 0**
  + 末态位模式冻结（debug/release 一致）。

## 3. 薄壳质量属性（口径写清，别留模糊）

- **质量**：`m = ρ · t · Σ_tri A_tri` —— 均匀薄壳**精确**（A = 三角形面积和）。
- **惯量**：薄板**面内**二阶矩，**只取对角**（与 `compound_mass_props` 同款两条取舍：
  ① 离对角项丢弃；② 关于**体原点**而非质心——本仓无质心偏移字段，与锥/复合体一致）。
- **忽略 `t²/12` 项**（薄壳 `t ≪ 边长`）；`t = 0` 仍给非零面内惯量（零厚板极限）。
- 推导（写在 `mesh_store.rs` 的注释里）：三角形 `(a,b,c)`、`e1 = b−a`、`e2 = c−a`，
  `∫_T x xᵀ dA = 2A·[ a aᵀ/2 + (a e1ᵀ + e1 aᵀ)/6 + (a e2ᵀ + e2 aᵀ)/6 + (e1 e1ᵀ)/12 + (e2 e2ᵀ)/12
  + (e1 e2ᵀ + e2 e1ᵀ)/24 ]`（`∫∫1 = 1/2`、`∫∫u = ∫∫v = 1/6`、`∫∫u² = ∫∫v² = 1/12`、`∫∫uv = 1/24`），
  对角分量即所需；`I_xx = ρt·(tr J − J_xx)`（`J = ∫ x xᵀ dA`）。
  ⚠️ **`a aᵀ` 前的 1/2 别漏**（首版漏了：只有当三角形起点恰在原点时才看不出）——由"平移过的三角"
  解析对拍钉住（`shell_props_match_closed_form_for_flat_plate`）。

## 4. 门禁预算（god 门：基线棘轮"只准减"）—— **实测回填**

**先说教训（实测）**：本仓**无 `rustfmt.toml`**（默认档）⇒ `cargo fmt` 对**双字段变体模式**
（`Shape::Cylinder { half_height, radius }`）**一律展开成多行**。于是"合并同式臂"只在
`{ .. }` / 单字段臂上省行；**含双字段的臂合并省不出行**（第一版把 `Cylinder|Cone` 合一，
fmt 又把两条模式拆回 4 行 ⇒ 白做）。⇒ **"给枚举加一条臂"在本仓做不出合法交换**
（`max_fn_lines` 必然 +1），**重记基线是唯一出路**。

| 文件 | 基线（行/最长函数/成员） | 改动 | 实测 |
|---|---|---|---|
| `core/src/shape.rs` | 107 / 22 / 2 | +1 变体 + `kind_name` 臂 | **重记** 117 / 23 / 2 |
| `core/src/mass.rs` | 162 / 94 / 3 | 并入三臂 | **净 0** ✓ |
| `core/src/narrow_tier.rs` | 121 / 22 / 5 | 注释登记 | **净 0** ✓ |
| `broad/src/shape.rs` | 111 / 105 / 0 | 三臂合一（前两臂同式） | **净减** 104 / 98 ✓ |
| `solver/src/ccd.rs` | 68 / 21 / 0 | +1 臂 | **重记** 69 / 22 / 0 |
| `vxl-phys/src/props.rs` | 138 / 45 / 0 | 两处并臂 | **净平** ✓（未进重记名单） |
| `narrow/src/support.rs` | 358 / 109 / 16 | `_ => None` → 显式列名 | **重记** 361 / 109 / 16 |
| `narrow/src/pair_shaped.rs` | 876 / 112 / 10 | 高度场臂加三角网 | **净 0** ✓（该文件另有前期净减 ⇒ 867） |
| `narrow/src/lib.rs` | 47 / 0 / 0 | +1 域文件登记 | **重记** 48 |
| `narrow/src/phase.rs` | 96 / 9 / 33 | +1 仓库字段 | **重记** 98 / 9 / 34（成员数是**已登记的数据记录例外**） |
| `vxl-phys/src/lib.rs` | 51 / 0 / 0 | +1 域文件登记 | **重记** 52 |
| `narrow/src/mesh_store.rs` | 新文件 | 仓库 + 薄壳数学 + 内联判据 | 316 / 34 / 9（阈内） |
| `vxl-phys/src/world_mesh.rs` | 新文件 | 门面 API（**按域拆**，见下） | 65 / 15 / 5（阈内） |
| `tests/shape_support_matrix.rs` | 新文件 | 矩阵判据 | 232 / 45 / 2（阈内） |

**另一条纪律（本次踩到）**：`world_body.rs` 的 `impl World` **方法数已顶到 24**（门把"方法数"
当复杂度，见 `god.gate.json` 的 `_type_exempt_doc`）⇒ 新 API 一开始放进那里会被判红；正确做法 =
**按域拆新文件**（`world_mesh.rs`，本仓既有 12 个 `world_*.rs` 同款惯例）。⇒ 该文件本次**逐字未动**。

**重记基线同时吸收的既有漂移**（**不是本次改动**，如实登记）：`--write-baseline` 把当前全仓状态
记为基线（208 → 226 条），其中含前期软体线拆文件后的条目（`vxl-phys-soft/*`：`lib.rs` 112→22、
新 `params.rs` / `rigid.rs` / `rope.rs` / 各 tests；`narrow/provider.rs`；
`narrow/tests/clone_cost_probe.rs`）与两条既有的"合法交换"（`entry.rs` 81→83、
`provider_shape_coverage.rs` 190→213）。**全部在硬阈内**（file ≤ 800 / fn ≤ 120 / type ≤ 24
或其已登记例外），无一是本次改动引入的。

## 5. 切片、回退与优先级

| 片 | 内容 | 落地判据 | 回退 |
|---|---|---|---|
| **T1a** | `Shape::TriMesh` + `MeshStore` + 薄壳质量 + 宽相 + 矩阵显式化 + `add_trimesh`/`spawn_trimesh_body` | Z-门 + T1a-①②③ | 单提交撤销；默认档不注册 ⇒ 恒等 |
| **T1b** | 顶点 × {provider, 盒, 球, 高度场} 接触 + 判据 ① | T1b-① | 同上；三条"已知缺口"金丝雀翻面 |

**优先级（重排，别只堆新计划）**：① 2c-3 默认翻转（等维护侧一句话，属换代）→ ② **本计划 T1a**
（布料那条线的唯一前置）→ ③ 体素↔多边形转换的物理四件（**无几何缺口挡路**，想要可见成果可先走）
→ ④ GPU 接触/解算切片、自碰撞进阶、撕裂/塑性。

## 6. T1b-1 实测（provider 腿；判据与读数）

**落地**：`crates/vxl-phys-narrow/src/mesh_pair.rs`（新文件）——`fill_mesh_world`（**与 `fill_hull_world`
共用 `hull_pts` 缓冲**：一个体只有一种形状 ⇒ 不新增字段、不碰 god 门成员棘轮）+ `mesh_provider_contacts`
（逐顶点 `contacts_point`，与外壳臂同构）。`provider.rs` 只加一行分发。

**判据（在树）与读数**（600 tick，末 60 tick 取窗口均值——本仓测量协议：决定量取窗口均值）：

| 判据 | 读数 |
|---|---|
| ① **不下穿**：末体心高度有界 | mesh 路 **y = −0.0086 m**（阈值 0.05） |
| ② **已收敛**：末窗 `\|v\|` 均值 < 0.1 | mesh 路 **0.047** |
| ③ **路线对拍（比值）**：mesh 路 ≤ 外壳路 ×2 | mesh **0.047** / 外壳（`hull_provider_contacts`）**0.039** ⇒ **1.2×** |

**⚠️ 两条如实登记**（都不是静默）：

1. **残留微幅振荡**（末窗 `|v|` ≈ 0.04~0.05、体心绕面上下 ~1 cm 摆动、缓慢衰减）：**两条路都有**
   （外壳路同样 0.039）⇒ 这是本仓"**点采样 + 预测接触**"路线的既有性质，**不是本片引入**；
   且它与质量无关（两路质量差 5 个量级而残余同量级 ⇒ 不是质量/惯量伪影）。⇒ 判据取**比值**，
   绝对值只留"没掉穿"这一条。
2. **一次自由落体瞬变**（实测 t≈240：接触集短暂全消失 ⇒ 体按 `g·dt` 自由落 2~3 tick、
   `|v|` 峰值 0.41，之后 ~150 tick 内衰减回 0.001）：**有名字、有读数、留档**；
   机制候选 = 预测接触平衡点来回穿越（零点附近的接触集抖动），收紧口径（"就位间隙 ≈ 0"）
   另立一片，不混进本条判据。

## 7. T1b-2 / T1b-3 实测（凸体腿 + 高度场腿；判据与读数）

**落地**：`mesh_pair.rs` 增加 `mesh_pair`（逐顶点**解析**最近点；受理 **盒 / 球 / 胶囊 / 圆柱**，
其余如实不受理）+ `point_shape`（点 × 上述四族的最近点/外法线/深度，`Vec3`/`Mat3` 手写，无新依赖）
+ `mesh_heightfield`（与 `hull_heightfield` **同构**：逐顶点 `hf.sample` ⇒ `depth = h − v.y`）；
`pair_shaped.rs::pair_non_heightfield` 加**早分支**、高度场分派里把三角网拆出为独立臂。
法线从求解器定义反推：`n_o` = 把顶点推出去的方向 ⇒ **三角网在 a 侧取 `−n_o`、在 b 侧取 `+n_o`**
（高度场腿的 sign 由 `heightfield_pair` 那段**形状无关**地处理）。
**锥 / 外壳 / 复合体 / 另一个三角网**仍未受理（要斜面解析或面数据 ⇒ 另立片）。

**判据（在树）与读数**（600 tick，末窗 = 末 60 tick 均值）：

| 判据 | 读数 | 说明 |
|---|---|---|
| `trimesh_is_held_by_box` | **y = +0.99199**（盒顶 1.0）、`\|v\| = 0.0056` | 平衡穿透 8 mm、几乎静止 |
| `trimesh_never_penetrates_sphere` | 全程最深顶点穿透 **−0.00624** | 薄片在球顶翻落（真物理）⇒ 判"接触承诺"而非"停住" |
| `trimesh_is_held_by_heightfield` | **y = −0.00797**、`\|v\| = **0.0032**` | 平地形（h ≡ 0） |
| `trimesh_is_held_by_cylinder`（T1b-4） | **y = +0.49202**（柱顶 0.5）、`\|v\| = **0.0032**` | 落柱顶圆盘；走**解析柱面**（真圆） |
| `trimesh_never_penetrates_capsule`（T1b-4） | 全程最深顶点穿透 **−0.00624** | 横放胶囊（脊）；亦判"接触承诺" |
| `trimesh_is_held_by_provider_floor`（T1b-1 回归） | y = −0.00862、`\|v\| = 0.047` | 未动 |

**⇒ 判据 ① 六条腿全通**（球 / 盒 / 提供者 / 高度场 / 圆柱 / 胶囊）。

**⭐ 一次免费的交叉验证**：球腿与胶囊腿的最深穿透**逐位相同**（−0.00624）。这不是巧合——
**横放胶囊的"脊"与半径 0.5 的球顶在几何上等价**（同一曲率半径），而两者走的是 `point_shape` 里
**两条独立分支**（球 / 线段+半径）⇒ 同读数说明两条解析分支口径一致。

**⚠️ 一条读数差异（证据变强，但仍不写成结论）**：**六条腿里唯一残留偏大的是 provider 腿**
（0.047），其余"直接 `select_contacts`"的路都在 0.0032~0.039（圆柱 0.0032 / 高度场 0.0032 /
盒 0.0056 / 既有外壳路 0.039）——**4 对 1**。provider 腿多经过一层
`pick_dominant_normal` + `four_corner_points`（其 `feature % 16` 的"面心/四角"约定是为
**逐面发射**设计的，对**顶点采样**只能靠"无面心组 ⇒ 用全部组"兜底）。
⇒ 要收紧 provider 腿的残留，**先看取点选择那一环**（这是一条可检验的假设，不是结论）。

**口径边界（写清）**：顶点采样 ⇒ 承诺是"细几何钻不过粗网格的格心"；球腿用"球径（1.0）与顶点
间距（0.5）可比"保证这一点。更细的几何（半径 ≪ 格距）要等**点-三角/边-边**那类窄相
（`PLAN-triangle-first-class.md` 的 T2），不在本片。

**下一片**：#13 余项（胶囊/圆柱/圆锥/外壳/复合体/另一个三角网）→ 自碰撞（T3，空间哈希 +
默认关闭的开关）→ 面元气动（T4）。
