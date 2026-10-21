//! flip：**FLIP 分层**的 CPU 口径档（`FluidFamily::CpuFlip`）——`ROUTE.md` §4.8「WCSPH → PBF/FLIP
//! 分层」的第二半（PBF 见 `pbf` 模块）。
//!
//! ## 口径（与 WCSPH/PBF 档的差别）
//!
//! FLIP = **粒子输运 + 网格投影**的混合：粒子照常带着**全速度**走，网格只负责"把速度场投影到
//! 无散"，粒子吃的是投影带来的**速度增量**（`v ← v + (u_new − u_old)`，Bridson《Fluid Simulation》
//! 第 7 章）。这一点是它相对 SPH 档的本质优势：**耗散只在网格那一步**，粒子不会像 SPH 那样把
//! 声波/黏性耗散累积到密度场上 ⇒ 低耗散、大尺度涡能留住（所以 GPU 高精档选它，见 `types.rs`）。
//!
//! ## 每子步八步（`substep`）
//!
//! 1. 邻域网格 + SPH 密度（`density_pass`）：只为层流黏度相位与 `densities()` 口径服务（与 PBF 档同）。
//! 2. 体外力（重力 + 层流黏度）+ CFL：**只改速度**（`pbf::apply_forces`，与 PBF 档共用一段代码）。
//! 3. 建 MAC 网格：从**流体粒子 AABB** 外扩 `MARGIN_CELLS` 格；单元边长 ≈ 粒子间距
//!    （经典 FLIP：grid_step ≈ 粒径）；单元数超 `MAX_CELLS` 时整体放大边长（防病态 AABB）。
//! 4. 分类单元：**空 / 流 / 固**。固 = 体中心落在固体内部（`contacts_point_boundary` 的
//!    `depth > probe ⇔ sdf < 0`，与 `boundary_pass` 同一提供者口径）；流 = 粒子三线性权重和 > 0；
//!    **固优先**（穿透进固体的粒子单元仍算固）。
//! 5. P2G：粒子速度按三线性散射到三套交错面（u/v/w），逐面权重归一；无权重面沿轴外推
//!    （自由表面外侧）；**固体面与网格外边界置无通量（0）**——网格外边界当闭合箱（§BC）。
//! 6. 散度 + 压力 Poisson（求解器在 `flip/solve.rs`）：默认 **CG**（共轭梯度 + 对角预处理，
//!    `cfg.flip_pressure_cg`）、A/B 档是固定次数 **Jacobi**；`cfg.flip_pressure_iterations`
//!    是迭代上限。空单元 `p = 0`（Dirichlet）、固体面 Neumann（不入模板）、固体单元不参与。
//!    几何与上一子步一致时用那份解**热启动**（见下"热启动"）。
//! 7. 面速度 `u ← u − (Δt/ρ)·∇p`（只施加在"至少一侧是流体"的非固体面上）。
//! 8. G2P：三线性采样投影后的面速度，FLIP 混合 `θ = cfg.flip_ratio`（θ = 1 纯 FLIP、
//!    θ = 0 纯 PIC）；CFL 限速后推进位置；末了 `boundary_pass`（复用统一提供者通道 ⇒
//!    固体推出/法向速度归零口径与 SPH 族**同一个**）。`pressures()` = 粒子所在单元压力（Pa）。
//!
//! ## 边界条件（§BC）
//!
//! - **固体**：无通量（面速度置 0、压力 Neumann）⇒ 固体不吸入不吸出；粒子的实际防穿透
//!   仍由 `boundary_pass` 兜底（网格只负责**流体侧**的不可压）。
//! - **空（自由表面）**：`p = 0` Dirichlet ⇒ 表面可自由变形、可破裂/合并。
//! - **网格外边界**：同固体（无通量）⇒ 等价闭合箱。这是**刻意的**：外扩 `MARGIN_CELLS`
//!   格保证自由表面永远在网格内部，所以外边界只可能在流体撑到网格边时才起作用（那时
//!   网格已按 AABB 重建过）。
//!
//! ## 确定性（§5 契约）
//!
//! 遍历序 = 单元索引序（i,j,k 固定）与粒子索引序；压力求解默认 **CG**（点积/逐元素更新全走
//! 固定索引序，无重排、无原子；A/B 档的 Jacobi 是全量读旧写新，也与遍历序无关），不用
//! Gauss-Seidel；无浮点归约重排 ⇒ 同输入两跑**逐位一致**（判据 `tests/flip_incompressibility_probe.rs`
//! 的 `flip_is_bitwise_reproducible`，跑的就是默认档）。
//!
//! ## 验证（`tests/flip_incompressibility_probe.rs`，水槽静水池 60 tick × 4 子步）
//!
//! ① **静水支撑**：判别量 = 末态柱高 / 起步柱高（`height_keep`）。投影若形同虚设，柱子会被
//! 自身重力压成饼 ⇒ 该比值先塌。实测（同场 60 tick，**两种求解器**）：
//!
//! | 迭代数 | 10 | 20 | 40 | 80 | 160 | 320 |
//! |---|---|---|---|---|---|---|
//! | Jacobi | 0.299 | 0.482 | 0.712 | 0.854 | 0.930 | 1.016 |
//! | CG 冷启动（提交 `781b1e1` 实测） | 0.894 | 1.038 | 1.037 | 1.037 | 1.037 | 1.037 |
//! | **CG 热启动（现默认）** | 0.979 | 1.037 | 1.037 | 1.037 | 1.037 | 1.037 |
//!
//! ⇒ 三件事：**(a)** 静水柱的离散解确实存在（`p = ρg(H−y)` 是"顶面 Dirichlet、底面 Neumann"
//! 下该 Poisson 系统的精确解）；**(b)** Jacobi 单调逼近但按 ~`N²` 次爬（320 次才追到 CG 的
//! 20 次）；**(c)** CG 在 **20 次左右就满足相对残差判据并自行停手**——40/80/160/320 四档的四个
//! 读数**逐位相同**（20 档差末位：判据触发点附近解路略有不同），所以迭代上限对它不再是成本项。
//! `height_keep` 略高于 1（1.037）是自由面单元中心口径的 3.7% 抬升，不是守恒性问题。
//! ② 同判据下的**净沉降**（池内质心 y 对比自由落体 `½gT² = 4.91 m`）：80 次档 ≈ 0.059 m（1.2%）。
//! ③ **低耗散**（本族立身之本）：同场景同迭代，θ = 0.95 的末态 `Σ|v|²` ≈ 17.4 vs 纯 PIC（θ = 0）
//! 的 2.1 ⇒ 混合比确实在起作用（若桥接项误写成 PIC，这条先红）。
//! ④ 逐位确定：`flip_is_bitwise_reproducible`。
//!
//! ## 热启动（压力场跨子步复用）
//!
//! `step` 每 **tick** 建一份 `FlipScratch` 并透传（`pbf::substep_of` → `substep`）⇒ 压力场在
//! **本 tick 的各子步之间**活下来：几何键一致（维度相同、`grid_step` 相对漂移 ≤ 1e-4、`origin`
//! 漂移 ≤ 半格；判据在 `solve.rs::warm_reusable`）就拿上一子步的解当 CG 的初始猜测，否则退回
//! 冷启动。**只给 CG 档**：Jacobi 是固定次数的 A/B 对照档，预置初值会把"N 次扫"的历史口径搅浑。
//!
//! **契约**：每 tick 现建 ⇒ 结果只依赖 **tick 起点状态**，不继承上一 tick 的压力历史（逐位复现、
//! 保存/恢复、状态继承三份契约因此原样成立）。这个选择的代价是每 tick 的**首子步**仍从零收敛；
//! 那笔账要等一份"活过 tick"的缓冲（= `FluidSystem` 再加一个字段 ⇒ 先还字段数这笔债）。
//!
//! **判据口径刻意不动**：停手判据仍是绝对值残差 `CG_RTOL*max(|b|,|r0|)`。若改成相对初值
//! （`CG_RTOL*|r0|`），热启动的子步会因为初残差本来就小而**过早停手**、解出更差的场 ⇒ 收敛档
//! 读数会跟着预算变化。保持绝对值 ⇒ 收敛档**同解**，只有欠解档受益。
//!
//! 实测（同场景；对照 = 临时关掉复用的同机同参跑；`diag_pressure_iteration_convergence`
//! 的"热启动"块会打印左边三列）：
//!
//! | 压力预算 | 10 | 20 | 80 |
//! |---|---|---|---|
//! | 冷启动 `height_keep` | 0.8935 | 1.0380 | 1.0367 |
//! | **热启动 `height_keep`** | **0.9793** | 1.0365 | 1.0367 |
//! | 压力相位耗时 冷/热（µs） | 1.07e6 / 1.14e6 | 2.06e6 / 2.12e6 | 5.35e6 / 5.03e6 |
//!
//! ⇒ 热启动把**同一预算下的进度**推前（cap=10 从 0.8935 到 0.9793），收敛档读数不变、耗时中性
//! （±7%，本机噪声量级）。它**不省迭代**：省迭代要求放松判据（上面那条被刻意否掉）或跨 tick
//! 继承（见"已知简化 1"）。判据 `flip_warm_start_improves_fixed_budget_accuracy`。
//!
//! ⚠️ **本档不保证粒子级密度/无重叠**：FLIP 的不可压约束装在**网格**上，粒子之间没有排斥；
//! 粒子叠在一起时网格只当它们是"一团"，没有任何项把它们分开（PIC 的重置会让重叠**冻住**而非
//! 散开）。所以 SPH 的 `ρ/ρ0` 对本族**不是判据**（求解器根本不读 `densities()`，它只是 `density_pass`
//! 出的被动诊断），本档的探针也只把它当**读数**印出来、不做横向断言——拿 `ρ` 去比 WCSPH/PBF
//! 是在拿别人的量尺量本族。治粒子级重叠要靠**最小距离约束**（朴素版已实测排除：加进投影只会把
//! 投影搅坏）；**APIC 不治这条**——它换的是传输算子，治的是传输耗散，不是粒子间排斥。不过它在静水槽上
//! 顺带把 `over_in` 从 0.856 降到 0.332（代价是 `ke` +18%），见下"APIC"节与"已知简化 4/5"。
//!
//! ## APIC（仿射传输）——**已实现；`C` 每子步从粒子场现算（买到旋转红利）；**θ 门控**已落（`θ ≤ 0.5`），默认开（默认 θ = 0.95 落在门控关段）**
//!
//! 实现（`cfg.flip_apic` 开**且** `flip_ratio ≤ θ_max`；**默认开**，默认 `θ = 0.95` 落在关段）：P2G 散射到网格角点的是
//! `v + C·(x_角点 − x_p)`，3×3 的 `C`（**世界单位** 1/s）由 `flip/affine.rs::estimate_from_field`
//! **每子步从粒子场现算**（`(pos, vel)` 的纯函数：邻居 `|p_j − p_i| ≤ h` 上做加权最小二乘 `C = M·D⁻¹`，
//! `M = Σ w·(v_j − v_i)⊗(p_j − p_i)`、`D = Σ w·d⊗d`，权重线性衰减 `w = 1 − r/h`；邻居用
//! `src/grid.rs` 的确定性 27 邻域网格，格内序 = 索引序 ⇒ 求和序是位置的确定函数）。
//! **`C` 不落任何跨 tick 缓冲** ⇒ 不进粒子状态，逐位复现/保存恢复/状态继承三份契约原样成立。
//!
//! **θ 门控**（`flip/affine.rs::apic_allowed`，`θ_max = 0.5`）：`flip_ratio` 高 ⇒ FLIP 占比大，APIC
//! 的仿射传输会把混进来的 FLIP 噪声也仿射化 ⇒ 红利被抵消。实测 `rot_err` 自 θ = 0.8 起反超 FLIP、
//! θ = 0.95 时 APIC 在红利面/代价面两头挨打 ⇒ **θ > 0.5 一律退回纯 FLIP**（关档或 θ 超限 ⇒ `apic = false` ⇒ 逐位退回 FLIP/PIC；默认档逐位不变 ⇒ 判据 G1）。
//!
//! 两处驯服（不驯服时近静场不可用）：
//! - **岭** `λ·(tr D / 3)`（`λ = 1e-2`）：表面粒子邻域是单侧的 ⇒ `D` 在缺失方向欠定，不压住
//!   就会把那侧**偏置拟合成假梯度**（实测静水槽 `ke` 17.2 → 80.1、过密 `over` 3.6 → 18.5）；
//! - **拟合质量收缩** `C ← (⟨C, M⟩ / T)·C`：`T = Σ w·|v_j − v_i|²` 是局部速度总散布、
//!   `⟨C, M⟩` 是被线性模型解释掉的那部分（LS 性质保证 `⟨C,M⟩ ≤ T`）⇒ 近静场里"信号"几乎
//!   全是噪声时 `s → 0`、`C` 自动哑火。
//!
//! `D` 真退化（`det D ≤ (tr D / 3)³·1e-6`，**相对**判据：`D` 是世界单位、绝对值随 `h` 缩放）或非有限 ⇒ 该粒子这一子步退回 FLIP/PIC 传输。
//!
//! **实测**（`tests/flip_rotation_probe.rs::flip_apic_derived_c_buys_rotation` 与 `#[ignore]`
//! 诊断 `diag_apic_rotation`、`diag_apic_theta_scan`）。零重力旋转团台架（无边界、无体外力 ⇒ 末态 `L_y`
//! 只被 P2G/G2P 那一趟三线性传输耗散；`l_keep` = 末态/起步角动量；`θ = 0`/`θ = 0.95` 分别是 PIC/FLIP）。
//! **下表为门控落盘前读数**（门控后 θ = 0.95 的 APIC 档不可达 ⇒ 等同 FLIP）：
//!
//! | 台架读数 | PIC | **+APIC** | FLIP | **FLIP+APIC**（门控前） |
//! |---|---|---|---|---|
//! | `l_keep` | 0.1453 | **0.9814** | 0.8655 | ~~1.0729~~ ⇒ 门控后 0.8655 |
//! | `rot_err` | 0.8644 | **0.4291** | 0.4519 | ~~0.6664~~ ⇒ 门控后 0.4519 |
//! | `ke` | 4.77e0 | 1.61e2 | 1.31e2 | 2.41e2 |
//!
//! ⇒ `θ = 0` 档开 APIC 把 `l_keep` 从 0.145 抬到 **0.981**（≈ 无耗散）、`rot_err` 腰斩
//! ⇒ **派生 `C` 真的买到了 APIC 的抗旋转耗散**（SPEC §4.8 要它治的病）。`ke` 那列看着大，
//! 但它是**自洽**的：`ke ≈ L²/(2I)`（把 `C` 写死成解析真值 ω× 的对照是 `l_keep` 1.081、
//! `rot_err` 0.138）⇒ 这是"少耗散"而不是"能量爆走"。**上一版"tick 级 `C`"（从网格重建、每 tick
//! 清零）只有 0.155** ⇒ 红利来自把 `C` 的来源换成粒子场这一刀，而不是 APIC 口径本身。
//!
//! 静水槽（`θ = 0.95`、80 次 CG；`flip_incompressibility_probe.rs::diag_apic_hydrostatic`）**门控前**读数：
//! `ke` 17.24 → 20.35（+18%）、`sink` 3.21e-2 → 3.50e-2（+9%）；代价集中在**近壁**——全体过密
//! `over` 3.603 → 4.138（+15%）而**体内部** `over_in` 0.856 → 0.332（−61%）
//! ⇒ 近静场代价温和但**不为零** ⇒ 这几档由 **θ 门控**整段关掉（`θ ≥ 0.8` 时 APIC 不可达），默认
//! `flip_ratio = 0.95` 落在关段 ⇒ 默认配置逐位等于纯 FLIP/PIC。账记在 `docs/OPEN-PROBLEMS.md` **P23**。
//!
//! 〔历史〕上一版由 tick 级 `C` 的负结果推导出"`C` 每 tick 清零 ⇒ 首子步纯 PIC"；那条推导只说明
//! "从**网格**重建 `C` 不行"（粒子场的跨格差没被磨过）——改从**粒子场**现算就绕开了自洽环，且不必动粒子状态。
//!
//! ## 已知简化（切片 1，逐条登记）
//!
//! 1. **跨 tick 不复用压力场**——`step` 每 tick 现建一份 `FlipScratch`（**tick 内**各子步已热
//!    启动，见"热启动"节；顺带把 scratch 分配从每子步一次降到每 tick 一次）。要让 `p` 活过
//!    tick 得给 `FluidSystem` 再加一个字段，而字段数（26 > 24）本身是登记债务 ⇒ 先还再借。
//!    代价：每 tick 的**首子步**仍从零收敛。**下一刀首选**。
//! 2. **压力求解**：默认已是 **CG**（对角预处理，`flip/solve.rs`）⇒ 静水柱被**真解出来**
//!    （验证①：20 次即 1.037 并停手），固定次数 Jacobi 只作为 A/B 档保留
//!    （`flip_pressure_cg = false`；它按 ~`N²` 次爬，是"预算限制解"的历史读数）。
//!    **收敛率这一半已结**，热启动这半已在 **tick 内**落地（见"热启动"节）；剩下的只是更细的
//!    预条件/多重网格（当前 20 次到顶 ⇒ 优先级低）与跨 tick 那半（= 简化 1）。
//! 3. **刚体反作用**：本档沿用 `boundary_pass` 的 2b 边界粒子账（`bforce`/`breact`），
//!    **没有**把网格的固体面压力积分成对刚体的反作用 ⇒ 网格的固体 BC 是"看不见的静止墙"。
//!    流体↔刚体的双向力请走 2b 边界粒子档（`set_boundary_particles`），登记为下一刀。
//! 4. 单元边长固定取粒子间距，不做自适应；**APIC 仿射传输已实现、派生 `C` 已买到旋转红利
//!    （`l_keep` 0.145 → 0.981），但近静场要付 +18% `ke` / +15% 近壁过密 `over`（见上"APIC"
//!    节 + `docs/OPEN-PROBLEMS.md` P23）⇒ 这几档由 θ 门控关掉（默认 `θ = 0.95`），本档默认是 FLIP/PIC 混合。
//! 5. **粒子级重叠无约束**（见上"⚠️"）⇒ `densities()` 对 FLIP 是被动读数。凡是需要密度场
//!    自洽的相位（`fluid_force.rs` 的负压钳零、表面张力、`density_pass` 的镜像鬼影边界）在
//!    本族下都只是"顺带算出来的"，**不被求解器消费**——别在本族上调它们的参数期待效果。
use crate::{system::ext::FluidPipelineExt, FluidSystem, ProviderColliders, Vec3};

// 网格层（建网格/分类/索引/三线性模板）拆在同名子模块里：`flip.rs` 一度 837 行、超 `god_gate`
// 的 800 行硬线；拆开后两层各自远在限内，上面那整段口径说明也不必为了行数砍掉。
mod affine;
mod grid;
mod solve;

use self::affine::{affine_apply, apic_allowed, estimate_from_field};
use self::grid::{cell_axis, weight_of};

/// 网格相对流体粒子 AABB 的外扩（单元数）：留出空单元作自由表面 Dirichlet 层，
/// 同时保证 P2G 的 2×2×2 三线性模板不会越出面数组边界。
const MARGIN_CELLS: f32 = 2.0;
/// 固体分类探针半径 = `grid_step × 本系数`。判据 `depth > probe ⇔ sdf < 0`（内点）；
/// 取 > 1 是为了让"紧贴流体的那层固体"也能判到（截断 SDF 只报近表面邻域）。
const SOLID_PROBE_FRAC: f32 = 1.5;
/// 单元数上限：超限则整体放大单元边长（病态/巨大 AABB 不吃穿内存）。
const MAX_CELLS: usize = 1 << 21;

/// 单元分类。
const CLASS_AIR: u8 = 0;
const CLASS_FLUID: u8 = 1;
const CLASS_SOLID: u8 = 2;

/// FLIP 相位缓冲（每子步一份；见模块头"已知简化 1"）。
///
/// 索引约定：单元 `c = (i*jd + j)*kd + k`（`jd/ky` = dims[1]/dims[2]）；三套**交错面**：
/// `u` 在 x 面（`(nx+1)×ny×nz`）、`v` 在 y 面、`w` 在 z 面；单元 `(i,j,k)` 占
/// `[origin + i·grid_step, origin + (i+1)·grid_step)`，面 `i` 在单元坐标 `i` 处。
pub(crate) struct FlipScratch {
    origin: Vec3,
    grid_step: f32,
    dims: [usize; 3],
    n_cells: usize,
    /// 0 = 空、1 = 流、2 = 固。
    class: Vec<u8>,
    /// 逐单元粒子权重和（流/空判据 + 归一）。
    weight: Vec<f32>,
    u: Vec<f32>,
    v: Vec<f32>,
    w: Vec<f32>,
    wu: Vec<f32>,
    wv: Vec<f32>,
    ww: Vec<f32>,
    /// 投影**前**的面速度（FLIP 增量的减项）。
    u0: Vec<f32>,
    v0: Vec<f32>,
    w0: Vec<f32>,
    p: Vec<f32>,
    /// Jacobi 的写新缓冲 / CG 的 `z = M^-1 r`（两个求解器互斥 => 复用一个）。
    p_tmp: Vec<f32>,
    div: Vec<f32>,
    /// CG 工作向量（解 `x` 就是 `p`）：残差 / 方向 / `A*d`。
    cg_r: Vec<f32>,
    cg_d: Vec<f32>,
    cg_q: Vec<f32>,
    /// 上一**子步**解出的压力场：本子步若几何键一致就拿来当 CG 的初始猜测（热启动）。
    p_prev: Vec<f32>,
    /// `p_prev` 的几何键 `(origin, grid_step, dims)`；`None` = 这份缓冲还没解过（本 tick 首子步）。
    warm_key: Option<(Vec3, f32, [usize; 3])>,
    /// 逐粒子 **APIC 仿射矩阵**（行主序 3×3，**世界单位** 1/s）：每子步 P2G 前由
    /// `affine::estimate_from_field` 从**粒子场**现算（`(pos, vel)` 的纯函数 ⇒ 不进粒子状态）。
    /// `cfg.flip_apic` 关时留空。
    affine: Vec<[f32; 9]>,
    /// 估梯度用的**确定性邻域网格**（`src/grid.rs` 的计数排序版，与 SPH 共用 `grid_box` 规则）。
    ngrid: crate::grid::UniformGrid,
}

impl FlipScratch {
    /// P2G：逐流体粒子把速度三线性散射到三套面 + 面权重。
    ///
    /// `cfg.flip_apic` 开**且** `flip_ratio ≤ θ_max`（门控见 `flip/affine.rs::apic_allowed`）时
    /// 先按**粒子场**现算 `C` 再散射**角点仿射速度**；关档（含 θ 超限）逐字退回纯 FLIP/PIC。
    fn p2g(&mut self, sys: &FluidSystem) {
        let apic = apic_allowed(&sys.cfg);
        if apic {
            estimate_from_field(sys, &mut self.ngrid, &mut self.affine);
        }
        let (nx, ny, nz) = (self.dims[0], self.dims[1], self.dims[2]);
        for i in 0..sys.n_fluid {
            let p = sys.dom.ps.pos[i];
            let v = sys.dom.ps.vel[i];
            let c = if apic { self.affine[i] } else { [0.0f32; 9] };
            let g = self.coord(p);
            let (i0, ux) = cell_axis(g.x);
            let (j0, uy) = cell_axis(g.y);
            let (k0, uz) = cell_axis(g.z);
            for di in 0..2usize {
                for dj in 0..2usize {
                    for dk in 0..2usize {
                        let w = weight_of(di, ux) * weight_of(dj, uy) * weight_of(dk, uz);
                        if w <= 0.0 {
                            continue;
                        }
                        let (ci, cj, ck) = (i0 + di, j0 + dj, k0 + dk);
                        // 角点偏移（**单元边长**为单位）：`x_角点 − x_p = grid_step·(di − u, …)`。
                        let off = Vec3::new(di as f32 - ux, dj as f32 - uy, dk as f32 - uz);
                        let va = if apic {
                            v + affine_apply(&c, off) * self.grid_step
                        } else {
                            v
                        };
                        // u（x 面）：沿 x 用面索引，沿 y/z 用单元索引。
                        if ci <= nx && cj < ny && ck < nz {
                            let idx = self.u_idx(ci, cj, ck);
                            self.u[idx] += va.x * w;
                            self.wu[idx] += w;
                        }
                        // v（y 面）。
                        if ci < nx && cj <= ny && ck < nz {
                            let idx = self.v_idx(ci, cj, ck);
                            self.v[idx] += va.y * w;
                            self.wv[idx] += w;
                        }
                        // w（z 面）。
                        if ci < nx && cj < ny && ck <= nz {
                            let idx = self.w_idx(ci, cj, ck);
                            self.w[idx] += va.z * w;
                            self.ww[idx] += w;
                        }
                    }
                }
            }
        }
    }

    /// 面归一 + 未知面外推 + 无通量面置 0 + 备份 `u0/v0/w0`（FLIP 增量的减项）。
    fn normalize_and_bc(&mut self) {
        normalize_faces(&mut self.u, &mut self.wu, self.dims, 0);
        normalize_faces(&mut self.v, &mut self.wv, self.dims, 1);
        normalize_faces(&mut self.w, &mut self.ww, self.dims, 2);
        self.zero_no_flux();
        self.u0.clear();
        self.u0.extend_from_slice(&self.u);
        self.v0.clear();
        self.v0.extend_from_slice(&self.v);
        self.w0.clear();
        self.w0.extend_from_slice(&self.w);
    }

    /// 无通量面：固体邻居（任一侧）与网格外边界 ⇒ 面速度置 0。
    fn zero_no_flux(&mut self) {
        let (nx, ny, nz) = (self.dims[0], self.dims[1], self.dims[2]);
        for i in 0..=nx {
            for j in 0..ny {
                for k in 0..nz {
                    let idx = self.u_idx(i, j, k);
                    let l = if i >= 1 {
                        self.class[self.cell_idx(i - 1, j, k)]
                    } else {
                        CLASS_SOLID
                    };
                    let r = if i < nx {
                        self.class[self.cell_idx(i, j, k)]
                    } else {
                        CLASS_SOLID
                    };
                    if l == CLASS_SOLID || r == CLASS_SOLID {
                        self.u[idx] = 0.0;
                    }
                }
            }
        }
        for i in 0..nx {
            for j in 0..=ny {
                for k in 0..nz {
                    let idx = self.v_idx(i, j, k);
                    let l = if j >= 1 {
                        self.class[self.cell_idx(i, j - 1, k)]
                    } else {
                        CLASS_SOLID
                    };
                    let r = if j < ny {
                        self.class[self.cell_idx(i, j, k)]
                    } else {
                        CLASS_SOLID
                    };
                    if l == CLASS_SOLID || r == CLASS_SOLID {
                        self.v[idx] = 0.0;
                    }
                }
            }
        }
        for i in 0..nx {
            for j in 0..ny {
                for k in 0..=nz {
                    let idx = self.w_idx(i, j, k);
                    let l = if k >= 1 {
                        self.class[self.cell_idx(i, j, k - 1)]
                    } else {
                        CLASS_SOLID
                    };
                    let r = if k < nz {
                        self.class[self.cell_idx(i, j, k)]
                    } else {
                        CLASS_SOLID
                    };
                    if l == CLASS_SOLID || r == CLASS_SOLID {
                        self.w[idx] = 0.0;
                    }
                }
            }
        }
    }

    /// 面速度投影：`u ← u − (Δt/ρ)(p_r − p_l)/grid_step`（只施加在"至少一侧是流体"的非固体面）。
    fn project_faces(&mut self, sys: &FluidSystem, dt: f32) {
        let inv = dt / sys.cfg.rest_density / self.grid_step;
        let (nx, ny, nz) = (self.dims[0], self.dims[1], self.dims[2]);
        for i in 1..nx {
            for j in 0..ny {
                for k in 0..nz {
                    let (cl, cr) = (
                        self.class[self.cell_idx(i - 1, j, k)],
                        self.class[self.cell_idx(i, j, k)],
                    );
                    if cl == CLASS_SOLID || cr == CLASS_SOLID {
                        continue;
                    }
                    if cl != CLASS_FLUID && cr != CLASS_FLUID {
                        continue;
                    }
                    let pl = if cl == CLASS_FLUID {
                        self.p[self.cell_idx(i - 1, j, k)]
                    } else {
                        0.0
                    };
                    let pr = if cr == CLASS_FLUID {
                        self.p[self.cell_idx(i, j, k)]
                    } else {
                        0.0
                    };
                    let idx = self.u_idx(i, j, k);
                    self.u[idx] -= inv * (pr - pl);
                }
            }
        }
        for i in 0..nx {
            for j in 1..ny {
                for k in 0..nz {
                    let (cl, cr) = (
                        self.class[self.cell_idx(i, j - 1, k)],
                        self.class[self.cell_idx(i, j, k)],
                    );
                    if cl == CLASS_SOLID || cr == CLASS_SOLID {
                        continue;
                    }
                    if cl != CLASS_FLUID && cr != CLASS_FLUID {
                        continue;
                    }
                    let pl = if cl == CLASS_FLUID {
                        self.p[self.cell_idx(i, j - 1, k)]
                    } else {
                        0.0
                    };
                    let pr = if cr == CLASS_FLUID {
                        self.p[self.cell_idx(i, j, k)]
                    } else {
                        0.0
                    };
                    let idx = self.v_idx(i, j, k);
                    self.v[idx] -= inv * (pr - pl);
                }
            }
        }
        for i in 0..nx {
            for j in 0..ny {
                for k in 1..nz {
                    let (cl, cr) = (
                        self.class[self.cell_idx(i, j, k - 1)],
                        self.class[self.cell_idx(i, j, k)],
                    );
                    if cl == CLASS_SOLID || cr == CLASS_SOLID {
                        continue;
                    }
                    if cl != CLASS_FLUID && cr != CLASS_FLUID {
                        continue;
                    }
                    let pl = if cl == CLASS_FLUID {
                        self.p[self.cell_idx(i, j, k - 1)]
                    } else {
                        0.0
                    };
                    let pr = if cr == CLASS_FLUID {
                        self.p[self.cell_idx(i, j, k)]
                    } else {
                        0.0
                    };
                    let idx = self.w_idx(i, j, k);
                    self.w[idx] -= inv * (pr - pl);
                }
            }
        }
    }

    /// G2P：三线性采样（投影前/后）→ FLIP 混合 → CFL → 推进位置；回写 `press`。
    ///
    /// 不碰 APIC 的 `C`：那由本档 `p2g` 每子步从粒子场现算（模块头"APIC"节）。
    fn g2p(&mut self, sys: &mut FluidSystem, dt: f32) {
        let theta = sys.cfg.flip_ratio.clamp(0.0, 1.0);
        let vmax = sys.cfg.max_speed_frac * sys.dom.kern.h / dt;
        let vmax2 = vmax * vmax;
        let (nx, ny, nz) = (self.dims[0], self.dims[1], self.dims[2]);
        for i in 0..sys.n_fluid {
            let p = sys.dom.ps.pos[i];
            let g = self.coord(p);
            let (i0, ux) = cell_axis(g.x);
            let (j0, uy) = cell_axis(g.y);
            let (k0, uz) = cell_axis(g.z);
            let mut acc = [0.0f32; 6];
            for di in 0..2usize {
                for dj in 0..2usize {
                    for dk in 0..2usize {
                        let w = weight_of(di, ux) * weight_of(dj, uy) * weight_of(dk, uz);
                        if w <= 0.0 {
                            continue;
                        }
                        let (ci, cj, ck) = (i0 + di, j0 + dj, k0 + dk);
                        if ci <= nx && cj < ny && ck < nz {
                            let s = self.u_idx(ci, cj, ck);
                            acc[0] += self.u[s] * w;
                            acc[3] += self.u0[s] * w;
                        }
                        if ci < nx && cj <= ny && ck < nz {
                            let s = self.v_idx(ci, cj, ck);
                            acc[1] += self.v[s] * w;
                            acc[4] += self.v0[s] * w;
                        }
                        if ci < nx && cj < ny && ck <= nz {
                            let s = self.w_idx(ci, cj, ck);
                            acc[2] += self.w[s] * w;
                            acc[5] += self.w0[s] * w;
                        }
                    }
                }
            }
            let vg = Vec3::new(acc[0], acc[1], acc[2]);
            let vg0 = Vec3::new(acc[3], acc[4], acc[5]);
            let v_flip = sys.dom.ps.vel[i] + (vg - vg0);
            let mut v = v_flip * theta + vg * (1.0 - theta);
            let s2 = v.length_squared();
            if s2 > vmax2 {
                v *= vmax / s2.sqrt();
            }
            sys.dom.ps.vel[i] = v;
            sys.dom.ps.pos[i] += v * dt;
            // `pressures()` = 粒子所在单元压力（Pa）；夹到网格内。
            let ci = i0.min(nx - 1);
            let cj = j0.min(ny - 1);
            let ck = k0.min(nz - 1);
            sys.dom.ps.press[i] = self.p[self.cell_idx(ci, cj, ck)];
        }
    }
}
/// 一个 FLIP 子步（`pbf::substep_of` 的第三族入口）：见模块头八步。
///
/// `sc` 由 `step` 每 **tick** 建一份并传入（不是这里新建）——这样压力场能跨**本 tick 的各
/// 子步**热启动；每个 tick 仍从空缓冲起（结果只依赖 tick 起点状态，见模块头"热启动"）。
pub(crate) fn substep(
    sys: &mut FluidSystem,
    dt: f32,
    providers: &dyn ProviderColliders,
    sc: &mut FlipScratch,
) {
    let nf = sys.n_fluid;
    if nf == 0 {
        return; // 边界粒子不单独驱动（同 WCSPH/PBF 档）
    }
    // 每子步重累加反作用（清零归本函数、÷ 子步数归 `step`，C2 口径与 SPH 族一致）。
    for f in sys.dom.ps.bforce[nf..].iter_mut() {
        *f = Vec3::ZERO;
    }
    // 相位 0/1：邻域网格 + SPH 密度（层流黏度相位与 `densities()` 口径；与 PBF 档同）。
    let t = vxl_phys_core::probe::start();
    sys.dom.grid.rebuild(&sys.dom.ps.pos, sys.dom.kern.h);
    sys.phase_us[0] += vxl_phys_core::probe::us(t);
    let t = vxl_phys_core::probe::start();
    sys.density_pass(providers);
    sys.phase_us[1] += vxl_phys_core::probe::us(t);
    // ② 体外力 + CFL（与 PBF 档共用；**只改速度**——位置要等投影后的速度来推）。
    crate::pbf::apply_forces(sys, dt);
    // ③④ 建网格 + 分类。
    sc.build(sys, providers);
    // ⑤ P2G + 归一/外推/无通量 + 备份。
    let t = vxl_phys_core::probe::start();
    sc.p2g(sys);
    sc.normalize_and_bc();
    sys.phase_us[3] += vxl_phys_core::probe::us(t);
    // ⑥⑦ 散度 + 压力（默认 CG，`cfg.flip_pressure_cg`）+ 面投影。
    let t = vxl_phys_core::probe::start();
    sc.solve_pressure(sys, dt);
    sc.project_faces(sys, dt);
    sys.phase_us[2] += vxl_phys_core::probe::us(t);
    // ⑧ G2P 混合 + CFL + 推进位置，再走统一提供者通道的边界投影。
    let t = vxl_phys_core::probe::start();
    sc.g2p(sys, dt);
    sys.boundary_pass(providers);
    sys.phase_us[4] += vxl_phys_core::probe::us(t);
}

/// 面归一：`w > tiny` 的除权重；未知面（`w ≤ tiny`）先清零，再沿轴两向各扫一趟外推
/// （未知面取上一已知面的值并把权重置为 1 ⇒ 单趟即可传播整段）。`axis` = 面法向轴。
fn normalize_faces(f: &mut [f32], w: &mut [f32], dims: [usize; 3], axis: usize) {
    let tiny = 1e-12f32;
    let n = f.len();
    for i in 0..n {
        if w[i] > tiny {
            f[i] /= w[i];
            w[i] = 1.0;
        } else {
            f[i] = 0.0;
            w[i] = 0.0;
        }
    }
    // 沿 `axis` 的正/负两向各一趟：把已知值填进未知面（自由表面外侧的惰性面）。
    let (nx, ny, nz) = (dims[0], dims[1], dims[2]);
    if axis == 0 {
        for j in 0..ny {
            for k in 0..nz {
                let mut carry = f32::NAN;
                for i in 0..=nx {
                    let idx = (i * ny + j) * nz + k;
                    if w[idx] > 0.0 {
                        carry = f[idx];
                    } else if carry.is_finite() {
                        f[idx] = carry;
                        w[idx] = 1.0;
                    }
                }
                carry = f32::NAN;
                for i in (0..=nx).rev() {
                    let idx = (i * ny + j) * nz + k;
                    if w[idx] > 0.0 {
                        carry = f[idx];
                    } else if carry.is_finite() {
                        f[idx] = carry;
                        w[idx] = 1.0;
                    }
                }
            }
        }
    } else if axis == 1 {
        for i in 0..nx {
            for k in 0..nz {
                let mut carry = f32::NAN;
                for j in 0..=ny {
                    let idx = (i * (ny + 1) + j) * nz + k;
                    if w[idx] > 0.0 {
                        carry = f[idx];
                    } else if carry.is_finite() {
                        f[idx] = carry;
                        w[idx] = 1.0;
                    }
                }
                carry = f32::NAN;
                for j in (0..=ny).rev() {
                    let idx = (i * (ny + 1) + j) * nz + k;
                    if w[idx] > 0.0 {
                        carry = f[idx];
                    } else if carry.is_finite() {
                        f[idx] = carry;
                        w[idx] = 1.0;
                    }
                }
            }
        }
    } else {
        for i in 0..nx {
            for j in 0..ny {
                let mut carry = f32::NAN;
                for k in 0..=nz {
                    let idx = (i * ny + j) * (nz + 1) + k;
                    if w[idx] > 0.0 {
                        carry = f[idx];
                    } else if carry.is_finite() {
                        f[idx] = carry;
                        w[idx] = 1.0;
                    }
                }
                carry = f32::NAN;
                for k in (0..=nz).rev() {
                    let idx = (i * ny + j) * (nz + 1) + k;
                    if w[idx] > 0.0 {
                        carry = f[idx];
                    } else if carry.is_finite() {
                        f[idx] = carry;
                        w[idx] = 1.0;
                    }
                }
            }
        }
    }
}
