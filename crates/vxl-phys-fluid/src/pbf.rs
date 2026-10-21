//! pbf：§4.8 的 **PBF 档**（`FluidFamily::CpuPbf`）—— Müller & Macklin 2013
//! *Position Based Fluids* 的**密度约束投影**。
//!
//! ## 与 WCSPH 档的分工（同一积分器上的另一种离散，不是第二套世界）
//!
//! | | CPU SPH（`CpuSph`） | PBF（`CpuPbf`） |
//! |---|---|---|
//! | 密度误差 → | **压力**（Tait EOS，p = B((ρ/ρ0)^γ−1)） | **位置投影**（约束 C_i = ρ_i/ρ0 − 1） |
//! | 不可压误差 | 由声速 c 压低（ρ0gH/c² 量级） | 由**迭代次数**压低（`density_iterations`） |
//! | 张力抑制 | spiky 中心不减益 + p ≥ 0 钳 | **一边倒约束 `λ ≤ 0`**（只推不拉；`s_corr` 默认关） |
//! | 数值黏度 | Monaghan Π（`artificial_viscosity`） | XSPH（`xsph_viscosity`，Müller 口径） |
//!
//! 因此 **PBF 不消费**  `sound_speed` / `gamma_tait` / `artificial_viscosity`
//! （Tait 状态方程与压缩波阻尼是 WCSPH 的构件，PBF 里没有压力这个量）。
//!
//! ## 一字不改地复用 WCSPH 档的既有积木
//!
//! 核常数与质量标定（`assemble`）、邻域表（`UniformGrid` 固定遍历序）、
//! 密度轮 `density_pass`（poly6 含自身项 + 镜像鬼影补壁 + 2b 边界粒子 `sum_b`）、
//! 物理层流黏度入口 `laminar::pass`（μ = 0 自短路）、非弹性边界投影 `boundary_pass`、
//! 反作用聚合 `aggregate_reactions`（tick 平均口径 C2 在 `step` 里，未复制）。
//!
//! ## 约束（每子步；Müller 2013 §4）
//!
//! ```text
//! C_i    = ρ_i/ρ0 − 1
//! λ_i    = −C_i / ( Σ_k |∇_{p_k} C_i|² + ε )    分母 = (1/ρ0²)(|Σ_j ∇_{p_i}W_ij|² + Σ_j |∇_{p_i}W_ij|²)
//! λ_i    ← min(λ_i, 0)                          一边倒：C_i < 0（自由表面）时禁拉力
//! Δp_i   = (1/ρ0) Σ_j (λ_i + λ_j + s_corr) ∇_{p_i} W(r_ij)
//! s_corr = −k · ( W(r_ij)/W(Δq) )^n · (|λ_i| + |λ_j|)   （仅张力对；本档默认 k = 0 ⇒ 关）
//! ```
//!
//! `∇_{p_i}W_ij = −ks(h−r)²·d̂`（`d̂ = (p_i−p_j)/r`）——与 WCSPH 力相位**同核同系数**
//! （spiky），这就是 §4.8 的"tensile instability 用 spiky 梯度抑制"。
//!
//! ## `s_corr`（**默认关**：`S_CORR_K = 0.0`，代码保留论文原式）
//!
//! 论文 Eq.12 的 `s_corr` 是**绝对量级**的短程排斥（本档代码即此式，系数 `k = S_CORR_K`）：
//! `s_corr = −k·(W(r_ij)/W(Δq))^4`，与 `λ` 直接相加。**默认关掉**，两条理由：
//!  1. **绝对量级** ⇒ 量级与局部密度误差**无关** ⇒ 在没有密度误差的地方它也是纯人工力，
//!     标不出通用 `k`（本仓单位下 `k` 稍大就把均匀晶格撑出人工空腔）；
//!  2. 论文的**相对**形式（`×(|λ_i| + |λ_j|)`）在**最需要它**的自由表面失效——表面
//!     `λ ≈ 1e-5` ⇒ `s_corr ≈ 0`。
//!
//! 两条路都标不出来 ⇒ 留 `k = 0`（逐位等价于不执行）。
//!
//! 而它要治的病（自由表面核被截断 ⇒ `C < 0` ⇒ `λ > 0` ⇒ 成对项净**吸引** ⇒ 水块自缩）
//! 现在**在源头治了**：`lambda_pass` 的**一边倒约束 `λ ≤ 0`**（同一个
//! `tensile_instability_suppression` 开关，与 CPU SPH 档的"负压钳零"同口径）让张力对不
//! 复存在 ⇒ 与 `s_corr` 叠加属重复抑制。保留该分支只为让论文 Algorithm 1 的结构在代码里
//! 可辨认、可 A/B；扫描读数见 `project` 内的 `s_corr` 注释（⚠️ 该标定出自一次性 scratch
//! 扫描，**仓内没有可复跑的标定探针**——这正是把 `k` 留在 0 的理由之一）。
//!
//! ## 2b 边界粒子（与 WCSPH 同数组、索引 ≥ `n_fluid`）
//!
//! 边界粒子**参与**密度（`density_pass`，Akinci 只吃流体邻居）与 `λ`（其 λ_j 出现在
//! 流体的 Δp_i 里）；但它的 Δp 不被施加（运动学冻结），而是折成**反作用力**
//! `bforce[j] −= m_i · Δp_ij / dt²`。逐对取**等大反向**（用流体粒子的 `m_i`，
//! 同 `fluid_force.rs` 的成对口径）⇒ `Σ_i m_i·Δp_i/dt²` 与 `Σ_j bforce[j]` 严格抵消
//! ⇒ 动量守恒由构造保证（判据 `tests/pbf_boundary_reaction.rs`）。
//!
//! ## 确定性（§5 契约）
//!
//! 邻域遍历序 = 网格固定序；`density_iterations` 固定；无原子、无浮点归约重排；
//! λ/Δp/速度回填都是"先全量累加、再统一写"（Jacobi 式）⇒ 与邻居遍历序无关，
//! 同输入两跑**逐位一致**（判据 `tests/pbf_incompressibility_probe.rs`）。
//! **默认档（`CpuSph`）只经过本模块的 `substep_of` 分派**（`fluid_step.rs` 的入口）——
//! 不执行任何 PBF 计算、不触碰 PBF 常量与 scratch ⇒ WCSPH / 金样逐位未变。
//!
//! ## 已知简化（切片 1，逐条登记）
//!
//! 1. **每子步分配两份 scratch**（Δp + λ）——`FluidSystem` 的字段数是 god 门登记
//!    债务（26 > 24），不再加字段；PBF 是 opt-in 档，先要正确性与可测性。
//! 2. **位置投影内每迭代重建一次网格/密度**（论文 Algorithm 1 即此）⇒ 每子步共
//!    `iters + 1` 次密度评估（第一个迭代也必须在 x* 上评，否则 `C` 与位置不同步）；
//!    本档正确性优先，多这一次密度评估登记为已知代价。
//! 3. `pressures()` 在 PBF 档恒 0（本档没有压力这个量；密度误差看 `densities()`）。
use crate::{DensityIterations, FluidConfig, FluidFamily, FluidSystem, ProviderColliders, Vec3};

/// 人工压力的无量纲强度（论文 k = 0.1；量纲口径见模块头）。
const S_CORR_K: f32 = 0.0;
/// 参考距离 Δq = 0.2h（论文默认）。
const S_CORR_DQ_FRAC: f32 = 0.2;
/// λ 分母的软化项 ε（孤立粒子防除零；本仓单位下实测量级 ~1e4 ⇒ 不改变有效值）。
const LAMBDA_EPS: f32 = 1e-6;
/// λ 的上下限（防"近零分母 + 极端欠密度"把 λ 顶成巨值；实测量级 1e-5，仅作熔断）。
const LAMBDA_CLAMP: f32 = 1.0;

/// PBF 相位缓冲（每**子步**一份；见模块头"已知简化 1"）。
pub(crate) struct PbfScratch {
    /// 逐粒累计位移 Σ_j Δp_ij（流体段；**整个子步**累计，`finish` 用它回填速度）。
    delta: Vec<Vec3>,
    /// **单次迭代**的位移 Δp_i（流体段）：投影必须"先把全部算齐、再一起挪"（见 [`project`]）。
    iter: Vec<Vec3>,
    /// 逐粒 λ（**覆盖全部粒子**：边界粒子的 λ_j 参与流体的 Δp_i）。
    lambda: Vec<f32>,
}

/// **相位分派**（`step` 的唯一子步入口）：`CpuPbf` ⇒ PBF 子步、`CpuFlip` ⇒ FLIP 子步；
/// 其余 ⇒ WCSPH 子步。
///
/// GPU 族在 `assemble` 就被显式拒绝（不静默跑 CPU）⇒ 这里只需分辨三族。
///
/// `flip_sc` 由 `step` 每 **tick** 建一份并透传（只有 FLIP 族用；PBF/WCSPH 子步自带缓冲），
/// 用来让压力场跨本 tick 的各子步热启动（见 `flip.rs` 模块头"热启动"）。
pub(crate) fn substep_of(
    sys: &mut FluidSystem,
    dt: f32,
    providers: &dyn ProviderColliders,
    flip_sc: &mut crate::flip::FlipScratch,
) {
    match sys.cfg.family {
        FluidFamily::CpuPbf => substep(sys, dt, providers),
        FluidFamily::CpuFlip => crate::flip::substep(sys, dt, providers, flip_sc),
        _ => sys.substep(dt, providers),
    }
}

/// 一个 PBF 子步：预测 → 密度约束迭代 → 速度回填 + XSPH → 边界投影 + 反作用聚合。
///
/// 相位计时口径（`phase_us`）：`[0]` 网格、`[1]` 密度、`[2]` 恒 0（本档无 Tait 压力）、
/// `[3]` 约束投影 + 速度回填、`[4]` 积分 + 边界投影。
pub(crate) fn substep(sys: &mut FluidSystem, dt: f32, providers: &dyn ProviderColliders) {
    let nf = sys.n_fluid;
    if nf == 0 {
        return; // 边界粒子不单独驱动（同 WCSPH 档）
    }
    let mut sc = PbfScratch {
        delta: vec![Vec3::ZERO; nf],
        iter: vec![Vec3::ZERO; nf],
        lambda: vec![0.0; sys.dom.ps.pos.len()],
    };
    // 每子步重累加反作用（清零归本函数、÷ 子步数归 `step`，C2 口径与 WCSPH 档一致）。
    for f in sys.dom.ps.bforce[nf..].iter_mut() {
        *f = Vec3::ZERO;
    }
    let t = vxl_phys_core::probe::start();
    // 预测**前**先在 x_t 建一次网格 + 评一次密度：`predict` 的层流黏度相位吃 `dens`
    // （`laminar.rs` 用 `(ρ_i+ρ_j)/2`，μ = 0 时短路），这是本档唯一的 x_t 快照。
    sys.dom.grid.rebuild(&sys.dom.ps.pos, sys.dom.kern.h);
    sys.phase_us[0] += vxl_phys_core::probe::us(t);
    let t = vxl_phys_core::probe::start();
    sys.density_pass(providers);
    sys.phase_us[1] += vxl_phys_core::probe::us(t);
    predict(sys, dt);
    let iters = iterations(&sys.cfg);
    for _ in 0..iters {
        // **每个**迭代（含第一个）都在**当前**位置重建邻域、重评密度：第一个迭代的位置即
        // 预测位 x*，论文 Algorithm 1 就是"找邻居 → 算 λ → 算 Δp → 挪"。若拿 x_t 的密度/
        // 网格去投影 x*，第一个迭代的 `C` 与位置就不同步（单子步位移可达 `0.4h`，邻域边界
        // 上会漏/多算邻居）。每迭代重评也保确定性（网格构建与遍历序无关）
        // ⚠️ 代价：每子步比"只建一次"多 1 次密度评估（本档正确性优先，见文件头简化表）。
        let t = vxl_phys_core::probe::start();
        sys.dom.grid.rebuild(&sys.dom.ps.pos, sys.dom.kern.h);
        sys.phase_us[0] += vxl_phys_core::probe::us(t);
        let t = vxl_phys_core::probe::start();
        sys.density_pass(providers);
        sys.phase_us[1] += vxl_phys_core::probe::us(t);
        let t = vxl_phys_core::probe::start();
        lambda_pass(sys, &mut sc);
        project(sys, &mut sc, dt);
        sys.phase_us[3] += vxl_phys_core::probe::us(t);
    }
    finish(sys, dt, providers, &sc);
}

/// 密度约束迭代档 → 次数（3 或 4；§4.8）。
fn iterations(cfg: &FluidConfig) -> u32 {
    match cfg.density_iterations {
        DensityIterations::Three => 3,
        DensityIterations::Four => 4,
    }
}

/// 体外力相位（PBF 与 FLIP 两档共用）：`v ← v + dt·(g + 层流黏度)` + CFL 限速；**不改位置**
/// （位置何时推进由各档决定：PBF 就在这一相位，FLIP 要等网格投影后的速度，否则会系统性下沉）。
///
/// 物理黏度入口与 WCSPH 档**同一个**（`laminar::pass`，μ = 0 自短路）。
pub(crate) fn apply_forces(sys: &mut FluidSystem, dt: f32) {
    for i in 0..sys.dom.ps.pos.len() {
        sys.dom.ps.acc[i] = sys.cfg.gravity;
    }
    crate::laminar::pass(sys);
    let vmax = sys.cfg.max_speed_frac * sys.dom.kern.h / dt;
    let vmax2 = vmax * vmax;
    let nf = sys.n_fluid;
    for i in 0..nf {
        let mut v = sys.dom.ps.vel[i] + sys.dom.ps.acc[i] * dt;
        let s2 = v.length_squared();
        if s2 > vmax2 {
            v *= vmax / s2.sqrt();
        }
        sys.dom.ps.vel[i] = v;
    }
}

/// 预测：`apply_forces` + `x* ← x + dt·v`（**只流体段**；边界冻结）。
fn predict(sys: &mut FluidSystem, dt: f32) {
    apply_forces(sys, dt);
    for i in 0..sys.n_fluid {
        sys.dom.ps.pos[i] += sys.dom.ps.vel[i] * dt;
    }
}

/// `λ_i = −C_i / (Σ_k |∇_{p_k}C_i|² + ε)`（**全部粒子**：边界粒子的 λ_j 要被流体读到）。
///
/// 分子 = `|Σ_j ∇_{p_i}W_ij|²`（k = i 那项）+ `Σ_j |∇_{p_i}W_ij|²`（k = j 那些项）。
fn lambda_pass(sys: &mut FluidSystem, sc: &mut PbfScratch) {
    let (h, ks, nt) = (sys.dom.kern.h, sys.dom.kern.ks, sys.dom.ps.pos.len());
    let inv_rho = 1.0 / sys.cfg.rest_density;
    sc.lambda.resize(nt, 0.0);
    for i in 0..nt {
        let mut g = Vec3::ZERO; // Σ_j ∇_{p_i}W_ij
        let mut sq = 0.0f32; // Σ_j |∇_{p_i}W_ij|²
        sys.for_neighbors(i, |_j, d, r2| {
            let r = r2.sqrt();
            let t = h - r;
            let kg = ks * t * t; // |∇W|（spiky）
            g += d * (kg / r.max(1e-9));
            sq += kg * kg;
        });
        let denom = (g.length_squared() + sq) * (inv_rho * inv_rho) + LAMBDA_EPS;
        let c = sys.dom.ps.dens[i] * inv_rho - 1.0;
        let lam = (-c / denom).clamp(-LAMBDA_CLAMP, LAMBDA_CLAMP);
        // **一边倒约束**（自由表面口径）：`C < 0` ⇒ `λ > 0` ⇒ 成对项是**吸引**，而"把邻域
        // 移回被截断的核内"几何上做不到 ⇒ 表面会变成**永不满足的常量拉力**（自己缩 →
        // 过密 → 再被顶回 ⇒ 湍流；实测水槽 `vmax` 4.8~7.5、`over_in` 11%，且迭代越多越差）。
        // WCSPH 档对应的是"负压钳零"（自由表面判据，`pressure_pass`）；这里由同一个
        // `tensile_instability_suppression` 开关统一：`λ ≤ 0`，**只推不拉**。
        sc.lambda[i] = if sys.cfg.tensile_instability_suppression {
            lam.min(0.0)
        } else {
            lam
        };
    }
}

/// `Δp_i = −(1/ρ0)Σ_j (λ_i + λ_j + s_corr)∇_{p_i}W_ij`：只施加到**流体段**；
/// 边界邻居的那一份折成反作用力（`−m_i·Δp_ij/dt²`，逐对等大反向）。
///
/// ⚠️ **那个负号不是笔误、是量纲/符号口径**：`for_neighbors` 给的是 `d = p_i − p_j`，而
/// Spiky 的 `W'(r) < 0` ⇒ 真梯度 `∇_{p_i}W = −ks(h−r)²·d̂`。`fluid_force.rs` 把符号折进
/// `a += d·coef`（正压 ⇒ `coef>0` ⇒ 沿 `+d̂` 推开）；PBF 这里是 `λ<0 ⇒ 压缩`（`λ = −C/denom`），
/// 要的是**排斥** ⇒ 与 WCSPH 同一读法就必须取负。**漏掉它会变成"压缩处互吸"**：实测
/// 反作用冲量大账残差 19%（水槽里 3 tick 打飞 4.8 m/s），补上后回到机器精度。
fn project(sys: &mut FluidSystem, sc: &mut PbfScratch, dt: f32) {
    let (h, h2, k6, ks, nf) = (
        sys.dom.kern.h,
        sys.dom.kern.h2,
        sys.dom.kern.k6,
        sys.dom.kern.ks,
        sys.n_fluid,
    );
    let inv_rho = 1.0 / sys.cfg.rest_density;
    let inv_dt2 = 1.0 / (dt * dt);
    let supp = sys.cfg.tensile_instability_suppression;
    // W(Δq)，Δq = 0.2h ⇒ (0.2h)² = 0.04h² ⇒ h² 与 0.96h² 的 poly6 值。
    let dq2 = S_CORR_DQ_FRAC * S_CORR_DQ_FRAC * h2;
    let tq = h2 - dq2;
    let wdq = k6 * tq * tq * tq;
    let PbfScratch {
        delta,
        iter,
        lambda,
    } = sc;
    let mut bforce = std::mem::take(&mut sys.dom.ps.bforce);
    for v in iter[..nf].iter_mut() {
        *v = Vec3::ZERO;
    }
    // 第一遍：**位置快照冻结**下算齐本次迭代的全部 Δp_i（Jacobi，即 Müller 算法 1 的两循环）。
    // ⚠️ 逐个"算完就挪"会让后处理的粒子对着**已被挪过**的邻居算 ⇒ 成对力不再等大反向 ⇒ 动量
    // 不守恒：实测反作用冲量大账残差 1.85%（改成两遍后回到机器精度——`d`/`r` 取负后逐位同值）。
    for i in 0..nf {
        let lam_i = lambda[i];
        let mi = sys.dom.ps.pmass[i];
        let mut dp = Vec3::ZERO;
        sys.for_neighbors(i, |j, d, r2| {
            let r = r2.sqrt();
            let t = h - r;
            let kg = ks * t * t;
            let mut s = lam_i + lambda[j];
            if supp {
                // **人工压力项**（论文 Algorithm 1 的 `s_corr`，**绝对量级、无 `λ` 因子**）。
                //
                // 它要治的病：自由表面粒子的邻域被截断 ⇒ `C < 0` ⇒ `λ > 0` ⇒ 成对项净**吸引**
                // ⇒ 水块自己缩（PBF 教科书级 volume loss）。**但这个病现在在源头治了**：
                // `lambda_pass` 按同一 `tensile_instability_suppression` 做了**一边倒约束**
                // `λ ≤ 0`（自由表面口径，与 WCSPH 的"负压钳零"对齐）⇒ 表面只剩 `j` 侧那点
                // 压缩推力、没有假吸引。`s_corr` 因此**默认关**（`S_CORR_K = 0.0`）。
                //
                // 为什么关而不是调：`s_corr` 是**绝对量级**的短程排斥，量级与局部密度误差
                // **无关** ⇒ 在没有密度误差的地方它也是纯人工力、标不出通用 K。实测（自由空间
                // 均匀晶格）K=1e-3 起就撑出人工空腔、K=3e-3 炸开（vmax 9.6、跨 15 m）；而论文的
                // **相对**形式（`×|λ|`）在表面失效（表面 `λ ≈ 1e-5` ⇒ `s_corr ≈ 0`，正是要用它
                // 的地方）。两者都不可标定 ⇒ 留 `K = 0` 供复现实验对照。
                //
                // （保留此分支是为了让论文 Algorithm 1 的结构在代码里仍可辨认、可 A/B；`K = 0`
                // 时它逐位等价于不执行。）
                let tt = h2 - r2;
                let w = k6 * tt * tt * tt;
                let q = (w / wdq).powi(4);
                s -= S_CORR_K * q;
            }
            let pair = d * (-s * kg / r.max(1e-9) * inv_rho); // Δp_ij
            dp += pair;
            if j >= nf {
                bforce[j] -= pair * (mi * inv_dt2);
            }
        });
        iter[i] = dp;
    }
    // 第二遍：一起挪；速度回填沿用**整个子步**的累计 `delta`（`finish`）。
    for i in 0..nf {
        let dp = iter[i];
        delta[i] += dp;
        sys.dom.ps.pos[i] += dp;
    }
    sys.dom.ps.bforce = bforce;
}

/// 收尾：重评密度（投影后的**末态**密度——不可压误差档读的就是它）→ 速度回填
/// `v ← v* + Δp/dt` → XSPH → 非弹性边界投影 → 反作用聚合。
fn finish(sys: &mut FluidSystem, dt: f32, providers: &dyn ProviderColliders, sc: &PbfScratch) {
    let nf = sys.n_fluid;
    let inv_dt = 1.0 / dt;
    let t = vxl_phys_core::probe::start();
    sys.dom.grid.rebuild(&sys.dom.ps.pos, sys.dom.kern.h);
    sys.density_pass(providers);
    sys.phase_us[0] += vxl_phys_core::probe::us(t);
    let t = vxl_phys_core::probe::start();
    for i in 0..nf {
        sys.dom.ps.vel[i] += sc.delta[i] * inv_dt;
    }
    xsph(sys);
    // CFL 限速（与 CPU SPH 档**同一口径**：钳在"本子步速度定稿处"）。`predict` 里那次钳的
    // 是 x* 用的预测速度；`Δp/dt` 与 XSPH 都在这之后叠加 ⇒ 这里再钳一次，保证
    // `velocities()` 对外读数永不超出 `max_speed_frac·h/dt`（两族口径一致）。
    let vmax = sys.cfg.max_speed_frac * sys.dom.kern.h * inv_dt;
    let vmax2 = vmax * vmax;
    for v in sys.dom.ps.vel[..nf].iter_mut() {
        let s2 = v.length_squared();
        if s2 > vmax2 {
            *v *= vmax / s2.sqrt();
        }
    }
    sys.phase_us[3] += vxl_phys_core::probe::us(t);
    let t = vxl_phys_core::probe::start();
    sys.boundary_pass(providers);
    sys.phase_us[4] += vxl_phys_core::probe::us(t);
    sys.aggregate_reactions();
}

/// XSPH 速度平滑（Müller 口径，与 WCSPH 档同一式）：`v_i += ε·Σ_j (v_j−v_i)·(m_j·2/(ρ_i+ρ_j)·W)`。
///
/// 逐对对称 ⇒ 动量守恒；**先全量累加进 `xsph`、再统一回填**（`v_i` 只被写一次
/// ⇒ 与邻居遍历序无关，逐位确定）。边界邻居照常参与（无滑移近似，同 WCSPH 档）。
fn xsph(sys: &mut FluidSystem) {
    let nf = sys.n_fluid;
    let eps = sys.cfg.xsph_viscosity;
    if eps <= 0.0 {
        return;
    }
    for i in 0..nf {
        let vi = sys.dom.ps.vel[i];
        let rho_i = sys.dom.ps.dens[i];
        let mut xs = Vec3::ZERO;
        sys.for_neighbors(i, |j, _d, r2| {
            let tt = sys.dom.kern.h2 - r2;
            let w = sys.dom.kern.k6 * tt * tt * tt;
            xs += (sys.dom.ps.vel[j] - vi)
                * (sys.dom.ps.pmass[j] * 2.0 / (rho_i + sys.dom.ps.dens[j]) * w);
        });
        sys.dom.ps.xsph[i] = xs;
    }
    for i in 0..nf {
        sys.dom.ps.vel[i] += sys.dom.ps.xsph[i] * eps;
    }
}
