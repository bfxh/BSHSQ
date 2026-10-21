//! `fluid_density` 的**算式本体**（密度相位的核）。
//!
//! **为什么单独成文件**：god 棘轮是**只准减**的量具（`fluid_density.rs` 的
//! `file_lines` 不得变大）⇒ 新增的核代码不能塞回原文件，放这里则**只受绝对阈值**
//! （800 行文件 / 120 行函数）管、不受棘轮管。
//!
//! **这里是「一个核、两种口径」的落点**：
//! · `DensityKind` = 密度相位的**口径**（`Fluid` / `Boundary`）——
//!   口径是**参数**，不是新方法 ⇒ 新加一种边界口径**不必再往 `FluidSystem` 添 `*_pass`**；
//! · `density_fluid_particle` = 密度相位的**唯一算式**（串行档与并行档都调它）；
//! · `density_chunk_parallel` = 分块并行的机械部分（流体段与 2b 段共用一份）。
//!
//! 逐位一致性：两种遍历形态调**同一个核** ⇒ 同一粒的算式与邻居遍历序完全相同
//! ⇒ 并行档与串行档逐位一致（不再依赖「两份手抄的算式恰好一致」）。
use super::super::{grid::UniformGrid, InteropContact, ProviderColliders, Vec3};

/// **壁面收集结果**（点 + 外法线；角部可有多面），≤ 8 面。
///
/// **为什么把两个平行数组合成一个结构体**：
/// ① args-gate 的软阈是 7（按**逗号**计数 ⇒ `&mut [Vec3; 8]` 这类带 `; 8` 的类型
///    会被数成 2 个参数）⇒ 两个 `&mut [Vec3; 8]` 形参会让核超阈；
/// ② 更本质：「一组壁面（点 + 法线）」本来就是**一个整体**，两个平行数组
///    是「用类型表达一个概念」的旧写法 ⇒ 合成结构体是设计改进，不是为过门而改。
#[derive(Clone, Copy)]
pub(crate) struct WallPlanes {
    /// 壁面接触点。
    pub pt: [Vec3; 8],
    /// 壁面外法线（与 `pt` 同下标）。
    pub n: [Vec3; 8],
}

impl WallPlanes {
    /// 清空（`pt`/`n` 归零 + 计数为 0）。
    pub(crate) fn new() -> Self {
        Self {
            pt: [Vec3::ZERO; 8],
            n: [Vec3::ZERO; 8],
        }
    }
}

/// **密度相位的口径**（`DensityKind`）——「边界口径是**参数**，不是新方法」。
///
/// 之前流体段与 2b 边界段各写一份算式、各占一个 `FluidSystem` 方法
/// （`density_pass` / `density_boundary_pass`）⇒ 加一种新口径 = 加一个 `*_pass`。
/// 现在两段共用**一个核**，口径由本枚举选 ⇒ 加一种新口径 = 加一个枚举变体
/// （+ 核里一处分支），**不必再往 `FluidSystem` 添方法**。
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub(crate) enum DensityKind {
    /// 流体粒子：ρ_i = m·(W(0) + Σ_j W(r_ij) + Σ_ghost W)（poly6，含自身项）
    /// + 边界镜像鬼影（近壁面反射补回固体一侧的**离散**核质量）。
    Fluid,
    /// 2b 边界粒子：**同一核、同一式**、含自身项，但密度只从**流体**取
    /// （`j < nf`）——Akinci 口径的「边界压力 = 外推的流体压力」：若让边界粒子
    /// 互相供密度，**薄体**（厚度 < ~2h）两侧的内移层会在体内互相穿透，
    /// ρ_b 被自身堆积顶到 q≫1 ⇒ p_b 爆抬 ⇒ 体积力/反作用整片失真
    /// （实测：0.12 m 盒给出 ρVg 的 5.9×、侧向 51 N 的假力，2026-09-22）。
    /// 只吃流体 ⇒ 贴壁处 ρ_b ≈ 该处流体密度 ⇒ p_b ≈ p_f，与「冻结流体粒子」
    /// 的原意一致，且对任意薄厚都稳定。自身项保留（与流体同式）。
    Boundary,
}

/// **密度核的只读上下文**（把 `FluidSystem` 的字段按需借出来 ⇒ 核是自由函数）。
#[derive(Clone, Copy)]
pub(crate) struct DensityCtx<'a> {
    pub pos: &'a [Vec3],
    pub pmass: &'a [f32],
    pub grid: &'a UniformGrid,
    pub h: f32,
    pub h2: f32,
    pub k6: f32,
    pub w0: f32,
    pub mass: f32,
    pub boundaries: &'a [u32],
    /// 流体粒子数（前缀长度）；≥ 该下标的是 2b 边界粒子。
    pub n_fluid: usize,
}

/// **密度相位的唯一算式**（一个核，两种口径）。
///
/// 串行档与并行档**都调这个函数** ⇒ 同一粒的算式与邻居遍历序完全相同
/// ⇒ 并行档与串行档**逐位一致**（原来靠「两份手抄的算式恰好一致」，
/// 现在靠「只有一份算式」——这是本改动与「纯搬移」的本质差别）。
///
/// `walls` 是壁面（点 + 外法线）的调用方缓冲：串行档可复用一份，
/// 并行档每块一份 ⇒ 无共享可变状态。
pub(crate) fn density_fluid_particle(
    ctx: DensityCtx<'_>,
    kind: DensityKind,
    i: usize,
    providers: &dyn ProviderColliders,
    scratch: &mut Vec<InteropContact>,
    walls: &mut WallPlanes,
) -> f32 {
    let nf = ctx.n_fluid;
    let pi = ctx.pos[i];
    match kind {
        DensityKind::Fluid => {
            // 壁面（镜像鬼影用；流体粒子专有，边界粒子不用）。
            let np = wall_planes_in(ctx.boundaries, ctx.h, pi, providers, scratch, walls);
            let mut sum = ctx.w0;
            let mut sum_b = 0.0f32;
            ctx.grid.for_neighbors_in(ctx.pos, ctx.h2, i, |j, d, r2| {
                let t = ctx.h2 - r2;
                let w = ctx.k6 * t * t * t;
                if j < nf {
                    sum += w;
                } else {
                    sum_b += ctx.pmass[j] * w;
                }
                for k in 0..np {
                    let pj = pi - d;
                    let dn = (pj - walls.pt[k]).dot(walls.n[k]);
                    let g = pj - walls.n[k] * (2.0 * dn);
                    let rg2 = (pi - g).length_squared();
                    if rg2 <= ctx.h2 {
                        let tg = ctx.h2 - rg2;
                        sum += ctx.k6 * tg * tg * tg;
                    }
                }
            });
            ctx.mass * sum + sum_b
        }
        DensityKind::Boundary => {
            // 2b：只吃**流体**邻居（Akinci 口径，理由见 `DensityKind::Boundary`）。
            let mut sum = 0.0f32;
            ctx.grid.for_neighbors_in(ctx.pos, ctx.h2, i, |j, _d, r2| {
                if j >= nf {
                    return; // 不吃边界-边界对
                }
                let t = ctx.h2 - r2;
                sum += ctx.pmass[j] * (ctx.k6 * t * t * t);
            });
            ctx.pmass[i] * ctx.w0 + sum
        }
    }
}

pub(crate) fn wall_planes_in(
    boundaries: &[u32],
    h: f32,
    pi: Vec3,
    providers: &dyn ProviderColliders,
    scratch: &mut Vec<InteropContact>,
    walls: &mut WallPlanes,
) -> usize {
    if boundaries.is_empty() {
        return 0;
    }
    let mut np = 0usize;
    for &bid in boundaries {
        if let Some(bb) = providers.bounds(bid) {
            // 预滤余量取 h（镜像带 = sdf < h）。
            let m = h;
            if pi.x < bb.min.x - m
                || pi.x > bb.max.x + m
                || pi.y < bb.min.y - m
                || pi.y > bb.max.y + m
                || pi.z < bb.min.z - m
                || pi.z > bb.max.z + m
            {
                continue;
            }
        }
        scratch.clear();
        if providers.contacts_point(bid, pi, h, scratch) {
            for c in scratch.iter() {
                // sdf = (p − 表面点)·外法线（粒子在固体外侧为正，
                // provider 无关）；穿透（≤ 0）交给投影，不补。
                let sdf = (pi - c.point).dot(c.normal);
                if sdf > 0.0 && sdf < h && np < 8 {
                    walls.pt[np] = c.point;
                    walls.n[np] = c.normal;
                    np += 1;
                }
            }
        }
    }
    np
}

/// **密度相位的分块并行**（流体段与 2b 段共用）。
///
/// 机械部分（分块 + `thread::scope` + spawn）只写**一份** ⇒ 两段只差
/// `kind` 与索引范围 ⇒ 「边界口径是参数」不是只有串行档成立。
///
/// 每块一份 `scratch`（壁面查询缓冲）⇒ 无共享可变状态 ⇒ 与串行**逐位一致**。
pub(crate) fn density_chunk_parallel(
    ctx: DensityCtx<'_>,
    dens: &mut [f32],
    range: std::ops::Range<usize>,
    kind: DensityKind,
    providers: &dyn ProviderColliders,
    threads: usize,
) {
    let (range_start, range_end) = (range.start, range.end);
    let n = range_end - range_start;
    if n == 0 {
        return;
    }
    let per = n.div_ceil(threads).max(1);
    let DensityCtx {
        pos,
        pmass,
        grid,
        h,
        h2,
        k6,
        w0,
        mass,
        boundaries,
        n_fluid,
    } = ctx;
    let (h, h2, k6, w0, mass) = (h, h2, k6, w0, mass);
    let out = &mut dens[range_start..range_end];
    std::thread::scope(|s| {
        for (ci, d_out) in out.chunks_mut(per).enumerate() {
            let base = range_start + ci * per;
            let (pos, pmass, grid, boundaries) = (&*pos, &*pmass, &*grid, &*boundaries);
            s.spawn(move || {
                let mut scratch: Vec<InteropContact> = Vec::new();
                for (k, d_out) in d_out.iter_mut().enumerate() {
                    let i = base + k;
                    let mut walls = WallPlanes::new();
                    *d_out = density_fluid_particle(
                        DensityCtx {
                            pos,
                            pmass,
                            grid,
                            h,
                            h2,
                            k6,
                            w0,
                            mass,
                            boundaries,
                            n_fluid,
                        },
                        kind,
                        i,
                        providers,
                        &mut scratch,
                        &mut walls,
                    );
                }
            });
        }
    });
}
