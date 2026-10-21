//! `fluid_force` 的**算式本体**（力相位的核）。
//!
//! **为什么单独成文件**：god 棘轮是**只准减**的量具（`fluid_force.rs` 的
//! `file_lines` 不得变大）⇒ 新增的核代码不能塞回原文件，放这里则**只受绝对阈值**
//! （800 行文件 / 120 行函数）管、不受棘轮管。
//!
//! **这里是「一个核」的落点**：`force_fluid_particle` = 力相位的**唯一算式**
//! （串行档与并行档都调它）。原来两份手抄的算式意味着「改一条公式要改两处、
//! 且两处可能悄悄分叉」。
//!
//! 逐位一致性：两种遍历形态调**同一个核** ⇒ 同一粒的算式与邻居遍历序完全相同
//! ⇒ 并行档与串行档逐位一致。
//!
//! `bforce` 参数 = 2b 反作用的累加器：`Some` = 串行档（就地累加，与原
//! `force_pass_serial` 同口径）；`None` = 并行档（反作用另走一趟
//! `boundary_reaction_serial`，见文件尾）。
use super::super::{grid::UniformGrid, Vec3};

/// **力相位的只读上下文**（核是自由函数 ⇒ 把 `FluidSystem` 的字段按需借出来）。
#[derive(Clone, Copy)]
pub(crate) struct ForceCtx<'a> {
    pub pos: &'a [Vec3],
    pub vel: &'a [Vec3],
    pub dens: &'a [f32],
    pub press: &'a [f32],
    pub pmass: &'a [f32],
    pub grid: &'a UniformGrid,
    pub h: f32,
    pub h2: f32,
    pub k6: f32,
    pub ks: f32,
    pub g: Vec3,
    pub alpha_c: f32,
    pub n_fluid: usize,
}

/// **力相位的唯一算式**（一个核）。
///
/// 串行档与并行档**都调这个函数** ⇒ 同一粒的算式与邻居遍历序完全相同
/// ⇒ 逐位一致（原来靠「两份手抄的算式恰好一致」，现在靠「只有一份算式」）。
///
/// 算式 = 重力 + 压力梯度（spiky，对称形式 p_i/ρ_i² + p_j/ρ_j²）
/// + XSPH 速度平滑（对称形式 Σ m·2/(ρ_i+ρ_j)·W·(v_j − v_i)；系数逐对对称
///   ⇒ 与压力项一样动量守恒）+ Monaghan 人工黏度（仅接近对 v_ij·d < 0）。
pub(crate) fn force_fluid_particle(
    ctx: ForceCtx<'_>,
    i: usize,
    mut bforce: Option<&mut [Vec3]>,
) -> (Vec3, Vec3) {
    let nf = ctx.n_fluid;
    let vi = ctx.vel[i];
    let rho_i = ctx.dens[i];
    let ci2 = ctx.press[i] / (rho_i * rho_i);
    let mi = ctx.pmass[i];
    let mut acc = ctx.g;
    let mut xs = Vec3::ZERO;
    ctx.grid.for_neighbors_in(ctx.pos, ctx.h2, i, |j, d, r2| {
        let rho_j = ctx.dens[j];
        let cj2 = ctx.press[j] / (rho_j * rho_j);
        let r = r2.sqrt();
        let t = ctx.h - r;
        let mj = ctx.pmass[j];
        // a_i += m_j·(p_i/ρ_i² + p_j/ρ_j²)·45/(πh⁶)·(h−r)²·d̂（d̂ 指离 j）。
        let coef = mj * (ctx.ks * t * t) * (ci2 + cj2);
        let denom = r.max(1e-9);
        acc += d * (coef / denom);
        // Monaghan 人工黏度：仅接近对（v_ij·d < 0），Π = α·c·μ/ρ̄、
        // μ = h·|v_ij·d|/(r² + 0.01h²)；逐对反对称 ⇒ 动量守恒，耗散法向动能。
        let vij = vi - ctx.vel[j];
        let vdn = -vij.dot(d);
        let cv = if vdn > 0.0 {
            let mu = vdn * ctx.h / (r2 + 0.01 * ctx.h2);
            let c = mj * (ctx.alpha_c * mu / (0.5 * (rho_i + rho_j))) * (ctx.ks * t * t);
            acc += d * (c / denom);
            c
        } else {
            0.0
        };
        if let Some(bf) = bforce.as_deref_mut() {
            if j >= nf {
                // 反作用（作用在边界粒子上）= −F_i = −m_i·d·(逐对系数/r)。
                bf[j] -= d * (mi * (coef + cv) / denom);
            }
        }
        let tt = ctx.h2 - r2;
        let w = ctx.k6 * tt * tt * tt;
        xs += (ctx.vel[j] - vi) * (mj * 2.0 / (rho_i + rho_j) * w);
    });
    (acc, xs)
}
