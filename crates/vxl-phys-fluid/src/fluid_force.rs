//! fluid_force：从 lib.rs 按域拆出（纯搬移，语义未改）。
use super::*;

mod kernel;
use kernel::{force_fluid_particle, ForceCtx};

impl FluidSystem {
    /// 压力：Tait p = B·((ρ/ρ0)^γ − 1)；γ=7 走整数次幂乘法展开。
    /// `tensile_instability_suppression`（默认开）= Monaghan 自由面钳制
    /// p ≥ 0：表面密度截断（q ≈ 0.5） otherwise 会给出 p ≈ −B，经对称
    /// spiky 形式变成巨大粒子间吸力（单子步 Δv ~ 10² m/s）⇒ 全体炸散。
    /// 覆盖**全部**粒子（含 2b 边界粒子——它们要参与压力对，也吃同一套钳制：
    /// 悬空/真空里的边界粒子 ρ 很小 ⇒ p 钳到 0 ⇒ 不产生凭空吸力）。
    pub(crate) fn pressure_pass(&mut self) {
        let rho0 = self.cfg.rest_density;
        let g = self.cfg.gamma_tait;
        let clamp = self.cfg.tensile_instability_suppression;
        for i in 0..self.pos.len() {
            let q = self.dens[i] / rho0;
            let mut p = if (g - 7.0).abs() < 1e-6 {
                let q2 = q * q;
                let q4 = q2 * q2;
                self.b_tait * (q * q2 * q4 - 1.0)
            } else {
                self.b_tait * (q.powf(g) - 1.0)
            };
            if clamp && p < 0.0 {
                p = 0.0;
            }
            self.press[i] = p;
        }
    }

    /// **力相位·唯一入口**（原 `force_pass`）。
    ///
    /// 算式在**一个核**里（`force_fluid_particle` 自由函数）：串行档与并行档
    /// **调同一个核** ⇒ 逐位一致靠「只有一份算式」保证，不再靠「两份手抄的算式恰好一致」。
    pub(crate) fn force_pass(&mut self) {
        let nf = self.n_fluid;
        let nt = self.pos.len();
        // 并行档（`cfg.threads > 1`）：逐粒独立 ⇒ 分块并行（逐位一致，见 config 注）。
        if self.cfg.threads > 1 {
            self.force_pass_parallel(nf, nt);
        } else {
            self.force_pass_serial(nf);
        }
        // 物理层流黏度（μ = 0 自短路）——必须在上面**赋值** `acc` 之后累加（见 `laminar.rs`）。
        crate::laminar::pass(self);
    }

    /// **力相位·并行档**（`threads > 1`）：`acc`/`xsph` 逐粒独立 ⇒ 分块并行；
    /// 唯一跨粒子的 `bforce`（2b 反作用）在有边界粒子时走**串行补趟** ⇒ 保逐位一致。
    fn force_pass_parallel(&mut self, nf: usize, nt: usize) {
        let threads = self.cfg.threads.max(1);
        let g = self.cfg.gravity;
        let alpha_c = self.cfg.artificial_viscosity * self.cfg.sound_speed;
        for f in &mut self.bforce[nf..] {
            *f = Vec3::ZERO;
        }
        {
            let Self {
                pos,
                vel,
                dens,
                press,
                acc,
                xsph,
                pmass,
                grid,
                h,
                h2,
                k6,
                ks,
                ..
            } = self;
            let (h, h2, k6, ks) = (*h, *h2, *k6, *ks);
            let per = nf.div_ceil(threads).max(1);
            std::thread::scope(|s| {
                for (ci, (acc_c, xs_c)) in acc[..nf]
                    .chunks_mut(per)
                    .zip(xsph[..nf].chunks_mut(per))
                    .enumerate()
                {
                    let base = ci * per;
                    let (pos, vel, dens, press, pmass, grid) =
                        (&*pos, &*vel, &*dens, &*press, &*pmass, &*grid);
                    s.spawn(move || {
                        for (k, (a_out, x_out)) in acc_c.iter_mut().zip(xs_c.iter_mut()).enumerate()
                        {
                            let i = base + k;
                            let (a, xs) = force_fluid_particle(
                                ForceCtx {
                                    pos,
                                    vel,
                                    dens,
                                    press,
                                    pmass,
                                    grid,
                                    h,
                                    h2,
                                    k6,
                                    ks,
                                    g,
                                    alpha_c,
                                    n_fluid: nf,
                                },
                                i,
                                None,
                            );
                            *a_out = a;
                            *x_out = xs;
                        }
                    });
                }
            });
        }
        // 2b 反作用：**串行补趟**（只有边界粒子存在时才走）⇒ 单独成相位（见文件尾）。
        if nt > nf {
            boundary_reaction_serial(self, nf, alpha_c);
        }
        self.aggregate_reactions();
    }

    /// **力相位·串行档**（默认档：`threads == 1` ⇒ 与历史行为逐位一致）。
    fn force_pass_serial(&mut self, nf: usize) {
        let g = self.cfg.gravity;
        let alpha_c = self.cfg.artificial_viscosity * self.cfg.sound_speed;
        // `for_neighbors` 借 `&self`，闭包里要写边界粒子受力 ⇒ 把缓冲整体借出
        // （`Vec::take` 是 O(1)，不分配），循环后归还。
        let mut bforce = std::mem::take(&mut self.bforce);
        for f in &mut bforce[nf..] {
            *f = Vec3::ZERO; // 每子步重新累加（反作用 = 本子步的力）
        }
        // 核要写 `bforce[j]`（2b 反作用）⇒ 把缓冲**借给核**（`&mut bforce[..]`）；
        // 借用作用域 = 单次调用 ⇒ 循环结束即释放，随后整块归还。
        for i in 0..nf {
            let (a, xs) = force_fluid_particle(
                ForceCtx {
                    pos: &self.pos,
                    vel: &self.vel,
                    dens: &self.dens,
                    press: &self.press,
                    pmass: &self.pmass,
                    grid: &self.grid,
                    h: self.h,
                    h2: self.h2,
                    k6: self.k6,
                    ks: self.ks,
                    g,
                    alpha_c,
                    n_fluid: nf,
                },
                i,
                Some(&mut bforce[..]),
            );
            self.acc[i] = a;
            self.xsph[i] = xs;
        }
        self.bforce = bforce;
        self.aggregate_reactions();
    }
}

/// **2b 反作用·串行补趟**（从 `force_pass_parallel` 按相位拆出；纯搬移，语义未改）：
/// 逐粒 `bforce`（只有边界粒子存在时才走）——与串行路径同一算式、同一遍历序 ⇒ 逐位一致。
/// 缓冲用 `take` 借出以便闭包内写。**自由函数**：`FluidSystem` 的方法数是 god 门的
/// 登记债务（只准减），相位拆出不占方法位（同 `laminar::pass` / `fluid_medium::scan` 先例）。
fn boundary_reaction_serial(sys: &mut FluidSystem, nf: usize, alpha_c: f32) {
    let mut bforce = std::mem::take(&mut sys.bforce);
    for i in 0..nf {
        let vi = sys.vel[i];
        let rho_i = sys.dens[i];
        let ci2 = sys.press[i] / (rho_i * rho_i);
        let mi = sys.pmass[i];
        sys.for_neighbors(i, |j, d, r2| {
            if j < nf {
                return;
            }
            let rho_j = sys.dens[j];
            let cj2 = sys.press[j] / (rho_j * rho_j);
            let r = r2.sqrt();
            let t = sys.h - r;
            let mj = sys.pmass[j];
            let coef = mj * (sys.ks * t * t) * (ci2 + cj2);
            let denom = r.max(1e-9);
            let cv = {
                let vij = vi - sys.vel[j];
                let vdn = -vij.dot(d);
                if vdn > 0.0 {
                    let mu = vdn * sys.h / (r2 + 0.01 * sys.h2);
                    mj * (alpha_c * mu / (0.5 * (rho_i + rho_j))) * (sys.ks * t * t)
                } else {
                    0.0
                }
            };
            bforce[j] -= d * (mi * (coef + cv) / denom);
        });
    }
    sys.bforce = bforce;
}
