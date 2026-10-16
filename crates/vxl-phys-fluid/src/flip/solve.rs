//! FLIP **压力求解器**：散度、压力 Poisson 的两种解法（CG / Jacobi）与两者共用的压力模板。
//!
//! 为什么单开一个文件：CG 把求解器从"一趟 Jacobi 扫"扩成了"Krylov 迭代 + 对角预处理"，
//! 塞在 `flip.rs` 里会把那个文件顶到 `god_gate` 的 800 行硬线附近（棘轮还只准减）。口径
//! （八步流程、边界条件、确定性契约）都在 `flip.rs` 模块头，这里只放"怎么解那一坨 Poisson"。
//!
//! 可见性：`flip.rs` 的 `substep` 只调 `solve_pressure`（`pub(super)`）；其余
//! （`solve_cg`/`solve_jacobi`/`precond`/`for_each_template_nb`/`dot`/`rinf`/`cg_a`）都是本文件
//! 的内部件，保持私有。
use super::{FlipScratch, CLASS_FLUID, CLASS_SOLID};
use crate::FluidSystem;

/// CG 的相对残差判据：`max|r| <= CG_RTOL*max|b|` 即停手。f32 的单精度极限在 1e-7 量级，
/// 1e-5 是"再迭代也不动"之前的一档（收敛轨迹见 `tests/flip_incompressibility_probe.rs`）。
const CG_RTOL: f32 = 1e-5;

impl FlipScratch {
    /// 散度 + 压力 Poisson（求解器见 `solve_cg` / `solve_jacobi`）：
    /// `Σ(p_nb - p_c) = (ρ/Δt)*grid_step²*div`，空单元 `p = 0`（Dirichlet）、
    /// 固体面/网格外边界不入模板（Neumann）。默认走 **CG**（`cfg.flip_pressure_cg`），
    /// 几何一致时用上一子步的解热启动（见模块头"热启动"）。
    pub(super) fn solve_pressure(&mut self, sys: &FluidSystem, dt: f32) {
        let grid_step = self.grid_step;
        let alpha = sys.cfg.rest_density / dt * grid_step * grid_step;
        let (nx, ny, nz) = (self.dims[0], self.dims[1], self.dims[2]);
        for i in 0..nx {
            for j in 0..ny {
                for k in 0..nz {
                    let c = self.cell_idx(i, j, k);
                    if self.class[c] != CLASS_FLUID {
                        self.div[c] = 0.0;
                        continue;
                    }
                    let d = (self.u[self.u_idx(i + 1, j, k)] - self.u[self.u_idx(i, j, k)])
                        + (self.v[self.v_idx(i, j + 1, k)] - self.v[self.v_idx(i, j, k)])
                        + (self.w[self.w_idx(i, j, k + 1)] - self.w[self.w_idx(i, j, k)]);
                    self.div[c] = d / grid_step;
                }
            }
        }
        // 热启动（只给 CG 档，见模块头"热启动"）：几何键与上一子步一致时，用那份解当初始
        // 猜测 —— 只填**当前是流体**的单元（现在不是流体的单元在解里恒为 0，别把旧值带进
        // `pressures()`）。Jacobi 档保持从零起：它是固定次数的 A/B 对照档，预置初值会把
        // "N 次扫"的历史口径搅浑。
        let warm = sys.cfg.flip_pressure_cg && self.warm_reusable();
        self.p.clear();
        self.p.resize(self.n_cells, 0.0);
        if warm {
            self.p.copy_from_slice(&self.p_prev);
            for c in 0..self.n_cells {
                if self.class[c] != CLASS_FLUID {
                    self.p[c] = 0.0;
                }
            }
        }
        let iters = sys.cfg.flip_pressure_iterations;
        if sys.cfg.flip_pressure_cg {
            self.solve_cg(alpha, iters, warm);
        } else {
            self.solve_jacobi(alpha, iters);
        }
        // 记账：把这次解出的场连几何键留下（下一子步若几何一致 => 热启动）。
        if sys.cfg.flip_pressure_cg {
            self.p_prev.clear();
            self.p_prev.extend_from_slice(&self.p);
            self.warm_key = Some((self.origin, self.grid_step, self.dims));
        }
    }

    /// 热启动复用判据：解过（`warm_key` 有值）、维度相同、`grid_step` 相对漂移 ≤ 1e-4、
    /// `origin` 漂移 ≤ 半格。
    ///
    /// 为什么允许漂移而不是要求键逐位相同：网格每子步按**流体 AABB** 重建，粒子哪怕动 1e-7
    /// 也会把 `origin` 的位模式改掉 —— 判据一严，热启动在真实场景里就等于没做。半格以内的
    /// 漂移下 `(i,j,k)` 仍指着**大致同一个**物理单元，旧场仍是个好初值；收敛与否始终由残差
    /// 判据说话，初值只影响迭代数。超限就退回冷启动（慢，但正确）。
    fn warm_reusable(&self) -> bool {
        let Some((origin, grid_step, dims)) = self.warm_key else {
            return false;
        };
        if dims != self.dims || self.p_prev.len() != self.n_cells {
            return false;
        }
        if (grid_step - self.grid_step).abs() > self.grid_step.abs() * 1e-4 {
            return false;
        }
        let d = origin - self.origin;
        let lim = 0.5 * self.grid_step.abs();
        d.x.abs() <= lim && d.y.abs() <= lim && d.z.abs() <= lim
    }

    /// 压力模板的**单点遍历**（唯一来源：Jacobi 的一趟扫与 CG 的 `A*d` 共用同一段邻居逻辑，
    /// 免得两份模板各自演化）。`f(nb, is_fluid)` 依次给出六向里**入模板**的邻居：越界（网格外
    /// 边界）与固体 = 无通量 => **不调用**；空单元 = Dirichlet 0 => 调用但 `is_fluid = false`
    /// （只贡献对角项）。遍历序固定（-x,+x,-y,+y,-z,+z，与 Jacobi 原序一致）。
    fn for_each_template_nb(&self, i: usize, j: usize, k: usize, mut f: impl FnMut(usize, bool)) {
        let (nx, ny, nz) = (self.dims[0], self.dims[1], self.dims[2]);
        let nbs = [
            (i.checked_sub(1).unwrap_or(usize::MAX), j, k),
            (i + 1, j, k),
            (i, j.checked_sub(1).unwrap_or(usize::MAX), k),
            (i, j + 1, k),
            (i, j, k.checked_sub(1).unwrap_or(usize::MAX)),
            (i, j, k + 1),
        ];
        for &(ni, nj, nk) in &nbs {
            if ni >= nx || nj >= ny || nk >= nz {
                continue;
            }
            let cn = self.cell_idx(ni, nj, nk);
            match self.class[cn] {
                CLASS_SOLID => {} // Neumann：不入模板
                CLASS_FLUID => f(cn, true),
                _ => f(cn, false), // 空：Dirichlet 0 => 只算对角
            }
        }
    }

    /// 对角预处理：`z = M^-1 r`，`M = diag A`（对角 = 入模板的邻居数）。非流体行是恒等
    /// （`p = 0` => `r = 0`）=> `z = 0`。
    fn precond(&self, r: &[f32], z: &mut [f32]) {
        let (nx, ny, nz) = (self.dims[0], self.dims[1], self.dims[2]);
        for i in 0..nx {
            for j in 0..ny {
                for k in 0..nz {
                    let c = self.cell_idx(i, j, k);
                    if self.class[c] != CLASS_FLUID {
                        z[c] = 0.0;
                        continue;
                    }
                    let mut ap = 0.0f32;
                    self.for_each_template_nb(i, j, k, |_, _| ap += 1.0);
                    z[c] = if ap > 0.0 { r[c] / ap } else { 0.0 };
                }
            }
        }
    }

    /// **共轭梯度**（默认档）：同一系统 `A p = b`，`b_c = -α*div_c`（仅流体单元非零），
    /// `(A x)_c = ap*x_c - Σ_{流体邻} x_nb`（空单元只进对角、固体/外边界不入模板）。
    /// 对角预处理；相对残差 `CG_RTOL` 或 `flip_pressure_iterations` 次封顶即停。
    ///
    /// 为什么换掉 Jacobi：Jacobi 在 `N` 格高的柱上要 ~`N²` 次才收敛（"已知简化 2"），固定 80
    /// 次只能**部分**抵消重力（实测 `height_keep` 0.85）；CG 的迭代数按 `sqrt(κ(A))` 走，同样
    /// 预算下残差低几个量级 => 静水柱的离散解 `p = ρg(H-y)` 被真正解出来（探针 CG 行）。
    ///
    /// 确定性：向量运算全是**固定索引序**的逐元素/归约（无并行重排、无原子）=> 与 Jacobi 同一
    /// 逐位契约（`flip_is_bitwise_reproducible`）。分解保险：`d*A*d` 非正或非有限即停手
    /// （SPD 下不该发生；宁停不炸，解停在当前迭代）。
    ///
    /// 热启动（`warm = true`，`p` 已被 `solve_pressure` 预置成上一子步的解）：初残差取
    /// `b - A*p`（多算一次 `A p`，换掉冷启动的几十次扫）。判据的分母也相应取
    /// `max(|b|, |r0|)`：冷启动下二者逐位相等（`r0 = b`）=> 行为**逐位不变**；热启动下
    /// `r0` 可能远小于 `b`（本来就快收敛了），只拿 `|b|` 当分母会在 `b = 0` 的场景逼着把解
    /// 磨到恰好 0，白撞上限。
    fn solve_cg(&mut self, alpha: f32, max_iters: u32, warm: bool) {
        let (nx, ny, nz) = (self.dims[0], self.dims[1], self.dims[2]);
        let mut r = std::mem::take(&mut self.cg_r);
        let mut d = std::mem::take(&mut self.cg_d);
        let mut q = std::mem::take(&mut self.cg_q);
        let mut z = std::mem::take(&mut self.p_tmp);
        r.clear();
        d.clear();
        q.clear();
        z.clear();
        r.resize(self.n_cells, 0.0);
        d.resize(self.n_cells, 0.0);
        q.resize(self.n_cells, 0.0);
        z.resize(self.n_cells, 0.0);
        // 右端 b -> r（非流体单元 = 0，即 Dirichlet 的恒等行）。
        let mut bmax = 0.0f32;
        for i in 0..nx {
            for j in 0..ny {
                for k in 0..nz {
                    let c = self.cell_idx(i, j, k);
                    if self.class[c] != CLASS_FLUID {
                        continue;
                    }
                    let b = -alpha * self.div[c];
                    r[c] = b;
                    bmax = bmax.max(b.abs());
                }
            }
        }
        // 热启动：残差不是 `b` 而是 `b - A*p_prev`（`cg_a` 与迭代里用的是同一段模板）。
        if warm {
            cg_a(self, &self.p, &mut q);
            for c in 0..self.n_cells {
                if self.class[c] == CLASS_FLUID {
                    r[c] -= q[c];
                }
            }
        }
        self.precond(&r, &mut z);
        d.copy_from_slice(&z);
        let mut rz = dot(&r, &z);
        let mut rmax = rinf(&r);
        let tol = CG_RTOL * bmax.max(rmax);
        let mut it = 0u32;
        while it < max_iters && rmax > tol {
            it += 1;
            cg_a(self, &d, &mut q);
            let dq = dot(&d, &q);
            if !dq.is_finite() || dq <= 0.0 {
                break; // 分解保险（SPD 下不可达）
            }
            let a = rz / dq;
            if !a.is_finite() {
                break;
            }
            for (pp, dd) in self.p.iter_mut().zip(d.iter()) {
                *pp += a * *dd;
            }
            for (rr, qq) in r.iter_mut().zip(q.iter()) {
                *rr -= a * *qq;
            }
            rmax = rinf(&r);
            if rmax <= tol {
                break;
            }
            self.precond(&r, &mut z);
            let rz_new = dot(&r, &z);
            let beta = rz_new / rz.max(f32::MIN_POSITIVE);
            for (dd, zz) in d.iter_mut().zip(z.iter()) {
                *dd = *zz + beta * *dd;
            }
            rz = rz_new;
        }
        self.cg_r = r;
        self.cg_d = d;
        self.cg_q = q;
        self.p_tmp = z;
    }

    /// 固定次数 Jacobi 扫（`flip_pressure_cg = false` 档；A/B 与复现实验用）：全量读旧、写新
    /// => 与遍历序无关。收敛慢（`N²`），但每次迭代最便宜、结构最简单。
    fn solve_jacobi(&mut self, alpha: f32, iters: u32) {
        let (nx, ny, nz) = (self.dims[0], self.dims[1], self.dims[2]);
        for _ in 0..iters {
            for i in 0..nx {
                for j in 0..ny {
                    for k in 0..nz {
                        let c = self.cell_idx(i, j, k);
                        if self.class[c] != CLASS_FLUID {
                            continue;
                        }
                        let mut ap = 0.0f32;
                        let mut sum = 0.0f32;
                        let p = &self.p;
                        self.for_each_template_nb(i, j, k, |cn, is_fluid| {
                            ap += 1.0;
                            if is_fluid {
                                sum += p[cn];
                            }
                        });
                        self.p_tmp[c] = if ap > 0.0 {
                            (sum - alpha * self.div[c]) / ap
                        } else {
                            0.0
                        };
                    }
                }
            }
            for c in 0..self.n_cells {
                if self.class[c] == CLASS_FLUID {
                    self.p[c] = self.p_tmp[c];
                }
            }
        }
    }
}

/// 定序点积（固定索引序 => 逐位可复现；f32 单精度，`f64_gate` 禁 f64）。
fn dot(a: &[f32], b: &[f32]) -> f32 {
    a.iter().zip(b.iter()).map(|(x, y)| x * y).sum()
}

/// 残差的 `max|.|`（CG 的停手判据量）。
fn rinf(a: &[f32]) -> f32 {
    a.iter().fold(0.0f32, |m, x| m.max(x.abs()))
}

/// `q = A*d`：`(A x)_c = ap*x_c - Σ_{流体邻} x_nb`（空单元只进对角、固体/外边界不入模板）。
fn cg_a(sc: &FlipScratch, x: &[f32], q: &mut [f32]) {
    let (nx, ny, nz) = (sc.dims[0], sc.dims[1], sc.dims[2]);
    for i in 0..nx {
        for j in 0..ny {
            for k in 0..nz {
                let c = sc.cell_idx(i, j, k);
                if sc.class[c] != CLASS_FLUID {
                    q[c] = 0.0;
                    continue;
                }
                let mut ap = 0.0f32;
                let mut sum = 0.0f32;
                sc.for_each_template_nb(i, j, k, |cn, is_fluid| {
                    ap += 1.0;
                    if is_fluid {
                        sum += x[cn];
                    }
                });
                q[c] = ap * x[c] - sum;
            }
        }
    }
}
