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
    /// `Σ(p_nb - p_c) = (ρ/Δt)*cell²*div`，空单元 `p = 0`（Dirichlet）、
    /// 固体面/网格外边界不入模板（Neumann）。默认走 **CG**（`cfg.flip_pressure_cg`）。
    pub(super) fn solve_pressure(&mut self, sys: &FluidSystem, dt: f32) {
        let cell = self.cell;
        let alpha = sys.cfg.rest_density / dt * cell * cell;
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
                    self.div[c] = d / cell;
                }
            }
        }
        self.p.clear();
        self.p.resize(self.n_cells, 0.0);
        let iters = sys.cfg.flip_pressure_iterations;
        if sys.cfg.flip_pressure_cg {
            self.solve_cg(alpha, iters);
        } else {
            self.solve_jacobi(alpha, iters);
        }
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
    fn solve_cg(&mut self, alpha: f32, max_iters: u32) {
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
        self.precond(&r, &mut z);
        d.copy_from_slice(&z);
        let mut rz = dot(&r, &z);
        let mut rmax = rinf(&r);
        let tol = CG_RTOL * bmax;
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
