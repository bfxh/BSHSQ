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
//!    （经典 FLIP：cell ≈ 粒径）；单元数超 `MAX_CELLS` 时整体放大边长（防病态 AABB）。
//! 4. 分类单元：**空 / 流 / 固**。固 = 体中心落在固体内部（`contacts_point_boundary` 的
//!    `depth > probe ⇔ sdf < 0`，与 `boundary_pass` 同一提供者口径）；流 = 粒子三线性权重和 > 0；
//!    **固优先**（穿透进固体的粒子单元仍算固）。
//! 5. P2G：粒子速度按三线性散射到三套交错面（u/v/w），逐面权重归一；无权重面沿轴外推
//!    （自由表面外侧）；**固体面与网格外边界置无通量（0）**——网格外边界当闭合箱（§BC）。
//! 6. 散度 + **Jacobi** 压力 Poisson（`cfg.flip_pressure_iterations` 次）：空单元 `p = 0`
//!    （Dirichlet）、固体面 Neumann（不入模板）、固体单元不参与。
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
//! 遍历序 = 单元索引序（i,j,k 固定）与粒子索引序；压力求解用 **Jacobi**（全量读旧、写新，
//! 与遍历序无关）而非 Gauss-Seidel；无原子、无浮点归约重排 ⇒ 同输入两跑**逐位一致**
//! （判据 `tests/flip_incompressibility_probe.rs`）。
//!
//! ## 验证（`tests/flip_incompressibility_probe.rs`，水槽静水池 60 tick × 4 子步）
//!
//! ① **静水支撑**：判别量 = 末态柱高 / 起步柱高（`height_keep`）。投影若形同虚设，柱子会被
//! 自身重力压成饼 ⇒ 该比值先塌。实测按迭代数 10/20/40/**80**/160/320 ⇒
//! `0.30 / 0.48 / 0.71 / 0.85 / 0.93 / 1.02` ⇒ **单调收敛、无平台、320 次回到 1.0**：
//! 这同时说明**离散公式本身是对的**（`p = ρg(H−y)` 是该 Poisson 系统在"顶面 Dirichlet、
//! 底面 Neumann"下的精确解），只是 Jacobi 的迭代预算限制了它。
//! ② 同判据下的**净沉降**（池内质心 y 对比自由落体 `½gT² = 4.91 m`）：80 次档 ≈ 0.059 m（1.2%）。
//! ③ **低耗散**（本族立身之本）：同场景同迭代，θ = 0.95 的末态 `Σ|v|²` ≈ 17.4 vs 纯 PIC（θ = 0）
//! 的 2.1 ⇒ 混合比确实在起作用（若桥接项误写成 PIC，这条先红）。
//! ④ 逐位确定：`flip_is_bitwise_reproducible`。
//!
//! ⚠️ **本档不保证粒子级密度/无重叠**：FLIP 的不可压约束装在**网格**上，粒子之间没有排斥；
//! 粒子叠在一起时网格只当它们是"一团"，没有任何项把它们分开（PIC 的重置会让重叠**冻住**而非
//! 散开）。所以 SPH 的 `ρ/ρ0` 对本族**不是判据**（求解器根本不读 `densities()`，它只是 `density_pass`
//! 出的被动诊断），本档的探针也只把它当**读数**印出来、不做横向断言——拿 `ρ` 去比 WCSPH/PBF
//! 是在拿别人的量尺量本族。治粒子级重叠要靠 APIC 的仿射矩阵或最小距离约束（登记为下一刀，
//! 与"已知简化 4"相邻）。
//!
//! ## 已知简化（切片 1，逐条登记）
//!
//! 1. **每子步重建网格 + 分配 scratch**——`FluidSystem` 字段数（26 > 24）与 `pbf` 档的
//!    `PbfScratch` 是同一笔登记债务；本档先要正确性与可测性。把压力场跨子步**热启动**能
//!    一次解决"简化 2"的迭代预算问题（稳态下 `p` 几乎不变），但那需要一份**活过子步**的缓冲；
//!    现在每子步现建 ⇒ 只能从零起算。**下一刀首选**。
//! 2. **压力求解是固定次数的 Jacobi**，不是多重网格/预条件 ⇒ 静水柱只**部分**抵消重力
//!    （Jacobi 在 N 格高柱上要 ~N² 次才收敛；见上"验证①"的实测收敛表）。默认
//!    `flip_pressure_iterations = 80` 是"水线掉幅 ≤ 15%"与"求解成本线性"的折中。
//!    换 **CG / 竖直方向线隐式（Thomas）/ 多重网格**是下一刀（`height_keep` 已可直接当验收量）。
//! 3. **刚体反作用**：本档沿用 `boundary_pass` 的 2b 边界粒子账（`bforce`/`breact`），
//!    **没有**把网格的固体面压力积分成对刚体的反作用 ⇒ 网格的固体 BC 是"看不见的静止墙"。
//!    流体↔刚体的双向力请走 2b 边界粒子档（`set_boundary_particles`），登记为下一刀。
//! 4. 单元边长固定取粒子间距，不做自适应；APIC 的仿射矩阵（`C` 矩阵）未实现 ⇒ 本档是
//!    FLIP/PIC 混合，不是 APIC（后者抗旋转耗散更强）。
//! 5. **粒子级重叠无约束**（见上"⚠️"）⇒ `densities()` 对 FLIP 是被动读数。凡是需要密度场
//!    自洽的相位（`fluid_force.rs` 的负压钳零、表面张力、`density_pass` 的镜像鬼影边界）在
//!    本族下都只是"顺带算出来的"，**不被求解器消费**——别在本族上调它们的参数期待效果。
use crate::{FluidSystem, ProviderColliders, Vec3};

// 网格层（建网格/分类/索引/三线性模板）拆在同名子模块里：`flip.rs` 一度 837 行、超 `god_gate`
// 的 800 行硬线；拆开后两层各自远在限内，上面那整段口径说明也不必为了行数砍掉。
mod grid;

use self::grid::{cell_axis, weight_of};

/// 网格相对流体粒子 AABB 的外扩（单元数）：留出空单元作自由表面 Dirichlet 层，
/// 同时保证 P2G 的 2×2×2 三线性模板不会越出面数组边界。
const MARGIN_CELLS: f32 = 2.0;
/// 固体分类探针半径 = `cell × 本系数`。判据 `depth > probe ⇔ sdf < 0`（内点）；
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
/// `[origin + i·cell, origin + (i+1)·cell)`，面 `i` 在单元坐标 `i` 处。
pub(crate) struct FlipScratch {
    origin: Vec3,
    cell: f32,
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
    p_tmp: Vec<f32>,
    div: Vec<f32>,
}

impl FlipScratch {
    /// P2G：逐流体粒子把速度三线性散射到三套面 + 面权重。
    fn p2g(&mut self, sys: &FluidSystem) {
        let (nx, ny, nz) = (self.dims[0], self.dims[1], self.dims[2]);
        for i in 0..sys.n_fluid {
            let p = sys.pos[i];
            let v = sys.vel[i];
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
                        // u（x 面）：沿 x 用面索引，沿 y/z 用单元索引。
                        if ci <= nx && cj < ny && ck < nz {
                            let idx = self.u_idx(ci, cj, ck);
                            self.u[idx] += v.x * w;
                            self.wu[idx] += w;
                        }
                        // v（y 面）。
                        if ci < nx && cj <= ny && ck < nz {
                            let idx = self.v_idx(ci, cj, ck);
                            self.v[idx] += v.y * w;
                            self.wv[idx] += w;
                        }
                        // w（z 面）。
                        if ci < nx && cj < ny && ck <= nz {
                            let idx = self.w_idx(ci, cj, ck);
                            self.w[idx] += v.z * w;
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

    /// 散度 + Jacobi 压力 Poisson：`Σ(p_nb − p_c) = (ρ/Δt)·cell²·div`，
    /// 空单元 `p = 0`（Dirichlet）、固体面不入模板（Neumann）。
    fn solve_pressure(&mut self, sys: &FluidSystem, dt: f32) {
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
        for _ in 0..iters {
            for i in 0..nx {
                for j in 0..ny {
                    for k in 0..nz {
                        let c = self.cell_idx(i, j, k);
                        if self.class[c] != CLASS_FLUID {
                            continue;
                        }
                        // 六个邻居；越界（网格外边界）与固体同 ⇒ 无通量（不入模板）。
                        let mut ap = 0.0f32;
                        let mut sum = 0.0f32;
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
                                // 固体：Neumann（不入模板）。
                                CLASS_SOLID => {}
                                // 流体：Dirichlet（读旧压力）。
                                CLASS_FLUID => {
                                    ap += 1.0;
                                    sum += self.p[cn];
                                }
                                // 空：Dirichlet 0（不进 sum）。
                                _ => {
                                    ap += 1.0;
                                }
                            }
                        }
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

    /// 面速度投影：`u ← u − (Δt/ρ)(p_r − p_l)/cell`（只施加在"至少一侧是流体"的非固体面）。
    fn project_faces(&mut self, sys: &FluidSystem, dt: f32) {
        let inv = dt / sys.cfg.rest_density / self.cell;
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
    fn g2p(&mut self, sys: &mut FluidSystem, dt: f32) {
        let theta = sys.cfg.flip_ratio.clamp(0.0, 1.0);
        let vmax = sys.cfg.max_speed_frac * sys.h / dt;
        let vmax2 = vmax * vmax;
        let (nx, ny, nz) = (self.dims[0], self.dims[1], self.dims[2]);
        for i in 0..sys.n_fluid {
            let p = sys.pos[i];
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
            let v_flip = sys.vel[i] + (vg - vg0);
            let mut v = v_flip * theta + vg * (1.0 - theta);
            let s2 = v.length_squared();
            if s2 > vmax2 {
                v *= vmax / s2.sqrt();
            }
            sys.vel[i] = v;
            sys.pos[i] += v * dt;
            // `pressures()` = 粒子所在单元压力（Pa）；夹到网格内。
            let ci = i0.min(nx - 1);
            let cj = j0.min(ny - 1);
            let ck = k0.min(nz - 1);
            sys.press[i] = self.p[self.cell_idx(ci, cj, ck)];
        }
    }
}
/// 一个 FLIP 子步（`pbf::substep_of` 的第三族入口）：见模块头八步。
pub(crate) fn substep(sys: &mut FluidSystem, dt: f32, providers: &dyn ProviderColliders) {
    let nf = sys.n_fluid;
    if nf == 0 {
        return; // 边界粒子不单独驱动（同 WCSPH/PBF 档）
    }
    // 每子步重累加反作用（清零归本函数、÷ 子步数归 `step`，C2 口径与 SPH 族一致）。
    for f in sys.bforce[nf..].iter_mut() {
        *f = Vec3::ZERO;
    }
    // 相位 0/1：邻域网格 + SPH 密度（层流黏度相位与 `densities()` 口径；与 PBF 档同）。
    let t = vxl_phys_core::probe::start();
    sys.grid.rebuild(&sys.pos, sys.h);
    sys.phase_us[0] += vxl_phys_core::probe::us(t);
    let t = vxl_phys_core::probe::start();
    sys.density_pass(providers);
    sys.phase_us[1] += vxl_phys_core::probe::us(t);
    // ② 体外力 + CFL（与 PBF 档共用；**只改速度**——位置要等投影后的速度来推）。
    crate::pbf::apply_forces(sys, dt);
    // ③④ 建网格 + 分类。
    let mut sc = FlipScratch::new();
    sc.build(sys, providers);
    // ⑤ P2G + 归一/外推/无通量 + 备份。
    let t = vxl_phys_core::probe::start();
    sc.p2g(sys);
    sc.normalize_and_bc();
    sys.phase_us[3] += vxl_phys_core::probe::us(t);
    // ⑥⑦ 散度 + Jacobi 压力 + 面投影。
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
