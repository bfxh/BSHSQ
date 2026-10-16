//! FLIP **网格层**：建网格、分类、索引与三线性模板（`flip.rs` 的表示半）。
//!
//! 拆出来的原因与口径都写在 `flip.rs` 模块头（索引约定、单元/交错面布局、已知简化）；
//! 这里只放"网格怎么建、单元怎么分类、坐标怎么索引"三件事，求解器（P2G/压力/投影/G2P）
//! 与桥接（`substep`）留在 `flip.rs`。这样两个文件都远在 `god_gate` 的 800 行硬线之下，
//! 文档也不必为了行数被砍。
//!
//! 可见性：`flip.rs` 要调的（`new`/`build`/`cell_idx`/`u_idx`/`v_idx`/`w_idx`/`coord`）
//! 标 `pub(super)`；只在层内用的（`center`/`classify`）保持私有；两条自由函数
//! （`cell_axis`/`weight_of`）是 P2G/G2P 与分类共用的三线性模板。
use super::{
    FlipScratch, CLASS_AIR, CLASS_FLUID, CLASS_SOLID, MARGIN_CELLS, MAX_CELLS, SOLID_PROBE_FRAC,
};
use crate::{FluidSystem, ProviderColliders, Vec3};
use vxl_phys_core::interop::InteropContact;

impl FlipScratch {
    /// 建一份空缓冲。FLIP 档由 `step` 每 **tick** 建一份（活过本 tick 的各子步 => 压力场
    /// 热启动；见 `flip.rs` 模块头"热启动"与 `fluid_step.rs`），故这里也要能被那一层看见。
    pub(crate) fn new() -> Self {
        Self {
            origin: Vec3::ZERO,
            cell: 1.0,
            dims: [0, 0, 0],
            n_cells: 0,
            class: Vec::new(),
            weight: Vec::new(),
            u: Vec::new(),
            v: Vec::new(),
            w: Vec::new(),
            wu: Vec::new(),
            wv: Vec::new(),
            ww: Vec::new(),
            u0: Vec::new(),
            v0: Vec::new(),
            w0: Vec::new(),
            p: Vec::new(),
            p_tmp: Vec::new(),
            div: Vec::new(),
            cg_r: Vec::new(),
            cg_d: Vec::new(),
            cg_q: Vec::new(),
            p_prev: Vec::new(),
            warm_key: None,
        }
    }

    #[inline]
    pub(super) fn cell_idx(&self, i: usize, j: usize, k: usize) -> usize {
        (i * self.dims[1] + j) * self.dims[2] + k
    }

    #[inline]
    pub(super) fn u_idx(&self, i: usize, j: usize, k: usize) -> usize {
        (i * self.dims[1] + j) * self.dims[2] + k
    }

    #[inline]
    pub(super) fn v_idx(&self, i: usize, j: usize, k: usize) -> usize {
        (i * (self.dims[1] + 1) + j) * self.dims[2] + k
    }

    #[inline]
    pub(super) fn w_idx(&self, i: usize, j: usize, k: usize) -> usize {
        (i * self.dims[1] + j) * (self.dims[2] + 1) + k
    }

    /// 单元中心世界坐标。
    #[inline]
    fn center(&self, i: usize, j: usize, k: usize) -> Vec3 {
        let half = 0.5 * self.cell;
        Vec3::new(
            self.origin.x + i as f32 * self.cell + half,
            self.origin.y + j as f32 * self.cell + half,
            self.origin.z + k as f32 * self.cell + half,
        )
    }

    /// 世界坐标 → 单元浮点坐标（可能越界；调用方自己判）。
    #[inline]
    pub(super) fn coord(&self, x: Vec3) -> Vec3 {
        Vec3::new(
            (x.x - self.origin.x) / self.cell,
            (x.y - self.origin.y) / self.cell,
            (x.z - self.origin.z) / self.cell,
        )
    }

    /// 建网格：流体 AABB 外扩 → 单元边长/维度 → 缓冲分配 → 分类。
    pub(super) fn build(&mut self, sys: &FluidSystem, providers: &dyn ProviderColliders) {
        let mut mn = Vec3::new(f32::INFINITY, f32::INFINITY, f32::INFINITY);
        let mut mx = Vec3::new(f32::NEG_INFINITY, f32::NEG_INFINITY, f32::NEG_INFINITY);
        for p in &sys.pos[..sys.n_fluid] {
            mn = Vec3::new(mn.x.min(p.x), mn.y.min(p.y), mn.z.min(p.z));
            mx = Vec3::new(mx.x.max(p.x), mx.y.max(p.y), mx.z.max(p.z));
        }
        // 空集/NaN 兜底：`!(mn ≤ mx)` 对 NaN 也为真（NaN 的 AABB 会一路带出病态维度）；
        // 用 `partial_cmp` 写等价判据是为了避开 `clippy::neg_cmp_op_on_partial_ord`
        // （它建议的 `mn > mx` 在 NaN 下语义不同，会漏掉这条兜底）。
        if !matches!(mn.x.partial_cmp(&mx.x), Some(core::cmp::Ordering::Less)) {
            // 给一个 h 见方的单格网格（不该发生，`substep` 已挡 n_fluid = 0）。
            mn = Vec3::ZERO;
            mx = Vec3::new(sys.h, sys.h, sys.h);
        }
        // 单元边长 ≈ 粒子间距（经典 FLIP：cell ≈ 粒径）；间距未设时退回 h/2。
        let base = if sys.spacing > 0.0 {
            sys.spacing
        } else {
            0.5 * sys.h
        };
        let mut cell = base.max(1e-6);
        let mut dims = [1usize; 3];
        // 超 `MAX_CELLS` 就整体放大（单元数 ~ cell⁻³ ⇒ 一次放大 1.5 倍通常够）。
        for _ in 0..64 {
            let m = cell * MARGIN_CELLS;
            let e = Vec3::new(
                (mx.x - mn.x) + 2.0 * m,
                (mx.y - mn.y) + 2.0 * m,
                (mx.z - mn.z) + 2.0 * m,
            );
            dims = [
                (e.x / cell).ceil() as usize + 1,
                (e.y / cell).ceil() as usize + 1,
                (e.z / cell).ceil() as usize + 1,
            ];
            if dims[0] * dims[1] * dims[2] <= MAX_CELLS {
                break;
            }
            cell *= 1.5;
        }
        self.origin = Vec3::new(
            mn.x - cell * MARGIN_CELLS,
            mn.y - cell * MARGIN_CELLS,
            mn.z - cell * MARGIN_CELLS,
        );
        self.cell = cell;
        self.dims = dims;
        self.n_cells = dims[0] * dims[1] * dims[2];
        let (nu, nv, nw) = (
            (dims[0] + 1) * dims[1] * dims[2],
            dims[0] * (dims[1] + 1) * dims[2],
            dims[0] * dims[1] * (dims[2] + 1),
        );
        self.class.clear();
        self.class.resize(self.n_cells, CLASS_AIR);
        self.weight.clear();
        self.weight.resize(self.n_cells, 0.0);
        self.u.clear();
        self.u.resize(nu, 0.0);
        self.v.clear();
        self.v.resize(nv, 0.0);
        self.w.clear();
        self.w.resize(nw, 0.0);
        self.wu.clear();
        self.wu.resize(nu, 0.0);
        self.wv.clear();
        self.wv.resize(nv, 0.0);
        self.ww.clear();
        self.ww.resize(nw, 0.0);
        self.u0.clear();
        self.u0.resize(nu, 0.0);
        self.v0.clear();
        self.v0.resize(nv, 0.0);
        self.w0.clear();
        self.w0.resize(nw, 0.0);
        self.p.clear();
        self.p.resize(self.n_cells, 0.0);
        self.p_tmp.clear();
        self.p_tmp.resize(self.n_cells, 0.0);
        self.div.clear();
        self.div.resize(self.n_cells, 0.0);
        self.classify(sys, providers);
    }

    /// 分类：固体（内点判据）→ 流体（粒子权重和 > 0）；**固优先**。
    fn classify(&mut self, sys: &FluidSystem, providers: &dyn ProviderColliders) {
        let probe = self.cell * SOLID_PROBE_FRAC;
        let mut out: Vec<InteropContact> = Vec::new();
        let (nx, ny, nz) = (self.dims[0], self.dims[1], self.dims[2]);
        // ① 固体：只在"存在边界"时才查（无边界 ⇒ 全空/流，零开销）。
        if !sys.boundaries.is_empty() {
            for i in 0..nx {
                for j in 0..ny {
                    for k in 0..nz {
                        let c = self.center(i, j, k);
                        let mut solid = false;
                        for &bid in &sys.boundaries {
                            out.clear();
                            if providers.contacts_point_boundary(bid, c, probe, &mut out) {
                                for ct in &out {
                                    if ct.depth > probe {
                                        solid = true;
                                    }
                                }
                            }
                        }
                        if solid {
                            let ci = self.cell_idx(i, j, k);
                            self.class[ci] = CLASS_SOLID;
                        }
                    }
                }
            }
        }
        // ② 流体：三线性权重散射（与 P2G 同一模板）⇒ 权重和 > 0 的**非固体**单元是流体。
        for i in 0..sys.n_fluid {
            let g = self.coord(sys.pos[i]);
            let (i0, ux) = cell_axis(g.x);
            let (j0, uy) = cell_axis(g.y);
            let (k0, uz) = cell_axis(g.z);
            for di in 0..2usize {
                for dj in 0..2usize {
                    for dk in 0..2usize {
                        let ci = i0 + di;
                        let cj = j0 + dj;
                        let ck = k0 + dk;
                        if ci >= nx || cj >= ny || ck >= nz {
                            continue;
                        }
                        let w = weight_of(di, ux) * weight_of(dj, uy) * weight_of(dk, uz);
                        let idx = self.cell_idx(ci, cj, ck);
                        if self.class[idx] != CLASS_SOLID {
                            self.weight[idx] += w;
                        }
                    }
                }
            }
        }
        for c in 0..self.n_cells {
            if self.class[c] != CLASS_SOLID && self.weight[c] > 0.0 {
                self.class[c] = CLASS_FLUID;
            }
        }
    }
}

/// 轴上的单元索引 + 小数权重（`g` = 单元浮点坐标）。
#[inline]
pub(super) fn cell_axis(g: f32) -> (usize, f32) {
    let fl = g.floor();
    let u = g - fl;
    // 负坐标/NaN 兜底：夹到 0（越界单元由调用方 `>= dims` 检查挡掉）。
    let i = if fl >= 0.0 { fl as usize } else { 0 };
    (i, if u.is_finite() { u } else { 0.0 })
}

/// 三线性权重：`di = 0` 取 `1 − u`，`di = 1` 取 `u`。
#[inline]
pub(super) fn weight_of(d: usize, u: f32) -> f32 {
    if d == 0 {
        1.0 - u
    } else {
        u
    }
}
