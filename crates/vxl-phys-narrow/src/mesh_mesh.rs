//! **三角网 × 三角网**（T2 续）—— `mesh_pair` 的**子模块**（`#[path]` 指向同目录本文件）。
//!
//! **为什么拆出来**：`mesh_pair.rs` 受 god 门**文件行数棘轮**（基线 538 行）—— 新代码留在那里
//! 会让它涨到 691 行（超出 `≤10%` 交换窗）。选**子模块**而不是 crate 级新文件，是因为本文件要
//! 用父模块的私有 `point_triangle`：**子模块天然能看见父模块的私有项**；crate 级新文件则要把它
//! 提成 `pub(crate)` 并改 `narrow/lib.rs` 加 `mod` 声明，而那个文件只剩 1 行预算（48 行 / 基线 49）
//! ⇒ 加声明就顶棘轮。
//!
//! **口径 = 双面**（`SURVEY` §8.4.47②）：薄壳**无内外** ⇒ 法线取「**从网面最近点指向顶点**」、
//! 接触距离 = 接触带 `skin`、**双向采样**。⚠️ 与父模块 `hull_vs_mesh` 的**单面**口径
//! （固定环绕法线、反面压入会**推得更深**）不同，这是刻意的：那条口径自己登记过这个边界。
use super::point_triangle;
use crate::{ContactPoint, ContactPoints, DefaultNarrowPhase, Manifold};
use vxl_phys_core::{Quat, Shape, Vec3};

/// **一次"顶点 × 三角"候选**（**双面口径**）：`q` = 三角上的最近点、`d = p − q`，
/// **法线 = 从网面最近点指向顶点**（顶点在哪侧就朝哪侧推）；`sgn` 把方向折到 **a→b** 约定。
///
/// 接触距离 = `band`、`depth = band − ‖d‖`；退化（`‖d‖ ≈ 0`）取环绕法线兜底。
fn mesh_probe(p: Vec3, w: (Vec3, Vec3, Vec3), band: f32, sgn: f32) -> Option<(Vec3, f32, Vec3)> {
    let (n_w, _, q) = point_triangle(p, w.0, w.1, w.2)?;
    let d = p - q;
    let dist = d.length();
    if dist >= band {
        return None;
    }
    let n = if dist > 1e-9 {
        d * (sgn / dist)
    } else {
        n_w * sgn
    };
    Some((n, band - dist, q))
}

impl DefaultNarrowPhase {
    /// **三角网参与的对的统一入口**：两个都是三角网 ⇒ [`Self::mesh_vs_mesh`]（T2 续）；
    /// 否则仍走 [`Self::mesh_pair`]（**原有函数一字未动**）。调用点只改这一行。
    #[allow(clippy::too_many_arguments)] // 两侧体号/形状/位姿 + 出参（与 `mesh_pair` 同形）
    pub(crate) fn mesh_dispatch(
        &mut self,
        a: u32,
        b: u32,
        sa: &Shape,
        sb: &Shape,
        pa: Vec3,
        ra: Quat,
        pb: Vec3,
        rb: Quat,
        out: &mut Vec<Manifold>,
    ) {
        if matches!(*sa, Shape::TriMesh { .. }) && matches!(*sb, Shape::TriMesh { .. }) {
            self.mesh_vs_mesh(a, b, sa, sb, pa, ra, pb, rb, out);
            return;
        }
        self.mesh_pair(a, b, sa, sb, pa, ra, pb, rb, out);
    }

    /// **三角网 × 三角网**（T2 续）：**双面口径** —— 采样**双向**（A 顶点 × B 三角 且
    /// B 顶点 × A 三角；只做一向时"另一侧顶点扎进来"完全没有候选），法线取
    /// "**从网面最近点指向顶点**"（见 [`mesh_probe`]），接触距离 = `skin`（扮演两片半厚之和）。
    /// 流形法线按 **a→b** 定号（与 `mesh_pair` / `hull_vs_mesh` 同约定）。
    ///
    /// **成本**：O(V_a·T_b + V_b·T_a) 暴力（小网无压力；大网要空间加速，属后续片）。
    /// **默认档不动**：不注册成对三角网的场景**逐位不变**（本函数根本不被调用）。
    #[allow(clippy::too_many_arguments)] // 两侧体号/形状/位姿 + 出参（与 `mesh_pair` 同形）
    pub(crate) fn mesh_vs_mesh(
        &mut self,
        a: u32,
        b: u32,
        sa: &Shape,
        sb: &Shape,
        pa: Vec3,
        ra: Quat,
        pb: Vec3,
        rb: Quat,
        out: &mut Vec<Manifold>,
    ) {
        let (Shape::TriMesh { mesh: ma, .. }, Shape::TriMesh { mesh: mb, .. }) = (*sa, *sb) else {
            return;
        };
        if !self.fill_mesh_world(0, a, sa, pa, ra) || !self.fill_mesh_world(1, b, sb, pb, rb) {
            return;
        }
        let na = self.hull_pts[0].len();
        let band = self.skin;
        // 两路扫描：(顶点侧, 面侧, 面仓库, a→b 的符号)。⚠️ `Manifold::normal` 的约定是
        // **从 a 指向 b**（`types.rs` 明写）⇒ 0 路（a 顶点 × b 面）的分离方向是 **−d**（`d` 指
        // 从 b 面到 a 顶点），1 路取反。**首版两路都取了 +1** ⇒ 实测两片被"**推拢**"（+0.01 的
        // 初始间隙被压到 0）—— "符号/约定要查档、别靠直觉" 的又一次。
        let dirs = [(0usize, 1usize, mb, -1.0f32), (1, 0, ma, 1.0)];
        // ① 主导方向 = 最深候选的法线（两路一起）
        let mut dom: Option<(Vec3, f32)> = None;
        for (vside, mside, mesh, sgn) in dirs {
            for vi in 0..self.hull_pts[vside].len() {
                let p = self.hull_pts[vside][vi];
                for ti in 0..self.meshes.tris(mesh).len() {
                    let tri = self.meshes.tris(mesh)[ti];
                    let w = (
                        self.hull_pts[mside][tri[0] as usize],
                        self.hull_pts[mside][tri[1] as usize],
                        self.hull_pts[mside][tri[2] as usize],
                    );
                    let Some((n, depth, _)) = mesh_probe(p, w, band, sgn) else {
                        continue;
                    };
                    if dom.is_none_or(|(_, d)| depth > d) {
                        dom = Some((n, depth));
                    }
                }
            }
        }
        let Some((n_dom, _)) = dom else {
            return;
        };
        // ② 同向候选（`feature` 按方向错开 ⇒ 两路的顶点序各自稳定、不撞号 ⇒ 跨帧可续接 warm）
        self.cand.clear();
        for (vside, mside, mesh, sgn) in dirs {
            let fbase = if vside == 0 { 0 } else { na as u32 };
            for vi in 0..self.hull_pts[vside].len() {
                let p = self.hull_pts[vside][vi];
                for ti in 0..self.meshes.tris(mesh).len() {
                    let tri = self.meshes.tris(mesh)[ti];
                    let w = (
                        self.hull_pts[mside][tri[0] as usize],
                        self.hull_pts[mside][tri[1] as usize],
                        self.hull_pts[mside][tri[2] as usize],
                    );
                    let Some((n, depth, q)) = mesh_probe(p, w, band, sgn) else {
                        continue;
                    };
                    if n.dot(n_dom) > 0.9 {
                        let feature = fbase + vi as u32 + 1;
                        self.cand.push(ContactPoint {
                            point: q,
                            depth,
                            feature,
                        });
                    }
                }
            }
        }
        if self.cand.is_empty() || !self.select_contacts(self.min_point_sep) {
            return;
        }
        out.push(Manifold {
            a,
            b,
            normal: n_dom,
            points: ContactPoints::from_slice(&self.cand),
        });
    }
}
