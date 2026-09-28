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
use crate::{ContactPoint, ContactPoints, DefaultNarrowPhase, Manifold};
use vxl_phys_core::{Quat, Shape, Vec3};

// **面侧三角的空间哈希 + 候选查询**在子模块 `mesh_grid`（拆出去的理由同 `mesh_pair.rs` → 本文件：
// **文件行数棘轮**）。
#[path = "mesh_grid.rs"]
mod mesh_grid;
use mesh_grid::{mesh_probe, TriGrid};

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
        // **空间哈希**（两路各一张：0 路 = A 顶点 × **B 的三角** ⇒ 查 `grid_b`；1 路相反）。
        // 建格 O(T)、查询 O(V) ⇒ 把 O(V·T) 降到 O(V+T+候选)；**候选序列与暴力同序** ⇒ 读数逐位不变。
        let grid_b = TriGrid::build(&self.hull_pts[1], self.meshes.tris(mb), band);
        let grid_a = TriGrid::build(&self.hull_pts[0], self.meshes.tris(ma), band);
        // 两路扫描：(顶点侧, 面侧, 面仓库, 面侧的格子, a→b 的符号)。⚠️ `Manifold::normal` 的约定是
        // **从 a 指向 b**（`types.rs` 明写）⇒ 0 路（a 顶点 × b 面）的分离方向是 **−d**（`d` 指
        // 从 b 面到 a 顶点），1 路取反。**首版两路都取了 +1** ⇒ 实测两片被"**推拢**"（+0.01 的
        // 初始间隙被压到 0）—— "符号/约定要查档、别靠直觉" 的又一次。
        let dirs = [
            (0usize, 1usize, mb, &grid_b, -1.0f32),
            (1, 0, ma, &grid_a, 1.0),
        ];
        // ① 主导方向 = 最深候选的法线（两路一起）
        let mut dom: Option<(Vec3, f32)> = None;
        for (vside, mside, mesh, grid, sgn) in dirs {
            for vi in 0..self.hull_pts[vside].len() {
                let p = self.hull_pts[vside][vi];
                for &tj in grid.query(p) {
                    let tri = self.meshes.tris(mesh)[tj as usize];
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
        // ② 同向候选（抽出成 `mesh_collect`：本函数是本地最长函数）
        self.cand.clear();
        for (vside, mside, mesh, grid, sgn) in dirs {
            let fbase = if vside == 0 { 0 } else { na as u32 };
            self.mesh_collect(vside, mside, mesh, grid, sgn, band, n_dom, fbase);
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

    /// **收集与主导法线同向的候选**（从 `mesh_vs_mesh` 抽出：那里是本地最长函数）。
    ///
    /// `feature = fbase + 顶点号 + 1`：两路的**顶点序各自稳定**、`fbase` 按方向错开 ⇒ 不撞号
    /// ⇒ 跨帧可续接 warm（顶点序稳定是前提，见 `mesh_pair.rs` 的口径注）。
    #[allow(clippy::too_many_arguments)] // 接触包：方向包（顶点侧/面侧/仓库/格子/符号）+ 带 + 法线 + 特征基址
    fn mesh_collect(
        &mut self,
        vside: usize,
        mside: usize,
        mesh: u32,
        grid: &TriGrid,
        sgn: f32,
        band: f32,
        n_dom: Vec3,
        fbase: u32,
    ) {
        for vi in 0..self.hull_pts[vside].len() {
            let p = self.hull_pts[vside][vi];
            for &tj in grid.query(p) {
                let tri = self.meshes.tris(mesh)[tj as usize];
                let w = (
                    self.hull_pts[mside][tri[0] as usize],
                    self.hull_pts[mside][tri[1] as usize],
                    self.hull_pts[mside][tri[2] as usize],
                );
                let Some((n, depth, q)) = mesh_probe(p, w, band, sgn) else {
                    continue;
                };
                if n.dot(n_dom) > 0.9 {
                    self.cand.push(ContactPoint {
                        point: q,
                        depth,
                        feature: fbase + vi as u32 + 1,
                    });
                }
            }
        }
    }
}
