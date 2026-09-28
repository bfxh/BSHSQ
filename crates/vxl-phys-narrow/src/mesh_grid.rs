//! **面侧三角的空间哈希 + 一次候选查询**（T2 续的加速片）—— `mesh_mesh` 的**子模块**。
//!
//! **为什么拆出来**：`mesh_mesh.rs` 同样受 god 门**文件行数棘轮**（只准减）—— 这块 ~100 行留在
//! 那里会让它超窗。拆成子模块后两边各自在阈内。
//!
//! **为什么需要它**：双面口径的采样本来是暴力 O(V·T) ⇒ 两片 `n×n` 的网是 **O(n⁴)**。实测标度
//! （release，每 tick 墙钟；探针跑完已删、数字留档在 `SURVEY` §8.4.49）：
//! `n=8 → 0.465 ms`、`n=12 → 2.20`、`n=16 → 6.42`、`n=20 → 13.99`（相对 n=8：4.7× / 13.8× /
//! 30.1× ⇒ 与四次方吻合）；真实规模 `n=32` 外推 ≈ **90 ms/tick** ⇒ 不可用。
//! 加本结构后同一条曲线：`0.374 / 0.958 / 1.93 / 3.60`（相对 2.56× / 5.16× / 9.64×）⇒
//! **n=20 时 3.89×**、且指数从 4 降到 ~2。
use super::super::point_triangle;
use std::collections::BTreeMap;
use vxl_phys_core::Vec3;

/// **一次"顶点 × 三角"候选**（**双面口径**）：`q` = 三角上的最近点、`d = p − q`，
/// **法线 = 从网面最近点指向顶点**（顶点在哪侧就朝哪侧推）；`sgn` 把方向折到 **a→b** 约定
/// （`Manifold::normal` = "从 a 指向 b"，见 `types.rs`）。接触距离 = `band`、`depth = band − ‖d‖`；
/// 退化（`‖d‖ ≈ 0`）取环绕法线兜底。
pub(super) fn mesh_probe(
    p: Vec3,
    w: (Vec3, Vec3, Vec3),
    band: f32,
    sgn: f32,
) -> Option<(Vec3, f32, Vec3)> {
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

/// 位置 → 格子（`inv` = 格子边长的倒数；`floor` + 饱和转换 ⇒ **确定性**）。
fn cell_key(p: Vec3, inv: f32) -> (i32, i32, i32) {
    (
        (p.x * inv).floor() as i32,
        (p.y * inv).floor() as i32,
        (p.z * inv).floor() as i32,
    )
}

/// 三角的 AABB（`(lo, hi)`）。
fn tri_bounds(a: Vec3, b: Vec3, c: Vec3) -> (Vec3, Vec3) {
    (
        Vec3::new(
            a.x.min(b.x).min(c.x),
            a.y.min(b.y).min(c.y),
            a.z.min(b.z).min(c.z),
        ),
        Vec3::new(
            a.x.max(b.x).max(c.x),
            a.y.max(b.y).max(c.y),
            a.z.max(b.z).max(c.z),
        ),
    )
}

/// **面侧三角的空间哈希**。
///
/// **完备性（不漏候选）**：三角按**外扩 `band` 的 AABB** 入格 ⇒ 若某顶点到该三角的距离 < `band`，
/// 则顶点落在外扩 AABB 内 ⇒ **顶点所在的格必在三角占用的格集里** ⇒ 只查顶点自己那一格即可 ✓
///
/// **确定性（保证读数逐位不变）**：建格按三角**升序**插入 ⇒ 桶内序 = 三角升序；查询按顶点升序
/// ⇒ 每格的候选序列与**暴力扫描的同序子序列逐项相同**（被空间剔除的那些本来就不会产生候选）
/// ⇒ 流形与判据读数**逐位不变**（本片实测：4 条判据读数全同，② 仍 `0.00e0`）。
///
/// **格子边长** = `max(band, 该网平均三角 AABB 最长边)`：取平均而非最大 ⇒ 常规网格下每个三角
/// 落进 ~8 格、顶点只查 1 格（O(T+V)）；**离群大三角**会落进更多格（退化网格的代价，如实登记）。
///
/// ⚠️ **本结构是每次调用局部建的**（在 `mesh_vs_mesh` 里）—— `DefaultNarrowPhase` 的成员棘轮
/// 没位子放常驻缓冲（`phase.rs` 已在豁免清单上且只准减）；代价是"每对每子步一次分配"，与它
/// 换掉的 O(n⁴) 相比可忽略。**将来腾出成员时应收进 `DefaultNarrowPhase` 复用。**
pub(super) struct TriGrid {
    cells: BTreeMap<(i32, i32, i32), Vec<u32>>,
    inv: f32,
}

impl TriGrid {
    pub(super) fn build(pts: &[Vec3], tris: &[[u32; 3]], band: f32) -> Self {
        // 平均三角 AABB 最长边（一次 O(T)）
        let mut mean = 0.0f32;
        for t in tris {
            let (lo, hi) = tri_bounds(pts[t[0] as usize], pts[t[1] as usize], pts[t[2] as usize]);
            let e = hi - lo;
            mean += e.x.max(e.y).max(e.z);
        }
        let cell_len = if tris.is_empty() {
            band
        } else {
            (mean / tris.len() as f32).max(band)
        };
        let inv = 1.0 / cell_len.max(1e-6);
        let mut cells: BTreeMap<(i32, i32, i32), Vec<u32>> = BTreeMap::new();
        for (ti, t) in tris.iter().enumerate() {
            let (lo, hi) = tri_bounds(pts[t[0] as usize], pts[t[1] as usize], pts[t[2] as usize]);
            let pad = Vec3::new(band, band, band); // 外扩 band ⇒ 只查顶点自己那一格即完备
            let (i0, j0, k0) = cell_key(lo - pad, inv);
            let (i1, j1, k1) = cell_key(hi + pad, inv);
            for i in i0..=i1 {
                for j in j0..=j1 {
                    for k in k0..=k1 {
                        cells.entry((i, j, k)).or_default().push(ti as u32);
                    }
                }
            }
        }
        Self { cells, inv }
    }

    /// 该顶点**唯一**要查的那一格（桶内序 = 三角升序 ⇒ 与暴力扫描同序）。
    pub(super) fn query(&self, p: Vec3) -> &[u32] {
        match self.cells.get(&cell_key(p, self.inv)) {
            Some(v) => v.as_slice(),
            None => &[],
        }
    }
}
