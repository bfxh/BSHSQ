//! **自动认边（A4 的兜底路线）**：从三角网格里认出**边界棱**与**凸棱**，直接喂给 `edges`。
//!
//! 设计文档 §8 把"自动认边"列为**兜底**（首选仍是作者显式标棱）：理由是凸凹语义依赖"外法向一致
//! 的绕序"。这条假设写在[`EdgeDetect`]的文档里 —— 模型的三角要**朝外**（与 `TriMesh` 那类建模面
//! 同口径）。判据 `tests/a4_auto_edges.rs` 里有一个盒子（12 条凸棱）来钉死这条假设。
//!
//! 认出来的棱可以直接当 [`AcousticGeometry::edges`](crate::AcousticGeometry::edges) 用；
//! 顺序按**顶点对字典序**（不是哈希序）⇒ 逐位可复现。

use crate::{AcousticGeometry, Edge, BANDS};
use vxl_phys::Vec3;

/// 自动认边的筛选条件。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EdgeDetect {
    /// **凸**二面角至少这么大（度）才认成衍射棱（共面与微折会被滤掉）。
    pub min_wedge_deg: f32,
    /// 认出来的棱统一用的逐带绕射系数。
    pub coeff: [f32; BANDS],
}

impl Default for EdgeDetect {
    fn default() -> Self {
        Self {
            min_wedge_deg: 20.0,
            coeff: [1.0; BANDS],
        }
    }
}

/// 一条合法三角：顶点索引、三个顶点、面积向量（= 法向 × 2A，方向按绕序）。
struct Face {
    idx: [u32; 3],
    v: [Vec3; 3],
    cross: Vec3,
}

impl AcousticGeometry<'_> {
    /// **自动认边**：把网格里的**边界棱**（只被一个三角用：刀刃）与**凸棱**（二面角 ≥
    /// [`EdgeDetect::min_wedge_deg`]）认成衍射棱。
    ///
    /// 跳过：坏索引 / 退化三角 / 共面或微折的共享棱 / **凹**棱 / 相邻面多于两个的棱（非流形 ——
    /// 不猜语义）。
    ///
    /// ⚠️ 前提：**三角绕序朝外一致**（凸凹判据是"另一个三角的第三个顶点在第一个三角平面外侧"
    /// ⇒ 只有外向绕序下才等价于几何上的凸）。
    pub fn detect_edges(&self, cfg: &EdgeDetect) -> Vec<Edge> {
        let n = self.verts.len();
        let cos_min = cfg.min_wedge_deg.to_radians().cos();
        let mut faces: Vec<Face> = Vec::new();
        for tri in self.tris.iter() {
            if tri.iter().any(|&k| k as usize >= n) {
                continue;
            }
            let v = [
                self.verts[tri[0] as usize],
                self.verts[tri[1] as usize],
                self.verts[tri[2] as usize],
            ];
            let cross = (v[1] - v[0]).cross(v[2] - v[0]);
            if cross.length() <= 1e-12 {
                continue; // 退化面：不参与认边
            }
            faces.push(Face {
                idx: *tri,
                v,
                cross,
            });
        }
        // 邻接：把每条棱的 (键, 面号) 收起来排序 —— 同键聚在一起，且面序确定（无哈希）。
        let mut pairs: Vec<((u32, u32), u32)> = Vec::new();
        for (fi, f) in faces.iter().enumerate() {
            for (i, j) in [(0usize, 1usize), (1, 2), (2, 0)] {
                let (k0, k1) = (f.idx[i], f.idx[j]);
                pairs.push(((k0.min(k1), k0.max(k1)), fi as u32));
            }
        }
        pairs.sort();
        let mut out: Vec<Edge> = Vec::new();
        let mut i = 0usize;
        while i < pairs.len() {
            let key = pairs[i].0;
            let mut j = i;
            while j < pairs.len() && pairs[j].0 == key {
                j += 1;
            }
            let (a, b) = (self.verts[key.0 as usize], self.verts[key.1 as usize]);
            let group = &pairs[i..j];
            if group.len() == 1 {
                // 边界棱：只被一个三角用 ⇒ 刀刃，绕它一定"没有第二面挡着"。
                out.push(Edge {
                    a,
                    b,
                    coeff: cfg.coeff,
                    wedge: None,
                });
            } else if group.len() == 2 {
                let (f1, f2) = (group[0].1 as usize, group[1].1 as usize);
                let (n1, n2) = (faces[f1].cross, faces[f2].cross);
                let wedge = n1.dot(n2) <= cos_min * n1.length() * n2.length();
                let c2 = faces[f2]
                    .v
                    .iter()
                    .copied()
                    .find(|p| (*p - a).length() > 1e-9 && (*p - b).length() > 1e-9)
                    .unwrap_or(a);
                let convex = n1.dot(c2 - a) < 0.0;
                if wedge && convex {
                    out.push(Edge {
                        a,
                        b,
                        coeff: cfg.coeff,
                        wedge: None,
                    });
                }
            }
            i = j;
        }
        out
    }
}
