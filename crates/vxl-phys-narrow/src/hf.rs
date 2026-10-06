//! hf：高度场族采样；接受带 = `skin + ws.inflate`（由 `heightfield_pair` 按 `velocity_band` 每对置位，#6）。
use super::*;

impl DefaultNarrowPhase {
    /// 球-高度场：采样 = 投影点双线性 + 所在格 3×3 邻域节点，取最深 ≤4。
    pub(crate) fn sphere_heightfield(
        &mut self,
        center: Vec3,
        radius: f32,
        hf: &HeightField,
    ) -> bool {
        self.ws.cand.clear();
        let r2 = radius * radius;
        let try_point = |cand: &mut Vec<ContactPoint>, px: f32, pz: f32, h: f32| {
            let dx = px - center.x;
            let dz = pz - center.z;
            let d2 = dx * dx + dz * dz;
            if d2 >= r2 {
                return;
            }
            let sy = center.y - (r2 - d2).sqrt();
            // skin 预期接触：允许深度小负值（speculative margin，§4.3），静止时不抖。
            if sy < h + self.skin + self.ws.inflate {
                cand.push(ContactPoint {
                    point: Vec3::new(px, h, pz),
                    depth: h - sy,
                    feature: 0,
                });
            }
        };
        // 1) 投影点（双线性高度；覆盖球心位于格心/格间的一切情形）。
        if let Some((h, _)) = hf.sample(center.x, center.z) {
            try_point(&mut self.ws.cand, center.x, center.z, h);
        }
        // 2) 所在格 + 邻域 3×3 网格节点。
        let ix0 = ((center.x - hf.origin_x) / hf.spacing).floor() as i64;
        let iz0 = ((center.z - hf.origin_z) / hf.spacing).floor() as i64;
        for dix in -1i64..=1 {
            for diz in -1i64..=1 {
                let ix = ix0 + dix;
                let iz = iz0 + diz;
                if ix < 0 || iz < 0 || ix >= hf.nx as i64 || iz >= hf.nz as i64 {
                    continue;
                }
                let h = hf.height_ix(ix as u32, iz as u32);
                let px = hf.origin_x + ix as f32 * hf.spacing;
                let pz = hf.origin_z + iz as f32 * hf.spacing;
                try_point(&mut self.ws.cand, px, pz, h);
            }
        }
        if self.ws.cand.is_empty() {
            return false;
        }
        self.select_contacts(self.min_point_sep)
    }

    /// 多面体顶点-高度场：逐顶点采样（skin 预期接触），最深 ≤4。
    pub(crate) fn poly_heightfield(
        &mut self,
        poly_idx: usize,
        pos: Vec3,
        rot: Quat,
        hf: &HeightField,
    ) -> bool {
        self.ws.poly_a.fill(&self.ws.polys[poly_idx], pos, rot);
        self.ws.cand.clear();
        for (idx, &v) in self.ws.poly_a.verts.iter().enumerate() {
            if let Some((h, _)) = hf.sample(v.x, v.z) {
                let depth = h - v.y;
                if depth > -self.skin - self.ws.inflate {
                    self.ws.cand.push(ContactPoint {
                        point: Vec3::new(v.x, h, v.z),
                        depth,
                        // 特征 = 盒顶点序号（跨帧稳定；盒侧不置侧位）。
                        feature: idx as u32,
                    });
                }
            }
        }
        if self.ws.cand.is_empty() {
            return false;
        }
        self.select_contacts(self.min_point_sep)
    }

    /// 胶囊 × 高度场：**沿中心线取 N 个样本**，每个样本按半径 r 的球处理
    /// （候选点 `depth = h − y + r`、接触点 `(x, h, z)`、法线取地形法线）。
    /// 覆盖：两端 + 等分中间点（平躺胶囊靠两端、竖直靠底端；`select_contacts` 取最深 ≤4）。
    /// `feature = 样本序号 + 1`（等分序稳定 ⇒ 跨帧可续接）。
    pub(crate) fn capsule_heightfield(
        &mut self,
        seg_a: Vec3,
        seg_b: Vec3,
        radius: f32,
        hf: &HeightField,
    ) -> bool {
        const SAMPLES: u32 = 5;
        self.ws.cand.clear();
        let denom = (SAMPLES - 1) as f32;
        for k in 0..SAMPLES {
            let t = k as f32 / denom;
            let s = seg_a + (seg_b - seg_a) * t;
            if let Some((hgt, _)) = hf.sample(s.x, s.z) {
                let depth = hgt - s.y + radius;
                if depth > -self.skin - self.ws.inflate {
                    self.ws.cand.push(ContactPoint {
                        point: Vec3::new(s.x, hgt, s.z),
                        depth,
                        feature: k + 1,
                    });
                }
            }
        }
        if self.ws.cand.is_empty() {
            return false;
        }
        self.select_contacts(self.min_point_sep)
    }
}

// ⚠️ **本文件曾有一份 `hull_heightfield`，2026-10-05 删除（死代码）**：
// 外壳 × 高度场在派发顺序上先落到 `support.rs::hull_pair`（`pair_shaped.rs` 里 hull 分支
// **排在** `heightfield_pair` 之前），后者的 L1 分支处理该组合并 `return`
// ⇒ `heightfield_pair` 的 `Shape::ConvexHull` 臂**永远不可达**。
// 实测证据（当时做法）：把本函数首行改成 `return false` 后，`vxl-phys-narrow` 全部测试与
// `vxl-phys --lib hull` 三个测试（含 `hull_on_heightfield` / `hull_rests_on_heightfield`）
// **仍全绿** ⇒ 它没有任何调用路径。两份实现的差别只有 `feature`（这里 +1、live 那份从 0）
// ⇒ 留着只会让人改错那一份（本仓"台账前提过期"的又一例）。
