//! **层级 Voronoi 细分（多轮碎裂）** —— `VoxelVolume` 的第二个 `impl`。
//!
//! 为什么单独一个文件：`voxel_volume.rs` 受 god 门棘轮（file_lines / max_fn 只准减），
//! 而"多轮 = 多次单轮"的组合逻辑属独立一族 ⇒ 放新文件（只判阈值）。新文件**不用 glob 导入**
//! （`glob-gate` 对新文件零基线）⇒ 下面显式 `use`。
//!
//! **实现口径（别再发明第二套划分）**：每轮都复用 `extract_where`（贪心合并 + 消费格）与
//! `fill_box`（把上一轮的盒**原样回填**再分）——`extract_where` 产出的盒是**格心对齐的 AABB**，
//! `fill_box` 对它是**精确互逆**的（`grid_of` 取下界、`ceil−1` 取上界，两侧边界已核对）。
//! 于是"第 r 轮的块"= "第 r−1 轮某块的格集"，血缘关系不引入新算法。
//!
//! **确定性**：种子走 `seeds_jittered`（格点序），域遍历序与 `extract_where` 扫描序固定，
//! 归属判据 = 格心到种子的平方距离、并列取序号小者（与单轮 `fracture_voronoi` 同款）。

use super::voxel_volume::VoxelVolume;
use vxl_phys_core::Vec3;

impl VoxelVolume {
    /// **层级 Voronoi 细分（多轮）**：第 1 轮在 `[min,max]` 内按 `per_round` 个抖动种子分域；
    /// 之后**每一轮对上一轮的每个块**在其 AABB 内再生成 `per_round` 个种子继续分。
    /// 返回**最细层**的每块盒列表（顺序 = 逐层种子序 ⇒ 确定性）。
    ///
    /// **开销**：`O(rounds × 块数 × 该块 AABB 格数)`；最细层块数上限 = `per_round^rounds`
    /// （**全局预算封顶由调用方负责** —— 这是策略，不塞进几何原语）。
    ///
    /// `jitter ∈ [0,1]` 透传给 `seeds_jittered`。`rounds = 1` 时与单轮
    /// `fracture_voronoi(min,max,seeds_jittered(min,max,per_round,jitter))` **逐块同序**。
    pub fn fracture_voronoi_hierarchical(
        &mut self,
        min: Vec3,
        max: Vec3,
        per_round: u32,
        rounds: u32,
        jitter: f32,
    ) -> Vec<Vec<(Vec3, Vec3)>> {
        if rounds == 0 || per_round == 0 {
            return Vec::new();
        }
        let seeds = Self::seeds_jittered(min, max, per_round as usize, jitter);
        let mut blocks = self.extract_blocks(min, max, &seeds);
        for _ in 1..rounds {
            let mut next: Vec<Vec<(Vec3, Vec3)>> = Vec::new();
            for block in &blocks {
                let Some(&(c0, h0)) = block.first() else {
                    continue;
                };
                // 上一轮已把这些格消费掉 ⇒ 按盒**原样回填**，再在本块 AABB 内继续分。
                let (mut lo, mut hi) = (c0 - h0, c0 + h0);
                for &(c, h) in block {
                    self.fill_box(c - h, c + h);
                    lo = Vec3::new(
                        lo.x.min(c.x - h.x),
                        lo.y.min(c.y - h.y),
                        lo.z.min(c.z - h.z),
                    );
                    hi = Vec3::new(
                        hi.x.max(c.x + h.x),
                        hi.y.max(c.y + h.y),
                        hi.z.max(c.z + h.z),
                    );
                }
                // **必要优化**：每格能再分出的非空块 ≤ 它的体素数（一个体素分不开）⇒
                // 种子数取 `min(per_round, 本格体素数)`。没有它，大曲线（如 per_round=316）
                // 会退化成 `k²` 次 `extract_where` 调用；有了它总调用数 ≤ 域内占据格数。
                let n_seeds = (per_round as usize).min(block_voxels(block, self.step));
                if n_seeds == 0 {
                    continue;
                }
                let seeds = Self::seeds_jittered(lo, hi, n_seeds, jitter);
                next.extend(self.extract_blocks(lo, hi, &seeds));
            }
            blocks = next;
        }
        blocks
    }

    /// 单轮核：在 `[min,max]` 内按 `seeds` 逐种子提取（每格恰属一个种子；并列取序号小者）。
    fn extract_blocks(&mut self, min: Vec3, max: Vec3, seeds: &[Vec3]) -> Vec<Vec<(Vec3, Vec3)>> {
        let (origin, step) = (self.origin, self.step);
        let mut out = Vec::new();
        for (si, &seed) in seeds.iter().enumerate() {
            let boxes = self.extract_where(min, max, |ix, iy, iz| {
                let c =
                    origin + Vec3::new(ix as f32 + 0.5, iy as f32 + 0.5, iz as f32 + 0.5) * step;
                let d_me = (c - seed).length_squared();
                for (sj, &other) in seeds.iter().enumerate() {
                    if sj == si {
                        continue;
                    }
                    let d_o = (c - other).length_squared();
                    if d_o < d_me || (d_o == d_me && sj < si) {
                        return false;
                    }
                }
                true
            });
            if !boxes.is_empty() {
                out.push(boxes);
            }
        }
        out
    }
}

/// 一个块（盒列表）含多少个体素：盒是**体素对齐**的 AABB ⇒ `2h/step` 就是每轴的格数
/// （`round` 只吃浮点表示误差）。
fn block_voxels(block: &[(Vec3, Vec3)], step: f32) -> usize {
    let mut n = 0usize;
    for &(_c, h) in block {
        let kx = ((2.0 * h.x / step).round() as usize).max(1);
        let ky = ((2.0 * h.y / step).round() as usize).max(1);
        let kz = ((2.0 * h.z / step).round() as usize).max(1);
        n += kx * ky * kz;
    }
    n
}
