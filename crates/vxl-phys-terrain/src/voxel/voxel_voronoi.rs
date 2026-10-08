//! **单轮 Voronoi 划分的唯一实现**（2026-10-09；"快速销毁"的算法底座）。
//!
//! 此前"单轮 `fracture_voronoi`"与"层级细分的单轮核 `voxel_hier::extract_blocks`"各写了一份
//! **逐种子 × 逐格 × 再逐种子** 的归属判据 ⇒ 复杂度 `O(格数 × 种子数²)`（`per_round = 316` 时
//! 是 316² 倍的距离运算）。本文件把它收敛成**一处**：先做**一遍归属图**（每格只算一次最近种子），
//! 再按归属做贪心合并。
//!
//! **判据（与旧实现逐位一致）**：归属 = 格心到种子的**平方距离**最小、并列取**序号小者**；
//! `extract_where` 的扫描序与 +X/+Y/+Z 贪心扩展方向不变 ⇒ 盒列表逐位相同（单测用**朴素实现**
//! 当陪测，端到端证据 = `m3_collapse` 金样哈希不变 —— 它覆盖整条破坏管线）。
//!
//! **复杂度**：归属 `O(格数 × 种子数)` + 每种子一次区域扫描（谓词 = 一次查表）`O(格数 × 种子数)`
//! ⇒ 合计 `O(格数 × 种子数)`（旧实现含一个额外的 `× 种子数` 因子）。
//!
//! 新文件不用 glob 导入（`glob-gate` 对新文件零基线）⇒ 下面显式 `use`。
use super::voxel_volume::VoxelVolume;
use vxl_phys_core::Vec3;

/// 区域内的**归属图**：每格记录最近种子的序号（`u32::MAX` = 空/无归属）。
struct OwnerMap {
    lo: (i32, i32, i32),
    dims: (usize, usize, usize),
    data: Vec<u32>,
}

impl OwnerMap {
    /// 查归属（越界 = 无归属；`extract_where` 只访问范围内格，越界分支是防御）。
    #[inline]
    fn at(&self, ix: i32, iy: i32, iz: i32) -> u32 {
        let (dx, dy, dz) = (ix - self.lo.0, iy - self.lo.1, iz - self.lo.2);
        if dx < 0 || dy < 0 || dz < 0 {
            return u32::MAX;
        }
        let (dx, dy, dz) = (dx as usize, dy as usize, dz as usize);
        if dx >= self.dims.0 || dy >= self.dims.1 || dz >= self.dims.2 {
            return u32::MAX;
        }
        self.data[(dx * self.dims.1 + dy) * self.dims.2 + dz]
    }
}

impl VoxelVolume {
    /// **单轮 Voronoi 划分（唯一实现）**：在 `[min,max]` 内按 `seeds` 分域、贪心合并成盒并消费格。
    /// 返回 `(种子序号, 盒列表)`，**只含非空种子、顺序 = 种子序**（⇒ 确定性，与旧实现同序）。
    pub(crate) fn voronoi_blocks(
        &mut self,
        min: Vec3,
        max: Vec3,
        seeds: &[Vec3],
    ) -> Vec<(usize, Vec<(Vec3, Vec3)>)> {
        if seeds.is_empty() {
            return Vec::new();
        }
        let owner = self.owner_map(min, max, seeds);
        let mut out = Vec::new();
        for (si, _) in seeds.iter().enumerate() {
            let boxes =
                self.extract_where(min, max, |ix, iy, iz| owner.at(ix, iy, iz) == si as u32);
            if !boxes.is_empty() {
                out.push((si, boxes));
            }
        }
        out
    }

    /// 一遍归属：占据格取 `(d², 序号)` 最小的种子（与旧判据**逐字同规则**）。
    fn owner_map(&self, min: Vec3, max: Vec3, seeds: &[Vec3]) -> OwnerMap {
        let lo = self.grid_of(min);
        let hi = self.grid_of(max);
        let (nx, ny, nz) = self.dims();
        let x0 = lo.0.max(0);
        let y0 = lo.1.max(0);
        let z0 = lo.2.max(0);
        let x1 = hi.0.min(nx as i32 - 1);
        let y1 = hi.1.min(ny as i32 - 1);
        let z1 = hi.2.min(nz as i32 - 1);
        let dims = (
            (x1 - x0 + 1).max(0) as usize,
            (y1 - y0 + 1).max(0) as usize,
            (z1 - z0 + 1).max(0) as usize,
        );
        let mut data = vec![u32::MAX; dims.0 * dims.1 * dims.2];
        for iz in z0..=z1 {
            for iy in y0..=y1 {
                for ix in x0..=x1 {
                    if !self.get(ix as u32, iy as u32, iz as u32) {
                        continue;
                    }
                    let c = self.grid_center(ix as u32, iy as u32, iz as u32);
                    let mut best = (f32::INFINITY, u32::MAX);
                    for (sj, &seed) in seeds.iter().enumerate() {
                        let d = (c - seed).length_squared();
                        let sj = sj as u32;
                        if d < best.0 || (d == best.0 && sj < best.1) {
                            best = (d, sj);
                        }
                    }
                    let at = ((ix - x0) as usize * dims.1 + (iy - y0) as usize) * dims.2
                        + (iz - z0) as usize;
                    data[at] = best.1;
                }
            }
        }
        OwnerMap {
            lo: (x0, y0, z0),
            dims,
            data,
        }
    }
}
