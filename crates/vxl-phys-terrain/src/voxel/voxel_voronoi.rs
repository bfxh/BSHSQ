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
//! **复杂度**：归属 `O(格数 × 种子数)` + 每种子只在**保守 AABB** 内扫描（见 `seed_box`）+
//! 裁剪本身 `O(种子数²)` ⇒ 合计 `O(格数 × 种子数)`（旧实现含一个额外的 `× 种子数` 因子，
//! 且每种子要扫**整个区域**）。
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

/// **种子的均匀网格**：归属图那一步的候选加速（每格只查邻近若干格，而不是全部种子）。
/// 网格只裁候选、**不改判据** —— 最近种子仍是"`(d², 序号)` 最小"，所以结果与全扫逐位一致。
///
/// ⚠️ **只在种子足够多时才建**（`GRID_MIN_SITES`）：实测（24³ 格、release）16/32 种子时网格
/// **反而慢**（建表 + 环遍历的常数项盖过省下的距离计算），64 起打平、128/256/512 对朴素分别是
/// 9.9/16.9/**35.7×**。拐点随"格数 × 种子数"移动 ⇒ 要改阈值就重跑 `tests/voronoi_parity.rs`
/// 的成本扫描（那是这把刻度的量具）。
const GRID_MIN_SITES: usize = 64;

struct SeedGrid {
    origin: Vec3,
    inv_bin: f32,
    bin: f32,
    dims: (i32, i32, i32),
    bins: Vec<Vec<u32>>,
    /// 逐环偏移（`r = 0,1,2,…` 的**壳**；预计算一次 ⇒ 逐格查询不用三重循环 + 过滤）。
    rings: Vec<Vec<[i32; 3]>>,
}

impl SeedGrid {
    /// 建网格：格边 `bin`（调用方给的量级 = 平均种子间距），包住 `[min, max + step]`
    /// （体素格心可能略超 `max`，留一格余量 ⇒ 查询时不必夹取）。
    fn build(min: Vec3, max: Vec3, bin: f32, seeds: &[Vec3]) -> Self {
        let inv_bin = 1.0 / bin;
        let dims = (
            (((max.x - min.x) * inv_bin).floor() as i32 + 1).max(1),
            (((max.y - min.y) * inv_bin).floor() as i32 + 1).max(1),
            (((max.z - min.z) * inv_bin).floor() as i32 + 1).max(1),
        );
        let mut bins: Vec<Vec<u32>> =
            vec![Vec::new(); dims.0 as usize * dims.1 as usize * dims.2 as usize];
        for (si, &s) in seeds.iter().enumerate() {
            let ix = (((s.x - min.x) * inv_bin).floor() as i32).clamp(0, dims.0 - 1);
            let iy = (((s.y - min.y) * inv_bin).floor() as i32).clamp(0, dims.1 - 1);
            let iz = (((s.z - min.z) * inv_bin).floor() as i32).clamp(0, dims.2 - 1);
            bins[(ix as usize * dims.1 as usize + iy as usize) * dims.2 as usize + iz as usize]
                .push(si as u32);
        }
        // 逐环壳偏移（r = 0 = 单格；r ≥ 1 = 该切比雪夫半径的六个面）
        let r_max = dims.0.max(dims.1).max(dims.2);
        let mut rings = Vec::with_capacity(r_max as usize + 1);
        for r in 0..=r_max {
            let mut shell = Vec::new();
            for dx in -r..=r {
                for dy in -r..=r {
                    for dz in -r..=r {
                        if dx.abs().max(dy.abs()).max(dz.abs()) == r {
                            shell.push([dx, dy, dz]);
                        }
                    }
                }
            }
            rings.push(shell);
        }
        Self {
            origin: min,
            inv_bin,
            bin,
            dims,
            bins,
            rings,
        }
    }

    /// 最近种子 `(d², 序号)`；环扩张到"未扫的格不可能更近"为止（下界 `(r−1)·bin`）。
    /// 中心格越界（体素格心落在网格外，防御分支）⇒ 退回全扫，绝不漏种子。
    fn nearest(&self, p: Vec3, seeds: &[Vec3]) -> (f32, u32) {
        let Some((cx, cy, cz)) = self.center(p) else {
            return full_scan(p, seeds);
        };
        let mut best = (f32::INFINITY, u32::MAX);
        for (r, shell) in self.rings.iter().enumerate() {
            let lower = (r.saturating_sub(1)) as f32 * self.bin;
            if lower > 0.0 && lower * lower > best.0 {
                break; // 更远的格全都不可能更近
            }
            for off in shell {
                self.scan_bin(cx + off[0], cy + off[1], cz + off[2], p, seeds, &mut best);
            }
        }
        if best.1 == u32::MAX {
            full_scan(p, seeds) // 不该发生（种子都在区域内）；防御
        } else {
            best
        }
    }

    /// 扫一格里的种子，按 `(d², 序号)` 取更小者（越界格直接跳过）。
    fn scan_bin(&self, ix: i32, iy: i32, iz: i32, p: Vec3, seeds: &[Vec3], best: &mut (f32, u32)) {
        if !self.in_bounds(ix, iy, iz) {
            return;
        }
        for &si in &self.bins[((ix * self.dims.1 + iy) * self.dims.2 + iz) as usize] {
            let d = (p - seeds[si as usize]).length_squared();
            if d < best.0 || (d == best.0 && si < best.1) {
                *best = (d, si);
            }
        }
    }

    /// `p` 所在的格索引；越界 ⇒ `None`（调用方退回全扫）。
    fn center(&self, p: Vec3) -> Option<(i32, i32, i32)> {
        let (ix, iy, iz) = self.bin_of(p);
        if self.in_bounds(ix, iy, iz) {
            Some((ix, iy, iz))
        } else {
            None
        }
    }

    #[inline]
    fn in_bounds(&self, ix: i32, iy: i32, iz: i32) -> bool {
        ix >= 0 && iy >= 0 && iz >= 0 && ix < self.dims.0 && iy < self.dims.1 && iz < self.dims.2
    }

    /// 世界坐标 → 格索引（不夹取；越界由调用方处理）。
    #[inline]
    fn bin_of(&self, p: Vec3) -> (i32, i32, i32) {
        let f = (p - self.origin) * self.inv_bin;
        (f.x.floor() as i32, f.y.floor() as i32, f.z.floor() as i32)
    }
}

/// 全扫（`(d², 序号)` 最小；与旧判据逐字同规则）。
fn full_scan(p: Vec3, seeds: &[Vec3]) -> (f32, u32) {
    let mut best = (f32::INFINITY, u32::MAX);
    for (si, &seed) in seeds.iter().enumerate() {
        let d = (p - seed).length_squared();
        let si = si as u32;
        if d < best.0 || (d == best.0 && si < best.1) {
            best = (d, si);
        }
    }
    best
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
            // 只在**保守 AABB** 内提取（P13）：盒必然包含该种子的 Voronoi 胞 ⇒ 盒列表逐位不变，
            // 但每次扫描从"全区域"降到"该种子的邻域"。
            let (lo, hi) = self.seed_box(min, max, si, seeds);
            let boxes = self.extract_where(lo, hi, |ix, iy, iz| owner.at(ix, iy, iz) == si as u32);
            if !boxes.is_empty() {
                out.push((si, boxes));
            }
        }
        out
    }

    /// 种子的**保守 AABB**：把外区域盒用与其余种子的**二分面半空间**逐次裁剪
    /// （`|p−sᵢ|² ≤ |p−sⱼ|²` ⇔ `2p·(sⱼ−sᵢ) ≤ |sⱼ|²−|sᵢ|²`，是线性不等式）。
    ///
    /// 每轴只做"**另一轴取最大贡献**"的保守收缩（不引线性规划）：对轴 `i` 与 `aᵢ > 0`，
    /// `pᵢ ≤ (b − Σ_{j≠i} max(a_j p_j)) / aᵢ`；`aᵢ < 0` 时同式给出下界（除负号翻转）。
    /// 最后**向外扩一格**并夹回外区域 —— ① 保住"盒 ⊇ 胞"；② `extract_where` 用 `floor` 取格，
    /// 扩一格保证边界格不被漏掉。种子自身恒满足所有约束 ⇒ 盒非空。
    fn seed_box(&self, min: Vec3, max: Vec3, si: usize, seeds: &[Vec3]) -> (Vec3, Vec3) {
        let (mut lo, mut hi) = (min, max);
        let s = seeds[si];
        let s2 = s.length_squared();
        for (sj, &other) in seeds.iter().enumerate() {
            if sj == si {
                continue;
            }
            let a = (other - s) * 2.0;
            let b = other.length_squared() - s2;
            // 其余轴一律取**最小贡献**才是保守界（写反会把盒缩得比胞还小 ⇒ 提取全空）：
            //   `aᵢ > 0` 时 `pᵢ ≤ (b − T)/aᵢ`，取 `T = min T` 得最大上界；
            //   `aᵢ < 0` 时 `pᵢ ≥ (b − T)/aᵢ`，同样取 `T = min T` 得**最小**（最宽松）下界。
            let mn = |aj: f32, lo_j: f32, hi_j: f32| if aj > 0.0 { aj * lo_j } else { aj * hi_j };
            let (mnx, mny, mnz) = (
                mn(a.x, lo.x, hi.x),
                mn(a.y, lo.y, hi.y),
                mn(a.z, lo.z, hi.z),
            );
            if a.x > 0.0 {
                hi.x = hi.x.min((b - (mny + mnz)) / a.x);
            } else if a.x < 0.0 {
                lo.x = lo.x.max((b - (mny + mnz)) / a.x);
            }
            if a.y > 0.0 {
                hi.y = hi.y.min((b - (mnx + mnz)) / a.y);
            } else if a.y < 0.0 {
                lo.y = lo.y.max((b - (mnx + mnz)) / a.y);
            }
            if a.z > 0.0 {
                hi.z = hi.z.min((b - (mnx + mny)) / a.z);
            } else if a.z < 0.0 {
                lo.z = lo.z.max((b - (mnx + mny)) / a.z);
            }
        }
        let e = Vec3::splat(self.step());
        ((lo - e).max(min), (hi + e).min(max))
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
        // 候选网格：**种子多到全扫吃亏时才建**（阈值与实测见 `GRID_MIN_SITES`）；格边取
        // "平均种子间距"量级（`(体积/种子数)^{1/3}`，下限一格）。它只裁候选（环扩张下界
        // `(r−1)·bin`）⇒ 最近种子仍是全扫的那个 ⇒ 逐位一致。
        let grid = if seeds.len() >= GRID_MIN_SITES {
            let volume = ((max.x - min.x) * (max.y - min.y) * (max.z - min.z)).max(0.0);
            let spacing = (volume / seeds.len() as f32).max(0.0).powf(1.0 / 3.0);
            let bin = spacing.max(self.step());
            Some(SeedGrid::build(
                min,
                max + Vec3::splat(self.step()),
                bin,
                seeds,
            ))
        } else {
            None
        };
        for iz in z0..=z1 {
            for iy in y0..=y1 {
                for ix in x0..=x1 {
                    if !self.get(ix as u32, iy as u32, iz as u32) {
                        continue;
                    }
                    let c = self.grid_center(ix as u32, iy as u32, iz as u32);
                    let best = match &grid {
                        Some(g) => g.nearest(c, seeds),
                        None => full_scan(c, seeds),
                    };
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
