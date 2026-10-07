//! **M3 运行时「分级碎裂」的接线**：把 `vxl-phys-destruction::impact_tiers` 那份纯函数策略
//! 接到引擎上（`CATALOG.md` 记的「⚠️ 本 crate 目前无消费方 ⇒ 待决：接线或合并」——这里按
//! **接线**办）。
//!
//! **为什么是新文件 + 扩展 trait**：`god.gate.json` 把 `World` 的方法数登记为**只准减**的债务，
//! 处置条目明写「域侧 API 拆成扩展 trait」⇒ 新 API 走 `impl DestructionExt for World`
//! （`god_gate.py` 对 `impl Trait for Type` 记的是 trait 的实现、不计入该类型的方法账），
//! 与 `world_step/conversion.rs` 同款先例。
//!
//! **与默认路径并存**：`World::apply_impact_destruction`（定半径球弹坑）**一字未动**；本文件是
//! **opt-in 的另一条**，所以默认档逐位不变（四哈希门都不走这条路径）。

use crate::{Quat, Shape, Vec3, VoxelConversionExt, World};

use vxl_phys_destruction::impact_tiers::{
    budget_cap, depth_rounds, impulse_level, sites_within_budget, TierCurve, REF_IMPULSE,
};
use vxl_phys_destruction::DestructionConfig;

/// Voronoi 种子的抖动幅度（相对弹坑盒）：`seeds_jittered` 按**格点序**生成 ⇒ 确定性。
const SEED_JITTER: f32 = 0.25;

/// 沿 `v` 的单位向量；零向量给零（与默认路径同款守卫）。
fn unit_or_zero(v: Vec3) -> Vec3 {
    if v.length_squared() > 1e-9 {
        v.normalize()
    } else {
        Vec3::ZERO
    }
}

/// 弹坑半径（与默认路径**同一式子**——两条路的几何一致，差别只在"整块球提取 vs Voronoi 分块"）。
fn crater_radius(approach: f32) -> f32 {
    (0.2 + 0.06 * approach).clamp(0.25, 0.9)
}

/// 阈值查表：按冲击体的**材质 id** 取（越界取末项；表空 ⇒ 退回 `REF_IMPULSE`）。
fn threshold_for(cfg: &DestructionConfig, mat: u32) -> f32 {
    match cfg.energy_thresholds.len() {
        0 => REF_IMPULSE,
        n => cfg.energy_thresholds[(mat as usize).min(n - 1)],
    }
}

/// **M3 运行时分级碎裂**（opt-in；与默认的定半径弹坑路径并存）。
pub trait DestructionExt {
    /// 把本 tick 的冲击按**冲量分级**转成 Voronoi 碎块，返回本次产出的碎块总数。
    ///
    /// 判据与默认路径**同源**（`record_impacts` 记录的「沿接触法向接近速度」+ 只对动态体），
    /// 差别只有三处：① 阈值取 `cfg.energy_thresholds`（按冲击体材质 id 查表）；
    /// ② 冲量 `J = m·approach` 经 `impulse_level` 分档，`sites_within_budget` 给 site 数
    /// （受 `cfg.budget` 封顶）；③ 弹坑域按 **Voronoi 分块**成碎块（默认路径整块球提取）。
    ///
    /// 确定性：记录序 → 纯函数分档 → `seeds_jittered`（格点序）→ `fracture_voronoi`
    /// （固定扫描序）⇒ 同输入同输出。
    ///
    /// **多轮细分**（`cfg.depth ∈ {One, Two, Three}`）：轮次 = `depth_rounds(cfg.depth)`；
    /// 每轮 site 数 = 曲线给的 site 数**按预算摊**（`per_round_sites`：取最大的 `k ≤ site` 与
    /// 最大的 `r ≤ rounds`，使 `k^r ≤` `FragmentBudget` 的 cap）—— 口径照 `depth_rounds` 的注
    /// 「层数决定**能碎几轮**、site 数决定**每轮几块**」。`r == 1` 时**仍走既有单轮路径**
    /// （`depth = One` 的逐位语义与本改动前完全一致）。
    ///
    /// 实现 = `VoxelVolume::fracture_voronoi_hierarchical`：每轮把上一轮的块**回填成体素**再分
    /// （复用同一套贪心提取），血缘 = "第 r 轮的块是第 r−1 轮某块的格集"。
    ///
    /// `curve` 由调用方给（`TierCurve::default()` 是起点锚点）：**预算封顶只有在曲线够大时
    /// 才会咬住**（默认曲线的 site 上限只有 43，远小于最小预算 `B1K` = 1024）。
    fn apply_impact_destruction_tiered(
        &mut self,
        id: u32,
        cfg: &DestructionConfig,
        curve: TierCurve,
        density: f32,
    ) -> usize;
}

impl DestructionExt for World {
    fn apply_impact_destruction_tiered(
        &mut self,
        id: u32,
        cfg: &DestructionConfig,
        curve: TierCurve,
        density: f32,
    ) -> usize {
        // 轮次 = `FractureDepth`；每轮 site 数再按 `FragmentBudget` 摊（见 `per_round_sites`）。
        let rounds = depth_rounds(cfg.depth);
        // 先在**只读**扫描里收集「挖点」（按记录序 ⇒ 确定性）：(球心, 半径, 每轮 site, 轮数)。
        let mut digs: Vec<(Vec3, f32, u32, u32)> = Vec::new();
        for rec in &self.impacts {
            if rec.provider != id || !self.bodies.is_dynamic(rec.body as usize) {
                continue;
            }
            let iu = rec.body as usize;
            let mass = 1.0 / self.bodies.inv_mass[iu].max(1e-9);
            let limit = threshold_for(cfg, self.bodies.material[iu]);
            let impulse = mass * rec.approach;
            if impulse < limit {
                continue;
            }
            // 参照冲量取「材质阈值」与 `REF_IMPULSE` 里较大的那个 ⇒ **刚好够触发的那一下 = 0 级**
            // （这正是 `REF_IMPULSE` 文档写的口径）。
            let level = impulse_level(impulse, REF_IMPULSE.max(limit));
            let (core, outer) = sites_within_budget(level, cfg.budget, curve);
            let sites = core.saturating_add(outer);
            if sites == 0 {
                continue;
            }
            let (per_round, rounds_eff) = per_round_sites(sites, rounds, budget_cap(cfg.budget));
            let r = crater_radius(rec.approach);
            digs.push((
                rec.point + unit_or_zero(rec.velocity) * (r * 1.05),
                r,
                per_round,
                rounds_eff,
            ));
        }
        let mut total = 0usize;
        for (center, r, per_round, rounds_eff) in digs {
            // 弹坑域 = 以球心为中心的立方体（边长 2r）；种子抖动格点由体素侧生成。
            let (min, max) = (center - Vec3::splat(r), center + Vec3::splat(r));
            if rounds_eff <= 1 {
                // 单轮：走既有 M3 预断裂路径（效应键 + bounds 刷新都在 `fracture_voronoi` 里）。
                let seeds = vxl_phys_terrain::voxel::VoxelVolume::seeds_jittered(
                    min,
                    max,
                    per_round as usize,
                    SEED_JITTER,
                );
                total += self.fracture_voronoi(id, min, max, &seeds, density);
            } else {
                total +=
                    fracture_voronoi_hier(self, id, center, r, (per_round, rounds_eff), density);
            }
        }
        total
    }
}

/// 预算与轮次 → `(每轮 site 数, 实际轮数)`：取最大的 `r ≤ rounds` 与最大的 `k ≤ sites`，
/// 满足 `k^r ≤ cap` —— 口径 = 「层数决定**能碎几轮**，site 数决定**每轮几块**」
/// （`impact_tiers::depth_rounds` 的注）。`k < 2` 时退化成单轮：碎一块不算"一层"。
fn per_round_sites(sites: u32, rounds: u32, cap: u32) -> (u32, u32) {
    let sites = sites.max(1);
    for r in (1..=rounds.max(1)).rev() {
        let mut k = sites;
        while k > 1 && !pow_le(k, r, cap) {
            k -= 1;
        }
        if k >= 2 || r == 1 {
            return (k, r);
        }
    }
    (sites, 1)
}

/// `k^r ≤ cap`（饱和乘法，避免溢出）。
fn pow_le(k: u32, r: u32, cap: u32) -> bool {
    let mut acc = 1u64;
    for _ in 0..r {
        acc *= k as u64;
        if acc > cap as u64 {
            return false;
        }
    }
    true
}

/// 多轮分层碎裂：`VoxelVolume::fracture_voronoi_hierarchical` → 逐盒 push 动态体。
/// 与单轮 `World::fracture_voronoi` **同款口径**：效应键一次、密度公式一致、末尾刷新 bounds。
/// `plan = (每轮 site 数, 轮数)`；弹坑域 = 以 `center` 为中心、边长 `2r` 的立方体。
fn fracture_voronoi_hier(
    world: &mut World,
    id: u32,
    center: Vec3,
    r: f32,
    plan: (u32, u32),
    density: f32,
) -> usize {
    if world.claim_extraction_effect(id).is_err() {
        return 0;
    }
    let Some(vol) = world.providers.voxel_mut(id) else {
        return 0;
    };
    let (min, max) = (center - Vec3::splat(r), center + Vec3::splat(r));
    let blocks = vol.fracture_voronoi_hierarchical(min, max, plan.0, plan.1, SEED_JITTER);
    let mut n = 0usize;
    for block in blocks {
        for (c, h) in block {
            let d = (1e-3 / (8.0 * h.x * h.y * h.z)).max(density).max(1e-3);
            world
                .bodies
                .push_dynamic(Shape::Box { half: h }, c, Quat::IDENTITY, d);
            n += 1;
        }
    }
    world.refresh_provider_bounds();
    n
}
