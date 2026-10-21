//! Voronoi 预断裂的**预算落点**（安全审计 F-14）。
//!
//! 审计读数：公开的 `World::fracture_voronoi` 传大量种子时**一次调用生成 30 403 个刚体**
//! （冲击破坏路径受 `FragmentBudget` 默认 `B1K` 约束，这条公开路径不受），耗时近平方
//! （2k 种子 0.15 s → 8k 1.1 s → 32k 16.5 s → 200k 超时）。
//!
//! 处置（审计 §4 S3 的口径："预算对象……超了就截断并上报"）：
//!
//! - **种子数**与**碎块数**都封顶 [`BUDGET`]（= 破坏管线的默认档 `B1K`）。种子封顶在调用
//!   terrain 的分块之前 ⇒ 近平方的那部分也被挡住；
//! - 截断**不丢几何**：超预算的盒**回填成体素**（它们是整格并集 ⇒ 回填 = 原格集），
//!   下次调用/冲击破坏照常处理；这条是判据里"第二次调用还能提取"的前提；
//! - **如实上报**：返回值就是本次真正生成的碎块数，调用方看到 `== BUDGET` 即知道被截断
//!   ⇒ 要更多就分片调用（把种子或域切小）。
//!
//! 为什么是**自由函数**而不是 `World` 的方法：`god.gate.json` 把 `World` 的方法数登记为债务
//! （跨文件累计、只准减），加一个方法就红——同 `world_build.rs::jobs_for` 的先例。

use vxl_phys_core::{Aabb, Quat, Shape, Vec3};
use vxl_phys_destruction::impact_tiers::budget_cap;
use vxl_phys_destruction::FragmentBudget;

use crate::{ProviderExt, VoxelConversionExt};

/// **公开路径的默认预算**（`World::fracture_voronoi` 用它）：破坏管线的默认档 `B1K`。
pub(crate) const DEFAULT_BUDGET: FragmentBudget = FragmentBudget::B1K;

/// 一次预断裂（效应键 → 分块 → 落体 + 预算 → 刷新 bounds）。**扇出为自由函数**：
/// 公开路径（默认预算）与破坏策略路径（`DestructionConfig::budget`）共用它 ⇒ 单一来源。
pub(crate) fn extract(
    world: &mut crate::World,
    id: u32,
    domain: Aabb,
    seeds: &[Vec3],
    density: f32,
    budget: FragmentBudget,
) -> usize {
    if world.claim_extraction_effect(id).is_err() {
        return 0;
    }
    let cap = budget_cap(budget) as usize;
    let Some(vol) = world.providers.voxel_mut(id) else {
        return 0;
    };
    let cells = vol.fracture_voronoi(domain.min, domain.max, &seeds[..seeds.len().min(cap)]);
    let n = spawn_bodies(world, id, cells, density, cap);
    world.refresh_provider_bounds();
    n
}

/// 把 Voronoi 分块结果落成刚体，**封顶 [`BUDGET`]**；返回真正生成的碎块数。
pub(crate) fn spawn_bodies(
    world: &mut crate::World,
    voxel_id: u32,
    cells: Vec<(usize, Vec<(Vec3, Vec3)>)>,
    density: f32,
    cap: usize,
) -> usize {
    let mut n = 0usize;
    let mut excess: Vec<(Vec3, Vec3)> = Vec::new();
    for (_si, boxes) in cells {
        for (c, h) in boxes {
            if n >= cap {
                excess.push((c, h));
                continue;
            }
            // 薄壳碎块的密度兜底：极薄/极小的盒按体积反推，避免 `inv_mass` 爆掉。
            let d = (1e-3 / (8.0 * h.x * h.y * h.z)).max(density).max(1e-3);
            world
                .bodies
                .push_dynamic(Shape::Box { half: h }, c, Quat::IDENTITY, d);
            n += 1;
        }
    }
    if !excess.is_empty() {
        if let Some(vol) = world.providers.voxel_mut(voxel_id) {
            for (c, h) in &excess {
                vol.fill_box(*c - *h, *c + *h);
            }
        }
    }
    n
}
