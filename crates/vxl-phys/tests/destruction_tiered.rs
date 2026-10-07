//! **M3 运行时「分级碎裂」的判据**（`DestructionExt::apply_impact_destruction_tiered`）。
//!
//! 默认路径（`apply_impact_destruction`，定半径球弹坑）的判据在 `src/tests/mesh_carve.rs`；
//! 本文件只守 **opt-in 那条**，钉三件事：
//! ① 它真的接到引擎上（同场景同发弹 ⇒ 产出碎块、体数变多）；
//! ② **可复现**（两次构造 ⇒ 碎块数/体数/末态哈希逐位一致）；
//! ③ `curve` 与 `FragmentBudget` **真的通了策略**——放大曲线 ⇒ site 数上去 ⇒ 碎块变多；
//!    同一大曲线下把预算收到 `B1K`（1024）⇒ site 被压回 1024 以内 ⇒ 碎块变少。
//!    （体素取 **0.1 细档**是为了让弹坑里的格数 ≫ site 数：0.5 档下弹坑只有 ~343 格，
//!    site 数根本咬不住，预算这条判据就成了摆设。）
//! ④ `DestructionConfig::depth` **真的生效**：`Two` / `Three` 走**层级 Voronoi 多轮细分**
//!    （`VoxelVolume::fracture_voronoi_hierarchical`），每轮 site 数按 `FragmentBudget` 摊
//!    （取最大 `k`、`r` 使 `k^r ≤ cap`）；`One` 仍走既有单轮路径（逐位不变）。
//!    判据 = **更细 + 可复现 + 不超预算**。
//!
//! 放**集成测试**而非 `src/tests.rs` 内联模块：后者受 `god.gate.json` 行数棘轮管（只准减），
//! 新文件只判阈值——这是仓内加测试的既定通道（`tests/debris_mass.rs` 头注同款说明）。

use vxl_phys::core::Quat;
use vxl_phys::{DestructionExt, PhysConfig, Shape, Vec3, World};
use vxl_phys_destruction::impact_tiers::TierCurve;
use vxl_phys_destruction::{DestructionConfig, FractureDepth, FragmentBudget};

/// 场景：0.1 细档体素（地板 + 一面墙），一发 12 m/s 的盒弹打上去。
fn scene() -> World {
    let mut w = World::new(PhysConfig::default());
    let mut vol =
        vxl_phys_terrain::voxel::VoxelVolume::new(Vec3::new(-1.5, 0.0, -1.5), 0.1, 32, 32, 32);
    // 地板（3 层）+ 墙（x ∈ [0,0.3)、y ∈ [0.3,1.5)、z ∈ [−0.6,0.6)）
    vol.fill_box(Vec3::new(-1.5, 0.0, -1.5), Vec3::new(1.7, 0.3, 1.7));
    vol.fill_box(Vec3::new(0.0, 0.3, -0.6), Vec3::new(0.3, 1.5, 0.6));
    w.add_voxel(vol);
    let bullet = w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(0.2),
        },
        Vec3::new(-1.0, 1.0, 0.0),
        Quat::IDENTITY,
        2000.0,
    );
    w.bodies.linvel[bullet as usize] = Vec3::new(12.0, 0.0, 0.0);
    w
}

/// 跑满 `ticks` 个 tick，返回 `(碎块总数, 末态动态体数, 末态哈希)`。
fn run_with(
    depth: FractureDepth,
    budget: FragmentBudget,
    curve: TierCurve,
    ticks: usize,
) -> (usize, usize, u128) {
    let mut w = scene();
    let cfg = DestructionConfig {
        budget,
        depth,
        ..DestructionConfig::default()
    };
    let mut debris = 0usize;
    for _ in 0..ticks {
        w.step();
        debris += w.apply_impact_destruction_tiered(0, &cfg, curve, 1000.0);
    }
    let dynamic = (0..w.bodies.len())
        .filter(|&i| w.bodies.is_dynamic(i))
        .count();
    (debris, dynamic, w.state_hash())
}

/// 既有三条判据用 `depth = One`（默认档）。
fn run(budget: FragmentBudget, curve: TierCurve, ticks: usize) -> (usize, usize, u128) {
    run_with(FractureDepth::One, budget, curve, ticks)
}

/// 起点锚点曲线（= `TierCurve::default()`，site 上限只有 43）。
fn small_curve() -> TierCurve {
    TierCurve::default()
}

/// 放大曲线（site 上限 2000，**超过最小预算 1024** ⇒ 预算才有机会咬住）。
fn big_curve() -> TierCurve {
    TierCurve {
        core0: 2000,
        core_step: 0,
        core_max: 2000,
        outer0: 0,
        outer_step: 0.0,
        outer_max: 0,
    }
}

#[test]
fn tiered_impact_fractures_and_is_reproducible() {
    let (debris, dynamic, hash) = run(FragmentBudget::B1K, small_curve(), 120);
    assert!(debris > 0, "应触发分级碎裂（产出碎块）debris={debris}");
    assert!(dynamic > 1, "碎块应成为新的动态体 dynamic={dynamic}");
    let (debris2, dynamic2, hash2) = run(FragmentBudget::B1K, small_curve(), 120);
    assert_eq!(
        (debris, dynamic),
        (debris2, dynamic2),
        "分级碎裂应可复现（计数）"
    );
    assert_eq!(hash, hash2, "分级碎裂应可复现（末态哈希逐位一致）");
}

#[test]
fn tiered_curve_and_budget_reach_the_strategy() {
    // ① 曲线通了：放大 site 数 ⇒ 碎块变多。
    let (small, _, _) = run(FragmentBudget::B100K, small_curve(), 120);
    let (big, _, _) = run(FragmentBudget::B100K, big_curve(), 120);
    assert!(big > small, "放大曲线应给更多碎块（小 {small} → 大 {big}）");
    // ② 预算通了：同一条大曲线，收到 B1K ⇒ site 被压到 1024 以内 ⇒ 碎块变少。
    let (capped, _, _) = run(FragmentBudget::B1K, big_curve(), 120);
    assert!(
        capped < big,
        "收预算应把碎块压下来（B100K {big} → B1K {capped}）"
    );
}

/// `depth` 真的生效：`Two` 应比 `One` 产出的碎块**更多**，且始终 ≤ `FragmentBudget` 的 cap。
#[test]
fn depth_two_fractures_finer_than_one() {
    let (one, _, _) = run_with(FractureDepth::One, FragmentBudget::B1K, small_curve(), 120);
    let (two, _, _) = run_with(FractureDepth::Two, FragmentBudget::B1K, small_curve(), 120);
    println!("depth 粒化（B1K / small_curve / 120 tick）：One={one} Two={two}");
    assert!(one > 0, "depth=One 应产出碎块（前置条件）one={one}");
    assert!(two > one, "depth=Two 应更细：one={one} two={two}");
    assert!(two <= 1024, "不得超 FragmentBudget::B1K（实得 {two}）");
}

/// 多轮也可复现：两次构造 ⇒ 计数与末态哈希逐位一致。
#[test]
fn depth_two_is_reproducible() {
    let a = run_with(FractureDepth::Two, FragmentBudget::B1K, small_curve(), 120);
    let b = run_with(FractureDepth::Two, FragmentBudget::B1K, small_curve(), 120);
    assert_eq!((a.0, a.1), (b.0, b.1), "多轮碎裂应可复现（计数）");
    assert_eq!(a.2, b.2, "多轮碎裂应可复现（末态哈希逐位一致）");
}

/// `Three` 也要真的生效（三段路径的预算摊法与 Two 不同：k³）——更细 + 不超预算。
#[test]
fn depth_three_is_finer_than_two_and_bounded() {
    let (two, _, _) = run_with(FractureDepth::Two, FragmentBudget::B1K, small_curve(), 120);
    let (three, _, _) = run_with(
        FractureDepth::Three,
        FragmentBudget::B1K,
        small_curve(),
        120,
    );
    println!("depth 粒化（B1K / small_curve / 120 tick）：Two={two} Three={three}");
    assert!(three > two, "depth=Three 应更细：two={two} three={three}");
    assert!(three <= 1024, "不得超 FragmentBudget::B1K（实得 {three}）");
}
