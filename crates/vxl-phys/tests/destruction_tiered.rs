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
//! ④ `DestructionConfig::depth` **不是装饰**：`Two` / `Three` 的多轮细分尚未实现 ⇒ 显式
//!    断言拒绝，**不静默降级成一轮**（多轮需要碎片自身的体素表示，属单独一片）。
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
fn run(budget: FragmentBudget, curve: TierCurve, ticks: usize) -> (usize, usize, u128) {
    let mut w = scene();
    let cfg = DestructionConfig {
        budget,
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

/// `depth` 不是装饰：多轮细分未实现 ⇒ 入口显式拒绝，不静默按一轮处理。
/// （多轮落地后，这条应改成"`Two` 真的比 `One` 产出更多碎块"的正向判据。）
#[test]
#[should_panic(expected = "尚未实现")]
fn tiered_multiround_depth_is_rejected_not_silently_downgraded() {
    let mut w = scene();
    let cfg = DestructionConfig {
        budget: FragmentBudget::B1K,
        depth: FractureDepth::Two,
        ..DestructionConfig::default()
    };
    // 断言在函数入口（先于冲击扫描）⇒ 不需要先 `step`。
    w.apply_impact_destruction_tiered(0, &cfg, small_curve(), 1000.0);
}
