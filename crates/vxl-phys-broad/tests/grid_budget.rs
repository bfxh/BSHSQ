//! **Grid 宽相的单体格数预算**（安全审计 F-07）。
//!
//! 审计读数：`GridBroadPhase`（非默认后端）在"200 m 地面 + 0.05 m 格"下 **OOM**（release 9 s
//! 撞 1.4 GB）；单个 x 轴 Inf 体 ⇒ 单步 2.9 s（`cell_of` 钳 ±1e6 ⇒ 2×10⁶ 格/轴）。
//! 处置：单体超过 `64³` 格即**跳过并计数**（`GridBroadPhase::skipped`），不截断、不静默。
//!
//! 三条：① 病态跨度（大地面 × 小格）⇒ 秒回 + `skipped == 1` + 不出配对；
//! ② 同场景换合理格边长 ⇒ `skipped == 0` 且配对正常（预算没误伤）；
//! ③ 巨大但有限的 AABB（半长 1e30 ⇒ 每轴钳到 ±1e6 ⇒ 2×10⁶ 格）同样被挡住。
//!    ⚠️ 反例口径：**位置**为 Inf 的体不会触发预算——`cell_of` 把它的 min/max 钳成同一个格，
//!    跨度退化成 1 格（说明"钳位"本身不是问题，**跨了半个钳位域**才是）。

use vxl_phys_broad::{BroadPhase, GridBroadPhase};
use vxl_phys_core::{BodySet, Quat, SerialJobSystem, Shape, Vec3};

/// 大静态地面 + 站在它上面的一小块动态体（后者与前者重叠 ⇒ 合理格边长下必出一对）。
fn scene() -> BodySet {
    let mut b = BodySet::new();
    b.push_static(
        Shape::Box {
            half: Vec3::new(100.0, 0.5, 100.0),
        },
        Vec3::new(0.0, -0.5, 0.0),
        Quat::IDENTITY,
    );
    b.push_dynamic(
        Shape::Box {
            half: Vec3::splat(0.25),
        },
        Vec3::new(0.0, 0.25, 0.0),
        Quat::IDENTITY,
        1000.0,
    );
    b
}

#[test]
fn pathological_cell_span_is_skipped_and_counted() {
    let b = scene();
    let mut bp = GridBroadPhase::new(0.05, 0.02);
    let jobs = SerialJobSystem;
    let pairs: Vec<(u32, u32)> = bp.compute_pairs(&b, &[], &[], &jobs).to_vec();
    assert_eq!(
        bp.skipped, 1,
        "200 m 地面 ÷ 0.05 m 格 = 4000² × 一格 ⇒ 必超预算"
    );
    assert!(pairs.is_empty(), "被跳过的体不得产出配对：{pairs:?}");
}

#[test]
fn sane_cell_size_is_not_penalized() {
    let b = scene();
    let mut bp = GridBroadPhase::new(1.0, 0.02);
    let jobs = SerialJobSystem;
    let pairs: Vec<(u32, u32)> = bp.compute_pairs(&b, &[], &[], &jobs).to_vec();
    assert_eq!(
        bp.skipped, 0,
        "1 m 格下 200 m 地面只占 201×2×201 格 ⇒ 在预算内"
    );
    assert_eq!(pairs.len(), 1, "地面与盒应当配成一对");
}

#[test]
fn huge_finite_aabb_is_within_the_budget_check() {
    let mut b = BodySet::new();
    b.push_static(
        Shape::Box {
            half: Vec3::splat(1e30),
        },
        Vec3::ZERO,
        Quat::IDENTITY,
    );
    let mut bp = GridBroadPhase::new(0.05, 0.02);
    let jobs = SerialJobSystem;
    let pairs: Vec<(u32, u32)> = bp.compute_pairs(&b, &[], &[], &jobs).to_vec();
    assert_eq!(
        bp.skipped, 1,
        "1e30 半长（钳到 ±1e6 ⇒ 2×10⁶ 格/轴）必须被预算挡住"
    );
    assert!(pairs.is_empty());
}
