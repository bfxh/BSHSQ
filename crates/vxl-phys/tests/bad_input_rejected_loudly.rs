//! **坏输入必须在入口响亮拒绝**（2026-10-10 安全审计 S1 的回归判据）。
//!
//! 审计把"公共 API 入口零校验"列为三个系统性缺陷簇之首：坏体号/坏索引不进内核，
//! 而是在 `step()` 里才 panic，或**静默**给出错的结果（NaN 扩散、静默穿地、被当非动态冻结）。
//! 审计同时明确：**入口拒绝、内核保留断言**——不许为了"防 panic"把索引改成 `get()` 后静默跳过
//! （那会把可见崩溃换成更隐蔽的逻辑错误）。本文件钉的就是"入口拒绝"这一半。
//!
//! 本轮覆盖两条已复现项：
//! - **F-05** `provider_id_of`：签名承诺 `Option`，旧实现直接索引 ⇒ 越界 panic。现在返回 `None`。
//! - **F-03** `add_joint`：端点体号越界旧实现要到下一次 `step()` 才 panic。现在在入口断言。
use vxl_phys::{Joint, JointKind, PhysConfig, Quat, Shape, Vec3, World};

fn world_with_two_bodies() -> World {
    let mut w = World::new(PhysConfig::default());
    w.add_static(
        Shape::Box {
            half: Vec3::splat(0.5),
        },
        Vec3::new(0.0, 0.5, 0.0),
        Quat::IDENTITY,
    );
    w
}

/// **F-05**：越界体号必须给出 `None`（签名承诺的语义），而不是 panic。
#[test]
fn provider_id_of_out_of_range_returns_none() {
    let w = world_with_two_bodies();
    assert_eq!(w.provider_id_of(0), None, "真体号不是 provider ⇒ None");
    assert_eq!(
        w.provider_id_of(77),
        None,
        "越界体号必须 None（旧实现 panic）"
    );
    assert_eq!(
        w.provider_id_of(u32::MAX),
        None,
        "超大体号必须 None（旧实现 panic）"
    );
}

/// **F-03**：关节端点体号越界 ⇒ **在 `add_joint` 当场**断言（而不是下一次 `step()` 才崩）。
#[test]
#[should_panic(expected = "关节端点体号越界")]
fn joint_with_out_of_range_body_is_rejected_at_entry() {
    let mut w = world_with_two_bodies();
    w.add_joint(Joint::new(
        JointKind::Spherical,
        0,
        12_345,
        Vec3::ZERO,
        Vec3::ZERO,
    ));
}

/// **非空洞**：合法体号的关节照常接上，且推进一步不炸 —— 证明上面那条不是"全都拒绝"。
#[test]
fn valid_joint_still_works() {
    let mut w = World::new(PhysConfig::default());
    let a = w.add_static(
        Shape::Box {
            half: Vec3::splat(0.1),
        },
        Vec3::new(0.0, 5.0, 0.0),
        Quat::IDENTITY,
    );
    let b = w.add_dynamic(
        Shape::Box {
            half: Vec3::splat(0.1),
        },
        Vec3::new(0.0, 4.8, 0.0),
        Quat::IDENTITY,
        10.0,
    );
    w.add_joint(Joint::new(
        JointKind::Spherical,
        a,
        b,
        Vec3::ZERO,
        Vec3::ZERO,
    ));
    for _ in 0..30 {
        w.step();
    }
    assert!(w.provider_id_of(b).is_none());
}
