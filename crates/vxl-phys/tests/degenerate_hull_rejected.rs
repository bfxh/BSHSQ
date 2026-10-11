//! **退化外壳 / 空壳复合体的入口处置**（安全审计 F-10：hull / compound 质量回退路径）。
//!
//! 审计读数：空、单点、全重复、共线点集 ⇒ 回退成极小尺寸 ⇒ `inv_mass = 1.25e8`、
//! `inv_inertia ≈ 1.9e16`（单体 200 步不出 NaN，但与正常质量体接触时是 1e8 级的质量比 ⇒
//! 解算器必然爆）。根因在 `mass_props` 的 `aabb_box_props`：它按 AABB 盒近似，而退化点云的
//! AABB 三条边被 `max(1e-4)` 兜底成 `(2e-4)³` ⇒ `m ≈ 8e-9 kg`。
//!
//! 处置遵循 S1 口径（入口拒绝、内核不猜）：**外壳在生成刚体时**要求点云至少是**二维**的
//! （AABB 至少两条边非零）；**复合体注册**要求至少一个非嵌套子形状。为什么允许一维为零：
//! 仓内真有合法用法——`shape_support_matrix.rs` 用**零厚度平板**外壳与三角网对拍路线差异。
//!
//! 四条：① 单点/共线 ⇒ 入口 panic；② 平板（一条零边）⇒ **放行**（仓内既有用例）；
//! ③ 立方体 ⇒ 放行；④ 空 / 只含嵌套复合体 ⇒ 入口 panic。

use vxl_phys::{CompoundChild, PhysConfig, Quat, Shape, Vec3, World};

fn world() -> World {
    World::new(PhysConfig::default())
}

/// 立方体点云（8 角；合法外壳）。
fn cube(half: f32) -> Vec<Vec3> {
    let mut pts = Vec::new();
    for sx in [-1.0f32, 1.0] {
        for sy in [-1.0f32, 1.0] {
            for sz in [-1.0f32, 1.0] {
                pts.push(Vec3::new(sx * half, sy * half, sz * half));
            }
        }
    }
    pts
}

#[test]
fn cube_hull_body_is_accepted() {
    let mut w = world();
    let h = w.add_hull(cube(0.5));
    let i = w.spawn_hull_body(h, Vec3::new(0.0, 2.0, 0.0), Quat::IDENTITY, 1000.0) as usize;
    // 盒惯量近似的质量 = ρ·(2h)³ = 1000·1 = 1000 kg ⇒ inv_mass = 1e-3（不是 1e8 那种病态值）。
    let inv = w.bodies.inv_mass[i];
    assert!(
        (inv - 1e-3).abs() < 1e-6,
        "立方体外壳的质量口径变了：inv_mass = {inv}"
    );
}

#[test]
fn flat_plate_hull_is_accepted() {
    // 仓内既有用例（shape_support_matrix 的 hull 路线）：零厚度平板必须继续可用。
    let mut w = world();
    let pts = vec![
        Vec3::new(-0.5, 0.0, -0.5),
        Vec3::new(0.5, 0.0, -0.5),
        Vec3::new(0.5, 0.0, 0.5),
        Vec3::new(-0.5, 0.0, 0.5),
    ];
    let h = w.add_hull(pts);
    let i = w.spawn_hull_body(h, Vec3::new(0.0, 2.0, 0.0), Quat::IDENTITY, 1000.0) as usize;
    assert!(w.bodies.inv_mass[i] > 0.0, "平板外壳应当仍是动态体");
}

#[test]
#[should_panic(expected = "退化外壳")]
fn single_point_hull_is_rejected_at_body_creation() {
    let mut w = world();
    let h = w.add_hull(vec![Vec3::ZERO]);
    let _ = w.spawn_hull_body(h, Vec3::ZERO, Quat::IDENTITY, 1000.0);
}

#[test]
#[should_panic(expected = "退化外壳")]
fn collinear_hull_is_rejected_at_body_creation() {
    let mut w = world();
    let pts = (0..4).map(|k| Vec3::new(k as f32, 0.0, 0.0)).collect();
    let h = w.add_hull(pts);
    let _ = w.spawn_hull_body(h, Vec3::ZERO, Quat::IDENTITY, 1000.0);
}

#[test]
#[should_panic(expected = "不合法")]
fn empty_compound_is_rejected_at_registration() {
    let mut w = world();
    let _ = w.add_compound(Vec::new());
}

#[test]
#[should_panic(expected = "不合法")]
fn nested_only_compound_is_rejected_at_registration() {
    // 只含嵌套复合体 ⇒ 会被窄相在注册时丢弃 ⇒ 空壳（AABB 退化）。入口就拒绝。
    let mut w = world();
    let inner = w.add_compound(vec![CompoundChild {
        shape: Shape::Sphere { radius: 0.1 },
        offset: Vec3::ZERO,
        rot: Quat::IDENTITY,
    }]);
    let _ = inner;
    let _ = w.add_compound(vec![CompoundChild {
        shape: Shape::Compound {
            compound: inner,
            half: Vec3::splat(0.1),
        },
        offset: Vec3::ZERO,
        rot: Quat::IDENTITY,
    }]);
}
