//! 质量属性的契约（从 `src/mass.rs` 的内联测试**搬出**：那边受 god 门行数棘轮管，
//! 新文件只判阈值），并补上 2026-10-10 安全审计 **F-09** 的入口拒绝判据。
//!
//! F-09：坏尺寸以前是**静默**的 —— `half = 0` ⇒ `mass = 0` ⇒ `inv_mass = inf` ⇒ 该体自身 NaN；
//! **负**尺寸 ⇒ `mass < 0` ⇒ `inv_mass < 0` ⇒ `is_dynamic`（判据 `inv_mass > 0`）为假
//! ⇒ **被当非动态静默冻结**（实测停在半空、`vy = 0`）。现在在质量属性入口当场断言。
use vxl_phys_core::{mass_props, Shape, Vec3};

#[test]
fn box_inertia_matches_formula() {
    let h = Vec3::new(0.5, 1.0, 2.0);
    let p = mass_props(&Shape::Box { half: h }, 2.0);
    let m = 2.0 * 8.0 * h.x * h.y * h.z;
    assert!((p.mass - m).abs() < 1e-4);
    let ix = m / 12.0 * ((2.0 * h.y) * (2.0 * h.y) + (2.0 * h.z) * (2.0 * h.z));
    assert!((1.0 / p.local_inv_inertia.x - ix).abs() < 1e-3);
}

#[test]
fn sphere_inertia() {
    let p = mass_props(&Shape::Sphere { radius: 1.0 }, 1.0);
    let m = 4.0 / 3.0 * core::f32::consts::PI;
    assert!((p.mass - m).abs() < 1e-4);
    assert!((1.0 / p.local_inv_inertia.x - 2.0 / 5.0 * m).abs() < 1e-4);
}

#[test]
fn cylinder_axis_is_y() {
    let p = mass_props(
        &Shape::Cylinder {
            half_height: 1.0,
            radius: 0.5,
        },
        1.0,
    );
    // Iy = 1/2 m r^2 < Ix —— 绕主轴更容易转。
    assert!(p.local_inv_inertia.y > p.local_inv_inertia.x);
}

/// **F-09 ①**：零尺寸必须**当场**拒绝（旧实现给出 `inv_mass = inf`）。
#[test]
#[should_panic(expected = "形状尺寸必须为正")]
fn zero_box_half_is_rejected() {
    mass_props(&Shape::Box { half: Vec3::ZERO }, 1000.0);
}

/// **F-09 ②**：**负**尺寸是"静默冻结"那条 —— 必须当场拒绝，而不是被当非动态。
#[test]
#[should_panic(expected = "形状尺寸必须为正")]
fn negative_box_half_is_rejected() {
    mass_props(
        &Shape::Box {
            half: Vec3::new(0.5, -0.5, 0.5),
        },
        1000.0,
    );
}

/// **F-09 ③**：零半径同理（球）。
#[test]
#[should_panic(expected = "形状尺寸必须为正")]
fn zero_sphere_radius_is_rejected() {
    mass_props(&Shape::Sphere { radius: 0.0 }, 1000.0);
}

/// **非空洞**：正常尺寸照常算 —— 证明上面三条不是"一律拒绝"。
#[test]
fn valid_dims_still_work() {
    let p = mass_props(
        &Shape::Box {
            half: Vec3::splat(0.5),
        },
        1000.0,
    );
    assert!((p.mass - 1000.0).abs() < 1e-3, "1 m³ × 1000 kg/m³");
    assert!(p.inv_mass.is_finite() && p.inv_mass > 0.0);
}
