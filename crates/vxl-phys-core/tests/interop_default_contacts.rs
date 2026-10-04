//! `vxl_phys_core::interop::CollisionProvider` **默认实现**的口径测试。
//!
//! 从 `src/interop.rs` 的 `#[cfg(test)] mod tests` **原样外迁**（2026-10-04，M2 切片）：
//! god 门按**文件行数**计棘轮（只准减），而本切片要给 trait 补「球 / 点」两个查询
//! ⇒ 让 src 净减、测试改走**公开 API**（`interop` 本就是 `pub mod`，口径一字未改）。
//!
//! ⚠️ 本文件不许出现 `unwrap` / `expect` / `.clone()`（unwrap/clone 门对新文件零基线）。

use vxl_phys_core::interop::{CollisionProvider, SurfaceHit};
use vxl_phys_core::{Aabb, Quat, Vec3};

/// 平面（y = 0）半空间场：`closest_point` 恒可解 ⇒ 用来量 `contacts_box` 的默认 8 角点采样。
struct PlaneField;

impl CollisionProvider for PlaneField {
    fn bounds(&self) -> Aabb {
        Aabb {
            min: Vec3::new(-100.0, -1.0, -100.0),
            max: Vec3::new(100.0, 0.0, 100.0),
        }
    }

    fn closest_point(&self, p: Vec3) -> Option<SurfaceHit> {
        Some(SurfaceHit {
            point: Vec3::new(p.x, 0.0, p.z),
            normal: Vec3::new(0.0, 1.0, 0.0),
            signed_dist: p.y,
        })
    }
}

#[test]
fn default_contacts_box_samples_corners() {
    let f = PlaneField;
    let mut out = Vec::new();
    // 盒心 y = 0.4、半高 0.5 ⇒ 底面 4 角穿透 0.1
    let any = f.contacts_box(
        Vec3::splat(0.5),
        Vec3::new(0.0, 0.4, 0.0),
        Quat::IDENTITY,
        0.02,
        &mut out,
    );
    assert!(any);
    assert_eq!(out.len(), 4, "只有底面 4 角进入 skin 带");
    for c in &out {
        assert!(
            (c.depth - 0.1).abs() < 1e-5,
            "穿透深度应为 0.1，实际 {}",
            c.depth
        );
        assert!((c.normal - Vec3::new(0.0, 1.0, 0.0)).length() < 1e-6);
    }
}

#[test]
fn speculative_contact_kept_within_skin() {
    let f = PlaneField;
    let mut out = Vec::new();
    // 底面在 y=0.01（未接触，缝 0.01 ≤ skin 0.02）⇒ 保留为预期接触（负深度）
    let any = f.contacts_box(
        Vec3::splat(0.5),
        Vec3::new(0.0, 0.51, 0.0),
        Quat::IDENTITY,
        0.02,
        &mut out,
    );
    assert!(any);
    assert_eq!(out.len(), 4);
    for c in &out {
        assert!(
            (c.depth + 0.01).abs() < 1e-5,
            "预期接触深度应 −0.01，实际 {}",
            c.depth
        );
    }
}

/// **默认不支持**的球 / 点查询：返回 false 且**一个点都不推**（调用方据此不产出接触）。
/// 这是与窄相侧 `ProviderColliders` 的同款缺省口径 —— 支持与否必须显式声明，
/// 免得"没覆写"被当成了和别处不同的隐式解。
#[test]
fn sphere_and_point_default_to_unsupported() {
    let f = PlaneField;
    let mut out = Vec::new();
    assert!(!f.contacts_sphere(Vec3::ZERO, 0.5, 0.02, &mut out));
    assert!(!f.contacts_point(Vec3::ZERO, 0.02, &mut out));
    assert!(out.is_empty(), "不支持时不得推接触：{out:?}");
}
