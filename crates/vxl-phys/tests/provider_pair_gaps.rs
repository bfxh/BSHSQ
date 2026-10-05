//! **不受理的「组合」缺口钉住**（与 `shape_support_matrix.rs` 同一套做法：现状写成断言，
//! 落地后**翻面**）。
//!
//! 缺口登记处：`docs/SURVEY-SHAPE-SUPPORT-MATRIX.md` §1 末尾的「组合缺口」块、
//! `docs/adr/0009-外部碰撞提供者通道.md`（provider-provider 需**对偶解法**）。
//!
//! 现状（2026-10-05 实测）：两条都**不产接触**，机制在窄相派发顺序里：
//! - **高度场 × 提供者**：`provider_pair` 排在 `heightfield_pair` 之前，而
//!   `provider_shape_contacts` 对 `Shape::HeightField` 落到显式 `false` ⇒ 整对结束；
//! - **提供者 × 提供者**：`provider_pair` 见两侧都是 `Shape::Provider` ⇒ 直接 `return true`。
//!
//! ⚠️ 影响 = 零：提供者（`world_build.rs`）与高度场都是**静态 Marker** ⇒ 求解器本来也不会
//! 从"静态×静态"产出约束。**本文件的作用是让它别再被当漏写**：真做对偶解法时这两条会红，
//! 那时按"翻面"改成"产接触 + 位形/哈希判据"，并同步 §1 的登记行。
//!
//! ⚠️ 本文件不许出现 `unwrap` / `expect` / `.clone()`（新文件零基线）。

use vxl_phys::Providers;
use vxl_phys_core::interop::ProviderColliders;
use vxl_phys_core::{BodySet, Quat, SerialJobSystem, Shape, Vec3};
use vxl_phys_narrow::{DefaultNarrowPhase, NarrowPhase};
use vxl_phys_terrain::voxel::VoxelVolume;

/// 4×4×4 格、边长 0.25 的实心块（覆盖 y ∈ [0,1]，与面高 0.5 的平高度场**必然重叠**）。
fn solid_block() -> VoxelVolume {
    let mut v = VoxelVolume::new(Vec3::new(-0.5, 0.0, -0.5), 0.25, 4, 4, 4);
    v.fill_box(Vec3::new(-0.5, 0.0, -0.5), Vec3::new(0.5, 1.0, 0.5));
    v
}

/// 一对静态 Marker 体走完整窄相，返回流形数（两条形状都给，便于两臂共用）。
fn manifold_count(
    sa: Shape,
    sb: Shape,
    hfs: &[vxl_phys::HeightField],
    providers: &dyn ProviderColliders,
) -> usize {
    let mut bodies = BodySet::new();
    let ia = bodies.push_static(sa, Vec3::ZERO, Quat::IDENTITY);
    let ib = bodies.push_static(sb, Vec3::ZERO, Quat::IDENTITY);
    let mut np = DefaultNarrowPhase::new(0.02);
    let mut out = Vec::new();
    np.collide(
        &bodies,
        &[(ia, ib)],
        hfs,
        providers,
        &mut out,
        &SerialJobSystem,
    );
    out.len()
}

/// **高度场 × 提供者** 当前不产接触（几何上重叠 ⇒ 不是"没碰到"，是"不受理"）。
#[test]
fn heightfield_vs_provider_yields_no_contacts_today() {
    let mut providers = Providers::default();
    let id = providers.push(solid_block());
    let hf = vxl_phys::HeightField::flat(-1.0, -1.0, 9, 9, 0.25, 0.5);
    let n = manifold_count(
        Shape::HeightField(0),
        Shape::Provider(id),
        std::slice::from_ref(&hf),
        &providers,
    );
    assert_eq!(
        n, 0,
        "高度场 × 提供者：当前不受理（见 SURVEY §1 的组合缺口块）——对偶解法落地后本行翻面"
    );
}

/// **提供者 × 提供者** 当前不产接触（ADR 0009：需要 provider 对偶解法）。
#[test]
fn provider_vs_provider_yields_no_contacts_today() {
    let mut providers = Providers::default();
    let a = providers.push(solid_block());
    let b = providers.push(solid_block());
    let n = manifold_count(Shape::Provider(a), Shape::Provider(b), &[], &providers);
    assert_eq!(
        n, 0,
        "提供者 × 提供者：当前不受理（ADR 0009 明确登记）——对偶解法落地后本行翻面"
    );
}
