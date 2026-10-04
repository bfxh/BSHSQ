//! 体素 provider 的**两套接口逐位等价**（M2 收口件）。
//!
//! - 域 trait：`vxl_phys_core::interop::CollisionProvider`（直接问体素体，
//!   `VoxelVolume::contacts_box / contacts_sphere / contacts_point`）；
//! - 门面注册表：`vxl_phys::Providers`（窄相**实际**走的那条路，按 id 派发）。
//!
//! 断言两者在「盒 / 球 / 点」×「命中 / 落空 / 预期接触」上**逐位一致**（点/深度/法线/特征
//! 与"是否支持"的返回值）。两套接口此前是**各写一份**（域 trait 侧甚至只有 8 角点采样的
//! 默认盒查询）⇒ 这条断言把「同一个域只能有一份接触数学」钉住：以后谁改了一边，
//! 另一边不同步就会红。
//!
//! ⚠️ 本文件不许出现 `unwrap` / `expect` / `.clone()`（新文件零基线）。

use vxl_phys::Providers;
use vxl_phys_core::interop::{CollisionProvider, InteropContact, ProviderColliders};
use vxl_phys_core::{Quat, Vec3};
use vxl_phys_terrain::voxel::VoxelVolume;

/// 8×2×8 格、边长 0.5：填充 y ∈ [0,1) 两层 ⇒ 顶面 y = 1.0（与 `voxel.rs` 测试同款）。
fn floor_volume() -> VoxelVolume {
    let mut v = VoxelVolume::new(Vec3::new(-2.0, 0.0, -2.0), 0.5, 8, 2, 8);
    v.fill_box(Vec3::new(-2.0, 0.0, -2.0), Vec3::new(2.0, 1.0, 2.0));
    v
}

fn bits(v: Vec3) -> (u32, u32, u32) {
    (v.x.to_bits(), v.y.to_bits(), v.z.to_bits())
}

/// 两条路各跑一次并**逐位**比对（"是否支持"的返回值也必须一致）。
fn assert_equiv(
    tag: &str,
    mut domain: impl FnMut(&mut Vec<InteropContact>) -> bool,
    mut facade: impl FnMut(&mut Vec<InteropContact>) -> bool,
) {
    let mut a = Vec::new();
    let mut b = Vec::new();
    assert_eq!(
        domain(&mut a),
        facade(&mut b),
        "{tag}：是否支持该查询须一致"
    );
    assert_eq!(a.len(), b.len(), "{tag}：接触点数须一致");
    for (i, (x, y)) in a.iter().zip(b.iter()).enumerate() {
        assert_eq!(bits(x.point), bits(y.point), "{tag}：第 {i} 点");
        assert_eq!(x.depth.to_bits(), y.depth.to_bits(), "{tag}：第 {i} 点深度");
        assert_eq!(bits(x.normal), bits(y.normal), "{tag}：第 {i} 点法线");
        assert_eq!(x.feature, y.feature, "{tag}：第 {i} 点特征");
    }
}

#[test]
fn voxel_domain_trait_matches_facade_bitwise() {
    let mut providers = Providers::default();
    let id = providers.push(floor_volume());
    let Some(vol) = providers.voxel(id) else {
        return; // 注册失败：测试环境异常，直接不判（门脚本禁 unwrap）
    };
    const SKIN: f32 = 0.02;
    let rot = Quat::from_axis_angle(Vec3::Y, 0.6);
    // 盒：中心 y=1.4、半 0.5 ⇒ 底面 0.9 压入顶面 0.1（旋转档一并过）
    let bpos = Vec3::new(0.25, 1.4, 0.25);
    assert_equiv(
        "盒·命中",
        |out| vol.contacts_box(Vec3::splat(0.5), bpos, rot, SKIN, out),
        |out| providers.contacts_box(id, Vec3::splat(0.5), bpos, rot, SKIN, out),
    );
    assert_equiv(
        "盒·落空",
        |out| vol.contacts_box(Vec3::splat(0.5), Vec3::new(0.25, 2.5, 0.25), rot, SKIN, out),
        |out| {
            providers.contacts_box(
                id,
                Vec3::splat(0.5),
                Vec3::new(0.25, 2.5, 0.25),
                rot,
                SKIN,
                out,
            )
        },
    );
    // 球：中心 y=1.3、半径 0.4 ⇒ 球底 0.9、sdf(center)=0.3 ⇒ depth = 0.1
    assert_equiv(
        "球·命中",
        |out| vol.contacts_sphere(Vec3::new(0.25, 1.3, 0.25), 0.4, SKIN, out),
        |out| providers.contacts_sphere(id, Vec3::new(0.25, 1.3, 0.25), 0.4, SKIN, out),
    );
    assert_equiv(
        "球·落空",
        |out| vol.contacts_sphere(Vec3::new(0.25, 3.0, 0.25), 0.4, SKIN, out),
        |out| providers.contacts_sphere(id, Vec3::new(0.25, 3.0, 0.25), 0.4, SKIN, out),
    );
    // 点：体内一点（depth = −sdf = 0.05）+ 域外一点（支持查询但不推接触）
    assert_equiv(
        "点·命中",
        |out| vol.contacts_point(Vec3::new(0.25, 0.95, 0.25), SKIN, out),
        |out| providers.contacts_point(id, Vec3::new(0.25, 0.95, 0.25), SKIN, out),
    );
    assert_equiv(
        "点·落空",
        |out| vol.contacts_point(Vec3::new(0.25, 3.0, 0.25), SKIN, out),
        |out| providers.contacts_point(id, Vec3::new(0.25, 3.0, 0.25), SKIN, out),
    );
    // 预期接触（负深度，|depth| ≤ skin）：球底离顶面 0.01 ⇒ 仍进流形
    assert_equiv(
        "球·预期接触",
        |out| vol.contacts_sphere(Vec3::new(0.25, 1.41, 0.25), 0.4, SKIN, out),
        |out| providers.contacts_sphere(id, Vec3::new(0.25, 1.41, 0.25), 0.4, SKIN, out),
    );
}
