//! 高度场 × provider 通道：**等价性证明**（M2 工作项「高度场迁到 provider 通道」的前置件）。
//!
//! 断言：`CollisionProvider::contacts_box / contacts_sphere / contacts_point` 的输出，与窄相**实际路由**
//! （`DefaultNarrowPhase::collide` 产出的流形）**逐位相同**——点数、点/深度/特征、法线（含符号）。
//! 这样"包一层、不改数学"才是**可验证**的，而不是靠推理；也把 M2 后续"真把派发改过去"这一步
//! 变成机械操作（判据已经钉在这里）。
//!
//! 跑法：`cargo test -p vxl-phys-narrow --test heightfield_interop`
//!
//! ⚠️ 本文件**不许出现 `unwrap`/`expect`**（unwrap-gate 按文件计数、新文件零基线）⇒ 一律用
//! `match` 取 `Option`。

use vxl_phys_core::interop::{CollisionProvider, NoProviders};
use vxl_phys_core::{BodySet, Quat, SerialJobSystem, Shape, Vec3};
use vxl_phys_narrow::heightfield::HeightField;
use vxl_phys_narrow::{DefaultNarrowPhase, NarrowPhase};

/// 斜坡（x 向梯度、法线逆坡）——比平地更能暴露点集/法线口径的差异。
fn ramp() -> HeightField {
    let mut hf = HeightField::flat(-5.0, -5.0, 11, 11, 1.0, 0.0);
    for iz in 0..11u32 {
        for ix in 0..11u32 {
            hf.set_height(ix, iz, ix as f32 * 0.5);
        }
    }
    hf
}

/// 走完整窄相路由（体对暴力），返回第一条流形的 `(法线, [(点, 深度, 特征)])`；无流形 = 空点表。
fn narrow_first(b: &BodySet, hf: &[HeightField]) -> (Vec3, Vec<(Vec3, f32, u32)>) {
    let mut np = DefaultNarrowPhase::new(0.02);
    narrow_first_in(&mut np, b, hf)
}

/// 同 `narrow_first`，但复用调用方的窄相实例（**外壳仓库注册在它上面** ⇒ 必须同一份）。
fn narrow_first_in(
    np: &mut DefaultNarrowPhase,
    b: &BodySet,
    hf: &[HeightField],
) -> (Vec3, Vec<(Vec3, f32, u32)>) {
    let mut pairs = Vec::new();
    for i in 0..b.len() as u32 {
        for j in (i + 1)..b.len() as u32 {
            pairs.push((i, j));
        }
    }
    let mut out = Vec::new();
    np.collide(b, &pairs, hf, &NoProviders, &mut out, &SerialJobSystem);
    match out.first() {
        Some(m) => (
            m.normal,
            m.points
                .iter()
                .map(|p| (p.point, p.depth, p.feature))
                .collect(),
        ),
        None => (Vec3::ZERO, Vec::new()),
    }
}

fn bits3(v: Vec3) -> (u32, u32, u32) {
    (v.x.to_bits(), v.y.to_bits(), v.z.to_bits())
}

/// 逐位比较两串接触（点、深度、特征）。
fn assert_same_points(narrow: &[(Vec3, f32, u32)], out: &[vxl_phys_core::interop::InteropContact]) {
    assert_eq!(narrow.len(), out.len(), "点数须一致");
    for (i, (p, d, f)) in narrow.iter().enumerate() {
        assert_eq!(bits3(*p), bits3(out[i].point), "第 {i} 点须逐位一致");
        assert_eq!(
            d.to_bits(),
            out[i].depth.to_bits(),
            "第 {i} 点深度须逐位一致"
        );
        assert_eq!(*f, out[i].feature, "第 {i} 点特征须一致");
    }
}

/// **盒 × 高度场**：provider 面 vs 窄相路由（hf 是 a ⇒ 两者的法线同为"地形外向"）。
#[test]
fn provider_box_contacts_match_narrow_path_bitwise() {
    let hf = ramp();
    let half = Vec3::splat(0.5);
    let pos = Vec3::new(2.5, 0.4, 0.0);
    let mut b = BodySet::new();
    let h = b.push_static(Shape::HeightField(0), Vec3::ZERO, Quat::IDENTITY);
    let s = b.push_dynamic(Shape::Box { half }, pos, Quat::IDENTITY, 1000.0);
    assert_ne!(h, s, "两体应有不同 id");
    let (narrow_n, narrow_pts) = narrow_first(&b, std::slice::from_ref(&hf));
    assert!(!narrow_pts.is_empty(), "窄相应产出流形");
    let mut out = Vec::new();
    assert!(
        hf.contacts_box(half, pos, Quat::IDENTITY, 0.02, &mut out),
        "provider 面应产出接触"
    );
    assert_same_points(&narrow_pts, &out);
    assert_eq!(bits3(narrow_n), bits3(out[0].normal), "法线须逐位一致");
}

#[test]
fn flat_field_closest_point_and_box_contacts() {
    let hf = HeightField::flat(-10.0, -10.0, 21, 21, 1.0, 0.5);
    let probe = Vec3::new(0.25, 1.0, -0.25);
    assert!(hf.closest_point(probe).is_some(), "场内点应有命中");
    let Some(hit) = hf.closest_point(probe) else {
        return;
    };
    assert!((hit.point.y - 0.5).abs() < 1e-6);
    assert!((hit.signed_dist - 0.5).abs() < 1e-6);
    assert!((hit.normal - Vec3::new(0.0, 1.0, 0.0)).length() < 1e-6);
    // 盒（半 0.5）落在 y=0.9 ⇒ 底面 4 角穿透 0.1：**专用路径**（`contacts_box` 覆写）应给 4 点
    let mut out = Vec::new();
    assert!(hf.contacts_box(
        Vec3::splat(0.5),
        Vec3::new(0.0, 0.9, 0.0),
        Quat::IDENTITY,
        0.02,
        &mut out
    ));
    assert_eq!(out.len(), 4);
    for c in &out {
        assert!((c.depth - 0.1).abs() < 1e-5, "depth={}", c.depth);
    }
    assert!(hf.closest_point(Vec3::new(99.0, 1.0, 0.0)).is_none());
}

/// **球 × 高度场**：provider 面（`contacts_sphere`）vs 窄相路由，逐位一致。
/// 斜坡在 x=2.5 处高度 3.75（`ramp()` 的 h = (x+5)·0.5）⇒ 球心 4.05、半径 0.4 时
/// 球底压入 0.10；该处 3×3 邻域节点全在球半径外 ⇒ 只有投影点一个样本。
#[test]
fn provider_sphere_contacts_match_narrow_path_bitwise() {
    let hf = ramp();
    let center = Vec3::new(2.5, 4.05, 0.0);
    let radius = 0.4;
    let mut b = BodySet::new();
    let h = b.push_static(Shape::HeightField(0), Vec3::ZERO, Quat::IDENTITY);
    let s = b.push_dynamic(Shape::Sphere { radius }, center, Quat::IDENTITY, 1000.0);
    assert_ne!(h, s, "两体应有不同 id");
    let (narrow_n, narrow_pts) = narrow_first(&b, std::slice::from_ref(&hf));
    assert!(!narrow_pts.is_empty(), "窄相应产出流形");
    let mut out = Vec::new();
    assert!(
        hf.contacts_sphere(center, radius, 0.02, &mut out),
        "provider 面应产出接触"
    );
    assert_same_points(&narrow_pts, &out);
    assert_eq!(bits3(narrow_n), bits3(out[0].normal), "法线须逐位一致");
}

/// **点查询 × 高度场**：provider 面 vs 窄相「**单顶点外壳**」的逐顶点路由
/// （`hull_heightfield` 对 1 个顶点就是一次点查询）。
///
/// 判据：点 / 深度 / **法线**逐位一致。`feature` 口径**有意不同**——provider 点查询没有
/// 顶点身份 ⇒ 0（与 `contacts_point_voxel` 同），窄相逐顶点给的是顶点序号 + 1
/// ⇒ 这里不比特征（其余三项仍逐位比，见 `assert_eq!` 的三条）。
#[test]
fn provider_point_contacts_match_narrow_single_vertex_hull() {
    let hf = ramp();
    let p = Vec3::new(2.5, 3.70, 0.0);
    let mut np = DefaultNarrowPhase::new(0.02);
    let hull = np.add_hull(vec![Vec3::ZERO]);
    let mut b = BodySet::new();
    let h = b.push_static(Shape::HeightField(0), Vec3::ZERO, Quat::IDENTITY);
    let s = b.push_dynamic(
        Shape::ConvexHull {
            hull,
            half: Vec3::ZERO,
        },
        p,
        Quat::IDENTITY,
        1000.0,
    );
    assert_ne!(h, s, "两体应有不同 id");
    let (narrow_n, narrow_pts) = narrow_first_in(&mut np, &b, std::slice::from_ref(&hf));
    assert_eq!(narrow_pts.len(), 1, "单顶点外壳应给 1 个接触");
    let mut out = Vec::new();
    assert!(
        hf.contacts_point(p, 0.02, &mut out),
        "provider 应支持点查询"
    );
    assert_eq!(out.len(), 1, "provider 点查询应给 1 个接触");
    assert_eq!(bits3(narrow_pts[0].0), bits3(out[0].point), "点须逐位一致");
    assert_eq!(
        narrow_pts[0].1.to_bits(),
        out[0].depth.to_bits(),
        "深度须逐位一致"
    );
    assert_eq!(bits3(narrow_n), bits3(out[0].normal), "法线须逐位一致");
}

#[test]
fn ramp_normal_points_downhill_outward() {
    let mut hf = HeightField::flat(-5.0, -5.0, 11, 11, 1.0, 0.0);
    for iz in 0..11u32 {
        for ix in 0..11u32 {
            hf.set_height(ix, iz, ix as f32);
        }
    }
    let probe = Vec3::new(2.5, 3.0, 0.0);
    assert!(hf.closest_point(probe).is_some(), "场内点应有命中");
    let Some(hit) = hf.closest_point(probe) else {
        return;
    };
    assert!(hit.normal.x < -0.5, "法线应逆坡：{:?}", hit.normal);
    assert!(hit.normal.y > 0.5);
}
