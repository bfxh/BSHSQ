//! **形状对的地面/静置类判据**（盒/球/圆柱/锥/胶囊/复合体）—— `tests` 的子模块。
//!
//! 2026-10-05 从 836 行的 `tests.rs` 按域拆出（纯搬移，只补显式 `use`）。

use super::manifolds_for;
use crate::{CompoundChild, DefaultNarrowPhase, NarrowPhase, Shape};
use vxl_phys_core::{BodySet, Quat, SerialJobSystem, Vec3};

/// 胶囊 × 盒地板（竖直）：接触判据 = **线段端点到地面的距离 < radius**（线段本身
/// 并未碰到地面）；应给出 1 个接触点、法线向上、深度 ≈ 压入量。
#[test]
fn capsule_vs_box_floor_vertical() {
    let mut b = BodySet::new();
    b.push_static(
        Shape::Box {
            half: Vec3::new(5.0, 0.5, 5.0),
        },
        Vec3::new(0.0, -0.5, 0.0),
        Quat::IDENTITY,
    );
    // 下帽端点 y = y0 − 0.4；下帽表面 = 端点 − 0.3 ⇒ 压住地面 1 cm 时 y0 = 0.69。
    b.push_dynamic(
        Shape::Capsule {
            half_height: 0.4,
            radius: 0.3,
        },
        Vec3::new(0.0, 0.69, 0.0),
        Quat::IDENTITY,
        1000.0,
    );
    let out = manifolds_for(&b, &[]);
    assert_eq!(
        out.len(),
        1,
        "应有 1 个流形（胶囊 × 地板），实得 {}",
        out.len()
    );
    let m = &out[0];
    // 法线约定 **a→b**：期望符号由流形自身推导（别假定地板一定是 `a`——
    // 配对索引是 `(i, j)` 生成序，而本测试的 push 序不保证与之对应）。
    let floor_is_a = m.a == 0;
    let want = if floor_is_a { 1.0 } else { -1.0 };
    assert!(
        m.normal.y * want > 0.99,
        "法线应指向 a→b（地板→胶囊），a={} b={} 实得 {:?}",
        m.a,
        m.b,
        m.normal
    );
    assert_eq!(
        m.points.len(),
        1,
        "竖直胶囊只有下帽接触，实得 {} 点",
        m.points.len()
    );
    let d = m.points[0].depth;
    assert!(
        (d - 0.01).abs() < 2e-3,
        "深度应 ≈1 cm（0.3 − 端点距 0.29），实得 {d}"
    );
}

/// 外壳 × 圆柱：曾因 `support_of` 缺圆柱/锥分支而**静默无接触**（`TECH-SURVEY.md` A9 ④ 留档）。
/// 判据：有接触、法线竖直、有正压入。
#[test]
fn hull_on_cylinder_cap() {
    let mut np = DefaultNarrowPhase::new(0.01);
    // 外壳：3×3×3 立方点云（半 0.3）。
    let mut pts: Vec<Vec3> = Vec::with_capacity(27);
    for x in -1..=1 {
        for y in -1..=1 {
            for z in -1..=1 {
                pts.push(Vec3::new(x as f32, y as f32, z as f32) * 0.3);
            }
        }
    }
    let hid = np.add_hull(pts);
    let mut b = BodySet::new();
    // 静立圆柱（半高 0.5、半径 0.4，中心 y=-0.5 ⇒ 顶面 y=0）。
    b.push_static(
        Shape::Cylinder {
            half_height: 0.5,
            radius: 0.4,
        },
        Vec3::new(0.0, -0.5, 0.0),
        Quat::IDENTITY,
    );
    // 外壳落在顶面（底面压入 1 cm ⇒ 中心 y = 0.29）。
    b.push_dynamic(
        Shape::ConvexHull {
            hull: hid,
            half: Vec3::splat(0.3),
        },
        Vec3::new(0.0, 0.29, 0.0),
        Quat::IDENTITY,
        1.0,
    );
    let mut pairs = Vec::new();
    for i in 0..b.len() as u32 {
        for j in (i + 1)..b.len() as u32 {
            pairs.push((i, j));
        }
    }
    let mut out = Vec::new();
    np.collide(
        &b,
        &pairs,
        &[],
        &vxl_phys_core::interop::NoProviders,
        &mut out,
        &SerialJobSystem,
    );
    assert!(!out.is_empty(), "外壳 × 圆柱应有接触（此前为静默无接触）");
    let m = &out[0];
    assert!(m.normal.y.abs() > 0.99, "法线应竖直，实得 {:?}", m.normal);
    let dmax = m.points.iter().map(|p| p.depth).fold(f32::MIN, f32::max);
    assert!(dmax > 0.0, "应有正压入，实得 {dmax}");
}

/// 圆锥 × 盒地板（坐底）：底圆盘多面化 ⇒ 应给出**多点**支撑（单点会晃）、法线竖直、
/// 最深压入 ≈ 压入量。
#[test]
fn cone_on_box_floor() {
    let mut b = BodySet::new();
    b.push_static(
        Shape::Box {
            half: Vec3::new(5.0, 0.5, 5.0),
        },
        Vec3::new(0.0, -0.5, 0.0),
        Quat::IDENTITY,
    );
    // 底面在 y0 − h；要让底圆盘压入地面（y=0）1 cm ⇒ y0 = 0.5 − 0.01 = 0.49。
    b.push_dynamic(
        Shape::Cone {
            half_height: 0.5,
            radius: 0.4,
        },
        Vec3::new(0.0, 0.49, 0.0),
        Quat::IDENTITY,
        1000.0,
    );
    let out = manifolds_for(&b, &[]);
    assert!(!out.is_empty(), "圆锥坐底应有接触");
    let m = &out[0];
    let floor_is_a = m.a == 0;
    let want = if floor_is_a { 1.0 } else { -1.0 };
    assert!(
        m.normal.y * want > 0.99,
        "法线应竖直（a→b），实得 {:?}",
        m.normal
    );
    let dmax = m.points.iter().map(|p| p.depth).fold(f32::MIN, f32::max);
    assert!((dmax - 0.01).abs() < 3e-3, "最深压入应 ≈1 cm，实得 {dmax}");
    assert!(
        m.points.len() >= 3,
        "坐底应为多点支撑（多点才不摇），实得 {} 点",
        m.points.len()
    );
}

/// 复合体（哑铃：两端盒 + 中间横杆）坐地：**每个接触的子形状各出一条流形**（≥2 条），
/// 且特征号按**子序号左移 16 位**编码 ⇒ 不同子形状的特征空间互不重叠（暖缓存不串号）。
#[test]
fn compound_dumbbell_on_floor() {
    let mut np = DefaultNarrowPhase::new(0.01);
    // 两端用**盒**（而非球）：盒-盒接触带 `feature`，才能验到"子序号并入特征号"这条路径
    // （球接触的 `feature` 恒为 0 = "无特征"哨兵，标记不碰它）。
    let cid = np.add_compound(vec![
        CompoundChild {
            shape: Shape::Box {
                half: Vec3::splat(0.3),
            },
            offset: Vec3::new(-0.6, 0.0, 0.0),
            rot: Quat::IDENTITY,
        },
        CompoundChild {
            shape: Shape::Box {
                half: Vec3::splat(0.3),
            },
            offset: Vec3::new(0.6, 0.0, 0.0),
            rot: Quat::IDENTITY,
        },
        CompoundChild {
            shape: Shape::Box {
                half: Vec3::splat(0.3),
            },
            offset: Vec3::new(0.6, 0.0, 0.0),
            rot: Quat::IDENTITY,
        },
        CompoundChild {
            shape: Shape::Box {
                half: Vec3::new(0.6, 0.1, 0.1),
            },
            offset: Vec3::ZERO,
            rot: Quat::IDENTITY,
        },
    ]);
    let mut b = BodySet::new();
    b.push_static(
        Shape::Box {
            half: Vec3::new(5.0, 0.5, 5.0),
        },
        Vec3::new(0.0, -0.5, 0.0),
        Quat::IDENTITY,
    );
    // 两球半径 0.3、横杆 1.2×0.2×0.2；球压入地面（y=0）1 cm ⇒ y0 = 0.29。
    b.push_dynamic(
        Shape::Compound {
            compound: cid,
            half: np.compound_half_extents(cid),
        },
        Vec3::new(0.0, 0.29, 0.0),
        Quat::IDENTITY,
        1000.0,
    );
    let mut pairs = Vec::new();
    for i in 0..b.len() as u32 {
        for j in (i + 1)..b.len() as u32 {
            pairs.push((i, j));
        }
    }
    let mut out = Vec::new();
    np.collide(
        &b,
        &pairs,
        &[],
        &vxl_phys_core::interop::NoProviders,
        &mut out,
        &SerialJobSystem,
    );
    assert!(
        out.len() >= 2,
        "两个球应各出一条流形（同体对多条），实得 {} 条",
        out.len()
    );
    let mut tags = std::collections::BTreeSet::new();
    for m in &out {
        assert!(
            m.normal.y.abs() > 0.99,
            "地面接触法线应竖直，实得 {:?}",
            m.normal
        );
        let dmax = m.points.iter().map(|p| p.depth).fold(f32::MIN, f32::max);
        assert!((dmax - 0.01).abs() < 3e-3, "压入应 ≈1 cm，实得 {dmax}");
        for p in m.points.iter() {
            assert!(
                p.feature >> 16 != 0,
                "特征号应带子序号标记，实得 {}",
                p.feature
            );
            tags.insert(p.feature >> 16);
        }
    }
    assert!(
        tags.len() >= 2,
        "不同子形状的特征空间应互不相同，实得 {tags:?}"
    );
}

/// 平躺胶囊：应给出**两个**接触点（两帽各一）——这是它稳定静置（不摇）的前提。
#[test]
fn capsule_flat_gives_two_points() {
    let mut b = BodySet::new();
    b.push_static(
        Shape::Box {
            half: Vec3::new(5.0, 0.5, 5.0),
        },
        Vec3::new(0.0, -0.5, 0.0),
        Quat::IDENTITY,
    );
    // 绕 Z 转 90° ⇒ 局部 +Y 变成世界 +X ⇒ 胶囊水平平躺，压入 1 cm（0.3 − 0.29）。
    let rot = Quat::from_axis_angle(Vec3::Z, core::f32::consts::FRAC_PI_2);
    b.push_dynamic(
        Shape::Capsule {
            half_height: 0.4,
            radius: 0.3,
        },
        Vec3::new(0.0, 0.29, 0.0),
        rot,
        1000.0,
    );
    let out = manifolds_for(&b, &[]);
    assert_eq!(out.len(), 1, "应有 1 个流形，实得 {}", out.len());
    assert_eq!(
        out[0].points.len(),
        2,
        "平躺胶囊应给 2 点支撑，实得 {}",
        out[0].points.len()
    );
}

#[test]
fn sphere_sphere_touch() {
    let mut b = BodySet::new();
    b.push_dynamic(
        Shape::Sphere { radius: 0.5 },
        Vec3::ZERO,
        Quat::IDENTITY,
        1.0,
    );
    b.push_dynamic(
        Shape::Sphere { radius: 0.5 },
        Vec3::new(0.9, 0.0, 0.0),
        Quat::IDENTITY,
        1.0,
    );
    let m = manifolds_for(&b, &[]);
    assert_eq!(m.len(), 1);
    assert!((m[0].normal.x - 1.0).abs() < 1e-5);
    assert!((m[0].points[0].depth - 0.1).abs() < 1e-5);
}

#[test]
fn sphere_above_box_normal_points_down() {
    let mut b = BodySet::new();
    // 球心在盒顶上方 0.4 → 穿透深度 = r - 0.4 = 0.1。
    b.push_dynamic(
        Shape::Sphere { radius: 0.5 },
        Vec3::new(0.0, 0.9, 0.0),
        Quat::IDENTITY,
        1.0,
    );
    b.push_static(
        Shape::Box {
            half: Vec3::new(2.0, 0.5, 2.0),
        },
        Vec3::ZERO,
        Quat::IDENTITY,
    );
    let m = manifolds_for(&b, &[]);
    assert_eq!(m.len(), 1);
    // a=球 在上，b=盒 → 法线 a→b 朝下。
    assert!(m[0].normal.y < -0.99, "normal {:?}", m[0].normal);
    assert!((m[0].points[0].depth - 0.1).abs() < 1e-4);
}

#[test]
fn box_box_resting_manifold() {
    let mut b = BodySet::new();
    b.push_dynamic(
        Shape::Box {
            half: Vec3::new(0.5, 0.5, 0.5),
        },
        Vec3::new(0.0, 0.95, 0.0),
        Quat::IDENTITY,
        1.0,
    );
    b.push_static(
        Shape::Box {
            half: Vec3::new(2.0, 0.5, 2.0),
        },
        Vec3::ZERO,
        Quat::IDENTITY,
    );
    let m = manifolds_for(&b, &[]);
    assert_eq!(m.len(), 1);
    assert!(m[0].normal.y < -0.99);
    assert!(!m[0].points.is_empty() && m[0].points.len() <= 4);
    assert!(m[0].points[0].depth > 0.0 && m[0].points[0].depth < 0.06);
}

#[test]
fn box_penetrating_deep_gives_points() {
    let mut b = BodySet::new();
    b.push_dynamic(
        Shape::Box {
            half: Vec3::splat(0.5),
        },
        Vec3::new(0.0, 0.7, 0.0),
        Quat::IDENTITY,
        1.0,
    );
    b.push_static(
        Shape::Box {
            half: Vec3::new(2.0, 0.5, 2.0),
        },
        Vec3::ZERO,
        Quat::IDENTITY,
    );
    let m = manifolds_for(&b, &[]);
    assert_eq!(m.len(), 1);
    assert_eq!(m[0].points.len(), 4);
}

#[test]
fn cylinder_on_ground() {
    let mut b = BodySet::new();
    b.push_dynamic(
        Shape::Cylinder {
            half_height: 0.5,
            radius: 0.3,
        },
        Vec3::new(0.0, 0.9, 0.0),
        Quat::IDENTITY,
        1.0,
    );
    b.push_static(
        Shape::Box {
            half: Vec3::new(2.0, 0.5, 2.0),
        },
        Vec3::ZERO,
        Quat::IDENTITY,
    );
    let m = manifolds_for(&b, &[]);
    assert_eq!(m.len(), 1);
    assert!(m[0].normal.y < -0.99);
}

#[test]
fn separated_boxes_no_manifold() {
    let mut b = BodySet::new();
    b.push_dynamic(
        Shape::Box {
            half: Vec3::splat(0.5),
        },
        Vec3::new(0.0, 5.0, 0.0),
        Quat::IDENTITY,
        1.0,
    );
    b.push_static(
        Shape::Box {
            half: Vec3::splat(0.5),
        },
        Vec3::ZERO,
        Quat::IDENTITY,
    );
    assert!(manifolds_for(&b, &[]).is_empty());
}
